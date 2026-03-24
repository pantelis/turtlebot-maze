#!/usr/bin/env python3
"""
Zenoh-based YOLOv8 object detector with CLIP embedding extraction.

Subscribes to ROS 2 camera images bridged via zenoh-bridge-ros2dds,
runs YOLOv8 inference, optionally computes CLIP embeddings for each
detection crop, and publishes results back over Zenoh.

Based on:
- https://github.com/eclipse-zenoh/zenoh-demos/tree/main/ROS2/zenoh-python-lidar-plot
- https://github.com/eclipse-zenoh/zenoh-demos/tree/main/computer-vision/face-recog
"""

import argparse
import base64
import json
import math
import socket
import time
from datetime import datetime, timezone

import cv2
import numpy as np
import torch
import zenoh
from pycdr2 import IdlStruct
from pycdr2.types import uint8, uint32, int32, float64
from dataclasses import dataclass, field
from typing import List
from ultralytics import YOLO
from PIL import Image as PILImage


# CDR struct matching sensor_msgs/msg/Image
# See: https://docs.ros2.org/latest/api/sensor_msgs/msg/Image.html
@dataclass
class Time(IdlStruct, typename="builtin_interfaces/msg/Time"):
    sec: int32
    nanosec: uint32


@dataclass
class Header(IdlStruct, typename="std_msgs/msg/Header"):
    stamp: Time
    frame_id: str


@dataclass
class Image(IdlStruct, typename="sensor_msgs/msg/Image"):
    header: Header
    height: uint32
    width: uint32
    encoding: str
    is_bigendian: uint8
    step: uint32
    data: List[uint8]


@dataclass
class Point(IdlStruct, typename="geometry_msgs/msg/Point"):
    x: float64
    y: float64
    z: float64


@dataclass
class Quaternion(IdlStruct, typename="geometry_msgs/msg/Quaternion"):
    x: float64
    y: float64
    z: float64
    w: float64


@dataclass
class Pose(IdlStruct, typename="geometry_msgs/msg/Pose"):
    position: Point
    orientation: Quaternion


@dataclass
class PoseWithCovariance(IdlStruct, typename="geometry_msgs/msg/PoseWithCovariance"):
    pose: Pose
    covariance: List[float64]


@dataclass
class Twist(IdlStruct, typename="geometry_msgs/msg/Twist"):
    linear: Point
    angular: Point


@dataclass
class TwistWithCovariance(IdlStruct, typename="geometry_msgs/msg/TwistWithCovariance"):
    twist: Twist
    covariance: List[float64]


@dataclass
class Odometry(IdlStruct, typename="nav_msgs/msg/Odometry"):
    header: Header
    child_frame_id: str
    pose: PoseWithCovariance
    twist: TwistWithCovariance


def quaternion_to_yaw(q: Quaternion) -> float:
    """Extract yaw angle from a quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def main():
    parser = argparse.ArgumentParser(description="Zenoh YOLOv8 Object Detector")
    parser.add_argument(
        "-e",
        "--connect",
        type=str,
        default="",
        help="Zenoh endpoint to connect to (empty = multicast)",
    )
    parser.add_argument(
        "-m",
        "--model",
        type=str,
        default="yolov8n.pt",
        help="Ultralytics model name or path",
    )
    parser.add_argument(
        "-c",
        "--confidence",
        type=float,
        default=0.5,
        help="Minimum detection confidence",
    )
    parser.add_argument(
        "--image-key",
        type=str,
        default="camera/color/image_raw",
        help="Zenoh key for camera images (D435i: camera/color/image_raw via zenoh-bridge-ros2dds)",
    )
    parser.add_argument(
        "--detection-key",
        type=str,
        default="tb/detections",
        help="Zenoh key to publish detections to",
    )
    parser.add_argument(
        "--max-fps", type=float, default=10.0, help="Maximum inference rate in Hz"
    )
    parser.add_argument(
        "--enable-embeddings",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Compute CLIP embeddings for each detection crop (default: enabled)",
    )
    parser.add_argument(
        "--clip-model",
        type=str,
        default="ViT-B-32",
        help="open_clip model architecture",
    )
    parser.add_argument(
        "--clip-pretrained",
        type=str,
        default="laion2b_s34b_b79k",
        help="open_clip pretrained weights tag",
    )
    parser.add_argument(
        "--odom-key",
        type=str,
        default="odom",
        help="Zenoh key for odometry (bridged from /odom)",
    )
    parser.add_argument(
        "--keyframe-dist",
        type=float,
        default=0.5,
        help="Minimum translation (m) to trigger a keyframe",
    )
    parser.add_argument(
        "--keyframe-angle",
        type=float,
        default=15.0,
        help="Minimum rotation (degrees) to trigger a keyframe",
    )
    parser.add_argument(
        "--run-id",
        type=str,
        default="",
        help="Session identifier (default: auto-generated as hostname-timestamp)",
    )
    args = parser.parse_args()

    if args.run_id:
        run_id = args.run_id
    else:
        run_id = f"{socket.gethostname()}-{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')}"
    print(f"Run ID: {run_id}")

    # Load YOLO model
    model = YOLO(args.model)
    print(f"Loaded model: {args.model}")
    print(f"Subscribing to: {args.image_key}")
    print(f"Publishing to:  {args.detection_key}")

    # Load CLIP model for embedding extraction
    clip_model = None
    clip_preprocess = None
    clip_device = None
    if args.enable_embeddings:
        import open_clip

        clip_device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        clip_model, _, clip_preprocess = open_clip.create_model_and_transforms(
            args.clip_model, pretrained=args.clip_pretrained
        )
        clip_model = clip_model.to(clip_device).eval()
        clip_embedding_dim = clip_model.visual.output_dim
        clip_tag = f"{args.clip_model}/{args.clip_pretrained}"
        print(
            f"CLIP embeddings enabled: {clip_tag} "
            f"({clip_embedding_dim}-dim) on {clip_device}"
        )

    min_interval = 1.0 / args.max_fps
    last_inference_time = 0.0

    # Keyframe state
    keyframe_dist_thresh = args.keyframe_dist
    keyframe_angle_thresh = math.radians(args.keyframe_angle)
    latest_pose = None  # (x, y, yaw)
    last_keyframe_pose = None  # (x, y, yaw)
    keyframe_id = 0

    print(
        f"Keyframe gating: dist={keyframe_dist_thresh}m, "
        f"angle={args.keyframe_angle}deg"
    )
    print(f"Subscribing to odom: {args.odom_key}")

    # Open Zenoh session
    conf = zenoh.Config()
    if args.connect:
        conf.insert_json5("connect/endpoints", json.dumps([args.connect]))

    session = zenoh.open(conf)
    pub = session.declare_publisher(args.detection_key)

    def odom_callback(sample):
        nonlocal latest_pose
        try:
            odom_msg = Odometry.deserialize(bytes(sample.payload))
        except Exception as e:
            print(f"Odom CDR deserialize error: {e}")
            return
        p = odom_msg.pose.pose.position
        yaw = quaternion_to_yaw(odom_msg.pose.pose.orientation)
        latest_pose = (p.x, p.y, yaw)

    def is_keyframe() -> bool:
        nonlocal last_keyframe_pose, keyframe_id
        if latest_pose is None:
            return False
        if last_keyframe_pose is None:
            last_keyframe_pose = latest_pose
            keyframe_id += 1
            return True
        dx = latest_pose[0] - last_keyframe_pose[0]
        dy = latest_pose[1] - last_keyframe_pose[1]
        dist = math.sqrt(dx * dx + dy * dy)
        dyaw = abs(latest_pose[2] - last_keyframe_pose[2])
        # Normalize angle to [0, pi]
        dyaw = abs(math.atan2(math.sin(dyaw), math.cos(dyaw)))
        if dist >= keyframe_dist_thresh or dyaw >= keyframe_angle_thresh:
            last_keyframe_pose = latest_pose
            keyframe_id += 1
            return True
        return False

    def image_callback(sample):
        nonlocal last_inference_time

        # Rate limit
        now = time.time()
        if now - last_inference_time < min_interval:
            return
        last_inference_time = now

        # Keyframe gate: only process if robot has moved enough
        if not is_keyframe():
            return

        # Deserialize CDR-encoded sensor_msgs/Image
        try:
            img_msg = Image.deserialize(bytes(sample.payload))
        except Exception as e:
            print(f"CDR deserialize error: {e}")
            return

        # Convert to numpy array
        if img_msg.encoding == "rgb8":
            channels = 3
            frame = np.frombuffer(bytes(img_msg.data), dtype=np.uint8).reshape(
                img_msg.height, img_msg.width, channels
            )
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        elif img_msg.encoding == "bgr8":
            channels = 3
            frame = np.frombuffer(bytes(img_msg.data), dtype=np.uint8).reshape(
                img_msg.height, img_msg.width, channels
            )
        else:
            print(f"Unsupported encoding: {img_msg.encoding}")
            return

        # Run YOLOv8 inference
        results = model.predict(frame, conf=args.confidence, verbose=False)

        # Build detections list
        detections = []
        crops = []
        for result in results:
            for box in result.boxes:
                cls_id = int(box.cls[0])
                bbox = [round(float(x), 1) for x in box.xyxy[0].tolist()]
                det = {
                    "class": model.names[cls_id],
                    "confidence": round(float(box.conf[0]), 3),
                    "bbox": bbox,
                }
                detections.append(det)

                # Crop the detection region for CLIP encoding
                if clip_model is not None:
                    x1, y1, x2, y2 = (
                        int(bbox[0]),
                        int(bbox[1]),
                        int(bbox[2]),
                        int(bbox[3]),
                    )
                    x1 = max(0, x1)
                    y1 = max(0, y1)
                    x2 = min(frame.shape[1], x2)
                    y2 = min(frame.shape[0], y2)
                    crop_bgr = frame[y1:y2, x1:x2]
                    if crop_bgr.size > 0:
                        crop_rgb = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB)
                        crops.append(PILImage.fromarray(crop_rgb))
                    else:
                        crops.append(None)

        # Batch-encode CLIP embeddings for all crops
        if clip_model is not None and crops:
            valid_indices = [i for i, c in enumerate(crops) if c is not None]
            if valid_indices:
                batch = torch.stack(
                    [clip_preprocess(crops[i]) for i in valid_indices]
                ).to(clip_device)
                with torch.no_grad():
                    embeddings = clip_model.encode_image(batch)
                # L2-normalize for cosine similarity
                embeddings = torch.nn.functional.normalize(embeddings, dim=-1)
                embeddings_np = embeddings.cpu().numpy()

                emb_idx = 0
                for i in range(len(detections)):
                    if i in valid_indices:
                        vec = embeddings_np[emb_idx]
                        detections[i]["embedding"] = base64.b64encode(
                            vec.astype(np.float32).tobytes()
                        ).decode("ascii")
                        detections[i]["embedding_dim"] = int(vec.shape[0])
                        detections[i]["embedding_model"] = clip_tag
                        emb_idx += 1

        # Build keyframe envelope with pose metadata
        pose_data = (
            {
                "map_x": round(latest_pose[0], 4),
                "map_y": round(latest_pose[1], 4),
                "map_yaw": round(latest_pose[2], 4),
            }
            if latest_pose
            else {}
        )

        for idx, det in enumerate(detections):
            det["det_id"] = f"{run_id}_kf{keyframe_id}_d{idx}"

        envelope = {
            "run_id": run_id,
            "keyframe_id": keyframe_id,
            "timestamp": now,
            **pose_data,
            "detections": detections,
        }
        payload = json.dumps(envelope)
        pub.put(payload.encode())

        if detections:
            classes = [d["class"] for d in detections]
            print(
                f"KF#{keyframe_id} Detected: {classes} "
                f"@ ({pose_data.get('map_x', '?')}, {pose_data.get('map_y', '?')})"
            )

    odom_sub = session.declare_subscriber(args.odom_key, odom_callback)
    img_sub = session.declare_subscriber(args.image_key, image_callback)
    print("Detector running. Press Ctrl+C to stop.")

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        img_sub.undeclare()
        odom_sub.undeclare()
        pub.undeclare()
        session.close()


if __name__ == "__main__":
    main()
