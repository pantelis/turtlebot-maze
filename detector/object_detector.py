#!/usr/bin/env python3
"""
Zenoh-based YOLOv8 object detector.

Subscribes to ROS 2 camera images bridged via zenoh-bridge-ros2dds,
runs YOLOv8 inference, publishes detection results back over Zenoh.

Based on:
- https://github.com/eclipse-zenoh/zenoh-demos/tree/main/ROS2/zenoh-python-lidar-plot
- https://github.com/eclipse-zenoh/zenoh-demos/tree/main/computer-vision/face-recog
"""

import argparse
import json
import time

import cv2
import numpy as np
import zenoh
from pycdr2 import IdlStruct
from pycdr2.types import uint8, uint32, int32
from dataclasses import dataclass, field
from typing import List
from ultralytics import YOLO


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
        default="rt/camera/image_raw",
        help="Zenoh key for camera images (bridged from ROS 2)",
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
    args = parser.parse_args()

    # Load model
    model = YOLO(args.model)
    print(f"Loaded model: {args.model}")
    print(f"Subscribing to: {args.image_key}")
    print(f"Publishing to:  {args.detection_key}")

    min_interval = 1.0 / args.max_fps
    last_inference_time = 0.0

    # Open Zenoh session
    conf = zenoh.Config()
    if args.connect:
        conf.insert_json5("connect/endpoints", json.dumps([args.connect]))

    session = zenoh.open(conf)
    pub = session.declare_publisher(args.detection_key)

    def image_callback(sample):
        nonlocal last_inference_time

        # Rate limit
        now = time.time()
        if now - last_inference_time < min_interval:
            return
        last_inference_time = now

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
        for result in results:
            for box in result.boxes:
                cls_id = int(box.cls[0])
                detections.append(
                    {
                        "class": model.names[cls_id],
                        "confidence": round(float(box.conf[0]), 3),
                        "bbox": [round(float(x), 1) for x in box.xyxy[0].tolist()],
                    }
                )

        # Publish JSON detections
        payload = json.dumps(detections)
        pub.put(payload.encode())

        if detections:
            classes = [d["class"] for d in detections]
            print(f"Detected: {classes}")

    sub = session.declare_subscriber(args.image_key, image_callback)
    print("Detector running. Press Ctrl+C to stop.")

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        sub.undeclare()
        pub.undeclare()
        session.close()


if __name__ == "__main__":
    main()
