#!/usr/bin/env python3
"""
ArUco marker detection test via Zenoh.

Subscribes to camera images from zenoh-bridge-ros2dds,
runs OpenCV ArUco detection, and reports found marker IDs.

Usage:
    python3 test_aruco_detection.py [--image-key camera/image_raw] [--duration 30]
"""

import argparse
import json
import time

import cv2
import numpy as np
import zenoh
from dataclasses import dataclass
from pycdr2 import IdlStruct
from pycdr2.types import int32, uint8, uint32
from typing import List


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
    parser = argparse.ArgumentParser(description="ArUco marker detection test")
    parser.add_argument(
        "--image-key",
        type=str,
        default="intel_realsense_r200_depth/image_raw",
        help="Zenoh key for camera images",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=30.0,
        help="How long to listen for markers (seconds)",
    )
    parser.add_argument(
        "-e",
        "--connect",
        type=str,
        default="tcp/localhost:7447",
        help="Zenoh endpoint",
    )
    args = parser.parse_args()

    # ArUco detector setup — markers use DICT_4X4_100
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    marker_size = 0.3  # 30cm markers

    # Zenoh session
    conf = zenoh.Config()
    if args.connect:
        conf.insert_json5("connect/endpoints", json.dumps([args.connect]))
    session = zenoh.open(conf)

    frame_count = 0
    detection_count = 0
    detected_ids = set()
    start_time = time.time()

    def image_callback(sample):
        nonlocal frame_count, detection_count
        elapsed = time.time() - start_time
        if elapsed > args.duration:
            return

        frame_count += 1
        if frame_count % 3 != 0:
            return

        try:
            img_msg = Image.deserialize(bytes(sample.payload))
        except Exception as e:
            print(f"CDR error: {e}")
            return

        if img_msg.encoding in ("rgb8", "bgr8"):
            frame = np.frombuffer(bytes(img_msg.data), dtype=np.uint8).reshape(
                img_msg.height, img_msg.width, 3
            )
            if img_msg.encoding == "rgb8":
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        else:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = detector.detectMarkers(gray)

        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                detected_ids.add(int(marker_id))
                detection_count += 1
                c = corners[i][0]
                cx_px = np.mean(c[:, 0])
                cy_px = np.mean(c[:, 1])
                print(
                    f"[{elapsed:.1f}s] Marker ID {marker_id} detected! "
                    f"Center: ({cx_px:.0f}, {cy_px:.0f}) px"
                )
        elif frame_count % 15 == 0:
            print(
                f"[{elapsed:.1f}s] Frame {frame_count}: "
                f"no markers ({len(rejected)} rejected candidates)"
            )

    sub = session.declare_subscriber(args.image_key, image_callback)
    print(f"ArUco detection test running for {args.duration}s")
    print(f"  Image key:  {args.image_key}")
    print(f"  Dictionary: DICT_4X4_100")
    print(f"  Looking for marker IDs: 60, 80")
    print(f"  Marker size: {marker_size}m")
    print()

    try:
        while time.time() - start_time < args.duration:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass

    sub.undeclare()
    session.close()

    print()
    print("=" * 50)
    print(f"Test complete. {frame_count} frames, {detection_count} detections.")
    if detected_ids:
        print(f"PASS: Detected marker IDs: {sorted(detected_ids)}")
        expected = {60, 80}
        missing = expected - detected_ids
        if missing:
            print(f"  Missing expected IDs: {sorted(missing)}")
        else:
            print("  All expected markers (60, 80) detected!")
    else:
        print("FAIL: No markers detected.")
        print("  Ensure robot camera is facing an ArUco marker panel.")


if __name__ == "__main__":
    main()
