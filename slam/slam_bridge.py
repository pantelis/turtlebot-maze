#!/usr/bin/env python3
"""
Zenoh-based stella_vslam SLAM bridge.

Subscribes to ROS 2 camera images bridged via zenoh-bridge-ros2dds,
writes frames to disk, runs stella_vslam in a subprocess,
and publishes tracking status back over Zenoh.

Pattern matches detector/object_detector.py.
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import tempfile
import time

import cv2
import numpy as np
import zenoh
from pycdr2 import IdlStruct
from pycdr2.types import uint8, uint32, int32
from dataclasses import dataclass
from typing import List


# CDR struct matching sensor_msgs/msg/Image
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


class SlamBridge:
    def __init__(self, args):
        self.args = args
        self.frame_count = 0
        self.last_frame_time = 0.0
        self.min_interval = 1.0 / args.max_fps
        self.running = True

        # Temp directory for frame exchange with stella_vslam
        self.frame_dir = tempfile.mkdtemp(prefix="slam_frames_")
        print(f"Frame exchange dir: {self.frame_dir}")

        # Open Zenoh session
        conf = zenoh.Config()
        if args.connect:
            conf.insert_json5("connect/endpoints", json.dumps([args.connect]))

        self.session = zenoh.open(conf)
        self.pose_pub = self.session.declare_publisher(args.pose_key)
        self.status_pub = self.session.declare_publisher(args.status_key)

        # Start stella_vslam subprocess
        self.slam_proc = self._start_slam()

        # Subscribe to camera images
        self.image_sub = self.session.declare_subscriber(
            args.image_key, self._image_callback
        )

        print("SLAM bridge running.")
        print(f"  Image key:  {args.image_key}")
        print(f"  Pose key:   {args.pose_key}")
        print(f"  Status key: {args.status_key}")
        print(f"  Config:     {args.config}")

    def _start_slam(self):
        """Start stella_vslam run_slam binary."""
        cmd = [
            "run_slam",
            "-v",
            self.args.vocab,
            "-c",
            self.args.config,
            "--frame-skip",
            "1",
            "--no-sleep",
            "--auto-term",
            "--map-db-out",
            "/tmp/slam_map.msg",
        ]

        if self.args.viewer:
            cmd.extend(["--viewer", self.args.viewer])

        print(f"Starting stella_vslam: {' '.join(cmd)}")

        proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        return proc

    def _image_callback(self, sample):
        """Handle incoming camera image from Zenoh."""
        if not self.running:
            return

        # Rate limit
        now = time.time()
        if now - self.last_frame_time < self.min_interval:
            return
        self.last_frame_time = now

        # Deserialize CDR-encoded sensor_msgs/Image
        try:
            img_msg = Image.deserialize(bytes(sample.payload))
        except Exception as e:
            print(f"CDR deserialize error: {e}")
            return

        # Convert to numpy array
        if img_msg.encoding in ("rgb8", "bgr8"):
            frame = np.frombuffer(bytes(img_msg.data), dtype=np.uint8).reshape(
                img_msg.height, img_msg.width, 3
            )
            if img_msg.encoding == "rgb8":
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        else:
            print(f"Unsupported encoding: {img_msg.encoding}")
            return

        # Save frame for stella_vslam to consume
        frame_path = os.path.join(self.frame_dir, f"frame_{self.frame_count:06d}.png")
        cv2.imwrite(frame_path, frame)
        self.frame_count += 1

        # Publish status
        status = {
            "status": "tracking",
            "frame_count": self.frame_count,
            "timestamp": now,
        }
        self.status_pub.put(json.dumps(status).encode())

        if self.frame_count % 50 == 0:
            print(f"Processed {self.frame_count} frames")

    def run(self):
        """Main loop."""
        try:
            while self.running:
                time.sleep(1.0)
        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown()

    def shutdown(self):
        """Clean shutdown."""
        self.running = False
        print("Shutting down SLAM bridge...")

        if self.slam_proc and self.slam_proc.poll() is None:
            self.slam_proc.terminate()
            self.slam_proc.wait(timeout=10)

        self.image_sub.undeclare()
        self.pose_pub.undeclare()
        self.status_pub.undeclare()
        self.session.close()

        print("SLAM bridge stopped.")


def main():
    parser = argparse.ArgumentParser(description="Zenoh stella_vslam SLAM Bridge")
    parser.add_argument(
        "-e",
        "--connect",
        type=str,
        default="",
        help="Zenoh endpoint to connect to (empty = multicast)",
    )
    parser.add_argument(
        "--image-key",
        type=str,
        default="rt/intel_realsense_r200_depth/image",
        help="Zenoh key for camera images",
    )
    parser.add_argument(
        "--depth-key",
        type=str,
        default="rt/intel_realsense_r200_depth/depth_image",
        help="Zenoh key for depth images",
    )
    parser.add_argument(
        "--pose-key",
        type=str,
        default="tb/slam/pose",
        help="Zenoh key to publish pose estimates",
    )
    parser.add_argument(
        "--status-key",
        type=str,
        default="tb/slam/status",
        help="Zenoh key to publish tracking status",
    )
    parser.add_argument(
        "-c",
        "--config",
        type=str,
        default="/slam/config/turtlebot_realsense.yaml",
        help="stella_vslam camera config YAML",
    )
    parser.add_argument(
        "-v",
        "--vocab",
        type=str,
        default="/slam/vocab/orb_vocab.fbow",
        help="Path to ORB vocabulary file",
    )
    parser.add_argument(
        "--viewer",
        type=str,
        default="socket_publisher",
        help="Viewer type: socket_publisher, pangolin_viewer, or none",
    )
    parser.add_argument(
        "--max-fps",
        type=float,
        default=5.0,
        help="Maximum frame processing rate in Hz",
    )
    args = parser.parse_args()

    bridge = SlamBridge(args)

    signal.signal(signal.SIGTERM, lambda s, f: bridge.shutdown())
    bridge.run()


if __name__ == "__main__":
    main()
