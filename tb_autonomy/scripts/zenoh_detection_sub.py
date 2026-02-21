#!/usr/bin/env python3
"""
ROS 2 node that subscribes to Zenoh detection results and republishes
them for the behavior tree to consume.

Runs a Zenoh subscriber in a background thread. The latest detections
are stored in-memory and queryable by the LookForObject behavior.
"""

import json
import threading

import rclpy
from rclpy.node import Node
import zenoh


class ZenohDetectionSubscriber(Node):
    """Bridges Zenoh detection results into ROS 2 for behavior tree consumption."""

    def __init__(self):
        super().__init__("zenoh_detection_sub")
        self.declare_parameter("zenoh_connect", "")
        self.declare_parameter("detection_key", "tb/detections")

        self._latest_detections = []
        self._lock = threading.Lock()

        # Open Zenoh session in background thread
        connect = self.get_parameter("zenoh_connect").value
        detection_key = self.get_parameter("detection_key").value

        conf = zenoh.Config()
        if connect:
            conf.insert_json5("connect/endpoints", json.dumps([connect]))

        self._session = zenoh.open(conf)
        self._sub = self._session.declare_subscriber(
            detection_key, self._detection_callback
        )
        self.get_logger().info(f"Subscribed to Zenoh key: {detection_key}")

    def _detection_callback(self, sample):
        try:
            detections = json.loads(bytes(sample.payload).decode())
            with self._lock:
                self._latest_detections = detections
        except (json.JSONDecodeError, UnicodeDecodeError) as e:
            self.get_logger().warn(f"Failed to parse detections: {e}")

    def get_detections(self):
        """Returns the latest detections list (thread-safe)."""
        with self._lock:
            return list(self._latest_detections)

    def destroy_node(self):
        self._sub.undeclare()
        self._session.close()
        super().destroy_node()


# Global singleton for behavior tree access
_instance = None


def get_detection_subscriber():
    """Returns the singleton ZenohDetectionSubscriber instance."""
    global _instance
    return _instance


def main(args=None):
    global _instance
    rclpy.init(args=args)
    _instance = ZenohDetectionSubscriber()
    rclpy.spin(_instance)
    _instance.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
