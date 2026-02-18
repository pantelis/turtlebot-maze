"""
Vision behaviors for TurtleBot.
"""

import json
import threading

import cv2
import cv_bridge
import rclpy
from rclpy.duration import Duration
import py_trees
from sensor_msgs.msg import Image

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

# Define HSV color space thresholds
hsv_threshold_dict = {
    "red": ((160, 220, 0), (180, 255, 255)),
    "green": ((40, 220, 0), (90, 255, 255)),
    "blue": ((100, 220, 0), (150, 255, 255)),
}


class LookForObject(py_trees.behaviour.Behaviour):
    """
    Gets images from the robot and looks for object using either:
    - HSV color space thresholding and blob detection (detector_type="hsv")
    - YOLOv8 detections via Zenoh subscriber (detector_type="yolo")
    """

    def __init__(
        self,
        name,
        color,
        node,
        img_timeout=3.0,
        visualize=True,
        detector_type="hsv",
        target_object="cup",
    ):
        super(LookForObject, self).__init__(name)
        self.color = color
        self.node = node
        self.img_timeout = Duration(nanoseconds=img_timeout * 1e9)
        self.viz_window_name = "Image with Detections"
        self.visualize = visualize
        self.detector_type = detector_type
        self.target_object = target_object

        # HSV-specific setup
        if self.detector_type == "hsv":
            self.hsv_min = hsv_threshold_dict[color][0]
            self.hsv_max = hsv_threshold_dict[color][1]
            if self.visualize:
                plt.figure(1)
                plt.axis("off")
                plt.title(self.viz_window_name)
                plt.ion()

        # YOLO-specific setup
        if self.detector_type == "yolo":
            self._zenoh_detections = []
            self._zenoh_lock = threading.Lock()
            self._zenoh_session = None
            self._zenoh_sub = None

    def initialise(self):
        """Starts all the vision related objects"""
        self.start_time = self.node.get_clock().now()

        if self.detector_type == "hsv":
            self.bridge = cv_bridge.CvBridge()
            params = cv2.SimpleBlobDetector_Params()
            params.minArea = 100
            params.maxArea = 100000
            params.filterByArea = True
            params.filterByColor = False
            params.filterByInertia = False
            params.filterByConvexity = False
            params.thresholdStep = 50
            self.detector = cv2.SimpleBlobDetector_create(params)
            self.latest_img_msg = None
            self.img_sub = self.node.create_subscription(
                Image, "/camera/image_raw", self.img_callback, 10
            )

        elif self.detector_type == "yolo":
            import zenoh

            conf = zenoh.Config()
            self._zenoh_session = zenoh.open(conf)
            self._zenoh_sub = self._zenoh_session.declare_subscriber(
                "tb/detections", self._zenoh_callback
            )
            self.logger.info(
                f"YOLO mode: subscribed to tb/detections, looking for '{self.target_object}'"
            )

    def update(self):
        """Looks for object detection based on detector_type."""
        now = self.node.get_clock().now()

        if self.detector_type == "hsv":
            return self._update_hsv(now)
        elif self.detector_type == "yolo":
            return self._update_yolo(now)

        self.logger.error(f"Unknown detector_type: {self.detector_type}")
        return py_trees.common.Status.FAILURE

    def _update_hsv(self, now):
        """Original HSV thresholding logic (unchanged)."""
        if self.latest_img_msg is None:
            if now - self.start_time < self.img_timeout:
                return py_trees.common.Status.RUNNING
            else:
                self.logger.info("Image timeout exceeded")
                return py_trees.common.Status.FAILURE

        img = self.bridge.imgmsg_to_cv2(self.latest_img_msg, desired_encoding="bgr8")
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_min, self.hsv_max)
        keypoints = self.detector.detect(mask)

        if self.visualize:
            labeled_img = cv2.drawKeypoints(
                img,
                keypoints,
                None,
                (255, 0, 0),
                cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
            )
            cv2.destroyAllWindows()
            cv2.imshow(self.viz_window_name, labeled_img)
            cv2.waitKey(100)

        if len(keypoints) == 0:
            self.logger.info("No objects detected")
            return py_trees.common.Status.FAILURE
        for k in keypoints:
            self.logger.info(f"Detected object at [{k.pt[0]}, {k.pt[1]}]")
        return py_trees.common.Status.SUCCESS

    def _update_yolo(self, now):
        """YOLO detection via Zenoh subscriber."""
        with self._zenoh_lock:
            detections = list(self._zenoh_detections)

        if not detections:
            if now - self.start_time < self.img_timeout:
                return py_trees.common.Status.RUNNING
            else:
                self.logger.info("Detection timeout exceeded (no detections received)")
                return py_trees.common.Status.FAILURE

        # Filter for target object
        matches = [
            d for d in detections if d.get("class", "") == self.target_object
        ]

        if not matches:
            self.logger.info(
                f"No '{self.target_object}' detected "
                f"(got: {[d['class'] for d in detections]})"
            )
            return py_trees.common.Status.FAILURE

        for m in matches:
            self.logger.info(
                f"Detected '{m['class']}' conf={m['confidence']} bbox={m['bbox']}"
            )
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.info(f"Terminated with status {new_status}")
        if self.detector_type == "hsv":
            self.img_sub = None
            self.latest_img_msg = None
        elif self.detector_type == "yolo":
            if self._zenoh_sub:
                self._zenoh_sub.undeclare()
                self._zenoh_sub = None
            if self._zenoh_session:
                self._zenoh_session.close()
                self._zenoh_session = None

    def img_callback(self, msg):
        self.latest_img_msg = msg

    def _zenoh_callback(self, sample):
        try:
            detections = json.loads(bytes(sample.payload).decode())
            with self._zenoh_lock:
                self._zenoh_detections = detections
        except (json.JSONDecodeError, UnicodeDecodeError):
            pass
