# PyTorch Object Detector via Zenoh — Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add a YOLOv8 object detector running in a separate PyTorch Docker container, communicating with the ROS 2 simulation via Zenoh pub/sub.

**Architecture:** `zenoh-bridge-ros2dds` runs as a sidecar in the ROS 2 container, transparently bridging DDS topics to Zenoh keys (prefix `rt/`). A standalone Python detector in a PyTorch container subscribes to camera frames via `pycdr2` CDR deserialization, runs YOLOv8, and publishes JSON detections back over Zenoh. The existing `LookForObject` behavior gains a `detector_type` parameter to switch between HSV and YOLO modes.

**Tech Stack:** Ultralytics YOLOv8, eclipse-zenoh, pycdr2, pytorch/pytorch:2.7.1-cuda12.8-cudnn9-runtime

**Design Doc:** `docs/plans/2026-02-18-pytorch-object-detector-design.md`

---

## Task 1: Create PyTorch Detector Container

**Files:**
- Create: `docker/Dockerfile.torch.gpu`
- Create: `detector/requirements.txt`

**Step 1: Create `detector/requirements.txt`**

```
ultralytics>=8.0.0
eclipse-zenoh>=1.0.0
pycdr2>=0.4.0
opencv-python-headless>=4.8.0
numpy>=1.24.0
```

**Step 2: Create `docker/Dockerfile.torch.gpu`**

Based on `/home/pantelis.monogioudis/local/web/sites/courses/course-containers/eng-ai-agents/docker/Dockerfile.torch.dev.gpu`:

```dockerfile
FROM pytorch/pytorch:2.7.1-cuda12.8-cudnn9-runtime

ARG USERNAME=vscode
ARG USER_UID=1001
ARG USER_GID=$USER_UID

# Create non-root user (eng-ai-agents pattern)
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc \
  && rm -rf /var/lib/apt/lists/*

# System packages
RUN apt-get update && DEBIAN_FRONTEND=noninteractive \
  apt-get -y install --no-install-recommends \
  curl git ffmpeg libgl1 libglib2.0-0 make bash-completion \
  && rm -rf /var/lib/apt/lists/*

# uv package manager (eng-ai-agents pattern)
COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/

# Freeze constraint file to respect base image packages
RUN mkdir -p /etc/pip \
  && pip list --format=freeze > /etc/pip/constraint.txt

# Install detector dependencies
COPY detector/requirements.txt /tmp/requirements.txt
RUN pip install -r /tmp/requirements.txt

# Download YOLOv8 nano weights at build time (avoids runtime download)
RUN python -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"

USER $USERNAME
RUN git config --global commit.gpgsign false
```

**Step 3: Verify build**

Run: `docker build -f docker/Dockerfile.torch.gpu -t turtlebot_detector:dev .`
Expected: Successful build, YOLOv8 weights cached in image.

**Step 4: Commit**

```bash
git add detector/requirements.txt docker/Dockerfile.torch.gpu
git commit -m "feat: add PyTorch detector Dockerfile based on eng-ai-agents pattern"
```

---

## Task 2: Write the Zenoh Object Detector Script

**Files:**
- Create: `detector/object_detector.py`

**Step 1: Create `detector/object_detector.py`**

This follows the [zenoh-python-lidar-plot](https://github.com/eclipse-zenoh/zenoh-demos/tree/main/ROS2/zenoh-python-lidar-plot) pattern: subscribe to a CDR-encoded ROS 2 topic via Zenoh, deserialize with `pycdr2`, process, publish results back.

```python
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
    parser.add_argument("-e", "--connect", type=str, default="",
                        help="Zenoh endpoint to connect to (empty = multicast)")
    parser.add_argument("-m", "--model", type=str, default="yolov8n.pt",
                        help="Ultralytics model name or path")
    parser.add_argument("-c", "--confidence", type=float, default=0.5,
                        help="Minimum detection confidence")
    parser.add_argument("--image-key", type=str, default="rt/camera/image_raw",
                        help="Zenoh key for camera images (bridged from ROS 2)")
    parser.add_argument("--detection-key", type=str, default="tb/detections",
                        help="Zenoh key to publish detections to")
    parser.add_argument("--max-fps", type=float, default=10.0,
                        help="Maximum inference rate in Hz")
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
                detections.append({
                    "class": model.names[cls_id],
                    "confidence": round(float(box.conf[0]), 3),
                    "bbox": [round(float(x), 1) for x in box.xyxy[0].tolist()],
                })

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
```

**Step 2: Smoke test locally (no ROS needed)**

Run: `cd detector && python -c "from object_detector import Image, Header, Time; print('CDR structs OK')"`
Expected: `CDR structs OK`

**Step 3: Commit**

```bash
git add detector/object_detector.py
git commit -m "feat: add Zenoh-based YOLOv8 object detector script"
```

---

## Task 3: Install zenoh-bridge-ros2dds in ROS 2 Dockerfile

**Files:**
- Modify: `docker/Dockerfile.gpu:24-29` (after Cyclone DDS install in base stage)

**Step 1: Add zenoh-bridge-ros2dds install after line 29**

Insert after the Cyclone DDS block (`ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`):

```dockerfile
# Install zenoh-bridge-ros2dds for bridging DDS topics to Zenoh
# See: https://github.com/eclipse-zenoh/zenoh-bridge-ros2dds
RUN apt-get update && apt-get install -y --no-install-recommends wget \
 && ARCH=$(dpkg --print-architecture) \
 && ZENOH_VERSION="1.2.1" \
 && wget -qO /tmp/zenoh-bridge.deb \
    "https://github.com/eclipse-zenoh/zenoh-bridge-ros2dds/releases/download/${ZENOH_VERSION}/zenoh-bridge-ros2dds_${ZENOH_VERSION}-1_${ARCH}.deb" \
 && dpkg -i /tmp/zenoh-bridge.deb \
 && rm /tmp/zenoh-bridge.deb \
 && rm -rf /var/lib/apt/lists/*
```

Also add `eclipse-zenoh` pip package (for the detection subscriber node) to line 24:

```dockerfile
RUN pip3 install --break-system-packages matplotlib transforms3d eclipse-zenoh
```

**Step 2: Verify build**

Run: `docker compose build base`
Expected: Successful build. `zenoh-bridge-ros2dds` binary present in image.

**Step 3: Commit**

```bash
git add docker/Dockerfile.gpu
git commit -m "feat: add zenoh-bridge-ros2dds and eclipse-zenoh to ROS container"
```

---

## Task 4: Add Zenoh Detection Subscriber Node

**Files:**
- Create: `tb_autonomy/scripts/zenoh_detection_sub.py`
- Modify: `tb_autonomy/CMakeLists.txt:38-43` (add to install scripts list)

**Step 1: Create `tb_autonomy/scripts/zenoh_detection_sub.py`**

```python
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
        self.get_logger().info(
            f"Subscribed to Zenoh key: {detection_key}"
        )

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
```

**Step 2: Add to CMakeLists.txt install scripts**

In `tb_autonomy/CMakeLists.txt`, add `scripts/zenoh_detection_sub.py` to the `install(PROGRAMS ...)` block (line 38-43):

```cmake
install(PROGRAMS
    scripts/autonomy_node.py
    scripts/test_move_base.py
    scripts/test_vision.py
    scripts/zenoh_detection_sub.py
    DESTINATION lib/${PROJECT_NAME}
)
```

**Step 3: Commit**

```bash
git add tb_autonomy/scripts/zenoh_detection_sub.py tb_autonomy/CMakeLists.txt
git commit -m "feat: add Zenoh detection subscriber ROS 2 node"
```

---

## Task 5: Add YOLO Mode to LookForObject Behavior

**Files:**
- Modify: `tb_autonomy/python/tb_behaviors/vision.py`

**Step 1: Modify `vision.py` to support `detector_type` parameter**

Replace the entire file. Key changes:
- `LookForObject.__init__` accepts `detector_type` and `target_object` params
- `update()` branches on `detector_type`: `"hsv"` runs existing logic, `"yolo"` reads from Zenoh subscriber
- HSV path is completely unchanged

```python
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

        # YOLO-specific setup: Zenoh detection subscriber
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
```

**Step 2: Verify HSV mode unchanged**

Run: `cd /overlay_ws && colcon build --packages-select tb_autonomy --symlink-install`
Expected: Build succeeds. HSV-only tests still pass.

**Step 3: Commit**

```bash
git add tb_autonomy/python/tb_behaviors/vision.py
git commit -m "feat: add yolo detector_type to LookForObject behavior via Zenoh"
```

---

## Task 6: Wire Parameters Through Autonomy Node and Launch Files

**Files:**
- Modify: `tb_autonomy/scripts/autonomy_node.py:39-41,53-54,88-91,134`
- Modify: `tb_autonomy/launch/tb_demo_behavior_py.launch.py:14-56`
- Modify: `tb_autonomy/launch/tb_demo_behavior_cpp.launch.py` (launch args only, C++ behavior unchanged)
- Modify: `.env`

**Step 1: Modify `autonomy_node.py`**

Add two new parameters after line 41:

```python
self.declare_parameter("detector_type", value="hsv")
self.declare_parameter("target_object", value="cup")
```

Read them after line 53:

```python
self.detector_type = self.get_parameter("detector_type").value
self.target_object = self.get_parameter("target_object").value
```

Update all `LookForObject(...)` constructor calls to pass through:

In `create_naive_tree` (line 88-91), change:
```python
LookForObject(
    f"find_{self.target_color}_{loc}",
    self.target_color,
    tree.node,
),
```
to:
```python
LookForObject(
    f"find_{self.target_color}_{loc}",
    self.target_color,
    tree.node,
    detector_type=self.detector_type,
    target_object=self.target_object,
),
```

In `create_queue_tree` (line 134), change:
```python
LookForObject(f"find_{self.target_color}", self.target_color, tree.node)
```
to:
```python
LookForObject(
    f"find_{self.target_color}",
    self.target_color,
    tree.node,
    detector_type=self.detector_type,
    target_object=self.target_object,
)
```

Update log line at line 57:
```python
if self.detector_type == "yolo":
    self.tree.node.get_logger().info(
        f"YOLO mode: looking for object '{self.target_object}'..."
    )
else:
    self.tree.node.get_logger().info(f"HSV mode: looking for color {self.target_color}...")
```

**Step 2: Modify `tb_demo_behavior_py.launch.py`**

Add new launch arguments after the `enable_vision` declaration:

```python
DeclareLaunchArgument(
    "detector_type",
    default_value=TextSubstitution(text="hsv"),
    description="Detector type: hsv (color threshold) or yolo (YOLOv8 via Zenoh)",
),
DeclareLaunchArgument(
    "target_object",
    default_value=TextSubstitution(text="cup"),
    description="COCO class name to search for (yolo mode only)",
),
```

Add to the Node parameters dict:

```python
"detector_type": LaunchConfiguration("detector_type"),
"target_object": LaunchConfiguration("target_object"),
```

**Step 3: Modify `tb_demo_behavior_cpp.launch.py`**

Add the same two `DeclareLaunchArgument` blocks. The C++ node won't use them yet, but the launch args are available for future C++ integration.

**Step 4: Add to `.env`**

```
# Object detector type: hsv (default) or yolo (requires detector container)
DETECTOR_TYPE=hsv

# Target object for YOLO detection (COCO class name)
TARGET_OBJECT=cup
```

**Step 5: Commit**

```bash
git add tb_autonomy/scripts/autonomy_node.py \
        tb_autonomy/launch/tb_demo_behavior_py.launch.py \
        tb_autonomy/launch/tb_demo_behavior_cpp.launch.py \
        .env
git commit -m "feat: wire detector_type and target_object params through launch stack"
```

---

## Task 7: Update docker-compose.yaml with Zenoh and Detector Services

**Files:**
- Modify: `docker-compose.yaml`

**Step 1: Add three new services**

Append before the commented-out section (after line 106):

```yaml
  # Zenoh router for cross-container pub/sub
  zenoh-router:
    image: eclipse/zenoh:latest
    network_mode: host
    ipc: host
    command: ["--adminspace-permissions", "rw"]

  # Zenoh DDS bridge (exposes ROS 2 DDS topics as Zenoh keys with rt/ prefix)
  zenoh-bridge:
    extends: overlay
    network_mode: host
    ipc: host
    command: zenoh-bridge-ros2dds

  # PyTorch YOLOv8 object detector (subscribes via Zenoh, no ROS 2 needed)
  detector:
    build:
      context: .
      dockerfile: docker/Dockerfile.torch.gpu
    network_mode: host
    ipc: host
    environment:
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=all
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: all
              capabilities: [gpu]
    volumes:
      - ./detector:/app
    working_dir: /app
    command: python object_detector.py --confidence 0.5 --model yolov8n.pt
```

**Step 2: Update `demo-behavior-py` to pass new env vars**

In the `demo-behavior-py` service command, add the new params:

```yaml
  demo-behavior-py:
    extends: overlay
    command: >
      ros2 launch tb_autonomy tb_demo_behavior_py.launch.py
      tree_type:=${BT_TYPE:?}
      enable_vision:=${ENABLE_VISION:?}
      target_color:=${TARGET_COLOR:?}
      detector_type:=${DETECTOR_TYPE:-hsv}
      target_object:=${TARGET_OBJECT:-cup}
```

Do the same for `demo-behavior-cpp`.

**Step 3: Commit**

```bash
git add docker-compose.yaml
git commit -m "feat: add zenoh-router, zenoh-bridge, and detector services to compose"
```

---

## Task 8: Integration Test — Full Pipeline

**Step 1: Start simulation + Zenoh + detector**

```bash
# Terminal 1: Simulation
docker compose up demo-world

# Terminal 2: Zenoh bridge (bridges DDS topics to Zenoh)
docker compose up zenoh-bridge

# Terminal 3: PyTorch detector (subscribes camera via Zenoh)
docker compose up detector

# Terminal 4: Behavior (yolo mode)
DETECTOR_TYPE=yolo TARGET_OBJECT=cup docker compose up demo-behavior-py
```

Expected: Detector container prints detected objects. Behavior tree finds target object and returns SUCCESS.

**Step 2: Verify HSV regression**

```bash
# Default: detector_type=hsv, no Zenoh/detector needed
docker compose up demo-world demo-behavior-py
```

Expected: Works exactly as before with HSV color thresholding.

**Step 3: Verify graceful degradation**

```bash
# YOLO mode WITHOUT detector running
DETECTOR_TYPE=yolo docker compose up demo-behavior-py
```

Expected: `LookForObject` times out after `img_timeout` seconds and returns FAILURE (no crash).

---

## Task 9: Update Documentation

**Files:**
- Modify: `README.md` — add Zenoh detector section
- Already saved: `docs/plans/2026-02-18-pytorch-object-detector-design.md`

**Step 1: Add to README.md**

Add a section documenting:
- New architecture diagram (Mermaid)
- How to run with YOLO detector
- Environment variables (`DETECTOR_TYPE`, `TARGET_OBJECT`)
- Docker services table

**Step 2: Commit**

```bash
git add README.md
git commit -m "docs: add PyTorch object detector via Zenoh documentation"
```

---

## Summary: File Changes

| File | Action | Task |
|------|--------|------|
| `detector/requirements.txt` | Create | 1 |
| `detector/object_detector.py` | Create | 2 |
| `docker/Dockerfile.torch.gpu` | Create | 1 |
| `docker/Dockerfile.gpu` | Modify (add zenoh-bridge + eclipse-zenoh pip) | 3 |
| `tb_autonomy/scripts/zenoh_detection_sub.py` | Create | 4 |
| `tb_autonomy/CMakeLists.txt` | Modify (add script install) | 4 |
| `tb_autonomy/python/tb_behaviors/vision.py` | Modify (add yolo mode) | 5 |
| `tb_autonomy/scripts/autonomy_node.py` | Modify (add params) | 6 |
| `tb_autonomy/launch/tb_demo_behavior_py.launch.py` | Modify (add launch args) | 6 |
| `tb_autonomy/launch/tb_demo_behavior_cpp.launch.py` | Modify (add launch args) | 6 |
| `.env` | Modify (add DETECTOR_TYPE, TARGET_OBJECT) | 6 |
| `docker-compose.yaml` | Modify (add 3 services) | 7 |
| `README.md` | Modify (add docs) | 9 |

## Execution Dependencies

```
Task 1 (Dockerfile.torch.gpu) ──→ Task 2 (detector script)
Task 3 (Dockerfile.gpu zenoh) ──→ Task 4 (detection sub node)
Task 5 (vision.py yolo mode) ──→ Task 6 (wire params)
Tasks 2,4,6 ──→ Task 7 (compose)
Task 7 ──→ Task 8 (integration test)
Task 8 ──→ Task 9 (docs)
```

Tasks 1-2 and 3-4 and 5-6 can run in parallel.
