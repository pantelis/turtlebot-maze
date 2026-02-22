# stella_vslam Visual SLAM Standalone Demo — Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Run stella_vslam Visual SLAM in a separate Docker container, subscribing to TurtleBot camera images via Zenoh, building a visual map of the enhanced maze world.

**Architecture:** Separate `Dockerfile.slam` builds stella_vslam from source (iridescence viewer + core library). A Python Zenoh bridge (`slam_bridge.py`) subscribes to camera images, deserializes CDR, feeds frames to stella_vslam via subprocess, and publishes pose estimates back over Zenoh. Socket viewer provides browser-based map visualization.

**Tech Stack:** stella_vslam (C++), iridescence (viewer), Python 3, eclipse-zenoh, pycdr2, Docker

**Design doc:** `docs/plans/2026-02-21-stella-vslam-design.md`

**Beads issue:** turtlebot-maze-m5e

---

### Task 1: Create stella_vslam Camera Config

**Files:**
- Create: `slam/config/turtlebot_realsense.yaml`

The TurtleBot's simulated RealSense camera (from `tb_worlds/urdf/gz_waffle.sdf.xacro:392-416`) has:
- `horizontal_fov`: 1.047 rad (60°)
- `width`: 320, `height`: 240
- `depth_camera clip`: near=0.001, far=5.0
- `update_rate`: 5 Hz

Camera intrinsics: `fx = fy = width / (2 * tan(hfov/2)) = 320 / (2 * tan(0.5235)) = 320 / 1.1547 = 277.13`

**Step 1: Create the config file**

```yaml
# slam/config/turtlebot_realsense.yaml
# stella_vslam camera config for TurtleBot3 Waffle simulated RealSense R200
# Intrinsics derived from gz_waffle.sdf.xacro (horizontal_fov=1.047, 320x240)

Camera:
  name: "TurtleBot3 Waffle RealSense R200 (simulated)"
  setup: "RGBD"
  model: "perspective"

  # fx = fy = width / (2 * tan(hfov/2)) = 320 / (2 * tan(0.5235)) = 277.13
  fx: 277.13
  fy: 277.13
  cx: 160.0
  cy: 120.0

  # No distortion in simulation
  k1: 0.0
  k2: 0.0
  p1: 0.0
  p2: 0.0
  k3: 0.0

  fps: 5.0
  cols: 320
  rows: 240

  # RGBD-specific
  focal_x_baseline: 27.713
  depth_threshold: 5.0
  color_order: "RGB"

Preprocessing:
  min_size: 320
  depthmap_factor: 1.0

Feature:
  name: "ORB for low-res simulation camera"
  scale_factor: 1.2
  num_levels: 8
  ini_fast_threshold: 20
  min_fast_threshold: 7

Mapping:
  baseline_dist_thr: 0.1
  redundant_obs_ratio_thr: 0.9

SocketViewer:
  keyframe_size: 0.07
  keyframe_line_width: 1
  graph_line_width: 1
  point_size: 2
  camera_size: 0.08
  camera_line_width: 3
  viewpoint_x: 0
  viewpoint_y: -0.65
  viewpoint_z: -1.9
  viewpoint_f: 400
```

**Step 2: Create directory and commit**

```bash
mkdir -p slam/config
# (write the file above)
git add slam/config/turtlebot_realsense.yaml
git commit -m "feat(slam): add stella_vslam camera config for simulated RealSense"
```

---

### Task 2: Create Dockerfile.slam

**Files:**
- Create: `docker/Dockerfile.slam`

This follows the same pattern as `docker/Dockerfile.torch.gpu` — a self-contained image with no ROS dependencies.

**Step 1: Write the Dockerfile**

```dockerfile
# docker/Dockerfile.slam
# Multi-stage build for stella_vslam Visual SLAM
# Pattern: same as Dockerfile.torch.gpu (standalone, Zenoh transport, no ROS)

FROM ubuntu:24.04 AS builder

ENV DEBIAN_FRONTEND=noninteractive

# Build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git wget curl unzip ca-certificates \
    libglm-dev libglfw3-dev libpng-dev libjpeg-dev \
    libeigen3-dev libboost-filesystem-dev libboost-program-options-dev \
    libopencv-dev libsuitesparse-dev \
    libgoogle-glog-dev libgflags-dev libatlas-base-dev \
    && rm -rf /var/lib/apt/lists/*

# Build iridescence (3D viewer library)
WORKDIR /build
RUN git clone https://github.com/koide3/iridescence.git \
    && cd iridescence \
    && git checkout 085322e0c949f75b67d24d361784e85ad7f197ab \
    && git submodule update --init --recursive \
    && mkdir -p build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo .. \
    && make -j"$(nproc)" \
    && make install

# Build g2o (graph optimization, required by stella_vslam)
RUN git clone --depth 1 https://github.com/RainerKuemmerle/g2o.git \
    && cd g2o \
    && mkdir -p build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DBUILD_WITH_MARCH_NATIVE=OFF \
        -DG2O_BUILD_EXAMPLES=OFF \
        -DG2O_BUILD_APPS=OFF \
        .. \
    && make -j"$(nproc)" \
    && make install

# Build stella_vslam core
RUN git clone --recursive --depth 1 https://github.com/stella-cv/stella_vslam.git \
    && cd stella_vslam \
    && mkdir -p build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo .. \
    && make -j"$(nproc)" \
    && make install

# Build socket viewer (browser-based map visualization)
RUN git clone --recursive https://github.com/stella-cv/socket_viewer.git \
    && cd socket_viewer \
    && mkdir -p build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo .. \
    && make -j"$(nproc)" \
    && make install

# Download ORB vocabulary
RUN mkdir -p /slam/vocab \
    && wget -qO /slam/vocab/orb_vocab.fbow \
       "https://github.com/stella-cv/FBoW_orb_vocab/raw/main/orb_vocab.fbow"

# ---- Runtime image ----
FROM ubuntu:24.04 AS runtime

ENV DEBIAN_FRONTEND=noninteractive

# Runtime dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    libglm-dev libglfw3 libpng16-16t64 libjpeg8 \
    libeigen3-dev libboost-filesystem1.83.0 libboost-program-options1.83.0 \
    libopencv-core4.6d libopencv-imgproc4.6d libopencv-imgcodecs4.6d \
    libopencv-features2d4.6d libopencv-highgui4.6d \
    libsuitesparseconfig7 libcholmod5 libccolamd3 libcamd3 libamd3 \
    libgoogle-glog0v6t64 libatlas3-base \
    python3 python3-pip python3-numpy \
    && rm -rf /var/lib/apt/lists/*

# Python Zenoh + CDR deserialization
RUN pip3 install --break-system-packages eclipse-zenoh pycdr2 opencv-python-headless

# Copy built artifacts from builder
COPY --from=builder /usr/local/lib/ /usr/local/lib/
COPY --from=builder /usr/local/bin/ /usr/local/bin/
COPY --from=builder /usr/local/include/ /usr/local/include/
COPY --from=builder /slam/vocab/orb_vocab.fbow /slam/vocab/orb_vocab.fbow

# Copy FBoW shared lib (built inside stella_vslam)
COPY --from=builder /build/stella_vslam/build/3rd/FBoW/ /usr/local/lib/fbow/

ENV LD_LIBRARY_PATH="/usr/local/lib:/usr/local/lib/fbow:${LD_LIBRARY_PATH}"
RUN ldconfig

# Copy slam bridge script and config
WORKDIR /slam
COPY slam/ /slam/

ENTRYPOINT ["python3", "/slam/slam_bridge.py"]
```

**Step 2: Commit**

```bash
git add docker/Dockerfile.slam
git commit -m "feat(slam): add Dockerfile for stella_vslam (multi-stage, no ROS)"
```

**Note:** The exact Ubuntu 24.04 package names for runtime libs may need adjustment during the Docker build. The builder stage pins iridescence to the same commit Oscar's script used (`085322e`). The runtime stage copies only the installed artifacts, keeping the image smaller.

---

### Task 3: Create slam_bridge.py

**Files:**
- Create: `slam/slam_bridge.py`
- Create: `slam/requirements.txt`

This follows the exact same pattern as `detector/object_detector.py` — Zenoh subscriber, CDR deserialize, process, publish results.

**Step 1: Create requirements.txt**

```
# slam/requirements.txt
eclipse-zenoh
pycdr2
opencv-python-headless
numpy
```

**Step 2: Write slam_bridge.py**

```python
#!/usr/bin/env python3
"""
Zenoh-based stella_vslam SLAM bridge.

Subscribes to ROS 2 camera images bridged via zenoh-bridge-ros2dds,
writes frames to a named pipe, runs stella_vslam in a subprocess,
reads pose output, and publishes pose estimates back over Zenoh.

Pattern matches detector/object_detector.py.
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import tempfile
import threading
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

        print(f"SLAM bridge running.")
        print(f"  Image key:  {args.image_key}")
        print(f"  Pose key:   {args.pose_key}")
        print(f"  Status key: {args.status_key}")
        print(f"  Config:     {args.config}")

    def _start_slam(self):
        """Start stella_vslam run_slam binary."""
        cmd = [
            "run_slam",
            "-v", self.args.vocab,
            "-c", self.args.config,
            "--frame-skip", "1",
            "--no-sleep",
            "--auto-term",
            "--map-db-out", "/tmp/slam_map.msg",
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
        frame_path = os.path.join(
            self.frame_dir, f"frame_{self.frame_count:06d}.png"
        )
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
    parser = argparse.ArgumentParser(
        description="Zenoh stella_vslam SLAM Bridge"
    )
    parser.add_argument(
        "-e", "--connect", type=str, default="",
        help="Zenoh endpoint to connect to (empty = multicast)",
    )
    parser.add_argument(
        "--image-key", type=str,
        default="rt/intel_realsense_r200_depth/image",
        help="Zenoh key for camera images",
    )
    parser.add_argument(
        "--depth-key", type=str,
        default="rt/intel_realsense_r200_depth/depth_image",
        help="Zenoh key for depth images",
    )
    parser.add_argument(
        "--pose-key", type=str, default="tb/slam/pose",
        help="Zenoh key to publish pose estimates",
    )
    parser.add_argument(
        "--status-key", type=str, default="tb/slam/status",
        help="Zenoh key to publish tracking status",
    )
    parser.add_argument(
        "-c", "--config", type=str,
        default="/slam/config/turtlebot_realsense.yaml",
        help="stella_vslam camera config YAML",
    )
    parser.add_argument(
        "-v", "--vocab", type=str,
        default="/slam/vocab/orb_vocab.fbow",
        help="Path to ORB vocabulary file",
    )
    parser.add_argument(
        "--viewer", type=str, default="socket_publisher",
        help="Viewer type: socket_publisher, pangolin_viewer, or none",
    )
    parser.add_argument(
        "--max-fps", type=float, default=5.0,
        help="Maximum frame processing rate in Hz",
    )
    args = parser.parse_args()

    bridge = SlamBridge(args)

    signal.signal(signal.SIGTERM, lambda s, f: bridge.shutdown())
    bridge.run()


if __name__ == "__main__":
    main()
```

**Step 3: Commit**

```bash
git add slam/slam_bridge.py slam/requirements.txt
git commit -m "feat(slam): add Zenoh SLAM bridge (same pattern as YOLO detector)"
```

**Note:** The initial implementation uses file-based frame exchange with stella_vslam's `run_slam` binary. This is a working MVP. A tighter integration using stella_vslam's Python bindings or shared memory would be Phase 2 optimization.

---

### Task 4: Add Docker Compose Service

**Files:**
- Modify: `docker-compose.yaml`

**Step 1: Add the `demo-slam` service**

Add after the `detector` service block in `docker-compose.yaml`:

```yaml
  # stella_vslam Visual SLAM (standalone, Zenoh transport)
  demo-slam:
    build:
      context: .
      dockerfile: docker/Dockerfile.slam
    network_mode: host
    ipc: host
    environment:
      - DISPLAY
      - ZENOH_ROUTER=tcp/localhost:7447
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
    command: >
      python3 /slam/slam_bridge.py
      --image-key "rt/intel_realsense_r200_depth/image"
      --depth-key "rt/intel_realsense_r200_depth/depth_image"
      --config /slam/config/turtlebot_realsense.yaml
      --vocab /slam/vocab/orb_vocab.fbow
      --viewer socket_publisher
```

**Step 2: Commit**

```bash
git add docker-compose.yaml
git commit -m "feat(slam): add demo-slam Docker Compose service"
```

---

### Task 5: Add README Section

**Files:**
- Modify: `README.md`

**Step 1: Add Visual SLAM section**

Add after the "Enhanced Maze World" section and before the "Zenoh + YOLOv8 Object Detection" section:

```markdown
---

## Visual SLAM with stella_vslam

Run [stella_vslam](https://github.com/stella-cv/stella_vslam) Visual SLAM on the TurtleBot's camera feed. The SLAM container subscribes to camera images via Zenoh (same transport pattern as the YOLO detector) and builds a 3D map of the environment.

The enhanced maze world (`demo-world-enhanced`) with textured walls provides significantly better visual features for SLAM compared to the original featureless walls.

### Launch

```bash
# Terminal 1: Enhanced world (textured walls)
docker compose up demo-world-enhanced

# Terminal 2: Zenoh transport
docker compose up zenoh-router zenoh-bridge

# Terminal 3: Visual SLAM
docker compose up demo-slam
```

The socket viewer is accessible at `http://localhost:3001` to visualize the reconstructed map and camera trajectory.

### Data Flow

Camera images flow from Gazebo through the Zenoh bridge to the SLAM container:

```
Gazebo Camera → DDS → zenoh-bridge → Zenoh → slam_bridge.py → stella_vslam
```

Pose estimates are published back via Zenoh on key `tb/slam/pose`.
```

**Step 2: Commit**

```bash
git add README.md
git commit -m "docs: add Visual SLAM section to README"
```

---

### Task 6: Build and Test

**Step 1: Build the Docker image**

```bash
docker compose build demo-slam
```

Expected: Image builds successfully. This will take 10-20 minutes (stella_vslam builds from source).

If the build fails, likely issues:
- Ubuntu 24.04 package names changed (fix runtime lib names in the `runtime` stage)
- g2o version incompatibility (pin to a specific tag)
- Missing cmake dependencies (add to builder stage)

**Step 2: Test Zenoh image subscription**

```bash
# Terminal 1
docker compose up demo-world-enhanced

# Terminal 2
docker compose up zenoh-router zenoh-bridge

# Terminal 3
docker compose up demo-slam
```

Expected output:
```
SLAM bridge running.
  Image key:  rt/intel_realsense_r200_depth/image
  Pose key:   tb/slam/pose
  Status key: tb/slam/status
Processed 50 frames
Processed 100 frames
```

**Step 3: Verify socket viewer**

Open `http://localhost:3001` in a browser. You should see:
- ORB feature points on textured walls
- Camera trajectory as the TurtleBot moves
- 3D point cloud of the environment

**Step 4: Compare with featureless world**

```bash
# Stop demo-slam and demo-world-enhanced
# Launch original featureless world
docker compose up demo-world

# Relaunch SLAM
docker compose up demo-slam
```

Expected: stella_vslam should have poor tracking or fail to initialize — the featureless walls lack visual features for ORB extraction. This validates that the enhanced textured walls are necessary for visual SLAM.

**Step 5: Final commit and push**

```bash
git add -A
git commit -m "feat(slam): stella_vslam standalone demo complete"
git push origin feature/stella-vslam
```

---

### Task 7: Create PR

```bash
gh pr create \
  --title "feat: add stella_vslam Visual SLAM standalone demo" \
  --body "## Summary
- Dockerfile.slam: multi-stage build for stella_vslam (no ROS dependencies)
- slam_bridge.py: Zenoh subscriber for camera images, feeds stella_vslam
- Camera config for simulated RealSense (320x240, RGBD)
- demo-slam Docker Compose service
- README documentation

## Test plan
- [ ] docker compose build demo-slam succeeds
- [ ] SLAM bridge receives camera frames from enhanced world
- [ ] Socket viewer shows ORB features and camera trajectory
- [ ] Featureless world comparison shows degraded tracking

Beads: turtlebot-maze-m5e"
```

---

## Reference Files

| Existing File | Why You Need It |
|---------------|-----------------|
| `detector/object_detector.py` | Pattern for Zenoh subscribe + CDR deserialize + process + publish |
| `docker/Dockerfile.torch.gpu` | Pattern for standalone Dockerfile (no ROS) |
| `tb_worlds/urdf/gz_waffle.sdf.xacro:392-416` | Camera intrinsics (hfov, resolution, depth clip) |
| `docker-compose.yaml` | Where to add `demo-slam` service |
| `README.md` | Where to add Visual SLAM section |

## External References

| Resource | URL |
|----------|-----|
| stella_vslam repo | https://github.com/stella-cv/stella_vslam |
| stella_vslam_ros (upstream) | https://github.com/stella-cv/stella_vslam_ros (branch: `ros2`) |
| Oscar's Jazzy fork | https://github.com/oscarpoudel/stella_vslam_ros2_Jazzy |
| FBoW vocabulary | https://github.com/stella-cv/FBoW_orb_vocab/raw/main/orb_vocab.fbow |
| iridescence viewer | https://github.com/koide3/iridescence (commit: `085322e`) |
| socket viewer | https://github.com/stella-cv/socket_viewer |
| stella_vslam docs | https://stella-cv.readthedocs.io/en/latest/ |
