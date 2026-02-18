# Design: Decoupled PyTorch Object Detector via Zenoh

**Date:** 2026-02-18
**Status:** Approved

## Problem

The current vision pipeline uses HSV color thresholding + OpenCV blob detection to find colored blocks. This is brittle under varying lighting and limited to 3 hardcoded colors. We want to detect general household objects (COCO classes) using YOLOv8, while keeping the ROS 2 and ML containers fully decoupled.

## Architecture

Two separate Docker containers communicate over Zenoh. The `zenoh-bridge-ros2dds` sidecar transparently exposes ROS 2 DDS topics as Zenoh keys (prefix `rt/`). The PyTorch container subscribes to camera frames via Zenoh and publishes detection results back — no ROS 2 installation needed on the detector side.

```
 ROS 2 Container (turtlebot_behavior)          PyTorch Container (detector)
 ─────────────────────────────────────          ────────────────────────────
 Gazebo → /camera/image_raw (DDS)              eclipse-zenoh subscriber
          ↓                                              ↓
 zenoh-bridge-ros2dds (sidecar)                 pycdr2 deserialize Image
   DDS ↔ Zenoh transparent bridge               cv2.imdecode → YOLOv8
          ↓                                              ↓
 Zenoh key: rt/camera/image_raw  ──────────→   object_detector.py
                                                         ↓
 Zenoh key: tb/detections        ←──────────   publish JSON detections
          ↓
 zenoh_detection_sub.py (ROS node)
          ↓
 LookForObject behavior (yolo mode)
```

### Why This Pattern

Following the [zenoh-python-lidar-plot](https://github.com/eclipse-zenoh/zenoh-demos/tree/main/ROS2/zenoh-python-lidar-plot) demo pattern:

- `zenoh-bridge-ros2dds` handles ROS 2 → Zenoh translation automatically (no custom publisher node)
- `pycdr2` deserializes CDR-encoded ROS 2 messages without needing ROS 2 installed
- The detector container is a pure Python process with zero ROS dependencies

Following the [face-recog](https://github.com/eclipse-zenoh/zenoh-demos/tree/main/computer-vision/face-recog) demo pattern:

- Each processing stage is independently deployable
- Zenoh UDP multicast auto-discovery (no endpoint configuration with `network_mode: host`)
- Lightweight serialization (JSON for detections, CDR passthrough for images)

## Zenoh Key Expressions

| Key | Direction | Payload Format | Description |
|-----|-----------|----------------|-------------|
| `rt/camera/image_raw` | ROS → PyTorch | CDR (`sensor_msgs/msg/Image`) | Camera frames (auto-bridged by zenoh-bridge-ros2dds) |
| `tb/detections` | PyTorch → ROS | JSON | Detection results array |

### Detection JSON Schema

```json
[
  {
    "class": "cup",
    "confidence": 0.87,
    "bbox": [x1, y1, x2, y2]
  }
]
```

## Components

### ROS 2 Container (modify existing `Dockerfile.gpu`)

| Component | Location | Change Type | Purpose |
|-----------|----------|-------------|---------|
| `Dockerfile.gpu` | `docker/` | Modify | Add `zenoh-bridge-ros2dds` binary install |
| `zenoh_detection_sub.py` | `tb_autonomy/scripts/` | New | ROS 2 node: subscribes to Zenoh key `tb/detections`, parses JSON, caches latest detections |
| `vision.py` | `tb_autonomy/python/tb_behaviors/` | Modify | Add `detector_type` param (`hsv`\|`yolo`); `yolo` mode reads from detection subscriber |
| `autonomy_node.py` | `tb_autonomy/scripts/` | Modify | Add `detector_type` and `target_object` parameters |
| Launch files | `tb_autonomy/launch/` | Modify | Add `detector_type`, `target_object` launch args; conditionally start zenoh bridge + detection sub |

**New pip dependency in ROS container:** `eclipse-zenoh` (for the detection subscriber only)

### PyTorch Container (new)

| Component | Location | Change Type | Purpose |
|-----------|----------|-------------|---------|
| `Dockerfile.torch.gpu` | `docker/` | New | Based on `pytorch/pytorch:2.7.1-cuda12.8-cudnn9-runtime` (eng-ai-agents pattern) |
| `object_detector.py` | `detector/` | New | Subscribes Zenoh `rt/camera/image_raw`, deserializes CDR Image, runs YOLOv8, publishes JSON to `tb/detections` |
| `requirements.txt` | `detector/` | New | `ultralytics`, `eclipse-zenoh`, `pycdr2`, `opencv-python-headless`, `numpy` |

### Docker Compose (modify existing)

| Service | Image | Purpose |
|---------|-------|---------|
| `zenoh-router` | `eclipse/zenoh:latest` | Zenoh router (optional for multi-host, harmless for single-host) |
| `zenoh-bridge` | ROS container + bridge cmd | Runs `zenoh-bridge-ros2dds` as sidecar alongside ROS 2 |
| `detector` | `Dockerfile.torch.gpu` | PyTorch YOLOv8 detector |

## Data Flow (step by step)

```
1. Gazebo camera plugin publishes sensor_msgs/Image to /camera/image_raw (DDS)
2. zenoh-bridge-ros2dds (sidecar) picks up DDS topic, exposes as Zenoh key rt/camera/image_raw
3. object_detector.py (PyTorch container) subscribes to rt/camera/image_raw via eclipse-zenoh
4. pycdr2 deserializes CDR bytes into Image fields (height, width, encoding, data)
5. numpy reshape + cv2 color convert → BGR frame
6. ultralytics model.predict(frame, conf=threshold) → list of detections
7. Filter + serialize to JSON, publish to Zenoh key tb/detections
8. zenoh_detection_sub.py (ROS container) subscribes to tb/detections via eclipse-zenoh
9. JSON parsed, latest detections cached in-memory
10. LookForObject behavior (yolo mode) checks cached detections for target_object match
11. Match found → BT SUCCESS; no match → BT FAILURE
```

## Switchability

The `detector_type` parameter on the autonomy node controls the pipeline:

| `detector_type` | Vision Pipeline | Requires |
|-----------------|-----------------|----------|
| `hsv` (default) | Existing HSV thresholding + blob detect | Nothing extra |
| `yolo` | Zenoh bridge + PyTorch container | `detector` + `zenoh-bridge` services running |

This means `docker compose up demo-world demo-behavior-py` still works without the detector for basic demos.

## New Parameters

| Parameter | Node | Default | Description |
|-----------|------|---------|-------------|
| `detector_type` | `autonomy_node` | `"hsv"` | `hsv` or `yolo` |
| `target_object` | `autonomy_node` | `"cup"` | COCO class name to search for (yolo mode) |
| `confidence_threshold` | `object_detector.py` | `0.5` | Minimum detection confidence |
| `model_name` | `object_detector.py` | `"yolov8n.pt"` | Ultralytics model weights |
| `zenoh_connect` | both | `""` | Zenoh endpoint (empty = multicast auto-discovery) |

## Dockerfile.torch.gpu (new, based on eng-ai-agents pattern)

```dockerfile
FROM pytorch/pytorch:2.7.1-cuda12.8-cudnn9-runtime

ARG USERNAME=vscode
ARG USER_UID=1001
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && apt-get update && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update && DEBIAN_FRONTEND=noninteractive \
  apt-get -y install --no-install-recommends \
  curl git ffmpeg libgl1 libglib2.0-0 \
  && rm -rf /var/lib/apt/lists/*

COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/

RUN pip list --format=freeze > /etc/pip/constraint.txt

COPY detector/requirements.txt /tmp/requirements.txt
RUN pip install -r /tmp/requirements.txt

USER $USERNAME
RUN git config --global commit.gpgsign false
```

## docker-compose.yaml Additions

```yaml
  # Zenoh router
  zenoh-router:
    image: eclipse/zenoh:latest
    network_mode: host

  # Zenoh DDS bridge (exposes ROS 2 topics as Zenoh keys)
  zenoh-bridge:
    extends: overlay
    network_mode: host
    command: zenoh-bridge-ros2dds

  # PyTorch object detector
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
    command: python object_detector.py
```

## Testing Strategy

1. **Unit test `object_detector.py`** — feed a test image via Zenoh, verify JSON output
2. **Integration test** — run full pipeline: Gazebo → bridge → detector → behavior tree finds object
3. **HSV regression** — verify `detector_type=hsv` still works unchanged
4. **No-detector graceful degradation** — `detector_type=yolo` without detector container running should timeout and return FAILURE, not crash

## File Tree (new/modified)

```
turtlebot-maze/
  detector/                          # NEW directory
    object_detector.py               # Zenoh subscriber + YOLOv8 inference
    requirements.txt                 # ultralytics, eclipse-zenoh, pycdr2, etc.
  docker/
    Dockerfile.gpu                   # MODIFY: add zenoh-bridge-ros2dds install
    Dockerfile.torch.gpu             # NEW: PyTorch detector container
  docker-compose.yaml                # MODIFY: add zenoh-router, zenoh-bridge, detector
  tb_autonomy/
    scripts/
      autonomy_node.py               # MODIFY: add detector_type, target_object params
      zenoh_detection_sub.py          # NEW: Zenoh → ROS detection bridge node
    python/tb_behaviors/
      vision.py                      # MODIFY: add yolo mode to LookForObject
    launch/
      tb_demo_behavior_py.launch.py   # MODIFY: add new launch args
      tb_demo_behavior_cpp.launch.py  # MODIFY: add new launch args
```
