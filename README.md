# TurtleBot Behavior Demos

Autonomous navigation demos for a simulated [ROBOTIS TurtleBot](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/#overview) using ROS 2 Jazzy and behavior trees.

The robot navigates a house environment searching for objects using vision (HSV color thresholding or YOLOv8 deep learning) and [Nav2](https://docs.nav2.org/)-based path planning.

**Behavior trees** drive the autonomy — see [this introduction](https://roboticseabass.com/2021/05/08/introduction-to-behavior-trees/) or the [BT textbook](https://arxiv.org/abs/1709.00084).
Docker workflows follow [this guide](https://roboticseabass.com/2023/07/09/updated-guide-docker-and-ros2/).

Originally by Sebastian Castro, 2021-2024.

---

## Architecture

### System Overview

```mermaid
graph LR
    subgraph ROS 2 Container
        GAZ[Gazebo Simulation] -->|sensor_msgs/Image| CAM[/camera/image_raw]
        NAV[Nav2 Stack] -->|navigation| TB[TurtleBot]
        BT[Behavior Tree] -->|goals| NAV
        BT -->|vision query| VIS[LookForObject]
    end

    subgraph Zenoh Transport
        ZB[zenoh-bridge-ros2dds]
        ZR[Zenoh Router]
    end

    subgraph PyTorch Container
        DET[YOLOv8 Detector]
    end

    CAM -->|DDS| ZB
    ZB -->|rt/camera/image_raw| DET
    DET -->|tb/detections JSON| ZB
    ZB -->|Zenoh sub| VIS
```

### Docker Services

| Service | Image / Dockerfile | Purpose |
|---|---|---|
| `base` | `Dockerfile.gpu` → `base` | ROS 2 Jazzy + Cyclone DDS + Gazebo dependencies |
| `overlay` | `Dockerfile.gpu` → `overlay` | Adds `tb_autonomy` + `tb_worlds` packages, Nav2, BT libs |
| `dev` | `Dockerfile.gpu` → `dev` | Development container with source mounts + Groot2 |
| `demo-world` | extends `overlay` | Launches Gazebo house world |
| `demo-behavior-py` | extends `overlay` | Python behavior tree demo (py_trees) |
| `demo-behavior-cpp` | extends `overlay` | C++ behavior tree demo (BehaviorTree.CPP) |
| `zenoh-router` | `eclipse/zenoh:latest` | Zenoh router for pub/sub discovery |
| `zenoh-bridge` | extends `overlay` | `zenoh-bridge-ros2dds` — bridges DDS topics to Zenoh keys |
| `detector` | `Dockerfile.torch.gpu` | PyTorch YOLOv8 object detector (zero ROS dependencies) |

### Vision Pipeline

Two detection modes, switchable via the `DETECTOR_TYPE` environment variable:

```mermaid
graph TD
    subgraph HSV Mode — detector_type=hsv
        IMG1[Camera Image] --> HSV[HSV Threshold + Blob Detection]
        HSV --> MATCH1{Color Match?}
        MATCH1 -->|yes| S1[BT SUCCESS]
        MATCH1 -->|no| F1[BT FAILURE]
    end

    subgraph YOLO Mode — detector_type=yolo
        IMG2[Camera Image] --> ZEN[Zenoh Bridge]
        ZEN --> YOLO[YOLOv8 Inference]
        YOLO --> JSON[JSON Detections]
        JSON --> ZEN2[Zenoh → ROS]
        ZEN2 --> MATCH2{Target Object?}
        MATCH2 -->|yes| S2[BT SUCCESS]
        MATCH2 -->|no| F2[BT FAILURE]
    end
```

### Repository Layout

```
turtlebot-maze/
├── tb_autonomy/              # ROS 2 autonomy package
│   ├── python/tb_behaviors/  #   Python behavior library (vision, navigation)
│   ├── scripts/              #   ROS nodes (autonomy_node, zenoh_detection_sub)
│   ├── launch/               #   Launch files (py + cpp demos)
│   ├── src/                  #   C++ behavior tree plugins
│   └── include/              #   C++ headers
├── tb_worlds/                # Gazebo worlds, maps, Nav2 config
├── detector/                 # Standalone PyTorch detector (no ROS)
│   ├── object_detector.py    #   Zenoh sub → YOLOv8 → Zenoh pub
│   └── requirements.txt      #   ultralytics, eclipse-zenoh, pycdr2
├── docker/                   # Dockerfiles + entrypoint
│   ├── Dockerfile.gpu        #   Multi-stage ROS 2 build (base/overlay/dev)
│   └── Dockerfile.torch.gpu  #   PyTorch container (CUDA + Ultralytics)
├── bt_xml/                   # Behavior tree XML definitions
├── docker-compose.yaml       # All service definitions
└── .env                      # Default environment variables
```

### Key Parameters

| Parameter | Default | Options | Description |
|---|---|---|---|
| `ROS_DISTRO` | `jazzy` | — | ROS 2 distribution |
| `TURTLEBOT_MODEL` | `3` | `3`, `4` | TurtleBot model |
| `BT_TYPE` | `queue` | `naive`, `queue` | Behavior tree variant |
| `ENABLE_VISION` | `true` | `true`, `false` | Enable vision behaviors |
| `TARGET_COLOR` | `blue` | `red`, `green`, `blue` | HSV detection target |
| `DETECTOR_TYPE` | `hsv` | `hsv`, `yolo` | Vision pipeline mode |
| `TARGET_OBJECT` | `cup` | Any COCO class | YOLO detection target |

---

## Setup

### Docker Setup (Recommended)
First, install Docker and Docker Compose using [the official install guide](https://docs.docker.com/engine/install/ubuntu/).

To run Docker containers with NVIDIA GPU support, you can optionally install the [NVIDIA Container Toolkit](https://github.com/NVIDIA/nvidia-docker).


First, clone this repository and go into the top-level folder:

```
git clone https://github.com/sea-bass/turtlebot3_behavior_demos.git
cd turtlebot3_behavior_demos
```

Build the Docker images.
This will take a while and requires approximately 5 GB of disk space.

```
docker compose build
```

### Local Setup

If you do not want to use Docker, you can directly clone this package to a Colcon workspace and build it provided you have the necessary dependencies.
As long as you can run the examples in the [TurtleBot3 manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/#overview), you should be in good shape.

First, make a Colcon workspace and clone this repo there:

```
mkdir -p turtlebot3_ws/src
cd turtlebot3_ws/src
git clone https://github.com/sea-bass/turtlebot3_behavior_demos.git
```

Clone the external dependencies:

```
sudo apt-get install python3-vcstool
vcs import < turtlebot3_behavior_demos/dependencies.repos
```

Set up any additional dependencies using rosdep:

```
sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
```

Ensure you have the necessary Python packages for these examples:

```
pip3 install matplotlib transforms3d
```

Then, build the workspace.

```
cd turtlebot3_ws
colcon build
```

NOTE: For best results, we recommend that you change your ROS Middleware (RMW) implementation to Cyclone DDS by following [these instructions](https://docs.ros.org/en/humble/Installation/DDS-Implementations/Working-with-Eclipse-CycloneDDS.html).

---

## Basic Usage

We use [Docker Compose](https://docs.docker.com/compose/) to automate building, as shown above, but also for various useful entry points into the Docker container once it has been built.
**All `docker compose` commands below should be run from your host machine, and not from inside the container**.

To enter a Terminal in the overlay container:

```
docker compose run overlay bash
```

Once inside the container, you can verify that display in Docker works by starting a basic Gazebo simulation included in the standard TurtleBot3 packages:

```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Alternatively, you can use the pre-existing `sim` service to do this in a single line:

```
docker compose up sim
```

If you want to develop using Docker, you can also launch a dev container using:

```
# Start the dev container
docker compose up dev

# Open as many interactive shells as you want to the container
docker compose exec -it dev bash
```

---

## Behavior Trees Demo

The robot navigates known locations searching for objects. Two vision modes are available:

- **HSV mode** (default): Color thresholding in the [HSV color space](https://en.wikipedia.org/wiki/HSL_and_HSV) — finds colored blocks (red, green, blue)
- **YOLO mode**: YOLOv8 deep learning — detects any [COCO class](https://docs.ultralytics.com/datasets/detect/coco/) object (cup, bottle, chair, etc.)

### Starting the Simulation

```bash
docker compose up demo-world
```

### HSV Mode (Default)

```bash
# Python behavior tree
docker compose up demo-behavior-py

# With custom parameters
TARGET_COLOR=green BT_TYPE=queue ENABLE_VISION=true docker compose up demo-behavior-py
```

### YOLO Mode (Deep Learning)

YOLO mode requires the Zenoh bridge and PyTorch detector services:

```bash
# Start Zenoh transport + detector
docker compose up zenoh-router zenoh-bridge detector

# In another terminal, start the behavior demo with YOLO mode
DETECTOR_TYPE=yolo TARGET_OBJECT=cup docker compose up demo-behavior-py
```

### Behavior Trees in Python

Uses [`py_trees`](https://py-trees.readthedocs.io/en/devel/) for behavior tree execution.

Customize via environment variables or the `.env` file:

```bash
TARGET_COLOR=green BT_TYPE=queue ENABLE_VISION=true docker compose up demo-behavior-py
```

Note that the behavior tree viewer ([`py_trees_ros_viewer`](https://github.com/splintered-reality/py_trees_ros_viewer)) should automatically discover the ROS node containing the behavior tree and visualize it.

After starting the commands above (plus doing some waiting and window rearranging), you should see the following.
The labeled images will appear once the robot reaches a target location.

![Example demo screenshot](./media/demo_screenshot_python.png)

### Behavior Trees in C++

Uses [`BehaviorTree.CPP`](https://www.behaviortree.dev/) with [Groot2](https://www.behaviortree.dev/groot/) for visualization.

```bash
docker compose up demo-behavior-cpp

# With custom parameters
TARGET_COLOR=green BT_TYPE=queue ENABLE_VISION=true docker compose up demo-behavior-cpp
```

YOLO mode works the same way — start the Zenoh + detector services, then:

```bash
DETECTOR_TYPE=yolo TARGET_OBJECT=cup docker compose up demo-behavior-cpp
```

> **Note:** Groot2 PRO is required for live behavior tree updates. Students can get a free license at [behaviortree.dev](https://www.behaviortree.dev/groot/).

![Example demo screenshot](./media/demo_screenshot_cpp.png)

---

## Zenoh + YOLOv8 Object Detection

The YOLO pipeline uses [Eclipse Zenoh](https://zenoh.io/) to decouple the ROS 2 simulation from the PyTorch inference container. This follows the [zenoh-python-lidar-plot](https://github.com/eclipse-zenoh/zenoh-demos/tree/main/ROS2/zenoh-python-lidar-plot) pattern.

### Data Flow

```mermaid
sequenceDiagram
    participant G as Gazebo Camera
    participant B as zenoh-bridge-ros2dds
    participant D as object_detector.py
    participant S as zenoh_detection_sub
    participant BT as Behavior Tree

    G->>B: sensor_msgs/Image (DDS)
    B->>D: rt/camera/image_raw (CDR via Zenoh)
    D->>D: pycdr2 deserialize → YOLOv8 inference
    D->>B: tb/detections (JSON via Zenoh)
    B->>S: Subscribe tb/detections
    S->>BT: Cached detections
    BT->>BT: LookForObject checks for target_object
```

### Zenoh Key Expressions

| Key | Direction | Format | Description |
|---|---|---|---|
| `rt/camera/image_raw` | ROS → Detector | CDR (`sensor_msgs/Image`) | Camera frames (auto-bridged) |
| `tb/detections` | Detector → ROS | JSON array | Detection results |

### Detection JSON Format

```json
[
  {"class": "cup", "confidence": 0.87, "bbox": [120, 80, 250, 310]},
  {"class": "bottle", "confidence": 0.72, "bbox": [300, 100, 380, 350]}
]
```

### Detector CLI Options

```bash
python object_detector.py \
  --model yolov8n.pt \
  --confidence 0.5 \
  --max-fps 10 \
  --image-key "rt/camera/image_raw" \
  --detection-key "tb/detections"
```
