# stella_vslam Visual SLAM Integration — Design

> **Beads issue:** turtlebot-maze-m5e

## Goal

Integrate stella_vslam Visual SLAM into the turtlebot-maze project as a standalone demo, using the enhanced maze world's textured walls for visual feature extraction.

## Background

[stella_vslam](https://github.com/stella-cv/stella_vslam) is a Visual SLAM framework (fork of OpenVSLAM, 1151 stars, actively maintained). PR #3 included an install script (`docker/install_stella_vslam.sh`) and referenced a [Jazzy-compatible ROS 2 wrapper](https://github.com/oscarpoudel/stella_vslam_ros2_Jazzy) by Oscar Poudel. Oscar's fork is 1 commit ahead of the upstream `stella-cv/stella_vslam_ros` `ros2` branch — two small fixes: `cv_bridge.h` → `cv_bridge.hpp` and removal of a deprecated `ament_environment_hooks` call.

## Architecture

stella_vslam runs in a **separate Docker container** (`Dockerfile.slam`) with zero ROS dependencies, following the same Zenoh transport pattern as the YOLO detector container.

```
Gazebo (RGBD camera)
    → DDS
    → zenoh-bridge-ros2dds
    → Zenoh
    → slam_bridge.py (in slam container)
    → stella_vslam C++ library (via Python bindings or subprocess)
    → Zenoh
    → zenoh-bridge-ros2dds
    → /slam/pose (ROS 2 topic)
```

## Phases

### Phase 1: Standalone Demo (this issue)

stella_vslam builds a visual map of the enhanced maze. Viewable via socket viewer (browser-based). No Nav2 integration.

### Phase 2: Nav2 Fusion (follow-up issue)

Fuse stella_vslam pose estimates with Nav2 localization (AMCL) for improved robot positioning.

## Components

| Component | Location | Purpose |
|-----------|----------|---------|
| `Dockerfile.slam` | `docker/` | Multi-stage build: iridescence + stella_vslam core + Python Zenoh wrapper |
| `slam_bridge.py` | `slam/` | Zenoh subscriber for camera images → CDR deserialize → feed stella_vslam → publish pose via Zenoh |
| `turtlebot_realsense.yaml` | `slam/config/` | stella_vslam camera config matching TurtleBot's simulated RealSense intrinsics |
| `orb_vocab.fbow` | Downloaded at build time | ORB feature vocabulary (FBoW format) |
| `demo-slam` | `docker-compose.yaml` | Docker Compose service for the SLAM container |

## Data Flow

| Zenoh Key | Direction | Format | Description |
|-----------|-----------|--------|-------------|
| `rt/intel_realsense_r200_depth/image` | Gazebo → SLAM | CDR (`sensor_msgs/Image`) | RGB frames from simulated RealSense |
| `rt/intel_realsense_r200_depth/depth` | Gazebo → SLAM | CDR (`sensor_msgs/Image`) | Depth frames |
| `tb/slam/pose` | SLAM → ROS | JSON | Camera pose (position + quaternion) |
| `tb/slam/status` | SLAM → ROS | JSON | Tracking status (tracking/lost/initializing) |

## Key Decisions

1. **Use upstream `stella-cv/stella_vslam_ros` `ros2` branch** with the two Jazzy patches applied in the Dockerfile, rather than depending on Oscar's personal fork.

2. **Python Zenoh wrapper** (`slam_bridge.py`) rather than C++ Zenoh integration — matches the detector pattern, faster to iterate, uses pycdr2 for CDR deserialization.

3. **Socket viewer** for Phase 1 rather than iridescence GUI — works headless in Docker, accessible from host browser at `localhost:3001`.

4. **Camera config** extracted from TurtleBot's SDF — the simulated RealSense has known intrinsics hardcoded in the stella_vslam config YAML.

## Docker Compose Service

```yaml
demo-slam:
  build:
    context: .
    dockerfile: docker/Dockerfile.slam
  network_mode: host
  environment:
    - ZENOH_ROUTER=tcp/localhost:7447
  command: >
    python3 slam_bridge.py
    --image-key "rt/intel_realsense_r200_depth/image"
    --depth-key "rt/intel_realsense_r200_depth/depth"
    --config /slam/config/turtlebot_realsense.yaml
    --vocab /slam/orb_vocab.fbow
```

## Launch Sequence

```bash
# Terminal 1: Enhanced world (textured walls for visual features)
docker compose up demo-world-enhanced

# Terminal 2: Zenoh transport
docker compose up zenoh-router zenoh-bridge

# Terminal 3: Visual SLAM
docker compose up demo-slam
```

## Testing

1. stella_vslam initializes and transitions from "initializing" to "tracking"
2. Pose estimates appear on `tb/slam/pose` Zenoh key
3. Socket viewer shows reconstructed map with ORB features on textured walls
4. Compare: enhanced world (textured) vs original world (featureless) — expect SLAM failure or poor tracking on featureless walls

## Files Changed

| File | Action |
|------|--------|
| `docker/Dockerfile.slam` | NEW |
| `slam/slam_bridge.py` | NEW |
| `slam/config/turtlebot_realsense.yaml` | NEW |
| `docker-compose.yaml` | MODIFY — add `demo-slam` service |
| `README.md` | MODIFY — add Visual SLAM section |

## Not in Scope (Phase 1)

- Nav2 localization fusion
- Map saving/loading
- ArUco marker-based scale correction
- Multi-session mapping
