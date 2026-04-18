# 3D Gaussian Splatting Capture Pipeline — Design Spec

## Overview

A dataset-capture pipeline that records RGB-D frames and poses from a simulated TurtleBot3 (D435i camera, Gazebo house world) into Nerfstudio format for offline 3D Gaussian Splatting reconstruction. Designed as a student assignment: the capture pipeline is provided as scaffolding; training, evaluation, and exploration strategy are experimentation space.

This is simulation-only. There is no physical robot or physical RealSense camera. All data comes from Gazebo sensor plugins via ROS 2 Jazzy / Zenoh.

## Motivation

Traditional robotics mapping produces sparse point clouds (ORB-SLAM) or 2D occupancy grids (SLAM Toolbox). Neither captures visual appearance. A robot with an occupancy grid of a kitchen knows where the walls are but cannot answer "what does the kitchen look like from the doorway?" or "is this the same kitchen I saw before?"

Gaussian Splatting produces a photorealistic, renderable 3D representation. This enables:

- **Visual re-localization** — render what the robot expects to see from a candidate pose, compare with what it actually sees. If they match, the robot has found its location. Fundamentally different from feature-matching (CLIP KNN) and potentially more robust in visually rich environments.
- **Scene understanding** — with language-embedded splats (future work), the 3D map becomes queryable: "where is the couch?" returns a rendered view and a pose.
- **Simulation-to-real transfer** — a splat trained in Gazebo produces photorealistic renders that can train vision models, narrowing the sim-real gap.
- **Navigation planning** — dense 3D geometry from splats generates better obstacle representations than sparse point clouds.

## Architecture

```
Gazebo House World (D435i sim)
    |
    | ros_gz_bridge → zenoh-bridge-ros2dds
    v
gs_capture.py (Zenoh subscriber)
    |
    | Keyframe gating (distance + angle)
    | ROS → Nerfstudio coordinate transform
    | Depth m → mm conversion
    v
data/captures/house_run_01/
    ├── transforms.json      (intrinsics + per-frame extrinsics)
    ├── images/
    │   ├── frame_0000.png   (RGB)
    │   ├── frame_0001.png
    │   └── ...
    └── depth/
        ├── frame_0000.png   (16-bit depth in mm)
        ├── frame_0001.png
        └── ...
    |
    | Student runs: ns-train splatfacto --data <path>
    v
Nerfstudio (Splatfacto)
    |
    | Student runs: ns-viewer --load-config <path>
    v
Trained Gaussian Splat Model + Interactive Viewer
```

## Capture Script: `gs_capture.py`

### Data sources (Zenoh keys)

| Zenoh Key | ROS Topic | Data | Frame |
|---|---|---|---|
| `camera/color/image_raw` | `/camera/color/image_raw` | RGB image (sensor_msgs/Image, CDR) | camera_optical_frame |
| `camera/depth/image_rect_raw` | `/camera/depth/image_rect_raw` | Depth image (sensor_msgs/Image, float32 meters, CDR) | camera_depth_frame |
| `odom` | `/odom` | Robot odometry (nav_msgs/Odometry, CDR) | odom → base_link |

The dataset's world frame is `odom` (NOT `map`). In Gazebo simulation, odom is ground-truth with zero drift, so this is equivalent to world-frame poses. Using odom avoids dependency on AMCL or SLAM Toolbox localization.

### Frame synchronization and capture trigger

The capture is **RGB-triggered**: each arriving RGB image is the event that may produce a keyframe. When an RGB frame arrives:

1. Check if the latest odom pose passes the keyframe gate (distance + angle threshold since last keyframe)
2. If yes, pair the RGB with the **most recent depth frame** and the **most recent odom pose**
3. Check staleness: if the depth timestamp is more than 200 ms older than the RGB timestamp, or odom is more than 200 ms stale, **drop the frame** (log a warning, do not write to disk)
4. If all three are fresh, write the keyframe

State management:
- `latest_rgb`: updated on every RGB callback (image + timestamp)
- `latest_depth`: updated on every depth callback (image + timestamp)
- `latest_odom`: updated on every odom callback (pose + timestamp)
- Keyframe gating runs inside the RGB callback only

This nearest-neighbor pairing is acceptable because Gazebo publishes RGB, depth, and odom synchronously at the same rate (5 Hz for cameras, higher for odom). The 200 ms staleness window catches transport delays without requiring exact timestamp matching.

### Keyframe gating

Same logic as `object_detector.py`: a frame is a keyframe if the robot has moved more than `keyframe_dist` meters or rotated more than `keyframe_angle` degrees since the last keyframe. Default thresholds: 0.3 m, 10 degrees. Students can tune these via CLI arguments.

Tighter thresholds produce denser captures (more frames, better reconstruction, slower training). Looser thresholds produce sparser captures (fewer frames, faster training, potential gaps). This tradeoff is part of the experimentation.

### Coordinate convention and pose transform

Nerfstudio's `transform_matrix` is **camera-to-world** (i.e. it places the camera in the world). It uses the OpenGL camera convention: x-right, y-up, z-backward (camera looks along -z).

ROS uses x-forward, y-left, z-up (REP 103) for the robot body, and the standard optical frame convention (x-right, y-down, z-forward) for cameras.

#### Source of truth for each transform

| Transform | Source | Type |
|---|---|---|
| `odom → base_link` | `/odom` topic (nav_msgs/Odometry) | Dynamic, per-frame |
| `base_link → camera_link` | URDF (`gz_waffle.sdf.xacro` line 467-470): `<pose>0.064 -0.065 0.094 0 0 0</pose>` | Static, translation only |
| ROS optical → Nerfstudio | Fixed rotation matrix (see below) | Static, constant |

Note: we use `odom → base_link` (from `/odom`), NOT `map → base_link`. The odom frame is the dataset's world frame. This avoids dependency on AMCL/SLAM Toolbox localization. For Gazebo simulation, odom is ground-truth (no drift).

#### Transform chain and multiplication order

```
T_nerfstudio = T_odom_baselink @ T_baselink_camera @ T_ros_optical_to_nerfstudio
```

Where:

**Step 1:** `T_odom_baselink` — 4x4 pose from `/odom` message:
```python
# From nav_msgs/Odometry: position (x,y,z) + quaternion (x,y,z,w)
T_odom_baselink = quaternion_matrix(qx, qy, qz, qw)
T_odom_baselink[:3, 3] = [px, py, pz]
```

**Step 2:** `T_baselink_camera` — static transform from URDF, translation only (camera is level with base_link, no rotation):
```python
T_baselink_camera = np.eye(4)
T_baselink_camera[:3, 3] = [0.064, -0.065, 0.094]  # from URDF camera_joint pose
```

**Step 3:** `T_ros_optical_to_nerfstudio` — converts from ROS camera optical frame (x-right, y-down, z-forward) to Nerfstudio/OpenGL (x-right, y-up, z-backward). This is a 180° rotation around the x-axis:
```python
T_ros_optical_to_nerfstudio = np.array([
    [1,  0,  0, 0],
    [0, -1,  0, 0],
    [0,  0, -1, 0],
    [0,  0,  0, 1],
], dtype=np.float64)
```

#### Worked numeric example

Robot at odom pose (1.0, 2.0, 0.0) with yaw=0 (facing x-forward), identity rotation:

```
T_odom_baselink = [[1, 0, 0, 1.0],
                    [0, 1, 0, 2.0],
                    [0, 0, 1, 0.0],
                    [0, 0, 0, 1.0]]

T_baselink_camera = [[1, 0, 0, 0.064],
                      [0, 1, 0, -0.065],
                      [0, 0, 1, 0.094],
                      [0, 0, 0, 1.0]]

T_ros_optical_to_nerfstudio = [[1,  0,  0, 0],
                                [0, -1,  0, 0],
                                [0,  0, -1, 0],
                                [0,  0,  0, 1]]

Result = T_odom_baselink @ T_baselink_camera @ T_ros_optical_to_nerfstudio
       = [[1,  0,  0, 1.064],
          [0, -1,  0, 1.935],    # y flipped
          [0,  0, -1, 0.094],    # z flipped (now camera looks along -z)
          [0,  0,  0, 1.0]]
```

This matrix goes into `transforms.json` as `transform_matrix`. Nerfstudio interprets it as camera-to-world in OpenGL convention.

### Camera intrinsics

From the simulated D435i configuration:

| Parameter | Value |
|---|---|
| Width | 320 |
| Height | 240 |
| fx | 277.13 |
| fy | 277.13 |
| cx | 160.0 |
| cy | 120.0 |

Written once in `transforms.json` as `fl_x`, `fl_y`, `cx`, `cy`, `w`, `h`.

### Depth handling

Gazebo produces float32 depth in meters. Nerfstudio expects depth as 16-bit PNG in millimeters for depth supervision.

```python
depth_mm = (depth_m * 1000.0).clip(0, 65535).astype(np.uint16)
cv2.imwrite(path, depth_mm)
```

### `transforms.json` format

```json
{
  "fl_x": 277.13,
  "fl_y": 277.13,
  "cx": 160.0,
  "cy": 120.0,
  "w": 320,
  "h": 240,
  "camera_model": "OPENCV",
  "frames": [
    {
      "file_path": "images/frame_0000.png",
      "depth_file_path": "depth/frame_0000.png",
      "transform_matrix": [
        [r00, r01, r02, tx],
        [r10, r11, r12, ty],
        [r20, r21, r22, tz],
        [0.0, 0.0, 0.0, 1.0]
      ]
    }
  ]
}
```

### CLI arguments

```
--connect          Zenoh endpoint (default: tcp/localhost:7447)
--output           Output directory (default: /data/captures/house_run)
--image-key        RGB topic (default: camera/color/image_raw)
--depth-key        Depth topic (default: camera/depth/image_rect_raw)
--odom-key         Odometry topic (default: odom)
--keyframe-dist    Distance threshold in meters (default: 0.3)
--keyframe-angle   Angle threshold in degrees (default: 10)
--max-frames       Stop after N frames (default: unlimited)
```

### Missing odom handling

If no odom has been received within 10 seconds of startup, the capture script logs a warning every 30 seconds:

```
WARNING: No odom received. Capture is paused — frames require valid poses.
```

**Capture pauses until odom resumes.** Frames without valid poses are never written to disk. A `transforms.json` entry with an invalid pose would produce garbage reconstruction — silently continuing is worse than waiting.

RGB and depth frames that arrive while odom is missing are discarded. When odom resumes, the staleness check (200 ms window) ensures the first captured frame has a fresh pose.

This differs from `object_detector.py` which falls back to time-based capture without poses. The detector can produce useful detections without odom (class labels and embeddings are still valid). The GS capture cannot — every frame requires a precise camera-to-world transform.

## Docker Services

### `gs-capture`

```yaml
gs-capture:
  build:
    context: .
    dockerfile: docker/Dockerfile.torch.gpu
  network_mode: host
  volumes:
    - ./detector:/app
    - ./data/captures:/data/captures
  working_dir: /app
  command: >
    python gs_capture.py
    --connect tcp/localhost:7447
    --output /data/captures/house_run
    --keyframe-dist 0.3
    --keyframe-angle 10
  depends_on:
    - zenoh-router
```

### `nerfstudio`

```yaml
nerfstudio:
  build:
    context: .
    dockerfile: docker/Dockerfile.nerfstudio
  network_mode: host
  ipc: host
  deploy:
    resources:
      reservations:
        devices:
          - capabilities: [gpu]
  volumes:
    - ./data/captures:/data/captures
    - ./data/ns_outputs:/data/ns_outputs
  working_dir: /data
  command: sleep infinity
```

Students interact via `docker exec -it <container> bash`, then run Nerfstudio commands manually. The `sleep infinity` keeps the container alive as a development environment.

### `Dockerfile.nerfstudio`

Based on NVIDIA CUDA base image with:
- Python 3.11
- PyTorch 2.x + CUDA
- gsplat
- nerfstudio (pip install)
- OpenCV, numpy

## File Structure

```
detector/
  gs_capture.py              # Zenoh capture script

docker/
  Dockerfile.nerfstudio      # Nerfstudio training environment

data/                        # Git-ignored
  captures/                  # Capture output directories
  ns_outputs/                # Nerfstudio training outputs
```

## Assignment Structure

### What we provide (scaffolding)

- `gs_capture.py` — complete capture script
- `Dockerfile.nerfstudio` — training environment
- `docker-compose.yaml` entries for both services
- Documentation: what Gaussian Splatting is, why it matters for robotics, how to run capture and training
- Example commands:
  ```bash
  # 1. Launch house world + capture
  docker compose up -d demo-world-house zenoh-router zenoh-bridge gs-capture

  # 2. Drive robot (teleop or Nav2 waypoints)
  # ... student explores the house ...

  # 3. Stop capture
  docker compose stop gs-capture

  # 4. Train (inside nerfstudio container)
  docker exec -it turtlebot-maze-nerfstudio-1 bash
  ns-train splatfacto \
    --data /data/captures/house_run \
    --output-dir /data/ns_outputs

  # 5. View (inside same container)
  ns-viewer --load-config /data/ns_outputs/house_run/splatfacto/<timestamp>/config.yml
  ```

  All Nerfstudio outputs go to `/data/ns_outputs` (mounted from `./data/ns_outputs` on the host). Students should always use `--output-dir /data/ns_outputs` to keep artifacts in the mounted volume.

### What students experiment with (open space)

| Experiment | What to vary | What to measure |
|---|---|---|
| Capture density | `--keyframe-dist` (0.1 to 1.0 m), `--keyframe-angle` (5 to 30 deg) | PSNR/SSIM on held-out views, training time, number of Gaussians |
| Exploration strategy | Teleop vs Nav2 waypoints vs autonomous exploration | Coverage completeness, reconstruction holes |
| Training config | `splatfacto` vs `splatfacto-big`, 7K vs 30K iterations | Quality metrics, training time |
| Depth supervision | With vs without `depth_file_path` in transforms.json | Geometric accuracy, floaters |
| Partial coverage | Single room vs full house | Can you navigate using a partial splat? |

### Stretch goals (optional, not graded)

- Capture a real scene with a personal phone (iPhone via Record3D/Polycam, Android via ARCore). Train a splat and compare with the Gazebo reconstruction.
- Render novel views from 5 random poses in the trained splat. Compare with actual Gazebo camera images from those poses. Compute PSNR/SSIM as a basic re-localization quality metric.
- Try `nerfacto` (NeRF) instead of `splatfacto` (Gaussian Splat). Compare training time, rendering speed, and visual quality.

### Deliverables

Students submit:
1. Capture configuration used (thresholds, exploration strategy)
2. Training configuration (method, iterations)
3. Quality metrics (PSNR, SSIM, LPIPS from Nerfstudio eval)
4. Screenshots or video of the trained splat viewer
5. Written analysis: what capture/training choices affected quality and why

## Phone Capture Path (stretch goal)

For students who want to capture real-world scenes:

**iPhone:** Use [Record3D](https://record3d.app/) or [Polycam](https://poly.cam/) to capture RGB-D sequences. Export as Record3D `.r3d` file or Polycam export. Process with:
```bash
ns-process-data record3d --data <path.r3d> --output-dir <output>
# or
ns-process-data polycam --data <polycam_export> --output-dir <output>
```

**Android:** Use ARCore-based capture apps. Export images, then process with COLMAP:
```bash
ns-process-data images --data <image_folder> --output-dir <output>
```

Both paths produce the same `transforms.json` format that `ns-train splatfacto` consumes. No custom code needed.

## Testing Strategy

- Unit test for coordinate transform: known ROS pose → expected Nerfstudio matrix
- Unit test for depth conversion: float32 meters → uint16 millimeters
- Integration test: run gs_capture against a short simulated run, verify `transforms.json` is valid and images are saved
- Validation: train a splat from captured data, visually inspect for correct geometry (no mirroring, no inverted depth)

## Out of Scope

- Real-time GS-SLAM streaming (future spec: RTG-SLAM integration)
- Semantic / language-embedded splats (future spec: LEGS/LangSplat)
- Splat-based re-localization pipeline (future spec: Splat-Loc)
- Splat-based navigation planning (future spec: Splat-Nav)
- Physical robot or physical D435i camera
