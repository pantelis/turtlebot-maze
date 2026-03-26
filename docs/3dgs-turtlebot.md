# 3D Gaussian Splatting for TurtleBot3 Navigation and Semantic Mapping

## Overview

Research findings on integrating 3D Gaussian Splatting (3DGS) with TurtleBot3 Waffle Pi, ROS 2 Jazzy, and Intel RealSense D435i RGB-D camera for real-time 3D reconstruction, semantic mapping, and visual re-localization.

## GS-SLAM Systems (RGB-D capable)

| System | FPS | VRAM | Input | Venue | Notes |
|--------|-----|------|-------|-------|-------|
| **RTG-SLAM** | 17.9 | 8.8 GB | RGB-D | SIGGRAPH 2024 | Best real-time RGB-D option, tested on 100 m² rooms, single opaque Gaussian per surface patch |
| **Photo-SLAM** | ~30 | N/A | Mono/Stereo/RGB-D | CVPR 2024 | Only system demoed on Jetson AGX Orin; uses ORB-SLAM3 tracking |
| **SplaTAM** | 0.3-2.25 | 7-10 GB | RGB-D | CVPR 2024 | Best reconstruction quality (2x better NVS than prior), offline-pace; rendering is 400 FPS after training |
| **GS-SLAM** | Near real-time | N/A | RGB-D | CVPR 2024 | Coarse-to-fine tracking, 100x faster rendering than NeRF SLAM |
| **MonoGS** | ~10 | 12-16 GB | Mono (RGB-D branch) | CVPR 2024 | Primarily monocular, has RGB-D support |
| **MonoGS++** | 5.57x faster than MonoGS | N/A | Mono | BMVC 2024 | Speed-optimized variant |

**Recommendation:** RTG-SLAM for real-time operation. SplaTAM for highest-quality offline reconstruction.

## Semantic / Language Gaussian Splatting

| Method | Speed | Approach | Venue |
|--------|-------|----------|-------|
| **LangSplat** | 199x faster than LERF | Scene-wise language autoencoder; CLIP features in latent space per Gaussian | CVPR 2024 Highlight |
| **LangSplatV2** | 450+ FPS rendering | Global 3D codebook for high-dim features | NeurIPS 2025 |
| **LEGS** (Berkeley) | Incremental, 3.5x faster training than LERF | Built on Nerfstudio Splatfacto; runs on mobile robot; 66% open-vocab accuracy | IROS 2024 |
| **LEGaussians** | Real-time rendering | Quantized CLIP + DINO features as discrete indices per Gaussian | CVPR 2024 |
| **Feature 3DGS** | Real-time rendering | Learns arbitrary feature fields (CLIP, SAM, etc.) alongside color | CVPR 2024 |
| **HAMMER** | 25x faster than MAGiC-SLAM | Multi-robot collaborative semantic GS with ROS communication | RAL 2025 |

**Recommendation:** LEGS for incremental semantic splatting on mobile robot. LangSplat for offline post-processing with highest semantic quality.

## ROS 2 Packages

| Package | ROS 2 Distro | Description | Repository |
|---------|-------------|-------------|------------|
| **ROSplat** | **Jazzy** | Online GS visualizer; custom `GaussianArray` messages; GPU-accelerated rendering | [github.com/shadygm/ROSplat](https://github.com/shadygm/ROSplat) |
| **RGBD-3DGS-SLAM** | **Humble** | MonoGS-based SLAM; subscribes to RGB + depth topics; publishes PointCloud2 + Path | [github.com/jagennath-hari/RGBD-3DGS-SLAM](https://github.com/jagennath-hari/RGBD-3DGS-SLAM) |
| **GS-SDF** | ROS 2 | LiDAR-augmented Gaussian Splatting + Neural SDF | [github.com/hku-mars/GS-SDF](https://github.com/hku-mars/GS-SDF) |
| **MM3DGS-SLAM** | ROS 2 | Multi-modal 3DGS SLAM (vision + depth + IMU) | [github.com/VITA-Group/MM3DGS-SLAM](https://github.com/VITA-Group/MM3DGS-SLAM) |

## Compute Requirements

### Training (offline, room-scale scene from collected data)

| Task | VRAM | GPU | Time |
|------|------|-----|------|
| Original 3DGS, 30K iterations | 24 GB | RTX 3090/4090 | 30-40 min |
| Original 3DGS, 7K iterations (preview) | 12-16 GB | RTX 3060+ | ~10 min |
| gsplat (optimized), same scene | 6 GB (4x less) | RTX 3090/4090 | 15% faster |

### Real-time Rendering / Inference

| Task | FPS | GPU |
|------|-----|-----|
| Original 3DGS rendering | 100-200+ | RTX 4090 (1080p) |
| LangSplatV2 (with language features) | 450+ | RTX 4090 |
| SplaTAM map rendering (after training) | 400 | RTX 4090 |

### Real-time SLAM

| System | FPS | VRAM |
|--------|-----|------|
| RTG-SLAM | 17.9 | 8.8 GB |
| Photo-SLAM | ~30 | N/A |
| SplaTAM | 0.3-2.25 | 7-10 GB |

### Re-localization

| System | Speed | Method |
|--------|-------|--------|
| Splat-Loc (Stanford) | ~25 Hz | Render-and-compare against pre-built GS map |
| 3DGS-ReLoc | N/A | Dedicated re-localization using 3DGS map |
| GS-Loc | N/A | Vision foundation model-driven (RAL 2025) |

**Bottom line: RTX 3090 or 4090 (24 GB) is sufficient for all tasks. No datacenter GPUs needed.**

## Frameworks

| Framework | Stars | Key Strengths | Status |
|-----------|-------|---------------|--------|
| **gsplat** | 4.7K | 4x less memory, 15% faster; PyTorch API; NVIDIA-backed | Most actively maintained |
| **Nerfstudio** | ~10K | Full pipeline; Splatfacto method; LEGS built on this | Very active |
| **Original 3DGS** (INRIA) | 21.1K | Reference implementation; most cited | Maintained, slower cadence |
| **OpenSplat** | ~1K | CPU/GPU; cross-platform | Active |

**Recommendation:** gsplat + Nerfstudio as the foundation.

## Feasibility for TurtleBot3 + D435i (640x480)

### Offline Reconstruction
Highly feasible. Collect RGB-D frames during Nav2 exploration, save with poses from ORB-SLAM or AMCL. Train on workstation with gsplat/Nerfstudio. 10-30 min for a single room, 8-16 GB VRAM.

### Online/Incremental Splatting
Feasible with server offload. TurtleBot streams RGB-D + poses via Zenoh to workstation (same pattern as current `slam_bridge.py`). RTG-SLAM at 17.9 FPS / 8.8 GB is the best option. Not viable on-robot without Jetson AGX Orin.

### Splat-Based Re-Localization
Active research area. Splat-Loc achieves ~25 Hz pose estimation by rendering expected views from candidate poses and comparing with actual camera feed. Alternative to current CLIP KNN + AGE graph traversal approach.

## Proposed Architecture

```
TurtleBot3 (D435i RGB-D + LiDAR)
    |
    | Zenoh (camera/color + camera/depth + odom)
    v
Workstation (RTX 3090/4090)
    |
    +---> RTG-SLAM (real-time 3D reconstruction, 17.9 FPS)
    |       |
    |       +---> Gaussian Splat Map (in-memory)
    |
    +---> LEGS / LangSplat (semantic features, CLIP per-Gaussian)
    |       |
    |       +---> Language-Embedded GS Map
    |
    +---> Splat-Loc (re-localization, ~25 Hz)
    |       |
    |       +---> Pose hypotheses
    |
    +---> ROSplat (Jazzy visualization)
    |
    +---> Current pipeline (detector → pgvector → AGE graph)
```

## Key References

- RTG-SLAM: [arxiv.org/abs/2404.19706](https://arxiv.org/abs/2404.19706)
- SplaTAM: [spla-tam.github.io](https://spla-tam.github.io/)
- LangSplat: [langsplat.github.io](https://langsplat.github.io/)
- LEGS: [berkeleyautomation.github.io/LEGS](https://berkeleyautomation.github.io/LEGS/)
- HAMMER: [arxiv.org/abs/2501.14147](https://arxiv.org/abs/2501.14147)
- gsplat: [github.com/nerfstudio-project/gsplat](https://github.com/nerfstudio-project/gsplat)
- ROSplat: [github.com/shadygm/ROSplat](https://github.com/shadygm/ROSplat)
- Awesome 3DGS SLAM: [github.com/KwanWaiPang/Awesome-3DGS-SLAM](https://github.com/KwanWaiPang/Awesome-3DGS-SLAM)
- 3DGS in Robotics survey: [arxiv.org/abs/2410.12262](https://arxiv.org/abs/2410.12262)
