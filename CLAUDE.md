# CLAUDE.md — Project Guide for Claude Code

## Project Overview

TurtleBot3 Behavior Demos — a ROS 2 (Jazzy) robotics project demonstrating autonomous navigation using behavior trees. A simulated TurtleBot navigates a house environment searching for colored blocks using vision and Nav2-based navigation.

## Repository Layout

- `tb_autonomy/` — ROS 2 package: autonomy behaviors (C++ via BehaviorTree.CPP, Python via py_trees)
- `tb_worlds/` — ROS 2 package: Gazebo simulation worlds, maps, Nav2 config
- `docker/` — Dockerfiles (CPU + GPU) and entrypoint scripts
- `bt_xml/` — Behavior tree XML definitions (naive and queue variants)
- `.github/workflows/` — CI: Docker build (`docker.yml`) and pre-commit formatting (`format.yml`)

## Languages

- **C++**: Core autonomy node and behavior tree plugins (`tb_autonomy/src/`, `tb_autonomy/include/`)
- **Python**: Autonomy node, behavior library, launch files (`tb_autonomy/python/`, `tb_autonomy/scripts/`, `*/launch/`)
- **CMake**: Build system (`CMakeLists.txt` files, ament_cmake)
- **XML/SDF/Xacro**: Behavior trees and simulation models

## Build System

ROS 2 colcon workspace. Build with:

```bash
colcon build --symlink-install
source install/setup.bash
```

## Linting & Formatting

Pre-commit hooks are configured (`.pre-commit-config.yaml`):

```bash
pre-commit run -a          # Run all hooks
pre-commit install         # Install as git hook
```

Hooks include:
- `black` — Python formatter
- `clang-format` v18 — C/C++ formatter
- `codespell` — Spell checker (ignore list: `atleast,inout,ether`)
- `yamllint` — YAML linter
- `markdown-link-check` — Markdown link validator
- Standard pre-commit checks (AST, YAML, merge conflicts, etc.)

## Docker

```bash
docker compose up demo-world         # Launch simulation
docker compose up demo-behavior-py   # Python behavior demo
docker compose up demo-behavior-cpp  # C++ behavior demo
```

## Issue Tracking

Use `bd` (beads) for task and issue tracking. See [beads documentation](https://github.com/steveyegge/beads).

## Key Configuration

- ROS distro: `jazzy` (set in `.env`)
- TurtleBot model: `3`
- Behavior tree type: `queue` (or `naive`)
- Vision target color: `blue` (or `red`, `green`)
