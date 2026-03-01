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

## Documentation Standards

All architecture and data-flow diagrams **must use Mermaid** — no ASCII art or image files.
This applies to README.md and all other markdown in the repo.

- Use `graph LR` / `graph TD` for flows, `sequenceDiagram` for message exchanges
- Apply `classDef` to **every** node — never leave siblings unstyled
- Dark-mode-safe fills with `color:#fff`:
  - Neutral: `fill:#37474f,stroke:#546e7a`
  - Blue: `fill:#0277bd,stroke:#01579b`
  - Green: `fill:#2e7d32,stroke:#1b5e20`
  - Orange: `fill:#e65100,stroke:#bf360c`
- Use `rgba(r,g,b,0.2–0.4)` for `rect` in sequence diagrams — never opaque light colors
- Subgraph labels: plain text only, no special characters or newlines

## Git Workflow

The `main` branch is protected — direct pushes are blocked for everyone including admins.
All changes must go through a pull request.

```bash
# 1. Create a feature branch
git checkout -b feat/my-change

# 2. Commit work normally
git add <files>
git commit -m "feat: description"

# 3. Push and open a PR
git push -u origin feat/my-change
gh pr create --title "Short title" --body "$(cat <<'EOF'
## Summary
- bullet points

## Test plan
- [ ] tests pass

🤖 Generated with [Claude Code](https://claude.com/claude-code)
EOF
)"

# 4. Enable auto-merge (merges immediately — no reviewer required)
gh pr merge --auto --squash
```

Auto-merge is enabled at the repo level: once any required CI checks pass the PR merges automatically.
Use `--squash` (default), `--merge`, or `--rebase` depending on the change.

## Issue Tracking

Use `bd` (beads) for task and issue tracking. See [beads documentation](https://github.com/steveyegge/beads).

**Before closing any issue**, always write a summary using `bd update <id> --notes "..."` that includes:
- What was done (key changes made, files modified)
- Root causes found (for bugs)
- Lessons learned (gotchas, non-obvious behaviour, useful debugging insights)
- Verification performed (tests run, output observed)

This makes closed issues a searchable knowledge base for future debugging sessions.

## Key Configuration

- ROS distro: `jazzy` (set in `.env`)
- TurtleBot model: `3`
- Behavior tree type: `queue` (or `naive`)
- Vision target color: `blue` (or `red`, `green`)
