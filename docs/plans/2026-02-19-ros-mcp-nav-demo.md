# ros-mcp-server Navigation Demo Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Create a `/navigate` slash command that lets users command the TurtleBot to navigate to a named location via ros-mcp-server tools.

**Architecture:** A single Markdown prompt file (`.claude/commands/navigate.md`) instructs Claude Code to connect to the robot via rosbridge, verify Nav2, ask the user for a location, send a navigation goal, and confirm arrival. A new README section documents setup and usage.

**Tech Stack:** Claude Code slash commands, ros-mcp-server MCP tools, Nav2 NavigateToPose action, rosbridge WebSocket

---

### Task 1: Create the `/navigate` slash command

**Files:**
- Create: `.claude/commands/navigate.md`

**Step 1: Create the commands directory**

```bash
mkdir -p .claude/commands
```

**Step 2: Write the slash command prompt file**

Create `.claude/commands/navigate.md` with this content:

```markdown
# Navigate Robot to Location

You are helping the user navigate the TurtleBot to a named location in the sim_house world using ros-mcp-server.

## Prerequisites

Before starting, remind the user that these services must be running:
- `docker compose up demo-world` (Gazebo simulation + Nav2)
- `docker compose up rosbridge` (WebSocket bridge on port 9090)

## Step 1: Connect to the robot

Use `connect_to_robot` with ip="127.0.0.1" and port=9090.

If connection fails, tell the user to check that the rosbridge service is running:
```
docker compose up rosbridge
```

## Step 2: Verify Nav2 is ready

Use `get_actions` to list available action servers. Confirm that `/navigate_to_pose` is present.

If `/navigate_to_pose` is not listed, tell the user that Nav2 is still initializing and to wait 10-20 seconds, then retry.

## Step 3: Get current robot pose

Use `subscribe_once` on topic `/amcl_pose` to read the robot's current position. Report the x, y coordinates to the user.

## Step 4: Ask user for destination

Present these 4 named locations and ask the user which one to navigate to:

| Location | x | y | Description |
|---|---|---|---|
| location1 | -1.0 | -0.5 | Southwest area |
| location2 | 0.5 | 4.0 | North area |
| location3 | 4.0 | 0.5 | East area |
| location4 | 2.75 | 2.5 | Center-east area |

## Step 5: Send navigation goal

Use `send_action_goal` with these parameters based on the chosen location:

- **action_name:** `/navigate_to_pose`
- **action_type:** `nav2_msgs/action/NavigateToPose`

Goal message (fill in the location values from the table below):

```json
{
  "pose": {
    "header": {
      "frame_id": "map"
    },
    "pose": {
      "position": {"x": X, "y": Y, "z": 0.0},
      "orientation": {"x": 0.0, "y": 0.0, "z": QZ, "w": QW}
    }
  }
}
```

**Pre-computed quaternion values (qx=0, qy=0 always):**

| Location | x | y | qz | qw |
|---|---|---|---|---|
| location1 | -1.0 | -0.5 | -0.9009 | 0.4339 |
| location2 | 0.5 | 4.0 | 0.3827 | 0.9239 |
| location3 | 4.0 | 0.5 | 0.7071 | 0.7071 |
| location4 | 2.75 | 2.5 | -0.7071 | 0.7071 |

Note: The user will be prompted to approve this action (send_action_goal is not pre-approved for safety).

## Step 6: Monitor navigation

Use `get_action_status` to check the status of the navigation goal. Report progress to the user.

If the status is failed or aborted, tell the user the navigation failed and suggest retrying or choosing a different location.

## Step 7: Confirm arrival

Once the action status shows success, use `subscribe_once` on `/amcl_pose` again to read the robot's final position. Report the final coordinates and confirm the robot has arrived at the destination.
```

**Step 3: Verify the file exists**

```bash
ls -la .claude/commands/navigate.md
```

Expected: file exists with the content above.

**Step 4: Commit**

```bash
git add .claude/commands/navigate.md
git commit -m "feat: add /navigate slash command for ros-mcp-server demo"
```

---

### Task 2: Add README section documenting the demo

**Files:**
- Modify: `README.md` (append after the Zenoh + YOLOv8 section, before the end of file)

**Step 1: Add the ros-mcp-server demo section**

Append the following section to `README.md` after the "Zenoh + YOLOv8 Object Detection" section:

```markdown
---

## ros-mcp-server Navigation Demo

Control the TurtleBot from [Claude Code](https://claude.ai/claude-code) using natural language. The [ros-mcp-server](https://github.com/ros-mcp/ros-mcp-server) connects Claude to the ROS 2 navigation stack via rosbridge WebSocket.

### Setup

1. Start the simulation and rosbridge:

```bash
# Terminal 1: Launch Gazebo + Nav2
docker compose up demo-world

# Terminal 2: Start rosbridge WebSocket (port 9090)
docker compose up rosbridge
```

2. Open the project in Claude Code.

### Usage

Type `/navigate` in Claude Code. The slash command will:

1. Connect to the robot via rosbridge
2. Verify Nav2 is ready
3. Show the robot's current position
4. Ask you to pick a destination (4 predefined locations in the house)
5. Send a navigation goal (you'll be asked to approve this action)
6. Monitor progress and confirm arrival

### Safety

The `send_action_goal` tool requires manual approval each time it is called. This is an intentional safety gate for robot-commanding operations.
```

**Step 2: Verify the README renders correctly**

```bash
grep -A 5 "ros-mcp-server Navigation Demo" README.md
```

Expected: the new section header and first few lines appear.

**Step 3: Commit**

```bash
git add README.md
git commit -m "docs: add ros-mcp-server navigation demo section to README"
```

---

### Task 3: Verify end-to-end (manual)

**This task is manual verification, not code.**

**Step 1: Ensure services are running**

```bash
docker compose up demo-world
# In another terminal:
docker compose up rosbridge
```

**Step 2: Open Claude Code and type `/navigate`**

Follow the interactive flow. Verify:
- Connection to rosbridge succeeds
- `/navigate_to_pose` action is discovered
- Current pose is reported
- Navigation goal is sent after user approval
- Robot moves to the chosen location in Gazebo/RViz
- Final pose is reported

**Step 3: Close the beads issue**

```bash
bd close turtlebot-maze-dwg --reason="Demo implemented and verified end-to-end"
bd sync
```
