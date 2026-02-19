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
