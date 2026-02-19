# Design: Navigate Robot via ros-mcp-server Demo

**Date:** 2026-02-19
**Beads issue:** turtlebot-maze-dwg
**Status:** Approved

## Summary

An interactive demo using a Claude Code slash command (`/navigate`) that guides
the user through commanding the TurtleBot to navigate to a named location in the
sim_house world via ros-mcp-server tools.

## Decisions

| Decision | Choice | Rationale |
|---|---|---|
| Demo format | Interactive walkthrough via slash command | Repeatable, guided, still interactive |
| Permission model | Manual approval for `send_action_goal` | Safety-first for robot commands |
| Target selection | Named locations only | Simpler, demonstrates Claude reasoning about the world |

## Slash Command: `/navigate`

**File:** `.claude/commands/navigate.md`

### Flow

1. **Connect** -- `connect_to_robot(ip="127.0.0.1", port=9090)`
2. **Verify** -- `get_actions` to confirm `/navigate_to_pose` is available
3. **Get current pose** -- `subscribe_once` on `/amcl_pose`
4. **Ask user** -- Present 4 named locations, ask which one
5. **Navigate** -- `send_action_goal` on `/navigate_to_pose` with pre-computed quaternion
6. **Monitor** -- `get_action_status` to track progress
7. **Confirm** -- `subscribe_once` on `/amcl_pose` to verify arrival

### Predefined Locations (embedded in prompt)

| Location | x | y | theta (rad) | qz | qw |
|---|---|---|---|---|---|
| location1 | -1.0 | -0.5 | -2.25 | -0.9009 | 0.4339 |
| location2 | 0.5 | 4.0 | 0.785 | 0.3827 | 0.9239 |
| location3 | 4.0 | 0.5 | 1.571 | 0.7071 | 0.7071 |
| location4 | 2.75 | 2.5 | -1.571 | -0.7071 | 0.7071 |

Quaternion values pre-computed (qx=0, qy=0) to avoid runtime trig errors.

### Error Handling

- **Connection failure** -- Tell user to verify `docker compose up rosbridge` is running
- **Nav2 not ready** -- If `/navigate_to_pose` action not listed, advise waiting for Nav2 initialization
- **Navigation failure** -- If action status is failed/aborted, report and suggest retry

## README Addition

New section documenting:
- Prerequisites: `demo-world` and `rosbridge` services must be running
- Usage: type `/navigate` in Claude Code
- What to expect: approval prompts for `send_action_goal` (intentional safety gate)

## Out of Scope

- Arbitrary coordinate input
- Pre-approving `send_action_goal` in settings
- Automated (non-interactive) script
- Camera/vision integration during navigation
