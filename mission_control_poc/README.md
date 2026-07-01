# Mission Control PoC

This is a small backend service that implements **System Control & Logistics** requirements from your spec:
- **Mission request validation** (destination exists in config)
- **Action state control**: *Idle → En-route → Paused → Completed*
- **Manual safe pause** (operator-triggered)
- **Scheduling policy** (queue + conflict prevention)
- **Blocked/trapped detection** with retries and help-request escalation
- **Mission logging** (mission table + event timeline)
- **Status interface** with ≥1 Hz refresh (WebSocket)
- **Config-driven destinations** (YAML) with reload endpoint

It started as a ROS2-independent mission-control backend. This repo now includes the ROS2/Nav2 adapter as well, so the server can run in:
- `sim` mode: the original in-process simulated robot
- `ros2` mode: direct Nav2 integration from `robot_server` without editing `catering_bot-main`

---

## Quick start

```bash
./setup_env_linux.sh
./run_server.sh
```

When using the ROS2 adapter with the current `catering_bot-main` package, prefer:

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export MISSION_CONTROL_ROBOT_BACKEND=ros2
export MISSION_CONTROL_ROS2_ROBOT_WORKSPACE=$HOME/robot_ws
export MISSION_CONTROL_ROS2_MAPPING_WORKSPACE=$HOME/dev_ws
export MISSION_CONTROL_ROS2_NAV_WORKSPACE=$HOME/robot_ws
export MISSION_CONTROL_ROS2_MAP_DIRECTORY=$HOME/dev_ws/src/my_bot/maps
./run_server.sh
```

`--reload` is convenient for the simulated backend, but it can create duplicate ROS 2 nodes/processes.

Note: the checked-in `.venv` in this folder is a macOS artifact and is not usable on Ubuntu. Use `./setup_env_linux.sh` to create `.venv-local` on this machine.

Open:
- http://127.0.0.1:8000/ui (dashboard UI)
- http://127.0.0.1:8000/docs (Swagger UI)
- WebSocket status stream: `ws://127.0.0.1:8000/ws/status`

---

## Config-driven destinations

Edit: `config/destinations.yaml`

```yaml
destinations:
  - name: "Storage"
    pose: {x: 0.0, y: 0.0, yaw: 0.0}
home_destination: "Storage"
```

Reload without restart:

```bash
curl -X POST http://127.0.0.1:8000/destinations/reload
```

---

## Example: create a mission (single trip)

```bash
curl -X POST http://127.0.0.1:8000/missions \
  -H "Content-Type: application/json" \
  -d '{
    "requested_by": "event-staff-17",
    "command_source": {"type":"user","id":"event-staff-17"},
    "to_destination": "Ballroom",
    "schedule_type": "single"
  }'
```

---

## Example: pause / resume / cancel

```bash
curl -X POST http://127.0.0.1:8000/missions/<MISSION_ID>/pause \
  -H "Content-Type: application/json" \
  -d '{"command_source":{"type":"operator","id":"supervisor-1"}}'
```

---

## Example: simulate a blocked condition (for testing)

The default robot is a **simulated robot** (`robot-1`).

```bash
curl -X POST http://127.0.0.1:8000/robots/robot-1/telemetry \
  -H "Content-Type: application/json" \
  -d '{"blocked": true}'
```

The mission control loop will:
- detect blocked within ~5 seconds
- attempt recovery (pause 2s → resume) up to 3 times
- then set the mission to **Paused** and mark `help_required=1`

Unblock:

```bash
curl -X POST http://127.0.0.1:8000/robots/robot-1/telemetry \
  -H "Content-Type: application/json" \
  -d '{"blocked": false}'
```

---

## Where ROS2/Nav2 would plug in

`mission_control/robot_adapter.py` now includes `Ros2RobotAdapter`. The server selects the backend from environment variables:

```bash
export MISSION_CONTROL_ROBOT_BACKEND=ros2
export MISSION_CONTROL_ROBOT_ID=robot-1
uvicorn app:app --port 8000
```

Defaults are chosen to match the current `catering_bot-main` launch/config:

- Nav2 action: `navigate_to_pose`
- map frame: `map`
- localization topic: `/amcl_pose`
- odometry topic: `/diff_cont/odom`
- manual override topic: `/cmd_vel_joy`
- map topic: `/map`
- initial pose topic: `/initialpose`
- goal preview topic: `/goal_pose`

The current `catering_bot-main` package does **not** include the older
`/sys_command` + `/sys_status` launcher bridge. The ROS2 adapter therefore
uses a local launcher by default:

- robot stack: `ros2 launch my_bot rpi_robot.launch.py`
- mapping: `ros2 launch my_bot central_compute.launch.py use_slam:=true use_nav2:=false`
- Nav2: `ros2 launch my_bot central_compute.launch.py use_slam:=false use_nav2:=true map:=...`
- save map: `ros2 run nav2_map_server map_saver_cli -f ...`

The Pi still needs the bare robot stack running on the same ROS domain. You can
start that manually from `catering_bot-main` using the project command sheet:

```bash
cd ~/robot_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch my_bot rpi_robot.launch.py
```

Optional ROS2 tuning env vars:

- `MISSION_CONTROL_ROS2_NODE_NAME`
- `MISSION_CONTROL_ROS2_NAVIGATE_ACTION`
- `MISSION_CONTROL_ROS2_MAP_FRAME`
- `MISSION_CONTROL_ROS2_MAP_TOPIC`
- `MISSION_CONTROL_ROS2_LOCALIZATION_TOPIC`
- `MISSION_CONTROL_ROS2_ODOM_TOPIC`
- `MISSION_CONTROL_ROS2_BATTERY_TOPIC`
- `MISSION_CONTROL_ROS2_JOYSTICK_TOPIC`
- `MISSION_CONTROL_ROS2_INITIAL_POSE_TOPIC`
- `MISSION_CONTROL_ROS2_GOAL_POSE_TOPIC`
- `MISSION_CONTROL_ROS2_LAUNCHER_MODE` (`local` by default, or `topic` for a custom bridge)
- `MISSION_CONTROL_ROS2_PACKAGE_NAME`
- `MISSION_CONTROL_ROS2_CENTRAL_WORKSPACE`
- `MISSION_CONTROL_ROS2_MAPPING_WORKSPACE`
- `MISSION_CONTROL_ROS2_NAV_WORKSPACE`
- `MISSION_CONTROL_ROS2_ROBOT_WORKSPACE`
- `MISSION_CONTROL_ROS2_MAP_DIRECTORY`
- `MISSION_CONTROL_ROS2_MAPPING_USE_JOYSTICK`
- `MISSION_CONTROL_ROS2_NAV_USE_JOYSTICK`
- `MISSION_CONTROL_ROS2_LAUNCH_RVIZ`
- `MISSION_CONTROL_ROS2_ACTION_TIMEOUT_S`
- `MISSION_CONTROL_ROS2_CONNECTION_TIMEOUT_S`
- `MISSION_CONTROL_ROS2_LOCALIZATION_TIMEOUT_S`
- `MISSION_CONTROL_ROS2_MANUAL_OVERRIDE_TIMEOUT_S`
- `MISSION_CONTROL_ROS2_STALL_TIMEOUT_S`
- `MISSION_CONTROL_ROS2_GOAL_TOLERANCE_M`

The adapter interface remains:

- `start_mission(mission_id, plan)`
- `pause()`
- `resume()`
- `cancel()`
- `reset_to_idle()`
- `snapshot()`

The **API + scheduler + logging** code remains unchanged. `Ros2RobotAdapter` resolves destination names from `config/destinations.yaml`, sends Nav2 goals, cancels/resends goals for pause/resume, and reports robot status back into the existing mission-control loop.
