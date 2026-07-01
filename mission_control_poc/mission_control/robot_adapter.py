from __future__ import annotations

import asyncio
import json
import math
import os
import random
import shlex
import signal
import subprocess
import threading
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml

from .config_loader import DestinationConfig
from .types import MissionOutcome, MissionState, RobotMode


@dataclass
class RobotTelemetry:
    robot_id: str
    state: MissionState
    mode: RobotMode
    current_mission_id: Optional[str]
    last_heartbeat_at: float
    connection_ok: bool
    localization_valid: bool
    obstacle_stop: bool
    blocked: bool
    battery_v: float
    pose: Dict[str, float]
    outcome: Optional[MissionOutcome] = None


@dataclass
class RobotPowerStatus:
    available: bool
    mode: str
    battery_percent: Optional[float] = None
    latency_ms: Optional[float] = None
    safety_lock: bool = False
    recent_log: Optional[str] = None


@dataclass(frozen=True)
class Ros2AdapterConfig:
    node_name: str = "mission_control_ros2_adapter"
    navigate_action_name: str = "navigate_to_pose"
    map_frame: str = "map"
    map_topic: Optional[str] = "/map"
    goal_pose_topic: Optional[str] = "/goal_pose"
    localization_topic: str = "/amcl_pose"
    odom_topic: str = "/diff_cont/odom"
    battery_topic: Optional[str] = "/battery_state"
    joystick_topic: Optional[str] = "/cmd_vel_joy"
    initial_pose_topic: Optional[str] = "/initialpose"
    global_localization_service: Optional[str] = "/reinitialize_global_localization"
    system_command_topic: Optional[str] = None
    system_status_topic: Optional[str] = None
    power_command_topic: Optional[str] = None
    power_mode_topic: Optional[str] = None
    power_log_topic: Optional[str] = "/robot_health/log"
    power_latency_topic: Optional[str] = None
    power_battery_percent_topic: Optional[str] = None
    power_ping_topic: Optional[str] = None
    launcher_mode: str = "local"
    package_name: str = "my_bot"
    robot_workspace: str = "$HOME/robot_ws"
    central_workspace: str = "$HOME/dev_ws"
    mapping_workspace: str = "$HOME/dev_ws"
    nav_workspace: str = "$HOME/robot_ws"
    map_directory: str = "$HOME/dev_ws/src/my_bot/maps"
    robot_launch_file: str = "rpi_robot.launch.py"
    central_launch_file: str = "central_compute.launch.py"
    mapping_use_joystick: bool = True
    nav_use_joystick: bool = False
    launch_rviz: bool = False
    power_ping_period_s: float = 0.5
    launcher_request_timeout_s: float = 8.0
    action_server_timeout_s: float = 5.0
    connection_timeout_s: float = 2.5
    localization_timeout_s: float = 5.0
    manual_override_timeout_s: float = 0.75
    localization_spin_angular_z: float = 0.25
    localization_spin_duration_s: float = 8.0
    localization_spin_rate_hz: float = 10.0
    stall_speed_epsilon: float = 0.02
    stall_angular_speed_epsilon: float = 0.05
    stall_detect_after_s: float = 0.5
    goal_tolerance_m: float = 0.35

    @classmethod
    def from_env(cls) -> "Ros2AdapterConfig":
        return cls(
            node_name=os.getenv("MISSION_CONTROL_ROS2_NODE_NAME", cls.node_name),
            navigate_action_name=os.getenv("MISSION_CONTROL_ROS2_NAVIGATE_ACTION", cls.navigate_action_name),
            map_frame=os.getenv("MISSION_CONTROL_ROS2_MAP_FRAME", cls.map_frame),
            map_topic=_env_optional_str("MISSION_CONTROL_ROS2_MAP_TOPIC", cls.map_topic),
            goal_pose_topic=_env_optional_str("MISSION_CONTROL_ROS2_GOAL_POSE_TOPIC", cls.goal_pose_topic),
            localization_topic=os.getenv("MISSION_CONTROL_ROS2_LOCALIZATION_TOPIC", cls.localization_topic),
            odom_topic=os.getenv("MISSION_CONTROL_ROS2_ODOM_TOPIC", cls.odom_topic),
            battery_topic=_env_optional_str("MISSION_CONTROL_ROS2_BATTERY_TOPIC", cls.battery_topic),
            joystick_topic=_env_optional_str("MISSION_CONTROL_ROS2_JOYSTICK_TOPIC", cls.joystick_topic),
            initial_pose_topic=_env_optional_str("MISSION_CONTROL_ROS2_INITIAL_POSE_TOPIC", cls.initial_pose_topic),
            global_localization_service=_env_optional_str(
                "MISSION_CONTROL_ROS2_GLOBAL_LOCALIZATION_SERVICE",
                cls.global_localization_service,
            ),
            system_command_topic=_env_optional_str("MISSION_CONTROL_ROS2_SYSTEM_COMMAND_TOPIC", cls.system_command_topic),
            system_status_topic=_env_optional_str("MISSION_CONTROL_ROS2_SYSTEM_STATUS_TOPIC", cls.system_status_topic),
            power_command_topic=_env_optional_str("MISSION_CONTROL_ROS2_POWER_COMMAND_TOPIC", cls.power_command_topic),
            power_mode_topic=_env_optional_str("MISSION_CONTROL_ROS2_POWER_MODE_TOPIC", cls.power_mode_topic),
            power_log_topic=_env_optional_str("MISSION_CONTROL_ROS2_POWER_LOG_TOPIC", cls.power_log_topic),
            power_latency_topic=_env_optional_str("MISSION_CONTROL_ROS2_POWER_LATENCY_TOPIC", cls.power_latency_topic),
            power_battery_percent_topic=_env_optional_str("MISSION_CONTROL_ROS2_POWER_BATTERY_PERCENT_TOPIC", cls.power_battery_percent_topic),
            power_ping_topic=_env_optional_str("MISSION_CONTROL_ROS2_POWER_PING_TOPIC", cls.power_ping_topic),
            launcher_mode=os.getenv("MISSION_CONTROL_ROS2_LAUNCHER_MODE", cls.launcher_mode).strip().lower(),
            package_name=os.getenv("MISSION_CONTROL_ROS2_PACKAGE_NAME", cls.package_name).strip() or cls.package_name,
            robot_workspace=os.getenv("MISSION_CONTROL_ROS2_ROBOT_WORKSPACE", cls.robot_workspace),
            central_workspace=os.getenv("MISSION_CONTROL_ROS2_CENTRAL_WORKSPACE", cls.central_workspace),
            mapping_workspace=os.getenv(
                "MISSION_CONTROL_ROS2_MAPPING_WORKSPACE",
                os.getenv("MISSION_CONTROL_ROS2_CENTRAL_WORKSPACE", cls.mapping_workspace),
            ),
            nav_workspace=os.getenv(
                "MISSION_CONTROL_ROS2_NAV_WORKSPACE",
                os.getenv("MISSION_CONTROL_ROS2_CENTRAL_WORKSPACE", cls.nav_workspace),
            ),
            map_directory=os.getenv("MISSION_CONTROL_ROS2_MAP_DIRECTORY", cls.map_directory),
            robot_launch_file=os.getenv("MISSION_CONTROL_ROS2_ROBOT_LAUNCH_FILE", cls.robot_launch_file).strip() or cls.robot_launch_file,
            central_launch_file=os.getenv("MISSION_CONTROL_ROS2_CENTRAL_LAUNCH_FILE", cls.central_launch_file).strip() or cls.central_launch_file,
            mapping_use_joystick=_env_bool("MISSION_CONTROL_ROS2_MAPPING_USE_JOYSTICK", cls.mapping_use_joystick),
            nav_use_joystick=_env_bool("MISSION_CONTROL_ROS2_NAV_USE_JOYSTICK", cls.nav_use_joystick),
            launch_rviz=_env_bool("MISSION_CONTROL_ROS2_LAUNCH_RVIZ", cls.launch_rviz),
            power_ping_period_s=_env_float("MISSION_CONTROL_ROS2_POWER_PING_PERIOD_S", cls.power_ping_period_s),
            launcher_request_timeout_s=_env_float("MISSION_CONTROL_ROS2_LAUNCHER_TIMEOUT_S", cls.launcher_request_timeout_s),
            action_server_timeout_s=_env_float("MISSION_CONTROL_ROS2_ACTION_TIMEOUT_S", cls.action_server_timeout_s),
            connection_timeout_s=_env_float("MISSION_CONTROL_ROS2_CONNECTION_TIMEOUT_S", cls.connection_timeout_s),
            localization_timeout_s=_env_float("MISSION_CONTROL_ROS2_LOCALIZATION_TIMEOUT_S", cls.localization_timeout_s),
            manual_override_timeout_s=_env_float("MISSION_CONTROL_ROS2_MANUAL_OVERRIDE_TIMEOUT_S", cls.manual_override_timeout_s),
            localization_spin_angular_z=_env_float(
                "MISSION_CONTROL_ROS2_LOCALIZATION_SPIN_ANGULAR_Z",
                cls.localization_spin_angular_z,
            ),
            localization_spin_duration_s=_env_float(
                "MISSION_CONTROL_ROS2_LOCALIZATION_SPIN_DURATION_S",
                cls.localization_spin_duration_s,
            ),
            localization_spin_rate_hz=_env_float(
                "MISSION_CONTROL_ROS2_LOCALIZATION_SPIN_RATE_HZ",
                cls.localization_spin_rate_hz,
            ),
            stall_speed_epsilon=_env_float("MISSION_CONTROL_ROS2_STALL_SPEED_EPSILON", cls.stall_speed_epsilon),
            stall_angular_speed_epsilon=_env_float("MISSION_CONTROL_ROS2_STALL_ANGULAR_EPSILON", cls.stall_angular_speed_epsilon),
            stall_detect_after_s=_env_float("MISSION_CONTROL_ROS2_STALL_TIMEOUT_S", cls.stall_detect_after_s),
            goal_tolerance_m=_env_float("MISSION_CONTROL_ROS2_GOAL_TOLERANCE_M", cls.goal_tolerance_m),
        )


class RobotAdapter:
    """Abstract robot adapter.

    The Mission Control layer talks to *this* interface.

    Later, your ROS2 integration becomes an implementation of this class
    (e.g., a Nav2 Action client) without changing the mission scheduler/API.
    """

    def __init__(self, robot_id: str):
        self.robot_id = robot_id

    async def start_mission(self, mission_id: str, plan: List[str]) -> None:
        raise NotImplementedError

    async def pause(self) -> None:
        raise NotImplementedError

    async def resume(self) -> None:
        raise NotImplementedError

    async def cancel(self) -> None:
        raise NotImplementedError

    async def reset_to_idle(self) -> None:
        """Clear any mission context after mission manager records completion."""
        raise NotImplementedError

    def snapshot(self) -> RobotTelemetry:
        raise NotImplementedError

    async def set_power_mode(self, mode: str) -> None:
        raise NotImplementedError

    async def send_manual_drive_command(self, linear: float, angular: float) -> None:
        raise NotImplementedError

    async def localize(self) -> Dict[str, Any]:
        raise NotImplementedError

    async def send_system_command(self, command: str, map_name: Optional[str] = None) -> None:
        raise NotImplementedError

    async def set_initial_pose(self, x: float, y: float, yaw: float) -> None:
        raise NotImplementedError

    async def set_goal_pose(self, x: float, y: float, yaw: float) -> None:
        raise NotImplementedError

    async def save_map(self, map_name: str) -> Dict[str, Any]:
        raise NotImplementedError

    async def delete_map(self, map_name: str) -> Dict[str, Any]:
        raise NotImplementedError

    async def load_map_preview(self, map_name: str) -> Dict[str, Any]:
        raise NotImplementedError

    def operator_snapshot(self) -> Dict[str, Any]:
        return {
            "map_available": False,
            "map": None,
            "goal_pose": None,
            "initial_pose": None,
            "system_commands_available": False,
            "initial_pose_available": False,
            "goal_pose_available": False,
            "last_system_command": None,
            "saved_maps": [],
            "current_map_name": None,
            "maps_directory": None,
            "launcher_message": None,
            "launcher_processes": {},
        }

    def power_snapshot(self) -> RobotPowerStatus:
        return RobotPowerStatus(available=False, mode="Unavailable")

    def shutdown(self) -> None:
        """Release adapter resources during server shutdown."""
        return None


class SimRobotAdapter(RobotAdapter):
    """A simple simulated robot.

    - Takes a mission plan: list of destination names.
    - "Drives" by sleeping.
    - Supports pause/resume/cancel.
    - Can be forced into a blocked condition for testing.
    """

    def __init__(self, robot_id: str, speed_scale: float = 1.0):
        super().__init__(robot_id)
        self._state: MissionState = MissionState.IDLE
        self._mode: RobotMode = RobotMode.AUTO
        self._current_mission_id: Optional[str] = None
        self._power_mode = "AUTO"
        self._power_safety_lock = False
        self._power_latency_ms = 12.0
        self._power_recent_log = "Power controls ready."

        self._connection_ok = True
        self._localization_valid = True
        self._obstacle_stop = False
        self._blocked = False

        self._battery_v = 24.0
        self._pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self._last_initial_pose: Optional[Dict[str, float]] = None
        self._last_goal_pose: Optional[Dict[str, float]] = None
        self._last_system_command: Optional[str] = None
        self._current_map_name: Optional[str] = None
        self._maps_directory = "/sim/maps"
        self._launcher_message = "Launcher ready."
        self._launcher_processes: Dict[str, bool] = {"robot": False, "slam": False, "nav": False}
        self._saved_maps: Dict[str, Dict[str, Any]] = {
            "test_map1": {
                "name": "test_map1",
                "width": 32,
                "height": 24,
                "resolution": 0.1,
                "origin": {"x": -1.6, "y": -1.2, "yaw": 0.0},
                "data": [0] * (32 * 24),
                "updated_at": time.time(),
            }
        }

        self._paused = asyncio.Event()
        self._paused.set()
        self._cancel_requested = False
        self._task: Optional[asyncio.Task] = None
        self._last_outcome: Optional[MissionOutcome] = None

        self._speed_scale = max(0.1, float(speed_scale))

    def set_manual_override(self, enabled: bool) -> None:
        self._mode = RobotMode.MANUAL_OVERRIDE if enabled else RobotMode.AUTO

    def set_blocked(self, blocked: bool) -> None:
        self._blocked = bool(blocked)

    def set_localization_valid(self, ok: bool) -> None:
        self._localization_valid = bool(ok)

    def set_obstacle_stop(self, stop: bool) -> None:
        self._obstacle_stop = bool(stop)

    async def start_mission(self, mission_id: str, plan: List[str]) -> None:
        if self._power_safety_lock or self._power_mode in {"STOP", "OFF"}:
            raise RuntimeError("Robot is off; cannot start autonomous mission.")
        if self._task and not self._task.done():
            raise RuntimeError("Robot already executing a mission.")
        self._current_mission_id = mission_id
        self._cancel_requested = False
        self._last_outcome = None
        self._state = MissionState.EN_ROUTE
        self._paused.set()
        self._task = asyncio.create_task(self._run_plan(plan))

    async def _run_plan(self, plan: List[str]) -> None:
        # Very rough: each leg takes 4-10 seconds scaled by speed_scale
        try:
            for _i, _dest in enumerate(plan):
                leg_time = random.uniform(4.0, 10.0) / self._speed_scale
                started = time.time()
                while time.time() - started < leg_time:
                    # Heartbeat + battery drain
                    self._battery_v = max(20.0, self._battery_v - 0.005)
                    self._pose["x"] += random.uniform(-0.02, 0.05)
                    self._pose["y"] += random.uniform(-0.02, 0.05)
                    self._pose["yaw"] += random.uniform(-0.01, 0.01)

                    # Pause handling
                    await self._paused.wait()

                    # Cancel handling
                    if self._cancel_requested:
                        self._state = MissionState.COMPLETED
                        self._last_outcome = MissionOutcome.CANCELED
                        return

                    if self._mode == RobotMode.MANUAL_OVERRIDE:
                        await asyncio.sleep(0.2)
                        continue

                    # Blocked handling: if blocked, just sit until unblocked or canceled.
                    if self._blocked or self._obstacle_stop:
                        await asyncio.sleep(0.2)
                        continue

                    await asyncio.sleep(0.2)

            self._state = MissionState.COMPLETED
            self._last_outcome = MissionOutcome.SUCCESS
        finally:
            # Keep current_mission_id until mission manager clears it
            pass

    async def pause(self) -> None:
        # Pausing affects motion; mission manager controls mission state separately.
        self._paused.clear()

    async def resume(self) -> None:
        self._paused.set()

    async def cancel(self) -> None:
        self._cancel_requested = True
        self._paused.set()

    async def reset_to_idle(self) -> None:
        # Note: in a real robot, you'd also clear navigation goals, etc.
        self._state = MissionState.IDLE
        self._current_mission_id = None
        self._cancel_requested = False
        self._last_outcome = None
        self._paused.set()

    def snapshot(self) -> RobotTelemetry:
        return RobotTelemetry(
            robot_id=self.robot_id,
            state=self._state,
            mode=self._mode,
            current_mission_id=self._current_mission_id,
            last_heartbeat_at=time.time(),
            connection_ok=self._connection_ok,
            localization_valid=self._localization_valid,
            obstacle_stop=self._obstacle_stop,
            blocked=self._blocked,
            battery_v=self._battery_v,
            pose=dict(self._pose),
            outcome=self._last_outcome,
        )

    async def set_power_mode(self, mode: str) -> None:
        command = mode.strip().upper()
        if command not in {"AUTO", "MANUAL", "RESET", "STOP", "ON", "OFF"}:
            raise ValueError(f"Unsupported power mode: {mode}")

        if command in {"RESET", "ON", "AUTO", "MANUAL"}:
            self._power_safety_lock = False
            self._power_mode = "ON"
            self._obstacle_stop = False
            self._blocked = False
            self._mode = RobotMode.AUTO
            self._power_recent_log = "Robot enabled."
            return

        if command in {"STOP", "OFF"}:
            self._power_mode = "OFF"
            self._power_safety_lock = True
            self._mode = RobotMode.MANUAL_OVERRIDE
            self._obstacle_stop = True
            if self._current_mission_id is not None and self._state == MissionState.EN_ROUTE:
                self._state = MissionState.PAUSED
                self._paused.clear()
            self._power_recent_log = "Emergency stop triggered."
            return

    async def send_manual_drive_command(self, linear: float, angular: float) -> None:
        if self._power_safety_lock:
            raise RuntimeError("Robot is off. Turn it on before manual driving.")

        linear = _clamp(float(linear), -1.0, 1.0)
        angular = _clamp(float(angular), -1.0, 1.0)

        self._mode = RobotMode.MANUAL_OVERRIDE
        if abs(linear) < 1e-4 and abs(angular) < 1e-4:
            self._mode = RobotMode.AUTO
            self._power_recent_log = "Manual drive stopped."
            return

        next_yaw = self._pose["yaw"] + (angular * 0.12)
        self._pose["yaw"] = next_yaw
        self._pose["x"] += math.cos(next_yaw) * linear * 0.18
        self._pose["y"] += math.sin(next_yaw) * linear * 0.18
        self._power_recent_log = f"Manual drive command: linear={linear:.2f}, angular={angular:.2f}"

    async def localize(self) -> Dict[str, Any]:
        if self._power_safety_lock:
            raise RuntimeError("Robot is off. Turn it on before localizing.")
        self._localization_valid = True
        self._power_recent_log = "Global localization simulated."
        return {
            "robot_id": self.robot_id,
            "ok": True,
            "message": "Global localization simulated.",
        }

    async def send_system_command(self, command: str, map_name: Optional[str] = None) -> None:
        normalized = command.strip().lower()
        if normalized not in {"launch_robot", "launch_slam", "launch_nav", "save_map", "kill_all"}:
            raise ValueError(f"Unsupported system command: {command}")
        if normalized == "launch_nav":
            if not map_name:
                raise RuntimeError("Select a map before launching navigation.")
            if map_name not in self._saved_maps:
                raise RuntimeError(f"Saved map '{map_name}' was not found.")
            self._current_map_name = map_name
            self._launcher_processes["nav"] = True
            self._launcher_message = f"Navigation launched with map {map_name}."
        elif normalized == "launch_robot":
            self._launcher_processes["robot"] = True
            self._launcher_message = "Robot launch command sent."
        elif normalized == "launch_slam":
            self._launcher_processes["slam"] = True
            self._current_map_name = None
            self._launcher_message = "Mapping mode launched."
        elif normalized == "kill_all":
            self._launcher_processes = {"robot": False, "slam": False, "nav": False}
            self._launcher_message = "All launcher processes stopped."
        elif normalized == "save_map":
            self._launcher_message = "Use named map save to persist a map."
        self._last_system_command = normalized
        self._power_recent_log = f"System command sent: {normalized}"

    async def set_initial_pose(self, x: float, y: float, yaw: float) -> None:
        self._last_initial_pose = {"x": float(x), "y": float(y), "yaw": float(yaw)}
        self._pose = dict(self._last_initial_pose)
        self._power_recent_log = f"Initial pose set to x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"

    async def set_goal_pose(self, x: float, y: float, yaw: float) -> None:
        self._last_goal_pose = {"x": float(x), "y": float(y), "yaw": float(yaw)}
        self._power_recent_log = f"Goal pose set to x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"

    async def save_map(self, map_name: str) -> Dict[str, Any]:
        normalized = map_name.strip()
        if not normalized:
            raise RuntimeError("Map name is required.")
        existing = {name.lower() for name in self._saved_maps}
        if normalized.lower() in existing:
            raise RuntimeError(f"Map '{normalized}' already exists.")

        self._saved_maps[normalized] = {
            "name": normalized,
            "width": 32,
            "height": 24,
            "resolution": 0.1,
            "origin": {"x": -1.6, "y": -1.2, "yaw": 0.0},
            "data": [0] * (32 * 24),
            "updated_at": time.time(),
        }
        self._launcher_message = f"Saved map {normalized}."
        return {"maps": sorted(self._saved_maps), "current_map_name": self._current_map_name}

    async def delete_map(self, map_name: str) -> Dict[str, Any]:
        normalized = map_name.strip()
        existing = next((name for name in self._saved_maps if name.lower() == normalized.lower()), None)
        if existing is None:
            raise RuntimeError(f"Map '{map_name}' was not found.")
        if existing == self._current_map_name and self._launcher_processes.get("nav"):
            raise RuntimeError("Cannot delete the current active navigation map.")
        del self._saved_maps[existing]
        self._launcher_message = f"Deleted map {existing}."
        return {"maps": sorted(self._saved_maps), "current_map_name": self._current_map_name}

    async def load_map_preview(self, map_name: str) -> Dict[str, Any]:
        existing = next((name for name in self._saved_maps if name.lower() == map_name.strip().lower()), None)
        if existing is None:
            raise RuntimeError(f"Map '{map_name}' was not found.")
        return dict(self._saved_maps[existing])

    def operator_snapshot(self) -> Dict[str, Any]:
        current_map = self._saved_maps.get(self._current_map_name) if self._current_map_name else None
        return {
            "map_available": current_map is not None,
            "map": dict(current_map) if current_map is not None else None,
            "goal_pose": dict(self._last_goal_pose) if self._last_goal_pose else None,
            "initial_pose": dict(self._last_initial_pose) if self._last_initial_pose else None,
            "system_commands_available": True,
            "initial_pose_available": True,
            "goal_pose_available": True,
            "last_system_command": self._last_system_command,
            "saved_maps": sorted(self._saved_maps),
            "current_map_name": self._current_map_name,
            "maps_directory": self._maps_directory,
            "launcher_message": self._launcher_message,
            "launcher_processes": dict(self._launcher_processes),
        }

    def power_snapshot(self) -> RobotPowerStatus:
        percent = max(0.0, min(100.0, ((self._battery_v - 20.0) / 4.0) * 100.0))
        return RobotPowerStatus(
            available=True,
            mode=self._power_mode,
            battery_percent=percent,
            latency_ms=self._power_latency_ms,
            safety_lock=self._power_safety_lock,
            recent_log=self._power_recent_log,
        )


class Ros2RobotAdapter(RobotAdapter):
    """ROS 2 / Nav2-backed adapter used by the mission-control scheduler."""

    def __init__(
        self,
        robot_id: str,
        dest_config: DestinationConfig,
        config: Optional[Ros2AdapterConfig] = None,
    ):
        super().__init__(robot_id)
        self._dest_config = dest_config
        self._config = config or Ros2AdapterConfig.from_env()
        self._ros = _import_ros2_modules()

        self._lock = threading.RLock()
        self._resume_event = threading.Event()
        self._resume_event.set()
        self._goal_done_event = threading.Event()
        self._shutdown_requested = False
        self._pause_requested = False
        self._cancel_requested = False
        self._cancel_future_in_flight = False

        self._state: MissionState = MissionState.IDLE
        self._mode: RobotMode = RobotMode.AUTO
        self._current_mission_id: Optional[str] = None
        self._current_plan: List[str] = []
        self._current_leg_index = 0
        self._current_destination: Optional[str] = None
        self._current_goal_pose: Optional[Dict[str, float]] = None
        self._last_outcome: Optional[MissionOutcome] = None

        self._connection_ok = False
        self._localization_valid = False
        self._obstacle_stop = False
        self._blocked = False
        self._battery_v = 0.0
        self._power_mode = "AUTO"
        self._power_battery_percent: Optional[float] = None
        self._power_latency_ms: Optional[float] = None
        self._power_recent_log: Optional[str] = None
        self._power_safety_lock = False
        self._map_snapshot: Optional[Dict[str, Any]] = None
        self._last_initial_pose: Optional[Dict[str, float]] = None
        self._last_goal_pose: Optional[Dict[str, float]] = None
        self._last_system_command: Optional[str] = None
        self._saved_map_names: List[str] = []
        self._current_map_name: Optional[str] = None
        self._maps_directory: Optional[str] = None
        self._launcher_message: Optional[str] = None
        self._launcher_processes: Dict[str, bool] = {}
        self._map_preview_cache: Dict[str, Dict[str, Any]] = {}
        self._pending_launcher_requests: Dict[str, Dict[str, Any]] = {}
        self._local_processes: Dict[str, subprocess.Popen[Any]] = {}
        self._pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self._linear_speed = 0.0
        self._angular_speed = 0.0
        now = time.time()
        self._last_heartbeat_at = now
        self._last_localization_at = 0.0
        self._last_motion_at = now
        self._last_joy_cmd_at = 0.0
        self._goal_active_since = 0.0

        self._send_goal_future = None
        self._active_goal_handle = None
        self._goal_result_status: Optional[int] = None
        self._goal_result_error: Optional[str] = None

        self._mission_thread: Optional[threading.Thread] = None
        self._spin_thread: Optional[threading.Thread] = None
        self._executor = None
        self._node = None
        self._navigate_client = None
        self._power_command_publisher = None
        self._power_ping_publisher = None
        self._manual_command_publisher = None
        self._global_localization_client = None
        self._initial_pose_publisher = None
        self._goal_pose_publisher = None
        self._system_command_publisher = None
        self._system_status_subscription = None
        self._context = None

        self._goal_status_succeeded = self._ros["GoalStatus"].STATUS_SUCCEEDED
        self._goal_status_aborted = self._ros["GoalStatus"].STATUS_ABORTED
        self._goal_status_canceled = self._ros["GoalStatus"].STATUS_CANCELED
        self._goal_status_unknown = self._ros["GoalStatus"].STATUS_UNKNOWN

        self._initialize_local_launcher_state()
        self._init_ros()

    def _local_launcher_enabled(self) -> bool:
        return self._config.launcher_mode == "local"

    def _initialize_local_launcher_state(self) -> None:
        if not self._local_launcher_enabled():
            return
        maps_dir = _expanded_path(self._config.map_directory)
        with self._lock:
            self._maps_directory = str(maps_dir)
            self._launcher_processes = {"robot": False, "slam": False, "nav": False}
            self._refresh_local_maps_locked()
            self._launcher_message = "Local catering_bot launcher ready."

    def _refresh_local_maps_locked(self) -> List[str]:
        maps_dir = _expanded_path(self._config.map_directory)
        self._maps_directory = str(maps_dir)
        if not maps_dir.exists():
            self._saved_map_names = []
            return []
        self._saved_map_names = sorted(path.stem for path in maps_dir.glob("*.yaml") if path.is_file())
        return list(self._saved_map_names)

    def _local_map_yaml_path(self, map_name: str) -> Path:
        maps_dir = _expanded_path(self._config.map_directory)
        clean_name = Path(str(map_name).strip()).stem
        if not clean_name:
            raise RuntimeError("Map name is required.")
        return maps_dir / f"{clean_name}.yaml"

    def _start_local_process_locked(self, key: str, workspace: str, ros_args: List[str]) -> None:
        self._stop_local_process_locked(key)
        command = _ros_workspace_command(_expanded_path(workspace), ros_args)
        process = subprocess.Popen(
            command,
            start_new_session=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
        )
        self._local_processes[key] = process
        self._launcher_processes[key] = True

    def _stop_local_process_locked(self, key: str) -> None:
        process = self._local_processes.pop(key, None)
        if process is None:
            self._launcher_processes[key] = False
            return
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait(timeout=2.0)
        self._launcher_processes[key] = False

    def _prune_local_processes_locked(self) -> None:
        for key, process in list(self._local_processes.items()):
            if process.poll() is not None:
                self._local_processes.pop(key, None)
                self._launcher_processes[key] = False

    def _send_local_launcher_command(self, command: str, map_name: Optional[str] = None) -> Dict[str, Any]:
        normalized = command.strip().lower()
        config = self._config
        with self._lock:
            self._prune_local_processes_locked()
            if normalized == "launch_robot":
                self._start_local_process_locked(
                    "robot",
                    config.robot_workspace,
                    ["ros2", "launch", config.package_name, config.robot_launch_file],
                )
                self._last_system_command = normalized
                self._launcher_message = "Robot stack launched."
            elif normalized == "launch_slam":
                self._stop_local_process_locked("nav")
                self._current_map_name = None
                self._start_local_process_locked(
                    "slam",
                    config.mapping_workspace,
                    [
                        "ros2",
                        "launch",
                        config.package_name,
                        config.central_launch_file,
                        "use_slam:=true",
                        "use_nav2:=false",
                        f"use_joystick:={_bool_arg(config.mapping_use_joystick)}",
                        f"use_rviz:={_bool_arg(config.launch_rviz)}",
                    ],
                )
                self._last_system_command = normalized
                self._launcher_message = "Mapping mode launched."
            elif normalized == "launch_nav":
                if not map_name:
                    raise RuntimeError("Select a saved map before launching navigation.")
                map_path = self._local_map_yaml_path(map_name)
                if not map_path.exists():
                    raise RuntimeError(f"Saved map '{map_name}' was not found.")
                self._stop_local_process_locked("slam")
                self._stop_local_process_locked("nav")
                self._current_map_name = map_path.stem
                self._start_local_process_locked(
                    "nav",
                    config.nav_workspace,
                    [
                        "ros2",
                        "launch",
                        config.package_name,
                        config.central_launch_file,
                        "use_slam:=false",
                        "use_nav2:=true",
                        f"use_joystick:={_bool_arg(config.nav_use_joystick)}",
                        f"use_rviz:={_bool_arg(config.launch_rviz)}",
                        f"map:={str(map_path)}",
                    ],
                )
                self._last_system_command = normalized
                self._launcher_message = f"Navigation launched with map {map_path.stem}."
            elif normalized == "kill_all":
                for key in ("nav", "slam", "robot"):
                    self._stop_local_process_locked(key)
                self._last_system_command = normalized
                self._launcher_message = "Launcher processes stopped."
            else:
                raise ValueError(f"Unsupported local launcher command: {command}")

            maps = self._refresh_local_maps_locked()
            self._last_heartbeat_at = time.time()
            return self._local_launcher_status_locked(maps=maps)

    def _local_launcher_status_locked(self, maps: Optional[List[str]] = None) -> Dict[str, Any]:
        self._prune_local_processes_locked()
        return {
            "ok": True,
            "maps": list(maps if maps is not None else self._refresh_local_maps_locked()),
            "current_map": self._current_map_name,
            "map_directory": self._maps_directory,
            "processes": dict(self._launcher_processes),
            "last_command": self._last_system_command,
            "message": self._launcher_message,
        }

    def _save_local_map(self, map_name: str) -> Dict[str, Any]:
        map_path = self._local_map_yaml_path(map_name)
        map_path.parent.mkdir(parents=True, exist_ok=True)
        output_base = map_path.with_suffix("")
        command = _ros_workspace_command(
            _expanded_path(self._config.mapping_workspace),
            ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", str(output_base)],
        )
        result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=20.0)
        if result.returncode != 0:
            raise RuntimeError(result.stdout.strip() or "Map save failed.")

        with self._lock:
            maps = self._refresh_local_maps_locked()
            self._launcher_message = f"Saved map {map_path.stem}."
            self._last_system_command = "save_map"
            return self._local_launcher_status_locked(maps=maps)

    def _delete_local_map(self, map_name: str) -> Dict[str, Any]:
        map_path = self._local_map_yaml_path(map_name)
        if not map_path.exists():
            raise RuntimeError(f"Map '{map_name}' was not found.")
        image_path = _image_path_from_map_yaml(map_path)
        if map_path.stem == self._current_map_name and self._launcher_processes.get("nav"):
            raise RuntimeError("Cannot delete the current active navigation map.")
        map_path.unlink()
        if image_path and image_path.exists():
            image_path.unlink()

        with self._lock:
            self._map_preview_cache.pop(map_path.stem, None)
            maps = self._refresh_local_maps_locked()
            self._launcher_message = f"Deleted map {map_path.stem}."
            self._last_system_command = "delete_map"
            return self._local_launcher_status_locked(maps=maps)

    def _load_local_map_preview_response(self, map_name: str) -> Dict[str, Any]:
        map_path = self._local_map_yaml_path(map_name)
        if not map_path.exists():
            raise RuntimeError(f"Map '{map_name}' was not found.")
        preview_map = _load_map_preview_from_yaml(map_path)
        with self._lock:
            self._map_preview_cache[map_path.stem] = dict(preview_map)
            maps = self._refresh_local_maps_locked()
            return {
                **self._local_launcher_status_locked(maps=maps),
                "preview_map": preview_map,
            }

    def _init_ros(self) -> None:
        rclpy = self._ros["rclpy"]
        self._context = self._ros["Context"]()
        rclpy.init(args=None, context=self._context)

        config = self._config
        node_name = f"{config.node_name}_{self.robot_id.replace('-', '_')}"
        adapter = self
        modules = self._ros

        class MissionBridgeNode(modules["Node"]):
            def __init__(self) -> None:
                super().__init__(node_name, context=adapter._context)
                adapter._navigate_client = modules["ActionClient"](
                    self,
                    modules["NavigateToPose"],
                    config.navigate_action_name,
                )
                if config.map_topic:
                    self.create_subscription(
                        modules["OccupancyGrid"],
                        config.map_topic,
                        adapter._handle_map,
                        10,
                    )
                if config.goal_pose_topic:
                    adapter._goal_pose_publisher = self.create_publisher(
                        modules["PoseStamped"],
                        config.goal_pose_topic,
                        10,
                    )
                    self.create_subscription(
                        modules["PoseStamped"],
                        config.goal_pose_topic,
                        adapter._handle_goal_pose,
                        10,
                    )
                self.create_subscription(
                    modules["PoseWithCovarianceStamped"],
                    config.localization_topic,
                    adapter._handle_localization_pose,
                    10,
                )
                self.create_subscription(
                    modules["Odometry"],
                    config.odom_topic,
                    adapter._handle_odom,
                    10,
                )
                if config.battery_topic:
                    self.create_subscription(
                        modules["BatteryState"],
                        config.battery_topic,
                        adapter._handle_battery,
                        10,
                    )
                if config.joystick_topic:
                    adapter._manual_command_publisher = self.create_publisher(
                        modules["Twist"],
                        config.joystick_topic,
                        10,
                    )
                    self.create_subscription(
                        modules["Twist"],
                        config.joystick_topic,
                        adapter._handle_joy_cmd,
                        10,
                    )
                if config.initial_pose_topic:
                    adapter._initial_pose_publisher = self.create_publisher(
                        modules["PoseWithCovarianceStamped"],
                        config.initial_pose_topic,
                        10,
                    )
                    self.create_subscription(
                        modules["PoseWithCovarianceStamped"],
                        config.initial_pose_topic,
                        adapter._handle_initial_pose,
                        10,
                    )
                if config.global_localization_service:
                    adapter._global_localization_client = self.create_client(
                        modules["Empty"],
                        config.global_localization_service,
                    )
                if config.system_command_topic:
                    adapter._system_command_publisher = self.create_publisher(
                        modules["String"],
                        config.system_command_topic,
                        10,
                    )
                if config.system_status_topic:
                    adapter._system_status_subscription = self.create_subscription(
                        modules["String"],
                        config.system_status_topic,
                        adapter._handle_system_status,
                        10,
                    )
                if config.power_mode_topic:
                    self.create_subscription(
                        modules["String"],
                        config.power_mode_topic,
                        adapter._handle_power_mode,
                        10,
                    )
                if config.power_log_topic:
                    self.create_subscription(
                        modules["String"],
                        config.power_log_topic,
                        adapter._handle_power_log,
                        10,
                    )
                if config.power_latency_topic:
                    self.create_subscription(
                        modules["Float32"],
                        config.power_latency_topic,
                        adapter._handle_power_latency,
                        10,
                    )
                if config.power_battery_percent_topic:
                    self.create_subscription(
                        modules["Float32"],
                        config.power_battery_percent_topic,
                        adapter._handle_power_battery_percent,
                        10,
                    )
                if config.power_command_topic:
                    adapter._power_command_publisher = self.create_publisher(
                        modules["String"],
                        config.power_command_topic,
                        10,
                    )
                if config.power_ping_topic:
                    adapter._power_ping_publisher = self.create_publisher(
                        modules["Float64MultiArray"],
                        config.power_ping_topic,
                        10,
                    )
                    self.create_timer(config.power_ping_period_s, adapter._publish_power_ping)

        self._node = MissionBridgeNode()
        self._executor = self._ros["SingleThreadedExecutor"](context=self._context)
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._spin_executor, daemon=True)
        self._spin_thread.start()

    def _spin_executor(self) -> None:
        try:
            self._executor.spin()
        except Exception as exc:
            if not self._shutdown_requested:
                print(f"[Ros2RobotAdapter] executor stopped unexpectedly: {exc}")

    def _handle_localization_pose(self, msg: Any) -> None:
        pose = msg.pose.pose
        now = time.time()
        with self._lock:
            self._pose = {
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "yaw": _quaternion_to_yaw(
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ),
            }
            self._localization_valid = True
            self._last_localization_at = now
            self._last_heartbeat_at = now

    def _handle_map(self, msg: Any) -> None:
        info = msg.info
        origin = info.origin
        now = time.time()
        with self._lock:
            self._map_snapshot = {
                "width": int(info.width),
                "height": int(info.height),
                "resolution": float(info.resolution),
                "origin": {
                    "x": float(origin.position.x),
                    "y": float(origin.position.y),
                    "yaw": _quaternion_to_yaw(
                        origin.orientation.x,
                        origin.orientation.y,
                        origin.orientation.z,
                        origin.orientation.w,
                    ),
                },
                "data": list(msg.data),
                "updated_at": now,
            }
            self._last_heartbeat_at = now

    def _handle_goal_pose(self, msg: Any) -> None:
        pose = msg.pose
        now = time.time()
        with self._lock:
            self._last_goal_pose = {
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "yaw": _quaternion_to_yaw(
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ),
            }
            self._last_heartbeat_at = now

    def _handle_initial_pose(self, msg: Any) -> None:
        pose = msg.pose.pose
        now = time.time()
        with self._lock:
            self._last_initial_pose = {
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "yaw": _quaternion_to_yaw(
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ),
            }
            self._last_heartbeat_at = now

    def _handle_odom(self, msg: Any) -> None:
        twist = msg.twist.twist
        pose = msg.pose.pose
        linear_speed = float(twist.linear.x)
        angular_speed = float(twist.angular.z)
        now = time.time()
        with self._lock:
            self._linear_speed = linear_speed
            self._angular_speed = angular_speed
            if (
                abs(linear_speed) >= self._config.stall_speed_epsilon
                or abs(angular_speed) >= self._config.stall_angular_speed_epsilon
            ):
                self._last_motion_at = now
            if not self._localization_valid:
                self._pose = {
                    "x": float(pose.position.x),
                    "y": float(pose.position.y),
                    "yaw": _quaternion_to_yaw(
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w,
                    ),
                }
            self._last_heartbeat_at = now

    def _handle_battery(self, msg: Any) -> None:
        now = time.time()
        with self._lock:
            self._battery_v = float(getattr(msg, "voltage", 0.0) or 0.0)
            self._last_heartbeat_at = now

    def _handle_joy_cmd(self, msg: Any) -> None:
        if abs(float(msg.linear.x)) < 1e-4 and abs(float(msg.angular.z)) < 1e-4:
            return
        now = time.time()
        with self._lock:
            self._last_joy_cmd_at = now
            self._last_heartbeat_at = now

    def _handle_power_mode(self, msg: Any) -> None:
        now = time.time()
        with self._lock:
            self._power_mode = str(getattr(msg, "data", "Unknown") or "Unknown").strip().upper()
            self._last_heartbeat_at = now

    def _handle_power_log(self, msg: Any) -> None:
        text = str(getattr(msg, "data", "") or "").strip()
        if not text:
            return
        upper = text.upper()
        now = time.time()
        with self._lock:
            self._power_recent_log = text
            if "SAFETY LOCK DEACTIVATED" in upper:
                self._power_safety_lock = False
            elif "EMERGENCY STOP" in upper or "LOCK ACTIVE" in upper:
                self._power_safety_lock = True
            self._last_heartbeat_at = now

    def _handle_power_latency(self, msg: Any) -> None:
        now = time.time()
        with self._lock:
            self._power_latency_ms = float(getattr(msg, "data", 0.0) or 0.0)
            self._last_heartbeat_at = now

    def _handle_power_battery_percent(self, msg: Any) -> None:
        now = time.time()
        with self._lock:
            self._power_battery_percent = float(getattr(msg, "data", 0.0) or 0.0)
            self._last_heartbeat_at = now

    def _handle_system_status(self, msg: Any) -> None:
        text = str(getattr(msg, "data", "") or "").strip()
        if not text:
            return
        try:
            payload = json.loads(text)
        except json.JSONDecodeError:
            with self._lock:
                self._launcher_message = text
                self._last_heartbeat_at = time.time()
            return

        now = time.time()
        with self._lock:
            self._apply_launcher_status_locked(payload)
            request_id = str(payload.get("request_id", "") or "")
            pending = self._pending_launcher_requests.get(request_id)
            if pending is not None:
                pending["response"] = payload
                pending["event"].set()
            self._last_heartbeat_at = now

    def _apply_launcher_status_locked(self, payload: Dict[str, Any]) -> None:
        maps = payload.get("maps")
        if isinstance(maps, list):
            self._saved_map_names = sorted(str(name) for name in maps if str(name).strip())

        if "current_map" in payload:
            current_map = payload.get("current_map")
            self._current_map_name = str(current_map) if current_map else None

        if "map_directory" in payload:
            map_directory = payload.get("map_directory")
            self._maps_directory = str(map_directory) if map_directory else None

        processes = payload.get("processes")
        if isinstance(processes, dict):
            self._launcher_processes = {str(key): bool(value) for key, value in processes.items()}

        if "last_command" in payload and payload.get("last_command"):
            self._last_system_command = str(payload["last_command"])

        if "message" in payload and payload.get("message"):
            self._launcher_message = str(payload["message"])

        preview_map = payload.get("preview_map")
        if isinstance(preview_map, dict) and preview_map.get("name"):
            map_name = str(preview_map["name"])
            self._map_preview_cache[map_name] = dict(preview_map)

        deleted_map = payload.get("deleted_map")
        if deleted_map:
            self._map_preview_cache.pop(str(deleted_map), None)

    def _publish_system_payload(self, payload: Dict[str, Any]) -> None:
        if self._system_command_publisher is None:
            raise RuntimeError("System command topic is not configured for this robot.")
        message = self._ros["String"]()
        message.data = json.dumps(payload)
        self._system_command_publisher.publish(message)

    def _send_launcher_request_sync(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        request_id = str(payload.get("request_id") or uuid.uuid4().hex)
        command = dict(payload)
        command["request_id"] = request_id
        event = threading.Event()

        with self._lock:
            self._pending_launcher_requests[request_id] = {"event": event}

        self._publish_system_payload(command)

        if not event.wait(timeout=self._config.launcher_request_timeout_s):
            with self._lock:
                self._pending_launcher_requests.pop(request_id, None)
            raise RuntimeError("Timed out waiting for launcher response.")

        with self._lock:
            pending = self._pending_launcher_requests.pop(request_id, None)
        if pending is None:
            raise RuntimeError("Launcher response was lost.")

        response = pending.get("response")
        if not isinstance(response, dict):
            raise RuntimeError("Launcher returned an invalid response.")
        if not bool(response.get("ok", False)):
            raise RuntimeError(str(response.get("message") or "Launcher request failed."))
        return response

    def _publish_power_ping(self) -> None:
        if self._power_ping_publisher is None or self._shutdown_requested:
            return
        message = self._ros["Float64MultiArray"]()
        message.data = [time.time()]
        self._power_ping_publisher.publish(message)

    def _handle_nav_feedback(self, _feedback_msg: Any) -> None:
        with self._lock:
            self._last_heartbeat_at = time.time()

    def _handle_goal_result(self, future: Any) -> None:
        try:
            result = future.result()
            status = int(result.status)
            error: Optional[str] = None
        except Exception as exc:
            status = self._goal_status_unknown
            error = str(exc)
        with self._lock:
            self._goal_result_status = status
            self._goal_result_error = error
            self._active_goal_handle = None
            self._last_heartbeat_at = time.time()
        self._goal_done_event.set()

    async def start_mission(self, mission_id: str, plan: List[str]) -> None:
        with self._lock:
            if self._state not in (MissionState.IDLE, MissionState.COMPLETED):
                raise RuntimeError("Robot already executing a mission.")
            if not plan:
                raise RuntimeError("Mission plan is empty.")
            self._current_mission_id = mission_id
            self._current_plan = list(plan)
            self._current_leg_index = 0
            self._current_destination = None
            self._current_goal_pose = None
            self._last_outcome = None
            self._cancel_requested = False
            self._pause_requested = False
            self._goal_result_status = None
            self._goal_result_error = None
            self._goal_done_event.clear()
            self._state = MissionState.EN_ROUTE
            self._resume_event.set()

            if self._mission_thread and self._mission_thread.is_alive():
                raise RuntimeError("Mission worker is still active.")

            self._mission_thread = threading.Thread(
                target=self._run_plan,
                args=(mission_id, list(plan)),
                daemon=True,
            )
            self._mission_thread.start()

    def _run_plan(self, mission_id: str, plan: List[str]) -> None:
        try:
            for index, destination in enumerate(plan):
                with self._lock:
                    self._current_leg_index = index
                    self._current_destination = destination

                while not self._shutdown_requested:
                    if self._cancel_requested:
                        self._mark_completed_locked(MissionOutcome.CANCELED)
                        return

                    if not self._resume_event.wait(timeout=0.1):
                        continue

                    status = self._send_goal_and_wait(destination)
                    if status == "paused":
                        continue
                    if status == "succeeded":
                        break
                    if status == "canceled":
                        self._mark_completed_locked(MissionOutcome.CANCELED)
                        return
                    if status == "aborted":
                        self._mark_completed_locked(MissionOutcome.ABORTED)
                        return
                    self._mark_completed_locked(MissionOutcome.FAILED)
                    return

            with self._lock:
                if self._current_mission_id == mission_id:
                    self._state = MissionState.COMPLETED
                    self._last_outcome = MissionOutcome.SUCCESS
        except Exception as exc:
            print(f"[Ros2RobotAdapter] mission worker error: {exc}")
            self._mark_completed_locked(MissionOutcome.FAILED)

    def _send_goal_and_wait(self, destination_name: str) -> str:
        self._dispatch_goal(destination_name)

        while not self._shutdown_requested:
            if self._cancel_requested or self._pause_requested:
                self._cancel_active_goal()

            if self._goal_done_event.wait(timeout=0.1):
                break

        with self._lock:
            status = self._goal_result_status
            self._goal_result_status = None
            self._goal_done_event.clear()
            paused = self._pause_requested
            canceled = self._cancel_requested

        if canceled:
            return "canceled"
        if paused and status == self._goal_status_canceled:
            return "paused"
        if status == self._goal_status_succeeded:
            return "succeeded"
        if status == self._goal_status_aborted:
            return "aborted"
        if status == self._goal_status_canceled:
            return "canceled"
        return "failed"

    def _dispatch_goal(self, destination_name: str) -> None:
        if self._navigate_client is None or self._node is None:
            raise RuntimeError("ROS 2 navigation client is not initialized.")
        if not self._navigate_client.wait_for_server(timeout_sec=self._config.action_server_timeout_s):
            raise RuntimeError("NavigateToPose action server is unavailable.")

        goal_msg, goal_pose, pose_msg = self._build_goal(destination_name)
        send_done = threading.Event()
        goal_response: Dict[str, Any] = {}

        future = self._navigate_client.send_goal_async(goal_msg, feedback_callback=self._handle_nav_feedback)

        def _on_goal_response(done_future: Any) -> None:
            try:
                goal_response["goal_handle"] = done_future.result()
            except Exception as exc:
                goal_response["error"] = exc
            finally:
                send_done.set()

        future.add_done_callback(_on_goal_response)
        while not send_done.wait(timeout=0.1):
            if self._shutdown_requested:
                raise RuntimeError("ROS 2 adapter is shutting down.")

        if "error" in goal_response:
            raise RuntimeError(f"Failed to send goal to Nav2: {goal_response['error']}")

        goal_handle = goal_response.get("goal_handle")
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"Nav2 rejected destination '{destination_name}'.")

        with self._lock:
            self._active_goal_handle = goal_handle
            self._goal_result_status = None
            self._goal_result_error = None
            self._goal_active_since = time.time()
            self._last_motion_at = self._goal_active_since
            self._current_goal_pose = goal_pose
            self._last_goal_pose = dict(goal_pose)
            self._state = MissionState.EN_ROUTE
            self._last_heartbeat_at = self._goal_active_since

        if self._goal_pose_publisher is not None:
            self._goal_pose_publisher.publish(pose_msg)

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._handle_goal_result)

    def _build_goal(self, destination_name: str) -> Any:
        destinations, _ = self._dest_config.get()
        destination = destinations.get(destination_name)
        if destination is None:
            raise RuntimeError(f"Unknown destination '{destination_name}'.")

        pose = destination.pose
        x = float(pose.get("x", 0.0))
        y = float(pose.get("y", 0.0))
        yaw = float(pose.get("yaw", 0.0))
        qz, qw = _yaw_to_quaternion(yaw)

        pose_msg = self._ros["PoseStamped"]()
        pose_msg.header.frame_id = self._config.map_frame
        pose_msg.header.stamp = self._node.get_clock().now().to_msg()
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        goal_msg = self._ros["NavigateToPose"].Goal()
        goal_msg.pose = pose_msg
        return goal_msg, {"x": x, "y": y, "yaw": yaw}, pose_msg

    async def pause(self) -> None:
        with self._lock:
            if self._state == MissionState.COMPLETED:
                return
            self._pause_requested = True
            self._state = MissionState.PAUSED
            self._resume_event.clear()
        self._cancel_active_goal()

    async def resume(self) -> None:
        with self._lock:
            if self._state == MissionState.COMPLETED:
                return
            self._pause_requested = False
            self._state = MissionState.EN_ROUTE
            self._resume_event.set()

    async def cancel(self) -> None:
        with self._lock:
            self._cancel_requested = True
            self._pause_requested = False
            self._resume_event.set()
            self._state = MissionState.COMPLETED
            self._last_outcome = MissionOutcome.CANCELED
        self._cancel_active_goal()

    def _cancel_active_goal(self) -> None:
        with self._lock:
            if self._cancel_future_in_flight or self._active_goal_handle is None:
                return
            goal_handle = self._active_goal_handle
            self._cancel_future_in_flight = True

        cancel_future = goal_handle.cancel_goal_async()

        def _clear_cancel_flag(_done_future: Any) -> None:
            with self._lock:
                self._cancel_future_in_flight = False

        cancel_future.add_done_callback(_clear_cancel_flag)

    def _mark_completed_locked(self, outcome: MissionOutcome) -> None:
        with self._lock:
            self._state = MissionState.COMPLETED
            self._last_outcome = outcome

    async def reset_to_idle(self) -> None:
        with self._lock:
            self._cancel_requested = True
            self._pause_requested = False
            self._resume_event.set()
        self._cancel_active_goal()

        if (
            self._mission_thread
            and self._mission_thread.is_alive()
            and threading.current_thread() is not self._mission_thread
        ):
            self._mission_thread.join(timeout=1.0)

        with self._lock:
            self._state = MissionState.IDLE
            self._current_mission_id = None
            self._current_plan = []
            self._current_leg_index = 0
            self._current_destination = None
            self._current_goal_pose = None
            self._last_outcome = None
            self._cancel_requested = False
            self._pause_requested = False
            self._goal_result_status = None
            self._goal_result_error = None
            self._goal_done_event.clear()
            self._active_goal_handle = None

    def snapshot(self) -> RobotTelemetry:
        with self._lock:
            now = time.time()
            self._mode = self._compute_mode_locked(now)
            self._connection_ok = self._compute_connection_ok_locked(now)
            self._localization_valid = self._compute_localization_ok_locked(now)
            self._blocked = self._compute_blocked_locked(now, self._mode)
            self._obstacle_stop = self._blocked
            return RobotTelemetry(
                robot_id=self.robot_id,
                state=self._state,
                mode=self._mode,
                current_mission_id=self._current_mission_id,
                last_heartbeat_at=self._last_heartbeat_at,
                connection_ok=self._connection_ok,
                localization_valid=self._localization_valid,
                obstacle_stop=self._obstacle_stop,
                blocked=self._blocked,
                battery_v=self._battery_v,
                pose=dict(self._pose),
                outcome=self._last_outcome,
            )

    async def set_power_mode(self, mode: str) -> None:
        command = mode.strip().upper()
        if command not in {"AUTO", "MANUAL", "RESET", "STOP", "ON", "OFF"}:
            raise ValueError(f"Unsupported power mode: {mode}")

        with self._lock:
            if command in {"RESET", "ON", "AUTO", "MANUAL"}:
                self._power_safety_lock = False
                self._power_mode = "ON"
                self._mode = RobotMode.AUTO
                self._power_recent_log = "> Robot enabled."
            elif command in {"STOP", "OFF"}:
                self._power_safety_lock = True
                self._pause_requested = True
                self._resume_event.clear()
                self._cancel_active_goal()
                self._power_mode = "OFF"
                self._power_recent_log = "> Robot stopped."

        if self._power_command_publisher is not None:
            message = self._ros["String"]()
            message.data = command
            self._power_command_publisher.publish(message)

    async def send_manual_drive_command(self, linear: float, angular: float) -> None:
        if self._manual_command_publisher is None:
            raise RuntimeError("Manual drive topic is not configured for this robot.")

        linear = _clamp(float(linear), -1.0, 1.0)
        angular = _clamp(float(angular), -1.0, 1.0)

        with self._lock:
            if self._power_safety_lock or self._power_mode in {"STOP", "OFF"}:
                raise RuntimeError("Robot is off. Turn it on before manual driving.")

        message = self._ros["Twist"]()
        message.linear.x = linear
        message.linear.y = 0.0
        message.linear.z = 0.0
        message.angular.x = 0.0
        message.angular.y = 0.0
        message.angular.z = angular
        self._manual_command_publisher.publish(message)

        with self._lock:
            self._last_heartbeat_at = time.time()
            if abs(linear) >= 1e-4 or abs(angular) >= 1e-4:
                self._last_joy_cmd_at = self._last_heartbeat_at
                self._power_recent_log = f"> Manual drive command: linear={linear:.2f}, angular={angular:.2f}"
            else:
                self._power_recent_log = "> Manual drive stopped."

    async def localize(self) -> Dict[str, Any]:
        if self._manual_command_publisher is None:
            raise RuntimeError("Manual drive topic is not configured for this robot.")
        if self._global_localization_client is None:
            raise RuntimeError("AMCL global localization service is not configured for this robot.")

        return await asyncio.to_thread(self._run_global_localization_spin)

    def _run_global_localization_spin(self) -> Dict[str, Any]:
        client = self._global_localization_client
        if client is None:
            raise RuntimeError("AMCL global localization service is not configured for this robot.")

        with self._lock:
            if self._power_safety_lock or self._power_mode in {"STOP", "OFF"}:
                raise RuntimeError("Robot is off. Turn it on before localizing.")
            self._power_recent_log = "> Starting global localization spin."

        if not client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError("AMCL global localization service is unavailable.")

        request = self._ros["Empty"].Request()
        future = client.call_async(request)
        done = threading.Event()
        response: Dict[str, Any] = {}

        def _on_done(done_future: Any) -> None:
            try:
                response["result"] = done_future.result()
            except Exception as exc:
                response["error"] = exc
            finally:
                done.set()

        future.add_done_callback(_on_done)
        if not done.wait(timeout=3.0):
            raise RuntimeError("Timed out calling AMCL global localization.")
        if "error" in response:
            raise RuntimeError(f"AMCL global localization failed: {response['error']}")

        angular = _clamp(
            self._config.localization_spin_angular_z,
            -1.0,
            1.0,
        )
        duration_s = max(0.0, float(self._config.localization_spin_duration_s))
        rate_hz = max(1.0, float(self._config.localization_spin_rate_hz))
        period_s = 1.0 / rate_hz
        deadline = time.time() + duration_s

        try:
            while time.time() < deadline and not self._shutdown_requested:
                self._publish_manual_twist(0.0, angular)
                time.sleep(period_s)
        finally:
            self._publish_manual_twist(0.0, 0.0)

        with self._lock:
            self._last_heartbeat_at = time.time()
            self._last_joy_cmd_at = 0.0
            self._power_recent_log = "> Global localization spin complete."

        return {
            "robot_id": self.robot_id,
            "ok": True,
            "message": "Global localization spin complete.",
            "duration_s": duration_s,
            "angular_z": angular,
        }

    def _publish_manual_twist(self, linear: float, angular: float) -> None:
        if self._manual_command_publisher is None:
            raise RuntimeError("Manual drive topic is not configured for this robot.")
        message = self._ros["Twist"]()
        message.linear.x = float(linear)
        message.linear.y = 0.0
        message.linear.z = 0.0
        message.angular.x = 0.0
        message.angular.y = 0.0
        message.angular.z = float(angular)
        self._manual_command_publisher.publish(message)

    async def send_system_command(self, command: str, map_name: Optional[str] = None) -> None:
        normalized = command.strip().lower()
        if normalized not in {"launch_robot", "launch_slam", "launch_nav", "save_map", "kill_all"}:
            raise ValueError(f"Unsupported system command: {command}")
        if self._local_launcher_enabled():
            response = await asyncio.to_thread(self._send_local_launcher_command, normalized, map_name)
            with self._lock:
                self._apply_launcher_status_locked(response)
                self._power_recent_log = str(response.get("message") or f"> System command sent: {normalized}")
            return
        if normalized == "launch_nav":
            if not map_name:
                raise RuntimeError("Select a saved map before launching navigation.")
            response = await asyncio.to_thread(
                self._send_launcher_request_sync,
                {"action": normalized, "map_name": map_name},
            )
            with self._lock:
                self._apply_launcher_status_locked(response)
                self._last_system_command = normalized
                self._power_recent_log = f"> Navigation launched with map: {map_name}"
                self._last_heartbeat_at = time.time()
            return

        self._publish_system_payload({"action": normalized})

        with self._lock:
            self._last_system_command = normalized
            self._power_recent_log = f"> System command sent: {normalized}"
            self._last_heartbeat_at = time.time()

    async def set_initial_pose(self, x: float, y: float, yaw: float) -> None:
        if self._initial_pose_publisher is None or self._node is None:
            raise RuntimeError("Initial pose topic is not configured for this robot.")

        qz, qw = _yaw_to_quaternion(float(yaw))
        message = self._ros["PoseWithCovarianceStamped"]()
        message.header.frame_id = self._config.map_frame
        message.header.stamp = self._node.get_clock().now().to_msg()
        message.pose.pose.position.x = float(x)
        message.pose.pose.position.y = float(y)
        message.pose.pose.position.z = 0.0
        message.pose.pose.orientation.x = 0.0
        message.pose.pose.orientation.y = 0.0
        message.pose.pose.orientation.z = qz
        message.pose.pose.orientation.w = qw
        message.pose.covariance = [0.0] * 36

        self._initial_pose_publisher.publish(message)

        with self._lock:
            self._last_initial_pose = {"x": float(x), "y": float(y), "yaw": float(yaw)}
            self._power_recent_log = f"> Initial pose sent: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
            self._last_heartbeat_at = time.time()

    async def set_goal_pose(self, x: float, y: float, yaw: float) -> None:
        if self._goal_pose_publisher is None or self._node is None:
            raise RuntimeError("Goal pose topic is not configured for this robot.")

        qz, qw = _yaw_to_quaternion(float(yaw))
        message = self._ros["PoseStamped"]()
        message.header.frame_id = self._config.map_frame
        message.header.stamp = self._node.get_clock().now().to_msg()
        message.pose.position.x = float(x)
        message.pose.position.y = float(y)
        message.pose.position.z = 0.0
        message.pose.orientation.x = 0.0
        message.pose.orientation.y = 0.0
        message.pose.orientation.z = qz
        message.pose.orientation.w = qw
        self._goal_pose_publisher.publish(message)

        with self._lock:
            self._last_goal_pose = {"x": float(x), "y": float(y), "yaw": float(yaw)}
            self._power_recent_log = f"> Goal pose sent: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
            self._last_heartbeat_at = time.time()

    async def save_map(self, map_name: str) -> Dict[str, Any]:
        if self._local_launcher_enabled():
            response = await asyncio.to_thread(self._save_local_map, map_name)
            with self._lock:
                self._apply_launcher_status_locked(response)
                self._power_recent_log = f"> Saved map: {map_name}"
                self._last_heartbeat_at = time.time()
            return response
        response = await asyncio.to_thread(
            self._send_launcher_request_sync,
            {"action": "save_map", "map_name": map_name},
        )
        with self._lock:
            self._apply_launcher_status_locked(response)
            self._power_recent_log = f"> Saved map: {map_name}"
            self._last_heartbeat_at = time.time()
        return response

    async def delete_map(self, map_name: str) -> Dict[str, Any]:
        if self._local_launcher_enabled():
            response = await asyncio.to_thread(self._delete_local_map, map_name)
            with self._lock:
                self._apply_launcher_status_locked(response)
                self._power_recent_log = f"> Deleted map: {map_name}"
                self._last_heartbeat_at = time.time()
            return response
        response = await asyncio.to_thread(
            self._send_launcher_request_sync,
            {"action": "delete_map", "map_name": map_name},
        )
        with self._lock:
            self._apply_launcher_status_locked(response)
            self._power_recent_log = f"> Deleted map: {map_name}"
            self._last_heartbeat_at = time.time()
        return response

    async def load_map_preview(self, map_name: str) -> Dict[str, Any]:
        if self._local_launcher_enabled():
            response = await asyncio.to_thread(self._load_local_map_preview_response, map_name)
            preview_map = response.get("preview_map")
            if not isinstance(preview_map, dict):
                raise RuntimeError("Map preview data is unavailable.")
            with self._lock:
                self._apply_launcher_status_locked(response)
                self._last_heartbeat_at = time.time()
            return dict(preview_map)
        response = await asyncio.to_thread(
            self._send_launcher_request_sync,
            {"action": "load_map_preview", "map_name": map_name},
        )
        preview_map = response.get("preview_map")
        if not isinstance(preview_map, dict):
            raise RuntimeError("Launcher did not return map preview data.")
        with self._lock:
            self._apply_launcher_status_locked(response)
            self._last_heartbeat_at = time.time()
        return dict(preview_map)

    def operator_snapshot(self) -> Dict[str, Any]:
        with self._lock:
            if self._local_launcher_enabled():
                self._prune_local_processes_locked()
                self._refresh_local_maps_locked()
            map_snapshot = None
            if self._map_snapshot is not None:
                map_snapshot = {
                    "width": self._map_snapshot["width"],
                    "height": self._map_snapshot["height"],
                    "resolution": self._map_snapshot["resolution"],
                    "origin": dict(self._map_snapshot["origin"]),
                    "data": list(self._map_snapshot["data"]),
                    "updated_at": self._map_snapshot["updated_at"],
                }

            return {
                "map_available": map_snapshot is not None,
                "map": map_snapshot,
                "goal_pose": dict(self._current_goal_pose) if self._current_goal_pose is not None else (
                    dict(self._last_goal_pose) if self._last_goal_pose is not None else None
                ),
                "initial_pose": dict(self._last_initial_pose) if self._last_initial_pose is not None else None,
                "system_commands_available": self._local_launcher_enabled() or self._system_command_publisher is not None,
                "initial_pose_available": self._initial_pose_publisher is not None,
                "goal_pose_available": self._goal_pose_publisher is not None,
                "last_system_command": self._last_system_command,
                "saved_maps": list(self._saved_map_names),
                "current_map_name": self._current_map_name,
                "maps_directory": self._maps_directory,
                "launcher_message": self._launcher_message,
                "launcher_processes": dict(self._launcher_processes),
            }

    def power_snapshot(self) -> RobotPowerStatus:
        with self._lock:
            return RobotPowerStatus(
                available=True,
                mode=self._power_mode,
                battery_percent=self._power_battery_percent,
                latency_ms=self._power_latency_ms,
                safety_lock=self._power_safety_lock,
                recent_log=self._power_recent_log,
            )

    def _compute_mode_locked(self, now: float) -> RobotMode:
        if self._power_mode in {"MANUAL", "STOP", "OFF"}:
            return RobotMode.MANUAL_OVERRIDE
        if self._config.joystick_topic and (now - self._last_joy_cmd_at) <= self._config.manual_override_timeout_s:
            return RobotMode.MANUAL_OVERRIDE
        return RobotMode.AUTO

    def _compute_connection_ok_locked(self, now: float) -> bool:
        return (now - self._last_heartbeat_at) <= self._config.connection_timeout_s

    def _compute_localization_ok_locked(self, now: float) -> bool:
        if not self._localization_valid:
            return False
        return (now - self._last_localization_at) <= self._config.localization_timeout_s

    def _compute_blocked_locked(self, now: float, mode: RobotMode) -> bool:
        if mode == RobotMode.MANUAL_OVERRIDE:
            return False
        if self._state != MissionState.EN_ROUTE:
            return False
        if self._pause_requested or self._cancel_requested:
            return False
        if self._current_goal_pose is None:
            return False
        distance_to_goal = math.hypot(
            self._current_goal_pose["x"] - self._pose.get("x", 0.0),
            self._current_goal_pose["y"] - self._pose.get("y", 0.0),
        )
        if distance_to_goal <= self._config.goal_tolerance_m:
            return False
        moving = (
            abs(self._linear_speed) >= self._config.stall_speed_epsilon
            or abs(self._angular_speed) >= self._config.stall_angular_speed_epsilon
        )
        if moving:
            return False
        if (now - self._goal_active_since) < self._config.stall_detect_after_s:
            return False
        return (now - self._last_motion_at) >= self._config.stall_detect_after_s

    def shutdown(self) -> None:
        if self._shutdown_requested:
            return

        self._shutdown_requested = True
        self._resume_event.set()
        self._cancel_active_goal()
        if self._local_launcher_enabled():
            with self._lock:
                for key in list(self._local_processes):
                    self._stop_local_process_locked(key)

        if self._mission_thread and self._mission_thread.is_alive():
            self._mission_thread.join(timeout=1.0)

        if self._executor is not None:
            self._executor.shutdown()
        if self._node is not None:
            self._node.destroy_node()
        if self._context is not None:
            self._ros["rclpy"].shutdown(context=self._context)
        if self._spin_thread and self._spin_thread.is_alive():
            self._spin_thread.join(timeout=1.0)


def _expanded_path(value: str) -> Path:
    return Path(os.path.expandvars(os.path.expanduser(value))).resolve()


def _bool_arg(value: bool) -> str:
    return "true" if bool(value) else "false"


def _ros_workspace_command(workspace: Path, ros_args: List[str]) -> List[str]:
    setup_file = workspace / "install" / "setup.bash"
    ros_command = " ".join(shlex.quote(str(part)) for part in ros_args)
    if setup_file.exists():
        return ["bash", "-lc", f"source {shlex.quote(str(setup_file))} && exec {ros_command}"]
    return ["bash", "-lc", f"exec {ros_command}"]


def _image_path_from_map_yaml(map_yaml_path: Path) -> Optional[Path]:
    raw = yaml.safe_load(map_yaml_path.read_text(encoding="utf-8")) or {}
    image = raw.get("image")
    if not image:
        return None
    image_path = Path(str(image))
    if not image_path.is_absolute():
        image_path = map_yaml_path.parent / image_path
    return image_path


def _load_map_preview_from_yaml(map_yaml_path: Path) -> Dict[str, Any]:
    raw = yaml.safe_load(map_yaml_path.read_text(encoding="utf-8")) or {}
    image_path = _image_path_from_map_yaml(map_yaml_path)
    if image_path is None or not image_path.exists():
        raise RuntimeError(f"Map image for '{map_yaml_path.stem}' was not found.")

    width, height, max_value, pixels = _read_pgm(image_path)
    negate = int(raw.get("negate", 0) or 0)
    occupied_thresh = float(raw.get("occupied_thresh", 0.65))
    free_thresh = float(raw.get("free_thresh", 0.25))
    origin_values = list(raw.get("origin", [0.0, 0.0, 0.0]))
    while len(origin_values) < 3:
        origin_values.append(0.0)

    occupancy = []
    scale = float(max_value) if max_value else 255.0
    for pixel in pixels:
        probability = (float(pixel) / scale) if negate else ((scale - float(pixel)) / scale)
        if probability > occupied_thresh:
            occupancy.append(100)
        elif probability < free_thresh:
            occupancy.append(0)
        else:
            occupancy.append(-1)

    return {
        "name": map_yaml_path.stem,
        "width": width,
        "height": height,
        "resolution": float(raw.get("resolution", 0.05)),
        "origin": {
            "x": float(origin_values[0]),
            "y": float(origin_values[1]),
            "yaw": float(origin_values[2]),
        },
        "data": occupancy,
        "updated_at": map_yaml_path.stat().st_mtime,
    }


def _read_pgm(path: Path) -> tuple[int, int, int, List[int]]:
    data = path.read_bytes()
    index = 0

    def next_token() -> bytes:
        nonlocal index
        while index < len(data):
            byte = data[index]
            if byte in b" \t\r\n":
                index += 1
                continue
            if byte == ord("#"):
                while index < len(data) and data[index] not in b"\r\n":
                    index += 1
                continue
            break
        start = index
        while index < len(data) and data[index] not in b" \t\r\n":
            index += 1
        if start == index:
            raise RuntimeError(f"Invalid PGM file: {path}")
        return data[start:index]

    magic = next_token()
    width = int(next_token())
    height = int(next_token())
    max_value = int(next_token())
    expected = width * height

    if magic == b"P5":
        if index < len(data) and data[index] in b" \t\r\n":
            index += 1
        payload = data[index:index + expected]
        if len(payload) != expected:
            raise RuntimeError(f"PGM image has unexpected size: {path}")
        return width, height, max_value, list(payload)

    if magic == b"P2":
        pixels = [int(next_token()) for _ in range(expected)]
        return width, height, max_value, pixels

    raise RuntimeError(f"Unsupported PGM format '{magic.decode(errors='replace')}' in {path}")


def create_robot_adapter_from_env(dest_config: DestinationConfig) -> RobotAdapter:
    backend = os.getenv("MISSION_CONTROL_ROBOT_BACKEND", "sim").strip().lower()
    robot_id = os.getenv("MISSION_CONTROL_ROBOT_ID", "robot-1").strip() or "robot-1"

    if backend == "sim":
        speed_scale = _env_float("MISSION_CONTROL_SIM_SPEED_SCALE", 1.0)
        return SimRobotAdapter(robot_id, speed_scale=speed_scale)
    if backend == "ros2":
        return Ros2RobotAdapter(robot_id=robot_id, dest_config=dest_config, config=Ros2AdapterConfig.from_env())

    raise ValueError("MISSION_CONTROL_ROBOT_BACKEND must be 'sim' or 'ros2'.")


def _env_float(name: str, default: float) -> float:
    raw = os.getenv(name)
    if raw is None or raw.strip() == "":
        return float(default)
    return float(raw)


def _env_optional_str(name: str, default: Optional[str]) -> Optional[str]:
    raw = os.getenv(name)
    if raw is None:
        return default
    value = raw.strip()
    return value or None


def _env_bool(name: str, default: bool) -> bool:
    raw = os.getenv(name)
    if raw is None or raw.strip() == "":
        return bool(default)
    return raw.strip().lower() in {"1", "true", "yes", "on"}


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _yaw_to_quaternion(yaw: float) -> tuple[float, float]:
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def _quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _import_ros2_modules() -> Dict[str, Any]:
    try:
        import rclpy
        from action_msgs.msg import GoalStatus
        from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
        from nav2_msgs.action import NavigateToPose
        from nav_msgs.msg import OccupancyGrid, Odometry
        from rclpy.action import ActionClient
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from sensor_msgs.msg import BatteryState
        from std_msgs.msg import Float32, Float64MultiArray, String
        from std_srvs.srv import Empty
    except ImportError as exc:
        raise RuntimeError(
            "ROS 2 Python packages are unavailable. Source your ROS 2 environment "
            "before starting mission_control with MISSION_CONTROL_ROBOT_BACKEND=ros2."
        ) from exc

    return {
        "rclpy": rclpy,
        "ActionClient": ActionClient,
        "BatteryState": BatteryState,
        "Context": Context,
        "Empty": Empty,
        "Float32": Float32,
        "Float64MultiArray": Float64MultiArray,
        "GoalStatus": GoalStatus,
        "NavigateToPose": NavigateToPose,
        "Node": Node,
        "OccupancyGrid": OccupancyGrid,
        "Odometry": Odometry,
        "PoseStamped": PoseStamped,
        "PoseWithCovarianceStamped": PoseWithCovarianceStamped,
        "SingleThreadedExecutor": SingleThreadedExecutor,
        "String": String,
        "Twist": Twist,
    }
