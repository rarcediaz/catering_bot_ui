"""Educational tests for the Mission Control PoC package.

Purpose:
- Demonstrate how to use each Python file in `mission_control/`.
- Act as executable documentation: read tests from top to bottom.

Run from `mission_control_poc/`:
    python -m unittest -v tests.test_mission_control_educational
"""

from __future__ import annotations

import asyncio
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from pydantic import ValidationError


# Ensure `import mission_control...` works whether tests are run from repo root
# or from `mission_control_poc/`.
PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import mission_control  # noqa: E402
from mission_control.api_models import (  # noqa: E402
    CreateMissionRequest,
    MissionCommandRequest,
    RobotGoalPoseRequest,
    RobotInitialPoseRequest,
    RobotMapDeleteRequest,
    RobotMapSaveRequest,
    RobotManualDriveRequest,
    RobotSystemCommandRequest,
    RobotTelemetryIn,
    TempDestinationRequest,
)
from mission_control.config_loader import DestinationConfig  # noqa: E402
from mission_control.robot_adapter import SimRobotAdapter, _load_map_preview_from_yaml  # noqa: E402
from mission_control.scheduler import MissionControl, MissionCreate  # noqa: E402
from mission_control.storage import Storage  # noqa: E402
from mission_control.types import (  # noqa: E402
    CommandSource,
    MissionOutcome,
    MissionState,
    RobotMode,
)


def write_demo_destinations(path: Path) -> None:
    """Create a minimal YAML config for tests and examples."""
    path.write_text(
        (
            "destinations:\n"
            '  - name: "Storage"\n'
            "    pose: {x: 0.0, y: 0.0, yaw: 0.0}\n"
            '  - name: "Hall_A"\n'
            "    pose: {x: 5.2, y: 1.1, yaw: 1.57}\n"
            '  - name: "Ballroom"\n'
            "    pose: {x: 12.4, y: -3.0, yaw: 3.14}\n"
            'home_destination: "Storage"\n'
        ),
        encoding="utf-8",
    )


class Test00PackageAndTypes(unittest.TestCase):
    """Covers: `mission_control/__init__.py` and `mission_control/types.py`."""

    def test_package_import_and_shared_types(self) -> None:
        # __init__.py: package import should succeed.
        self.assertTrue(hasattr(mission_control, "__package__"))

        # types.py: enums + CommandSource are used throughout API and scheduler.
        self.assertEqual(MissionState.REQUESTED.value, "Requested")
        self.assertEqual(MissionState.IDLE.value, "Idle")
        self.assertEqual(MissionState.EN_ROUTE.value, "En-route")
        self.assertEqual(MissionState.WAITING_FOR_RETURN.value, "WaitingForReturn")
        self.assertEqual(MissionState.RETURNING.value, "Returning")
        self.assertEqual(RobotMode.MANUAL_OVERRIDE.value, "ManualOverride")

        source = CommandSource(source_type="user", source_id="tablet-1", meta={"screen": "dispatch"})
        self.assertEqual(
            source.to_dict(),
            {"type": "user", "id": "tablet-1", "meta": {"screen": "dispatch"}},
        )


class Test01ConfigLoader(unittest.TestCase):
    """Covers: `mission_control/config_loader.py`."""

    def setUp(self) -> None:
        self.tmpdir = tempfile.TemporaryDirectory()
        self.config_path = Path(self.tmpdir.name) / "destinations.yaml"
        write_demo_destinations(self.config_path)

    def tearDown(self) -> None:
        self.tmpdir.cleanup()

    def test_load_validate_and_home_destination(self) -> None:
        # Usage pattern: create loader -> load YAML -> validate user inputs.
        config = DestinationConfig(self.config_path)
        destinations, home = config.load()

        self.assertIn("Hall_A", destinations)
        self.assertTrue(config.validate("Ballroom"))
        self.assertFalse(config.validate("UnknownRoom"))
        self.assertEqual(home, "Storage")
        self.assertEqual(config.home(), "Storage")
        self.assertEqual(len(config.list()), 3)

    def test_upsert_destination_overwrites_existing_entry(self) -> None:
        config = DestinationConfig(self.config_path)
        config.load()

        destination = config.upsert_destination(
            "Temp Destination",
            {"x": 1.25, "y": -0.75, "yaw": 0.5},
            notes="Updated from map panel",
        )

        self.assertEqual(destination.pose["x"], 1.25)
        self.assertTrue(config.validate("Temp Destination"))
        self.assertEqual(config.list()[-1].name, "Temp Destination")


class Test02Storage(unittest.TestCase):
    """Covers: `mission_control/storage.py`."""

    def setUp(self) -> None:
        self.tmpdir = tempfile.TemporaryDirectory()
        self.db_path = Path(self.tmpdir.name) / "mission_control.sqlite3"
        self.storage = Storage(self.db_path)
        self.storage.init()

    def tearDown(self) -> None:
        self.tmpdir.cleanup()

    def test_mission_crud_and_event_timeline(self) -> None:
        now = time.time()
        mission = {
            "id": "mission-edu-1",
            "created_at": now,
            "requested_by": "student",
            "command_source": '{"type":"user","id":"student"}',
            "from_dest": None,
            "to_dest": "Hall_A",
            "schedule_type": "single",
            "state": MissionState.IDLE.value,
            "assigned_robot_id": None,
            "started_at": None,
            "completed_at": None,
            "outcome": MissionOutcome.NONE.value,
            "retries": 0,
            "help_required": 0,
            "last_update_at": now,
            "notes": "educational test",
        }

        # create/get/update/list are the core storage operations.
        self.storage.create_mission(mission)
        loaded = self.storage.get_mission("mission-edu-1")
        self.assertIsNotNone(loaded)
        self.assertEqual(loaded["to_dest"], "Hall_A")

        self.storage.update_mission("mission-edu-1", state=MissionState.EN_ROUTE.value, assigned_robot_id="robot-1")
        loaded = self.storage.get_mission("mission-edu-1")
        self.assertEqual(loaded["state"], MissionState.EN_ROUTE.value)
        self.assertEqual(loaded["assigned_robot_id"], "robot-1")

        # mission_events is your audit trail.
        self.storage.append_event("mission-edu-1", "dispatched", {"robot_id": "robot-1"})
        events = self.storage.list_events("mission-edu-1")
        self.assertEqual(events[0]["event"], "dispatched")
        self.assertEqual(events[0]["details"]["robot_id"], "robot-1")

    def test_robot_upsert(self) -> None:
        # upsert_robot inserts first, then updates on later calls.
        self.storage.upsert_robot(
            "robot-1",
            state=MissionState.IDLE.value,
            mode=RobotMode.AUTO.value,
            current_mission_id=None,
            last_heartbeat_at=time.time(),
            connection_ok=1,
            localization_valid=1,
            obstacle_stop=0,
            blocked=0,
            battery_v=24.0,
            x=0.0,
            y=0.0,
            yaw=0.0,
        )
        self.storage.upsert_robot("robot-1", blocked=1, battery_v=23.4)
        robot = self.storage.get_robot("robot-1")
        self.assertEqual(robot["blocked"], 1)
        self.assertEqual(robot["battery_v"], 23.4)

    def test_delete_completed_and_all_missions(self) -> None:
        now = time.time()
        completed = {
            "id": "mission-complete-1",
            "created_at": now,
            "requested_by": "student",
            "command_source": '{"type":"user","id":"student"}',
            "from_dest": None,
            "to_dest": "Hall_A",
            "schedule_type": "single",
            "state": MissionState.COMPLETED.value,
            "assigned_robot_id": "robot-1",
            "started_at": now,
            "completed_at": now,
            "outcome": MissionOutcome.SUCCESS.value,
            "retries": 0,
            "help_required": 0,
            "last_update_at": now,
            "notes": "",
        }
        queued = {
            "id": "mission-queued-1",
            "created_at": now + 1,
            "requested_by": "student",
            "command_source": '{"type":"user","id":"student"}',
            "from_dest": None,
            "to_dest": "Ballroom",
            "schedule_type": "single",
            "state": MissionState.IDLE.value,
            "assigned_robot_id": None,
            "started_at": None,
            "completed_at": None,
            "outcome": MissionOutcome.NONE.value,
            "retries": 0,
            "help_required": 0,
            "last_update_at": now + 1,
            "notes": "",
        }

        self.storage.create_mission(completed)
        self.storage.create_mission(queued)
        self.storage.append_event("mission-complete-1", "mission_completed", {"robot_id": "robot-1"})
        self.storage.append_event("mission-queued-1", "mission_created", {"robot_id": None})

        deleted_completed = self.storage.delete_completed_missions()
        self.assertEqual(deleted_completed, 1)
        self.assertIsNone(self.storage.get_mission("mission-complete-1"))
        self.assertEqual(self.storage.list_events("mission-complete-1"), [])
        self.assertIsNotNone(self.storage.get_mission("mission-queued-1"))

        deleted_all = self.storage.delete_all_missions()
        self.assertEqual(deleted_all, 1)
        self.assertEqual(self.storage.list_missions(), [])

    def test_delete_all_missions_keeps_pending_requests(self) -> None:
        now = time.time()
        request = {
            "id": "request-1",
            "created_at": now,
            "requested_by": "student",
            "command_source": '{"type":"user","id":"student"}',
            "from_dest": None,
            "to_dest": "Hall_A",
            "schedule_type": "single",
            "state": MissionState.REQUESTED.value,
            "assigned_robot_id": None,
            "started_at": None,
            "completed_at": None,
            "outcome": MissionOutcome.NONE.value,
            "retries": 0,
            "help_required": 0,
            "last_update_at": now,
            "notes": "",
        }
        queued = {**request, "id": "started-1", "state": MissionState.IDLE.value, "created_at": now + 1}

        self.storage.create_mission(request)
        self.storage.create_mission(queued)
        deleted = self.storage.delete_all_missions()

        self.assertEqual(deleted, 1)
        self.assertIsNotNone(self.storage.get_mission("request-1"))
        self.assertIsNone(self.storage.get_mission("started-1"))

    def test_delete_requested_missions_clears_pending_requests(self) -> None:
        now = time.time()
        request = {
            "id": "request-1",
            "created_at": now,
            "requested_by": "student",
            "command_source": '{"type":"user","id":"student"}',
            "from_dest": None,
            "to_dest": "Hall_A",
            "schedule_type": "single",
            "state": MissionState.REQUESTED.value,
            "assigned_robot_id": None,
            "started_at": None,
            "completed_at": None,
            "outcome": MissionOutcome.NONE.value,
            "retries": 0,
            "help_required": 0,
            "last_update_at": now,
            "notes": "",
        }
        queued = {**request, "id": "started-1", "state": MissionState.IDLE.value, "created_at": now + 1}

        self.storage.create_mission(request)
        self.storage.create_mission(queued)
        self.storage.append_event("request-1", "request_created", {})
        deleted = self.storage.delete_requested_missions()

        self.assertEqual(deleted, 1)
        self.assertIsNone(self.storage.get_mission("request-1"))
        self.assertEqual(self.storage.list_events("request-1"), [])
        self.assertIsNotNone(self.storage.get_mission("started-1"))


class Test03ApiModels(unittest.TestCase):
    """Covers: `mission_control/api_models.py`."""

    def test_valid_request_models(self) -> None:
        # This is how app.py validates incoming JSON payloads.
        create_req = CreateMissionRequest(
            requested_by="alice",
            command_source={"type": "user", "id": "alice"},
            to_destination="Hall_A",
            schedule_type="single",
        )
        self.assertEqual(create_req.schedule_type, "single")

        command_req = MissionCommandRequest(command_source={"type": "operator", "id": "supervisor-1"})
        self.assertEqual(command_req.command_source.type, "operator")

        manual_req = RobotManualDriveRequest(
            linear=0.5,
            angular=-0.25,
            command_source={"type": "operator", "id": "dashboard-1"},
        )
        self.assertEqual(manual_req.linear, 0.5)
        self.assertEqual(manual_req.angular, -0.25)

        sys_req = RobotSystemCommandRequest(
            command="launch_robot",
            command_source={"type": "operator", "id": "dashboard-1"},
        )
        self.assertEqual(sys_req.command, "launch_robot")

        nav_req = RobotSystemCommandRequest(
            command="launch_nav",
            map_name="test_map1",
            command_source={"type": "operator", "id": "dashboard-1"},
        )
        self.assertEqual(nav_req.map_name, "test_map1")

        pose_req = RobotInitialPoseRequest(
            x=1.2,
            y=-0.4,
            yaw=0.75,
            command_source={"type": "operator", "id": "dashboard-1"},
        )
        self.assertEqual(pose_req.yaw, 0.75)

        goal_req = RobotGoalPoseRequest(
            x=2.4,
            y=3.5,
            yaw=1.2,
            command_source={"type": "operator", "id": "dashboard-1"},
        )
        self.assertEqual(goal_req.yaw, 1.2)

        save_req = RobotMapSaveRequest(
            map_name="Office",
            command_source={"type": "operator", "id": "dashboard-1"},
        )
        self.assertEqual(save_req.map_name, "Office")

        delete_req = RobotMapDeleteRequest(
            map_name="Office",
            command_source={"type": "operator", "id": "dashboard-1"},
        )
        self.assertEqual(delete_req.map_name, "Office")

        temp_req = TempDestinationRequest(
            x=1.0,
            y=2.0,
            yaw=0.0,
            command_source={"type": "operator", "id": "dashboard-1"},
        )
        self.assertEqual(temp_req.x, 1.0)

        tel = RobotTelemetryIn(blocked=True, manual_override_active=True, battery_v=23.8)
        self.assertTrue(tel.blocked)
        self.assertTrue(tel.manual_override_active)
        self.assertEqual(tel.battery_v, 23.8)

    def test_invalid_schedule_type_rejected(self) -> None:
        with self.assertRaises(ValidationError):
            CreateMissionRequest(
                requested_by="alice",
                command_source={"type": "user", "id": "alice"},
                to_destination="Hall_A",
                schedule_type="loop_forever",
            )


class Test04RobotAdapter(unittest.IsolatedAsyncioTestCase):
    """Covers: `mission_control/robot_adapter.py`."""

    async def test_manual_override_does_not_block_start(self) -> None:
        # Manual command priority is handled by the velocity mux, not by blocking mission dispatch.
        adapter = SimRobotAdapter("robot-1", speed_scale=1.0)
        adapter.set_manual_override(True)
        await adapter.start_mission("mission-1", ["Hall_A"])
        self.assertEqual(adapter.snapshot().current_mission_id, "mission-1")

    async def test_start_cancel_and_reset(self) -> None:
        # Typical adapter lifecycle used by the scheduler.
        adapter = SimRobotAdapter("robot-1", speed_scale=1.0)
        await adapter.start_mission("mission-2", ["Hall_A"])
        self.assertEqual(adapter.snapshot().state, MissionState.EN_ROUTE)

        await adapter.cancel()

        deadline = time.time() + 2.0
        while time.time() < deadline:
            if adapter.snapshot().state == MissionState.COMPLETED:
                break
            await asyncio.sleep(0.05)

        self.assertEqual(adapter.snapshot().state, MissionState.COMPLETED)

        await adapter.reset_to_idle()
        snapshot = adapter.snapshot()
        self.assertEqual(snapshot.state, MissionState.IDLE)
        self.assertIsNone(snapshot.current_mission_id)

    async def test_power_modes_require_reset_after_stop(self) -> None:
        adapter = SimRobotAdapter("robot-1", speed_scale=1.0)

        await adapter.set_power_mode("STOP")
        self.assertEqual(adapter.power_snapshot().mode, "OFF")
        self.assertTrue(adapter.power_snapshot().safety_lock)
        self.assertEqual(adapter.snapshot().mode, RobotMode.MANUAL_OVERRIDE)

        await adapter.set_power_mode("RESET")
        self.assertFalse(adapter.power_snapshot().safety_lock)
        self.assertEqual(adapter.power_snapshot().mode, "ON")
        self.assertEqual(adapter.snapshot().mode, RobotMode.AUTO)

        await adapter.set_power_mode("AUTO")
        self.assertEqual(adapter.power_snapshot().mode, "ON")
        self.assertEqual(adapter.snapshot().mode, RobotMode.AUTO)

    async def test_manual_drive_uses_priority_without_manual_mode(self) -> None:
        adapter = SimRobotAdapter("robot-1", speed_scale=1.0)

        start_pose = dict(adapter.snapshot().pose)

        await adapter.send_manual_drive_command(0.5, 0.0)
        moved_pose = adapter.snapshot().pose
        self.assertNotEqual(start_pose["x"], moved_pose["x"])
        self.assertEqual(adapter.snapshot().mode, RobotMode.MANUAL_OVERRIDE)

        await adapter.send_manual_drive_command(0.0, 0.0)
        self.assertIn("stopped", adapter.power_snapshot().recent_log.lower())

    async def test_initial_pose_and_system_command_in_sim_adapter(self) -> None:
        adapter = SimRobotAdapter("robot-1", speed_scale=1.0)

        await adapter.set_initial_pose(1.0, 2.0, 0.5)
        await adapter.send_system_command("launch_robot")
        await adapter.set_goal_pose(2.5, -0.75, 0.25)

        operator = adapter.operator_snapshot()
        self.assertEqual(operator["initial_pose"]["x"], 1.0)
        self.assertEqual(operator["initial_pose"]["y"], 2.0)
        self.assertEqual(operator["goal_pose"]["x"], 2.5)
        self.assertEqual(operator["last_system_command"], "launch_robot")
        self.assertTrue(operator["system_commands_available"])

    async def test_global_localization_action_in_sim_adapter(self) -> None:
        adapter = SimRobotAdapter("robot-1", speed_scale=1.0)
        adapter.set_localization_valid(False)

        result = await adapter.localize()

        self.assertTrue(result["ok"])
        self.assertTrue(adapter.snapshot().localization_valid)
        self.assertIn("localization", adapter.power_snapshot().recent_log.lower())

    async def test_map_catalog_save_delete_and_launch_nav_in_sim_adapter(self) -> None:
        adapter = SimRobotAdapter("robot-1", speed_scale=1.0)

        await adapter.save_map("Office")
        self.assertIn("Office", adapter.operator_snapshot()["saved_maps"])

        await adapter.send_system_command("launch_nav", map_name="Office")
        self.assertEqual(adapter.operator_snapshot()["current_map_name"], "Office")

        with self.assertRaises(RuntimeError):
            await adapter.delete_map("Office")

        await adapter.send_system_command("kill_all")
        await adapter.delete_map("Office")
        self.assertNotIn("Office", adapter.operator_snapshot()["saved_maps"])

    def test_map_preview_loader_reads_catering_bot_maps(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            pgm = root / "tiny_map.pgm"
            pgm.write_bytes(b"P5\n3 2\n255\n\xfe\xcd\x00\x00\xfe\xcd")
            yaml_path = root / "tiny_map.yaml"
            yaml_path.write_text(
                (
                    "image: tiny_map.pgm\n"
                    "mode: trinary\n"
                    "resolution: 0.05\n"
                    "origin: [-2.0, -3.0, 0.5]\n"
                    "negate: 0\n"
                    "occupied_thresh: 0.65\n"
                    "free_thresh: 0.25\n"
                ),
                encoding="utf-8",
            )

            preview = _load_map_preview_from_yaml(yaml_path)

        self.assertEqual(preview["name"], "tiny_map")
        self.assertEqual(preview["width"], 3)
        self.assertEqual(preview["height"], 2)
        self.assertEqual(preview["origin"]["x"], -2.0)
        self.assertEqual(len(preview["data"]), 6)
        self.assertIn(100, preview["data"])


class Test05Scheduler(unittest.IsolatedAsyncioTestCase):
    """Covers: `mission_control/scheduler.py` by integrating with config/storage/adapter."""

    async def asyncSetUp(self) -> None:
        self.tmpdir = tempfile.TemporaryDirectory()
        self.db_path = Path(self.tmpdir.name) / "mission_control.sqlite3"
        self.config_path = Path(self.tmpdir.name) / "destinations.yaml"
        write_demo_destinations(self.config_path)

        self.storage = Storage(self.db_path)
        self.storage.init()

        self.dest_config = DestinationConfig(self.config_path)
        self.dest_config.load()

        self.mc = MissionControl(storage=self.storage, dest_config=self.dest_config)
        self.adapter = SimRobotAdapter("robot-1", speed_scale=100.0)
        self.mc.register_robot(self.adapter)

    async def asyncTearDown(self) -> None:
        await self.mc.stop()
        self.tmpdir.cleanup()

    async def test_create_dispatch_complete_flow(self) -> None:
        # End-to-end path:
        # create mission -> scheduler dispatches -> adapter completes -> scheduler records success.
        mission_id = self.mc.create_mission(
            MissionCreate(
                requested_by="alice",
                command_source=CommandSource("user", "alice"),
                to_destination="Hall_A",
                schedule_type="single",
            )
        )

        await self.mc._tick_once()  # dispatch pending mission
        mission = self.storage.get_mission(mission_id)
        self.assertEqual(mission["state"], MissionState.EN_ROUTE.value)
        self.assertEqual(mission["assigned_robot_id"], "robot-1")

        deadline = time.time() + 3.0
        while time.time() < deadline:
            await asyncio.sleep(0.05)
            await self.mc._tick_once()
            mission = self.storage.get_mission(mission_id)
            if mission["state"] == MissionState.COMPLETED.value:
                break

        self.assertEqual(mission["state"], MissionState.COMPLETED.value)
        self.assertEqual(mission["outcome"], MissionOutcome.SUCCESS.value)

    async def test_request_waits_until_started(self) -> None:
        request_id = self.mc.create_request(
            MissionCreate(
                requested_by="alice",
                command_source=CommandSource("user", "alice"),
                to_destination="Hall_A",
                schedule_type="single",
            )
        )

        await self.mc._tick_once()
        request = self.storage.get_mission(request_id)
        self.assertEqual(request["state"], MissionState.REQUESTED.value)
        self.assertIsNone(self.adapter.snapshot().current_mission_id)

        self.mc.start_request(
            request_id,
            CommandSource("operator", "dashboard-1"),
            assigned_robot_id="robot-1",
        )
        await self.mc._tick_once()

        mission = self.storage.get_mission(request_id)
        self.assertEqual(mission["state"], MissionState.EN_ROUTE.value)
        self.assertEqual(mission["assigned_robot_id"], "robot-1")

    async def test_round_trip_waits_for_return_confirmation(self) -> None:
        mission_id = self.mc.create_mission(
            MissionCreate(
                requested_by="alice",
                command_source=CommandSource("user", "alice"),
                to_destination="Hall_A",
                schedule_type="round_trip",
                from_destination="Storage",
                assigned_robot_id="robot-1",
            )
        )

        await self.mc._tick_once()
        mission = self.storage.get_mission(mission_id)
        self.assertEqual(mission["state"], MissionState.EN_ROUTE.value)

        deadline = time.time() + 3.0
        while time.time() < deadline:
            await asyncio.sleep(0.05)
            await self.mc._tick_once()
            mission = self.storage.get_mission(mission_id)
            if mission["state"] == MissionState.WAITING_FOR_RETURN.value:
                break

        self.assertEqual(mission["state"], MissionState.WAITING_FOR_RETURN.value)
        self.assertEqual(mission["outcome"], MissionOutcome.NONE.value)
        self.assertEqual(self.adapter.snapshot().state, MissionState.IDLE)
        self.assertIsNone(self.adapter.snapshot().current_mission_id)

        result = await self.mc.start_return_trip(mission_id, CommandSource("operator", "dashboard-1"))
        self.assertEqual(result["return_destination"], "Storage")

        mission = self.storage.get_mission(mission_id)
        self.assertEqual(mission["state"], MissionState.RETURNING.value)

        deadline = time.time() + 3.0
        while time.time() < deadline:
            await asyncio.sleep(0.05)
            await self.mc._tick_once()
            mission = self.storage.get_mission(mission_id)
            if mission["state"] == MissionState.COMPLETED.value:
                break

        self.assertEqual(mission["state"], MissionState.COMPLETED.value)
        self.assertEqual(mission["outcome"], MissionOutcome.SUCCESS.value)

    async def test_ingest_telemetry_updates_storage_and_adapter(self) -> None:
        # This is how external robot/bridge inputs can update mission control state.
        self.mc.ingest_robot_telemetry(
            "robot-1",
            {
                "blocked": True,
                "manual_override_active": True,
                "battery_v": 23.5,
                "x": 1.2,
                "y": 3.4,
                "yaw": 0.5,
            },
        )

        robot = self.storage.get_robot("robot-1")
        self.assertEqual(robot["blocked"], 1)
        self.assertEqual(robot["mode"], RobotMode.MANUAL_OVERRIDE.value)
        self.assertEqual(robot["battery_v"], 23.5)
        self.assertEqual(robot["x"], 1.2)
        self.assertTrue(self.adapter.snapshot().blocked)
        self.assertEqual(self.adapter.snapshot().mode, RobotMode.MANUAL_OVERRIDE)

    async def test_completed_adapter_outcome_is_preserved(self) -> None:
        mission_id = self.mc.create_mission(
            MissionCreate(
                requested_by="alice",
                command_source=CommandSource("user", "alice"),
                to_destination="Hall_A",
                schedule_type="single",
                assigned_robot_id="robot-1",
            )
        )
        self.storage.update_mission(
            mission_id,
            assigned_robot_id="robot-1",
            state=MissionState.EN_ROUTE.value,
            started_at=time.time(),
        )

        self.adapter._current_mission_id = mission_id
        self.adapter._state = MissionState.COMPLETED
        self.adapter._last_outcome = MissionOutcome.FAILED

        await self.mc._handle_completions({"robot-1": self.adapter.snapshot()})

        mission = self.storage.get_mission(mission_id)
        self.assertEqual(mission["state"], MissionState.COMPLETED.value)
        self.assertEqual(mission["outcome"], MissionOutcome.FAILED.value)

    async def test_recovery_attempts_escalate_after_three_retries(self) -> None:
        # Demonstrates blocked recovery policy in scheduler.py.
        mission_id = self.mc.create_mission(
            MissionCreate(
                requested_by="alice",
                command_source=CommandSource("user", "alice"),
                to_destination="Ballroom",
                schedule_type="single",
                assigned_robot_id="robot-1",
            )
        )
        self.storage.update_mission(
            mission_id,
            assigned_robot_id="robot-1",
            state=MissionState.EN_ROUTE.value,
            started_at=time.time(),
        )

        async def fast_sleep(_seconds: float) -> None:
            return None

        with patch("mission_control.scheduler.asyncio.sleep", new=fast_sleep):
            await self.mc._attempt_recovery(mission_id, "robot-1")
            await self.mc._attempt_recovery(mission_id, "robot-1")
            await self.mc._attempt_recovery(mission_id, "robot-1")

        mission = self.storage.get_mission(mission_id)
        self.assertEqual(mission["retries"], 3)
        self.assertEqual(mission["help_required"], 1)
        self.assertEqual(mission["state"], MissionState.PAUSED.value)

    async def test_blocked_detection_ignores_manual_override(self) -> None:
        mission_id = self.mc.create_mission(
            MissionCreate(
                requested_by="alice",
                command_source=CommandSource("user", "alice"),
                to_destination="Ballroom",
                schedule_type="single",
                assigned_robot_id="robot-1",
            )
        )
        self.storage.update_mission(
            mission_id,
            assigned_robot_id="robot-1",
            state=MissionState.EN_ROUTE.value,
            started_at=time.time(),
        )
        self.mc._blocked_since["robot-1"] = time.time() - 10.0

        snapshot = self.adapter.snapshot()
        snapshot.current_mission_id = mission_id
        snapshot.state = MissionState.EN_ROUTE
        snapshot.mode = RobotMode.MANUAL_OVERRIDE
        snapshot.blocked = True
        snapshot.obstacle_stop = True

        await self.mc._blocked_detection({"robot-1": snapshot})

        self.assertNotIn("robot-1", self.mc._blocked_since)
        events = self.storage.list_events(mission_id)
        self.assertFalse(any(event["event"] == "blocked_detected" for event in events))

    async def test_build_plan_for_single_and_round_trip(self) -> None:
        # _build_plan dispatches only the active leg. Round trips wait for a Return command.
        single = self.mc._build_plan({"to_dest": "Hall_A", "schedule_type": "single"})
        self.assertEqual(single, ["Hall_A"])

        round_trip = {"to_dest": "Hall_A", "from_dest": "Ballroom", "schedule_type": "round_trip"}
        self.assertEqual(self.mc._build_plan(round_trip), ["Hall_A"])
        self.assertEqual(self.mc._return_destination_for(round_trip), "Ballroom")

        round_trip_home = {"to_dest": "Hall_A", "from_dest": None, "schedule_type": "round_trip"}
        self.assertEqual(self.mc._build_plan(round_trip_home), ["Hall_A"])
        self.assertEqual(self.mc._return_destination_for(round_trip_home), "Storage")

    async def test_clear_all_missions_requires_inactive_robot(self) -> None:
        mission_id = self.mc.create_mission(
            MissionCreate(
                requested_by="alice",
                command_source=CommandSource("user", "alice"),
                to_destination="Hall_A",
                schedule_type="single",
                assigned_robot_id="robot-1",
            )
        )
        self.storage.update_mission(
            mission_id,
            assigned_robot_id="robot-1",
            state=MissionState.EN_ROUTE.value,
            started_at=time.time(),
        )

        self.adapter._current_mission_id = mission_id
        self.adapter._state = MissionState.EN_ROUTE

        with self.assertRaises(RuntimeError):
            await self.mc.clear_all_missions()

    async def test_set_robot_power_mode_pauses_active_mission_and_updates_snapshot(self) -> None:
        mission_id = self.mc.create_mission(
            MissionCreate(
                requested_by="alice",
                command_source=CommandSource("user", "alice"),
                to_destination="Hall_A",
                schedule_type="single",
                assigned_robot_id="robot-1",
            )
        )
        self.storage.update_mission(
            mission_id,
            assigned_robot_id="robot-1",
            state=MissionState.EN_ROUTE.value,
            started_at=time.time(),
        )
        self.adapter._current_mission_id = mission_id
        self.adapter._state = MissionState.EN_ROUTE

        power = await self.mc.set_robot_power_mode("robot-1", "STOP", CommandSource("operator", "dashboard-1"))

        self.assertEqual(power["mode"], "OFF")
        mission = self.storage.get_mission(mission_id)
        self.assertEqual(mission["state"], MissionState.PAUSED.value)

        snapshot = self.mc.snapshot()
        robot = next(robot for robot in snapshot["robots"] if robot["id"] == "robot-1")
        self.assertEqual(robot["power"]["mode"], "OFF")
        self.assertTrue(robot["power"]["safety_lock"])

    async def test_manual_drive_command_uses_priority_without_manual_mode(self) -> None:
        result = await self.mc.send_robot_manual_drive_command(
            "robot-1",
            0.5,
            0.0,
            CommandSource("operator", "dashboard-1"),
        )

        self.assertEqual(result["robot_id"], "robot-1")
        self.assertEqual(result["linear"], 0.5)
        self.assertEqual(result["angular"], 0.0)

    async def test_operator_snapshot_tracks_initial_pose_and_system_commands(self) -> None:
        await self.mc.set_robot_initial_pose(
            "robot-1",
            0.75,
            -1.25,
            0.33,
            CommandSource("operator", "dashboard-1"),
        )
        await self.mc.send_robot_system_command(
            "robot-1",
            "launch_nav",
            CommandSource("operator", "dashboard-1"),
            map_name="test_map1",
        )

        snapshot = self.mc.robot_operator_snapshot("robot-1")
        self.assertEqual(snapshot["initial_pose"]["x"], 0.75)
        self.assertEqual(snapshot["initial_pose"]["y"], -1.25)
        self.assertEqual(snapshot["last_system_command"], "launch_nav")
        self.assertEqual(snapshot["robot_id"], "robot-1")

    async def test_map_management_and_goal_pose_through_scheduler(self) -> None:
        await self.mc.save_robot_map("robot-1", "Office", CommandSource("operator", "dashboard-1"))
        await self.mc.send_robot_system_command(
            "robot-1",
            "launch_nav",
            CommandSource("operator", "dashboard-1"),
            map_name="Office",
        )
        await self.mc.set_robot_goal_pose(
            "robot-1",
            4.0,
            -2.0,
            0.5,
            CommandSource("operator", "dashboard-1"),
        )

        snapshot = self.mc.robot_operator_snapshot("robot-1")
        self.assertIn("Office", snapshot["saved_maps"])
        self.assertEqual(snapshot["current_map_name"], "Office")
        self.assertEqual(snapshot["goal_pose"]["x"], 4.0)


class Test06MappingNotes(unittest.TestCase):
    """Covers: `mission_control/mapping.txt` project-progress helper notes."""

    def test_mapping_file_contains_step_summary(self) -> None:
        mapping_path = PROJECT_ROOT / "mission_control" / "mapping.txt"
        self.assertTrue(mapping_path.exists())

        text = mapping_path.read_text(encoding="utf-8")
        self.assertIn("Step 1", text)
        self.assertIn("Step 6", text)


if __name__ == "__main__":
    unittest.main(verbosity=2)
