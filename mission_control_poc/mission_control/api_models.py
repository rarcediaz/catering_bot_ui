from __future__ import annotations

from typing import Any, Dict, List, Literal, Optional

from pydantic import BaseModel, Field


class CommandSourceModel(BaseModel):
    type: str = Field(..., examples=["user", "operator", "system"])
    id: str = Field(..., examples=["alice", "supervisor-tablet-1"])
    meta: Optional[Dict[str, Any]] = None


class CreateMissionRequest(BaseModel):
    requested_by: str = Field(..., description="Human-readable requester (staff username, device id, etc.)")
    command_source: CommandSourceModel
    to_destination: str
    schedule_type: Literal["single", "round_trip"] = "single"
    from_destination: Optional[str] = None
    assigned_robot_id: Optional[str] = None
    map_name: Optional[str] = None
    notes: str = ""


class CreateMissionResponse(BaseModel):
    mission_id: str


class CreateRequestResponse(BaseModel):
    request_id: str
    request_number: int


class MissionCommandRequest(BaseModel):
    command_source: CommandSourceModel


class RobotPowerCommandRequest(BaseModel):
    mode: Literal["ON", "OFF", "RESET", "STOP", "AUTO", "MANUAL"]
    command_source: CommandSourceModel


class RobotManualDriveRequest(BaseModel):
    linear: float = Field(..., ge=-1.0, le=1.0, description="Forward/backward speed command.")
    angular: float = Field(..., ge=-1.0, le=1.0, description="Left/right turn command.")
    command_source: CommandSourceModel


class RobotSystemCommandRequest(BaseModel):
    command: Literal["launch_robot", "launch_slam", "launch_nav", "save_map", "kill_all"]
    map_name: Optional[str] = None
    command_source: CommandSourceModel


class RobotInitialPoseRequest(BaseModel):
    x: float
    y: float
    yaw: float = 0.0
    command_source: CommandSourceModel


class RobotGoalPoseRequest(BaseModel):
    x: float
    y: float
    yaw: float = 0.0
    command_source: CommandSourceModel


class RobotMapSaveRequest(BaseModel):
    map_name: str = Field(..., min_length=1)
    command_source: CommandSourceModel


class RobotMapDeleteRequest(BaseModel):
    map_name: str = Field(..., min_length=1)
    command_source: CommandSourceModel


class TempDestinationRequest(BaseModel):
    x: float
    y: float
    yaw: float = 0.0
    notes: str = ""
    command_source: CommandSourceModel


class DestinationSaveRequest(BaseModel):
    name: str = Field(..., min_length=1)
    x: float
    y: float
    yaw: float = 0.0
    notes: str = ""
    command_source: CommandSourceModel


class RobotTelemetryIn(BaseModel):
    connection_ok: Optional[bool] = None
    localization_valid: Optional[bool] = None
    obstacle_stop: Optional[bool] = None
    blocked: Optional[bool] = None
    manual_override_active: Optional[bool] = None
    battery_v: Optional[float] = None
    x: Optional[float] = None
    y: Optional[float] = None
    yaw: Optional[float] = None


class StatusSnapshot(BaseModel):
    server_time: float
    destinations: List[Dict[str, Any]]
    robots: List[Dict[str, Any]]
    missions: List[Dict[str, Any]]


class MissionDetail(BaseModel):
    mission: Dict[str, Any]
    events: List[Dict[str, Any]]
