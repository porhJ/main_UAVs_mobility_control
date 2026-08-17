# autonomy_types.py

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Any

import numpy as np


@dataclass
class Vec3:
    x: float
    y: float
    z: float


@dataclass
class WorldObject:
    id: str
    type: str
    position: Vec3
    confidence: float = 0.0
    properties: dict[str, Any] = field(default_factory=dict)


@dataclass
class VehicleState:
    position: Vec3 = field(default_factory=lambda: Vec3(0.0, 0.0, 0.0))

    velocity: Vec3 = field(default_factory=lambda: Vec3(0.0, 0.0, 0.0))

    armed: bool = False
    offboard: bool = False
    accepting_setpoints: bool = False


@dataclass
class WorldState:
    vehicle: VehicleState = field(default_factory=VehicleState)
    objects: list[WorldObject] = field(default_factory=list)


class ActionType(Enum):
    IDLE = auto()
    TAKEOFF = auto()
    GOTO = auto()
    HOVER = auto()
    LAND = auto()


@dataclass
class PilotAction:
    id: int
    type: ActionType = ActionType.IDLE
    target: WorldObject | None = None
    goto: Vec3 | None = None


class ControlMode(Enum):
    NONE = auto()
    POSITION = auto()
    VELOCITY = auto()
    LAND = auto()


@dataclass
class ControlCommand:
    mode: ControlMode = ControlMode.NONE

    position_mode: Vec3 = field(default_factory=lambda: Vec3(0.0, 0.0, 0.0))
    velocity_mode: Vec3 = field(default_factory=lambda: Vec3(0.0, 0.0, 0.0))

    yaw: float | None = None
    yaw_rate: float | None = None


# util
def reached(p1: Vec3, p2: Vec3, tol=0.1):
    pos1 = np.array([p1.x, p1.y, p1.z])
    pos2 = np.array([p2.x, p2.y, p2.z])

    distance = np.linalg.norm(pos1 - pos2)
    return distance <= tol
