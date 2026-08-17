# mission_executor.py
# to track each mission; e.g. goto, if goto is finished or not.
from dataclasses import dataclass
from enum import Enum, auto
from abc import ABC, abstractmethod
import numpy as np

from autonomy_types import reached, ControlCommand
from typing_extensions import final

from .autonomy_types import (
    ActionType,
    PilotAction,
    WorldState,
)


class ActionStatus(Enum):
    SUCCESS = auto()
    FAILURE = auto()
    PROGRESS = auto()

@dataclass
class mission_state:
    current_mission: ActionType | None = None
    state: ExecutorStatus | None = None
    mission_state: MissionSa | None = None

class ActionExecutor(ABC):
    def __init__(self, action: PilotAction):
        self.action = action

    @abstractmethod
    def tick(self, world):
        pass
@final
class GotoExecutor(ActionExecutor):

    @final
    class State(Enum):
        NAV = auto()
        REACHED = auto()

    def __init__(self, action: PilotAction):
        super().__init__(action)
        self.state = self.State.NAV

    def tick(self, state: WorldState):

        if self.action.target != None:
            target = self.action.target.position
        else:
            target = self.action.goto

        if target != None and reached(state.vehicle.position, target):
            self.state = self.State.REACHED
            self.




@final
class MissionExecutor:
    def __init__(self):
        self.status = mission_state()

    def set_state(self, pilot_action: PilotAction, world_state: WorldState, dt: float):
        if self.status.mission_state != MissionState.PROGRESS:
           self.status.current_mission = pilot_action.type

        match pilot_action.type:
            case ActionType.IDLE:
                self.status.state = ExecutorState.IDLE
            case ActionType.

            return self.status
