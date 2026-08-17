# decision_agent.py

# Interfaces between perception and agents
# Perception - world_model -> agent - pilot -> executor - sequence of discrete commands (e.g. goto, land, takeoff) -> offboard_master -> PX4

from .autonomy_types import (
    ActionType,
    PilotAction,
    WorldState,
)


class DecisionAgent:

    def decide(self, state: WorldState,) -> PilotAction:

        if not state.vehicle.armed:
            return PilotAction(
                type=ActionType.TAKEOFF
            )

        if not state.vehicle.accepting_setpoints:
            return PilotAction(
                type=ActionType.IDLE
            )

        return PilotAction(
            type=ActionType.HOVER
        )