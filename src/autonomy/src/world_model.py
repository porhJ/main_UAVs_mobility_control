# world_model.py

# Interfaces between perception and agents
# Perception - world_model -> agent - pilot -> executor - sequence of discrete commands (e.g. goto, land, takeoff) -> offboard_master -> PX4

from .autonomy_types import (
    VehicleState,
    WorldObject,
    WorldState,
)


class WorldModel:
    def __init__(self):
        self._state = WorldState()

    def update_vehicle(self, vehicle: VehicleState):
        self._state.vehicle = vehicle

    def update_objects(self, objects: list[WorldObject]):
        self._state.objects = objects

    def state(self) -> WorldState:
        return self._state