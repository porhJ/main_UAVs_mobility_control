# autonomy_core.py
# this is logic for autonomy_node. 

from .world_model import WorldModel
from .decision_agent import DecisionAgent
from .mission_executor import MissionExecutor


class AutonomyCore:
    def __init__(self):
        self.world_model = WorldModel()
        self.agent = DecisionAgent()
        self.executor = MissionExecutor()

    def tick(self, dt: float):
        world = self.world_model.state()

        action = self.agent.decide(world)

        objective = self.executor.tick(
            action,
            world,
            dt
        )

        return objective