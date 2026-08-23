from .metric import Metric


class TotalCollisionsBehavior(Metric):
    def __init__(self, history=1):
        super().__init__(name="Total_Collisions", history_size=history)
        self.total_collisions = 0

    @property
    def population(self):
        return self.world.population

    def calculate(self):
        for agent in self.population:
            if agent.collision_flag:
                self.total_collisions += 1
        self.set_value(self.total_collisions)
