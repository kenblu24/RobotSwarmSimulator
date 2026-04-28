import numpy as np
from .AbstractMetric import AbstractMetric


class EntityLen(AbstractMetric):

    def __init__(
        self,
        history=100,
        target_name=None,
        attribute=None,
    ):
        super().__init__(name="EntityLen", history_size=history)
        self.target_name = target_name
        self.target = None
        self.attribute = attribute

    def find_target(self):
        if self.target_name is None:
            raise ValueError("AgentCount metric requires a target_name.")

        for it in (
            self.world.population,
            self.world.objects,
            self.world.spawners,
            self.world.metrics,
        ):
            for thing in it:
                if thing.name == self.target_name:
                    return thing

        if self.target_name == 'population':
            return self.world.population
        elif self.target_name == 'objects':
            return self.world.objects
        elif self.target_name == 'spawners':
            return self.world.spawners
        elif self.target_name == 'metrics':
            return self.world.metrics
        return None

    def calculate(self):
        if self.target is None:
            self.target = self.find_target()

        if self.attribute is not None:
            ent = getattr(self.target, self.attribute)
        else:
            ent = self.target

        self.set_value(None if ent is None else len(ent))
