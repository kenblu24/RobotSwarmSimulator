# from abc import ABC
import numpy as np


class Spawner:
    """
    Spawner: An abstract object creator
    """
    def __init__(self, world, seed='unspecified', **kwargs):
        self.world = world
        self.mark_for_deletion = False
        self.oneshot = False
        self.spawned = 0

        if seed == 'unspecified':
            self.set_seed(world.rng.integers(0, 2**31))
        else:
            self.set_seed(seed)

    def set_seed(self, seed):
        self.seed = np.random.randint(0, 2**31) if seed is None else seed
        self.rng = np.random.default_rng(self.seed)
        return self.seed

    def step(self):
        pass

    # def set_to_world(self, world):
    #     """
    #     Set the initialization of the world agents.
    #     """
    #     if not hasattr(self, "positions"):
    #         raise Exception("Abstract Initialization Class must have the 'positions' attributes assigned")

    #     for i in range(len(world.population)):
    #         world.population[i].set_pos_vec(self.positions[i])
    #         world.population[i].name = str(i)
