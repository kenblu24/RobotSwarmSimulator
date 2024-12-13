# from abc import ABC

class Spawner:
    """
    Spawner: An abstract object creator
    """
    def __init__(self, world, **kwargs):
        self.world = world
        self.mark_for_deletion = False
        self.oneshot = False
        self.spawned = 0

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
