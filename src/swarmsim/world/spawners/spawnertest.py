from swarmsim.world.spawners.AgentSpawner import UniformCircleAgentSpawner

import numpy as np

class Empty:
    pass
mw = Empty()
setattr(mw, "rng", np.random.default_rng(np.random.randint(0, 10000)))
cs = UniformCircleAgentSpawner(mw, [100, 0], 3)

print(mw.rng.random())

print(cs.generate_points_in_circle(4))