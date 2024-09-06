from functools import partial
import warnings
from multiprocessing import Pool
from tqdm.contrib.concurrent import process_map

from ...world.simulate import main as sim


def simulate(world_config, terminate_function, show_gui=False):
    try:
        world = sim(world_config, show_gui=show_gui, stop_detection=terminate_function, step_size=5)
        return world
    except Exception as e:
        warnings.WarningMessage("World could not be simulated: " + str(e))
    return None

def simulate_batch(world_config_list, terminate_function, show_gui=False):
    ret = []
    for w in world_config_list:
        ret.append(simulate(w, terminate_function, show_gui=False))
    return ret

class MultiWorldSimulation:
    """
    A Multi-Threaded Implementation of the novel_swarms.world.simulate package
    """

    def __init__(self, pool_size=4, single_step=False, with_gui=False):
        self.single_step = single_step
        self.with_gui = with_gui
        self.pool_size = pool_size

    def execute(self, world_setup: list, world_stop_condition=None, batched=False, use_tqdm=False):

        if not world_setup:
            raise Exception("No world_setup list provided to execute.")

        ret = []
        if not self.single_step:
            bundles = world_setup
            fn = simulate_batch if batched else simulate
            fn = partial(fn, terminate_function=world_stop_condition)
            if use_tqdm is True:
                ret = process_map(fn, bundles)
            elif use_tqdm:
                ret = process_map(fn, bundles, tqdm_class=use_tqdm)
            else:
                ret = list(map(fn, bundles))
            # with Pool(self.pool_size) as pool:
            #     ret = pool.starmap(fn, bundles)
        else:
            for w in world_setup:
                if batched:
                    ret.append(simulate_batch(w, world_stop_condition, show_gui=self.with_gui))
                else:
                    ret.append(simulate(w, world_stop_condition, show_gui=self.with_gui))
        return ret

