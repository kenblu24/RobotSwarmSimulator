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
    A Multi-Threaded Implementation of the swarmsim.world.simulate package
    """

    def __init__(self, pool_size=None, single_step=False, with_gui=False, use_tqdm=False, hide_tqdm=True):
        self.single_step = single_step
        self.with_gui = with_gui
        self.pool_size = pool_size
        self.use_tqdm = use_tqdm
        self.hide_tqdm = hide_tqdm

    def delete_lines_above(self, n=1):
        if not self.hide_tqdm:
            return
        FKG = f"\033[F\033[2K\033[1G"  # Cursor up one line, clear line, cursor to beginning of line
        for _i in range(n):
            print(FKG, flush=True, end='')

    def execute(self, world_setup: list, world_stop_condition=None, batched=False):
        if not world_setup:
            raise Exception("No world_setup list provided to execute.")
        # print("hello")
        ret = []
        if not self.single_step:
            bundles = world_setup
            fn = simulate_batch if batched else simulate
            fn = partial(fn, terminate_function=world_stop_condition)
            if self.use_tqdm is True:
                ret = process_map(fn, bundles, max_workers=self.pool_size)
                self.delete_lines_above()
            elif self.use_tqdm:
                ret = process_map(fn, bundles, max_workers=self.pool_size, tqdm_class=self.use_tqdm)
                self.delete_lines_above(1)
            else:
                # ret = list(map(fn, bundles))
                with Pool(self.pool_size) as pool:
                    ret = pool.map(fn, bundles)
        else:
            for w in world_setup:
                if batched:
                    ret.append(simulate_batch(w, world_stop_condition, show_gui=self.with_gui))
                else:
                    ret.append(simulate(w, world_stop_condition, show_gui=self.with_gui))
        return ret
