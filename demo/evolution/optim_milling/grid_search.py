"""
Find the best Homogeneous Agents for Milling
"""
from ctypes import ArgumentError
from io import BytesIO
import argparse
import numpy as np
from tqdm import tqdm
from src.novel_swarms.results.Experiment import Experiment
from src.novel_swarms.optim.CMAES import CMAES
# from src.novel_swarms.world.initialization.PredefInit import PredefinedInitialization

from .milling_search import DECISION_VARS, PERFECT_CIRCLE_SCORE
from .milling_search import get_world_generator
from .milling_search import fitness


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("--name", type=str, help="Name of the experiment", default=None)
    parser.add_argument("--root", type=str, help="Experiment folder root", default=None)
    parser.add_argument("-n", type=int, default=10, help="Number of agents")
    parser.add_argument("-t", type=int, default=1000, help="Environment Horizon")
    parser.add_argument("--processes", type=int, default=1, help="Number of running concurrent processes")
    parser.add_argument("--dimension", type=int, nargs=4, help="How many bins to discretize the decision variables into")

    args = parser.parse_args()

    exp = Experiment(root="demo/results/out" if not args.root else args.root, title=args.name)

    # Fix the initial conditions if params indicate
    # init = None
    # if args.fixed_config:
    #     init = FixedInitialization("demo/configs/flockbots-icra/init_translated.csv")

    # Save World Config by sampling from generator
    world_gen_example = get_world_generator(args.n, args.t)
    sample_worlds = world_gen_example([-1, -1, -1, -1], [-1, -1, -1, -1])
    # sample_worlds[0].stop_at = 1003
    # sim(world_config=sample_worlds[0], save_every_ith_frame=2, save_duration=1000)

    sample_worlds[0].save_yaml(exp)

    cmaes = CMAES(
        fitness,
        genome_to_world=get_world_generator(args.n, args.t),
        dvars=DECISION_VARS,
        num_processes=args.processes,
        show_each_step=False,
        target=PERFECT_CIRCLE_SCORE,
        experiment=exp,
        pop_size=0,
    )

    cmaes.sweep_parameters(args.dimension)

# run with:
# python -m demo.evolution.optim_milling.grid_search --name "connormill_10n_1000t_d3333" -n 10 -t 1000 --processes 24 --dimension 3 3 3 3
# this generates a grid of 3x3x3x3 = 81 configurations and runs them all.
