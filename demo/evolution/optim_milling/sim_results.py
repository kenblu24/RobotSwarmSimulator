"""
Find the best Homogeneous Agents for Milling
"""
from ctypes import ArgumentError
import numpy as np
from io import BytesIO
import argparse
from src.novel_swarms.optim.CMAES import CMAES
from src.novel_swarms.world.initialization.PredefInit import PredefinedInitialization

from .milling_search import DECISION_VARS, SCALE, BL
# from .milling_search import fitness
from .milling_search import get_world_generator


def metric_to_canon(genome: tuple[float, float, float, float], scale=SCALE):
    v0, w0, v1, w1 = genome
    v0 *= scale / BL
    v1 *= scale / BL
    return (v0, w0, v1, w1)


def canon_to_metric(genome: tuple[float, float, float, float], scale=SCALE):
    v0, w0, v1, w1 = genome
    v0 /= scale / BL
    v1 /= scale / BL
    return (v0, w0, v1, w1)


def run(world_config, gui=True):
    from src.novel_swarms.world.simulate import main as sim
    w = sim(world_config=world_config, save_every_ith_frame=2, save_duration=1000, show_gui=gui)
    try:
        return w.behavior[0].out_average()[1]
    except BaseException:
        pass


if __name__ == "__main__":
    """
    Example usage:
    `python -m demo.evolution.optim_milling.sim_results --v0 0.1531 --w0 0.3439 --v1 0.1485 --w1 0.1031 --n 10 --t 1000`
    """

    parser = argparse.ArgumentParser()

    parser.add_argument("--n", type=int, default=10, help="Number of agents")
    parser.add_argument("--t", type=int, default=1000, help="Environment Horizon")
    parser.add_argument("--no-stop", action="store_true", help="Whether to stop at T limit or not")
    parser.add_argument("--print", action="store_true")
    parser.add_argument("--nogui", action="store_true")
    parser.add_argument("--discrete-bins", default=None, help="How many bins to discretize the decision variables into")
    parser.add_argument('--positions', default=None,
                             help="file containing agent positions")
    genome_parser = parser.add_mutually_exclusive_group(required=True)
    genome_parser.add_argument(
        "--genome",
        type=float,
        help="meters/second genome",
        default=None,
        nargs=4,
    )
    genome_parser.add_argument(
        "--normalized_genome",
        type=float,
        help="Genome values (4 floats expected between [0, 1])",
        default=None,
        nargs=4,
    )
    genome_parser.add_argument(
        "--metric_genome",
        type=float,
        help="Genome values (4 floats expected between [0, 1])",
        default=None,
        nargs=4,
    )

    args = parser.parse_args()

    gui = not args.nogui

    if args.normalized_genome:
        genome = args.normalized_genome

        if args.discrete_bins:
            increment = 1 / (int(args.discrete_bins) - 1)
            genome = [CMAES.round_to_nearest(x, increment=increment) for x in genome]

        genome = DECISION_VARS.from_normalized_to_scaled(genome)

    elif args.metric_genome:
        genome = metric_to_canon(args.metric_genome)

    else:
        genome = args.genome

    if args.discrete_bins and not args.normalized_genome:
        raise ArgumentError(args.discrete_bins, "Discrete binning can only be used with --normalized_genome")

    if args.print:
        metric_genome = canon_to_metric(genome)
        m = metric_genome
        g = genome
        print(f"v0   (m/s):\t{m[0]:>16.12f}\tv1   (m/s):\t{m[2]:>16.12f}")
        print(f"v0 (canon):\t{g[0]:>16.12f}\tv1 (canon):\t{g[2]:>16.12f}")
        print(f"w0 (rad/s):\t{g[1]:>16.12f}\tw1 (rad/s):\t{g[3]:>16.12f}")

    # Save World Config by sampling from generator
    world_generator = get_world_generator(args.n, args.t)
    world_config, *_ = world_generator(genome, [-1, -1, -1, -1])

    if args.no_stop:
        world_config.stop_at = None
    else:
        world_config.stop_at = args.t

    if args.positions:
        import pandas as pd
        fpath = args.positions

        with open(fpath, 'rb') as f:
            xlsx = f.read()
        xlsx = pd.ExcelFile(BytesIO(xlsx))
        sheets = xlsx.sheet_names

        n_runs = len(sheets)

        pinit = PredefinedInitialization()  # num_agents isn't used yet here

        def setup_i(i):
            pinit.set_states_from_xlsx(args.positions, sheet_number=i)
            pinit.rescale(SCALE)
            world_config.init_type = pinit
            return world_config

        fitnesses = [run(setup_i(i), gui) for i in range(n_runs)]
        print("Circlinesses")
        print(fitnesses)
    else:
        fitness = run(world_config, gui)
        print(f"Circliness: {fitness}")

