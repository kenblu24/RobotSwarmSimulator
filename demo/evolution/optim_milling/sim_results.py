"""
Find the best Homogeneous Agents for Milling
"""
from ctypes import ArgumentError
from io import BytesIO
import argparse
import numpy as np
from tqdm import tqdm
from src.novel_swarms.optim.CMAES import CMAES
from src.novel_swarms.world.initialization.PredefInit import PredefinedInitialization

from .milling_search import DECISION_VARS, SCALE, BL
# from .milling_search import fitness


def metric_to_canon(genome: tuple[float, float, float, float], body_length, scale=SCALE):
    v0, w0, v1, w1 = genome
    v0 *= scale / body_length
    v1 *= scale / body_length
    return (v0, w0, v1, w1)


def canon_to_metric(genome: tuple[float, float, float, float], body_length, scale=SCALE):
    v0, w0, v1, w1 = genome
    v0 /= scale / body_length
    v1 /= scale / body_length
    return (v0, w0, v1, w1)


def run(args, genome, callback=lambda x: x) -> float:
    from src.novel_swarms.world.simulate import main as sim
    from .milling_search import get_world_generator

    world_generator = get_world_generator(args.n, args.t)
    world_config, *_ = world_generator(genome, [-1, -1, -1, -1])
    # note: world_config contains some persistent stuff like behaviors

    gui = not args.nogui

    if args.no_stop:
        world_config.stop_at = None
    else:
        world_config.stop_at = args.t

    world_config = callback(world_config)

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

    parser.add_argument("-n", type=int, default=10, help="Number of agents")
    parser.add_argument("-t", type=int, default=1000, help="Environment Horizon")
    parser.add_argument("--no-stop", action="store_true", help="If specified, the simulation will not terminate at T timesteps")
    parser.add_argument("--print", action="store_true")
    parser.add_argument("--nogui", action="store_true")
    parser.add_argument("--discrete-bins", help="How many bins to discretize the decision variables into")
    parser.add_argument('--positions', help="file containing agent positions")
    parser.add_argument("-b", "--bodylength", type=float, help="body length value")
    genome_parser = parser.add_mutually_exclusive_group(required=True)
    genome_parser.add_argument(
        "--genome",
        type=float,
        help="meters/second genome (4 floats expected: v0, w0, v1, w1)",
        default=None,
        nargs=4,
    )
    genome_parser.add_argument(
        "--normalized_genome",
        type=float,
        help="Normalized genome values (4 floats expected between [0, 1]: v0, w0, v1, w1)",
        default=None,
        nargs=4,
    )
    genome_parser.add_argument(
        "--bodylength_genome",
        type=float,
        help="Genome values (4 floats expected: v0, w0, v1, w1)",
        default=None,
        nargs=5,
    )

    args = parser.parse_args()

    bl = args.bodylength

    if args.normalized_genome:
        genome = args.normalized_genome

        if args.discrete_bins:
            increment = 1 / (int(args.discrete_bins) - 1)
            genome = CMAES.round_to_nearest(genome, increment=increment)

        genome = DECISION_VARS.from_unit_to_scaled(genome)

    elif args.genome:
        genome = args.genome

    elif args.bodylength_genome:
        genome = canon_to_metric(args.genome, bl)

    if args.discrete_bins and not args.normalized_genome:
        raise ArgumentError(args.discrete_bins, "Discrete binning can only be used with --normalized_genome")

    if args.print:
        g = genome
        print(f"v0   (m/s):\t{g[0]:>16.12f}\tv1   (m/s):\t{g[2]:>16.12f}")
        if bl is not None:
            c = metric_to_canon(g, bl)
            print(f"v0 (canon):\t{c[0]:>16.12f}\tv1 (canon):\t{c[2]:>16.12f}")
        print(f"w0 (rad/s):\t{g[1]:>16.12f}\tw1 (rad/s):\t{g[3]:>16.12f}")

    if args.positions:
        import pandas as pd
        fpath = args.positions

        with open(fpath, 'rb') as f:
            xlsx = f.read()
        xlsx = pd.ExcelFile(BytesIO(xlsx))
        sheets = xlsx.sheet_names

        n_runs = len(sheets)

        pinit = PredefinedInitialization()  # num_agents isn't used yet here

        def callback_factory(i):
            def callback(world_config):
                pinit.set_states_from_xlsx(args.positions, sheet_number=i)
                pinit.rescale(SCALE)
                world_config.init_type = pinit
                return world_config
            return callback

        def run_with_positions(i) -> float:
            return run(args, genome, callback=callback_factory(i))

        fitnesses = [run_with_positions(i) for i in tqdm(range(n_runs))]
        print("Circlinesses")
        print(fitnesses)
    else:
        fitness = run(args, genome)
        print(f"Circliness: {fitness}")

