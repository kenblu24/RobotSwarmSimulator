"""
test world startup
"""

import pathlib as pl
import argparse
from io import BytesIO
from ctypes import ArgumentError

import numpy as np
from tqdm import tqdm

from swarmsim import yaml
from swarmsim.config import get_agent_class
from swarmsim.agent.control.Controller import Controller
from swarmsim.world.spawners.AgentSpawner import UniformAgentSpawner
from swarmsim.world.RectangularWorld import RectangularWorldConfig

from swarmsim.metrics import Circliness

# from swarmsim.world.simulate import main as sim

# from swarmsim.world.initialization.FixedInit import FixedInitialization
# from swarmsim.world.spawners.ExcelSpawner import ExcelSpawner

SCALE = 1

MS_MAX = 0.2  # max speed in m/s
BL = 0.151  # body length
BLS_MAX = MS_MAX / BL  # max speed in body lengths per second

PERFECT_CIRCLE_SCORE = -1.0
CIRCLINESS_HISTORY = 450

code_dir = pl.Path(__file__).parent.resolve()


def fitness(world_set):
    total = 0
    for w in world_set:
        total += w.metrics[0].out_average()[1]
    avg = total / len(world_set)
    return -avg


def get_world_generator(n_agents, horizon, round_genome=False):
    def gene_to_world(genome, hash_val):
        # from swarmsim.config import register_agent_type, store
        # from swarmsim.agent.StaticAgent import StaticAgent, StaticAgentConfig

        # register_agent_type("StaticAgent", StaticAgent, StaticAgentConfig)

        # build agent config and inject controller
        with open(code_dir / 'turbopi.yaml', 'r') as f:
            agent_config = yaml.load(f)
        _agent_cls, agent_config = get_agent_class(agent_config)
        agent_config.controller = {"type": Controller, "controller": genome}

        # get world config from file
        world_config = RectangularWorldConfig.from_yaml(code_dir / 'world.yaml')

        # build spawner config and inject agent config
        spawner_config = {
            'type': UniformAgentSpawner,
            'agent': agent_config,
            'n': n_agents,
            'avoid_overlap': True,
            'facing': "away",
            'region': [[3, 3], [3, 6], [6, 6], [6, 3]],
        }

        # modify world config
        # world.seed = 0
        world_config.metrics = [
            # Circliness(avg_history_max=CIRCLINESS_HISTORY)
        ]
        # world.population_size = n_agents
        world_config.stop_at = horizon
        world_config.detectable_walls = False
        world_config.spawners.append(spawner_config)

        world_config.factor_zoom(zoom=SCALE)
        # world.addAgentConfig(goal_agent)
        world_config.metadata = {"hash": hash(tuple(list(hash_val)))}
        worlds = [world_config]

        return worlds

    return gene_to_world


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
    from swarmsim.world.simulate import main as sim
    # from .milling_search import get_world_generator

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
        return w.metrics[0].out_average()[1]
    except BaseException:
        pass


if __name__ == "__main__":
    """
    Example usage:
    `python -m demo.evolution.optim_milling.sim_results --v0 0.1531 --w0 0.3439 --v1 0.1485 --w1 0.1031 --n 10 --t 1000`
    """

    parser = argparse.ArgumentParser()

    parser.add_argument("-n", type=int, default=0, help="Number of agents")
    parser.add_argument("-t", type=int, default=1000, help="Environment Horizon")
    parser.add_argument("--no-stop", action="store_true", help="If specified, the simulation will not terminate at T timesteps")
    parser.add_argument("--print", action="store_true")
    parser.add_argument("--nogui", action="store_true")
    parser.add_argument("--discrete-bins", help="How many bins to discretize the decision variables into")
    parser.add_argument("--positions", help="file containing agent positions")
    parser.add_argument("-b", "--bodylength", type=float, help="body length value")
    genome_parser = parser.add_mutually_exclusive_group(required=False)
    genome_parser.add_argument(
        "--genome",
        type=float,
        help="meters/second genome (4 floats expected: v0, w0, v1, w1)",
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

    if args.genome:
        genome = args.genome

    elif args.bodylength_genome:
        genome = canon_to_metric(args.genome, bl)
    else:
        genome = [0.0798, 0.4, 0.1755, 0.0]

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

        with open(fpath, "rb") as f:
            xlsx = f.read()
        xlsx = pd.ExcelFile(BytesIO(xlsx))
        sheets = xlsx.sheet_names

        n_runs = len(sheets)

        # pinit = PredefinedInitialization()  # num_agents isn't used yet here

        def callback_factory(i):
            def callback(world_config):
                # pinit.set_states_from_xlsx(args.positions, sheet_number=i)
                # pinit.rescale(SCALE)
                # world_config.init_type = pinit
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


"""
Exact Icra Command:
python -m demo.evolution.optim_milling.milling_search --name "test_mill_optim" --n 10 --t 1000 --processes 15 --pop-size 15 --iters 100
"""
