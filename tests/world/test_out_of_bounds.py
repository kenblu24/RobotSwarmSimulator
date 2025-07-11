from pathlib import Path

from swarmsim.world.RectangularWorld import RectangularWorld, RectangularWorldConfig
from swarmsim.sensors.BinaryFOVSensor import BinaryFOVSensor
from swarmsim.agent.Agent import Agent
from swarmsim.world.simulate import main as simulate
from ..util import load_custom_yaml


def stop_after_one_step(world: RectangularWorld, n_frames: int) -> bool:
    return world.total_steps == n_frames

def test_out_of_bounds() -> None:
    yaml_path: Path = Path(__file__).parent / "out_of_bounds_world.yaml"
    spec, world_setup = load_custom_yaml(yaml_path)
    world_config = RectangularWorldConfig(**world_setup)
    world = RectangularWorld(world_config)

    last_frame: int = 0
    n_frames: int = 50
    for _ in range(1, n_frames + 1):
        simulate(world, show_gui=False,
            stop_detection=lambda w: stop_after_one_step(w, n_frames=last_frame + 1)
        )
        last_frame = world.total_steps

        # > How many agents are there?
        assert len(world.population) == 4
        # > How many agents are there?
        assert len(world.population) >= 1

        agent_info: list[tuple[int, str]] = [
            # index, name
            (0, "agent1"),
            (2, "agent2"),
        ]

        for index, name in agent_info:
            # > Is the first agent the "test-dummy" agent?
            agent1: Agent = world.population[index]
            assert agent1.name == name
            # > Does this agent have a sensor?
            assert len(agent1.sensors) == 1
            bfov1 = agent1.sensors[0]
            # > Is this agent's only sensor a BinaryFOVSensor?
            assert bfov1.as_config_dict()["type"] == "BinaryFOVSensor"
            detected1 = bfov1.current_state == 1
            assert detected1 == spec[f"{name}CanSee"]
