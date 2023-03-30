import random
import math
from novel_swarms.sensors.AbstractSensor import AbstractSensor
from novel_swarms.sensors.GenomeDependentSensor import GenomeBinarySensor
from novel_swarms.sensors.StaticSensor import StaticSensor
from novel_swarms.behavior.AngularMomentum import AngularMomentumBehavior
from novel_swarms.behavior.AverageSpeed import AverageSpeedBehavior
from novel_swarms.behavior.GroupRotationBehavior import GroupRotationBehavior
from novel_swarms.behavior.RadialVariance import RadialVarianceBehavior
from novel_swarms.behavior.ScatterBehavior import ScatterBehavior
from novel_swarms.sensors.BinaryLOSSensor import BinaryLOSSensor
from novel_swarms.behavior.DistanceToGoal import DistanceToGoal
from novel_swarms.world.goals.Goal import AreaGoal
from novel_swarms.world.simulate import main as simulate
from novel_swarms.sensors.BinaryFOVSensor import BinaryFOVSensor
from novel_swarms.sensors.SensorSet import SensorSet
from novel_swarms.config.AgentConfig import DiffDriveAgentConfig, StaticAgentConfig, UnicycleAgentConfig, LevyAgentConfig
from novel_swarms.config.WorldConfig import RectangularWorldConfig
from novel_swarms.config.HeterogenSwarmConfig import HeterogeneousSwarmConfig
from novel_swarms.world.obstacles.Wall import Wall
import numpy as np

if __name__ == "__main__":

    AGGREGATION_CONTROLLER = [12.5, 0.5, 12.5, -0.5]
    SEED = None
    GUI_PADDING = 15
    BL = 15.1
    N_AGENTS = 10
    WIDTH, HEIGHT = int(BL * 29.8), int(BL * 29.8)

    sensors = SensorSet([
        BinaryFOVSensor(
            theta=14 / 2,
            distance=(BL * 5),
            bias=-4,
            degrees=True,
            false_positive=0.0,
            false_negative=0.0,
            # Rectangle Representing Environment Boundaries
            walls=[[GUI_PADDING, GUI_PADDING], [GUI_PADDING + WIDTH, GUI_PADDING + HEIGHT]],
            wall_sensing_range=(BL * 4),
            time_step_between_sensing=2,
        )
    ])

    base_config = UnicycleAgentConfig(
        controller=AGGREGATION_CONTROLLER,
        agent_radius=BL / 2,
        dt=0.13,  # 130ms sampling period
        sensors=sensors,
        seed=None,
        idiosyncrasies=True,
        body_filled=True,
        body_color=(255, 0, 0),
        trace_length=10000,
        trace_color=(255, 255, 255)
    )

    agent_levy = LevyAgentConfig(
        base_config,
        levy_constant=1.1,
        turning_rate=2.0,
        forward_rate=12.5,
        step_scale=30.0
    )

    agent_config_b = UnicycleAgentConfig(
        controller=AGGREGATION_CONTROLLER,
        agent_radius=BL / 2,
        dt=0.13,  # 130ms sampling period
        sensors=sensors,
        seed=None,
        idiosyncrasies=False,
        body_filled=True,
        body_color=(255, 0, 0)
    )

    heterogeneous_swarm_config = HeterogeneousSwarmConfig()
    heterogeneous_swarm_config.add_sub_populuation(agent_levy, 1)
    heterogeneous_swarm_config.add_sub_populuation(agent_config_b, 0)

    behavior = [
        # DistanceToGoal()
    ]

    goals = [AreaGoal(200, 200, 75, 20)]
    objects = [
        Wall(None, 180, 193, 120, 2),
        Wall(None, 180, 193, 2, 91),
        Wall(None, 300, 193, 2, 91),
        Wall(None, 238, 350, 2, 135),
    ]

    initial_conditions = [(238, 50, 0)]

    world_config = RectangularWorldConfig(
        size=(500, 500),
        n_agents=1,
        seed=SEED,
        behavior=behavior,
        show_walls=True,
        collide_walls=True,
        agent_initialization=initial_conditions,
        agentConfig=heterogeneous_swarm_config,
        padding=15,
        objects=objects,
        goals=goals,
    )

    # def stop_when(world):
    #     if world.total_steps > 100 and world.behavior[0].out_average()[1] == 0:
    #         return True
    #     return False
    #
    # time_to_goal = []
    # for i in range(100):
    #     world = simulate(world_config=world_config, stop_detection=stop_when, step_size=10)
    #     time_to_goal.append(world.total_steps)
    #     print(time_to_goal)

    world = simulate(world_config=world_config, step_size=5)