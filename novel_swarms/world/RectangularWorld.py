import math
import random
import numpy as np
from typing import List, Tuple
import pygame.draw
from ..agent.Agent import Agent
from ..agent.DiffDriveAgent import DifferentialDriveAgent
from ..config.WorldConfig import RectangularWorldConfig
from ..agent.AgentFactory import AgentFactory
from .World import World
from ..util.timer import Timer


def distance(pointA, pointB) -> float:
    return math.dist(pointA, pointB)


class RectangularWorld(World):
    def __init__(self, config: RectangularWorldConfig = None):
        if config is None:
            raise Exception("RectangularWorld must be instantiated with a WorldConfig class")

        super().__init__(config.w, config.h)
        self.config = config
        self.population_size = config.population_size
        self.behavior = config.behavior
        self.padding = config.padding
        self.objects = config.objects
        self.goals = config.goals

        if config.seed is not None:
            # print(f"World Instantiated with Seed: {config.seed}")
            # print(f"TESTING RAND: {random.random()}")
            random.seed(config.seed)

        self.population = [
            AgentFactory.create(config.agentConfig, name=f"{i}") for i in range(self.population_size)
        ]

        ac = config.agentConfig
        if config.defined_start:
            for i in range(len(config.agent_init)):
                init = config.agent_init[i]
                noise_x = ((np.random.random() * 2) - 1) * 20
                noise_y = ((np.random.random() * 2) - 1) * 20
                noise_theta = ((np.random.random() * 2) - 1) * (np.pi / 8)
                self.population[i].x_pos = init[0] + noise_x
                self.population[i].y_pos = init[1] + noise_y
                if len(init) > 2:
                    self.population[i].angle = init[2] + noise_theta

        elif ac.x is None and config.seed is not None:
            for agent in self.population:
                agent.x_pos = random.randint(0 + ac.agent_radius, ac.world.w - ac.agent_radius)
                agent.y_pos = random.randint(0 + ac.agent_radius, ac.world.h - ac.agent_radius)
                agent.angle = random.random() * 2 * math.pi

        # print([(a.x_pos, a.y_pos, a.angle) for a in self.population])

        for i in range(len(self.objects)):
            self.objects[i].world = self

        self.behavior = config.behavior
        for b in self.behavior:
            b.attach_world(self)

    def step(self):
        """
        Cycle through the entire population and take one step. Calculate Behavior if needed.
        """
        agent_step_timer = Timer("Population Step")
        for agent in self.population:
            if not issubclass(type(agent), Agent):
                raise Exception("Agents must be subtype of Agent, not {}".format(type(agent)))

            agent.step(
                check_for_world_boundaries=self.withinWorldBoundaries,
                check_for_agent_collisions=self.preventAgentCollisions,
                world=self
            )
        # agent_step_timer.check_watch()

        behavior_timer = Timer("Behavior Calculation Step")
        for behavior in self.behavior:
            behavior.calculate()
        # behavior_timer.check_watch()

    def draw(self, screen):
        """
        Cycle through the entire population and draw the agents. Draw Environment Walls if needed.
        """
        if self.config.show_walls:
            p = self.config.padding
            w = self.config.w
            h = self.config.h
            pygame.draw.rect(screen, (200, 200, 200), pygame.Rect((p, p), (w - (2 * p), h - (2 * p))), 1)

        for world_obj in self.objects:
            world_obj.draw(screen)

        for world_goal in self.goals:
            world_goal.draw(screen)

        for agent in self.population:
            if not issubclass(type(agent), Agent):
                raise Exception("Agents must be subtype of Agent, not {}".format(type(agent)))
            agent.draw(screen)

    def getNeighborsWithinDistance(self, center: Tuple, r, excluded=None) -> List:
        """
        Given the center of a circle, find all Agents located within the circumference defined by center and r
        """
        filtered_agents = []
        for agent in self.population:
            if not issubclass(type(agent), Agent):
                raise Exception("Agents must be subtype of Agent, not {}".format(type(agent)))
            if distance(center, (agent.x_pos, agent.y_pos)) < r:
                if agent != excluded:
                    filtered_agents.append(agent)
        return filtered_agents

    def onClick(self, pos) -> None:
        neighborhood = self.getNeighborsWithinDistance(pos, self.population[0].radius)

        # Remove Highlights from all agents
        for n in self.population:
            n.is_highlighted = False

        if self.gui is not None and len(neighborhood) == 0:
            self.gui.set_selected(None)
            return

        if self.gui is not None:
            self.gui.set_selected(neighborhood[0])
            neighborhood[0].is_highlighted = True

    def withinWorldBoundaries(self, agent: DifferentialDriveAgent):
        """
        Set agent position with respect to the world's boundaries and the bounding box of the agent
        """
        padding = self.padding

        # Prevent Left Collisions
        agent.x_pos = max(agent.radius + padding, agent.x_pos)

        # Prevent Right Collisions
        agent.x_pos = min((self.bounded_width - agent.radius - padding), agent.x_pos)

        # Prevent Top Collisions
        agent.y_pos = max(agent.radius + padding, agent.y_pos)

        # Prevent Bottom Collisions
        agent.y_pos = min((self.bounded_height - agent.radius - padding), agent.y_pos)

        # agent.angle += (math.pi / 720)
        self.handleWallCollisions(agent)

    def handleWallCollisions(self, agent: DifferentialDriveAgent):
        # Check for distances between the agent and the line segments
        for obj in self.objects:
            segs = obj.get_sensing_segments()
            c = (agent.x_pos, agent.y_pos)
            for p1, p2 in segs:
                # From https://stackoverflow.com/questions/24727773/detecting-rectangle-collision-with-a-circle
                x1, y1 = p1
                x2, y2 = p2
                x3, y3 = c
                px = x2 - x1
                py = y2 - y1

                something = px * px + py * py

                u = ((x3 - x1) * px + (y3 - y1) * py) / float(something)

                if u > 1:
                    u = 1
                elif u < 0:
                    u = 0

                x = x1 + u * px
                y = y1 + u * py

                dx = x - x3
                dy = y - y3

                dist = math.sqrt(dx * dx + dy * dy)
                if dist < agent.radius:
                    agent.y_pos -= dy
                    agent.x_pos -= dx

    def preventAgentCollisions(self, agent: DifferentialDriveAgent, forward_freeze=False) -> None:
        agent_center = agent.getPosition()
        minimum_distance = agent.radius * 2
        target_distance = minimum_distance + 0.001

        neighborhood = self.getNeighborsWithinDistance(agent_center, minimum_distance, excluded=agent)
        if len(neighborhood) == 0:
            return

        remaining_attempts = 10
        while len(neighborhood) > 0 and remaining_attempts > 0:

            # Check ALL Bagged agents for collisions
            for i in range(len(neighborhood)):
                colliding_agent = neighborhood[i]
                center_distance = distance(agent_center, colliding_agent.getPosition())

                if center_distance > minimum_distance:
                    # colliding_agent.collision_flag = False
                    continue

                if agent.stop_on_collision:
                    agent.stopped_duration = 3

                agent.collision_flag = True
                colliding_agent.collision_flag = True
                if colliding_agent.detection_id == 2:
                    agent.detection_id = 2

                # print(f"Overlap. A: {agent_center}, B: {colliding_agent.getPosition()}")
                distance_needed = target_distance - center_distance
                a_to_b = colliding_agent.getPosition() - agent_center
                b_to_a = agent_center - colliding_agent.getPosition()

                # Check to see if the collision takes place in the forward facing direction
                if forward_freeze:
                    heading = agent.getFrontalPoint()
                    dot = np.dot(a_to_b, heading)
                    mag_a = np.linalg.norm(a_to_b)
                    mag_b = np.linalg.norm(heading)
                    angle = np.arccos(dot / (mag_a * mag_b))
                    degs = np.degrees(abs(angle))
                    if degs < 30:
                        # print(f"Collision at angle {degs}.")
                        agent.stopped_duration = 2
                        continue

                    # Now Calculate B_to_A
                    heading = colliding_agent.getFrontalPoint()
                    dot = np.dot(b_to_a, heading)
                    mag_a = np.linalg.norm(b_to_a)
                    mag_b = np.linalg.norm(heading)
                    angle = np.arccos(dot / (mag_a * mag_b))
                    degs = np.degrees(abs(angle))
                    if degs < 30:
                        # print(f"Collision at angle {degs}.")
                        colliding_agent.stopped_duration = 2
                        continue

                # If distance super close to 0, we have a problem. Add noise.
                SIGNIFICANCE = 0.0001
                if b_to_a[0] < SIGNIFICANCE and b_to_a[1] < SIGNIFICANCE:
                    MAGNITUDE = 0.001
                    direction = 1
                    if random.random() > 0.5:
                        direction = -1
                    agent.x_pos += random.random() * direction * MAGNITUDE

                    direction = 1
                    if random.random() > 0.5:
                        direction = -1
                    agent.y_pos += random.random() * direction * MAGNITUDE

                    agent_center = agent.getPosition()
                    center_distance = distance(agent_center, colliding_agent.getPosition())
                    distance_needed = target_distance - center_distance
                    b_to_a = agent_center - colliding_agent.getPosition()

                pushback = (b_to_a / np.linalg.norm(b_to_a)) * distance_needed

                # print(base, a_to_b, theta)
                delta_x = pushback[0]
                delta_y = pushback[1]

                if math.isnan(delta_x) or math.isnan(delta_y):
                    break

                # print(delta_x, delta_y)

                agent.x_pos += delta_x
                agent.y_pos += delta_y

                # agent.angle += (math.pi / 720)
                agent_center = agent.getPosition()

            neighborhood = self.getNeighborsWithinDistance(agent_center, minimum_distance, excluded=agent)
            remaining_attempts -= 1

    def getBehaviorVector(self):
        behavior = np.array([s.out_average()[1] for s in self.behavior])
        return behavior
