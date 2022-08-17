import pygame
import math, random
import numpy as np
from typing import List, Tuple
from src.behavior.GroupRotationBehavior import GroupRotationBehavior
from src.behavior.ScatterBehavior import ScatterBehavior
from src.behavior.RadialVariance import RadialVarianceBehavior
from src.behavior.AngularMomentum import AngularMomentumBehavior
from src.behavior.AverageSpeed import AverageSpeedBehavior
from src.world.World import World
from src.agent.Agent import Agent
from src.agent.DiffDriveAgent import DifferentialDriveAgent

class RectangularWorld(World):

    def __init__(self, w, h, pop_size = 20):
        super().__init__(w, h)
        self.population_size = pop_size

    def setup(self, controller=[]):
        self.population = [DifferentialDriveAgent(name=f"Bot_{i}", controller=controller) for i in range(self.population_size)]
        
        # self.population = [
        #     DifferentialDriveAgent(angle=0, controller=[-0.7, -1.0, 1.0, -1.0], x = 700, y = 700),
        #     DifferentialDriveAgent(angle=0, controller=[-0.7, -1.0, 1.0, -1.0], x = 700, y = 700)
        # ]
        # Pile On - Testing if stacked collisions work as expected
        # self.population = [
        #     DifferentialDriveAgent(angle=math.pi / 2, controller=[0.99, 1, 1, 1], x = 250, y = 250) for i in range(self.population_size)
        # ]
        
        world_radius = np.linalg.norm([self.bounded_width/2, self.bounded_height/2])
        self.behavior = [
            AverageSpeedBehavior(population = self.population),
            AngularMomentumBehavior(population = self.population, r = world_radius),
            RadialVarianceBehavior(population = self.population, r= world_radius),
            ScatterBehavior(population = self.population, r = world_radius),
            GroupRotationBehavior(population = self.population)
        ]

    def step(self):
        """
        Cycle through the entire population and take one step. Calculate Behavior if needed.
        """
        for agent in self.population:
            if not issubclass(type(agent), DifferentialDriveAgent):
                raise Exception("Agents must be subtype of Agent, not {}".format(type(agent)))
            
            agent.step(
                check_for_world_boundaries = self.withinWorldBoundaries, 
                check_for_agent_collisions = self.preventAgentCollisions, 
                check_for_sensor = self.checkForSensor
            )

        for behavior in self.behavior:
            behavior.calculate()

    def draw(self, screen):
        """
        Cycle through the entire population and draw the agents.
        """

        for agent in self.population:
            if not issubclass(type(agent), DifferentialDriveAgent):
                raise Exception("Agents must be subtype of Agent, not {}".format(type(agent)))
            agent.draw(screen)

    def getNeighborsWithinDistance(self, center: Tuple, r, excluded = None) -> List:
        """
        Given the center of a circle, find all Agents located within the circumference defined by center and r
        """
        filtered_agents = []
        for agent in self.population:
            if not issubclass(type(agent), Agent):
                raise Exception("Agents must be subtype of Agent, not {}".format(type(agent)))
            if self.distance(center, (agent.x_pos, agent.y_pos)) < r:
                if agent != excluded:
                    filtered_agents.append(agent)
        return filtered_agents
    
    def onClick(self, pos) -> None:
        neighborhood = self.getNeighborsWithinDistance(pos, self.population[0].radius)

        # Remove Highlights from everyone
        for n in self.population:
            n.is_highlighted = False

        if(len(neighborhood) == 0): 
            self.gui.set_selected(None)
            return

        if self.gui != None:
            self.gui.set_selected(neighborhood[0])
            neighborhood[0].is_highlighted = True
    
    def withinWorldBoundaries(self, agent: DifferentialDriveAgent):
        """
        Set agent position with respect to the world's boundaries and the bounding box of the agent
        """
        padding = 10

        # Prevent Left Collisions
        agent.x_pos = max(agent.radius + padding, agent.x_pos)

        # Prevent Right Collisions
        agent.x_pos = min((self.bounded_width - agent.radius - padding), agent.x_pos)

        # Prevent Top Collisions
        agent.y_pos = max(agent.radius + padding, agent.y_pos)

        # Prevent Bottom Collisions
        agent.y_pos = min((self.bounded_height - agent.radius - padding), agent.y_pos)
        
        # agent.angle += (math.pi / 720)

    def preventAgentCollisions(self, agent: DifferentialDriveAgent) -> None:
        """
        Using a set of neighbors that collide with the agent and the bounding box of those neighbors,
            push the agent to the edge of the box and continue checking for collisions until the agent is in a
            safe location OR we have expired the defined timeout.
        """

        agent_center = agent.getPosition()

        minimum_distance = agent.radius * 2
        target_distance = minimum_distance + (0.1)

        neighborhood = self.getNeighborsWithinDistance(agent_center, minimum_distance, excluded=agent)
        if(len(neighborhood) == 0):
            return

        remaining_attempts = 10

        while(len(neighborhood) > 0 and remaining_attempts > 0):

            # Check ALL Bagged agents for collisions
            for i in range(len(neighborhood)):
                
                colliding_agent = neighborhood[i]
                center_distance = self.distance(agent_center, colliding_agent.getPosition())

                if(center_distance > minimum_distance): 
                    continue

                # print(f"Overlap. A: {agent_center}, B: {colliding_agent.getPosition()}")
                distance_needed = target_distance - center_distance

                base = np.array([1, 0])
                a_to_b = agent_center - colliding_agent.getPosition()

                # If distance super close to 0, we have a problem. Add noise.
                SIGNIFICANCE = 0.0001
                if a_to_b[0] < SIGNIFICANCE and a_to_b[1] < SIGNIFICANCE:
                    MAGNITUDE = 0.001
                    dir = 1
                    if random.random() > 0.5:
                        dir = -1
                    agent.x_pos += random.random() * (dir) * MAGNITUDE
                    
                    dir = 1
                    if random.random() > 0.5:
                        dir = -1
                    agent.y_pos += random.random() * (dir) * MAGNITUDE
                    
                    agent_center = agent.getPosition()
                    center_distance = self.distance(agent_center, colliding_agent.getPosition())
                    distance_needed = target_distance - center_distance
                    a_to_b = agent_center - colliding_agent.getPosition()

                pushback = (a_to_b / np.linalg.norm(a_to_b)) * distance_needed

                # print(base, a_to_b, theta)

                delta_x = pushback[0]
                delta_y = pushback[1]

                if(math.isnan(delta_x) or math.isnan(delta_y)):
                    break

                # print(delta_x, delta_y)

                agent.x_pos += delta_x
                agent.y_pos += delta_y
                
                # agent.angle += (math.pi / 720)
                agent_center = agent.getPosition()
            
            neighborhood = self.getNeighborsWithinDistance(agent_center, minimum_distance, excluded=agent)
            remaining_attempts -= 1

    def checkForSensor(self, source_agent: DifferentialDriveAgent) -> bool:
        sensor_position = source_agent.getPosition()
        
        # Equations taken from Dunn's 3D Math Primer for Graphics, section A.12
        p_0 = np.array([sensor_position[0], sensor_position[1]])
        d = np.array(source_agent.getLOSVector())
        d_hat = d / np.linalg.norm(d)
        
        for agent in self.population:
            if agent == source_agent:
                continue

            c = np.array([agent.x_pos, agent.y_pos])
            e = c - p_0
            a = np.dot(e, d_hat)

            r_2 = agent.radius * agent.radius
            e_2 = np.dot(e, e)
            a_2 = a * a

            has_intersection = (r_2 - e_2 + a_2) >= 0
            if has_intersection and a >= 0:
                source_agent.agent_in_sight = agent
                return True

        source_agent.agent_in_sight = None
        return False

    def getBehaviorVector(self):
        behavior = np.array([s.out_average()[1] for s in self.behavior])
        return behavior

    def generalEquationOfALine(self, pointA: Tuple, pointB: Tuple) -> Tuple:
        x1, y1 = pointA
        x2, y2 = pointB

        a = y1 - y2
        b = x2 - x1
        c = (x1 - x2) * y1 + (y2 - y1) * x1
        return a, b, c

    def distance(self, pointA, pointB) -> float:
        return math.dist(pointA, pointB)
