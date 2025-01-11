import math
from functools import partial

from ..config import store
from ..util.geometry.svg_extraction import SVG, remove_classes, first_match
import random
from dataclasses import dataclass

import numpy as np
import pygame
import pygame.draw

from ..agent.Agent import Agent

# from ..agent.DiffDriveAgent import DifferentialDriveAgent
# from .. agent.HumanAgent import HumanDrivenAgent
# from ..config.WorldConfig import RectangularWorldConfig
# from ..agent.AgentFactory import AgentFactory
# from ..config.HeterogenSwarmConfig import HeterogeneousSwarmConfig
from .World import World, AbstractWorldConfig
from ..config import associated_type, filter_unexpected_fields
from ..util.timer import Timer
from ..util.collider.AABB import AABB
from .goals.Goal import CylinderGoal
from .objects.Wall import Wall

# typing
from typing import List, Tuple

COLLISION_CLASSES = ['nocollide', 'collide']

min_zoom = 0.0001

distance = math.dist


remove_special_classes = partial(remove_classes, classes=COLLISION_CLASSES + ['Layer_1'])


def get_collision_config(collides: bool | str | None):
    if collides is None:
        return {}
    if isinstance(collides, str):
        match collides:
            case 'collide':
                collides = True
            case 'nocollide':
                collides = False
            case _:
                raise ValueError(f"Unknown collision value: {collides}")
    return {'collides': collides}


@associated_type("RectangularWorld")
@filter_unexpected_fields
@dataclass
class RectangularWorldConfig(AbstractWorldConfig):
    size: tuple[float, float] | np.ndarray = (5, 5)
    show_walls: bool = True
    collide_walls: bool = True
    detectable_walls: bool = False
    time_step: float = 1 / 60

    def factor_zoom(self, zoom):
        print("RectangularWorld Factor_Zoom called", zoom, self.size)
        self.size = np.asarray(self.size) * zoom
        self.size *= zoom
        for goal in self.goals:
            goal.center[0] *= zoom
            goal.center[1] *= zoom
            goal.r *= zoom
            goal.range *= zoom
        # self.init_type.rescale(zoom)


class RectangularWorld(World):
    def __init__(self, config: RectangularWorldConfig):
        # if config is None:
        #     raise Exception("RectangularWorld must be instantiated with a WorldConfig class")

        super().__init__(config)
        self.config = config
        # self.padding = config.padding
        # self.population_size = config.population_size
        self.zoom = 1.0
        self._original_zoom = 1.0
        self.pos = np.array([0.0, 0.0])
        self.mouse_position = np.array([0, 0])
        self._mouse_dragging_last_pos = np.array([0.0, 0.0])
        self.dt = config.time_step

        self.selected = None
        self.highlighted_set = []
        self.human_controlled = []

        if config.seed is not None:
            # print(f"World Instantiated with Seed: {config.seed}")
            # print(f"TESTING RAND: {random.random()}")
            random.seed(config.seed)

        # self.heterogeneous = False

        self.population = []

        StaticObject, StaticObjectConfig = store.agent_types['StaticObject']
        for entry in config.objects:
            if not isinstance(entry, dict):
                continue
            # check if entry contains a "from_svg" key with a string value
            if 'from_svg' not in entry:
                continue
            if isinstance((svg:=entry['from_svg']), str):
                svg = SVG(svg)
                paths = svg.get_polygons()
                paths += svg.get_rects()
                # create StaticObject for each polygon/rect/circle
                for path, classes in paths:
                    points = np.asarray(path, dtype=np.float64)
                    collides = get_collision_config(first_match(classes, COLLISION_CLASSES))
                    classes = ' '.join(remove_special_classes(classes))
                    agent_config = StaticObjectConfig(points=points, team=classes, **collides)
                    self.objects.append(StaticObject(agent_config, self))
                circles = svg.get_circles()
                for circle, classes in circles:
                    x, y, r = circle
                    collides = get_collision_config(first_match(classes, COLLISION_CLASSES))
                    classes = ' '.join(remove_special_classes(classes))
                    agent_config = StaticObjectConfig(position=np.array([x, y]), agent_radius=r,
                                                      team=classes, **collides)
                    self.objects.append(StaticObject(agent_config, self))
            else:
                raise TypeError("Expected a string value for 'from_svg' key in 'objects' list.")

    def step(self):
        """
        Cycle through the entire population and take one step. Calculate Behavior if needed.
        """
        super().step()
        # agent_step_timer = Timer("Population Step")

        self.spawners = [s for s in self.spawners if not s.mark_for_deletion]
        for spawner in self.spawners:
            spawner.step()

        for agent in self.population:
            if not issubclass(type(agent), Agent):
                msg = f"Agents must be subtype of Agent, not {type(agent)}"
                raise Exception(msg)

            agent.step(
                check_for_world_boundaries=self.withinWorldBoundaries if self.config.collide_walls else None,
                check_for_agent_collisions=self.preventAgentCollisions,
                world=self,
            )
            self.handleGoalCollisions(agent)
        # agent_step_timer.check_watch()

        # behavior_timer = Timer("Behavior Calculation Step")
        for metric in self.metrics:
            metric.calculate()
        # behavior_timer.check_watch()

    def draw(self, screen, offset=None):
        """
        Cycle through the entire population and draw the agents. Draw Environment Walls if needed.
        """
        if offset is None:
            offset = (self.pos, self.zoom)
        # pan, zoom = np.asarray(offset[0], dtype=np.int32), offset[1]
        # if self.config.show_walls:
        #     p = self.config.padding * zoom
        #     size = np.asarray(self.config.size) * zoom
        #     pad = np.array((p, p))
        #     a = pan + pad  # upper left corner
        #     b = pan + size - pad * 2
        #     pygame.draw.rect(screen, (200, 200, 200), pygame.Rect(a, b), 1)

        for world_obj in self.objects:
            world_obj.draw(screen, offset)

        for world_goal in self.goals:
            world_goal.draw(screen, offset)

        for agent in self.population:
            agent.draw(screen, offset)

    def getNeighborsWithinDistance(self, center: Tuple, r, excluded=None) -> List:
        """
        Given the center of a circle, find all Agents located within the circumference defined by center and r
        """
        filtered_agents = []
        for agent in self.population:
            if not issubclass(type(agent), Agent):
                msg = f"Agents must be subtype of Agent, not {type(agent)}"
                raise Exception(msg)
            if distance(center, (agent.get_x_pos(), agent.get_y_pos())) < r:
                if agent != excluded:
                    filtered_agents.append(agent)
        return filtered_agents

    def onClick(self, event) -> None:
        viewport_pos = np.asarray(event.pos)
        pos = (viewport_pos - self.pos) / self.zoom
        d = self.population[0].radius * 1.1
        neighborhood = self.getNeighborsWithinDistance(pos, d)

        # Remove Highlights from all agents
        if self.selected is not None:
            self.selected.is_highlighted = False

        if len(neighborhood) == 0:
            self.selected = None
            if self.gui is not None:
                self.gui.set_selected(None)
            return

        self.selected = neighborhood[0]
        if self.gui is not None:
            self.gui.set_selected(neighborhood[0])
            neighborhood[0].is_highlighted = True

    def onZoom(self, mouse_event, scroll_event):
        if not (mouse_event.type == pygame.MOUSEBUTTONUP and scroll_event.type == pygame.MOUSEWHEEL):
            raise TypeError("Expected a mouse button up and scroll event.")

        pos = np.asarray(mouse_event.pos)
        v = scroll_event.precise_y
        self.do_zoom(pos, v)

    def do_zoom(self, point, v):
        v *= 0.4
        old_zoom = self.zoom
        self.zoom = self.zoom * (2**v)
        self.zoom = max(self.zoom, min_zoom)
        # print(f"zoom: {round(old_zoom, 6): >10f} --> {round(self.zoom, 6): >10f}")
        point = point.astype(np.float64)
        center_px = np.asarray(self.config.size) / 2
        point -= center_px
        self.pos = (self.pos - point) * self.zoom / old_zoom + point

    def zoom_reset(self):
        self.zoom = self.original_zoom
        self.pos = np.array([0.0, 0.0])

    def on_mouse(self, pos):
        viewport_pos = self.mouse_position = np.asarray(pos)
        pos = (viewport_pos - self.pos) / self.zoom
        if self.gui:
            self.gui.set_mouse_world_coordinates(pos)

    @property
    def original_zoom(self):
        return self._original_zoom

    @original_zoom.setter
    def original_zoom(self, value):
        self._original_zoom = value
        self.zoom = value

    def handle_middle_mouse_events(self, events):
        for event in events:
            if event.type == pygame.MOUSEBUTTONDOWN:
                self._mouse_dragging_last_pos = np.asarray(event.pos)

    def handle_middle_mouse_held(self, pos):
        pos = np.asarray(pos)
        delta = pos - self._mouse_dragging_last_pos
        self.pos += delta
        self._mouse_dragging_last_pos = pos

    def withinWorldBoundaries(self, agent: Agent):
        """
        Set agent position with respect to the world's boundaries and the bounding box of the agent
        """
        # padding = self.padding

        old_x, old_y = agent.get_x_pos(), agent.get_y_pos()

        # # Prevent Left Collisions
        # agent.set_x_pos(max(agent.radius + padding, agent.get_x_pos()))

        # # Prevent Right Collisions
        # agent.set_x_pos(min((self.bounded_width - agent.radius - padding), agent.get_x_pos()))

        # # Prevent Top Collisions
        # agent.set_y_pos(max(agent.radius + padding, agent.get_y_pos()))

        # # Prevent Bottom Collisions
        # agent.set_y_pos(min((self.bounded_height - agent.radius - padding), agent.get_y_pos()))

        # agent.angle += (math.pi / 720)
        self.handleWallCollisions(agent)

        if agent.get_x_pos() != old_x or agent.get_y_pos() != old_y:
            return True
        return False

    def handleGoalCollisions(self, agent):
        for goal in self.goals:
            if isinstance(goal, CylinderGoal):
                correction = agent.build_collider().collision_then_correction(goal.get_collider())
                if correction is not None:
                    agent.set_x_pos(agent.get_x_pos() + correction[0])
                    agent.set_y_pos(agent.get_y_pos() + correction[1])

    def handleWallCollisions(self, agent: Agent):
        # Check for distances between the agent and the line segments
        in_collision = False
        for obj in self.objects:
            segs = obj.get_sensing_segments()
            c = (agent.get_x_pos(), agent.get_y_pos())
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
                    in_collision = True
                    agent.set_y_pos(agent.get_y_pos() - (np.sign(dy) * (agent.radius - abs(dy) + 1)))
                    agent.set_x_pos(agent.get_x_pos() - (np.sign(dx) * (agent.radius - abs(dx) + 1)))

                # dx = x - x3 - agent.radius
                # if dx < 0:
                #     in_collision = True
                #     agent.set_x_pos(agent.get_x_pos() - dx)
                # dy = y - y3 - agent.radius
                # if dy < 0:
                #     in_collision = True
                #     agent.set_y_pos(agent.get_y_pos() - dy)

        return in_collision

    def preventAgentCollisions(self, agent: Agent, forward_freeze=False) -> None:
        agent.pos = agent.getPosition()
        minimum_distance = agent.radius * 2
        target_distance = minimum_distance + 0.001

        # neighborhood = self.getNeighborsWithinDistance(agent_center, minimum_distance, excluded=agent)
        # if len(neighborhood) == 0:
        #     return

        for _ in range(10):  # limit attempts
            collided_agents = [other for other in self.population if agent != other
                               and agent.get_aabb().intersects_bb(other.get_aabb())]
            if not collided_agents:
                break
            # Check ALL Bagged agents for collisions
            for other in collided_agents:
                center_distance = distance(agent.pos, other.getPosition())
                if center_distance > minimum_distance:
                    # colliding_agent.collision_flag = False
                    continue

                if agent.stop_on_collision:
                    agent.stopped_duration = 3

                agent.collision_flag = True
                other.collision_flag = True
                if other.detection_id == 2:
                    agent.detection_id = 2

                # print(f"Overlap. A: {agent.pos}, B: {colliding_agent.pos}")
                distance_needed = target_distance - center_distance
                # a_to_b = other.pos - agent.pos
                b_to_a = agent.pos - other.pos

                # Check to see if the collision takes place in the forward facing direction
                if forward_freeze and self.collision_forward(agent, other):
                    continue

                # If distance super close to 0, we have a problem. Add noise.
                SIGNIFICANCE = 0.0001
                if b_to_a[0] < SIGNIFICANCE and b_to_a[1] < SIGNIFICANCE:
                    MAGNITUDE = (0.001, 0.001)
                    # direction = 1
                    # if random.random() > 0.5:
                    #     direction = -1
                    # agent.set_x_pos(agent.get_x_pos() + (random.random() * direction * MAGNITUDE))

                    # direction = 1
                    # if random.random() > 0.5:
                    #     direction = -1
                    # agent.set_y_pos(agent.get_y_pos() + (random.random() * direction * MAGNITUDE))
                    agent.pos += self.rng.uniform(-MAGNITUDE, MAGNITUDE)

                    center_distance = distance(agent.pos, other.pos)
                    distance_needed = target_distance - center_distance
                    b_to_a = agent.pos - other.pos

                pushback = (b_to_a / np.linalg.norm(b_to_a)) * distance_needed

                if np.isnan(pushback).any():
                    break

                agent.pos += pushback

    def getAgentsMatchingYRange(self, bb: AABB):
        ret = []
        for agent in self.population:
            if bb.in_y_range(agent.get_aabb()):
                ret.append(agent)
        return ret

    def getBehaviorVector(self):
        behavior = np.array([s.out_average()[1] for s in self.metrics])
        return behavior

    @property
    def behavior_dict(self):
        return {s.name: s for s in self.metrics}

    def removeAgent(self, agent):
        agent.deleted = True
        self.population.remove(agent)

    def collision_forward(self, agent, colliding_agent):
        a_to_b = colliding_agent.getPosition() - agent.getPosition()
        b_to_a = agent.getPosition() - colliding_agent.getPosition()
        heading = agent.getFrontalPoint()
        dot = np.dot(a_to_b, heading)
        mag_a = np.linalg.norm(a_to_b)
        mag_b = np.linalg.norm(heading)
        angle = np.arccos(dot / (mag_a * mag_b))
        degs = np.degrees(abs(angle))
        if degs < 30:
            # print(f"Collision at angle {degs}.")
            agent.stopped_duration = 2
            return True

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
            return True
        return False

    def handle_key_press(self, event):
        for a in self.population:
            a.on_key_press(event)

        if self.selected is not None:
            if event.key == pygame.K_l:
                self.selected.simulate_error("Death")
            if event.key == pygame.K_o:
                self.selected.simulate_error("Divergence")
            if event.key == pygame.K_p:
                self.removeAgent(self.selected)
            if event.key == pygame.K_a:
                COLORS = [(247, 146, 86), (146, 220, 229), (235, 185, 223), (251, 209, 162), (99, 105, 209)]
                self.selected.body_color = COLORS[len(self.highlighted_set) % len(COLORS)]
                self.selected.is_highlighted = True
                self.highlighted_set.append(self.selected)
            if event.key == pygame.K_h:
                i = self.population.index(self.selected)
                # new_human = HumanDrivenAgent(self.selected.config)
                # new_human.x_pos = self.selected.x_pos
                # new_human.y_pos = self.selected.y_pos
                # new_human.angle = self.selected.angle
                # self.population[i] = new_human
                # self.human_controlled.append(new_human)

        if event.key == pygame.K_c:
            for agent in self.highlighted_set:
                agent.is_highlighted = False
                agent.body_color = agent.config.body_color
            for agent in self.human_controlled:
                i = self.population.index(agent)
                new_bot = Agent(agent.config)
                new_bot.x_pos = agent.get_x_pos()
                new_bot.y_pos = agent.get_y_pos()
                new_bot.angle = agent.angle
                self.population[i] = new_bot
            self.human_controlled = []
            self.highlighted_set = []

    def handle_held_keys(self, keys):
        for agent in self.human_controlled:
            agent.handle_key_press(keys)

    def as_config_dict(self):
        return self.config.as_dict()
