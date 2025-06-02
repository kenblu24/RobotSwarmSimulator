import pymunk
import pygame
import random
import sys
from pymunk.pygame_util import DrawOptions

width = 600
height = 600


class Ball:
    def __init__(self, position, space):
        self.mass = 1
        self.shape = pymunk.Poly.create_box(None, size=(50, 10))
        self.moment = pymunk.moment_for_poly(self.mass, self.shape.get_vertices())
        self.body = pymunk.Body(self.mass, self.moment)
        self.shape.body = self.body
        leg1 = pymunk.Segment(self.body, (-20, -30), (-10, 0), 3)  # 2
        leg2 = pymunk.Segment(self.body, (20, -30), (10, 0), 3)

        self.shape.body.position = position
        space.add(self.shape, self.body, leg1, leg2)


class Ground:
    def __init__(self, space):
        self.body = pymunk.Body(0, 0, body_type=pymunk.Body.STATIC)
        self.shape = pymunk.Poly.create_box(self.body, (width, 10))
        self.shape.body.position = (width//2, 10)
        space.add(self.shape, self.body)


def main():

    pygame.init()
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption("The ball drops")
    clock = pygame.time.Clock()

    draw_options = DrawOptions(screen)

    space = pymunk.Space()
    # space.gravity = 0, -100
    ground = Ground(space)
    ball = Ball((300, 300), space)

    ball.body.angular_velocity = 1
    ball.body.apply_impulse_at_local_point((10, 0))

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit(0)
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                sys.exit(0)

        ball.body.apply_force_at_local_point((0, 10))

        screen.fill((0, 0, 0))
        space.debug_draw(draw_options)
        space.step(1/50.0)
        pygame.display.update()
        clock.tick(50)


if __name__ == '__main__':
    sys.exit(main())