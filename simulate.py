import math

import pygame
from src.gui.agentGUI import DifferentialDriveGUI
from src.world.RectangularWorld import RectangularWorld

screen = None
FRAMERATE = 128
WORLD_WIDTH = 500
WORLD_HEIGHT = 500
GUI_WIDTH = 200


# define a main function
def main(controller=None):
    # initialize the pygame module
    pygame.init()
    # load and set the logo
    # logo = pygame.image.load("logo32x32.png")
    # pygame.display.set_icon(logo)
    pygame.display.set_caption("Swarm Simulation")

    # screen must be global so that other modules can access + draw to the window
    global screen
    screen = pygame.display.set_mode((WORLD_WIDTH + GUI_WIDTH, WORLD_HEIGHT))

    # define a variable to control the main loop
    running = True
    paused = False

    # Create the simulation world
    world = RectangularWorld(WORLD_WIDTH, WORLD_HEIGHT, pop_size=5)
    # world.setup(controller=[-0.7, 0.3, 1.0, 1.0])
    # world.setup(controller=[-0.7, -1.0, 1.0, -1.0])

    if controller is not None:
        world.setup(controller=controller, seed=10)
    else:
        # world.setup(controller=[-0.7, 0.3, 1.0, 1.0])
        # world.setup(controller=[-0.7, -1.0, 1.0, -1.0])
        # world.setup(controller=[1, 0.58008062, 0.9649, 0.7992])
        world.setup(controller=[0.64, 0.75, 0.06, 0.09])  # "Flocking" Behavior

    # Create the GUI
    gui = DifferentialDriveGUI(x=WORLD_WIDTH, y=0, h=WORLD_HEIGHT, w=GUI_WIDTH)
    gui.set_title("Differential Drive")


    # Attach the world to the gui and visa versa
    gui.set_world(world)
    world.attach_gui(gui)

    total_allowed_steps = None
    steps_taken = 0
    steps_per_frame = 1

    # Main loop
    while running:
        # Looped Event Handling
        for event in pygame.event.get():
            # Cancel the game loop if user quits the GUI
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    paused = not paused
                    print(f"Paused on Simulation Step: {steps_taken}")
                if event.key == pygame.K_RSHIFT:
                    steps_per_frame *= 2
                    steps_per_frame = min(steps_per_frame, 50)
                if event.key == pygame.K_LSHIFT and steps_per_frame > 1:
                    steps_per_frame /= 2
                    steps_per_frame = round(steps_per_frame)
            if event.type == pygame.MOUSEBUTTONUP:
                pos = pygame.mouse.get_pos()
                world.onClick(pos)

        if paused:
            pygame.time.Clock().tick(FRAMERATE)
            continue

        # Calculate Steps - Stop if we reach desired frame
        for _ in range(steps_per_frame):
            if total_allowed_steps is not None:
                if steps_taken > total_allowed_steps:
                    break
            world.step()
            steps_taken += 1
            if steps_taken % 100 == 0:
                print(f"Total steps: {steps_taken}")

        # Draw!
        screen.fill((0, 0, 0))
        world.draw(screen)
        gui.draw(screen)
        pygame.display.flip()

        # Limit the FPS of the simulation to FRAMERATE
        pygame.time.Clock().tick(FRAMERATE)


# run the main function only if this module is executed as the main script
# (if you import this as a module then nothing is executed)
if __name__ == "__main__":
    # call the main function
    # custom_controller = [-0.4516, -0.5024, -0.2318, 1.0, -1.16*math.pi]

    # Complex Circuits (For n > 30 this is just a circle)
    # Use n = 25
    # custom_controller = [-0.4, -0.5, -0.2318, 1.0, -1.16 * math.pi]

    # Complex Circuits 2
    # custom_controller = [0.6836, 0.95, 0.86, -0.9, -1.72*math.pi]

    # Membrane (for N > 50)
    # custom_controller = [0.6336, 0.95, 0.86, -0.9, -1.72*math.pi]

    # Partner Search (Dependent on stochastic initialization, unreliable)
    # custom_controller = [-0.549, -0.4532, -0.2683, -1.0, -(math.pi/2)]

    # Localized Aggregation
    # custom_controller = [-0.0471, -1.0, -1.0, 0.1820, (-1.3 * math.pi)]

    # Normal Aggregation
    custom_controller = [-0.7, -1.0, 1.0, -1.0]

    # Normal Cyclic Pursuit
    # custom_controller = [-0.7, 0.3, 1.0, 1.0]

    # custom_controller = [-0.7, -1.0, 1.0, -0.7, -1.0, 1.0, -1.0, -1.0, 0, math.pi]

    main(controller=custom_controller)
