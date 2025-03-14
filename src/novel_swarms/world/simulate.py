import pygame
from ..gui.agentGUI import DifferentialDriveGUI
from .World import World_from_config, World
from ..util.timer import Timer

screen = None
FRAMERATE = 200


def main(
    world_config,
    show_gui=True,
    gui=None,
    stop_detection=None,
    world_key_events=False,
    gui_key_events=False,
    subscribers=None,
    save_duration=1200,
    save_every_ith_frame=3,
    save_time_per_frame=50,
    step_size=1,
    start_paused=False,
    viewport_zoom=100.0,
):
    # initialize the pygame module
    if show_gui:
        pygame.init()
        pygame.display.set_caption("Swarm Simulation")

    if isinstance(world_config, World):
        world = world_config
    else:
        world = World_from_config(world_config)

    # screen must be global so that other modules can access + draw to the window
    global screen
    gui_width = 300
    if gui:
        gui_width = gui.w
    if show_gui:
        w, h = world.config.size
        dims = (w * viewport_zoom + gui_width, h * viewport_zoom)
        screen = pygame.display.set_mode(dims, pygame.RESIZABLE)

    # define a variable to control the main loop
    running = True
    paused = start_paused
    draw_world = True

    # Create the simulation world
    world.original_zoom = viewport_zoom
    world._screen_cache = screen if screen else None
    world.setup()

    # Attach any subscribers to the world
    world_subscribers = []
    if subscribers:
        world_subscribers = subscribers

    # Create the GUI
    if show_gui and not gui:
        gui = DifferentialDriveGUI(x=dims[0], y=0, h=dims[1], w=gui_width)
        gui.position = "sidebar_right"

    # Attach the world to the gui and vice versa
    if gui:
        gui.set_world(world)
        gui.set_screen(screen)
        world.attach_gui(gui)

    total_allowed_steps = getattr(world, 'stop_at', world.config.stop_at)
    steps_taken = 0
    steps_per_frame = step_size
    slowdown_level = 0

    # labels = [pygame.K_RETURN, pygame.K_q, pygame.K_0, pygame.K_KP0, pygame.K_1, pygame.K_KP1, pygame.K_2, pygame.K_KP2,
    #           pygame.K_3, pygame.K_KP3, pygame.K_4, pygame.K_KP4, pygame.K_5, pygame.K_KP5]

    do_plot = False

    def background_color():
        return getattr(world, 'background_color', world.config.background_color)

    def draw():
        if gui and screen:
            gui.set_time(steps_taken)
            gui.sim_paused = paused
            screen.fill(background_color())
            if draw_world:
                world.draw(screen)
            # gui.step()
            if gui.track_all_mouse:
                gui.recieve_mouse(pygame.mouse.get_rel())
            if gui.track_all_events:
                gui.recieve_events(pygame.event.get())
            gui.draw(screen)

    # Main loop
    time_me = Timer("World Step")
    step_timer = Timer("step")
    clock = pygame.time.Clock() if gui else None
    eclock = pygame.time.Clock() if gui else None
    while running:
        # Looped Event Handling
        if gui:
            scroll_event = None
            scroll_event_up = None
            middle_mouse_events = []
            events = pygame.event.get()
            for event in events:
                # Cancel the game loop if user quits the GUI
                if event.type == pygame.QUIT:
                    return world
                if event.type == pygame.KEYDOWN:
                    if event.key in (pygame.K_SPACE, pygame.K_k):
                        paused = not paused
                        # print(f"Paused on Simulation Step: {steps_taken}")
                    elif event.key in (pygame.K_RIGHT, pygame.K_l) and paused:
                        # ON Right Arrow Pressed, draw single frame
                        world.step()
                        steps_taken += 1
                        draw()
                        pygame.display.flip()
                    elif event.key == pygame.K_r:
                        # world = WorldFactory.create(world_config)
                        steps_taken = 0
                    elif event.key == pygame.K_RSHIFT:
                        if slowdown_level > 0:
                            slowdown_level -= 1
                        else:
                            steps_per_frame *= 2
                            steps_per_frame = min(steps_per_frame, 256)
                    elif event.key == pygame.K_LSHIFT:
                        if steps_per_frame > 1:
                            steps_per_frame //= 2
                            round(max(steps_per_frame, 1))
                        else:
                            slowdown_level += 1
                    # elif event.key == pygame.K_w:
                    #     draw_world = not draw_world
                    elif event.key == pygame.K_F3:
                        from .WorldIO import WorldIO
                        WorldIO.save_world(world)
                    elif event.key == pygame.K_F4:
                        from .subscribers.World2Gif import World2Gif
                        world_subscribers.append(World2Gif(duration=save_duration, every_ith_frame=save_every_ith_frame, time_per_frame=save_time_per_frame))
                    elif event.key == pygame.K_u:
                        pass
                    elif event.key == pygame.K_i:
                        do_plot = not do_plot
                    elif event.key == pygame.K_o:
                        pass
                    elif event.key == pygame.K_p:
                        pass
                    elif event.key == pygame.K_j:
                        pass
                    elif event.key == pygame.K_l and paused:
                        pass
                        # step_all_snns()
                    elif event.key == pygame.K_KP0:
                        world.zoom_reset()

                    if world_key_events:
                        world.handle_key_press(event)
                    if gui and gui_key_events:
                        gui.pass_key_events(event)
                    # if event.key in labels:
                    #     return event.key, steps_taken

                elif event.type == pygame.MOUSEBUTTONUP:
                    if event.button == 1:
                        world.onClick(event)
                    if event.button == 2:
                        middle_mouse_events.append(event)
                    elif event.button in (4, 5):
                        scroll_event_up = event
                elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 2:
                    middle_mouse_events.append(event)
                elif event.type == pygame.MOUSEWHEEL:
                    scroll_event = event

            if scroll_event and scroll_event_up:
                world.onZoom(scroll_event_up, scroll_event)

            buttons = pygame.mouse.get_pressed()
            if middle_mouse_events:
                world.handle_middle_mouse_events(middle_mouse_events)
            elif buttons[1]:  # middle mouse button
                world.handle_middle_mouse_held(pygame.mouse.get_pos())

            if world_key_events:
                keys = pygame.key.get_pressed()
                world.handle_held_keys(keys)

        if gui:
            if steps_per_frame >= 1:
                gui.speed = f"{steps_per_frame}x"
            gui.fps = (clock.get_fps(), eclock.get_fps())
            world.on_mouse(pygame.mouse.get_pos())
            world.events = events

        skip = False
        if slowdown_level > 0:
            period = (1.5 ** slowdown_level) / FRAMERATE
            if step_timer() < period:
                skip = True
            else:
                step_timer.restart()
            gui.speed = f"/{slowdown_level + 1}"

        if paused or skip:
            draw()
            pygame.display.flip()
            clock.tick(FRAMERATE)
            continue
        # Calculate Steps - Stop if we reach desired frame
        for _ in range(steps_per_frame):

            if callable(stop_detection) and stop_detection(world):
                running = False
                return world

            try:
                if total_allowed_steps >= 0 and steps_taken > total_allowed_steps:
                    running = False
                    return world
            except TypeError:
                pass

            world.step()

            # Broadcast to any world subscribers
            _ = [sub.notify(world, screen) for sub in world_subscribers]

            steps_taken += 1
            # if steps_taken % 1000 == 0:
            # print(f"Total steps: {steps_taken}")

        # Limit the FPS of the simulation to FRAMERATE
        if gui:
            draw()
            pygame.display.flip()
            eclock.tick()
            clock.tick(FRAMERATE)
