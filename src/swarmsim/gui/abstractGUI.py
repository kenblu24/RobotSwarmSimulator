from ..agent.Agent import Agent


class AbstractGUI:
    def __init__(self, x=0, y=0, w=0, h=0):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.track_all_events = False
        self.track_all_mouse = False
        self._selected = []

    def on_set_selected_single(self, agent: Agent):
        pass

    def on_set_selected_multiple(self, agents):
        pass

    def on_clear_selected(self, previously_selected):
        pass

    def on_selected_event(self, prev, new):
        pass

    @property
    def selected(self):
        return self._selected

    @selected.setter
    def selected(self, agents):
        prev = self._selected
        self.on_selected_event(prev, agents)
        if isinstance(agents, Agent):
            self._selected = [agents]
            self.on_set_selected_single(agents)
        else:
            self._selected = agents
            if agents:
                self.on_set_selected_multiple(agents)

        if prev and not self._selected:
            self.on_clear_selected(prev)

    def set_selected(self, agents):
        self.selected = agents

    def clear_selected(self):
        prev = self._selected
        self._selected = []
        self.on_selected_event(prev, self._selected)
        self.on_clear_selected(prev)

    def draw(self, screen, offset=((0, 0), 1.0)):
        pass
        # pygame.draw.rect(screen, color=(10,10,10), rect=Rect((self.x, self.y),(self.w, self.h)))

    def recieve_events(self, events):
        pass

    def recieve_mouse(self, mouse_rel):
        pass

    def set_screen(self, screen):
        self.screen = screen

    def step(self):
        pass

    def pass_key_events(self, events):
        pass
