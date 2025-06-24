from typing import Tuple

from numpy import average


class AbstractMetric():
    __badvars__ = ['world']  # variables that should not be pickled
    instantaneous = True

    def __init__(self, name: str, history_size=100):
        self.name = name
        self.history_size = history_size
        self.reset()

    def reset(self):
        self.current_value = None
        self.value_history = []

    def attach_world(self, world):
        self.world = world

    def set_value(self, value):
        # Keep Track of the [self.history_size] most recent values
        self.value_history.append(value)
        if self.history_size is not None and len(self.value_history) > self.history_size:
            self.value_history = self.value_history[1:]

        self.current_value = value

    def out_current(self) -> Tuple:
        try:
            return (self.name, self.value_history[-1])
        except IndexError:
            return (self.name, None)

    @property
    def value(self):
        return self.current_value

    def out_average(self) -> Tuple:
        return (self.name, average(self.value_history))

    @property
    def average(self):
        return average(self.value_history)

    def draw(self, screen, zoom=1.0):
        pass

    def as_config_dict(self):
        return {"name": self.name, "history_size": self.history_size}

    def calculate(self):
        pass

    # prevent pickling errors
    def __getstate__(self):
        d = self.__dict__.copy()
        for k in self.__badvars__:
            d.pop(k, None)
        return d
