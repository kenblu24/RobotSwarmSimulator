from .AbstractController import AbstractController


shared_controllers = {}


class StaticController(AbstractController):
    def __init__(self, agent=None, parent=None, output=(0.0, 0.0)):
        self.output = output
        super().__init__(agent=agent, parent=parent)

    def as_config_dict(self):
        return {'output': self.output}

    def __str__(self):
        body = '\n'.join([f'  u_{i}: {x: >8.4f}' for i, x in enumerate(self.output)])
        return 'StaticController:\n' + body

    def __repr__(self):
        return f"{self.__class__.__name__}: {self.output}"

    def get_actions(self, agent):
        return self.output


def zero_controller(d: int = 2):
    if shared_controllers.get("zero_controller", None) is None:
        shared_controllers[d] = StaticController((0,) * d)
    return shared_controllers[d]
