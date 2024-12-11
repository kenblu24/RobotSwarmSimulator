from .Controller import Controller


shared_controllers = {}


class StaticController(Controller):
    def __init__(self, output=(0, 0)):
        self.list_based = False
        self.output = output
        self.controller_as_method = self.control_method
        super().__init__(self.control_method)

    def control_method(self, *args, **kwargs):
        """
        An example of a "from scratch" controller that you can code with any information contained within the agent class
        """
        return self.output


def zero_controller(d: int = 2):
    if shared_controllers.get("zero_controller", None) is None:
        shared_controllers[d] = StaticController((0,) * d)
    return shared_controllers[d]
