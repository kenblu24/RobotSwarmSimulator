import ast
from .AbstractMetric import AbstractMetric


class AnyAgent(AbstractMetric):

    def __init__(
        self,
        name="AnyAgent",
        history=None,
        include_teams=None,
        exclude_teams=None,
        exclude_names=None,
        expression=None,
        value_on_found='agent',
        default=None,
        error='raise',
        default_aggregation=None,
    ):
        self.default = default
        self._expression = expression
        super().__init__(name=name, history_size=history)
        self.value_on_found = value_on_found
        self.include_teams = include_teams
        self.exclude_teams = exclude_teams
        self.exclude_names = exclude_names
        self.error = error
        self.default_aggregation = default_aggregation

    def reset(self):
        super().reset()
        self.current_value = self.default
        self.expression = self.expression

    @property
    def expression(self):
        return self._expression

    @expression.setter
    def expression(self, value):
        self._expression = value
        if value and self.world:
            self.compiled_expr = self.world.jenv.compile_expression(self._expression)
        else:
            self.compiled_expr = None

    def attach_world(self, world):
        super().attach_world(world)
        self.expression = self.expression

    def apply_filter(self, bag):
        if self.include_teams is not None:
            bag = [x for x in bag if x.team in self.include_teams]
        if self.exclude_teams is not None:
            bag = [x for x in bag if x.team not in self.exclude_teams]
        if self.exclude_names is not None:
            bag = [x for x in bag if x.name not in self.exclude_names]
        return bag

    def calculate(self):

        for it in (
            self.world.population,
            self.world.objects,
        ):
            for agent in self.apply_filter(it):
                try:
                    if (r := self.compiled_expr(agent=agent)):
                        if self.value_on_found == 'agent':
                            self.set_value(agent)
                        elif self.value_on_found == 'expression':
                            self.set_value(r)
                        else:
                            self.set_value(self.value_on_found)
                        return
                except (AttributeError, IndexError) as e:
                    if self.error == 'raise':
                        raise e
                    elif self.error == 'skip':
                        pass
                    elif self.error == 'default':
                        self.set_value(self.default)
                        return
                    elif self.error == 'warn':
                        raise NotImplementedError from e
                    else:
                        msg = f"Unknown error type {self.error}"
                        raise ValueError(msg) from e
