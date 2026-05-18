import ast
from .AbstractMetric import AbstractMetric


class EntityLen(AbstractMetric):

    def __init__(
        self,
        name="EntityLen",
        history=None,
        target_name=None,
        attribute=None,
        default_aggregation=None,
    ):
        super().__init__(name=name, history_size=history)
        self.target_name = target_name
        self.target = None
        self.attribute = attribute
        self.compiled_expr = None
        self.default_aggregation = default_aggregation

    def attach_world(self, world):
        super().attach_world(world)
        if (
            self.world
            and self.attribute is not None
            and not isinstance(ast.parse(self.attribute, mode='eval').body, ast.Name)
        ):
            self.compiled_expr = self.world.jenv.compile_expression('target.' + self.attribute)

    def find_target(self):
        if self.target_name is None:
            raise ValueError("AgentCount metric requires a target_name.")

        for it in (
            self.world.population,
            self.world.objects,
            self.world.spawners,
            self.world.metrics,
        ):
            for thing in it:
                if thing.name == self.target_name:
                    return thing

        if self.target_name == 'population':
            return self.world.population
        elif self.target_name == 'objects':
            return self.world.objects
        elif self.target_name == 'spawners':
            return self.world.spawners
        elif self.target_name == 'metrics':
            return self.world.metrics
        return None

    def calculate(self):
        if self.target is None:
            self.target = self.find_target()

        if self.compiled_expr is not None:
            ent = self.compiled_expr(target=self.target)
        elif self.attribute is not None:
            ent = getattr(self.target, self.attribute)
        else:
            ent = self.target

        self.set_value(None if ent is None else len(ent))
