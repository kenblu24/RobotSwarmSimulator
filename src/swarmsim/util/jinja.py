from collections import ChainMap
import json

from jinja2 import Environment as BaseEnvironment
from jinja2 import Template as BaseTemplate
from jinja2.environment import TemplateModule, _environment_config_check
from jinja2.environment import TemplateExpression as BaseTemplateExpression

from jinja2.exceptions import TemplateSyntaxError, TemplateRuntimeError
from jinja2.parser import Parser
from jinja2.runtime import Undefined
from jinja2 import nodes

from jinja2.utils import consume

from ..yaml.include import IncludeLoader, INCLUDE_TAG, search_file
import yaml

import typing as t
from typing import Any, IO


class TemplateExpression(BaseTemplateExpression):
    def __call__(__self__, *args: t.Any, **kwargs: t.Any) -> t.Optional[t.Any]:
        context = __self__._template.new_context(dict(*args, **kwargs))
        consume(__self__._template.root_render_func(context))
        rv = context.vars["result"]
        if __self__._undefined_to_none and isinstance(rv, Undefined):
            rv = None
        return rv


class ContextualExpression(BaseTemplateExpression):
    def __init__(self, template: "Template", undefined_to_none: bool, parent=None, update="raise_undefined") -> None:
        self._template = template
        self._undefined_to_none = undefined_to_none
        if parent is None:
            raise ValueError("parent must be set")
        self._parent = parent
        self.update = update

    def __call__(__self__, *args: t.Any, **kwargs: t.Any) -> t.Optional[t.Any]:
        argdict = dict(*args, **kwargs)

        if __self__.update not in ["raise_undefined", "warn_undefined", "run_undefined", "always"]:
            msg = f"update must be 'raise_undefined' or 'warn_undefined'. Got {__self__.update}"
            raise ValueError(msg)

        if __self__._parent._context is Undefined:
            if __self__.update == "raise_undefined" or __self__.update == "warn_undefined":
                msg = (f"ContextualExpression(update='raise_undefined') requires parent"
                        " to have run and exported its context before calling. Either run"
                        " template.export_with(*args, **kwargs) on the parent or set update='update'")
                if __self__.update == "raise_undefined":
                    raise TemplateRuntimeError(msg)
                elif __self__.update == "warn_undefined":
                    __self__._parent.environment.logger.warning(msg)
            elif __self__.update == "run_undefined":
                __self__._parent.export_with(**argdict)
        elif __self__.update == "always":
            __self__._parent.export_with(**argdict)

        cmap = ChainMap(argdict, __self__._parent._context) if __self__._parent._context else argdict
        context = __self__._template.new_context(cmap)
        consume(__self__._template.root_render_func(context))
        rv = context.vars["result"]
        if __self__._undefined_to_none and isinstance(rv, Undefined):
            rv = None
        return rv


class Template(BaseTemplate):
    _context = Undefined  # HACK: should be set in __init__
    saved_module = None

    def compile_ctxpr(self, source: str, undefined_to_none: bool = True, update: str = "raise_undefined"):
        # from jijna2.environment.compile_expression()
        """A handy helper method that returns a callable that accepts keyword
        arguments that appear as variables in the expression.  If called it
        returns the result of the expression.

        This is useful if applications want to use the same rules as Jinja
        in template "configuration files" or similar situations.

        Example usage:

        >>> env = Environment()
        >>> expr = env.compile_expression('foo == 42')
        >>> expr(foo=23)
        False
        >>> expr(foo=42)
        True

        Per default the return value is converted to `None` if the
        expression returns an undefined value.  This can be changed
        by setting `undefined_to_none` to `False`.

        >>> env.compile_expression('var')() is None
        True
        >>> env.compile_expression('var', undefined_to_none=False)()
        Undefined

        .. versionadded:: 2.1
        """
        parser = Parser(self.environment, source, state="variable")
        try:
            expr = parser.parse_expression()
            if not parser.stream.eos:
                raise TemplateSyntaxError(
                    "chunk after expression", parser.stream.current.lineno, None, None
                )
            expr.set_environment(self.environment)
        except TemplateSyntaxError:
            self.environment.handle_exception(source=source)

        body = [nodes.Assign(nodes.Name("result", "store"), expr, lineno=1)]
        expression_template = self.environment.from_string(nodes.Template(body, lineno=1))
        return ContextualExpression(expression_template, undefined_to_none, parent=self, update=update)

    def run_get_exports(self, *args, **kwargs):
        ctx = self.new_context(dict(*args, **kwargs))
        consume(self.root_render_func(ctx))
        return ctx.get_exported()

    def export_with_context(self, context, save_module=True):
        body_stream = list(self.root_render_func(context))
        self._context = context.get_exported()
        if save_module:
            self.saved_module = TemplateModule(self, context, body_stream)

    def export_with(__self__, *args, save_module=True, **kwargs):
        __self__.export_with_context(__self__.new_context(dict(*args, **kwargs)),
                                     save_module=save_module)

    @property
    def has_cached_context(self):
        return self._context is not Undefined


class Environment(BaseEnvironment):
    template_class: t.Type[Template] = Template

    def compile_expression(
        self, source: str, undefined_to_none: bool = True
    ) -> "TemplateExpression":
        """A handy helper method that returns a callable that accepts keyword
        arguments that appear as variables in the expression.  If called it
        returns the result of the expression.

        This is useful if applications want to use the same rules as Jinja
        in template "configuration files" or similar situations.

        Example usage:

        >>> env = Environment()
        >>> expr = env.compile_expression('foo == 42')
        >>> expr(foo=23)
        False
        >>> expr(foo=42)
        True

        Per default the return value is converted to `None` if the
        expression returns an undefined value.  This can be changed
        by setting `undefined_to_none` to `False`.

        >>> env.compile_expression('var')() is None
        True
        >>> env.compile_expression('var', undefined_to_none=False)()
        Undefined

        .. versionadded:: 2.1
        """
        parser = Parser(self, source, state="variable")
        try:
            expr = parser.parse_expression()
            if not parser.stream.eos:
                raise TemplateSyntaxError(
                    "chunk after expression", parser.stream.current.lineno, None, None
                )
            expr.set_environment(self)
        except TemplateSyntaxError:
            self.handle_exception(source=source)

        body = [nodes.Assign(nodes.Name("result", "store"), expr, lineno=1)]
        template = self.from_string(nodes.Template(body, lineno=1))
        return TemplateExpression(template, undefined_to_none)


def make_default_jinja_env(*args, **kwargs):
    import numpy as np
    import math
    env = Environment(
        *args,
        extensions=[
            'jinja2.ext.do',
            'jinja2.ext.loopcontrols',
            # 'jinja2.ext.with_',
        ],
        **kwargs)
    env.filters['sum'] = sum
    env.filters['bin'] = bin
    env.filters['hex'] = hex
    env.filters['alltrue'] = all
    env.filters['anytrue'] = any
    env.globals['np'] = np
    env.globals['math'] = math
    return env


def make_template_env(env=None, **kwargs):
    if env is None:
        env = make_default_jinja_env()
    env.block_start_string = '<%'
    env.block_end_string = '%>'
    env.variable_start_string = '<{'
    env.variable_end_string = '}>'
    env.line_statement_prefix = '#%>'
    _environment_config_check(env)
    return env


def load_template(path, env=None, **kwargs):
    if env is None:
        from ..util.jinja import make_template_env
        env = make_template_env()
    with open(path, "r") as f:
        template = env.from_string(f.read())
    return template.render(**kwargs)


def load_template_ctx(path, env=None, **kwargs):
    if env is None:
        from ..util.jinja import make_template_env
        env = make_template_env()
    with open(path, "r") as f:
        template = env.from_string(f.read())
    template.export_with(**kwargs)
    return ''.join(template.saved_module._body_stream), template._context


#
# Jinja Template Preprocessing when loading YAML
#
# Load order for load_yaml_with_ctx():
# file.yaml
# 1. Preprocess jinja into pure YAML, save exported jinja vars
# 2. Compose YAML
# 3. Merge <<: !include x.yaml, its exported vars are added beneath parent context
# 4. Construct all YAML objects, processing non-merge key !include tags
# 5. !include file2.yaml
# 6.    Preprocess file2.yaml jinja with the parent context. Parent context can't be modified by file2.yaml.
# 7.    etc.
# ... return the final YAML object


class TemplateYAMLLoader(IncludeLoader):
    def __init__(self, stream: IO | str, name=None, jinja_env=None, jinja_ctx=None) -> None:
        """Initialise Loader."""
        super().__init__(stream, name=name)
        self.jinja_env = jinja_env or make_template_env()
        self.jinja_ctx = {} if jinja_ctx is None else jinja_ctx

    # for merge keys in YAML i.e. <<: !include filename.yaml
    def compose_include_merge_node(self, node):
        path = search_file(self.file_path.parent, node.value)
        s, ctx = load_template_ctx(path, env=self.jinja_env, yaml_loader=self.__class__, **self.jinja_ctx)
        self.jinja_ctx = ChainMap(self.jinja_ctx, ctx)
        import yaml
        return yaml.compose(s, self.__class__)


def construct_include(loader: TemplateYAMLLoader, node: yaml.Node) -> Any:
    """Read the contents of a yaml/text/json file into a node"""
    node_path = search_file(loader.file_path.parent, loader.construct_scalar(node))  # pyright: ignore[reportArgumentType]

    ext = node_path.suffix

    if ext in ('.yaml', '.yml'):
        return load_yaml_template(node_path, name=node_path, env=loader.jinja_env,
                                    yaml_loader_cls=loader.__class__, context=loader.jinja_ctx)
    else:
        s = load_template(node_path)
        if ext in ('.json', ):
            return json.load(s)
        return s


yaml.add_constructor(INCLUDE_TAG, construct_include, TemplateYAMLLoader)


def load_yaml_template(path, name=None, env=None, yaml_loader_cls=TemplateYAMLLoader, context=None, **kwargs):
    """
    Parse the first YAML document in a stream
    and produce the corresponding Python object.
    """
    if name is None:
        name = path
    ctx = ChainMap(kwargs)
    if context is not None:
        ctx.maps.append(context)
    s, exported = load_template_ctx(path, env=env, **ctx)
    ctx.maps.append(exported)
    loader = yaml_loader_cls(s, name=name, jinja_env=env, jinja_ctx=ctx)
    try:
        return loader.get_single_data()
    finally:
        loader.dispose()
