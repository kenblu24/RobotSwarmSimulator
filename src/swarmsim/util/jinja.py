from collections import ChainMap

from jinja2 import Environment as BaseEnvironment
from jinja2 import Template as BaseTemplate
from jinja2.environment import TemplateModule
from jinja2.environment import TemplateExpression as BaseTemplateExpression

from jinja2.exceptions import TemplateSyntaxError, TemplateRuntimeError
from jinja2.parser import Parser
from jinja2.runtime import Undefined
from jinja2 import nodes

from jinja2.utils import consume

import typing as t


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

    def export_with_context(self, context):
        consume(self.root_render_func(context))
        self._context = context.get_exported()

    def export_with(__self__, *args, **kwargs):
        __self__.export_with_context(__self__.new_context(dict(*args, **kwargs)))

    @property
    def module_from_cached_context(self):
        return TemplateModule(self, self.new_context(self._context))

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
