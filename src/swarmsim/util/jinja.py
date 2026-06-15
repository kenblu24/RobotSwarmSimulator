""" Custom modifications to Jinja templating engine.

.. currentmodule:: swarmsim.util.jinja

This module adjusts and extends the behavior of the Jinja2 templating engine.

.. seealso::

    This module is used with our custom YAML loader and config system.
    :doc:`/guide/yaml`
    :doc:`/guide/config`
    :py:mod:`yaml`

    It is also used by the :py:mod:`JinjaMetric` module.
    :py:mod:`JinjaMetric`

.. autoclass:: Environment
    :members:
    :undoc-members:

.. autoclass:: Template
    :members:
    :undoc-members:

.. autoclass:: ContextualExpression
    :members:
    :undoc-members:

.. autoclass:: TemplateExpression
    :members:
    :undoc-members:

.. autofunction:: make_default_jinja_env

.. autofunction:: make_template_env

.. autofunction:: load_template

.. autofunction:: load_template_ctx

"""

from warnings import warn
from collections import ChainMap

from jinja2 import Environment as BaseEnvironment
from jinja2 import Template as BaseTemplate
from jinja2.environment import TemplateModule, _environment_config_check
from jinja2.environment import TemplateExpression as BaseTemplateExpression

from jinja2.exceptions import TemplateSyntaxError, TemplateRuntimeError
from jinja2.parser import Parser
from jinja2.runtime import Undefined
from jinja2 import nodes

from jinja2.utils import consume

import typing as t


class TemplateExpression(BaseTemplateExpression):
    # unmodified except we use __self__ to avoid name collisions
    # otherwise, if 'self' in kwargs, __call__ will receive two 'self' arguments and fail
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
                    warn(msg, RuntimeWarning, stacklevel=2)
            elif __self__.update in ("run_undefined", "always"):
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
    USE_CLOUDPICKLE = True

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
        """Compile and render the template and get exported variables.

        Parameters
        ----------
        *args
            Positional arguments to pass to the template.
        **kwargs
            Keyword arguments to pass to the template. These become variables
            that can be accessed in the template.

        Returns
        -------
        dict
            Dictionary with names and values of variables set in the template.
        """
        ctx = self.new_context(dict(*args, **kwargs))
        consume(self.root_render_func(ctx))
        return ctx.get_exported()

    def export_with_context(self, context, save_module=True):
        """Render the template with injected context and save the exported variables.

        The results are placed in :py:attr:`_context` and :py:attr:`saved_module` attributes.

        Parameters
        ----------
        context : :py:class:`jinja2.runtime.Context`
            A context object to run the template with.
        save_module : bool, default=True
            If True, save the module object that was created during the render.

        Returns
        -------
        None
        """
        body_stream = list(self.root_render_func(context))
        self._context = context.get_exported()
        if save_module:
            self.saved_module = TemplateModule(self, context, body_stream)

    def export_with(__self__, *args, save_module=True, **kwargs):
        """Render the template with keyword arguments and save the exported variables.

        The results are placed in :py:attr:`_context` and :py:attr:`saved_module` attributes.

        Parameters
        ----------
        *args
            Positional arguments to pass to the template.
        save_module : bool, default=True
            If True, save the module object that was created during the render.
        **kwargs
            Keyword arguments to pass to the template. These become variables
            that can be accessed in the template.

        Returns
        -------
        None
        """
        __self__.export_with_context(__self__.new_context(dict(*args, **kwargs)),
                                     save_module=save_module)

    def __reduce_ex__(self, protocol):
        # allow Template to be pickled
        from inspect import ismodule
        state = super().__getstate__().copy()
        d = dict(state['globals'])
        # remove modules from state because they can't be pickled
        # instead, save the module names
        modules = {k: v.__name__ for k, v in d.items() if ismodule(v)}
        state['globals'] = {k: v for k, v in d.items() if not ismodule(v)}
        state['__modules__'] = modules

        if self.USE_CLOUDPICKLE:
            from cloudpickle import Pickler, dumps
            from cloudpickle.cloudpickle import _function_getstate

            # dummy class so we can use Pickler._function_getnewargs as static
            class Dummy:
                globals_ref = {}

            # extract function defaults which may contain references to things outside current scope
            newargs = Pickler._function_getnewargs(Dummy(), self.root_render_func)
            func_state = _function_getstate(self.root_render_func)
            state['__root_render_func__'] = dumps(newargs, protocol=protocol)  # cloudpickle the code
            state['__root_render_func_state__'] = func_state  # let regular pickler handle this
            del state['root_render_func']

        # USE_CLOUDPICKLE=False implies root_render_func doesn't need to be removed from state
        # and dill/cloudpickle will handle it like normal

        # return unpickler function, positional args, state, list, dict, closure
        # https://docs.python.org/3/library/pickle.html#object.__reduce__
        return self._from_pickle, (state,)

    # unpickler calls this
    @classmethod
    def _from_pickle(
        cls,
        state,
        closure=None,
    ):
        import importlib
        from pickle import loads
        # try retrieving root_render_func from state
        func = state.pop('root_render_func', None)
        # try retrieving pickled function from state
        func_newargs = state.pop('__root_render_func__', None)
        func_state = state.pop('__root_render_func_state__', None)
        environment = state.pop('environment')
        globals_ = state.pop('globals')
        modules = state.pop('__modules__', {})

        # here's what cloudpickle's function reducer would usually return
        # (_make_function, newargs, state, None, None, _function_setstate)
        # load pickled function kinda like cloudpickle would:
        if func_newargs is not None and func_state is not None:
            from cloudpickle.cloudpickle import _function_setstate, _make_function
            func_newargs = loads(func_newargs)  # load pickled code obj
            root_render_func = _make_function(*func_newargs)
            _function_setstate(root_render_func, func_state)

        if func is not None:  # this works if template was saved using dill/cloudpickle
            root_render_func = loads(func)

        # this is used in Template._from_namespace
        namespace = {
            'name': state.pop('name'),
            '__file__': state.pop('filename'),
            'blocks': state.pop('blocks'),
            'root': root_render_func,
            'debug_info': state.pop('_debug_info'),
        }

        namespace['root'] = root_render_func

        self = cls._from_namespace(environment, namespace, globals_)

        # set any remaining instance state
        try:
            super().__setstate__(state)
        except AttributeError:
            # if we're unpickling a base class, we don't have a __setstate__ method
            self.__dict__.update(state)
        # import any modules that were saved in state
        for k, v in modules.items():
            self.globals[k] = importlib.import_module(v)

        return self


    @property
    def has_cached_context(self):
        return self._context is not Undefined


class Environment(BaseEnvironment):
    template_class: t.Type[Template] = Template
    global_modules: t.List[t.Any] = None

    # modules can't be pickled.
    # this system allows modules to be added to env.globals
    def add_global_module(self, name: str, modulename: str | None = None):
        import importlib
        if self.global_modules is None:
            self.global_modules = []
        if modulename is None:
            modulename = name
        self.global_modules.append((name, modulename))
        self.globals[name] = importlib.import_module(modulename)

    def refresh_global_modules(self):
        import importlib
        if self.global_modules is None:
            return
        for name, modulename in self.global_modules:
            self.globals[name] = importlib.import_module(modulename)

    def remove_global_module(self, name: str):
        if self.global_modules is None:
            return
        self.global_modules.remove((name, self.globals[name].__name__))
        del self.globals[name]

    # called when pickling. removes modules from globals before serializing object state.
    def __getstate__(self):
        from inspect import ismodule
        state = super().__getstate__()
        if 'globals' in state:
            state['globals'] = {k: v for k, v in state['globals'].items() if not ismodule(v)}
        return state

    # re-imports modules after unpickling.
    # the module names are already in self.global_modules so just need to import and add to globals.
    def __setstate__(self, state):
        try:
            super().__setstate__(state)
        except AttributeError:
            # if we're unpickling a base class, we don't have a __setstate__ method
            self.__dict__.update(state)
        self.refresh_global_modules()

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
    env.add_global_module('np', 'numpy')
    env.add_global_module('math')
    return env


def make_template_env(env: t.Optional[Environment] = None, **kwargs):
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
