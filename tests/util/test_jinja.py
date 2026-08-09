import pytest
import pathlib as pl
wd = pl.Path(__file__).parent


import swarmsim.util.jinja as jinja
from swarmsim.util.jinja import load_template


@pytest.fixture
def default_env():
    from swarmsim.util.jinja import make_default_jinja_env
    return make_default_jinja_env()


@pytest.fixture
def template_env():
    from swarmsim.util.jinja import make_template_env
    return make_template_env()


def test_load_template(default_env):
    res = load_template(wd / "helloworld.jinja", env=default_env)
    assert res == "Hello, World!"


def test_pickle_env_clean(default_env):
    import pickle
    pickle.dumps(default_env)


def test_pickle_env_dirty(default_env):
    res = load_template(wd / "helloworld.jinja", env=default_env)
    assert res == "Hello, World!"
    import pickle
    pickle.dumps(default_env)


def test_pickle_env_module(default_env):
    import math
    import jinja2
    default_env.add_global_module('math')
    res = load_template(wd / "helloworld.jinja", env=default_env)
    assert res == "Hello, World!"
    import pickle
    p = pickle.dumps(default_env)
    env = pickle.loads(p)
    assert math.pi == env.compile_expression('math.pi')()
    env.remove_global_module('math')
    with pytest.raises(jinja2.exceptions.UndefinedError):
        env.compile_expression('math.pi')()


def test_pickle_expression(default_env):
    expr = default_env.compile_expression('1 + 1 + x')
    import pickle
    p = pickle.dumps(expr)
    assert expr(x=2) == 4
    expr2 = pickle.loads(p)
    assert expr2(x=2) == 4


def test_pickle_expression_nomodules(default_env):
    modules_to_remove = [n for n, _m in default_env.global_modules]
    for n in modules_to_remove:
        default_env.remove_global_module(n)
    expr = default_env.compile_expression('1 + 1 + x')
    import pickle
    p = pickle.dumps(expr)
    assert expr(x=2) == 4
    expr2 = pickle.loads(p)
    assert expr2(x=2) == 4


def test_pickle_expression_recursive_reference(default_env):
    d = {'foo': 1}
    default_env.globals['test'] = d
    expr = default_env.compile_expression('test["foo"] + x')
    d['expr'] = expr
    import pickle
    p = pickle.dumps(expr)
    assert expr(x=2) == 3
    expr2 = pickle.loads(p)
    assert expr2(x=2) == 3


def test_load_custom_template(template_env):
    res = load_template(wd / "helloworld_custom.jinja", env=template_env)
    assert res == "Hello, World!"


def test_load_custom_context(template_env):
    from swarmsim.util.jinja import load_template_ctx
    rendered, ctx = load_template_ctx(wd / "helloworld_custom.jinja", env=template_env)
    assert rendered == "Hello, World!"
    assert ctx['s'] == "Hello, World!"


def test_compile_expression(template_env):
    expr = template_env.compile_expression('1 + 1 + x')
    assert expr(x=2) == 4


@pytest.fixture
def thing_template(default_env) -> jinja.Template:
    s = """
    {% set thing = 'world' %}
    """
    return default_env.from_string(s)


def test_thing_module(thing_template):
    thing_template.export_with()
    assert thing_template.saved_module.thing == 'world'


def test_compile_ctxpr(thing_template: jinja.Template):
    expr = thing_template.compile_ctxpr('thing|title', update='always')
    assert expr() == 'World'


def test_compile_ctxpr_undefined(thing_template: jinja.Template):
    assert not thing_template.has_cached_context
    with pytest.raises(jinja.TemplateRuntimeError):
        expr = thing_template.compile_ctxpr('thing|title', update='raise_undefined')
        expr()
    assert not thing_template.has_cached_context
    with pytest.raises(ValueError):
        expr = thing_template.compile_ctxpr('thing', update=None)  # type: ignore
        expr()
    assert not thing_template.has_cached_context
    with pytest.warns():
        expr = thing_template.compile_ctxpr('thing', update='warn_undefined')
        try:
            expr()
        except TypeError:
            pass
    assert not thing_template.has_cached_context
    expr = thing_template.compile_ctxpr('thing|title', update='run_undefined')
    assert expr() == 'World'
    assert thing_template.has_cached_context


def test_module(thing_template: jinja.Template):
    assert thing_template.module.thing == 'world'


def test_get_exports(thing_template: jinja.Template):
    assert not thing_template.has_cached_context
    assert not thing_template.saved_module
    assert thing_template.run_get_exports() == {'thing': 'world'}
    assert not thing_template.has_cached_context
    assert not thing_template.saved_module


def test_saved_module(thing_template: jinja.Template):
    assert not thing_template.has_cached_context
    assert not thing_template.saved_module
    thing_template.export_with()
    assert thing_template.saved_module.thing == 'world'
