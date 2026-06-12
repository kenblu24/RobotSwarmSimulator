# typing:
from typing import Iterable
import pickle


# Source - https://stackoverflow.com/a/50049341
# Posted by pevogam
# Retrieved 2026-06-08, License - CC BY-SA 3.0

def test_pickle(xThing, lTested=[]):
    import pickle
    if id(xThing) in lTested:
        return lTested
    sType = type(xThing).__name__
    print('Testing {0}...'.format(sType))

    if sType in ['type', 'int', 'str', 'bool', 'NoneType', 'unicode']:
        print('...too easy')
        return lTested
    if sType == 'dict':
        print('...testing members')
        for k in xThing:
            lTested = test_pickle(xThing[k], lTested)
        print('...tested members')
        return lTested
    if sType == 'list':
        print('...testing members')
        for x in xThing:
            lTested = test_pickle(x)
        print('...tested members')
        return lTested

    lTested.append(id(xThing))
    oClass = type(xThing)

    for s in dir(xThing):
        if s.startswith('_'):
            print('...skipping *private* thingy')
            continue
        # if it is an attribute: Skip it
        try:
            xClassAttribute = oClass.__getattribute__(oClass, s)
        except (AttributeError, TypeError):
            pass
        else:
            if type(xClassAttribute).__name__ == 'property':
                print('...skipping property')
                continue

        xAttribute = xThing.__getattribute__(s)
        print('Testing {0}.{1} of type {2}'.format(sType, s, type(xAttribute).__name__))
        if type(xAttribute).__name__ == 'function':
            print("...skipping function")
            continue
        if type(xAttribute).__name__ in ['method', 'instancemethod']:
            print('...skipping method')
            continue
        if type(xAttribute).__name__ == 'HtmlElement':
            continue
        if type(xAttribute) == dict:
            print('...testing dict values for {0}.{1}'.format(sType, s))
            for k in xAttribute:
                lTested = test_pickle(xAttribute[k])
                continue
            print('...finished testing dict values for {0}.{1}'.format(sType, s))

        try:
            oIter = xAttribute.__iter__()
        except (AttributeError, TypeError):
            pass
        except AssertionError:
            pass  # lxml elements do this
        else:
            print('...testing iter values for {0}.{1} of type {2}'.format(sType, s, type(xAttribute).__name__))
            for x in xAttribute:
                lTested = test_pickle(x, lTested)
            print('...finished testing iter values for {0}.{1}'.format(sType, s))

        try:
            xAttribute.__dict__
        except AttributeError:
            pass
        else:
            # this attribute should be explored seperately...
            lTested = test_pickle(xAttribute, lTested)
            continue
        print(0, xThing, xAttribute)
        pickle.dumps(xAttribute)

    print('Testing {0} as complete object'.format(sType))
    pickle.dumps(xThing)
    return lTested


# modified version which uses the __getstate__ method
def test_picklex(x, tested: set[int] | None = None):
    if tested is None:
        tested = set()
    if id(x) in tested:
        return tested
    tested.add(id(x))
    xtype = type(x).__name__
    print(f'Testing {xtype} | {x!r:50}')

    if xtype in ['type', 'int', 'str', 'bool', 'NoneType', 'unicode']:
        # print('...too easy')
        return tested

    # if hasattr(x, '__reduce__') or hasattr(x, '__reduce_ex__'):
    #     reduced = x.__reduce__() if hasattr(x, '__reduce__') else x.__reduce_ex__(2)
    #     reduce_name = '__reduce__' if hasattr(x, '__reduce__') else '__reduce_ex__'
    #     print(f'...testing {reduce_name} for {xtype} | {x!r:50}')
    #     match reduced:
    #         case (_cls, args):
    #             state = listitems = dictitems = _func = None
    #         case (_cls, args, state):
    #             listitems = dictitems = _func = None
    #         case (_cls, args, state, listitems):
    #             dictitems = _func = None
    #         case (_cls, args, state, listitems, dictitems):
    #             _func = None
    #         case (_cls, args, state, listitems, dictitems, _func):
    #             pass
    #         case _:
    #             msg = f'Unexpected __reduce__ return value: {reduced}'
    #             raise TypeError(msg)
    #     test_picklex(args, tested)
    #     test_picklex(state, tested)
    #     test_picklex(listitems, tested)
    #     test_picklex(dictitems, tested)
    #     print(f'Finished testing {xtype} with custom reducer | {x!r:50}')
    #     return tested

    # if hasattr(x, '__getnewargs_ex__'):
    #     print(f'...testing __getnewargs_ex__ for {xtype} | {x!r:50}')
    #     args, kwargs = x.__getnewargs_ex__()
    #     test_picklex(args, tested)
    #     test_picklex(kwargs, tested)
    # elif hasattr(x, '__getnewargs__'):
    #     print(f'...testing __getnewargs__ for {xtype} | {x!r:50}')
    #     args = x.__getnewargs__()
    #     test_picklex(args, tested)

    # if hasattr(x, '__getstate__'):
    #     print(f'...testing __getstate__ for {xtype} | {x!r:50}')
    #     x = x.__getstate__()
    #     xtype = type(x).__name__

    if xtype == 'dict':
        print('...testing dict members')
        for k in x:
            tested = test_picklex(x[k], tested)
        print(f'...finished testing dict members')
        return tested
    if xtype == 'list':
        print('...testing list members')
        for i in x:
            tested = test_picklex(i)
        print('...finished testing list members')
        return tested

    oClass = type(x)

    for s in dir(x):
        if s.startswith('_'):
            print(f'...skipping *private* {s}')
            continue
        # if it is an attribute: Skip it
        try:
            xClassAttribute = oClass.__getattribute__(oClass, s)
        except (AttributeError, TypeError):
            pass
        else:
            if type(xClassAttribute).__name__ == 'property':
                print('...skipping property')
                continue

        xAttribute = x.__getattribute__(s)
        print(f'Testing {xtype}.{s} of type {type(xAttribute).__name__}')
        if type(xAttribute).__name__ == 'function':
            print("...skipping function")
            continue
        if type(xAttribute).__name__ in ['method', 'instancemethod']:
            print('...skipping method')
            continue
        if type(xAttribute).__name__ == 'HtmlElement':
            continue
        if type(xAttribute) == dict:
            print(f'...testing dict values for {xtype}.{s}')
            for k in xAttribute:
                tested = test_picklex(xAttribute[k], tested)
                continue
            print(f'...finished testing dict values for {xtype}.{s}')

        # try:
        #     oIter = xAttribute.__iter__()
        # except (AttributeError, TypeError):
        #     pass
        # except AssertionError:
        #     pass  # lxml elements do this
        # else:
        #     print(f'...testing iter values for {xtype}.{s} of type {type(xAttribute).__name__}')
        #     for x in xAttribute:
        #         tested = test_picklex(x, tested)
        #     print(f'...finished testing iter values for {xtype}.{s}')

        # try:
        #     xAttribute.__dict__
        # except AttributeError:
        #     pass
        # else:
        #     # this attribute should be explored seperately...
        #     tested = test_picklex(xAttribute, tested)
        #     continue
        print(0, x, xAttribute)
        test_picklex(xAttribute, tested)
        # pickle.dumps(xAttribute)

    print(f'Testing {xtype} as complete object | {x!r:50}')
    pickle.dumps(x)
    return tested

import swarmsim
from swarmsim.util.jinja import make_default_jinja_env

env = make_default_jinja_env()

expr = env.compile_expression('1 + 1 + x')
print(expr(x=2))

# test_picklex(env)
pickle.dumps(env)
