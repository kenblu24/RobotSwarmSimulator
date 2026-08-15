import numpy as np

from typing import Any


class FlagSet(set):
    def __init__(self, flags):
        self.flags = {}
        if isinstance(flags, dict):
            for k, v in flags.items():
                if isinstance(v, int):
                    if v > 0:
                        self.flags[k] = v
                else:
                    msg = f"FlagSet only supports int values, not {type(v)}"
                    raise ValueError(msg)
        elif isinstance(flags, FlagSet):
            self.flags = flags.flags
        else:
            for flag in flags:
                self.flags[flag] = 1

    def __contains__(self, flag):
        return flag in self.flags

    def __iter__(self):
        return iter(self.flags)

    def __len__(self):
        return len(self.flags)

    def __repr__(self):
        return f"FlagSet({self.flags})"

    def __str__(self):
        return f"FlagSet({self.flags})"

    def __eq__(self, other):
        if isinstance(other, FlagSet):
            return self.flags == other.flags
        else:
            return self.flags == other

    def __getitem__(self, flag):
        try:
            return self.flags[flag]
        except KeyError:
            return 0

    def __setitem__(self, flag, value):
        if not isinstance(value, int):
            msg = f"FlagSet only supports int values, not {type(value)}"
            raise ValueError(msg)
        if value < 1:
            self.flags.pop(flag, None)
        self.flags[flag] = int(value)

    def __delitem__(self, flag):
        self.flags.pop(flag, None)

    def clear(self):
        self.flags.clear()

    def copy(self):
        return FlagSet(self.flags.copy())

    def update(self, other):
        if isinstance(other, FlagSet):
            self.flags.update(other.flags)
        elif isinstance(other, set):
            raise ValueError("FlagSet.update() does not support updating from a set.")
        else:
            self.flags.update(other)

    def inc(self, flag):
        if isinstance(flag, set):
            for f in flag:
                self.inc(f)
        else:
            self[flag] += 1

    def dec(self, flag):
        self[flag] -= 1

    def remove(self, flag):
        del self.flags[flag]

    def pop(self, flag):
        return self.flags.pop(flag, 0)

    def list(self):
        return list(self.flags.keys())

    def items(self):
        return self.flags.items()

    def values(self):
        return self.flags.values()

    def keys(self):
        return self.flags.keys()

    def __or__(self, value):
        return self.flags | value

    def __ior__(self, value):
        self.flags |= value
        return self

    def setdefault(self, key, default=None):
        return self.flags.setdefault(key, default)

    def get(self, key, default=0):
        return self.flags.get(key, default)

    def popitem(self):
        return self.flags.popitem()

    def __reversed__(self):
        return self.flags.__reversed__()

    def reversed(self):
        return self.flags.reversed()

    def __add__(self, other):
        new = self.copy()
        if isinstance(other, FlagSet):
            for flag, value in other.flags.items():
                new[flag] += value
        elif isinstance(other, dict):
            for flag, value in other.items():
                new[flag] += value
        elif isinstance(other, set):
            for flag in other:
                new.inc(flag)
        else:
            new.inc(other)
        return new

    def __iadd__(self, other):
        if isinstance(other, FlagSet):
            for flag, value in other.flags.items():
                self[flag] += value
        elif isinstance(other, dict):
            for flag, value in other.items():
                self[flag] += value
        elif isinstance(other, set):
            for flag in other:
                self.inc(flag)
        else:
            self.inc(other)
        return self

    def __sub__(self, other):
        new = self.copy()
        if isinstance(other, FlagSet):
            for flag, value in other.flags.items():
                new[flag] -= value
        elif isinstance(other, dict):
            for flag, value in other.items():
                new[flag] -= value
        elif isinstance(other, set):
            for flag in other:
                new.dec(flag)
        else:
            new.dec(other)
        return new

    def __isub__(self, other):
        if isinstance(other, FlagSet):
            for flag, value in other.flags.items():
                self[flag] -= value
        elif isinstance(other, dict):
            for flag, value in other.items():
                self[flag] -= value
        elif isinstance(other, set):
            for flag in other:
                self.dec(flag)
        else:
            self.dec(other)
        return self

    def countOf(self, flag):
        return self.flags.get(flag, 0)

    def __ge__(self, other):
        if isinstance(other, FlagSet):
            return set(self.flags) >= set(other.flags)
        elif isinstance(other, set):
            return set(self.flags) >= other

    def __gt__(self, other):
        if isinstance(other, FlagSet):
            return set(self.flags) > set(other.flags)
        elif isinstance(other, set):
            return set(self.flags) > other

    def __le__(self, other):
        if isinstance(other, FlagSet):
            return set(self.flags) <= set(other.flags)
        elif isinstance(other, set):
            return set(self.flags) <= other

    def __lt__(self, other):
        if isinstance(other, FlagSet):
            return set(self.flags) < set(other.flags)
        elif isinstance(other, set):
            return set(self.flags) < other


def slicen(s: slice) -> slice:
    if not isinstance(s, slice):
        msg = f"Expected slice, but received {type(s)}"
        raise TypeError(msg)
    start = int(s.start) if s.start is not None else None
    stop = int(s.stop) if s.stop is not None else None
    step = int(s.step) if s.step is not None else None
    return slice(start, stop, step)


def slice_indices(s: slice, max_len: int = 0) -> list[int]:
    s = slicen(s)
    if s.step is None or s.step >= 0 or s.stop is None:
        stop = max_len if s.stop is None else min(s.stop, max_len)
    else:  # step is negative, stop is not None. Ignore stop
        stop = max_len
    return list(range(stop))[s]


class HookList(list):
    # need these to exist when unpickling (since __init__ is not called then)
    _add_callbacks = ()
    _del_callbacks = ()

    def __init__(self, iterable=None, add_callbacks=None, del_callbacks=None):
        if iterable is None:
            iterable = []
        super().__init__(iterable)
        self._add_callbacks = list(self._add_callbacks)
        self._del_callbacks = list(self._del_callbacks)
        self._add_callbacks += [] if add_callbacks is None else list(add_callbacks)
        self._del_callbacks += [] if del_callbacks is None else list(del_callbacks)
        for func in self._add_callbacks:
            for obj in self:
                func(obj)

    def register_add_callback(self, func):
        self._add_callbacks.append(func)

    def register_del_callback(self, func):
        self._del_callbacks.append(func)

    def append(self, obj):
        for func in self._add_callbacks:
            func(obj)
        return super().append(obj)

    def extend(self, iterable):
        li = list(iterable)
        for func in self._add_callbacks:
            for obj in li:
                func(obj)
        return super().extend(li)

    def insert(self, index, obj):
        super().insert(index, obj)
        for func in self._add_callbacks:
            func(obj)

    def __iadd__(self, value):
        for func in self._add_callbacks:
            for obj in value:
                func(obj)
        return super().__iadd__(value)

    def __imul__(self, value):
        for func in self._add_callbacks:
            for _i in range(value):
                for obj in self:
                    func(obj)
        return super().__imul__(value)

    def pop(self, index=-1):
        for func in self._del_callbacks:
            func(self[index])
        return super().pop(index)

    def remove(self, value):
        for func in self._del_callbacks:
            func(value)
        return super().remove(value)

    def __setitem__(self, idx, value):
        def set_single(idx, value):
            for func in self._del_callbacks:
                func(self[idx])
            for func in self._add_callbacks:
                func(value)
            super().__setitem__(idx, value)

        if isinstance(idx, (int, np.integer)):
            set_single(idx, value)
            return

        n = len(self)
        if isinstance(idx, slice):
            sl = slicen(idx)
            idx = slice_indices(idx)
            if sl.step in (None, 1):
                # need to do these separately since it's not necessarily 1-to-1
                # HACK: performance may be O(nlogn) since using del and insert
                for i in reversed(idx):
                    self.__delitem__(i)
                for x in reversed(value):
                    self.insert(sl.start or 0, x)
            else:
                # Extended slice
                if len(idx) != len(value):
                    msg = (f"attept to assign sequence of {len(value)} items"
                           f"to extended slice of size {len(idx)}.")
                    raise ValueError(msg)
        elif isinstance(idx, list):
            if len(idx) != len(value):
                msg = (f"attept to assign sequence of {len(value)} values "
                       f"to {len(idx)} indices.")
                raise ValueError(msg)
            if any(not (0 <= i < n) for i in idx):
                raise ValueError("Received indices out of range.")
        for i, x in zip(idx, value):
            set_single(i, x)

    def __delitem__(self, idx):
        if isinstance(idx, slice):
            idx = slice_indices(idx)
            # need to do these separately since it's not necessarily 1-to-1
            # HACK: performance may be O(nlogn) since using del and insert
            for i in reversed(idx):
                for func in self._del_callbacks:
                    func(self[i])
                super().__delitem__(i)
        else:
            for func in self._del_callbacks:
                func(self[idx])
            return super().__delitem__(idx)


class RefProp:
    _remove_value = None

    def __init__(self, backref_name):
        self.backref_name = backref_name

    def __set_name__(self, owner, name):
        self._name = name
        self._owner = owner
        self._prop = f"_{name}"

    def __get__(self, instance, _owner):
        return getattr(instance, self._prop)

    def __set__(self, instance, value):
        child = getattr(instance, self._prop, self._remove_value)
        if child is not self._remove_value:  # clear the old backref
            setattr(child, self.backref_name, self._remove_value)
        setattr(instance, self._prop, value)  # set on parent
        setattr(value, self.backref_name, instance)  # set backref on child

    def __delete__(self, parent):
        setattr(getattr(parent, self._prop), self.backref_name, self._remove_value)
        delattr(parent, self._prop)


class RefListProp:
    def __set_name__(self, owner, name):
        self._name = name
        self._owner = owner
        self._prop = f"_{name}"

    def __get__(self, instance, owner):
        return getattr(instance, self._prop)

    def __set__(self, instance, value):
        getattr(instance, self._prop)[:] = value

    def __delete__(self, parent):
        getattr(parent, self._prop).clear()
        delattr(parent, self._prop)


class RefList(HookList):
    _remove_value = None

    def __init__(self, backref, backref_name, add_callbacks=None, del_callbacks=None):
        self.backref: Any = backref
        self.backref_name = backref_name
        self._add_callbacks += (self.add_backref,)
        self._del_callbacks += (self.remove_backref,)
        super().__init__(add_callbacks, del_callbacks)

    def add_backref(self, obj):
        setattr(obj, self.backref_name, self.backref)

    def remove_backref(self, obj):
        if self._remove_value == '__delattr__':
            delattr(obj, self.backref_name)
        else:
            setattr(obj, self.backref_name, self._remove_value)


class NameRef(str):
    def __new__(cls, name):
        return super().__new__(cls, name)


class NameList(HookList):
    def __init__(self, iterable=None, add_callbacks=None, del_callbacks=None):
        super().__init__(iterable, add_callbacks, del_callbacks)
        self._name_map = {}
