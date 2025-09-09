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


def slice_indices(s: slice, max_len: int = 0) -> list[int]:
    if s.step is None or s.step >= 0 or s.stop is None:
        stop = max_len if s.stop is None else min(s.stop, max_len)
    else:  # step is negative, stop is not None. Ignore stop
        stop = max_len
    return list(range(stop))[s]


class HookList(list):
    def __init__(self, iterable=None, add_callbacks=None, del_callbacks=None):
        if iterable is None:
            iterable = []
        super().__init__(iterable)
        self._add_callbacks = [] if add_callbacks is None else add_callbacks
        self._del_callbacks = [] if del_callbacks is None else del_callbacks
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

    def __setitem__(self, key, value):
        n = len(self)
        if isinstance(key, slice):
            indices = slice_indices(key, n)
            for func in self._del_callbacks:
                for i in indices:
                    func(self[i])
            for func in self._add_callbacks:
                for obj in value:
                    func(obj)
        else:
            for func in self._del_callbacks:
                func(self[key])
            for func in self._add_callbacks:
                func(value)
        super().__setitem__(key, value)
