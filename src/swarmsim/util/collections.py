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
