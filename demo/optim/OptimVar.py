class CMAESVarSet:
    def __init__(self, index_ref):
        """
        Expects a dictionary of the form [str : tuple], where the ith key is the name of the variable being controlled by the ith bounds
        and the tuple is the (min, max) bounds for the variable.
        """
        self.named_dict = index_ref
        self.from_unit_to_scaled = self.unit_unnormalize
        self.from_scaled_to_unit = self.unit_normalize

    @property
    def min_set(self):
        return [v[0] for v in self.named_dict.values()]

    @property
    def max_set(self):
        return [v[1] for v in self.named_dict.values()]

    @property
    def names(self):
        return list(self.named_dict.keys())

    def __len__(self):
        return len(self.named_dict)

    def unit_normalize(self, vector):
        return [self.map_to_unit(x, a, b) for x, a, b in zip(vector, self.min_set, self.max_set)]

    def unit_unnormalize(self, vector):
        return [self.map_from_unit(x, a, b) for x, a, b in zip(vector, self.min_set, self.max_set)]

    def as_dict(self):
        return self.named_dict

    def as_ordered_dict(self):
        return {"__order__": list(self.named_dict.keys()), **self.named_dict}

    @staticmethod
    def from_ordered_dict(ordered_dict):
        order = ordered_dict.pop("__order__")
        for key in order:
            if key not in ordered_dict:
                raise ValueError(f"Key {key} in __order__ not found in unordered dict. Refusing to create dict.")
        for key in ordered_dict:
            if key not in order:
                raise ValueError(f"Key {key} was found in dict but missing in __order__. Cannot order dict.")
        ordered_dict = {key: ordered_dict[key] for key in order}
        return CMAESVarSet(ordered_dict)

    @staticmethod
    def map_to_unit(x, in_a, in_b):
        return (x - in_a) / (in_b - in_a)

    @staticmethod
    def map_from_unit(x, out_a, out_b):
        return x * (out_b - out_a) + out_a

    @staticmethod
    def map_to_range(x, in_a, in_b, out_a, out_b):
        return (x - in_a) * (out_b - out_a) / (in_b - in_a) + out_a
