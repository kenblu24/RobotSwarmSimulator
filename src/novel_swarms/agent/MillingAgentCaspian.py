from typing import Tuple, Any
# import pygame
import random
import math
import numpy as np
from dataclasses import dataclass
from .MazeAgentCaspian import MazeAgentCaspian, MazeAgentCaspianConfig
from ..util.timer import Timer

# typing
from ..config.WorldConfig import RectangularWorldConfig
from ..sensors.SensorSet import SensorSet
from ..world.World import World

import neuro
import caspian


@dataclass
class MillingAgentCaspianConfig(MazeAgentCaspianConfig):
    def create(self, name=None):
        return MillingAgentCaspian(self, name)


class MillingAgentCaspian(MazeAgentCaspian):

    def __init__(self, config: MillingAgentCaspianConfig = None, name=None, network: dict = None) -> None:
        if config is None:
            config = MillingAgentCaspianConfig()

        super().__init__(config=config, name=name)

    @staticmethod  # to get encoder structure/#neurons for external network generation (EONS)
    def get_default_encoders(neuro_tpc):
        encoder_params = {
            "dmin": [0] * 2,  # two bins for each binary input + random
            "dmax": [1] * 2,
            "interval": neuro_tpc,
            "named_encoders": {"s": "spikes"},
            "use_encoders": ["s"] * 2
        }
        decoder_params = {
            # see notes near where decoder is used
            "dmin": [0] * 4,
            "dmax": [1] * 4,
            "divisor": neuro_tpc,
            "named_decoders": {"r": {"rate": {"discrete": False}}},
            "use_decoders": ["r"] * 4
        }
        encoder = neuro.EncoderArray(encoder_params)
        decoder = neuro.DecoderArray(decoder_params)

        return (
            encoder.get_num_neurons(),
            decoder.get_num_neurons(),
            encoder,
            decoder
        )

    def setup_encoders(self, class_homogenous=True) -> None:
        # Note: encoders/decoders *can* be saved to or read from the network. not implemented yet.

        # Setup encoder
        # for each binary raw input, we encode it to constant spikes on bins, kinda like traditional one-hot

        # Setup decoder
        # Read spikes to a discrete set of floats using rate-based decoding

        x = MillingAgentCaspian if class_homogenous else self

        encoders = x.get_default_encoders(self.neuro_tpc)

        x.n_inputs, x.n_outputs, x.encoder, x.decoder = encoders

    def run_processor(self, observation):
        b2oh = self.bool_to_one_hot

        # translate observation to vector
        input_vector = b2oh(observation)
        # input_vector += (1,)  # add 1 as constant on input to 4th input neuron

        spikes = self.encoder.get_spikes(input_vector)
        self.processor.apply_spikes(spikes)
        self.processor.run(5)
        self.processor.run(self.neuro_tpc)
        # action: bool = bool(proc.output_vectors())  # old. don't use.
        if self.neuro_track_all:
            self.neuron_counts = self.processor.neuron_counts()
        data = self.decoder.get_data_from_processor(self.processor)
        # three bins. One for +v, -v, omega.
        v = 0.2 * (data[1] - data[0])
        w = 2.0 * (data[3] - data[2])
        return v, w
        # return (0.08, 0.4) if not observation else (0.18, 0.0)  # CMA best controller
