from functools import cache
# import pygame
import random
import math
import numpy as np
from dataclasses import dataclass
from .MazeAgent import MazeAgent, MazeAgentConfig
from ..util.collider.AABB import AABB
from ..util.collider.CircularCollider import CircularCollider
from ..util.timer import Timer

# typing
from typing import Any, override

from ..config.WorldConfig import RectangularWorldConfig
from ..sensors.SensorSet import SensorSet
from ..world.World import World
from .control.Controller import Controller

import neuro
import caspian


@dataclass
class MazeAgentCaspianConfig(MazeAgentConfig):
    x: float | None = None
    y: float | None = None
    angle: float | None = None
    world: World | None = None
    world_config: RectangularWorldConfig | None = None
    seed: Any = None
    agent_radius: float = 5
    dt: float = 1.0
    sensors: SensorSet | None = None
    idiosyncrasies: Any = False
    delay: str | int | float = 0
    sensing_avg: int = 1
    stop_on_collision: bool = False
    stop_at_goal: bool = False
    body_color: tuple[int, int, int] = (255, 255, 255)
    body_filled: bool = False
    catastrophic_collisions: bool = False
    trace_length: tuple[int, int, int] | None = None
    trace_color: tuple[int, int, int] | None = None
    network: dict = None
    neuro_tpc: int | None = 10
    controller: Controller | None = None
    neuro_track_all: bool = False
    track_io: bool = False
    scale_forward_speed: float = 0.2  # m/s forward speed factor
    scale_turning_rates: float = 2.0  # rad/s turning rate factor
    type: str = ""

    def __post_init__(self):
        if self.stop_at_goal is not False:
            raise NotImplementedError  # not tested

    def as_dict(self):
        return self.asdict()

    def as_config_dict(self):
        return self.asdict()

    def asdict(self):
        return dict(self.as_generator())

    def __badvars__(self):
        return ["world", "world_config"]

    def as_generator(self):
        for key, value in self.__dict__.items():
            if any(key == bad for bad in self.__badvars__()):
                continue
            if hasattr(value, "asdict"):
                yield key, value.asdict()
            elif hasattr(value, "as_dict"):
                yield key, value.as_dict()
            elif hasattr(value, "as_config_dict"):
                yield key, value.as_config_dict()
            else:
                yield key, value

    @override
    def create(self, name=None):
        return MazeAgentCaspian(self, name)


class MazeAgentCaspian(MazeAgent):
    max_forward_speed = 0.2  # m/s
    max_turning_speed = 2.0  # rad/s

    def __init__(self, config: MazeAgentConfig | None = None, name=None, network: dict[str, Any] | None = None) -> None:
        if config is None:
            config = MazeAgentCaspianConfig()

        super().__init__(config=config, name=name)

        self.scale_v = config.scale_forward_speed  # m/s
        self.scale_w = config.scale_turning_rates  # rad/s

        self.network: neuro.Network = network if network is not None else config.network

        # for tracking neuron activity
        self.neuron_counts = None
        self.neuron_ids = None
        self.neuro_track_all = config.neuro_track_all

        # how many ticks the neuromorphic processor should run for
        if config.neuro_tpc is None:
            try:
                app_params = self.network.get_data("application")
                self.neuro_tpc = app_params["encoder_ticks"]
            except RuntimeError as err:
                raise RuntimeError("Could not find application parameters in network and no neuro_tpc specified.") from err
        else:
            self.neuro_tpc = config.neuro_tpc
        # now that we have the ticks per processor cycle, we can setup encoders & decoders
        self.setup_encoders()

        self.processor_params = self.network.get_data("processor")
        self.setup_processor(self.processor_params)

        self.rng = random.Random()

        # typing
        self.n_inputs: int
        self.n_outputs: int
        self.encoder: neuro.EncoderArray
        self.decoder: neuro.DecoderArray
        self.processor: caspian.Processor

    @staticmethod  # to get encoder structure/#neurons for external network generation (EONS)
    def get_default_encoders(neuro_tpc=1):
        encoder_params = {
            "dmin": [0] * 5,  # two bins for each binary input + random
            "dmax": [1] * 5,
            "interval": neuro_tpc,
            "named_encoders": {"s": "spikes"},
            "use_encoders": ["s"] * 5
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

        x = MazeAgentCaspian if class_homogenous else self

        encoders = x.get_default_encoders(self.neuro_tpc)

        x.n_inputs, x.n_outputs, x.encoder, x.decoder = encoders

    def setup_processor(self, pprops):
        # pprops = processor.get_configuration()
        self.processor = caspian.Processor(pprops)
        self.processor.load_network(self.network)
        neuro.track_all_output_events(self.processor, self.network)  # track only output fires

        if self.neuro_track_all:  # used for visualizing network activity
            neuro.track_all_neuron_events(self.processor, self.network)
            self.network.make_sorted_node_vector()
            self.neuron_ids = [x.id for x in self.network.sorted_node_vector]

    @staticmethod
    def bool_to_one_hot(x: bool):
        return (0, 1) if x else (1, 0)

    def run_processor(self, observation):
        b2oh = self.bool_to_one_hot

        # translate observation to vector
        if observation == 0:
            input_vector = b2oh(0) + b2oh(0)
        elif observation == 1:
            input_vector = b2oh(1) + b2oh(0)
        elif observation == 2:
            input_vector = b2oh(1) + b2oh(1)
        else:
            raise ValueError("Expected 0, 1, or 2 as observation.")
        input_vector += (1,)  # add 1 as constant on input to 4th input neuron
        # input_vector += (self.rng.randint(0, 1),)  # add random input to 4th input neuron

        spikes = self.encoder.get_spikes(input_vector)
        self.processor.apply_spikes(spikes)
        self.processor.run(10)
        self.processor.run(self.neuro_tpc)
        # action: bool = bool(proc.output_vectors())  # old. don't use.
        if self.neuro_track_all:
            self.neuron_counts = self.processor.neuron_counts()
        data = self.decoder.get_data_from_processor(self.processor)
        """  old wheelspeed code.
            # four bins. Two for each wheel, one for positive, one for negative.
            wl, wr = 2 * (data[1] - data[0]), 2 * (data[3] - data[2])
            return (wl, wr)
        """
        # three bins. One for +v, -v, omega.
        v = self.scale_v * (data[1] - data[0])
        w = self.scale_w * (data[3] - data[2])
        return v, w

    def get_actions(self) -> tuple[float, float]:
        sensor_state = self.sensors.getState()
        sensor_detection_id = self.sensors.getDetectionId()
        self.set_color_by_id(sensor_detection_id)

        v, omega = self.run_processor(sensor_state)
        self.requested = v, omega
        return self.requested
