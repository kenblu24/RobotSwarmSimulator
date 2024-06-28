from typing import Any, override
# import pygame
import random
import math
import numpy as np
from dataclasses import dataclass
from .MazeAgent import MazeAgent
from ..config.AgentConfig import MazeAgentConfig
from ..util.collider.AABB import AABB
from ..util.collider.CircularCollider import CircularCollider
from ..util.timer import Timer

# typing
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
    stop_on_collision: bool = False
    stop_at_goal: bool = False
    body_color: tuple[int, int, int] = (255, 255, 255)
    body_filled: bool = False
    catastrophic_collisions: bool = False
    trace_length: tuple[int, int, int] | None = None
    trace_color: tuple[int, int, int] | None = None
    network: neuro.json = None
    neuro_tpc: int = 10
    controller: Controller | None = None
    neuro_track_all: bool = False
    track_io: bool = False
    type: str = ""

    def __post_init__(self):
        if self.stop_at_goal is not False:
            raise NotImplementedError  # not tested

        self.as_dict = self.asdict

    def asdict(self):
        for key, value in self.__dict__:
            if callable(value.asdict):
                yield key, value.asdict()
            elif callable(value.as_dict):
                yield key, value.as_dict()
            elif callable(value.as_config_dict):
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

        self.network = network if network is not None else config.network

        # for tracking neuron activity
        self.neuron_counts = None
        self.neuron_ids = None
        self.neuro_track_all = config.neuro_track_all

        # how many ticks the neuromorphic processor should run for
        self.neuro_tpc = config.neuro_tpc

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
    def get_default_encoders(neuro_tpc):
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
        v = max_forward_speed * (data[1] - data[0])
        w = max_turning_speed * (data[3] - data[2])
        return v, w

    def get_actions(self) -> tuple[float, float]:
        sensor_state = self.sensors.getState()
        sensor_detection_id = self.sensors.getDetectionId()
        self.set_color_by_id(sensor_detection_id)

        v, omega = self.run_processor(sensor_state)
        v *= 10
        self.requested = v, omega
        return self.requested
