# Robot Swarm Simulator
Contributors: Connor Mattson, Jeremy Clark, Daniel S. Brown, Kevin Zhu

## Documentation

[Online Documentation](https://kenblu24.github.io/RobotSwarmSimulator/guide/install.html)

## Setup
Install RSS with pip
    
    pip install --upgrade git+ssh://git@github.com/kenblu24/RobotSwarmSimulator.git@main

or, for faster install with uv (`pip install uv`) preface any pip commands with `uv`, i.e. `uv pip install...`

To install as editable:

    git clone git@github.com:kenblu24/RobotSwarmSimulator.git
    cd RobotSwarmSimulator
    pip install -e .

See also:
* [Installation Guide](https://kenblu24.github.io/RobotSwarmSimulator/guide/install.html)
* [Development Installation](https://kenblu24.github.io/RobotSwarmSimulator/devel/install.html)

<!-- Test Simulation

    python -m demo.simulation.cyclic_pursuit

Test Evolution (Novelty Search) - Will take a long time to evolve.

    python -m demo.evolution.novelty_search -->


## Demos

**Code for novelty search has moved to https://github.com/kenblu24/novelswarms-es**

We invite you to augment cautiously and carefully test output validity.
