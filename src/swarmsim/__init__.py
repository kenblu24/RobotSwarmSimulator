# __all__ = [

# ]

try:
    import importlib.metadata
    __version__ = importlib.metadata.version(__package__ or "swarmsim")
except (ImportError, StopIteration):
    __version__ = "unknown"


def print_debugversions():
    """Prints the versions of the operating system and Python."""
    import platform
    import numpy
    import scipy
    print(f"RobotSwarmSimulator: {__version__}")
    print(f"OS: {platform.platform()}")
    print(f"Python: {platform.python_version()}")
    print(f"Numpy: {numpy.__version__}")
    print(f"Scipy: {scipy.__version__}")
