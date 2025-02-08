import pathlib as pl

from ..datasets.GenomeDataSet import GenomeDataSet


data_path = pl.Path(__file__).parent / '../../data/diffdrivegenomes'
data_path = data_path.expanduser().absolute()

preset_behaviors = {'cyclic', 'aggregation', 'wall-following', 'dispersal', 'milling', 'random'}
_PRESET_BEHAVIOR_FILES = {name: (data_path / name).with_suffix('.yaml') for name in preset_behaviors}
filenames = _PRESET_BEHAVIOR_FILES


class DiffDriveDataset(GenomeDataSet):



    def __init__(self, file=None, array_like=None):
        self.CYCLIC_PURSUIT = GenomeDataSet(filenames["cyclic"], name="cyclic_pursuit")
        self.AGGREGATION = GenomeDataSet(filenames["aggregation"], name="aggregation")
        self.WALL_FOLLOWING = GenomeDataSet(filenames["wall-following"], name="wall_following")
        self.DISPERSAL = GenomeDataSet(filenames["dispersal"], name="dispersal")
        self.MILLING = GenomeDataSet(filenames["milling"], name="milling")
        self.RANDOM = GenomeDataSet(filenames["random"], name="random")

        super().__init__(file=file, array_like=array_like)
