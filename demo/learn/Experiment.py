import time
import pathlib as pl

DEFAULT_PARENT = "out"


class Experiment:
    def __init__(self, root=None, title=None):
        if root is None:
            root = pl.Path.cwd() / DEFAULT_PARENT
        if title is None:
            title = f"e{int(time.time())}"

        self.parent = pl.Path(root)
        path = self.parent / title

        if not self.parent.is_dir():
            self.parent.mkdir(exist_ok=False, parents=True)
        elif self.parent.is_file():
            raise Exception("The parent directory is a file")

        i = 1
        while path.exists():
            path = path.with_name(f"{title}_{i}")
            i += 1

        path.mkdir()
        self.path = path

    def add_sub(self, title):
        """
        Given a title, return the path to a new directory created under the experiment with name title
        """
        if title is None:
            raise Exception("The add_sub method requires a non-null title parameter")

        new_dir = self.path / title
        new_dir.mkdir(exist_ok=True, parents=False)
        return new_dir
