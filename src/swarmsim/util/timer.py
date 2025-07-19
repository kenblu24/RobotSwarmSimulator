import time


class Timer:
    def __init__(self, name=''):
        self.restart()
        self.name = name
        self.stop = None

    def stop(self):
        self.stop = time.time()
        return self.check_watch()

    def restart(self):
        self.start = time.time()

    @property
    def elapsed(self):
        if self.stop is not None:
            return self.stop - self.start
        else:
            return time.time() - self.start

    def __call__(self):
        return self.elapsed

    def __str__(self):
        return f"{self.name} : {self.elapsed}s"

    def __repr__(self):
        return f"<Timer{' ' if self.name else ''}{self.name}: start={self.start}, {f'stop={self.stop}, ' if self.stop is not None else ''}elapsed={self.elapsed}s>"
