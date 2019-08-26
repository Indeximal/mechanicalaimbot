import time
from collections import deque


class StopwatchData:
    def __init__(self, partials):
        self.partials = partials

    def latest_start_time(self):
        return self.partials[-1][1]

    def partial_durations_min_avg_max(self):
        durations = dict()

        for lap in self.partials:
            _, last = lap[0]
            for name, t in lap[1:]:
                if name in durations:
                    durations[name].append(t - last)
                else:
                    durations[name] = [t - last]
                last = t

        ret = dict()
        for name in durations:
            deltas = durations[name]
            ret[name] = (min(deltas), sum(deltas) / len(deltas), max(deltas))

        return ret

    def newest_lap_dict(self):
        start_name, last = self.partials[-1][0]
        ret = {start_name: last}
        for name, t in self.partials[-1][1:]:
            ret[name] = t - last
            last = t

    def avg_lap_time(self):
        if len(self.partials) < 2:
            return 0
        starts = [lap[0][1] for lap in self.partials]
        deltas = [t2 - t1 for t1, t2 in zip(starts[:-1], starts[1:])]
        return sum(deltas) / len(deltas)


class MultiStopwatch:
    """Util for measuring time differences"""
    def __init__(self, maxlaps=None):
        self.maxlaps = None
        self.partials_deque = deque(maxlen=maxlaps)

    def lap(self):
        self.partials_deque.append(list())
        self.partial("")

    def partial(self, name):
        self.partials_deque[-1].append((name, time.time()))

    def get_frozen_data(self):
        return StopwatchData(list(self.partials_deque))
