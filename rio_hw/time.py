import time

sleep = time.sleep
_now = time.monotonic
# _now = time.perf_counter
# _now = time.time
_now_ns = time.monotonic_ns
# _now_ns = time.perf_counter_ns
# _now_ns = time.time_ns


TIME_OFFSET = round(time.time() - _now())
TIME_OFFSET_NS = TIME_OFFSET * int(1e9)


def now():  # monotonic unix time
    return _now() + TIME_OFFSET


def now_ns():  # monotonic unix time ns
    return _now_ns() + TIME_OFFSET_NS


class Rate:
    def __init__(self, freq):
        self.period = 1.0 / freq if freq > 0 else 0
        self.start_time = now()

    def real_freq(self):
        return 1.0 / (now() - self.start_time)

    def sleep(self):
        dt = self.start_time + self.period - now()
        if dt > 0:
            sleep(dt)
            self.start_time += self.period
        else:
            self.start_time = now()

    def precise_sleep(self):
        end_time = self.start_time + self.period
        precise_wait(end_time)
        self.start_time = end_time

    def __enter__(self):
        self.start_time = now()
        return self

    def __exit__(self, *args):
        precise_wait(self.start_time + self.period)


def precise_sleep(dt: float, slack_dt: float = 0.001):
    # combine time.sleep and spinning to minimize jitter
    t_start = _now()
    if dt > slack_dt:
        sleep(dt - slack_dt)
    t_end = t_start + dt
    while _now() < t_end:
        pass


def precise_wait(t_end: float, slack_dt: float = 0.001):
    # combine time.sleep and spinning to minimize jitter
    t_start = now()
    t_wait = t_end - t_start
    if t_wait > 0:
        t_sleep = t_wait - slack_dt
        if t_sleep > 0:
            sleep(t_sleep)
        while now() < t_end:
            pass
