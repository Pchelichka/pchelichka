import sys

def sig_handler(_signo, _stack_frame_):
    sys.exit(0)

def clip(value: int, lower: int, upper: int):
    return lower if value < lower else upper if value > upper else value

def channel_value(value: float):
    return clip(round(value), 1000, 2000)

