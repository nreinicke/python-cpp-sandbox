from ._core import __doc__, __version__, add, subtract, get_rho_air, TimeTrace

import pandas as pd

from pathlib import Path

def time_trace_from_csv(file: str, time_col = "time_s", speed_col = "speed_m__s") -> TimeTrace:
    path = Path(file)
    df = pd.read_csv(path)
    time = df[time_col].values
    speed = df[speed_col].values
    trace = TimeTrace(time, speed)
    return trace