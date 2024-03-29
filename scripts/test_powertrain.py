import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

resdir = Path("../build/dev/") if Path("../build/dev/").exists() else Path("../build/")

df = pd.read_csv(resdir / "results.csv")

fig, ax = plt.subplots(2, 1, sharex=True, figsize=(10, 10))
ax[0].plot(df.time_s, df.fuel_pwr_w * 1e-3, label='fuel')
ax[0].plot(df.time_s, df.brake_pwr_w * 1e-3, label='brake')
ax[0].plot(df.time_s, df.req_pwr_w * 1e-3, label='req')
ax[0].plot(df.time_s, df.drag_pwr_w * 1e-3, label='drag')
ax[0].plot(df.time_s, df.accel_pwr_w * 1e-3, label='accel')
ax[0].plot(df.time_s, df.rr_pwr_w * 1e-3, label='rr')
ax[0].legend()
ax[0].set_ylabel('Power [kW]')

ax[-1].plot(df.time_s, df.speed_m__s, label='achieved')
ax[-1].plot(df.time_s, df.speed_m__s, label='prescribed', linestyle='--')
ax[-1].set_xlabel('Time [s]')
ax[-1].set_ylabel('Speed [m/s]')
ax[-1].legend()
plt.xlim([0, 200])

plt.show()