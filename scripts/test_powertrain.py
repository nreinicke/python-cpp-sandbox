import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("../build/dev/results.csv")

fig, ax = plt.subplots(2, 1, sharex=True, figsize=(10, 10))
ax[0].plot(df.time, df.fuel_pwr_w * 1e-3, label='fuel')
ax[0].plot(df.time, df.brake_pwr_w * 1e-3, label='brake')
ax[0].plot(df.time, df.req_pwr_w * 1e-3, label='req')
ax[0].plot(df.time, df.drag_pwr_w * 1e-3, label='drag')
ax[0].plot(df.time, df.accel_pwr_w * 1e-3, label='accel')
ax[0].plot(df.time, df.rr_pwr_w * 1e-3, label='rr')
ax[0].legend()
ax[0].set_ylabel('Power [kW]')

ax[-1].plot(df.time, df.speed, label='achieved')
ax[-1].plot(df.time, df.speed, label='prescribed', linestyle='--')
ax[-1].set_xlabel('Time [s]')
ax[-1].set_ylabel('Speed [m/s]')
ax[-1].legend()
plt.xlim([0, 200])

plt.show()