%matplotlib notebook  

import time, serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque

PORT = "/dev/cu.usbserial-0001"
BAUD = 115200

HISTORY_SIZE = 800
READS_PER_FRAME = 10
INTERVAL_MS = 30

serialport = None

def get_mag_data():
    global serialport
    if serialport is None or not serialport.is_open:
        serialport = serial.Serial(PORT, BAUD, timeout=0.05)
        print("Opened", serialport.name)
        time.sleep(1.5)
        serialport.reset_input_buffer()

    line = serialport.readline().decode("utf-8", errors="ignore").strip()
    if not line.startswith("MAG:"):
        return None
    try:
        return tuple(map(float, line[4:].split(",")))
    except ValueError:
        return None

mag_x = deque(maxlen=HISTORY_SIZE)
mag_y = deque(maxlen=HISTORY_SIZE)
mag_z = deque(maxlen=HISTORY_SIZE)

fig, ax = plt.subplots()
ax.set_aspect("equal", adjustable="box")

# create artists ONCE
sc_xy = ax.scatter([], [], c="r", s=6, label="X–Y")
sc_yz = ax.scatter([], [], c="g", s=6, label="Y–Z")
sc_zx = ax.scatter([], [], c="b", s=6, label="Z–X")
ax.legend(loc="upper right")

anim = None
def onClick(event):
    if anim:
        anim.event_source.stop()

def animate(i):
    for _ in range(READS_PER_FRAME):
        ret = get_mag_data()
        if ret is None:
            continue
        x, y, z = ret
        mag_x.append(x); mag_y.append(y); mag_z.append(z)

    if len(mag_x) < 2:
        return sc_xy, sc_yz, sc_zx

    x = np.fromiter(mag_x, dtype=float)
    y = np.fromiter(mag_y, dtype=float)
    z = np.fromiter(mag_z, dtype=float)

    sc_xy.set_offsets(np.column_stack([x, y]))
    sc_yz.set_offsets(np.column_stack([y, z]))
    sc_zx.set_offsets(np.column_stack([z, x]))

    if len(mag_x) == HISTORY_SIZE:
        anim.event_source.stop()

    return sc_xy, sc_yz, sc_zx

fig.canvas.mpl_connect("button_press_event", onClick)
anim = FuncAnimation(fig, animate, interval=INTERVAL_MS, blit=True, cache_frame_data=False)
plt.show()
