from rplidar import RPLidar
import numpy as np
import queue
from datetime import datetime
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from random import randrange
import threading
import time
import math

lidarData = queue.Queue()
lidar = RPLidar('com5')
fig, ax = plt.subplots()
sc = ax.scatter([], [])
ax.set_xlim([-10, 10])
ax.set_ylim([-10, 10])

def pol2cart(rho, phi):
    x = rho * np.cos(np.radians(phi))
    y = rho * np.sin(np.radians(phi))
    return(x, y)

# def connectLidar(port):
#     info = lidar.get_info()
#     print(info)
#     health = lidar.get_health()
#     print(health)
#     return lidar

def producerThread():
    for i, scan in enumerate(lidar.iter_scans()):
        print('%d: Got %d measurments' % (i, len(scan)))
        if lidarData.empty():
            lidarData.put(scan)

def consumerThread(frame):
    try:
        data = np.array(lidarData.get_nowait())
        dist = data[:, 2]
        angle = data[:, 1]

        x, y = pol2cart(dist, angle)

        sc.set_offsets(list(zip(x/200, y/200)))
        # Set sizes...
        return sc
    except Exception:
        return sc

producer = threading.Thread(target=producerThread, daemon=True)
producer.start()

ani = FuncAnimation(fig, consumerThread, interval=200)

plt.show()
