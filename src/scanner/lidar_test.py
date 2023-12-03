# from rplidar_new import RPLidar
from Lidar import Lidar
# from rplidar_new import RPLidar
import threading
import time
import numpy as np

# lidar_bottom = RPLidar('COM3', baudrate=115000, timeout=1)
# lidar_top = RPLidar('COM5', baudrate=115000, timeout=1)
# lidars = [lidar_top, lidar_bottom]


import serial.tools.list_ports
ports = serial.tools.list_ports.comports()

# finds the com ports the two lidars are on
# determines which lidar is the top lidar
# returns a tuple (lidar_top, lidar_bottom)
def auto_get_lidar(top_dist_lim, bot_dist_lim, top_ang_lim, bot_ang_lim):
    ports = serial.tools.list_ports.comports()

    lidar_ports = []
    for port, desc, hwid in sorted(ports):
        if "Silicon Labs CP210x" in desc:
            lidar_ports.append(port)

    if len(lidar_ports) < 2:
        raise ValueError("Less than two lidar ports found.")

    lidar1 = Lidar(lidar_ports[0], dist_lim=(0, 50), angle_lim=(-90, -60))
    lidar2 = Lidar(lidar_ports[1], dist_lim=(0, 50), angle_lim=(-90, -60))

    # Assuming Lidar.startScan(), Lidar.stopScan(), Lidar.get3DPointCloud(), and Lidar.disconnect() are defined elsewhere
    lidar1.startScan()
    lidar2.startScan()
    time.sleep(2)
    lidar1.stopScan()
    lidar2.stopScan()

    pc1 = lidar1.get3DPointCloud(angular_speed=0)
    pc2 = lidar2.get3DPointCloud(angular_speed=0)

    mean1 = np.nanmean(pc1[:, 2]) if np.any(np.isfinite(pc1[:, 2])) else float('inf')
    mean2 = np.nanmean(pc2[:, 2]) if np.any(np.isfinite(pc2[:, 2])) else float('inf')

    lidar1.disconnect()
    lidar2.disconnect()

    return (lidar1, lidar2) if mean1 <= mean2 else (lidar2, lidar1)

print(sort_lidars_by_mean())


# for lidar in lidars:
#     info = lidar.get_info()
#     print(info)

# for lidar in lidars:
#     health = lidar.get_health()
#     print(health)

# def scan(lidar):
#     for i, scan in enumerate(lidar.iter_scans('express')):
#         print('%d: Got %d measurments' % (i, len(scan)))
#         print(scan)
#         if i > 10:
#             break
#     lidar.stop()
#     lidar.stop_motor()
#     lidar.disconnect()

# bot_scan = threading.Thread(target=scan, args=(lidar_bottom,))
# top_scan = threading.Thread(target=scan, args=(lidar_top,))

# bot_scan.start()
# top_scan.start()