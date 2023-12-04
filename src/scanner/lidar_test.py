# from rplidar_new import RPLidar
from Lidar import Lidar
from rplidar_new import RPLidar
import threading
import time
import numpy as np
import sys

lidar_bottom = RPLidar('COM3', baudrate=115200, timeout=3)
lidar_top = RPLidar('COM6', baudrate=115200, timeout=3)
lidars = [lidar_bottom, lidar_top]


# finds the com ports the two lidars are on
# determines which lidar is the top lidar
# returns a tuple (lidar_top, lidar_bottom)

for lidar in lidars:
    # sys.setswitchinterval(3000)
    info = lidar.get_info()
    health = lidar.get_health()
    print(info)
    print(health)

def scan(lidar):
    sys.setswitchinterval(3000)
    for i, scan in enumerate(lidar.iter_scans('express')):
        print('%d: Got %d measurments' % (i, len(scan)))
        # print(scan)
        if i > 10:
            break
        sys.setswitchinterval(0.005)
        time.sleep(0.0001)
        sys.setswitchinterval(3000) 
    sys.setswitchinterval(3000)    
    lidar.stop()
    lidar.disconnect()
    sys.setswitchinterval(0.005)
    time.sleep(0.001) 

def busy_work(lidar):
    pass

bot_scan = threading.Thread(target=scan, args=(lidar_bottom,))
top_scan = threading.Thread(target=scan, args=(lidar_top,))

bot_scan.start()
top_scan.start()