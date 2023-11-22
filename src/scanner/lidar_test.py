from rplidar_new import RPLidar
import logging
import threading
import time
import serial 
import subprocess  
import shlex

lidar_bottom = RPLidar('COM3', baudrate=115200, timeout=1)
lidar_top = RPLidar('COM4', baudrate=115200, timeout=1)
lidars = [lidar_top, lidar_bottom]

for lidar in lidars:
    info = lidar.get_info()
    print(info)

for lidar in lidars:
    health = lidar.get_health()
    print(health)

def scan(lidar):
    for i, scan in enumerate(lidar.iter_scans('express')):
        print('%d: Got %d measurments' % (i, len(scan)))
        print(scan)
        if i > 10:
            break
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()

bot_scan = threading.Thread(target=scan, args=(lidar_bottom,))
top_scan = threading.Thread(target=scan, args=(lidar_top,))

bot_scan.start()
top_scan.start()