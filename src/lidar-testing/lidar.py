import numpy as np
from queue import Queue
from rplidar import RPLidar
import time
import threading
import Visualization
import PointCloud
from matplotlib import pyplot as plt

# converts 2d polar (in degrees) to 2d cartesian
def pol2cart(radius, angle):
    x = radius * np.cos(np.radians(angle))
    y = radius * np.sin(np.radians(angle))
    return(x, y)

def cart2pol(x, y):
    r = np.sqrt(np.square(x) + np.square(y))
    theta = np.degrees(np.arctan2(y, x))
    return r, theta

def cylindrical2cart(r, theta, z):
    x, y = pol2cart(r, theta)
    return (x, y, z)

# convert lidar scan data (dist, angle from z axis, time) to x, y, z
# angular speed in degrees per second
def lidar2d_to_3d(scan, angular_speed=30, dist_from_axis=30):
    # convert lidar dist, lidar angle to x, y in lidar plane
    lidar_dist = scan[:, 0]
    lidar_angle = scan[:, 1]
    t = scan[:, 2]
    
    y, x = pol2cart(lidar_dist, lidar_angle)

    # shift coordinate space so that (0, 0) is on the axis of rotation
    r = dist_from_axis - x
    theta = -angular_speed * t
    z = y

    return cylindrical2cart(r, theta, z)

# perform dft to estimate angular speed of turntable
def estimate_angular_speed(dist, time, show_plot=False):
# get the average dist for each unique time point
    unique_times = np.unique(time)
    averages = []

    # Loop through each unique value and compute the average of the distances
    for t in unique_times:
        indices = np.where(time == t)  # Get the indices where the second coordinate equals the unique value
        mean_value = np.mean(dist[indices])  # Compute the mean of the first coordinate for these indices
        averages.append(mean_value)

    # remove 'dc' offset
    averages = np.subtract(averages, np.mean(averages))

    # pad to 100000 samples to improve frequency resolution
    averages = np.pad(averages, (0, 100000), mode='constant')
    
    sampling_rate = 1/(np.mean(np.diff(unique_times)))
    dft_result = np.fft.fft(averages)
    freqs = np.fft.fftfreq(len(averages), 1 / sampling_rate)

    if show_plot:
        # plot frequency spectrum
        plt.figure()
        plt.plot(np.abs(freqs) * 360, np.abs(dft_result))  # plot the magnitude spectrum
        plt.xlabel('angular speed (degrees/second)')
        plt.ylabel('Magnitude')
        plt.show()

    # Find the index of the maximum DFT magnitude:
    max_magnitude_idx = np.argmax(np.abs(dft_result))

    # Get the frequency corresponding to the maximum DFT magnitude:
    max_magnitude_freq = np.abs(freqs[max_magnitude_idx])

    # get the weighted average of all frequencies
    # weighted_sum = np.sum(np.abs(freqs) * np.abs(dft_result))
    # sum_of_weights = np.sum(np.abs(dft_result))
    # weighted_average_frequency = weighted_sum / sum_of_weights

    estimated_angular_speed = max_magnitude_freq * 360
    print(f"Estimated angular speed: {estimated_angular_speed}")
    return estimated_angular_speed
    
class Lidar():

    def __init__(self, port, dist_lim=(0,60), angle_lim=(30,150), angular_speed=34.2, dist_from_axis=30):
        self.TURNTABLE_RADIUS = 8 # cm
        self.lidar = RPLidar(port)
        self.curr_scan = np.empty((0, 3))
        self.plotting_buffer =  Queue()
        self.plotting = False
        self.scanning = False
        self.min_dist = dist_lim[0]
        self.max_dist = dist_lim[1]
        self.min_ang = angle_lim[0]
        self.max_ang = angle_lim[1]
        self.angular_speed = angular_speed
        self.dist_from_axis = dist_from_axis
        self.start_scan_time = None
        self.background_points = np.array([])

        def scanThread():
            for i, scan in enumerate(self.lidar.iter_scans()):
                # add data to plotting queue
                scan = np.array(scan)
                scan = scan[:, [2, 1]] # get data as [dist, angle]
                scan[:, 0] = scan[:, 0]/10 # convert from mm to cm
                # filter
                scan = scan[(scan[:, 0] > self.min_dist) & (scan[:,0] < self.max_dist)]
                scan = scan[(scan[:, 1] > self.min_ang) & (scan[:, 1] < self.max_ang)]

                if self.plotting:
                    x, y = pol2cart(scan[:,0], scan[:, 1])
                    self.plotting_buffer.put(np.column_stack((y, x)))

                # update cumulative scanning data
                if self.scanning:
                    self.scan_time = time.time() - self.start_scan_time
                    scan_with_time = np.column_stack((scan, np.full(scan[:,0].shape, self.scan_time))) # add third coordinate: time
                    self.curr_scan = np.vstack((self.curr_scan, scan_with_time))
    
        t = threading.Thread(target=scanThread, daemon=True)
        t.start()

    def calibrate(self, calibration_time=5):
        if self.scanning:
            raise("There is an ongoing scan, cannot calibrate")
        
        print(f"START CALIBRATION, DURATION: {calibration_time} s")
        self.startScan()
        time.sleep(calibration_time)
        self.stopScan()

        lidar_dist = self.curr_scan[:, 0]
        lidar_angle = self.curr_scan[:, 1]
        y, x = pol2cart(lidar_dist, lidar_angle)

        # get background points to filter out later
        self.background_points = np.column_stack((x,y))
        
        print(x)

        # estimate distance from axis of rotation
        mask = PointCloud.filter(np.column_stack((x, np.full(x.shape, 0))), 5, 0.01) # get points that form veritcal line

        print(mask)

        self.dist_from_axis = np.median(x[mask[:, 0]]) + self.TURNTABLE_RADIUS
        print(f"Estimated dist from axis: {self.dist_from_axis}")

        print("DONE CALIBRATION")

    def remove_background(self):
        # convert lidar dist, lidar angle to x, y in lidar plane
        lidar_dist = self.curr_scan[:, 0]
        lidar_angle = self.curr_scan[:, 1]
        t = self.curr_scan[:, 2]
        
        y, x = pol2cart(lidar_dist, lidar_angle)
        
        # remove background points
        if self.background_points.size > 0:
            mask = PointCloud.subtract_point_clouds(np.column_stack((x, y)), self.background_points, 0.25)
            lidar_dist = lidar_dist[mask]
            lidar_angle = lidar_angle[mask]
            t = t[mask]
            self.curr_scan = np.column_stack((lidar_dist, lidar_angle, t))
            
    
    def startScan(self):
        if self.scanning:
            raise("Already Scanning")
        
        self.curr_scan = np.empty((0, 3))
        self.start_scan_time = time.time()
        self.scanning = True
        
    def showPlot(self, thread_function):
        self.plotting_buffer = Queue()
        self.plotting = True

        t = threading.Thread(target=thread_function, daemon=True)
        t.start()
            
        def plot_callback():
            self.plotting = False
            self.disconnect()
        
        Visualization.plot2d_realtime(self.plotting_buffer, callback=plot_callback)

    # stop scan and remove background
    def stopScan(self):
        self.scanning = False

    def get3DPointCloud(self):
        # estimate angular speed
        self.angular_speed = estimate_angular_speed(self.curr_scan[:,0], self.curr_scan[:, 2])

        result = np.column_stack(lidar2d_to_3d(self.curr_scan, self.angular_speed, self.dist_from_axis))

        # filter outliers
        # mask = PointCloud.filter(result, 3, 1)
        # print(mask)

        return result
        
    def disconnect(self):
        self.lidar.stop()
        self.lidar.stop_motor()
        self.lidar.disconnect()