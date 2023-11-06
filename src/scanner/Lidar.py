import numpy as np
from queue import Queue
from rplidar import RPLidar
import time
import threading
import Visualization
import PointCloud
from matplotlib import pyplot as plt
from scipy.stats import linregress

# convert lidar scan data (dist, angle from z axis, time) to x, y, z
# angular speed in degrees per second
def lidar2d_to_3d(scan, angular_speed=30, dist_from_axis=30):
    # convert lidar dist, lidar angle to x, y in lidar plane
    lidar_dist = scan[:, 0]
    lidar_angle = scan[:, 1]
    t = scan[:, 2]
    
    x, y = PointCloud.pol2cart(lidar_dist, lidar_angle)

    # shift coordinate space so that (0, 0) is on the axis of rotation
    r = dist_from_axis - x
    theta = -angular_speed * t
    z = y

    return PointCloud.cylindrical2cart(r, theta, z)

# perform dft to estimate angular speed of turntable
def estimate_angular_speed(dist, time, freq_range=(25, 45), show_plot=False):
# get the average dist for each unique time point
    unique_times = np.unique(time)
    averages = []

    sampling_rate = 1/(np.mean(np.diff(unique_times)))

    # Loop through each unique value and compute the average of the distances
    for t in unique_times:
        indices = np.where(time == t)  # Get the indices where the second coordinate equals the unique value
        mean_value = np.mean(dist[indices])  # Compute the mean of the first coordinate for these indices
        averages.append(mean_value)

    # # remove 'dc' offset
    averages = np.subtract(averages, np.mean(averages))

    # pad to 100000 samples to improve frequency resolution
    averages = np.pad(averages, (0, 100000), mode='constant')
    
    dft_result = np.fft.fft(averages)
    freqs = np.fft.fftfreq(len(averages), 1 / sampling_rate)

    if show_plot:
        # plot frequency spectrum
        plt.figure()
        plt.plot(np.abs(freqs) * 360, np.abs(dft_result))  # plot the magnitude spectrum
        plt.xlabel('angular speed (degrees/second)')
        plt.ylabel('Magnitude')
        plt.show()


    # get indices in range frequency min and frequency max
    freq_idx = ((freqs * 360) > 25) & ((freqs * 360) < 45)

    # Find the index of the maximum DFT magnitude:
    max_magnitude_idx = np.argmax(np.abs(dft_result[freq_idx]))

    # Get the frequency corresponding to the maximum DFT magnitude:
    max_magnitude_freq = np.abs(freqs[freq_idx][max_magnitude_idx])

    # get the weighted average of all frequencies
    # weighted_sum = np.sum(np.abs(freqs) * np.abs(dft_result))
    # sum_of_weights = np.sum(np.abs(dft_result))
    # weighted_average_frequency = weighted_sum / sum_of_weights

    estimated_angular_speed = max_magnitude_freq * 360
    print(f"Estimated angular speed: {estimated_angular_speed}")
    return estimated_angular_speed
    
# estimate the rotation needed to orient a line to the vertical
def estimate_vertical_rotation(points):
    # Extract x and y coordinates from the points
    x = [p[0] for p in points]
    y = [p[1] for p in points]

    # Perform linear regression
    slope = np.polyfit(x, y, 1)[0]

    # Calculate the angle with respect to the horizontal axis
    theta_vertical = np.arctan(-slope)

    # Convert angle to degrees
    theta_vertical_degrees = np.degrees(theta_vertical)
    
    # The rotation needed to make the line vertical
    rotation_needed_degrees = -theta_vertical_degrees
    return rotation_needed_degrees

# estimate the horizontal distance to a vertical line
def estimate_dist_to_line(points):
    x = points[:, 0]
    # estimate distance from axis of rotation
    mask = PointCloud.knn_filter(np.column_stack((x, np.full(x.shape, 0))), 5, 0.01) # get points that form veritcal line
    return np.median(x[mask[:, 0]])

# filter all points that arent approximately vertical
def get_vertical_line(points):
    x = points[:, 0]
    mask = PointCloud.knn_filter(np.column_stack((x, np.full(x.shape, 0))), 10, 0.01) # get points that form veritcal line
    return points[mask[:, 0]]

class Lidar():

    def __init__(self, port: str, dist_lim: tuple[float, float]=(0,60), angle_lim: tuple[float, float]=(30,150), angular_speed: float=34.2, dist_from_axis: float=30):
        """Initialize a lidar and start a background thread.

        Parameters
        ----------
        port : str
            Serial port name.
        dist_lim : (float, float)
            Lidar distance limits to include (cm).
        angle_lim : (float, float)
            Lidar angle limits to include (deg).
        angular_speed : float
            Angular speed of the turntable (deg/s).
            Currently ignored in favor of FFT.
        dist_from_axis : float
            Horizontal distance from lidar to the turntable's axis of rotation (cm).
        """

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
        self.turntable_height = 0
        self.start_scan_time = None
        self.background_points = np.array([])
        self.angular_bias = 0
        self.vertical_bias = 0
        self.horizontal_bias = 0

        # Start background thread to stream data from lidar
        self.kill_thread = False
        self.scan_thread = threading.Thread(target=self._scan_thread, daemon=True)
        self.scan_thread.start()
    
    def _scan_thread(self):
        """Background thread that samples from the lidar.
        If self.scanning is true, then self.curr_scan contains raw lidar data.
        This thread can be killed by setting self.kill_thread to true.
        """
        for i, scan in enumerate(self.lidar.iter_scans(min_len=1)):
            # add data to plotting queue
            scan_time = time.time()
            scan = np.array(scan)
            scan = scan[(scan[:, 0] == 15)]  # remove noisy points
            scan = scan[:, [2, 1]] # get data as [dist, angle]
            scan[:, 0] = scan[:, 0]/10 # convert from mm to cm

            # apply angular bias
            scan[:, 1] = scan[:, 1] + 180 + self.angular_bias # convert angles so that 0 degrees is the z axis 
            scan[:, 1][(scan[:, 1] > 180)] -= 360 
            
            # filter
            scan = scan[(scan[:, 0] > self.min_dist) & (scan[:,0] < self.max_dist)]
            scan = scan[(scan[:, 1] > self.min_ang) & (scan[:, 1] < self.max_ang)]

            # apply horizontal and vertical bias
            x, y = PointCloud.pol2cart(scan[:, 0], scan[:, 1])
            y = y + self.vertical_bias
            x = x + self.horizontal_bias
            scan[:, 0], scan[:, 1] = PointCloud.cart2pol(x, y)

            if self.plotting:
                x, y = PointCloud.pol2cart(scan[:, 0], scan[:, 1])
                self.plotting_buffer.put(np.column_stack((x, y)))

            # update cumulative scanning data
            if self.scanning:
                data_point_time = 0.15 * (scan[:, 1]/360) + (scan_time) - self.start_scan_time 
                scan_with_time = np.column_stack((scan, data_point_time)) # add third coordinate: time
                self.curr_scan = np.vstack((self.curr_scan, scan_with_time))
            
            if self.kill_thread:
                return

    def startScan(self, start_time=None):
        """Start a new scan, clearing previous scan data.
        """

        if self.scanning:
            raise("Already Scanning")
        
        if start_time == None:
            self.start_scan_time = time.time()
        else:
            self.start_scan_time = start_time

        self.curr_scan = np.empty((0, 3))
        self.scanning = True

    def stopScan(self):
        """Stop the scan.
        """

        self.scanning = False

    # estimates and sets the angular bias assuming the current scan is a vertical line
    # returns the horizontal distance to the vertical line after rotating it
    def calibrate_on_current(self):
        x, y = PointCloud.pol2cart(self.curr_scan[:, 0], self.curr_scan[:, 1])
        # points = get_vertical_line(np.column_stack((x, y)))
        for i in range(10):
            x, y = PointCloud.pol2cart(self.curr_scan[:, 0], self.curr_scan[:, 1] + self.angular_bias)
            self.angular_bias += estimate_vertical_rotation(np.column_stack((y, x)))
        dist =  estimate_dist_to_line(np.column_stack((x, y)))
        print(f"Esimated angular bias: {self.angular_bias}")
        print(f"Esimated horizontal distance: {dist}")
        return dist

    def set_bias(self, horizontal=None, angular=None, vertical=None):
        self.horizontal_bias = horizontal if horizontal is not None else self.horizontal_bias
        self.vertical_bias = vertical if vertical is not None else self.vertical_bias
        self.angular_bias = angular if angular is not None else self.angular_bias

    def set_lims(self, dist_lim=None, angle_lim=None):
        if dist_lim is None:
            dist_lim = (self.min_dist, self.max_dist)
        if angle_lim is None:
            angle_lim = (self.min_ang, self.max_ang)

        self.min_dist = dist_lim[0] 
        self.max_dist = dist_lim[1]
        self.min_ang = angle_lim[0]
        self.max_ang = angle_lim[1]

    def set_current_to_background(self, estimate_params=True):
        """Perform the calibration sequence using data from startScan-stopScan.
        """

        lidar_dist = self.curr_scan[:, 0]
        lidar_angle = self.curr_scan[:, 1]
        x, y = PointCloud.pol2cart(lidar_dist, lidar_angle)

        # get background points to filter out later
        self.background_points = np.column_stack((x,y))

        if estimate_params:
            # estimate distance from axis of rotation
            mask = PointCloud.knn_filter(np.column_stack((x, np.full(x.shape, 0))), 5, 0.01) # get points that form veritcal line
            self.turntable_height = y[mask[:, 0]].max() + 0.4
            self.dist_from_axis = np.median(x[mask[:, 0]]) + self.TURNTABLE_RADIUS
            print(f"Estimated dist from axis: {self.dist_from_axis}")
            print(f"Turntable Height: {self.turntable_height}")
            print("DONE CALIBRATION")
            return self.dist_from_axis, self.turntable_height
    
    def set_background_params(self, dist_from_axis, turntable_height):
        self.dist_from_axis = dist_from_axis
        self.turntable_height = turntable_height

    def remove_background_on_current(self):
        """Remove background points in-place from data from startScan-stopScan.
        """

        # convert lidar dist, lidar angle to x, y in lidar plane
        lidar_dist = self.curr_scan[:, 0]
        lidar_angle = self.curr_scan[:, 1]
        t = self.curr_scan[:, 2]
        
        x, y = PointCloud.pol2cart(lidar_dist, lidar_angle)
        
        # remove background points
        if self.background_points.size > 0:
            mask = PointCloud.subtract_point_clouds(np.column_stack((x, y)), self.background_points, 0.25)
            mask = mask & (y > self.turntable_height)
            lidar_dist = lidar_dist[mask]
            lidar_angle = lidar_angle[mask]
            t = t[mask]
            self.curr_scan = np.column_stack((lidar_dist, lidar_angle, t))
            
    def showPlot(self, thread_function):
        """Plot the lidar data (converted to cartesian x,y) in real time.

        Parameters
        ----------
        thread_function : Callable
            Function to run while plotting.
        """

        self.plotting_buffer = Queue()
        self.plotting = True

        t = threading.Thread(target=thread_function, daemon=True)
        t.start()
            
        def plot_callback():
            self.plotting = False
            self.disconnect()
        
        Visualization.plot2d_realtime(self.plotting_buffer, callback=plot_callback)

    def get3DPointCloud(self, scan: np.ndarray=None, angular_speed=None) -> np.ndarray:
        """Get an array of points (x,y,z) from the provided scan data or the current scan data.

        Parameters
        ----------
        scan : np.ndarray
            Array of lidar data. If None, use the current scan data.
        
        Returns
        -------
        np.ndarray
            Array of points.
        """

        if scan is None:
            scan = self.curr_scan

        if angular_speed == None:
            # estimate angular speed
            self.angular_speed = estimate_angular_speed(scan[:,0], scan[:, 2])
        else:
            self.angular_speed = angular_speed

        result = np.column_stack(lidar2d_to_3d(scan, self.angular_speed, self.dist_from_axis))

        return result

    def disconnect(self):
        """Stop and disconnect the lidar.
        """

        self.lidar.stop()
        self.lidar.stop_motor()
        self.lidar.disconnect()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """Attempt to kill the background thread. Also try to disconnect the lidar.
        """

        self.kill_thread = True
        self.scan_thread.join()
        self.disconnect()
