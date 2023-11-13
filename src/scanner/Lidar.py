import numpy as np
from queue import Queue
from rplidar import RPLidar, RPLidarException
import time
import threading
import Visualization
import PointCloud
from matplotlib import pyplot as plt
from scipy.optimize import minimize
from scipy.spatial import procrustes
from scipy.signal import argrelextrema, correlate

# convert lidar scan data (dist, angle from z axis, time) to x, y, z
# angular speed in degrees per second
def lidar2d_to_3d(scan, angular_speed=30, pos=(0,0,0), angular_pos=(0,0,0)):
    # convert lidar dist, lidar angle to x, y in lidar plane
    lidar_dist = scan[:, 0]
    lidar_angle = scan[:, 1]
    t = scan[:, 2]
    
    x, z = PointCloud.pol2cart(lidar_dist, lidar_angle)
    y_placeholder = np.full(x.shape, 0)
    lidar_points_unrotated = np.column_stack((x, y_placeholder, z))

    # frame of reference of lidar
    lidar_points = PointCloud.rotate_points(angular_pos[0], angular_pos[1], angular_pos[2], lidar_points_unrotated)
    lidar_points[:,0] = -lidar_points[:, 0]

    def yaw_rotation(yaws):
        # Convert angles from degrees to radians
        yaws = np.radians(yaws)

        # Create the batch of yaw rotation matrices
        cos_yaws = np.cos(yaws)
        sin_yaws = np.sin(yaws)

        # Construct the rotation matrices
        Rz = np.zeros((yaws.size, 3, 3))
        Rz[:, 0, 0] = cos_yaws
        Rz[:, 0, 1] = -sin_yaws
        Rz[:, 1, 0] = sin_yaws
        Rz[:, 1, 1] = cos_yaws
        Rz[:, 2, 2] = 1

        return Rz

    # efficient method for rotating a list of points by a list of yaws
    def apply_yaw_to_points(points, yaws):
        R = yaw_rotation(yaws)
        # Apply rotation to each point
        transformed_points = np.einsum('ijk,ik->ij', R, points)
        
        return transformed_points

    # points in frame of reference of axis of rotation    
    points_unrotated = pos + lidar_points # assume lidar is facing (0,0,0) so subtract instead of add
    yaw = -angular_speed * t  # yaw
    points = apply_yaw_to_points(yaws=yaw, points=points_unrotated)
    points = np.reshape(points, scan.shape)
    return points

# perform dft to estimate angular speed of turntable
def estimate_angular_speed(dist, time, freq_range=(25, 45), show_plot=False):
    # get the average dist for each unique time point
    unique_times = np.unique(time)
    averages = []

    sampling_rate = 1/(np.mean(np.diff(unique_times)))

    # Loop through each unique value and compute the average of the distances
    for t in unique_times:
        indices = np.where(time == t)  # Get the indices where the second coordinate equals the unique value
        mean_value = np.median(dist[indices])
        averages.append(mean_value)

    # remove 'dc' offset
    averages = averages - np.mean(averages)

    # pad to 100000 samples to improve frequency resolution
    averages = np.pad(averages, (0, 100000), mode='constant')
    dft_result = np.fft.fft(averages)  
    freqs = np.fft.fftfreq(len(averages), 1/sampling_rate) * 360

    if show_plot:
        # plot frequency spectrum
        plt.figure()
        plt.plot(np.abs(freqs), np.abs(dft_result))  # plot the magnitude spectrum
        plt.xlabel('angular speed (degrees/second)')
        plt.ylabel('Magnitude')
        plt.show()

    def find_dominant_frequency(dft, freqs, min_freq, max_freq):
        """
        Optimized function to find the frequency within min_freq and max_freq whose multiples have the greatest combined weight in the DFT.

        :param dft: Array of DFT magnitudes
        :param freqs: Array of frequencies corresponding to the DFT magnitudes
        :param min_freq: Minimum frequency of the range to consider
        :param max_freq: Maximum frequency of the range to consider
        :return: The frequency with the greatest combined weight of its multiples
        """
        # Precompute multiples for each frequency
        dft = np.abs(dft)
        freq_multiples = {f: f * np.arange(1, int(freqs.max() / f) + 1) for f in freqs if min_freq <= f <= max_freq}
        freq_multiples = {f: m for f, m in freq_multiples.items()}

        # Find the dominant frequency
        max_combined_weight = 0
        dominant_freq = None

        for freq, multiples in freq_multiples.items():
            indices = np.nonzero(np.isin(freqs, multiples))[0]
            combined_weight = np.sum(dft[indices])

            if combined_weight > max_combined_weight:
                max_combined_weight = combined_weight
                dominant_freq = freq

        return dominant_freq
    domanant_freq = find_dominant_frequency(dft_result, freqs, 25, 45)
    print(f"Estimated angular speed: {domanant_freq} deg/sec")
    return domanant_freq

pc_combined = None
def calibrate_lidars(top_scan, bottom_scan, initial_guess, ground_truth=None):
    global pc_combined
    angular_speed = estimate_angular_speed(bottom_scan[:, 0], 
                                           bottom_scan[:, 2])
    # if ground_truth is not None:
    #     ground_truth = ground_truth[(ground_truth[:, 2] > (ground_truth.min() + 2)) & (ground_truth[:, 2] < (ground_truth.max() - 2))]

    def loss_function(params):
        global pc_combined
        pos_top = params[0:3]
        angle_top = params[3:6]
        pos_bottom = params[6:9]
        angle_bottom = params[9:12]
        
        pc_top = lidar2d_to_3d(top_scan, angular_speed, pos=pos_top, angular_pos=angle_top)
        pc_bottom = lidar2d_to_3d(bottom_scan, angular_speed, pos=pos_bottom, angular_pos=angle_bottom)

        # pc_top = pc_top[(pc_top[:, 2] > (pc_top.min() + 2)) & (pc_top[:, 2] < (pc_top.max() - 2))]
        # pc_bottom = pc_bottom[(pc_bottom[:, 2] > (pc_bottom.min() + 2)) & (pc_bottom[:, 2] < (pc_bottom.max() - 2))]

        # make sure point clouds are aligned
        base_loss = 0    
        pc_combined = np.vstack((pc_top, pc_bottom))
        if ground_truth is not None:
            T, _ ,_ = PointCloud.icp(pc_combined, ground_truth, max_iterations=100)
            pc = PointCloud.apply_icp_transform(pc_combined, T)
            base_loss = PointCloud.chamfer_distance(pc, ground_truth)
        else:
            # want them to line up but not be too far from initial guess
            initial_guess_pc = lidar2d_to_3d(bottom_scan, angular_speed, pos=initial_guess[6:9], angular_pos=initial_guess[9:12])
            base_loss = PointCloud.chamfer_distance(pc_combined, initial_guess_pc)
        
        disalignment_loss = PointCloud.chamfer_distance(pc_top, pc_bottom)
        loss = base_loss + 2 * disalignment_loss

        # Visualization.plot3d(pc)
        # Visualization.plot3d(pc)
        # time.sleep(10)
        print(f"base loss: {base_loss}, d loss: {disalignment_loss}")
        print(loss)
        return loss

    optimized_params = minimize(loss_function, initial_guess, method='SLSQP').x
    print("Calibration parameters:", optimized_params)
    return optimized_params, pc_combined

class Lidar():

    def __init__(self, port: str, dist_lim: tuple[float, float]=(0,60), angle_lim: tuple[float, float]=(30,150), pos=(0,0,0), angular_pos=(0,0,0)):
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
        self.buffer =  Queue()
        self.user_request_data = False
        self.scanning = False
        self.min_dist = dist_lim[0]
        self.max_dist = dist_lim[1]
        self.min_ang = angle_lim[0]
        self.max_ang = angle_lim[1]
        self.angular_speed = 0
        self.dist_from_axis = 0
        self.turntable_height = 0
        self.start_scan_time = None
        self.background_data = np.array([])
        self.pos = pos
        self.angular_pos = angular_pos

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
            scan[:, 1] = scan[:, 1] + 180  # convert angles so that 0 degrees is the z axis 
            scan[:, 1][(scan[:, 1] > 180)] -= 360 
            
            # filter
            scan = scan[(scan[:, 0] > self.min_dist) & (scan[:,0] < self.max_dist)]
            scan = scan[(scan[:, 1] > self.min_ang) & (scan[:, 1] < self.max_ang)]

            # apply horizontal and vertical bias
            x, y = PointCloud.pol2cart(scan[:, 0], scan[:, 1])
            scan[:, 0], scan[:, 1] = PointCloud.cart2pol(x, y)

            if self.user_request_data:
                x, y = PointCloud.pol2cart(scan[:, 0], scan[:, 1])
                self.buffer.put(np.column_stack((x, y)))

            # update cumulative scanning data
            if self.scanning:
                data_point_time = 0.00 * (scan[:, 1]/360) + (scan_time) - self.start_scan_time 
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
    def calibrate_on_current(self, ground_truth, initial_guess):
        angular_speed = estimate_angular_speed(self.curr_scan[:, 0], 
                                               self.curr_scan[:, 2])
        
        # ideal_cube = PointCloud.stl_to_mesh('calibration/rect.stl')
        # ideal_cube_pc = PointCloud.mesh_to_pc(ideal_cube, 5000)
        ground_truth = ground_truth[(ground_truth[:, 2] > (ground_truth.min() + 2)) & (ground_truth[:, 2] < (ground_truth.max() - 2))]
        def loss_function(params):
            pos = params[:3]
            angular_pos = params[3:6]

            pc = lidar2d_to_3d(self.curr_scan, angular_speed, pos=pos, angular_pos=angular_pos)
            pc = pc[(pc[:, 2] > (pc.min() + 2)) & (pc[:, 2] < (pc.max() - 2))]

            # make sure point clouds are aligned
            T, _ ,_ = PointCloud.icp(pc, ground_truth, max_iterations=100)
            pc = PointCloud.apply_icp_transform(pc, T)

            # Visualization.plot3d(pc)
            # Visualization.plot3d(pc)
            # time.sleep(10)

            chamfer_loss = PointCloud.chamfer_distance(pc, ground_truth)
            return chamfer_loss

        optimized_params = minimize(loss_function, initial_guess, method='SLSQP').x
        print("Calibration parameters:", optimized_params)
        
        # set parameters
        self.pos = optimized_params[:3]
        self.angular_pos = optimized_params[3:]
        return optimized_params

    def set_params(self, params):
        self.pos = params[:3]
        self.angular_pos = params[3:]

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
        dist = self.curr_scan[:, 0]
        angle = self.curr_scan[:, 1]
        x, z =  PointCloud.pol2cart(dist, angle)

        # get background points to filter out later
        self.background_data = self.curr_scan

        if estimate_params:
            # estimate distance from axis of rotation
            mask = PointCloud.knn_filter(np.column_stack((x, np.full(x.shape, 0))), 5, 0.01) # get points that form veritcal line
            self.turntable_height = z[mask[:, 0]].max() + 0.4
            self.dist_from_axis = np.median(x[mask[:, 0]]) + self.TURNTABLE_RADIUS
            print(f"Estimated dist from axis: {self.dist_from_axis}")
            print(f"Turntable Height: {self.turntable_height}")
            print("DONE CALIBRATION")
            return self.dist_from_axis, self.turntable_height
    
    def set_background_params(self, dist_from_axis, turntable_height):
        self.dist_from_axis = dist_from_axis
        self.turntable_height = turntable_height

    def remove_background_on_current(self, use_calib_params=True):
        """Remove background points in-place from data from startScan-stopScan.
        """
        if use_calib_params:
            # convert lidar dist, lidar angle to x, y in lidar plane
            pc = self.get3DPointCloud(angular_speed=0)
            x = pc[:, 0]
            z = pc[:, 2]
        else:
            dist = self.curr_scan[:, 0]
            angle = self.curr_scan[:, 1]
            x, z =  PointCloud.pol2cart(dist, angle)
        
        background_points = self.get3DPointCloud(self.background_data, angular_speed=0)
        background_points = background_points[:, [0, 2]]

        # remove background points
        if background_points.size > 0:
            mask = PointCloud.subtract_point_clouds(np.column_stack((x, z)), background_points, 0.25)
            mask = mask & (z > self.turntable_height)
            self.curr_scan = self.curr_scan[mask]
            
    def get_buffer(self):
        """Get buffer of the 2d lidar data that is updated in realtime

        Parameters
        ----------
        thread_function : Callable
            Function to run while plotting.
        """
        self.buffer = Queue()
        self.user_request_data = True
        return self.buffer

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

        result = lidar2d_to_3d(scan, self.angular_speed, pos=self.pos, 
                                        angular_pos=self.angular_pos)

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
