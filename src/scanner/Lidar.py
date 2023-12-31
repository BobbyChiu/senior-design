import numpy as np
from queue import Queue
from rplidar_new import RPLidar
import time
import threading
import Visualization
import PointCloud
from matplotlib import pyplot as plt
from scipy.optimize import minimize
from sklearn.decomposition import PCA
import serial.tools.list_ports
pc_combined = None
pc_top = None
pc_bottom = None
iter = 0

def start_stop_scan(lidars, scan_time):
    start_time = time.time()
    for l in lidars:
        l.startScan(start_time)
    time.sleep(scan_time)
    for l in lidars:
        l.stopScan()

# estimate the positions and orientations of the lidars
def calibrate(lidar_top, lidar_bottom, initial_guess, ref):
    print(lidar_top.curr_scan.shape)
    angular_speed_top = estimate_angular_speed(lidar_top.curr_scan)
    angular_speed_bottom = estimate_angular_speed(lidar_bottom.curr_scan)
    angular_speed = (angular_speed_top + angular_speed_bottom)/2

    initial_guess = np.append(initial_guess, [0, 0, 0, 0, 0, 0])

    # remove outliers from lidar scans
    top = lidar2d_to_3d(lidar_top.curr_scan, angular_speed=angular_speed)
    bottom = lidar2d_to_3d(lidar_bottom.curr_scan, angular_speed=angular_speed)
    top, ind = PointCloud.remove_statistical_outliers(top, 20, 2)
    lidar_top.curr_scan = lidar_top.curr_scan[ind]
    bottom, ind = PointCloud.remove_statistical_outliers(bottom, 20, 2)
    lidar_bottom.curr_scan = lidar_bottom.curr_scan[ind]

    print(lidar_top.curr_scan.shape)

    # preprocess reference object
    ref_height = ref[:, 2].max() - ref[:, 2].min()
    ref_min = ref[:,2].min()
    ref_no_bottom = ref[ref[:, 2] > (ref_min + ref_height * 0.1)]

    # minimize over all parameters
    def loss_all_params(params):
        global pc_combined
        global pc_bottom
        global pc_top
        global iter

        pos_top = params[0:3]
        pos_bottom = params[3:6]
        angle_top = params[6:9]
        angle_bottom = params[9:12]
        transformation = params[12:18]

        # current scan estimate
        pc_top = lidar2d_to_3d(lidar_top.curr_scan, angular_speed, pos=pos_top, angular_pos=angle_top)
        pc_bottom = lidar2d_to_3d(lidar_bottom.curr_scan, angular_speed, pos=pos_bottom, angular_pos=angle_bottom)
        pc_ref = np.vstack((pc_top, pc_bottom))

        # find error due to difference between reference scans and reference object 
        pc_ref = PointCloud.apply_transformation(transformation, pc_ref)

        reconstrction_loss = PointCloud.chamfer_distance(pc_ref, ref_no_bottom)

        regularization = (np.sum((np.subtract(params[1:3], initial_guess[1:3])**2)) + 
                          np.sum((np.subtract(params[4:12], initial_guess[4:12])**2)))

        # Visualize every 20 iterations
        if iter % 20 == 0:
            Visualization.plot_dual_3d_clouds(pc_ref, ref_no_bottom)
            print(f"reconstruction loss: {reconstrction_loss}, reg: {regularization}")
            iter = 0
        iter += 1

        loss = reconstrction_loss + regularization / 32
        return loss
    
    optimized_params = minimize(loss_all_params, initial_guess, method='Powell').x
    print("Calibration parameters:", optimized_params)
    optimized_params = np.append(optimized_params[:12], (angular_speed_top, angular_speed_bottom))
    
    # apply optimal params
    lidar_top.set_params(np.append(optimized_params[0:3], optimized_params[6:9]))
    lidar_bottom.set_params(np.append(optimized_params[3:6], optimized_params[9:12]))
    lidar_top.angular_speed = optimized_params[12]
    lidar_bottom.angular_speed = optimized_params[13]
    return optimized_params, pc_top, pc_bottom

# automatically find the ports of the lidars and set their limits
def auto_get_lidars(top_dist_lim, top_ang_lim, bot_dist_lim, bot_ang_lim):
    ports = serial.tools.list_ports.comports()

    lidar_ports = []
    for port, desc, hwid in sorted(ports):
        if "Silicon Labs CP210x" in desc:
            lidar_ports.append(port)

    if len(lidar_ports) < 2:
        raise ValueError("Less than two lidar ports found.")

    lidar1 = Lidar(lidar_ports[0], dist_lim=(0, 50), angle_lim=(-90, -60))
    lidar2 = Lidar(lidar_ports[1], dist_lim=(0, 50), angle_lim=(-90, -60))

    start_stop_scan([lidar1, lidar2], 2)

    pc1 = lidar1.get3DPointCloud(angular_speed=0)
    pc2 = lidar2.get3DPointCloud(angular_speed=0)

    mean1 = np.nanmean(pc1[:, 2]) if np.any(np.isfinite(pc1[:, 2])) else float('inf')
    mean2 = np.nanmean(pc2[:, 2]) if np.any(np.isfinite(pc2[:, 2])) else float('inf')

    lidar_top, lidar_bottom = (lidar1, lidar2) if mean1 <= mean2 else (lidar2, lidar1)

    lidar_top.set_lims(top_dist_lim, top_ang_lim)
    lidar_bottom.set_lims(bot_dist_lim, bot_ang_lim)

    return (lidar_top, lidar_bottom)

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
    yaw = angular_speed * t  # yaw
    points = apply_yaw_to_points(yaws=yaw, points=points_unrotated)
    points = np.reshape(points, scan.shape)
    return points

# perform dft to estimate angular speed of turntable
def estimate_angular_speed(scan, freq_range=(25, 45), show_plot=False):
    # Remove outliers
    pc = lidar2d_to_3d(scan)
    pc, ind = PointCloud.remove_statistical_outliers(pc, 20, 2)
    scan = scan[ind]

    if scan.size == 0:
        return 0

    dist = scan[:, 0]
    angle = scan[:, 1]
    time = scan[:, 2]

    # Convert polar coordinates to Cartesian coordinates
    x, z = PointCloud.pol2cart(dist, angle)
    points = np.column_stack((x, z))

    # Identify unique time points
    unique_times = np.unique(time)

    # Number of bins for histogram
    num_bins = 10

    # Initialize an array to hold the histogram data
    histogram_features = np.zeros((len(unique_times), num_bins * 2))

    # Process each time slice
    for i, t in enumerate(unique_times):
        # Extract distances corresponding to the current time slice
        indices = np.where(time == t)
        time_slice_x = x[indices]
        time_slice_z = z[indices]

        # print(time_slice_x)

        # Calculate the histogram for the time slice
        hist_x, _ = np.histogram(time_slice_x, bins=num_bins, range=(np.min(x), np.max(x)))
        hist_z, _ = np.histogram(time_slice_z, bins=num_bins, range=(np.min(z), np.max(z)))

        # Normalize the histogram
        hist_normalized_x = hist_x / np.sum(hist_x)
        hist_normalized_z = hist_z / np.sum(hist_z)

        # print(hist_normalized_x)

        # Store the normalized histogram as the feature for this time slice
        histogram_features[i, :] = np.append(hist_normalized_x, hist_normalized_z)


    pca = PCA(n_components=1)
    features = pca.fit_transform(histogram_features)
    features = features.flatten()

    # Flatten the histogram features to create a single 1D array
    # features = histogram_features.flatten()

    # Remove the 'DC' offset
    features = features - np.mean(features)

    # Pad the features to improve frequency resolution
    features_padded = np.pad(features, (0, 100000 - len(features)), mode='constant')

    # Perform the Discrete Fourier Transform (DFT)
    dft_result = np.fft.fft(features_padded)

    # Calculate the sampling rate based on the time intervals
    sampling_rate = 1 / np.mean(np.diff(unique_times))

    # Calculate the frequencies corresponding to the DFT results
    freqs = np.fft.fftfreq(len(features_padded), 1 / sampling_rate) * 360
    
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
            combined_weight = np.sum(dft[indices]**2)

            if combined_weight > max_combined_weight:
                max_combined_weight = combined_weight
                dominant_freq = freq

        return dominant_freq
    domanant_freq = find_dominant_frequency(dft_result, freqs, 25, 40)
    print(f"Estimated angular speed: {domanant_freq} deg/sec")
    return domanant_freq

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

        # misc params
        self.TURNTABLE_RADIUS = 8 # cm
        self.curr_scan = np.empty((0, 3))
        self.buffer =  Queue()
        self.user_request_data = False
        self.scanning = False
        self.min_dist = dist_lim[0]
        self.max_dist = dist_lim[1]
        self.min_ang = angle_lim[0]
        self.max_ang = angle_lim[1]
        self.angular_speed = 33
        self.start_scan_time = None
        self.background_data = np.array([])
        self.pos = pos
        self.angular_pos = angular_pos
        self.lidar = RPLidar(port)
        
        self.kill_thread = False

    def _scan_thread(self):
        """Background thread that samples from the lidar.
        If self.scanning is true, then self.curr_scan contains raw lidar data.
        This thread can be killed by setting self.kill_thread to true.
        """

        # start scan
        self.lidar.connect()
        self.scan_generator = self.lidar.iter_scans(scan_type='express', max_buf_meas=3000)

        for scan in self.scan_generator:
            # add data to plotting queue
            scan = np.array(scan).astype('float')
            
            # make sure to not thread swtich while processing data 

            if len(scan.shape) > 2:
                scan = np.reshape(scan, (scan.shape[0] * scan.shape[1], scan.shape[2]))

            scan = scan[:, [2, 1]] # get data as [dist, angle]
            scan[:, 0] = scan[:, 0]/10 # convert from mm to cm

            scan[:, 1] = scan[:, 1] + 180  # convert angles so that 0 degrees is the z axis 
            scan[:, 1][(scan[:, 1] > 180)] -= 360 
            
            # filter
            scan = scan[(scan[:, 0] > self.min_dist) & (scan[:,0] < self.max_dist)]
            scan = scan[(scan[:, 1] > self.min_ang) & (scan[:, 1] < self.max_ang)]

            # update cumulative scanning data
            scan_time = time.time()
            data_point_time = scan_time - self.start_scan_time 
            scan_with_time = np.column_stack((scan, np.full(scan[:, 0].shape, data_point_time))) # add third coordinate: time
            self.curr_scan = np.vstack((self.curr_scan, scan_with_time))

            if self.user_request_data:
                x, y = PointCloud.pol2cart(scan[:, 0], scan[:, 1])
                self.buffer['2d'].put(np.column_stack((x, y)))
                # convert to 3D points and put in buffer
                new_points = self.get3DPointCloud(scan=scan_with_time, angular_speed=self.angular_speed)
                self.buffer['3d'].put(new_points)

            if self.kill_thread:
                self.lidar.stop()
                self.lidar.disconnect()
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
        self.resumeScan()

    def stopScan(self):
        """Stop the scan.
        """
        self.kill_thread = True
        self.scan_thread.join()
        self.scanning = False

    def resumeScan(self):
        """Resume the current scan, keeping previous scan data.
        """
        if self.scanning:
            raise("Already Scanning")

        self.kill_thread = False
        self.scanning = True
        self.scan_thread = threading.Thread(target=self._scan_thread, daemon=True)
        self.scan_thread.start()

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

    def set_current_to_background(self):
        """Set background using data from startScan-stopScan.
        """
        # set background points to filter out later
        self.background_data = self.curr_scan

    def remove_background_on_current(self, use_calib_params=True):
        """Remove background points in-place from data from startScan-stopScan.
        """
        dist = self.curr_scan[:, 0]
        angle = self.curr_scan[:, 1]
        x, z = PointCloud.pol2cart(dist, angle)
        mask = np.ones(x.size).astype('bool')
        
        if self.background_data.size > 0:
            background_points = np.column_stack(PointCloud.pol2cart(self.background_data[:, 0], self.background_data[:, 1]))

            # remove background points
            mask = PointCloud.subtract_point_clouds(np.column_stack((x, z)), background_points, 1)

        if use_calib_params:
            # remove points that are outside of the turntable
            mask = mask & ((x > (self.pos[0] - self.TURNTABLE_RADIUS)) & (x < (self.pos[0] + self.TURNTABLE_RADIUS)))
        self.curr_scan = self.curr_scan[mask]

    def get_buffer(self):
        """Get buffer of the 2d lidar data that is updated in realtime

        Parameters
        ----------
        thread_function : Callable
            Function to run while plotting.
        """
        self.buffer = {'2d' : Queue(), '3d': Queue(), '3d_1' : Queue()}
        self.user_request_data = True
        return self.buffer

    def get3DPointCloud(self, scan: np.ndarray=None, angular_speed=None, pos=None, angular_pos=None) -> np.ndarray:
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
            if scan.size > 4:
                angular_speed = estimate_angular_speed(scan)
                self.angular_speed = angular_speed
            else:
                angular_speed = self.angular_speed

        if pos == None:
            pos = self.pos

        if angular_pos == None:
            angular_pos = self.angular_pos

        result = lidar2d_to_3d(scan, angular_speed, pos=pos, 
                                        angular_pos=angular_pos)

        return result
    
    def disconnect(self):
        self.lidar.stop()
        self.lidar.disconnect()
