import numpy as np
from queue import Queue
from rplidar_new import RPLidar, MAX_MOTOR_PWM
import time
import threading
import Visualization
import PointCloud
from matplotlib import pyplot as plt
from scipy.optimize import minimize
from scipy.interpolate import interp1d
from serial.serialutil import SerialTimeoutException
from sklearn.decomposition import PCA
import open3d as o3d
import serial.tools.list_ports

def post_processing(pc, knn=None, rad_out_rem_params=(10,5), stat_out_rem_params=(20, 2)):

    # filtering to remove outliers
    if knn:
        for k, threshold in knn.items():
            mask = PointCloud.knn_filter(pc, k, threshold)
            pc = pc[mask[:, 0]]

    # smoothening
    # pc = PointCloud.gaussian_filter_radius(pc, 1, 1)
    # estimated_normals = PointCloud.estimate_normals(pc, radius=10)
    # pc = PointCloud.bilateral_filter_point_cloud(pc, 
    #                                             estimated_normals, 
    #                                             radius=10, 
    #                                             sigma_s=1, 
    #                                             sigma_n=0.1) 


    pc = PointCloud.get_surface_points(pc, voxel_size=0.05)
                                                
    # open3d outlier remover
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc)
    # statistical outlier removal
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=stat_out_rem_params[0], 
                                             std_ratio=stat_out_rem_params[1])
    pcd = pcd.select_by_index(ind)
    # # radius outlier removal
    # cl, ind = pcd.remove_radius_outlier(nb_points=rad_out_rem_params[0],
    #                                     radius=rad_out_rem_params[1])
    # pcd = pcd.select_by_index(ind)
    pc = np.asarray(pcd.points)

    # add flat bottom surface
    pc = PointCloud.add_bottom_surface(pc, 
                                       z_percentile=1, 
                                       percent_height=1, 
                                       grid_spacing=0.1, 
                                       crop=True)
    
    # # optional: add flat top surface
    # pc[:, 2] = - pc[:, 2]
    # pc = PointCloud.add_bottom_surface(pc, 
    #                                    z_percentile=1,
    #                                    percent_height=1,
    #                                    grid_spacing=0.1,
    #                                    crop=True)
    
    # pc[:, 2] = - pc[:, 2]
    return pc

def start_stop_scan(lidars, scan_time):
    start_time = time.time()
    for l in lidars:
        l.startScan(start_time)
    time.sleep(scan_time)
    for l in lidars:
        l.stopScan()

def calibrate(lidar_top, lidar_bottom, initial_guess, ref_obj):
    optimized_params, pc_top_cal, pc_bottom_cal = calibrate_lidars(
                                        lidar_top.curr_scan, 
                                        lidar_bottom.curr_scan, 
                                        initial_guess=initial_guess,
                                        ref=ref_obj)
    
    # apply optimal params
    top_params = optimized_params[0:6]
    lidar_top.set_params(top_params)
    bottom_params = optimized_params[6:12]
    lidar_bottom.set_params(bottom_params)
    return optimized_params, pc_top_cal, pc_bottom_cal

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
    dist = scan[:, 0]
    angle = scan[:, 1]
    time = scan[:, 2]

    # Convert polar coordinates to Cartesian coordinates
    x, z = PointCloud.pol2cart(dist, angle)
    points = np.column_stack((x, z))

    # Identify unique time points
    unique_times = np.unique(time)

    # Number of bins for histogram
    num_bins = 20

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

pc_combined = None
pc_top = None
pc_bottom = None
def calibrate_lidars(top_scan, bottom_scan, initial_guess, ref):
    angular_speed_top = estimate_angular_speed(top_scan)
    angular_speed_bottom = estimate_angular_speed(bottom_scan)
    initial_guess = np.append(initial_guess, [0, 0, 0, 0, 0 ,0])

    def loss_function(params):
        global pc_combined
        global pc_bottom
        global pc_top
        pos_top = params[0:3]
        angle_top = params[3:6]
        pos_bottom = params[6:9]
        angle_bottom = params[9:12]
        transformation = params[12:18]

        # current scan estimate
        pc_top = lidar2d_to_3d(top_scan, angular_speed_top, pos=pos_top, angular_pos=angle_top)
        pc_bottom = lidar2d_to_3d(bottom_scan, angular_speed_bottom, pos=pos_bottom, angular_pos=angle_bottom)

        # make sure point clouds are aligned    
        pc_combined = np.vstack((pc_top, pc_bottom))
        # pc_combined = post_processing(pc_combined)

        # find error due to difference between reference scans and reference object 
        pc_ref = PointCloud.apply_transformation(transformation, pc_combined)
        reconstrction_loss = PointCloud.chamfer_distance(pc_ref, ref)
        
        # find error due to disalignment between the two current top and bottom scans
        disalignment_loss = PointCloud.chamfer_distance(pc_top, pc_bottom)

        regularization = np.sum((np.subtract(params[0:12], initial_guess[0:12]))**2)
        loss = reconstrction_loss + disalignment_loss + regularization / 2

        print(f"reg loss: {reconstrction_loss}, d loss: {disalignment_loss}")
        print(loss)
        return loss
    

    optimized_params = minimize(loss_function, initial_guess, method='SLSQP').x
    print("Calibration parameters:", optimized_params)
    return optimized_params, pc_top, pc_bottom

# calibrate without reference, returns scanning parameters that result in point clouds that are aligned
def calibrate_no_ref(top_scan, bottom_scan, initial_guess):
    initial_guess = np.array(initial_guess)

    def loss_function(params):
        global pc_bottom
        global pc_top
        pos_top = params[0:3]
        angle_top = params[3:6]
        pos_bottom = params[6:9]
        angle_bottom = params[9:12]

        # current scan estimate
        pc_top = lidar2d_to_3d(top_scan, params[12], pos=pos_top, angular_pos=angle_top)
        pc_bottom = lidar2d_to_3d(bottom_scan, params[13], pos=pos_bottom, angular_pos=angle_bottom)

        # find error due to disalignment between the two current top and bottom scans
        disalignment_loss = PointCloud.chamfer_distance(pc_top, pc_bottom)

        mask = np.ones(params.size, dtype=bool)
        regularization = np.sum((np.subtract(params[mask], initial_guess[mask]))**2)
        loss = disalignment_loss + regularization / 4

        print(f"reg loss: {regularization}, d loss: {disalignment_loss}")
        print(loss)
        return loss
    
    optimized_params = minimize(loss_function, initial_guess, method='SLSQP').x
    print("Calibration parameters:", optimized_params)
    return optimized_params, pc_top, pc_bottom

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
        self.angular_speed = 0
        self.dist_from_axis = 0
        self.turntable_height = 0
        self.start_scan_time = None
        self.background_data = np.array([])
        self.pos = pos
        self.angular_pos = angular_pos
        self.lidar = RPLidar(port)

        # connect to lidar
        info = self.lidar.get_info()
        print(info)
        health = self.lidar.get_health()
        print(health)
        
        self.kill_thread = False

    def _scan_thread(self):
        """Background thread that samples from the lidar.
        If self.scanning is true, then self.curr_scan contains raw lidar data.
        This thread can be killed by setting self.kill_thread to true.
        """
        # start scan
        self.scan_generator = self.lidar.iter_scans(scan_type='express', max_buf_meas=3000)

        for scan in self.scan_generator:
            # add data to plotting queue
            scan_time = time.time()
            scan = np.array(scan).astype('float')

            if len(scan.shape) > 2:
                scan = np.reshape(scan, (scan.shape[0] * scan.shape[1], scan.shape[2]))

            scan = scan[:, [2, 1]] # get data as [dist, angle]
            scan[:, 0] = scan[:, 0]/10 # convert from mm to cm

            scan[:, 1] = scan[:, 1] + 180  # convert angles so that 0 degrees is the z axis 
            scan[:, 1][(scan[:, 1] > 180)] -= 360 
            
            # filter
            scan = scan[(scan[:, 0] > self.min_dist) & (scan[:,0] < self.max_dist)]
            scan = scan[(scan[:, 1] > self.min_ang) & (scan[:, 1] < self.max_ang)]

            if self.user_request_data:
                x, y = PointCloud.pol2cart(scan[:, 0], scan[:, 1])
                self.buffer.put(np.column_stack((x, y)))

            # update cumulative scanning data
            if self.scanning:
                data_point_time = scan_time - self.start_scan_time 
                scan_with_time = np.column_stack((scan, np.full(scan[:, 0].shape, data_point_time))) # add third coordinate: time
                self.curr_scan = np.vstack((self.curr_scan, scan_with_time))

            if self.kill_thread:
                self.lidar.stop()
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

        self.kill_thread = False
        self.curr_scan = np.empty((0, 3))
        self.scanning = True
        self.scan_thread = threading.Thread(target=self._scan_thread, daemon=True)
        self.scan_thread.start()
        

    def stopScan(self):
        """Stop the scan.
        """
        self.kill_thread = True
        self.scan_thread.join()
        self.scanning = False

    # estimates and sets the angular bias assuming the current scan is a vertical line
    # returns the horizontal distance to the vertical line after rotating it
    def calibrate_on_current(self, ground_truth, initial_guess):
        angular_speed = estimate_angular_speed(self.curr_scan)

        initial_guess = np.append(initial_guess, [0, 0, 0, 0, 0 ,0])

        pc_init = lidar2d_to_3d(self.curr_scan, angular_speed, pos=initial_guess[0:3], angular_pos=initial_guess[3:6])

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pc_init)

        # # # g statistical outlier removal
        # cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2)
        # pcd.points = o3d.utility.Vector3dVector(self.curr_scan)
        # pcd = pcd.select_by_index(ind)
        # self.curr_scan = np.asarray(pcd.points)

        # ground_truth = ground_truth - ground_truth.mean()

        def loss_function(params):
            pos = params[:3]
            angular_pos = params[3:6]
            tranformation = params[6:12]

            pc = lidar2d_to_3d(self.curr_scan, angular_speed, pos=pos, angular_pos=angular_pos)

            # make sure point clouds are aligned
            # pc = PointCloud.icp(pc, ground_truth)
            pc = PointCloud.apply_transformation(tranformation, pc)

            # Visualization.plot3d(pc)
            # Visualization.plot3d(pc)
            # time.sleep(10)

            chamfer_loss = PointCloud.chamfer_distance(pc, ground_truth, only_a_to_b=True)
            regularization = np.sum((np.subtract(params[0:6], initial_guess[0:6]))**2)
            loss = chamfer_loss + regularization
            print(f" reconstruction: {chamfer_loss} reg: {regularization}")
            return loss

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
        if self.curr_scan.size < 4:
            return

        turntable = self.curr_scan[(self.curr_scan[:, 1] > -10) & (self.curr_scan[:, 1] < 0)]
        dist = turntable[:, 0]
        angle = turntable[:, 1]
        x, z =  PointCloud.pol2cart(dist, angle)

        # get background points to filter out later
        self.background_data = self.curr_scan

        if estimate_params:
            # estimate distance from axis of rotation
            mask = PointCloud.knn_filter(np.column_stack((x, np.full(x.shape, 0))), 5, 0.1) # get points that form veritcal line
            turntable_height = z[mask[:, 0]].max() + 0.6
            dist_from_axis = np.median(x[mask[:, 0]]) + self.TURNTABLE_RADIUS
            self.set_background_params(dist_from_axis, turntable_height)
            print(f"Estimated dist from axis: {self.dist_from_axis}")
            print(f"Turntable Height: {self.turntable_height}")
            return self.dist_from_axis, self.turntable_height
    
    def set_background_params(self, dist_from_axis, turntable_height):
        self.dist_from_axis = dist_from_axis
        self.turntable_height = turntable_height

    def remove_background_on_current(self, use_calib_params=True):
        """Remove background points in-place from data from startScan-stopScan.
        """
        dist = self.curr_scan[:, 0]
        angle = self.curr_scan[:, 1]
        x, z = PointCloud.pol2cart(dist, angle)
        mask = np.ones(x.size).astype('bool')
        
        if self.background_data.size >= 4:
            background_points = np.column_stack(PointCloud.pol2cart(self.background_data[:, 0], self.background_data[:, 1]))

            # remove background points
            mask = PointCloud.subtract_point_clouds(np.column_stack((x, z)), background_points, 0.1)

        mask = mask & ((z > self.turntable_height) & (x > (self.dist_from_axis - self.TURNTABLE_RADIUS)) & (x < (self.dist_from_axis + self.TURNTABLE_RADIUS)))
        self.curr_scan = self.curr_scan[mask]

        # g statistical outlier removal
        pc= self.get3DPointCloud(pos=(self.dist_from_axis, 0, 0), angular_pos=(0,0,0))
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pc)
        cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2)
        pcd.points = o3d.utility.Vector3dVector(self.curr_scan)
        pcd = pcd.select_by_index(ind)
        self.curr_scan = np.asarray(pcd.points)
            
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
            self.angular_speed = estimate_angular_speed(scan)
        else:
            self.angular_speed = angular_speed

        if pos == None:
            pos = self.pos

        if angular_pos == None:
            angular_pos = self.angular_pos

        result = lidar2d_to_3d(scan, self.angular_speed, pos=self.pos, 
                                        angular_pos=self.angular_pos)

        return result
