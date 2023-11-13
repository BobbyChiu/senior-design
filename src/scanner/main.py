import PointCloud
from Lidar import Lidar, estimate_angular_speed, lidar2d_to_3d, calibrate_lidars
from Visualization import plot3d, showMesh, plot2d_realtime
import time
import numpy as np
import open3d as o3d
import threading

def post_processing(pc):
    num_points = pc.shape[0]
    # filtering to remove outliers
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc)

    # statistical outlier removal
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=100, std_ratio=2)
    pcd = pcd.select_by_index(ind)
    # # radius outlier removal
    cl, ind = pcd.remove_radius_outlier(nb_points=5, radius=0.25)
    pcd = pcd.select_by_index(ind)
    pc = np.asarray(pcd.points)  

    # make sure top and bottom are closed
    num_points = pc.shape[0]
    vertical_range = pc[:, 2].max() - pc[:, 2].min()
    pc = PointCloud.fillFace(pc, 
                             density=20, num_points_from_edge=int(num_points/100), dist_from_edge=vertical_range/30, 
                             bottom=True, top=True)

    # smoothening
    # estimated_normals = PointCloud.estimate_normals(pc, radius=0.5)
    # pc = PointCloud.bilateral_filter_point_cloud(pc, estimated_normals, 
    #                                              radius=0.5, sigma_s=0.1, sigma_n=0.1)
    return pc

def do_scan(lidars, scan_time):
    start_time = time.time()
    for l in lidars:
        l.startScan(start_time)
    time.sleep(scan_time)
    for l in lidars:
        l.stopScan()

LIDAR_VERTICAL_SEPARATION = 15.722
def self_calibrate(lidar_top, lidar_bottom):
    initial_guess = [lidar_top.dist_from_axis, 0, LIDAR_VERTICAL_SEPARATION, # lidar top pos
                    0, 0, 0,  # lidar top angle
                    lidar_bottom.dist_from_axis, 0, 0, # lidar bottom pos
                    0, 0, 0]  # lidar bottom angle
    # ideal_rect = PointCloud.stl_to_mesh('calibration/rect.stl')
    # ideal_rect = PointCloud.mesh_to_pc(ideal_rect, 5000)
    optimized_params, calibration_cloud = calibrate_lidars(lidar_top.curr_scan, 
                                        lidar_bottom.curr_scan, 
                                        initial_guess,
                                        ground_truth=None)
    
    # apply optimal params
    top_params = optimized_params[0:6]
    lidar_top.set_params(top_params)
    bottom_params = optimized_params[6:12]
    lidar_bottom.set_params(bottom_params)
    return optimized_params, calibration_cloud

def scan(lidar_top, lidar_bottom):
    lidars = [lidar_top, lidar_bottom]
    calibration_lims = [(-45, 45), (0, 45)]
    calibration_time = 30
    background_collection_time = 10
    background_lims = [(-45, 45), (-15, 45)]
    scan_time = 60

    calibrate = input("Calibrate (y/n)? ") == "y"
    if calibrate:
        # get background
        print(f"START BACKGROUND COLLECTION, DURATION: {background_collection_time} s")
        for l, lim in zip(lidars, background_lims):
            l.set_lims(lim)
        do_scan(lidars, background_collection_time)
        dist_from_axis, turntable_height = lidar_bottom.set_current_to_background(estimate_params=True)
        lidar_top.set_current_to_background(estimate_params=False)
        lidar_top.set_background_params(dist_from_axis, turntable_height - LIDAR_VERTICAL_SEPARATION)
        input("Done getting background: press enter to start calibrating")

        # calibrate
        print(f"START CALIBRATION, DURATION: {calibration_time} s")
        for l, lim in zip(lidars, calibration_lims):
            l.set_lims(lim)
        do_scan(lidars, calibration_time)

        # remove background
        for l in lidars:
            l.remove_background_on_current(use_calib_params=False)
        # no need to bias the turntable height once we have calib params
        lidar_top.turntable_height += LIDAR_VERTICAL_SEPARATION

        PointCloud.to_file(lidar_bottom.curr_scan, folder="lidar-data-2d", filename="rect-test-bottom.xyz")
        PointCloud.to_file(lidar_top.curr_scan, folder="lidar-data-2d", filename="rect-test-top.xyz")

        initial_guess = [dist_from_axis, 0, LIDAR_VERTICAL_SEPARATION, # lidar top pos
                         0, 0, 0,  # lidar top angle
                         dist_from_axis, 0, 0, # lidar bottom pos
                         0, 0, 0]  # lidar bottom angle
        # ideal_rect = PointCloud.stl_to_mesh('calibration/rect.stl')
        # ideal_rect = PointCloud.mesh_to_pc(ideal_rect, 5000)
        optimized_params, calibration_cloud = calibrate_lidars(lidar_top.curr_scan, 
                                            lidar_bottom.curr_scan, 
                                            initial_guess,
                                            ground_truth=None)
        
        plot3d(calibration_cloud)

        # save calibration data
        PointCloud.to_file(optimized_params, folder="calibration", filename="calibration.data")
        PointCloud.to_file(lidar_top.background_data, folder="calibration", filename="top_background.xyz")
        PointCloud.to_file(lidar_bottom.background_data, folder="calibration", filename="bottom_background.xyz")
    else:
        # read in calibration data
        optimized_params = PointCloud.from_file("calibration/calibration.data")
        top_background = PointCloud.from_file("calibration/top_background.xyz")

        lidar_top.curr_scan = top_background
        bottom_background = PointCloud.from_file("calibration/bottom_background.xyz")
        lidar_bottom.curr_scan = bottom_background

        # set background params
        dist_from_axis, turntable_height = lidar_bottom.set_current_to_background(estimate_params=True)
        lidar_top.set_current_to_background(estimate_params=False)
        lidar_top.set_background_params(dist_from_axis, turntable_height)
    
    # apply optimal params
    top_params = optimized_params[0:6]
    lidar_top.set_params(top_params)
    lidar_top.turntable_height = lidar_bottom.turntable_height
    bottom_params = optimized_params[6:12]
    lidar_bottom.set_params(bottom_params)

    # Scanning
    input("Place object on turntable and press enter to start scanning.")
    print(f"START SCAN, DURATION: {scan_time} s")
    do_scan(lidars, scan_time)

    # Processing
    for l in lidars:
        l.remove_background_on_current(use_calib_params=True)
    # save 2d lidar data
    PointCloud.to_file(lidar_bottom.curr_scan, folder="lidar-data-2d")
    PointCloud.to_file(lidar_top.curr_scan, folder="lidar-data-2d")
    # estimated_angular_speed = estimate_angular_speed(lidar_bottom.curr_scan[:, 0], lidar_bottom.curr_scan[:, 2])
    self_calibrate(lidar_top, lidar_bottom) # make sure scans align
    pc_bottom = lidar_bottom.get3DPointCloud()
    pc_top = lidar_top.get3DPointCloud()

    pc_final = np.vstack((pc_bottom, pc_top))
    # save pc_final before processing
    PointCloud.to_file(pc_final, folder="lidar-data-xyz")

    # show point clouds
    plot3d(pc_top) 
    plot3d(pc_bottom)
    plot3d(pc_final)

    # Filtering and smoothening
    pc_final = post_processing(pc_final)
    plot3d(pc_final)

    # Generate Mesh
    mesh = PointCloud.to_mesh(pc_final)
    showMesh(mesh) # show mesh
    
    # save as stl file
    PointCloud.mesh_to_stl(mesh, 'stl')

if __name__ == '__main__':
    with Lidar('com5', dist_lim=(0,50), angle_lim=(-15,45)) as lidar_bottom:
        with Lidar('com3', dist_lim=(0,50), angle_lim=(-45,45)) as lidar_top:
            # prepare plotting of lidar data
            lidar_bottom_buffer = lidar_bottom.get_buffer()
            lidar_top_buffer = lidar_top.get_buffer()
            # start scan in separate thread
            t = threading.Thread(target=scan, args=(lidar_top, lidar_bottom), daemon=True)
            t.start()
            # plot lidar data
            plot2d_realtime(buffers=(lidar_top_buffer, lidar_bottom_buffer), 
                            colors=['red', 'blue'])
            
################# TESTING ##############################
# scan_top = PointCloud.from_file("lidar-data-2d/rect-test-top.xyz")
# scan_bottom = PointCloud.from_file("lidar-data-2d/rect-test-bottom.xyz")
# ideal_rect = PointCloud.stl_to_mesh('calibration/rect.stl')
# ideal_rect = PointCloud.mesh_to_pc(ideal_rect, 2000)
# initial_guess = [38.157, 0, 15.722, # lidar top pos
#                 0, 0, 0,  # lidar top angle
#                 38.157, 0, 0, # lidar bottom pos
#                 0, 0, 0]  # lidar bottom angle
# params, pc = calibrate_lidars(scan_top, scan_bottom, initial_guess)
# plot3d(pc)

# pc = PointCloud.from_file("lidar-data-xyz/sandia_w_cal.xyz")
# plot3d(pc)
# pc = post_processing(pc)
# plot3d(pc)
# mesh = PointCloud.to_mesh(pc)
# showMesh(mesh)


# [ 4.13459601e+01  9.52212398e-01 -2.52597540e-06  7.19210162e-01
#  -1.38803407e+00  1.96679853e-03  3.25303470e+01]

# scan = PointCloud.from_file("lidar-data-2d/test_cube.xyz")
# scan = scan[(scan[:, 1 ] > 2)]
# # # # # 4.07669793e+01  8.55490147e-01  4.71374552e-06  6.53887391e-01
# # # # #  -7.94570564e+00  3.65472907e-02
# # # # # pos=(4.13459601e+01, 9.52212398e-01, -2.52597540e-06), angular_pos=(7.19210162e-01, -1.38803407e+00, 1.96679853e-03)
# angular_speed = estimate_angular_speed(scan[:, 0],  scan[:, 2], show_plot=True)
# pc = lidar2d_to_3d(scan, angular_speed, pos=(41.5, 0 ,0 ), angular_pos=(0, 0, 0))
# plot3d(pc) # show point cloud
# # pc = post_processing(pc)
# # plot3d(pc) # show point cloud
# # mesh = PointCloud.to_mesh(pc) 
# # showMesh(mesh) # show mesh
# # # # save as stl file
# # PointCloud.mesh_to_stl(mesh, 'stl')

