import PointCloud
from Lidar import Lidar, estimate_angular_speed
from Visualization import plot3d, showMesh
import time
import numpy as np
import open3d as o3d

def post_processing(pc):
    plot3d(pc)
    # filtering to remove outliers

    # open3d statistical outlier remover
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc)

    # g statistical outlier removal
    # cl, ind = pcd.remove_statistical_outlier(nb_neighbors=100, std_ratio=0.5)
    # denoised_pcd = pcd.select_by_index(ind)
    # radius outlier removal
    cl, ind = pcd.remove_radius_outlier(nb_points=8, radius=0.25)
    denoised_pcd = pcd.select_by_index(ind)

    pc = np.asarray(denoised_pcd.points)   

    # smoothening
    pc = PointCloud.gaussian_filter(pc, 25, 4)
    # make sure top and bottom are closed
    pc = PointCloud.fillFace(pc, bottom=True, top=True)
    # generate mesh
    return pc

if __name__ == '__main__':
    LIDAR_VERTICAL_SEPARATION = 15.722
    # with Lidar('com3', dist_lim=(0,50), angle_lim=(-45,45)) as lidar_bottom:
    #     with Lidar('com4', dist_lim=(0,50), angle_lim=(-45,45)) as lidar_top:
    #         def new_main():

    #             calibrate = input("Calibrate (y/n)? ") == "y"
    #             if calibrate:
    #                 # Calibrate
    #                 lidar_bottom.set_lims(angle_lim=(0, 45))
    #                 lidar_top.set_lims(angle_lim=(-15, 30))
    #                 calibration_time = 10
    #                 print(f"START CALIBRATION, DURATION: {calibration_time} s")
    #                 lidar_bottom.startScan()
    #                 lidar_top.startScan()
    #                 time.sleep(calibration_time)
    #                 lidar_top.stopScan()
    #                 lidar_bottom.stopScan()

    #                 # apply angular, horizontal, and vertical biases
    #                 horizontal_dist_top = lidar_top.calibrate_on_current()
    #                 horizontal_dist_bottom = lidar_bottom.calibrate_on_current()
    #                 horizontal_top_bias = horizontal_dist_bottom - horizontal_dist_top 

    #                 PointCloud.to_file((horizontal_top_bias, lidar_top.angular_bias, lidar_bottom.angular_bias),
    #                                    folder="calibration", filename="calibration.data")

    #                 lidar_top.set_bias(horizontal=horizontal_top_bias, vertical=LIDAR_VERTICAL_SEPARATION)    
    #                 input("Done calibration: press enter to continue")
                
    #             else:
    #                 # read in calibration data
    #                 calibration_data = PointCloud.from_file("calibration/calibration.data")
    #                 lidar_top.set_bias(horizontal=calibration_data[0], vertical=LIDAR_VERTICAL_SEPARATION, angular=calibration_data[1])
    #                 lidar_bottom.set_bias(angular=calibration_data[2])

    #             # get background
    #             background_collection_time = 10
    #             print(f"START BACKGROUND COLLECTION, DURATION: {background_collection_time} s")
    #             lidar_bottom.set_lims(angle_lim=(-15, 45))
    #             lidar_top.set_lims(angle_lim=(-45, 0))
    #             lidar_bottom.startScan()
    #             lidar_top.startScan()
    #             time.sleep(background_collection_time)
    #             lidar_top.stopScan()
    #             lidar_bottom.stopScan()
    #             dist_from_axis, turntable_height = lidar_bottom.set_current_to_background(estimate_params=True)
    #             lidar_top.set_current_to_background(estimate_params=False)
    #             lidar_top.set_background_params(dist_from_axis, turntable_height)
    #             input("Done getting background: press enter to start scanning")

    #             # Scanning
    #             scan_time = 120
    #             print(f"START SCAN, DURATION: {scan_time} s")
    #             start_time = time.time()
    #             lidar_bottom.startScan(start_time)
    #             lidar_top.startScan(start_time)
    #             time.sleep(scan_time)
    #             lidar_bottom.stopScan()
    #             lidar_top.stopScan()

    #             # Processing
    #             lidar_top.remove_background_on_current()
    #             lidar_bottom.remove_background_on_current()
    #             angular_speed = estimate_angular_speed(lidar_bottom.curr_scan[:, 0], lidar_bottom.curr_scan[:, 2])
    #             pc_top = lidar_top.get3DPointCloud(angular_speed=angular_speed)
    #             pc_bottom = lidar_bottom.get3DPointCloud(angular_speed=angular_speed)
    #             pc_final = np.vstack((pc_bottom, pc_top))
    #             # save pc_final before processing
    #             PointCloud.to_file(pc_final, folder="lidar-data-xyz")

    #             plot3d(pc_top) # show point clouds
    #             plot3d(pc_bottom)
    #             pc_final = PointCloud.combine_point_clouds(pc_bottom, pc_top)
    #             plot3d(pc_final)

    #             # Filtering and smoothening
    #             pc_final = post_processing(pc_final)
    #             plot3d(pc_final)

    #             # Generate Mesh
    #             mesh = PointCloud.to_mesh(pc_final)
    #             showMesh(mesh) # show mesh
               
    #             # save as stl file
    #             PointCloud.mesh_to_stl(mesh, 'stl')

            # lidar_bottom.showPlot(new_main)
            # estimate_angular_speed(lidar.curr_scan[:,0], lidar.curr_scan[:, 2], show_plot=True)

pc = PointCloud.from_file("lidar-data-xyz/new_sandia.xyz")
plot3d(pc) # show point cloud
pc = post_processing(pc)
plot3d(pc) # show point cloud
mesh = PointCloud.to_mesh(pc) 
showMesh(mesh) # show mesh
# save as stl file
# PointCloud.mesh_to_stl(mesh, 'stl')

