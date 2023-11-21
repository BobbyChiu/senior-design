import PointCloud
from Lidar import Lidar, estimate_angular_speed, lidar2d_to_3d, calibrate_lidars, start_stop_scan, calibrate
from Visualization import plot3d, showMesh, plot2d_realtime, plot_dual_3d_clouds
import ArgSource
import time
import argparse
import pathlib
import numpy as np
import open3d as o3d
from traceback import print_exc
import threading
from main import do_processing_on_point_cloud

LIDAR_VERTICAL_SEPARATION = 15.722

def post_processing(pc, knn=None):
    # filtering to remove outliers
    # open3d statistical outlier remover
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc)

    # # g statistical outlier removal
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=100, std_ratio=2)
    pcd = pcd.select_by_index(ind)
    # radius outlier removal
    cl, ind = pcd.remove_radius_outlier(nb_points=5, radius=0.25)
    pcd = pcd.select_by_index(ind)

    pc = np.asarray(pcd.points)

    # filtering to remove outliers
    if knn:
        for k, threshold in knn.items():
            mask = PointCloud.knn_filter(pc, k, threshold)
            pc = pc[mask[:, 0]]

    # smoothening
    pc = PointCloud.gaussian_filter_radius(pc, 0.1, 1)
    estimated_normals = PointCloud.estimate_normals(pc, radius=0.5)
    pc = PointCloud.bilateral_filter_point_cloud(pc, 
                                                estimated_normals, 
                                                radius=0.5, 
                                                sigma_s=1, 
                                                sigma_n=0.001)   
                                                    

    # make sure top and bottom are closed
    num_points = pc.shape[0]
    vertical_range = pc[:, 2].max() - pc[:, 2].min()
    pc = PointCloud.fillFace(pc, 
                            density=20, num_points_from_edge=int(num_points/200), dist_from_edge=vertical_range/30, 
                            bottom=True, top=True)

    plot3d(pc) # show point cloud
    # estimated_normals = PointCloud.estimate_normals(pc, radius=0.5)
    # pc = PointCloud.bilateral_filter_point_cloud(pc, estimated_normals, 
    #                                              radius=0.5, sigma_s=0.1, sigma_n=0.1)
    return pc

# top = PointCloud.from_file("lidar-data-2d/sandia-test.xyz")
# estimate_angular_speed(top, show_plot=True)

bottom = PointCloud.from_file("calibration/ref-duck-bottom.xyz")

top = PointCloud.from_file("calibration/ref-duck-top.xyz")

bottom_back = PointCloud.from_file("calibration/bottom_background.xyz")
top_back = PointCloud.from_file("calibration/top_background.xyz")

lidar_bottom = Lidar('COM5', dist_lim=(20,60), angle_lim=(-15,45))
lidar_top = Lidar('COM3', dist_lim=(20,60), angle_lim=(-45,0))
lidar_bottom.disconnect()
lidar_top.disconnect()

lidar_bottom.curr_scan = bottom_back
lidar_top.curr_scan = top_back

dist_from_axis, turntable_height = lidar_bottom.set_current_to_background(estimate_params=True)
lidar_top.set_current_to_background(estimate_params=False)
lidar_top.set_background_params(dist_from_axis, turntable_height - LIDAR_VERTICAL_SEPARATION)

lidar_bottom.curr_scan = bottom
lidar_top.curr_scan = top

lidar_bottom.remove_background_on_current()
lidar_top.remove_background_on_current()



#  calibrate
calibration_duck = PointCloud.stl_to_mesh('calibration/duck.stl')
calibration_duck = PointCloud.mesh_to_pc(calibration_duck, 10000) / 10

# params_bottom = lidar_bottom.calibrate_on_current(calibration_duck, [lidar_bottom.dist_from_axis, 0, 0, 0, 0, 0])
# params_top = lidar_top.calibrate_on_current(calibration_duck, [lidar_top.dist_from_axis, 0, LIDAR_VERTICAL_SEPARATION, 0, 0, 0])

initial_guess = [lidar_top.dist_from_axis, 0, 15.722, # lidar top pos
                0, 0, 0,  # lidar top angle
                lidar_bottom.dist_from_axis, 0, 0, # lidar bottom pos
                0, 0, 0]  # lidar bottom angle

optimal_params, top_calibration_cloud, bottom_calibration_cloud = calibrate(lidar_top, lidar_bottom, initial_guess, calibration_duck) # make sure scans align
print(f"Optimal Scanning Params: {optimal_params}")
plot_dual_3d_clouds(top_calibration_cloud, bottom_calibration_cloud, 'red', 'blue')


# # calibrate
# calibration_duck = PointCloud.stl_to_mesh('calibration/duck.stl')
# calibration_duck = PointCloud.mesh_to_pc(calibration_duck, 1000) / 10
# initial_guess = [34.5, 0, 15.722, # lidar top pos
#                 0, 0, 0,  # lidar top angle
#                 34.5, 0, # lidar bottom pos
#                 0, 0, 0]  # lidar bottom angle



# optimal_params, top_cal_cloud, bot_cal_cloud = calibrate_lidars(top, bottom, initial_guess, calibration_duck)
# print(f"Optimal Scanning Params: {optimal_params}")
# plot_dual_3d_clouds(top_cal_cloud, bot_cal_cloud, 'red', 'blue')
# # plot3d(np.vstack((top_cal_cloud, bot_cal_cloud)))
# # initial_guess = [37.17, 0, 15.722, # lidar top pos
# #                 0, 0, 0,  # lidar top angle
# #                 37.17, 0, # lidar bottom pos
#                 0, 0, 0]  # lidar bottom angle
    
# top = PointCloud.from_file("lidar-data-2d/latest_top.xyz")
# bottom = PointCloud.from_file("lidar-data-2d/latest_bottom.xyz")

# optimized_params, calibration_cloud = calibrate_lidars(
#                                     top, 
#                                     bottom, 
#                                     initial_guess=initial_guess,
#                                     top_ref_scan=None,
#                                     bottom_ref_scan=None,
#                                     ref=None)

# plot3d(calibration_cloud)

# pc_final = do_processing_on_point_cloud(calibration_cloud, **{
#     'remove_radius_outlier' : {
#         'nb_points' : 10,
#         'radius' : 0.25
#     },
#     'gaussian' : {
#         'k' : 0.25,
#         'sigma' : 2
# }
# })

# PointCloud.to_file(pc_final, folder="lidar-data-xyz", filename="penultimate_sandia.xyz")
# mesh = PointCloud.to_mesh(pc_final)
# showMesh(mesh)
# PointCloud.mesh_to_stl(mesh, folder="stl", filename="penultimate_sandia.stl")

# pc = PointCloud.from_file('lidar-data-xyz/sandia_w_cal.xyz')
# plot3d(pc)
# # Filtering and smoothening
# pc_final = do_processing_on_point_cloud(pc, **{
#     'remove_radius_outlier' : {
#         'nb_points' : 6,
#         'radius' : 0.25
#     },
#     'gaussian' : {
#         'k' : 0.5,
#         'sigma' : 0.1
#     },
#     'k_nn_thresholding' : {
#         1 : 0.1,
#         2 : 0.2,
#         3 : 0.3,
#         4 : 0.4,
#         5 : 0.5
#     }
# })
# plot3d(pc_final)
# mesh = PointCloud.to_mesh(pc_final)
# showMesh(mesh)

# PointCloud.mesh_to_stl(mesh, folder='stl', filename='final_sandia.stl')

# scan = PointCloud.from_file("lidar-data-2d/20231112_230619.xyz")
# estimate_angular_speed(scan, show_plot=True)