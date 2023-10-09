import PointCloud
from Lidar import Lidar, estimate_angular_speed
from Visualization import plot3d

import time

if __name__ == '__main__':
    # do_lidar(save_data=True, show_3d_plot=True, view='vertical_slice')

    # show_3d_plot from 2d data file
    # lidar_cumulative_scan = [np.empty((0, 3))] # all lidar data collected so far
    # lidar_cumulative_scan[0] = np.loadtxt('lidar-data/cube.csv', delimiter=",")
    # x, y, z = lidar_to_3d(lidar_cumulative_scan[0], angular_speed=33.75, dist_from_axis=31.5)
    # plot3d(x, y, z)

    # cube is 33.5, 31.5
    # sandia is 33.75, 42.5

    # # # show 3d plot from 3d data file
    # lidar_cumulative_scan = [np.empty((0, 3))] # all lidar data collected so far
    # lidar_cumulative_scan[0] = np.loadtxt('lidar-data/george.xyz')
    # x, y, z = lidar_cumulative_scan[0][:,0],  lidar_cumulative_scan[0][:,1],  lidar_cumulative_scan[0][:,2]
    # plot3d(x, y, z)

    # # # # show 3d plot from 3d data file
    # lidar_cumulative_scan = [np.empty((0, 3))] # all lidar data collected so far
    # lidar_cumulative_scan[0] = np.loadtxt('../puflow/output/george.xyz')
    # x, y, z = lidar_cumulative_scan[0][:,0],  lidar_cumulative_scan[0][:,1],  lidar_cumulative_scan[0][:,2]
    # plot3d(x, y, z)

    # # # convert 2d data file to 3d data file
    # lidar_cumulative_scan = [np.empty((0, 3))] # all lidar data collected so far
    # lidar_cumulative_scan[0] = np.loadtxt('lidar-data/cube.csv', delimiter=",")
    # x, y, z = lidar_to_3d(lidar_cumulative_scan[0], angular_speed=33.5, dist_from_axis=31.5)
    # np.savetxt('lidar-data/cube.xyz', np.column_stack((x, y, z)), fmt="%f")

    # lidar = Lidar('com3', dist_lim=(0,60), angle_lim=(30,150), angular_speed=30, dist_from_axis=30)

    # def new_main():
    #     print("5 Seconds till calibration. Remove any objects on turntable.")
    #     time.sleep(5)
    #     lidar.calibrate(5)
    #     print("5 Seconds till scanning. Place object on turntable.")
    #     time.sleep(5)
    #     lidar.startScan()
    #     print("Scanning for 40 seconds")
    #     time.sleep(40)
    #     lidar.stopScan()
    #     lidar.remove_background()
    #     pc = lidar.get3DPointCloud()
    #     plot3d(pc)
    #     # save to file
    #     PointCloud.to_file(pc, folder="lidar-data-xyz")
    #     PointCloud.to_file(lidar.curr_scan, folder="lidar-data-2d")

    # lidar.showPlot(new_main)
    # estimate_angular_speed(lidar.curr_scan[:,0], lidar.curr_scan[:, 2], show_plot=True)

    pc = PointCloud.from_file("lidar-data-xyz/20231009_124200.xyz")
    scan = PointCloud.from_file("lidar-data-2d/20231009_124200.xyz")
    plot3d(pc)
    estimate_angular_speed(scan[:,0], scan[:, 2], show_plot=True)


    # # mesh
    # points = np.loadtxt('../puflow/output/george.xyz')
    # cloud = pv.PolyData(points)
    # mesh = cloud.delaunay_2d(tol=0.01)
    # # Plot the mesh
    # plotter = pv.Plotter()
    # plotter.add_mesh(mesh, color='white')
    # plotter.show()