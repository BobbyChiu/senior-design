import PointCloud
from Lidar import Lidar, estimate_angular_speed
from Visualization import plot3d, showMesh
import time
import numpy as np

if __name__ == '__main__':
    lidar = Lidar('com3', dist_lim=(0,100), angle_lim=(0,180), angular_speed=30, dist_from_axis=30)

    def new_main():
        print("10 Seconds till calibration. Remove any objects on turntable.")
        time.sleep(10)
        lidar.calibrate(5)
        print("10 Seconds till scanning. Place object on turntable.")
        time.sleep(10)
        lidar.startScan()
        print("Scanning for 30 seconds")
        time.sleep(30)
        lidar.stopScan()
        lidar.remove_background()
        pc = lidar.get3DPointCloud()
        plot3d(pc) # show point cloud
        # filtering to remove outliers
        for i in range(2, 10):
            mask = PointCloud.knn_filter(pc, i, 0.2 * i)
            pc = pc[mask[:, 0]]
        # smoothening
        pc = PointCloud.median_filter(pc, 25)
        # make sure top and bottom are closed
        pc = PointCloud.fillFace(pc, bottom=True, top=True)
        # generate mesh
        mesh = PointCloud.to_mesh(pc) 
        plot3d(pc) # show point cloud
        showMesh(mesh) # show mesh
        PointCloud.to_file(pc, folder="lidar-data-xyz")
        PointCloud.to_file(lidar.curr_scan, folder="lidar-data-2d")
        # save as stl file
        PointCloud.mesh_to_stl(mesh, 'stl', 'first_sandia.stl')

    lidar.showPlot(new_main)
    estimate_angular_speed(lidar.curr_scan[:,0], lidar.curr_scan[:, 2], show_plot=True)

    # pc = PointCloud.from_file("lidar-data-xyz/20231014_203147.xyz")
    # # filtering to remove outliers
    # for i in range(2, 10):
    #     mask = PointCloud.knn_filter(pc, i, 0.2 * i)
    #     pc = pc[mask[:, 0]]
    # # smoothening
    # pc = PointCloud.median_filter(pc, 25)
    # # make sure top and bottom are closed
    # pc = PointCloud.fillFace(pc, bottom=True, top=True)
    # # generate mesh
    # mesh = PointCloud.to_mesh(pc) 
    # plot3d(pc) # show point cloud
    # showMesh(mesh) # show mesh

    # # save as stl file
    # PointCloud.mesh_to_stl(mesh, 'stl', 'first_sandia.stl')