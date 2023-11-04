import PointCloud
from Lidar import Lidar, estimate_angular_speed
from Visualization import plot3d, showMesh
import time


def do_calibration(lidar: Lidar, **kwargs) -> None:
    calibration_time = 5
    print(f"START CALIBRATION, DURATION: {calibration_time} s")
    lidar.startScan()
    time.sleep(calibration_time)
    lidar.stopScan()
    lidar.calibrate_on_current()


def do_scan(lidar: Lidar, **kwargs) -> None:
    print("Scanning for 30 seconds")
    lidar.startScan()
    time.sleep(30)
    lidar.stopScan()


def do_processing_on_lidar(lidar: Lidar, **kwargs):
    lidar.remove_background_on_current()
    pc = lidar.get3DPointCloud()

    plot3d(pc) # show point cloud

    return pc

def do_processing_on_point_cloud(pc, **kwargs):
    # filtering to remove outliers
    for i in range(2, 10):
        mask = PointCloud.knn_filter(pc, i, 0.2 * i)
        pc = pc[mask[:, 0]]

    # smoothening
    pc = PointCloud.median_filter(pc, 25)

    # make sure top and bottom are closed
    pc = PointCloud.fillFace(pc, bottom=True, top=True)


def do_generate_mesh(pc, **kwargs):
    # generate mesh
    mesh = PointCloud.to_mesh(pc)
    plot3d(pc) # show point cloud
    showMesh(mesh) # show mesh

    return mesh

def do_generate_files(pc=None, lidar=None, mesh=None, **kwargs) -> None:
    if pc:
        PointCloud.to_file(pc, folder="lidar-data-xyz")

    if lidar:
        PointCloud.to_file(lidar.curr_scan, folder="lidar-data-2d")

    if mesh:
        # save as stl file
        PointCloud.mesh_to_stl(mesh, 'stl', 'first_sandia.stl')



def do_pipeline(**kwargs) -> None:
    with Lidar('com3', dist_lim=(0,100), angle_lim=(0,180), angular_speed=30, dist_from_axis=30) as lidar:
        def new_main():
            # Prepare for calibration
            print("10 Seconds till calibration. Remove any objects on turntable.")
            time.sleep(10)

            # Calibrate
            do_calibration(lidar=lidar)

            # Prepare for scanning
            print("10 Seconds till scanning. Place object on turntable.")
            time.sleep(10)

            # Scanning
            do_scan(lidar=lidar)

            # Processing
            pc = do_processing_on_lidar(lidar=lidar)
            pc = do_processing_on_point_cloud(pc)

            # Generate
            mesh = do_generate_mesh(pc)
            do_generate_files(pc=pc, lidar=lidar, mesh=mesh)

        lidar.showPlot(new_main)
        estimate_angular_speed(lidar.curr_scan[:,0], lidar.curr_scan[:, 2], show_plot=True)


if __name__ == '__main__':
    # do_pipeline()


    pc = PointCloud.from_file("lidar-data-xyz/george.xyz")
    plot3d(pc)

    do_processing_on_point_cloud(pc)
    # make sure top and bottom are closed
    # pc = PointCloud.fillFace(pc, bottom=True, top=True)

    mesh = do_generate_mesh(pc)

    # do_generate_files(mesh=mesh)
    # PointCloud.mesh_to_stl(mesh, 'stl', 'wide_sandia.stl')

    # scan = PointCloud.from_file("lidar-data-2d/20231015_133430.xyz")
    # estimate_angular_speed(scan[:,0], scan[:, 2], show_plot=True)
