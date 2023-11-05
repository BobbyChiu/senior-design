import PointCloud
from Lidar import Lidar, estimate_angular_speed
from Visualization import plot3d, showMesh
import time
import argparse
import pathlib


def do_calibration(lidar: Lidar, **kwargs) -> None:
    # Prepare for calibration
    print("10 Seconds till calibration. Remove any objects on turntable.")
    time.sleep(10)

    calibration_time = 5
    print(f"START CALIBRATION, DURATION: {calibration_time} s")
    lidar.startScan()
    time.sleep(calibration_time)
    lidar.stopScan()
    lidar.calibrate_on_current()


def do_scan(lidar: Lidar, **kwargs) -> None:
    # Prepare for scanning
    print("10 Seconds till scanning. Place object on turntable.")
    time.sleep(10)

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
            do_calibration(lidar=lidar, **kwargs)

            # Prepare for scanning
            print("10 Seconds till scanning. Place object on turntable.")
            time.sleep(10)

            # Scanning
            do_scan(lidar=lidar, **kwargs)

            # Processing
            pc = do_processing_on_lidar(lidar=lidar, **kwargs)
            pc = do_processing_on_point_cloud(pc=pc, **kwargs)

            # Generate
            mesh = do_generate_mesh(pc=pc, **kwargs)
            do_generate_files(pc=pc, lidar=lidar, mesh=mesh, **kwargs)

        lidar.showPlot(new_main)
        estimate_angular_speed(lidar.curr_scan[:,0], lidar.curr_scan[:, 2], show_plot=True)


if __name__ == '__main__':
    # Calibration arguments
    calibration_parser = argparse.ArgumentParser(add_help=False)
    calibration_parser.add_argument('--calibration-duration', type=float, help='calibration duration in seconds', metavar='SECONDS')

    # Scan arguments
    scan_parser = argparse.ArgumentParser(add_help=False)
    scan_parser.add_argument('--scan-duration', type=float, help='scan duration in seconds', metavar='SECONDS')

    # Open point cloud file arguments
    open_parser = argparse.ArgumentParser(add_help=False)
    open_parser.add_argument('--open-xyz', type=pathlib.Path, help='xyz point cloud file to open', metavar='XYZ_FILE')

    # Processing arguments
    processing_parser = argparse.ArgumentParser(add_help=False)
    processing_parser.add_argument('--remove-background', action='store_true', help='remove background points after scanning')
    # Filtering
    processing_parser.add_argument('--k-nn-thresholding', nargs='*', help='apply sequential k-nearest neighbor filtering given by space-separated list of space-separated k and threshold', metavar='K THRESHOLD ')
    # Smoothening
    processing_parser.add_argument('--gaussian', nargs=2, help='apply gaussian filter given by space-separated k and sigma', metavar=('K', 'SIGMA'))

    # Generation arguments
    generation_parser = argparse.ArgumentParser(add_help=False)
    generation_parser.add_argument('--save-as-stl', type=pathlib.Path, help='stl file to save to', metavar='STL_FILE')
    generation_parser.add_argument('--save-as-xyz', type=pathlib.Path, help='xyz file to save to', metavar='XYZ_FILE')

    parser = argparse.ArgumentParser(parents=[calibration_parser, scan_parser, open_parser, processing_parser, generation_parser])
    actions = ['calibrate', 'scan', 'open', 'generate']
    parser.add_argument('do', choices=actions, help=f'action to take, {actions}', metavar='ACTION')

    def parse_arguments(arguments: list[str]) -> dict:
        args_dict = vars(parser.parse_args(arguments))
        if args_dict['do'] == 'calibrate':
            if not args_dict['calibration_duration']:
                raise KeyError('Calibration duration is required')
        elif args_dict['do'] == 'scan':
            if not args_dict['scan_duration']:
                raise KeyError('Scan duration is required')
        elif args_dict['do'] == 'open':
            if not args_dict['open_xyz']:
                raise KeyError('xyz file is required')
        elif args_dict['do'] == 'generate':
            # K-nn thresholding
            knn_args = args_dict['k_nn_thresholding']
            if knn_args:
                if len(knn_args) % 2 != 0:
                    raise ValueError(f'Number of K-nn arguments must be even but is {len(knn_args)}')
                knn_list = [{
                    'k': int(knn_args[i]),
                    'threshold': float(knn_args[i + 1]),
                } for i in range(0, len(knn_args), 2)]
                args_dict['k_nn_thresholding'] = knn_list

            # Gaussian filter
            gaussian_args = args_dict['gaussian']
            if gaussian_args:
                args_dict['gaussian'] = {
                    'k': int(gaussian_args[0]),
                    'sigma': float(gaussian_args[1]),
                }

            if not args_dict['save_as_stl'] and not args_dict['save_as_xyz']:
                raise KeyError('Save file is required')

        return args_dict


    test_input = [
        ['calibrate', '--calibration-duration', '5'],
        ['scan', '--scan-duration', '30'],
        ['generate',
            '--remove-background',
            '--k-nn-thresholding',
                '2', '0.4',
                '3', '0.6',
                '4', '0.8',
                '5', '1.0',
                '6', '1.2',
                '7', '1.4',
                '8', '1.6',
                '9', '1.8',
                '10', '2.0',
            '--gaussian', '0', '1',
            '--save-as-stl', 'first-sandia.stl',
            '--save-as-xyz', 'first-sandia.xyz',
        ],
    ]

    while True:
        # Receive input
        for arguments in test_input:
            arg_dict = parse_arguments(arguments)
            print(arg_dict)

        # Exit condition
        if True:
            break


    # with Lidar('com3', dist_lim=(0,100), angle_lim=(0,180), angular_speed=30, dist_from_axis=30) as lidar:
    #     def new_main():
    #         # Calibrate
    #         do_calibration(lidar=lidar)

    #         # Scanning
    #         do_scan(lidar=lidar)

    #         # Processing
    #         pc = do_processing_on_lidar(lidar=lidar)
    #         pc = do_processing_on_point_cloud(pc=pc)

    #         # Generate
    #         mesh = do_generate_mesh(pc=pc)
    #         do_generate_files(pc=pc, lidar=lidar, mesh=mesh)

    #     lidar.showPlot(new_main)
    #     estimate_angular_speed(lidar.curr_scan[:,0], lidar.curr_scan[:, 2], show_plot=True)


    # pc = PointCloud.from_file("lidar-data-xyz/george.xyz")
    # plot3d(pc)

    # do_processing_on_point_cloud(pc)
    # make sure top and bottom are closed
    # pc = PointCloud.fillFace(pc, bottom=True, top=True)

    # mesh = do_generate_mesh(pc)

    # do_generate_files(mesh=mesh)
    # PointCloud.mesh_to_stl(mesh, 'stl', 'wide_sandia.stl')

    # scan = PointCloud.from_file("lidar-data-2d/20231015_133430.xyz")
    # estimate_angular_speed(scan[:,0], scan[:, 2], show_plot=True)
