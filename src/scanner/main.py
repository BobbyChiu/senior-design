import PointCloud
from Lidar import Lidar, estimate_angular_speed
from Visualization import plot3d, showMesh
import ArgSource
import time
import argparse
import pathlib
import numpy as np
import open3d as o3d
from traceback import print_exc


def do_calibration(lidar: Lidar, **kwargs) -> None:
    # Prepare for calibration
    print("10 Seconds till calibration. Remove any objects on turntable.")
    time.sleep(10)

    calibration_time = kwargs['calibration_duration']
    print(f"START CALIBRATION, DURATION: {calibration_time} s")
    lidar.startScan()
    time.sleep(calibration_time)
    lidar.stopScan()
    lidar.calibrate_on_current()


def do_scan(lidar: Lidar, **kwargs):
    # Prepare for scanning
    print("10 Seconds till scanning. Place object on turntable.")
    time.sleep(10)

    scan_duration = kwargs['scan_duration']
    print(f"Scanning for {scan_duration} seconds")
    lidar.startScan()
    time.sleep(scan_duration)
    lidar.stopScan()

    if kwargs['remove_background']:
        lidar.remove_background_on_current()

    pc = lidar.get3DPointCloud()
    plot3d(pc) # show point cloud
    return pc


def do_processing_on_point_cloud(pc, **kwargs):
    # filtering to remove outliers

    # open3d statistical outlier remover
    rrad_args = kwargs['remove_radius_outlier']
    if rrad_args:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pc)

        # g statistical outlier removal
        # cl, ind = pcd.remove_statistical_outlier(nb_neighbors=100, std_ratio=0.5)
        # denoised_pcd = pcd.select_by_index(ind)
        # radius outlier removal
        cl, ind = pcd.remove_radius_outlier(nb_points=rrad_args['nb_points'], radius=rrad_args['radius'])
        denoised_pcd = pcd.select_by_index(ind)

        pc = np.asarray(denoised_pcd.points)

    # filtering to remove outliers
    knn_args = kwargs['k_nn_thresholding']
    if knn_args:
        for k, threshold in knn_args.items():
            mask = PointCloud.knn_filter(pc, k, threshold)
            pc = pc[mask[:, 0]]

    # smoothening
    gaussian_args = kwargs['gaussian']
    if gaussian_args:
        pc = PointCloud.gaussian_filter(pc, gaussian_args['k'], gaussian_args['sigma'])

    # make sure top and bottom are closed
    pc = PointCloud.fillFace(pc, bottom=True, top=True)

    plot3d(pc) # show point cloud
    return pc


def do_generate_mesh(pc, **kwargs):
    # generate mesh
    mesh = PointCloud.to_mesh(pc)
    showMesh(mesh) # show mesh

    return mesh


def do_generate_files(pc=None, lidar=None, mesh=None, **kwargs) -> None:
    # TODO: Do something with kwargs['scaling']

    if pc:
        PointCloud.to_file(pc, filename=kwargs.get('save_as_xyz'))

    if lidar:
        PointCloud.to_file(lidar.curr_scan, folder="lidar-data-2d")

    if mesh:
        # save as stl file
        PointCloud.mesh_to_stl(mesh, filename=kwargs.get('save_as_stl'))


if __name__ == '__main__':
    launch_parser = argparse.ArgumentParser()
    arg_sources = ['script', 'socket']
    launch_parser.add_argument('--args-from', default='script', choices=arg_sources, help=f'how to run commands, {arg_sources}', metavar='TYPE')

    # Calibration arguments
    calibration_parser = argparse.ArgumentParser(add_help=False)
    calibration_parser.add_argument('--calibration-duration', type=float, help='calibration duration in seconds', metavar='SECONDS')

    # Scan arguments
    scan_parser = argparse.ArgumentParser(add_help=False)
    scan_parser.add_argument('--scan-duration', type=float, help='scan duration in seconds', metavar='SECONDS')
    scan_parser.add_argument('--remove-background', action='store_true', help='remove background points after scanning')

    # Open point cloud file arguments
    open_parser = argparse.ArgumentParser(add_help=False)
    open_parser.add_argument('--open-xyz', type=pathlib.Path, help='xyz point cloud file to open', metavar='XYZ_FILE')

    # Processing arguments
    process_parser = argparse.ArgumentParser(add_help=False)
    # Filtering
    process_parser.add_argument('--remove-radius-outlier', nargs=2, help='remove points with not enough neighbors given by a number of points and radius in centimeters', metavar=('NUMBER_OF_POINTS', 'RADIUS'))
    process_parser.add_argument('--k-nn-thresholding', nargs='*', help='apply sequential k-nearest neighbor filtering given by space-separated list of space-separated k and threshold', metavar='K THRESHOLD ')
    # Smoothening
    process_parser.add_argument('--gaussian', nargs=2, help='apply gaussian filter given by space-separated k and sigma', metavar=('K', 'SIGMA'))

    # Generation arguments
    generation_parser = argparse.ArgumentParser(add_help=False)
    generation_parser.add_argument('--scaling', type=float, help='scaling factor on generated files')
    generation_parser.add_argument('--save-as-stl', type=pathlib.Path, help='stl file to save to', metavar='STL_FILE')
    generation_parser.add_argument('--save-as-xyz', type=pathlib.Path, help='xyz file to save to', metavar='XYZ_FILE')

    parser = argparse.ArgumentParser(parents=[calibration_parser, scan_parser, open_parser, process_parser, generation_parser], exit_on_error=False)
    actions = ['!help', 'calibrate', 'scan', 'open', 'process', 'generate', '!quit']
    parser.add_argument('do', nargs='?', default='!help', choices=actions, help=f'action to take, {actions}', metavar='ACTION')
    parser.add_argument('ignore', nargs='*', default=[], help='other positional arguments after the first are ignored')

    def parse_arguments(arguments: list[str]) -> dict:
        args_dict = vars(parser.parse_intermixed_args(arguments))
        args_dict['do'] = args_dict['do']
        if args_dict['do'] == 'calibrate':
            if not args_dict['calibration_duration']:
                raise KeyError('Calibration duration is required')
        elif args_dict['do'] == 'scan':
            if not args_dict['scan_duration']:
                raise KeyError('Scan duration is required')
        elif args_dict['do'] == 'open':
            if not args_dict['open_xyz']:
                raise KeyError('xyz file is required')
        elif args_dict['do'] == 'process':
            # Remove radius outlier
            rrad_args = args_dict['remove_radius_outlier']
            if rrad_args:
                args_dict['remove_radius_outlier'] = {
                    'nb_points': int(rrad_args[0]),
                    'radius': float(rrad_args[1]),
                }

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

        elif args_dict['do'] == 'generate':
            if not args_dict['save_as_stl'] and not args_dict['save_as_xyz']:
                raise KeyError('Save file is required')

        return args_dict


    input_streamlike = [
        ['calibrate', '--calibration-duration', '5'],
        ['scan', '--scan-duration', '30', '--remove-background'],
        ['process',
            '--remove-radius-outlier', '8', '0.25',
            '--gaussian', '25', '4',
        ],
        ['generate',
            '--scaling', '1.0',
            '--save-as-stl', 'stl/first-sandia.xyz',
            '--save-as-xyz', 'lidar-data-xyz',
        ],
    ]


    argsource: ArgSource.ArgSource = None
    launch_ns = launch_parser.parse_args()
    if launch_ns.args_from == 'script':
        argsource = ArgSource.ScriptSource(input_streamlike)
    elif launch_ns.args_from == 'socket':
        argsource = ArgSource.SocketSource('localhost', 12369)


    LIDAR_VERTICAL_SEPARATION = 15.722
    with (
        argsource,
        Lidar('com3', dist_lim=(0,100), angle_lim=(0,180), angular_speed=30, dist_from_axis=30) as lidar,
    ):
    #     with Lidar('com4', dist_lim=(0,50), angle_lim=(-45,45)) as lidar_top:
        # Keep state
        pc_raw = None
        pc_processed = None
        mesh = None

        for arguments in argsource.command_generator():
            output_string = ''
            try:
                # Receive input
                arg_dict = parse_arguments(arguments)

                # Do action
                if arg_dict['do'] == '!help':
                    output_string = parser.format_help()
                elif arg_dict['do'] == 'calibrate':
                    do_calibration(lidar=lidar, **arg_dict)
                    output_string = 'Calibrate finished.'
                elif arg_dict['do'] == 'scan':
                    pc_raw = do_scan(lidar=lidar, **arg_dict)
                    output_string = 'Scan finished.'
                elif arg_dict['do'] == 'open':
                    pc_raw = PointCloud.from_file(arg_dict['open_xyz'])
                    output_string = 'Open finished.'
                elif arg_dict['do'] == 'process':
                    pc_processed = do_processing_on_point_cloud(pc=pc_raw, **arg_dict)
                    output_string = 'Process finished.'
                elif arg_dict['do'] == 'generate':
                    mesh = do_generate_mesh(pc=pc_processed, **arg_dict)
                    do_generate_files(pc=pc_processed, lidar=lidar, mesh=mesh, **arg_dict)
                    output_string = 'Generate finished.'
                elif arg_dict['do'] == '!quit':
                    # Exit condition
                    output_string = '!quit'
                    break
                else:
                    raise ValueError(f'unknown ACTION: {arg_dict["do"]}')
            except Exception as e:
                print_exc()
                output_string = f'{str(e)}\nType "!help" for actions and arguments.\n'
            finally:
                argsource.output_string(output_string)


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


    # pc = PointCloud.from_file("lidar-data-xyz/new_sandia.xyz")
    # plot3d(pc) # show point cloud
    # pc = post_processing(pc)
    # plot3d(pc) # show point cloud
    # mesh = PointCloud.to_mesh(pc)
    # showMesh(mesh) # show mesh
    # save as stl file
    # PointCloud.mesh_to_stl(mesh, 'stl')

