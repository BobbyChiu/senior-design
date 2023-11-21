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

LIDAR_VERTICAL_SEPARATION = 15.722

def do_calibration(lidar_bottom: Lidar, lidar_top: Lidar, **kwargs) -> None:
    lidars = [lidar_top, lidar_bottom]
    background_collection_time = 10
    # get background
    print(f"START BACKGROUND COLLECTION, DURATION: {background_collection_time} s")
    start_stop_scan(lidars, background_collection_time)
    dist_from_axis, turntable_height = lidar_bottom.set_current_to_background(estimate_params=True)
    lidar_top.set_current_to_background(estimate_params=False)
    lidar_top.set_background_params(dist_from_axis, turntable_height - LIDAR_VERTICAL_SEPARATION)
    
    # TODO : what to do about this???? we need a separate 'get_background' step
    input("Done getting background: press enter to start calibrating")

    # get calibration scans
    print(f"START CALIBRATION, DURATION: {kwargs['calibration_duration']} s")
    start_stop_scan(lidars, kwargs['calibration_duration'])

    # save calibration scans
    PointCloud.to_file(lidar_bottom.curr_scan, folder="calibration", filename="ref-duck-bottom.xyz")
    PointCloud.to_file(lidar_top.curr_scan, folder="calibration", filename="ref-duck-top.xyz")

    # remove background
    for l in lidars:
        l.remove_background_on_current(use_calib_params=False)

    # save backgrounds
    PointCloud.to_file(lidar_top.background_data, folder="calibration", filename="top_background.xyz")
    PointCloud.to_file(lidar_bottom.background_data, folder="calibration", filename="bottom_background.xyz")
    # calibrate
    calibration_duck = PointCloud.stl_to_mesh('calibration/duck.stl')
    calibration_duck = PointCloud.mesh_to_pc(calibration_duck, 1000) / 10
    initial_guess = [lidar_top.dist_from_axis, 0, LIDAR_VERTICAL_SEPARATION, # lidar top pos
                    0, 0, 0,  # lidar top angle
                    lidar_bottom.dist_from_axis, 0, 0, # lidar bottom pos
                    0, 0, 0]  # lidar bottom angle
    optimal_params, top_calibration_cloud, bottom_calibration_cloud = calibrate(lidar_top, lidar_bottom, initial_guess, calibration_duck) # make sure scans align
    print(f"Optimal Scanning Params: {optimal_params}")
    plot_dual_3d_clouds(top_calibration_cloud, bottom_calibration_cloud, 'red', 'blue')

    # save calibration data
    PointCloud.to_file(optimal_params, folder="calibration", filename="calibration.data")

    # TODO : what to do about this???? we need a separate 'get_background' step
    input("Done calibration: press enter to start scanning")


def do_scan(lidar_bottom: Lidar, lidar_top: Lidar, **kwargs):

    if lidar_top.pos[0] == 0 or lidar_bottom.pos[0] == 0:
        #read in calibration data
        top_background = PointCloud.from_file("calibration/top_background.xyz")
        bottom_background = PointCloud.from_file("calibration/bottom_background.xyz")

        # set backgrounds
        lidar_top.curr_scan = top_background
        lidar_bottom.curr_scan = bottom_background

        # set background params
        dist_from_axis, turntable_height = lidar_bottom.set_current_to_background(estimate_params=True)
        lidar_top.set_current_to_background(estimate_params=False)
        lidar_top.set_background_params(dist_from_axis, turntable_height - LIDAR_VERTICAL_SEPARATION)

        # set calibration data
        optimal_params =  PointCloud.from_file("calibration/calibration.data")
        lidar_top.set_params(params=optimal_params[0:6])
        lidar_bottom.set_params(params=optimal_params[6:12])
        print("Using previous calibration data")

    # Prepare for scanning
    print("10 Seconds till scanning. Place object on turntable.")
    time.sleep(10)

    scan_duration = kwargs['scan_duration']
    print(f"Scanning for {scan_duration} seconds")
    start_stop_scan([lidar_top, lidar_bottom], scan_duration)

    # save scans
    PointCloud.to_file(lidar_bottom.curr_scan, folder="calibration", filename="scan_top.xyz")
    PointCloud.to_file(lidar_top.curr_scan, folder="calibration", filename="scan_bottom.xyz")

    # if kwargs['remove_background']:
    lidar_bottom.remove_background_on_current(use_calib_params=True)
    lidar_top.remove_background_on_current(use_calib_params=True)

    # save 2d lidar data
    # PointCloud.to_file(lidar_bottom.curr_scan, folder="lidar-data-2d")
    # PointCloud.to_file(lidar_top.curr_scan, folder="lidar-data-2d")
    # estimated_angular_speed = estimate_angular_speed(lidar_bottom.curr_scan)

    # ideal_cube = PointCloud.stl_to_mesh('calibration/rect.stl')
    # ideal_cube = PointCloud.mesh_to_pc(ideal_cube, 1000)
    # optimal_parameters, cloud = self_calibrate(lidar_top, lidar_bottom, 
    #                                            LIDAR_VERTICAL_SEPARATION, 
    #                                            ref_obj=ideal_cube)
    # print(f"Optimal scanning params: {optimal_parameters}")
    pc_bottom = lidar_bottom.get3DPointCloud()
    pc_top = lidar_top.get3DPointCloud()

    pc_final = np.vstack((pc_bottom, pc_top))
    # save pc_final before processing
    # PointCloud.to_file(pc_final, folder="lidar-data-xyz")

    plot3d(pc_final) # show point cloud
    return pc_final


def do_processing_on_point_cloud(pc, **kwargs):
    # filtering to remove outliers
    # open3d statistical outlier remover
    rrad_args = kwargs.get('remove_radius_outlier')
    if rrad_args:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pc)

        # # g statistical outlier removal
        cl, ind = pcd.remove_statistical_outlier(nb_neighbors=100, std_ratio=1.75)
        pcd = pcd.select_by_index(ind)
        # radius outlier removal
        cl, ind = pcd.remove_radius_outlier(nb_points=rrad_args['nb_points'], radius=rrad_args['radius'])
        pcd = pcd.select_by_index(ind)

        pc = np.asarray(pcd.points)

    # filtering to remove outliers
    knn_args = kwargs.get('k_nn_thresholding')
    if knn_args:
        for k, threshold in knn_args.items():
            mask = PointCloud.knn_filter(pc, k, threshold)
            pc = pc[mask[:, 0]]

    # make sure top and bottom are closed
    num_points = pc.shape[0]
    vertical_range = pc[:, 2].max() - pc[:, 2].min()
    pc = PointCloud.fillFace(pc, 
                            density=10, num_points_from_edge=int(num_points/100), dist_from_edge=vertical_range/30, 
                            bottom=True, top=True)

    # smoothening
    gaussian_args = kwargs.get('gaussian')
    if gaussian_args:
        pc = PointCloud.gaussian_filter_radius(pc, gaussian_args['k'], gaussian_args['sigma'] / 10)
        estimated_normals = PointCloud.estimate_normals(pc, radius=1)
        pc = PointCloud.bilateral_filter_point_cloud(pc, 
                                                     estimated_normals, 
                                                    radius=gaussian_args['k'], 
                                                    sigma_s= gaussian_args['sigma'], 
                                                    sigma_n=0.001)   
                                                    
    plot3d(pc) # show point cloud
    # estimated_normals = PointCloud.estimate_normals(pc, radius=0.5)
    # pc = PointCloud.bilateral_filter_point_cloud(pc, estimated_normals, 
    #                                              radius=0.5, sigma_s=0.1, sigma_n=0.1)
    return pc


def do_generate_mesh(pc, **kwargs):
    # generate mesh
    mesh = PointCloud.to_mesh(pc)
    showMesh(mesh) # show mesh

    return mesh


def do_generate_files(pc=None, lidar_bottom=None, lidar_top=None, mesh=None, **kwargs) -> None:
    # TODO: Do something with kwargs['scaling']

    if pc is not None:
        PointCloud.to_file(pc, filename=kwargs.get('save_as_xyz'))

    if lidar_bottom is not None:
        PointCloud.to_file(lidar_bottom.curr_scan, folder="lidar-data-2d", filename="latest_bottom.xyz")

    if lidar_top is not None:
        PointCloud.to_file(lidar_top.curr_scan, folder="lidar-data-2d", filename="latest_top.xyz")

    if mesh is not None:
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
        # ['calibrate', '--calibration-duration', '60'],
        ['scan', '--scan-duration', '60', '--remove-background'],
        ['process',
            '--remove-radius-outlier', '3', '0.25',
        ],
        ['generate',
            '--scaling', '1.0',
            '--save-as-stl', 'stl/latest_stl.stl',
            '--save-as-xyz', 'lidar-data-xyz/latest_pc.xyz',
        ],
    ]


    argsource: ArgSource.ArgSource = None
    launch_ns = launch_parser.parse_args()
    if launch_ns.args_from == 'script':
        argsource = ArgSource.ScriptSource(input_streamlike)
    elif launch_ns.args_from == 'socket':
        argsource = ArgSource.SocketSource('localhost', 12369)

    lidar_bottom = Lidar('COM5', dist_lim=(10,50), angle_lim=(-20,45))
    lidar_top = Lidar('COM3', dist_lim=(10,50), angle_lim=(-45,0))
    with argsource:
        def process_commands():
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
                        do_calibration(lidar_bottom, lidar_top, **arg_dict)
                        output_string = 'Calibrate finished.'
                    elif arg_dict['do'] == 'scan':
                        pc_raw = do_scan(lidar_bottom, lidar_top, **arg_dict)
                        output_string = 'Scan finished.'
                    elif arg_dict['do'] == 'open':
                        pc_raw = PointCloud.from_file(arg_dict['open_xyz'])
                        output_string = 'Open finished.'
                    elif arg_dict['do'] == 'process':
                        pc_processed = do_processing_on_point_cloud(pc=pc_raw, **arg_dict)
                        output_string = 'Process finished.'
                    elif arg_dict['do'] == 'generate':
                        mesh = do_generate_mesh(pc=pc_processed, **arg_dict)
                        do_generate_files(pc=pc_processed, 
                                        lidar_bottom=lidar_bottom, 
                                        lidar_top=lidar_top,
                                        mesh=mesh,
                                        **arg_dict)
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

        top_buffer = lidar_top.get_buffer()
        bottom_buffer = lidar_bottom.get_buffer()

        while top_buffer.empty() or bottom_buffer.empty():
            time.sleep(0.001)

        # run in separate thread since Matplotlib requires the main thread
        t = threading.Thread(target=process_commands, daemon=True)
        t.start()

        try:
            plot2d_realtime([top_buffer, bottom_buffer], ['red', 'blue'], interval=10)
        finally:
            lidar_top.disconnect()
            lidar_bottom.disconnect()


