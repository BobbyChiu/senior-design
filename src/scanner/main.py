import PointCloud
from Lidar import Lidar, start_stop_scan, calibrate, auto_get_lidars
from Visualization import plot3d, showMesh, plot_dual_3d_clouds, RealTimePlotWindow
import ArgSource
import time
import argparse
import pathlib
import numpy as np
import open3d as o3d
from traceback import print_exc
import threading
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QInputDialog
import sys

LIDAR_VERTICAL_SEPARATION = 15.722
kill_test_thread = False

def calibrate_from_file(lidar_top: Lidar, lidar_bottom: Lidar):
    # read in background data
    try:
        top_background = PointCloud.from_file("calibration/top_background.xyz")
        lidar_top.curr_scan = top_background
        lidar_top.set_current_to_background()
    except FileNotFoundError:
        print("Top background file not found")
    
    try:
        bottom_background = PointCloud.from_file("calibration/bottom_background.xyz")
        lidar_bottom.curr_scan = bottom_background
        lidar_bottom.set_current_to_background()
    except FileNotFoundError:
        print("Bottom background file not found")

    # read calibration data
    try:
        optimal_params = np.loadtxt("calibration/calibration.data")
        lidar_top.set_params(np.append(optimal_params[0:3], optimal_params[6:9]))
        lidar_bottom.set_params(np.append(optimal_params[3:6], optimal_params[9:12]))
        lidar_top.angular_speed = optimal_params[12]
        lidar_bottom.angular_speed = optimal_params[13]
        print("Using previous calibration data")
    except FileNotFoundError:
        print("No calibration data found, please calibrate first")

def do_test_mode(lidar_top: Lidar, lidar_bottom: Lidar, mainWin : RealTimePlotWindow):
    # init
    calibrate_from_file(lidar_top, lidar_bottom)
    begin_time = time.time()
    pc_top = np.empty((0, 3))
    pc_bottom = np.empty((0, 3))   
    lidar_top.startScan(begin_time)
    lidar_bottom.startScan(begin_time)
    while not kill_test_thread:
        # scanning for 15 secs
        cycle_time = time.time()
        while time.time() - cycle_time < 15:
            if kill_test_thread:
                lidar_top.stopScan()
                lidar_bottom.stopScan()
                return
            time.sleep(0.001)
        # pause scanning, start processing
        lidar_top.stopScan()
        lidar_bottom.stopScan()

        # remove background
        lidar_top.remove_background_on_current()
        lidar_bottom.remove_background_on_current()
        
        # convert to 3d
        pc_bottom = lidar_bottom.get3DPointCloud()
        pc_top = lidar_top.get3DPointCloud()
        mainWin.set_plot_3d(data_bottom=pc_bottom, data_top=pc_top, plot_num=1)
        pc_combined = np.vstack((pc_top, pc_bottom))
        
        # post processing
        proc_params = {'remove_statistical_outlier' : {
                            'nb_neighbors' : 20,
                            'std_ratio' : 2,
                        },
                       'add_bottom' : True
                        }
        pc_final = do_processing_on_point_cloud(pc_combined, **proc_params)
        mainWin.set_plot_3d(pc_final, plot_num=2)
        
        # get mesh
        mesh = PointCloud.to_mesh(pc_final, voxel_size=0.1)
        mainWin.update_mesh(mesh)

        # resume scanning if time is not up
        if time.time() - begin_time > 60:
            mainWin.clear_plot_3d(1)
            begin_time = time.time()
            lidar_top.startScan(begin_time)
            lidar_bottom.startScan(begin_time)
        else:
            lidar_top.resumeScan()
            lidar_bottom.resumeScan()


def do_get_background(lidar_bottom: Lidar, lidar_top: Lidar, **kwargs) -> None:
    mainWin.clear_plot_3d(1)
    lidars = [lidar_top, lidar_bottom]
    background_collection_time = kwargs['background_duration']
    # get background
    print(f"START BACKGROUND COLLECTION, DURATION: {background_collection_time} s")
    start_stop_scan(lidars, background_collection_time)
    lidar_bottom.set_current_to_background()
    lidar_top.set_current_to_background()

    # save backgrounds
    PointCloud.to_file(lidar_top.background_data, folder="calibration", filename="top_background.xyz")
    PointCloud.to_file(lidar_bottom.background_data, folder="calibration", filename="bottom_background.xyz")


def do_calibration(lidar_bottom: Lidar, lidar_top: Lidar, **kwargs) -> None:
    mainWin.clear_plot_3d(1)

    lidars = [lidar_top, lidar_bottom]

    # get calibration scans
    print(f"START CALIBRATION, DURATION: {kwargs['calibration_duration']} s")
    start_stop_scan(lidars, kwargs['calibration_duration'])

    # save calibration scans
    PointCloud.to_file(lidar_bottom.curr_scan, folder="calibration", filename="ref-duck-bottom.xyz")
    PointCloud.to_file(lidar_top.curr_scan, folder="calibration", filename="ref-duck-top.xyz")

    # remove background
    for l in lidars:
        l.remove_background_on_current(use_calib_params=False)

    # calibrate
    calibration_duck = PointCloud.stl_to_mesh('calibration/duck.stl')
    calibration_duck = PointCloud.mesh_to_pc(calibration_duck, 5000) / 10
    initial_guess = [0, 0, LIDAR_VERTICAL_SEPARATION, # lidar top pos
                    0, 0, 0,
                    0, 0, 0,  # lidar top angle
                    0, 0, 0]  # lidar bottom angle
    
    optimal_params, top_calibration_cloud, bottom_calibration_cloud = calibrate(lidar_top, lidar_bottom, initial_guess, calibration_duck) # make sure scans align
    print(f"Optimal Scanning Params: {optimal_params}")
    plot_dual_3d_clouds(bottom_calibration_cloud, top_calibration_cloud, 'blue', 'red')

    PointCloud.to_file(top_calibration_cloud, folder="calibration", filename="top_cal.xyz")
    PointCloud.to_file(bottom_calibration_cloud, folder="calibration", filename="bottom_cal.xyz")

    # save calibration data
    np.savetxt("calibration/calibration.data", optimal_params, fmt="%f")


def do_scan(lidar_bottom: Lidar, lidar_top: Lidar, **kwargs):
    mainWin.clear_plot_3d(1)

    scan_duration = kwargs['scan_duration']

    if scan_duration == 99:
        # popup window to ask for scan duration
        scan_duration = mainWin.get_input()
        

    print(f"Scanning for {scan_duration} seconds")
    start_stop_scan([lidar_top, lidar_bottom], scan_duration)

    # save scans before background removal
    PointCloud.to_file(lidar_bottom.curr_scan, folder="lidar-data-2d", filename="scan_bottom.xyz")
    PointCloud.to_file(lidar_top.curr_scan, folder="lidar-data-2d", filename="scan_top.xyz")

    # if kwargs['remove_background']:
    lidar_bottom.remove_background_on_current()
    lidar_top.remove_background_on_current()

    # save scans after background removal
    PointCloud.to_file(lidar_bottom.curr_scan, folder="lidar-data-2d", filename="scan_bottom.xyz")
    PointCloud.to_file(lidar_top.curr_scan, folder="lidar-data-2d", filename="scan_top.xyz")

    pc_bottom = lidar_bottom.get3DPointCloud()
    pc_top = lidar_top.get3DPointCloud()

    pc_final = np.vstack((pc_bottom, pc_top))
    # save pc_final before processing
    PointCloud.to_file(pc_final, folder="lidar-data-xyz")

    plot3d(pc_final) # show point cloud
    return pc_final


def do_processing_on_point_cloud(pc, **kwargs):

    # open3d statistical outlier remover
    stat_args = kwargs.get('remove_statistical_outlier')
    if stat_args:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pc)
        # statistical outlier removal
        cl, ind = pcd.remove_statistical_outlier(nb_neighbors=stat_args['nb_neighbors'], 
                                                std_ratio=stat_args['std_ratio'])
        pcd = pcd.select_by_index(ind)
        pc = np.asarray(pcd.points)

    # open3d radius outlier removal
    rrad_args = kwargs.get('remove_radius_outlier')
    if rrad_args:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pc)
        # radius outlier removal
        cl, ind = pcd.remove_radius_outlier(nb_points=rrad_args['nb_points'], radius=rrad_args['radius'])
        pcd = pcd.select_by_index(ind)
        pc = np.asarray(pcd.points)

    # # filtering to remove outliers
    # knn_args = kwargs.get('k_nn_thresholding')
    # if knn_args:
    #     for k, threshold in knn_args.items():
    #         mask = PointCloud.knn_filter(pc, k, threshold)
    #         pc = pc[mask[:, 0]]

    # smoothening
    gaussian_args = kwargs.get('gaussian')
    if gaussian_args:
        pc = PointCloud.gaussian_filter_radius(pc, gaussian_args['k'], gaussian_args['sigma'])

    bilateral_args = kwargs.get('bilateral')
    if bilateral_args:
        estimated_normals = PointCloud.estimate_normals(pc, radius=1)
        pc = PointCloud.bilateral_filter_point_cloud(pc, 
                                                     estimated_normals, 
                                                    radius=bilateral_args['k'], 
                                                    sigma_s=bilateral_args['sigma_s'], 
                                                    sigma_n=bilateral_args['sigma_n'])   

    if kwargs.get('add_bottom'):
        # add flat bottom surface
        pc = PointCloud.add_bottom_surface(pc, 
                                        z_percentile=1, 
                                        percent_height=1, 
                                        grid_spacing=0.1, 
                                        crop=True)
    
    if kwargs.get('add_top'):
        # add flat top surface
        pc[:, 2] = - pc[:, 2]
        pc = PointCloud.add_bottom_surface(pc, 
                                        z_percentile=1,
                                        percent_height=1,
                                        grid_spacing=0.1,
                                        crop=True)
        pc[:, 2] = - pc[:, 2]
        
    plot3d(pc, plot_num=2)
    return pc


def do_generate_mesh(pc, **kwargs):
    # generate mesh
    mesh = PointCloud.to_mesh(pc, 0.1)
    showMesh(mesh) # show mesh

    return mesh


def do_generate_files(pc=None, lidar_bottom=None, lidar_top=None, mesh=None, **kwargs) -> None:
    # TODO: Do something with kwargs['scaling']

    if pc is not None and kwargs.get('save_as_xyz') is not None:
        PointCloud.to_file(pc, filename=kwargs.get('save_as_xyz'))

    if lidar_bottom is not None:
        PointCloud.to_file(lidar_bottom.curr_scan, folder="lidar-data-2d", filename="latest_bottom.xyz")

    if lidar_top is not None:
        PointCloud.to_file(lidar_top.curr_scan, folder="lidar-data-2d", filename="latest_top.xyz")

    if mesh is not None and kwargs.get('save_as_stl') is not None:
        # save as stl file
        PointCloud.mesh_to_stl(mesh, filename=kwargs.get('save_as_stl'))


if __name__ == '__main__':
    launch_parser = argparse.ArgumentParser()
    arg_sources = ['script', 'socket']
    launch_parser.add_argument('--args-from', default='script', choices=arg_sources, help=f'how to run commands, {arg_sources}', metavar='TYPE')

    # Get background arguments
    background_parser = argparse.ArgumentParser(add_help=False)
    background_parser.add_argument('--background-duration', type=float, help='get background duration in seconds', metavar='SECONDS')

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
    process_parser.add_argument('--remove-statistical-outlier', nargs=2, help='remove points that are sparse given by a number of points and standard deviation', metavar=('NUMBER_OF_POINTS', 'STDEV'))
    # process_parser.add_argument('--k-nn-thresholding', nargs='*', help='apply sequential k-nearest neighbor filtering given by space-separated list of space-separated k and threshold', metavar='K THRESHOLD ')
    # Smoothening
    process_parser.add_argument('--gaussian', nargs=2, help='apply gaussian filter given by space-separated k and sigma', metavar=('K', 'SIGMA'))
    process_parser.add_argument('--bilateral', nargs=3, help='apply bilateral filter given by space-separated k, sigma_s, sigma_n', metavar=('K', 'SIGMA_S', 'SIGMA_N'))
    process_parser.add_argument('--add-bottom', action='store_true', help='add flat bottom surface')
    process_parser.add_argument('--add-top', action='store_true', help='add flat top surface')

    # Generation arguments
    generation_parser = argparse.ArgumentParser(add_help=False)
    generation_parser.add_argument('--scaling', type=float, help='scaling factor on generated files')
    generation_parser.add_argument('--save-as-stl', type=pathlib.Path, help='stl file to save to', metavar='STL_FILE')
    generation_parser.add_argument('--save-as-xyz', type=pathlib.Path, help='xyz file to save to', metavar='XYZ_FILE')

    # Test mode arguments
    test_parser = argparse.ArgumentParser(add_help=False)
    test_parser.add_argument('--toggle-test', action='store_true', help='toggle test mode (live scan viewer, disables data collection)')

    parser = argparse.ArgumentParser(parents=[background_parser, calibration_parser, scan_parser, open_parser, process_parser, generation_parser, test_parser], exit_on_error=False)
    actions = ['!help', 'get-background', 'calibrate', 'scan', 'open', 'process', 'generate', 'test', '!quit']
    parser.add_argument('do', nargs='?', default='!help', choices=actions, help=f'action to take, {actions}', metavar='ACTION')
    parser.add_argument('ignore', nargs='*', default=[], help='other positional arguments after the first are ignored')

    def parse_arguments(arguments: list[str]) -> dict:
        args_dict = vars(parser.parse_intermixed_args(arguments))
        args_dict['do'] = args_dict['do']
        if args_dict['do'] == 'get-background':
            if not args_dict['background_duration']:
                raise KeyError('Background duration is required')
        elif args_dict['do'] == 'calibrate':
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

            # Remove statistical outlier
            rrad_args = args_dict['remove_statistical_outlier']
            if rrad_args:
                args_dict['remove_statistical_outlier'] = {
                    'nb_neighbors': int(rrad_args[0]),
                    'std_ratio': float(rrad_args[1]),
                }

            # Gaussian filter
            gaussian_args = args_dict['gaussian']
            if gaussian_args:
                args_dict['gaussian'] = {
                    'k': float(gaussian_args[0]),
                    'sigma': float(gaussian_args[1]),
                }

            # Bilateral filter
            bilateral_args = args_dict['bilateral']
            if bilateral_args:
                args_dict['bilateral'] = {
                    'k': float(bilateral_args[0]),
                    'sigma_s': float(bilateral_args[1]),
                    'sigma_n': float(bilateral_args[2]),
                }

        elif args_dict['do'] == 'test':
            if not args_dict['toggle_test']:
                raise KeyError('"--toggle-test" flag is required')

        return args_dict


    input_streamlike = [
        # ['get-background', '--background-duration', '10'],
        # ['calibrate', '--calibration-duration', '60'],
        ['scan', '--scan-duration', '60', '--remove-background'],
        ['process',
            # '--remove-radius-outlier', '?', '?',
            '--remove-statistical-outlier', '20', '2',
            # '--gaussian', '?', '?',
            # '--bilateral', '?', '?', '0.001',
            '--add-bottom',
            '--add-top',
        ],
        ['generate',
            '--scaling', '1.0',
            '--save-as-stl', 'stl/latest_stl.stl',
            '--save-as-xyz', 'lidar-data-xyz/latest_pc.xyz',
        ],
        ['test', '--toggle-test'],
        ['test', '--toggle-test'],
    ]


    argsource: ArgSource.ArgSource = None
    launch_ns = launch_parser.parse_args()
    if launch_ns.args_from == 'script':
        argsource = ArgSource.ScriptSource(input_streamlike)
    elif launch_ns.args_from == 'socket':
        argsource = ArgSource.SocketSource('localhost', 12369
        )

    
    lidar_top, lidar_bottom = auto_get_lidars((10, 50), (-30, 0),
                                              (10, 50), (-15, 45))
    calibrate_from_file(lidar_top, lidar_bottom)
    mainWin = None
    with argsource:
        def process_commands():
            # Keep state
            pc_raw = None
            pc_processed = None
            mesh = None

            global kill_test_thread
            test_mode = False
            test_thread = None

            for arguments in argsource.command_generator():
                output_string = ''
                try:
                    # Receive input
                    arg_dict = parse_arguments(arguments)

                    # Do action
                    if arg_dict['do'] == '!help':
                        output_string = parser.format_help()
                    elif arg_dict['do'] == '!quit':
                        # Exit condition
                        output_string = '!quit'
                        break
                    elif test_mode:
                        if arg_dict['do'] == 'test' and arg_dict['toggle_test']:
                            test_mode = False
                            kill_test_thread = True
                            test_thread.join()
                            output_string = 'Changed test mode to disabled.'
                        else:
                            raise ValueError('Test mode is currently enabled. Disable test mode to enable other commands.')
                    else:
                        if arg_dict['do'] == 'get-background':
                            do_get_background(lidar_bottom, lidar_top, **arg_dict)
                            output_string = 'Getting background finished.'
                        elif arg_dict['do'] == 'calibrate':
                            do_calibration(lidar_bottom, lidar_top, **arg_dict)
                            output_string = 'Calibrate finished.'
                        elif arg_dict['do'] == 'scan':
                            pc_raw = do_scan(lidar_bottom, lidar_top, **arg_dict)
                            output_string = 'Scan finished.'
                        elif arg_dict['do'] == 'open':
                            pc_raw = PointCloud.from_file(arg_dict['open_xyz'])
                            plot3d(pc_raw)
                            output_string = 'Open finished.'
                        elif arg_dict['do'] == 'process':
                            if pc_raw is None:
                                raise RuntimeError('No raw point cloud. Perform a scan or open a point cloud file first.')
                            pc_processed = do_processing_on_point_cloud(pc=pc_raw, **arg_dict)
                            output_string = 'Process finished.'
                        elif arg_dict['do'] == 'generate':
                            if pc_processed is None:
                                raise RuntimeError('No processed point cloud. Perform a process first.')
                            mesh = do_generate_mesh(pc=pc_processed, **arg_dict)
                            do_generate_files(pc=pc_processed,
                                              lidar_bottom=lidar_bottom,
                                              lidar_top=lidar_top,
                                              mesh=mesh,
                                              **arg_dict)
                            output_string = 'Generate finished.'
                        elif arg_dict['do'] == 'test' and arg_dict['toggle_test']:
                            test_mode = True
                            output_string = 'Changed test mode to enabled.'
                            test_thread = threading.Thread(target=do_test_mode, 
                                                                args=(lidar_top, lidar_bottom, mainWin))
                            
                            print("here")
                            kill_test_thread = False
                            test_thread.start()
                        else:
                            raise ValueError(f'unknown ACTION: {arg_dict["do"]}')
                except Exception as e:
                    print_exc()
                    output_string = f'{str(e)}\nType "!help" for actions and arguments.\n'
                finally:
                    argsource.output_string(output_string)
        try:
            top_buffer = lidar_top.get_buffer()
            bottom_buffer = lidar_bottom.get_buffer()

            # run in separate thread since Matplotlib requires the main thread
            t = threading.Thread(target=process_commands, daemon=True)
            t.start()
            
            app = QApplication(sys.argv)
            mainWin = RealTimePlotWindow(top_buffer, bottom_buffer, [(255, 0, 0), (0, 255, 0)])
            mainWin.show()
            sys.exit(app.exec_())
        finally:
            lidar_bottom.disconnect()
            lidar_top.disconnect()



