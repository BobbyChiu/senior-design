from rplidar import RPLidar
import numpy as np
import os
import queue
from datetime import datetime
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import time
import plotly
import plotly.graph_objs as go
import pyvista as pv


# converts 2d polar (in degrees) to 2d cartesian
def pol2cart(radius, angle):
    x = radius * np.cos(np.radians(angle))
    y = radius * np.sin(np.radians(angle))
    return (x, y)


# takes as input 2d lidar_scan and outputs x, y for 360 degree top-down view
def plot_data_cart(lidar_scan):
    dist = lidar_scan[:, 0]
    angle = lidar_scan[:, 1]

    return pol2cart(dist, angle)


# takes as input 2d lidar_scan and outputs x, y for top-down view
def plot_data_vert_slice(lidar_scan):
    x, y = plot_data_cart(lidar_scan)
    return (y, x)


def lidar_filter_scan(data, max_dist, min_angle, max_angle):
    data = data[data[:, 0] < max_dist]
    data = data[data[:, 1] > min_angle]
    data = data[data[:, 1] < max_angle]
    return data


# convert lidar scan data (dist, angle from z axis, time) to x, y, z
# angular speed in degrees per second
def lidar_to_3d(scan, angular_speed=30, dist_from_axis=30):
    # convert lidar dist, lidar angle to x, y in lidar plane
    lidar_dist = scan[:, 0]
    lidar_angle = scan[:, 1]
    lidar_scan_time = scan[:, 2]

    y, x = pol2cart(lidar_dist, lidar_angle)

    # shift coordinate space so that (0, 0) is on the axis of rotation
    r = dist_from_axis - x
    theta = -angular_speed * lidar_scan_time
    z = y

    # convert cylindrical coordinates to cartesian
    x, y = pol2cart(r, theta)

    return (x, y, z)


# plot 3d points
def plot3d(x, y, z):
    trace = go.Scatter3d(
        x=x,
        y=y,
        z=z,
        mode="markers",
        marker={
            "size": 5,
            "opacity": 0.8,
        },
    )
    # Configure the layout.
    layout = go.Layout(margin={"l": 0, "r": 0, "b": 0, "t": 0})
    data = [trace]
    plot_figure = go.Figure(data=data, layout=layout)
    plot_figure.update_scenes(aspectmode="data")
    # Render the plot
    plotly.offline.iplot(plot_figure)


def producerThread(lidar, lidar_plot_queue, lidar_cumulative_scan, should_filter=True):
    start_time = time.time()
    for i, scan in enumerate(lidar.iter_scans()):
        # add data to plotting queue
        scan = np.array(scan)
        scan = scan[:, [2, 1]]  # get data as [dist, angle]
        scan[:, 0] = scan[:, 0] / 10
        if should_filter:
            scan = lidar_filter_scan(scan, max_dist=60, min_angle=30, max_angle=150)

        lidar_plot_queue.put(scan)

        # update cumulative scanning data
        scan_time = time.time() - start_time
        print(scan_time)
        if scan_time > 10 and lidar_cumulative_scan != None:
            scan_with_time = np.column_stack(
                (scan, np.full(scan[:, 0].shape, scan_time - 10))
            )  # add third coordinate: time
            lidar_cumulative_scan[0] = np.vstack(
                (lidar_cumulative_scan[0], scan_with_time)
            )


def consumerThread(frame, sc, lidar_queue, preprocessor):
    try:
        lidar_scan = lidar_queue.get_nowait()
        x, y = preprocessor(lidar_scan)

        # print(f"x: {x}")

        sc.set_offsets(list(zip(x, y)))
    except Exception as e:
        raise (e)
    finally:
        return sc


# for plot_type_2d can use either 'vertical_slice' or 'top_down'
def do_lidar(save_data=False, show_3d_plot=False, view=plot_data_vert_slice):
    lidar = RPLidar("com3")

    if not save_data and not show_3d_plot:
        output_buffer = None
    else:
        output_buffer = [np.empty((0, 3))]

    # plot
    fig, ax = plt.subplots()
    sc = ax.scatter([], [])
    lidar_plotting_queue = queue.Queue()  # queue for plotting

    # set axis limits
    if view == "top_down":
        plot_preprocessor = plot_data_cart
        should_filter = False
        ax.set_xlim([-100, 100])
        ax.set_ylim([-100, 100])
    elif view == "vertical_slice":
        plot_preprocessor = plot_data_vert_slice
        should_filter = True
        ax.set_xlim([0, 50])
        ax.set_ylim([-25, 25])
    else:
        raise ("Invalid view: acceptable values are 'vertical_slice', 'top_down'")

    # start producer thread
    producer = threading.Thread(
        target=producerThread,
        args=(lidar, lidar_plotting_queue, output_buffer, should_filter),
        daemon=True,
    )
    producer.start()

    # start consumer thread
    ani = FuncAnimation(
        fig,
        consumerThread,
        fargs=(sc, lidar_plotting_queue, plot_preprocessor),
        interval=100,
    )
    plt.show()  # loops until q is pressed

    # when consumer thread dies...
    # stop lidar
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()

    # # save lidar scan
    if save_data:
        filename = "lidar-data/" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv"
        np.savetxt(filename, output_buffer[0], delimiter=",", fmt="%f")

    # plot all scanned points
    # lidar_cumulative_scan = [np.empty((0, 3))] # all lidar data collected so far
    # lidar_cumulative_scan[0] = np.loadtxt('lidar-data/20230924_145028.csv', delimiter=",")
    if show_3d_plot:
        x, y, z = lidar_to_3d(output_buffer[0])
        plot3d(x, y, z)


if __name__ == "__main__":
    # do_lidar(save_data=True, show_3d_plot=True, view='vertical_slice')

    # show_3d_plot from 2d data file
    # lidar_cumulative_scan = [np.empty((0, 3))] # all lidar data collected so far
    # lidar_cumulative_scan[0] = np.loadtxt('lidar-data/sandia4.csv', delimiter=",")
    # x, y, z = lidar_to_3d(lidar_cumulative_scan[0], angular_speed=33.75, dist_from_axis=42.5)
    # plot3d(x, y, z)

    # cube is 33.5, 31.5
    # sandia is 33.75, 42.5

    # # show 3d plot from 3d data file
    # lidar_cumulative_scan = [np.empty((0, 3))] # all lidar data collected so far
    # lidar_cumulative_scan[0] = np.loadtxt('../puflow/input/sandia.xyz')
    # x, y, z = lidar_cumulative_scan[0][:,0],  lidar_cumulative_scan[0][:,1],  lidar_cumulative_scan[0][:,2]
    # plot3d(x, y, z)

    # # # # show 3d plot from 3d data file
    # lidar_cumulative_scan = [np.empty((0, 3))] # all lidar data collected so far
    # lidar_cumulative_scan[0] = np.loadtxt('../puflow/output/sandia.xyz')
    # x, y, z = lidar_cumulative_scan[0][:,0],  lidar_cumulative_scan[0][:,1],  lidar_cumulative_scan[0][:,2]
    # plot3d(x, y, z)

    # # # convert 2d data file to 3d data file
    # lidar_cumulative_scan = [np.empty((0, 3))] # all lidar data collected so far
    # lidar_cumulative_scan[0] = np.loadtxt('lidar-data/cube.csv', delimiter=",")
    # x, y, z = lidar_to_3d(lidar_cumulative_scan[0], angular_speed=33.5, dist_from_axis=31.5)
    # np.savetxt('lidar-data/cube.xyz', np.column_stack((x, y, z)), fmt="%f")

    # Print contents of the current directory
    print(os.listdir("."))
    # mesh
    points = np.loadtxt("./lidar-data/update-gearge.xyz")
    cloud = pv.PolyData(points)
    mesh = cloud.delaunay_2d(tol=0.01)
    # Plot the mesh
    plotter = pv.Plotter()
    # Add the mesh to plotter
    mesh = mesh.extract_geometry()
    # smooth the data
    mesh = mesh.smooth(n_iter=100)
    # show scalar bar with z position coloring
    plotter.add_mesh(
        mesh,
        color="tan",
        # scalars=mesh.points[:, 2],
        show_edges=True,
        # scalar_bar_args={"title": "Z Position"},
    )
    plotter.show()
