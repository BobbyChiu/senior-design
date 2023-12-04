import PyQt5
import pyqtgraph as pg
import pyqtgraph.opengl
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QLabel, QGridLayout
from PyQt5.QtGui import QImage, QPixmap, QVector3D
from PyQt5.QtWebEngineWidgets import QWebEngineView
import plotly.io as pio
from PyQt5.QtCore import pyqtSignal, QTimer, QObject, Qt
import sys
import plotly
import plotly.graph_objs as go
import open3d as o3d
import numpy as np
from pyqtgraph.Qt import QtCore, QtWidgets
import pyqtgraph.opengl as gl

plotter = None

def plot_dual_3d_clouds(points_3d_1, points_3d_2, color_1, color_2):
    # First point cloud
    x1 = points_3d_1[:,0]
    y1 = points_3d_1[:,1]
    z1 = points_3d_1[:,2]
    trace1 = go.Scatter3d(x=x1, y=y1, z=z1,
                          mode='markers',
                          marker={'size': 2, 'color': color_1, 'opacity': 0.8})

    # Second point cloud
    x2 = points_3d_2[:,0]
    y2 = points_3d_2[:,1]
    z2 = points_3d_2[:,2]
    trace2 = go.Scatter3d(x=x2, y=y2, z=z2,
                          mode='markers',
                          marker={'size': 2, 'color': color_2, 'opacity': 0.8})

    # Configure the layout.
    layout = go.Layout(margin={'l': 0, 'r': 0, 'b': 0, 't': 0})
    data = [trace1, trace2]
    plot_figure = go.Figure(data=data, layout=layout)
    plot_figure.update_scenes(aspectmode='data')
    
    # Render the plot
    plotly.offline.iplot(plot_figure)


# plot 3d points
def plot3d(points_3d, plot_num=1):
    global plotter
    if plotter is not None:
        plotter.set_plot_3d(points_3d, plot_num)
    else:
        x = points_3d[:,0]
        y = points_3d[:,1]
        z = points_3d[:,2]

        trace = go.Scatter3d(x=x,y=y,z=z,
            mode='markers',
            marker={
                'size': 1,
                'opacity': 0.8,
            }
        )
        # Configure the layout.
        layout = go.Layout(
            margin={'l': 0, 'r': 0, 'b': 0, 't': 0}
        )
        data = [trace]
        plot_figure = go.Figure(data=data, layout=layout)
        plot_figure.update_scenes(aspectmode='data')
        # Render the plot
        plotly.offline.iplot(plot_figure)

def convert_color(color):
    """ Convert an RGB color to RGBA with 0-1 range """
    return (color[0] / 255, color[1] / 255, color[2] / 255, 0.2)  # Assuming opaque color

class RealTimePlotWindow(QMainWindow):

    update_plot_signal = pyqtSignal(int, np.ndarray)

    def __init__(self, buffers1, buffers2, colors):
        super().__init__()

        global plotter
        plotter = self
        super().__init__()
        self.buffers1 = buffers1
        self.buffers2 = buffers2
        self.colors = colors
        self.pc1_top = np.empty((0,3))
        self.pc1_bottom = np.empty((0,3))
        self.pc2 = np.empty((0,3))
        self.mesh = None
        self.initUI()

    def initUI(self):
        self.setWindowTitle('Real-time Plots and Mesh Visualization')
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        layout = QtWidgets.QGridLayout()
        central_widget.setLayout(layout)

        # 2D plot
        self.plot2d = pg.PlotWidget(background=(61, 54, 53))  # Dark background
        self.scatter2d_1 = pg.ScatterPlotItem(pen=None, brush=pg.mkBrush(*self.colors[0]))
        self.scatter2d_2 = pg.ScatterPlotItem(pen=None, brush=pg.mkBrush(*self.colors[1]))
        self.plot2d.addItem(self.scatter2d_1)
        self.plot2d.addItem(self.scatter2d_2)
        layout.addWidget(self.plot2d, 0, 0)

        # First 3D scatter plot
        self.view3d_1 = gl.GLViewWidget()
        self.view3d_1.setBackgroundColor((61, 54, 53))  # Dark background

        # set camera
        self.view3d_1.opts['center'] = QVector3D(0, 0, 5)  # Center point of the view
        self.view3d_1.opts['distance'] = 30  # Distance of the camera from the center point
        self.view3d_1.opts['elevation'] = 0  # Elevation angle in degrees
        self.view3d_1.opts['azimuth'] = 0   # Azimuth angle in degrees

        # Create the first set of points
        self.scatter3d_1 = gl.GLScatterPlotItem()
        self.view3d_1.addItem(self.scatter3d_1)
        # Create the second set of points
        self.scatter3d_2 = gl.GLScatterPlotItem()
        self.view3d_1.addItem(self.scatter3d_2)
        grid1 = gl.GLGridItem()
        self.view3d_1.addItem(grid1)
        layout.addWidget(self.view3d_1, 0, 1)

        # Second 3D scatter plot
        self.view3d_2 = gl.GLViewWidget()
        self.view3d_2.setBackgroundColor((61, 54, 53))  # Dark background
        self.scatter3d_2_1 = gl.GLScatterPlotItem()
        self.view3d_2.addItem(self.scatter3d_2_1)
        # Create the second set of points
        grid2 = gl.GLGridItem()
        self.view3d_2.addItem(grid2)
        layout.addWidget(self.view3d_2, 1, 1)

        # QLabel for Open3D mesh image
        self.mesh_label = QtWidgets.QLabel()
        layout.addWidget(self.mesh_label, 1, 0)

        layout.setColumnStretch(0, 1)  # First column
        layout.setColumnStretch(1, 1)  # Second column
        layout.setRowStretch(0, 1)     # First row
        layout.setRowStretch(1, 1)     # Second row

        # Timer for real-time plot updates
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(100)  # Update interval in milliseconds

    def update_plots(self):

       # Update 2D plot with data from both buffers
        if not self.buffers1['2d'].empty():
            data2d = self.buffers1['2d'].get()
            self.scatter2d_1.setData(data2d[:, 0], data2d[:, 1])

        if not self.buffers2['2d'].empty():
            data2d = self.buffers2['2d'].get()
            self.scatter2d_2.setData(data2d[:, 0], data2d[:, 1])

        # Update first 3D scatter plot with data from both buffers
        maxPointSize = 0.1
        minPointSize = 0.01

        # if self.pc1_top.size == 0:
            # self.scatter3d_1.setData(pos=np.array([]), color=convert_color(self.colors[1]), size=1, pxMode=False)

        if not self.buffers1['3d'].empty():
            data3d = self.buffers1['3d'].get()
            self.pc1_top = np.vstack((self.pc1_top, data3d))
            z = self.pc1_top[:, 2]  # Assuming z-axis depth
            sizes = (z - z.min()) / (z.max() - z.min()) * maxPointSize + minPointSize
            self.scatter3d_1.setData(pos=self.pc1_top, color=convert_color(self.colors[0]), size=sizes, pxMode=False)

        if not self.buffers2['3d'].empty():
            data3d = self.buffers2['3d'].get()
            self.pc1_bottom = np.vstack((self.pc1_bottom, data3d))
            z = self.pc1_bottom[:, 2]  # Assuming z-axis depth
            sizes = (z - z.min()) / (z.max() - z.min()) * maxPointSize + minPointSize
            self.scatter3d_2.setData(pos=self.pc1_bottom, color=convert_color(self.colors[1]), size=sizes, pxMode=False)

        # Update second 3D scatter plot with data from both buffers
        if self.pc2.size > 0:
            z = self.pc2[:, 2]  # Assuming z-axis depth
            sizes = (z - z.min()) / (z.max() - z.min()) * maxPointSize + minPointSize
            self.scatter3d_2_1.setData(pos=self.pc2, color=convert_color((155, 155, 155)), size=sizes, pxMode=False)
        # Update Open3D mesh image
        self.update_mesh_image(self.mesh)

    def set_plot_3d(self, data, plot_num=1):
        """
        Set or update the data for the specified 3D plot.

        Parameters:
        data (np.ndarray): Data to set in the plot (Nx3 array for x, y, z coordinates).
        plot_num (int): Which plot to update (1 or 2).
        """
        if plot_num==1:
            self.pc1_bottom = data
            self.pc1_top = np.empty((0,3))

        elif plot_num==2:
            self.pc2 = data

    def clear_plot_3d(self, plot_num=1):
        if plot_num == 1:
            # Clear the first 3D scatter plot
            self.scatter3d_1.setData(pos=np.array([]))  # Set empty data to clear the plot
        elif plot_num == 2:
            # Clear the second 3D scatter plot
            self.scatter3d_2_1.setData(pos=np.array([]))  # Set empty data to clear the plot
        else:
            print("Invalid plot number. Please use 1 or 2.")

    def update_mesh_image(self, mesh):
        if mesh is None:
            return
         # Set up offscreen rendering in Open3D
        vis = o3d.visualization.Visualizer()
        vis.create_window(visible=False)  # Create an offscreen window
        vis.add_geometry(mesh)            # Add your mesh

        # Set up offscreen rendering in Open3D
        vis = o3d.visualization.Visualizer()
        vis.create_window(visible=False)  # Create an offscreen window
        vis.add_geometry(mesh)            # Add your mesh

        # Adjust the view and render the image
        vis.update_geometry(mesh)
        vis.poll_events()
        vis.update_renderer()

        ctr = vis.get_view_control()
        ctr.set_zoom(0.5)  # Adjust zoom level
        ctr.set_front([0, 0, 1])  # Set the front direction as a vector
        ctr.set_lookat([0, 0, 5])  # Set the point at which the camera is looking
        ctr.set_up([0, 1, 0])  # Set the up direction as a vector

        # Capture the image
        image = vis.capture_screen_float_buffer(do_render=True)
        vis.destroy_window()

        # Convert Open3D Image to a format suitable for PyQt QLabel
        image = np.asarray(image)
        image = (image * 255).astype(np.uint8)  # Convert to 8-bit channel
        image = np.flipud(image)                # Flip the image vertically
        height, width, channel = image.shape
        image = np.require(image, np.uint8, 'C')
        qimage = QImage(image, width, height, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimage)

        scaled_pixmap = pixmap.scaled(self.mesh_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.mesh_label.setPixmap(scaled_pixmap)
        self.mesh_label.setPixmap(scaled_pixmap)
        self.mesh_label.setAlignment(Qt.AlignCenter)  # Center the image


def plot2d_realtime(buffers, colors, interval=25, xlim=(0, 80), ylim=(-40, 40), bias=0):
    app = QApplication([])
    win = pg.GraphicsLayoutWidget(show=True)
    win.setBackground((50, 50, 50))
    plot2d = win.addPlot()
    plot2d.setXRange(*xlim)
    plot2d.setYRange(*ylim)

    scatter_plots = [pg.ScatterPlotItem(pen=pg.mkPen(None), brush=pg.mkBrush(color)) for color in colors]

    for scatter_plot in scatter_plots:
        plot2d.addItem(scatter_plot)

    def update():
        # Update 2D plot
        for scatter_plot, buffer in zip(scatter_plots, buffers):
            if not buffer.empty():
                data = buffer.get()
                scatter_plot.setData(data[:, 0], data[:, 1])

        # # Update 3D plot (modify as needed for your 3D data)
        # if not buffer.empty():
        #     data_3d = buffer.get()  # Assuming 3D data is also in the buffer
        #     fig_3d.data[0].x = data_3d[:, 0]
        #     fig_3d.data[0].y = data_3d[:, 1]
        #     fig_3d.data[0].z = data_3d[:, 2]

    timer = QTimer()
    timer.timeout.connect(update)
    timer.start(interval)

    if (sys.flags.interactive != 1) or not hasattr(pg.QtCore, 'PYQT_VERSION'):
        QApplication.instance().exec_()

    return win
    
def showMesh(mesh):
    o3d.visualization.draw_geometries([mesh], mesh_show_wireframe=True, mesh_show_back_face=True)