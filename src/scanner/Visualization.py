import pyqtgraph as pg
from PyQt5.QtWidgets import QMainWindow, QInputDialog
from PyQt5.QtGui import QVector3D
from PyQt5.QtCore import pyqtSignal
import plotly
import plotly.graph_objs as go
import open3d as o3d
import numpy as np
from pyqtgraph.Qt import QtCore, QtWidgets
import pyqtgraph.opengl as gl
import vtk
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
import time
vtk.vtkObject.GlobalWarningDisplayOff()

plotter = None
def open3d_mesh_to_vtk_polydata(open3d_mesh):
    # Convert Open3D mesh to VTK polydata

    # Extract vertices and faces from Open3D mesh
    vertices = np.asarray(open3d_mesh.vertices)
    faces = np.asarray(open3d_mesh.triangles)

    # Create a VTK points array
    points = vtk.vtkPoints()
    for v in vertices:
        points.InsertNextPoint(v)

    # Create a VTK cell array for triangles
    triangles = vtk.vtkCellArray()
    for f in faces:
        triangle = vtk.vtkTriangle()
        triangle.GetPointIds().SetId(0, f[0])
        triangle.GetPointIds().SetId(1, f[1])
        triangle.GetPointIds().SetId(2, f[2])
        triangles.InsertNextCell(triangle)

    # Create a polydata object
    polydata = vtk.vtkPolyData()
    polydata.SetPoints(points)
    polydata.SetPolys(triangles)

    return polydata

def plot_dual_3d_clouds(points_3d_1, points_3d_2, color_1='red', color_2='green'):
    global plotter
    if plotter is not None:
        plotter.clear_plot_3d(1)
        plotter.set_plot_3d(points_3d_1, points_3d_2, 1)
    else:
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
        plotter.clear_plot_3d(plot_num)
        plotter.set_plot_3d(points_3d, plot_num=plot_num)
        pass
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

def showMesh(mesh):
    if plotter is not None:
        plotter.update_mesh(mesh)
    else:
        o3d.visualization.draw_geometries([mesh], mesh_show_wireframe=True, mesh_show_back_face=True)

# To fix the issue of the plot not updating, we need to create a custom GLViewWidget
class CustomGLViewWidget(gl.GLViewWidget):
    def mousePressEvent(self, event):
        super().mousePressEvent(event)
        self.setFocus()

class RealTimePlotWindow(QMainWindow):
    input_signal = pyqtSignal()

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
        self.need_update_mesh = False
        self.input_signal.connect(self._get_input)
        self.initUI()

    def get_input(self):
        self.input_value = None
        self.input_signal.emit()

        while self.input_value is None:
            time.sleep(0.1)

        return self.input_value
            
    def _get_input(self):
        value, ok = QInputDialog.getInt(self, "Input Dialog", "Enter a value:")
        if ok:
            # Emit the inputted value
            self.input_value = value
        else:
            self.input_value = "No value entered"

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

        # Setup the VTK widget for 3D visualization
        self.vtkWidget = QVTKRenderWindowInteractor(central_widget)

        # Make sure VTK widget releases focus when unselected
        self.vtkWidget.mouseReleaseEvent = self.on_vtkWidget_mouseRelease

        layout.addWidget(self.vtkWidget, 1, 0)

        # Setup VTK renderer
        self.renderer = vtk.vtkRenderer()
        self.vtkWidget.GetRenderWindow().AddRenderer(self.renderer)
        self.renderer.SetBackground(61/255, 54/255, 53/255)

        # First 3D scatter plot
        self.view3d_1 = CustomGLViewWidget()
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
        self.view3d_2 = CustomGLViewWidget()
        self.view3d_2.setBackgroundColor((61, 54, 53))  # Dark background
        self.scatter3d_2_1 = gl.GLScatterPlotItem()
        self.view3d_2.addItem(self.scatter3d_2_1)
        # Create the second set of points
        grid2 = gl.GLGridItem()
        self.view3d_2.addItem(grid2)
        layout.addWidget(self.view3d_2, 1, 1)

        layout.setColumnStretch(0, 1)  # First column
        layout.setColumnStretch(1, 1)  # Second column
        layout.setRowStretch(0, 1)     # First row
        layout.setRowStretch(1, 1)     # Second row

        # Timer for real-time plot updates
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(50)  # Update interval in milliseconds

    def on_vtkWidget_mouseRelease(self, event):
        # Call the original mouseReleaseEvent function
        QVTKRenderWindowInteractor.mouseReleaseEvent(self.vtkWidget, event)

        # Fixes strange bug where the 3D plots lose focus until the user clicks on the 2d plot
        self.plot2d.setFocus()

    def update_plots(self):
        """ Update the plots with new data"""
        def convert_color(color):
            """ Convert an RGB color to RGBA with 0-1 range """
            return (color[0] / 255, color[1] / 255, color[2] / 255, 1)

       # Update 2D plot with data from both buffers
        if not self.buffers1['2d'].empty() :
            data2d = self.buffers1['2d'].get()
            self.scatter2d_1.setData(data2d[:, 0], data2d[:, 1])

        if not self.buffers2['2d'].empty():
            data2d = self.buffers2['2d'].get()
            self.scatter2d_2.setData(data2d[:, 0], data2d[:, 1])

        # Update first 3D scatter plot with bottom data
        if not self.buffers1['3d'].empty():
            data3d = self.buffers1['3d'].get()
            self.pc1_top = np.vstack((self.pc1_top, data3d))
        self.scatter3d_1.setData(pos=self.pc1_top, 
                                 color=convert_color(self.colors[0]), 
                                 size=0.05, 
                                 pxMode=False)

        # Update first 3D scatter plot with top data
        if not self.buffers2['3d'].empty():
            data3d = self.buffers2['3d'].get()
            self.pc1_bottom = np.vstack((self.pc1_bottom, data3d))
        self.scatter3d_2.setData(pos=self.pc1_bottom, 
                                 color=convert_color(self.colors[1]), 
                                 size=0.05, 
                                 pxMode=False)

        # Update second 3D scatter plot
        self.scatter3d_2_1.setData(pos=self.pc2, 
                                   color=convert_color((155, 155, 155)), 
                                   size=0.05, 
                                   pxMode=False)
        
        # Update Mesh
        if self.need_update_mesh:
            # If an actor has already been added, remove it
            if hasattr(self, 'actor'):
                self.renderer.RemoveActor(self.actor)

            # Setup VTK mapper and actor
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputData(self.mesh)

            self.actor = vtk.vtkActor()
            self.actor.SetMapper(mapper)

            # Add actor to the renderer
            self.renderer.AddActor(self.actor)
            self.renderer.ResetCamera()
            self.need_update_mesh = False

        # Render the scene, even if no mesh is present
        self.vtkWidget.GetRenderWindow().Render()

    def set_plot_3d(self, data_bottom, data_top=np.empty((0, 3)), plot_num=1):
        """
        Set or update the data for the specified 3D plot.

        Parameters:
        data (np.ndarray): Data to set in the plot (Nx3 array for x, y, z coordinates).
        plot_num (int): Which plot to update (1 or 2).
        """
        if plot_num==1:
            self.pc1_bottom = data_bottom
            self.pc1_top = data_top
        elif plot_num==2:
            self.pc2 = data_bottom

    def clear_plot_3d(self, plot_num=1):
        if plot_num == 1:
            # Clear the first 3D scatter plot
            self.pc1_top = np.empty((0, 3)) 
            self.pc1_bottom = np.empty((0, 3))
        elif plot_num == 2:
            self.pc2 = np.empty((0, 3))  
        else:
            print("Invalid plot number. Please use 1 or 2.")
    
    def update_mesh(self, open3d_mesh):
        # Convert Open3D mesh to VTK polydata
        vtk_polydata = open3d_mesh_to_vtk_polydata(open3d_mesh)
        self.mesh = vtk_polydata
        self.need_update_mesh = True