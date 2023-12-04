import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer
import sys
import plotly
import plotly.graph_objs as go
import open3d as o3d
import numpy as np

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
def plot3d(points_3d):
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

def plot2d_realtime(buffers, colors, interval=25, xlim=(0, 80), ylim=(-40, 40), bias=0):
    app = QApplication([])
    win = pg.GraphicsLayoutWidget(show=True)
    win.setBackground((50, 50, 50))
    plot = win.addPlot()
    plot.setXRange(*xlim)
    plot.setYRange(*ylim)

    scatter_plots = [pg.ScatterPlotItem(pen=pg.mkPen(None), brush=pg.mkBrush(color)) for color in colors]

    for scatter_plot in scatter_plots:
        plot.addItem(scatter_plot)

    def update():
        for scatter_plot, buffer in zip(scatter_plots, buffers):
            if not buffer.empty():
                data = buffer.get()
                scatter_plot.setData(data[:, 0], data[:, 1])

    timer = QTimer()
    timer.timeout.connect(update)
    timer.start(interval)

    if (sys.flags.interactive != 1) or not hasattr(pg.QtCore, 'PYQT_VERSION'):
        QApplication.instance().exec_()
    
def showMesh(mesh):
    o3d.visualization.draw_geometries([mesh], mesh_show_wireframe=True, mesh_show_back_face=True)