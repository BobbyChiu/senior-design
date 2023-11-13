import plotly
import plotly.graph_objs as go
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
import open3d as o3d
import numpy as np

# plot 3d points
def plot3d(points_3d):
    x = points_3d[:,0]
    y = points_3d[:,1]
    z = points_3d[:,2]

    trace = go.Scatter3d(x=x,y=y,z=z,
        mode='markers',
        marker={
            'size': 3,
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

def plot2d_realtime(buffers, colors, interval=50, xlim=(0, 50), ylim=(-25,25)): 
    # plot
    fig, ax = plt.subplots()
    sc = ax.scatter([], [])

    ax.set_xlim(xlim)
    ax.set_ylim(ylim)

    last_buffer_data = np.full(len(buffers), None)
    def consumerThread(frame):
        # update buffer data
        for i, (buffer, color) in enumerate(zip(buffers, colors)):
            try:
                new_data = buffer.get_nowait()
                last_buffer_data[i] = (new_data[:,0], 
                                       new_data[:,1], 
                                       np.full(new_data.shape[0], color))
            except Exception as e:
                pass
        # concatenate data from all buffers
        x, y, c = ([], [], [])
        for buffer_data in last_buffer_data:
            if buffer_data is None:
                continue
            x = np.append(x, buffer_data[0])
            y = np.append(y, buffer_data[1])
            c = np.append(c, buffer_data[2])
        # update plot
        try:
            sc.set_offsets(list(zip(x, y)))
            sc.set_color(c)
        finally:
            return sc
    
    ani = FuncAnimation(fig, 
                        consumerThread,
                        interval=interval)
    plt.show() # loops until q is pressed
    
def showMesh(mesh):
    o3d.visualization.draw_geometries([mesh], mesh_show_wireframe=True, mesh_show_back_face=True)