import plotly
import plotly.graph_objs as go
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
import open3d as o3d

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

def plot2d_realtime(buffer_xy, type='vertical_slice', interval=50, xlim=(0, 50), ylim=(-25,25), callback=None):
    # plot
    fig, ax = plt.subplots()
    sc = ax.scatter([], [])

    ax.set_xlim(xlim)
    ax.set_ylim(ylim)

    def consumerThread(frame):
        try:
            data_time_t = buffer_xy.get_nowait()
            x = data_time_t[:,0]
            y = data_time_t[:,1]
            
            sc.set_offsets(list(zip(x, y)))
        except Exception as e:
            raise(e)
        finally:
            return sc
    
    ani = FuncAnimation(fig, 
                        consumerThread,
                        interval=interval)
    plt.show() # loops until q is pressed

    if callback != None:
        callback()
    
def showMesh(mesh):
    o3d.visualization.draw_geometries([mesh], mesh_show_wireframe=True, mesh_show_back_face=True)