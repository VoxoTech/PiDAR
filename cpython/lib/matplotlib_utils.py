import numpy as np
from scipy.spatial.transform import Rotation

import matplotlib.pyplot as plt
import matplotlib.animation as animation


class plot_2D:
    def __init__(self, pause=0.005):
        self.fig = plt.figure(figsize=(8, 8))
        self.ax = self.fig.add_subplot(111)
        self.plotrange = 20
        self.ax.set_ylim([-self.plotrange, self.plotrange])
        self.ax.set_xlim([-self.plotrange, self.plotrange])
        self.ax.set_title('LiDAR LD06', fontsize=18)
        self.ax.set_facecolor('darkblue')
        self.ax.xaxis.grid(True, color='darkgray', linestyle='dashed')
        self.ax.yaxis.grid(True, color='darkgray', linestyle='dashed')
        self.ani = self.init_visualisation()
        self.pause = pause
    
    def init_visualisation(self):
        def init():
            return self.ax,

        # convert grayscale value to RGB-array
        def color_array(luminance):
            a = np.array(luminance)
            return np.column_stack((a, a, a))

        def animate(i):
            if hasattr(self, 'line'):
                self.line.remove()

            rgb = color_array(self.luminance_list)
            self.line = self.ax.scatter(self.x_list, self.y_list, c=rgb/255, s=1)
            return self.line,

        self.ani = animation.FuncAnimation(self.fig, animate, init_func=init, frames=1, interval=1, blit=True)
        return self.ani
    
    def update_data(self, x_list, y_list, luminance_list):
        self.x_list = x_list
        self.y_list = y_list
        self.luminance_list = luminance_list
        
        # pause to update plot
        plt.pause(self.pause)


# def plot_3D(pointcloud):
#     xs = pointcloud[:, 0]
#     ys = pointcloud[:, 1]
#     zs = pointcloud[:, 2]
#     fig = plt.figure(figsize=(10, 6))
#     ax = fig.add_subplot(projection='3d')
#     ax.set_box_aspect((np.ptp(xs), np.ptp(ys), np.ptp(zs)))
#     img = ax.scatter(xs, ys, zs, s=1)  # , c=t_low, cmap=plt.hot())
#     fig.colorbar(img)
#     ax.set_xlabel('X')
#     ax.set_ylabel('Y')
#     ax.set_zlabel('Z')
#     plt.show()


# def rotate_3D(points3d, rotation_axis, rotation_degrees):
#     # define 3D rotation
#     rotation_radians = np.radians(rotation_degrees)
#     rotation_vector = rotation_radians * rotation_axis
#     rotation = Rotation.from_rotvec(rotation_vector)

#     result_points = points3d.copy()

#     if points3d.shape[1] == 4:
#         # remove intensity column
#         points3d = np.delete(points3d, -1, axis=1)
    
#     # apply rotation to each point
#     for i, point in enumerate(points3d):
#         result_points[i][0:3] = rotation.apply(point)

#     return result_points
