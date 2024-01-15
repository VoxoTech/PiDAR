import numpy as np
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
        self.ani = self.__initialize_animation__()
        self.pause = pause

    def __initialize_animation__(self):
        return animation.FuncAnimation(self.fig, self.animate, init_func=self.init, frames=1, interval=1, blit=True)

    def __gray2rgb__(self, luminances):
        return np.column_stack((luminances, luminances, luminances))
    
    def init(self):
        return self.ax,

    def animate(self, i):
        line = getattr(self, 'line', None)
        if line is not None:
            line.remove()

        self.line = self.ax.scatter(self.x_list, self.y_list, c=self.color_list/255, s=1)
        return self.line,

    def update_lists(self, x_list, y_list, luminance_list):
        self.x_list = np.asarray(x_list)
        self.y_list = np.asarray(y_list)
        self.color_list = self.__gray2rgb__(np.asarray(luminance_list))
        plt.pause(self.pause)

    def update_coordinates(self, points_2d):  # points_2d: np.array([[luminance, x, y,], ...])
        self.x_list = points_2d[:, 0]
        self.y_list = points_2d[:, 1]
        self.color_list = self.__gray2rgb__(points_2d[:, 2])
        plt.pause(self.pause)

    def close(self):
        plt.close()


###################################################
## OBSOLETE!

# from scipy.spatial.transform import Rotation

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
