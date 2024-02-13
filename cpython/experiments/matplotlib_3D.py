import numpy as np
import matplotlib.pyplot as plt


def plot_3D(pointcloud):
    xs = pointcloud[:, 0]
    ys = pointcloud[:, 1]
    zs = pointcloud[:, 2]
    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(projection='3d')
    ax.set_box_aspect((np.ptp(xs), np.ptp(ys), np.ptp(zs)))
    img = ax.scatter(xs, ys, zs, s=1)  # , c=t_low, cmap=plt.hot())
    fig.colorbar(img)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()
