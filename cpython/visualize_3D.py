from lib.file_utils import merge_data
from lib.open3d_utils import plot_3D
# from lib.matplotlib_utils import plot_3D


path = "cpython/data"
pointcloud = merge_data(path, angle_step=1, format='npy')

plot_3D(pointcloud)
