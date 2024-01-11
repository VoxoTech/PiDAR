from lib.file_utils import merge_csv_dir
from lib.open3d_utils import plot_3D  # rotate_3D
# from lib.matplotlib_utils import plot_3D, rotate_3D


csv_dir = "cpython/data"
pointcloud = merge_csv_dir(csv_dir, angle_step=1)

plot_3D(pointcloud)
