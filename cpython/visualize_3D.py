from lib.file_utils import merge_data
from lib.pointcloud import pcd_from_np
from lib.visualization import opengl_fallback, visualize
from lib.pointcloud import export_pointcloud


opengl_fallback()

path = "data"
pointcloud_array = merge_data(path, angle_step=0.25, format='npy')
pcd = pcd_from_np(pointcloud_array)

export_pointcloud([pcd], "export/3d-scan", type="e57")
visualize([pcd], unlit=True)
