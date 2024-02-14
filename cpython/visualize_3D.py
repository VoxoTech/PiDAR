from lib.file_utils import merge_data
from lib.pointcloud import pcd_from_np
from lib.visualization import opengl_fallback, visualize
from lib.pointcloud import export_pointcloud
# from lib.transformation import transform


opengl_fallback()

path = "data"
pointcloud = merge_data(path, angle_step=0.3, format='npy')
pcd = pcd_from_np(pointcloud)

export_pointcloud([pcd], "export/3d-scan", type="e57")
visualize([pcd], unlit=True)
