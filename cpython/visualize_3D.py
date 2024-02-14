from lib.file_utils import merge_data
from lib.pointcloud import pcd_from_np
from lib.visualization import opengl_fallback, visualize


opengl_fallback()

path = "data"
pointcloud = merge_data(path, angle_step=0.3, format='npy')
pcd = pcd_from_np(pointcloud)
visualize([pcd], unlit=True)
