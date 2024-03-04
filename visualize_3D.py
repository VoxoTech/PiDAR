from lib.pointcloud import merge_data, export_pointcloud
from lib.file_utils import list_files
from lib.visualization import opengl_fallback, visualize

opengl_fallback()

filepaths = list_files("data", type='npy')  # "C:/DOKUMENTE/Projekte/python/PiDAR/data"
pcd = merge_data(filepaths, angle_step=0.48464451, offset=(0, -0.374, -0.102), up_axis="Z", columns="XZI", as_pcd=True)

visualize([pcd], view="front")
export_pointcloud([pcd], "export/3d-scan", type="e57")
