from lib.pointcloud import merge_data, export_pointcloud
from lib.file_utils import list_files
from lib.visualization import opengl_fallback, visualize

opengl_fallback()

filepaths = list_files("data", type='npy')
pcd = merge_data(filepaths, angle_step=0.48464451, offset=(0, 2.5, -4), up_axis="Z", columns="XZI", as_pcd=True)

export_pointcloud([pcd], "export/3d-scan", type="e57")
visualize([pcd], unlit=True)
