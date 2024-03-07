from os import path
import cv2
import open3d as o3d
import numpy as np

from lib.pointcloud import merge_2D_points, pcd_from_np, transform, angular_lookup, angular_from_cartesian, export_pointcloud_threaded # colormap_pcd
from lib.file_utils import angles_from_filenames
from lib.visualization import opengl_fallback, visualize


opengl_fallback()

data_dir = "data"
scan_id = "scan_02"
output_type = "ply" # ply or e57
output_path = f"export/{scan_id}.{output_type}"
pano = cv2.imread("export/pano_02.jpg")

as_ascii = False
scene_scale = 0.01   # 0.001 for mm -> m
pano_scale = 0.5     # image size 50 % 
y_offset = -0.374    # -37.4 mm
z_offset = 0.2       # offset in mm * -1
z_rotate = -15.5     # degrees


filepaths, angles = angles_from_filenames(path.join(data_dir, scan_id), name="image", ext="npy")
print(f"{len(filepaths)} files found (min: {min(angles)}, max: {max(angles)}).")

array_3D = merge_2D_points(filepaths, angle_step=0.48464451, offset=(0, y_offset, 0), up_vector=(0,0,1), columns="XZI")
pcd = pcd_from_np(array_3D, estimate_normals=True)
pcd = transform(pcd, translate=(0, 0, z_offset))
print("Merge completed.")

# scale pointcloud
if scene_scale !=1:
    pcd = transform(pcd, scale=scene_scale)

# # colorize pointcloud
# pcd = colormap_pcd(pcd, gamma=8, cmap="viridis")

# ANGULAR LOOKUP
angular_points = angular_from_cartesian(np.asarray(pcd.points))
colors = angular_lookup(angular_points, pano, scale=pano_scale, z_rotate=z_rotate)
pcd.colors = o3d.utility.Vector3dVector(np.asarray(colors))

# visualize while exporting
export_pointcloud_threaded(pcd, output_path, ply_ascii=as_ascii)
visualize([pcd], view="front", unlit=True)
