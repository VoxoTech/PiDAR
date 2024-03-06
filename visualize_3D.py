from os import path

from lib.pointcloud import merge_2D_points, pcd_from_np, transform, colormap_pcd, export_pointcloud_threaded
from lib.file_utils import list_files
from lib.visualization import opengl_fallback, visualize


opengl_fallback()

scan_id = "scan_01"
output_type = "ply" # ply or e57
output_path = f"export/{scan_id}.{output_type}"
as_ascii = True
scale = 0.001  # 0.001 for mm -> m

filepaths = list_files(path.join("data", scan_id), type='npy')
array_3D = merge_2D_points(filepaths, angle_step=0.48464451, offset=(0, -0.374, -0.102), up_vector=(0,0,1), columns="XZI")
pcd = pcd_from_np(array_3D, estimate_normals=True)
print("Merge completed.")

# colorize pointcloud
pcd = colormap_pcd(pcd, gamma=8, cmap="viridis")

# scale pointcloud
if scale !=1:
    pcd = transform(pcd, scale=scale)

# visualize while exporting
export_pointcloud_threaded(pcd, output_path, ply_ascii=as_ascii)
visualize([pcd], view="front")
