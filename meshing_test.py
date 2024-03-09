import open3d as o3d
from lib.visualization import opengl_fallback, visualize
from lib.mesh import mesh_from_poisson 

opengl_fallback()

pcd = o3d.io.read_point_cloud("export/scan_01.ply")
mesh = mesh_from_poisson(pcd, depth=11, k=50, estimate_normals=True, density_threshold=0.1)
visualize([pcd, mesh], unlit=True, point_size=2, point_colors="normal")
