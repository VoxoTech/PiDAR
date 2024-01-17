import open3d as o3d
import numpy as np

from lib.file_utils import merge_data
from lib.open3d_utils import plot_3D
# from lib.matplotlib_utils import plot_3D


path = "cpython/data"
pointcloud = merge_data(path, angle_step=1, format='npy')

pcd = plot_3D(pointcloud)


pcd.estimate_normals()
pcd.orient_normals_consistent_tangent_plane(100)

print("run Poisson surface reconstruction")
mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=9)
o3d.visualization.draw_geometries([mesh])

print("remove low density vertices")
vertices_to_remove = densities < np.quantile(densities, 0.01)
mesh.remove_vertices_by_mask(vertices_to_remove)
o3d.visualization.draw_geometries([mesh])
