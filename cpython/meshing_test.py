"""
http://www.open3d.org/docs/latest/tutorial/Advanced/surface_reconstruction.html

normals: http://www.open3d.org/docs/release/python_api/open3d.geometry.PointCloud.html
"""

import open3d as o3d
from lib.visualization import visualize
from lib.mesh import mesh_from_ball_pivoting, mesh_from_alpha_shape, estimate_mesh_normals, mesh_from_poisson


# download demo data
DemoICPPointClouds = o3d.data.DemoICPPointClouds()
path0, path1, path2 = DemoICPPointClouds.paths
pcd = o3d.io.read_point_cloud(path0)

# alpha shape mesh
mesh = mesh_from_alpha_shape(pcd)
visualize([mesh], unlit=False)

# ball pivoting mesh
mesh = mesh_from_ball_pivoting(pcd)
visualize([mesh], unlit=False)

# poisson mesh
mesh = mesh_from_poisson(pcd, depth=10, normal_plane=100, density_threshold=0.1)
visualize([mesh], unlit=False)
