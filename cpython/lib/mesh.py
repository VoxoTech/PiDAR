import numpy as np
import open3d as o3d
import os


def sample_poisson_disk(pcd, count=1000000):
    return pcd.sample_points_poisson_disk(count)

def estimate_mesh_normals(mesh):
    return mesh.compute_vertex_normals()

def mesh_optimize(mesh, count=1000000):
    mesh = mesh.simplify_quadric_decimation(count)
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_duplicated_vertices()
    mesh.remove_non_manifold_edges()
    return mesh

def mesh_from_alpha_shape(pcd,  alpha=0.03):
    # estimate_mesh_normals(mesh)
    return o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)

def mesh_from_ball_pivoting(pcd):
    '''https://towardsdatascience.com/5-step-guide-to-generate-3d-meshes-from-point-clouds-with-python-36bad397d8ba'''
    distances = pcd.compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances)
    radius = avg_dist * 2
    ball_radii = o3d.utility.DoubleVector([radius, radius * 2])
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, ball_radii)
    return mesh

def mesh_from_poisson(pcd, depth=9, normal_plane=100, density_threshold=0.1, return_densities=False):
    pcd.estimate_normals()
    pcd.orient_normals_consistent_tangent_plane(normal_plane)
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=depth)

    if density_threshold > 0:
        remove_low_density_vertices(mesh, densities, quantile=density_threshold)

    if return_densities:
        return mesh, densities
    else:
        return mesh

def remove_low_density_vertices(mesh, densities, quantile=0.1):
    vertices_to_remove = densities < np.quantile(densities, quantile)
    return mesh.remove_vertices_by_mask(vertices_to_remove)


if __name__ == "__main__":

    bunny = o3d.data.BunnyMesh()
    mesh  = o3d.io.read_triangle_mesh(bunny.path)
    estimate_mesh_normals(mesh)

    pcd = sample_poisson_disk(mesh, count=5000)
    o3d.visualization.draw_geometries([mesh, pcd])


    # alpha shape mesh
    mesh = mesh_from_alpha_shape(pcd)
    estimate_mesh_normals(mesh)
    o3d.visualization.draw_geometries([mesh])

    # ball pivoting mesh
    mesh = mesh_from_ball_pivoting(pcd)
    # mesh = mesh_optimize(mesh, count=1000000)
    o3d.visualization.draw_geometries([mesh])

    # poisson mesh  # TODO: broken with this dataset
    mesh = mesh_from_poisson(pcd, depth=10, normal_plane=100, density_threshold=0.1)
    o3d.visualization.draw_geometries([mesh])
