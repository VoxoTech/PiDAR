""" 
* How to rotate a 3D vector about an axis in Python *

Rotate a vector v about axis by taking the component of v perpendicular to axis,
rotating it theta in the plane perpendicular to axis, then add the component of v parallel to axis.

Let a be a unit vector along an axis axis. Then a = axis/norm(axis).
Let A = I x a, the cross product of a with an identity matrix I.
Then exp(theta,A) is the rotation matrix.
Finally, dotting the rotation matrix with the vector will rotate the vector.

rotation
https://www.kite.com/python/answers/how-to-rotate-a-3d-vector-about-an-axis-in-python
https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.html
"""

import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation


def rotate_3D(points3d, rotation_axis, rotation_degrees):
    rotation_radians = np.radians(rotation_degrees)
    rotation_vector = rotation_radians * rotation_axis
    rotation_matrix = Rotation.from_rotvec(rotation_vector).as_matrix()

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points3d[:, 0:3])
    pcd.rotate(rotation_matrix, center=(0, 0, 0))

    result_points = np.asarray(pcd.points)
    if points3d.shape[1] == 4:
        # reattach intensity column
        result_points = np.column_stack((result_points, points3d[:, 3]))

    return result_points


def plot_3D(pointcloud):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pointcloud[:, 0:3])
    
    if pointcloud.shape[1] == 4:
        # normalize intensity values to [0, 1]
        intensities = pointcloud[:, 3] / np.max(pointcloud[:, 3])

        # map intensities to a grayscale color map
        colors = np.column_stack([intensities, intensities, intensities])
        pcd.colors = o3d.utility.Vector3dVector(colors)

    o3d.visualization.draw_geometries([pcd])
    return pcd


class IMU_Visualizer:
    def __init__(self):
        # Initialize Open3D visualizer
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window()

        # Create a 3D box to represent the sensor
        self.box = o3d.geometry.TriangleMesh.create_box(width=0.1, height=0.1, depth=0.1)
        self.box.compute_vertex_normals()
        self.box.paint_uniform_color([0.1, 0.1, 0.7])  # Blue color

        # Add the box to the visualizer
        self.vis.add_geometry(self.box)

    def update_visualization(self, pos, quat):
        # Convert quaternion to rotation matrix
        R = o3d.geometry.get_rotation_matrix_from_quaternion(quat)

        # Apply the translation and rotation to the box
        transformation = np.eye(4)
        transformation[:3, :3] = R
        transformation[:3, 3] = pos
        self.box.transform(transformation)

        # Update the visualizer
        self.vis.update_geometry(self.box)
        self.vis.poll_events()
        self.vis.update_renderer()
