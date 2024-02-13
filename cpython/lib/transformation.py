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

import open3d as o3d
import numpy as np
import copy
from scipy.spatial.transform import Rotation as R


def rotate_3D(points3d, rotation_axis, rotation_degrees, use_o3d=True):
    rotation_radians = np.radians(rotation_degrees)
    rotation_vector = rotation_radians * rotation_axis
    rotation = R.from_rotvec(rotation_vector)

    if use_o3d:
        rotation_matrix = rotation.as_matrix()
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points3d[:, 0:3])
        pcd.rotate(rotation_matrix, center=(0, 0, 0))
        result_points = np.asarray(pcd.points)
        if points3d.shape[1] == 4:
            # reattach intensity column
            result_points = np.column_stack((result_points, points3d[:, 3]))
    else:
        result_points = points3d.copy()
        if points3d.shape[1] == 4:
            # remove intensity column
            points3d = np.delete(points3d, -1, axis=1)
        # apply rotation to each point
        for i, point in enumerate(points3d):
            result_points[i][0:3] = rotation.apply(point)

    return result_points



def get_transform_vectors(transform_M):
    # Extract translation (top-right 3x1 sub-matrix)
    translation = transform_M[:3, 3]

    # Extract rotation (top-left 3x3 sub-matrix), make a copy to avoid read only error
    rotation_M = np.array(transform_M[:3, :3])
    # Convert rotation matrix to Euler angles
    r = R.from_matrix(rotation_M)
    euler_angles = r.as_euler('xyz', degrees=True)

    return translation, euler_angles

def transform(pcd, transformation=None, translate=None, euler_rotate_deg=None, pivot=(0,0,0)):
    pcd_temp = copy.deepcopy(pcd)
    
    if transformation is not None:
        pcd_temp.transform(transformation)
    
    if translate is not None:
        pcd_temp.translate(translate)

    if euler_rotate_deg is not None:
        euler_rotate_rad = np.deg2rad(euler_rotate_deg)
        rotation_matrix = pcd_temp.get_rotation_matrix_from_xyz(euler_rotate_rad)
        pcd_temp.rotate(rotation_matrix, center=pivot)

    return pcd_temp
