import numpy as np
import open3d as o3d
import pye57
import copy
import os

from scipy.spatial.transform import Rotation as R


# set verbosity level to suppress Open3D debug messages
def set_verbosity():
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)  # .Debug

# load point cloud from file (3D: pcd, ply, e57 | 2D: csv, npy), return as pcd object or numpy table
# columns parameter: "XYZ" for 3D, "XZ" for 2D vertical, "I" for intensity or "RGB" for color
def import_pointcloud(filepath, columns="XYZI", csv_delimiter=",", as_array=False):
    type = os.path.splitext(filepath)[1][1:]

    if type == "pcd" or type == "ply":
        pcd = o3d.io.read_point_cloud(filepath)

        if as_array:
            return np.column_stack(np.asarray(pcd.points), np.asarray(pcd.colors) * 255)

    elif type == "e57":
        # Create an E57 object with read mode and read data
        e57 = pye57.E57(filepath, mode='r')
        data = e57.read_scan_raw()
        e57.close()

        # Create a numpy array with the point cloud data
        points = np.column_stack([data["cartesianX"], data["cartesianY"], data["cartesianZ"]])
        colors = np.column_stack([data["colorRed"],   data["colorGreen"], data["colorBlue"]])

        if as_array:
            return np.column_stack(np.asarray(points), np.asarray(colors))

        # Normalize the color values to the range [0, 1]
        colors = colors / 255

        # Create an open3d point cloud object
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)

    elif type == "csv":
        array = np.loadtxt(filepath, delimiter=csv_delimiter)
        if as_array:
            return array
    
        pcd = pcd_from_np(array, columns=columns)

    elif type == 'npy':
        array = np.load(filepath)
        if as_array:
            return array
    
        pcd = pcd_from_np(array, columns=columns)

    else:
        raise ValueError("Unsupported file type: " + type)
    
    return pcd

# export point cloud to file (pcd, ply, e57, csv)
def export_pointcloud(pcd, savepath, type="pcd", write_ascii=True, compressed=True):
    if type == "pcd" or type == "ply":
        if write_ascii:
            compressed = False
        o3d.io.write_point_cloud(savepath+"."+type, pcd, write_ascii=write_ascii, compressed=compressed),

    elif type == "csv":
        if not isinstance(pcd, np.ndarray):
            array = np.asarray(pcd.points)
        np.savetxt(savepath+"."+type, array, delimiter=",")

    elif type == "e57":
        # if a single point cloud is provided, convert it to a list
        if isinstance(pcd, o3d.geometry.PointCloud):
            pcd_list = [pcd]
        elif isinstance(pcd, list): 
            pcd_list = pcd

        # Create an E57 object with write mode
        e57 = pye57.E57(savepath+"."+type, mode='w')

        # Write each point cloud to the E57 file as a separate scan
        for pcd in pcd_list:
            # Convert open3d point cloud to numpy array
            points = np.asarray(pcd.points)
            colors = np.asarray(pcd.colors) * 255

            # Create a dictionary with keys for each coordinate and color
            data_raw = {
                "cartesianX": points[:, 0],
                "cartesianY": points[:, 1],
                "cartesianZ": points[:, 2],
                "colorRed"  : colors[:, 0],
                "colorGreen": colors[:, 1],
                "colorBlue" : colors[:, 2]}

            # Write the point cloud data to the E57 file
            e57.write_scan_raw(data_raw)
        e57.close()

# estimate normals for a point cloud
def estimate_point_normals(pcd, radius=0.1, max_nn=30):
    KD_search_param = o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn)
    pcd.estimate_normals(search_param=KD_search_param)
    return pcd

# convert numpy array to open3d point cloud
# supports 2D and 3D points, intensity and RGB colors
def pcd_from_np(array, columns="XYZI"):
    columns = columns.upper()
    zeros = np.zeros((array.shape[0], 1))
    pcd = o3d.geometry.PointCloud()

    # 3D points
    if "XYZ" in columns:
        pcd.points = o3d.utility.Vector3dVector(array[:, 0:3])
        color_i = 3

     # 2D points
    else:
        color_i = 2
        if "XY" in columns:
            pcd.points = o3d.utility.Vector3dVector(np.hstack((array[:, 0:2], zeros)))
        elif "XZ" in columns:
            pcd.points = o3d.utility.Vector3dVector(np.hstack((array[:, 0:1], zeros, array[:, 1:2])))
        elif "YZ" in columns:
            pcd.points = o3d.utility.Vector3dVector(np.hstack((zeros, array[:, 0:2])))
        else:
            raise ValueError("Unsupported point cloud type: " + type)
    
    # colors
    if "I" in columns:  # type == "XYZI" -> array.shape[1] == 4:
        intensities = array[:, color_i] / 255  # normalize intensity values
        pcd.colors = o3d.utility.Vector3dVector(np.column_stack([intensities, intensities, intensities]))
    elif "RGB" in columns:
        pcd.colors = o3d.utility.Vector3dVector(array[:, color_i:color_i+3] / 255)

    return pcd

# load list of point cloud files and merge them by angle.
# returns pcd or numpy array
def merge_data(filepaths, angle_step=0, offset=(0,0,0), up_axis="Z", columns="XZI", csv_delimiter=",", as_pcd=True):
    if up_axis.upper() == "Z":
        rotation_axis = np.array([0, 0, 1])

    # init result object with (X,Y,Z, intensity)
    pointcloud = np.zeros((1, 4))
    angle = 0

    for filepath in filepaths:
        points2d = import_pointcloud(filepath, columns, csv_delimiter, as_array=True)

        # insert 3D Y=0 after column 0 so 2D-Y becomes 3D-Z (Z-up: image is now vertical)
        points3d = np.insert(points2d, 1, values=0, axis=1)

        points3d = rotate_3D(points3d, angle, translation_vector=offset, rotation_axis=(0,0,1))
        pointcloud = np.append(pointcloud, points3d, axis=0)
        angle += angle_step

    if as_pcd:
        return pcd_from_np(pointcloud)
    else:
        return pointcloud


###########################################
# Transformation functions

def rotate_3D(points3d, rotation_degrees, translation_vector=(0,0,0), rotation_axis=(0,0,1)):
    rotation_axis = np.array(rotation_axis)

    rotation_radians = np.radians(rotation_degrees)
    rotation_vector = rotation_radians * rotation_axis
    rotation = R.from_rotvec(rotation_vector)
    rotation_matrix = rotation.as_matrix()
    pcd = o3d.geometry.PointCloud()

    pcd.points = o3d.utility.Vector3dVector(points3d[:, 0:3])  # column 4 are intensities 

    if points3d.shape[1] == 4:
        intensities = points3d[:, 3]
        colors = np.stack([intensities]*3, axis=-1)
        pcd.colors = o3d.utility.Vector3dVector(colors)

    # Perform translation before rotation, adjusting Y based on translation and initial Z
    pcd.translate(translation_vector)
    # pcd.points[:, 1] += pcd.points[:, 2] * translation_vector[1]  # Update Y based on translation and Z

    # Rotate the point cloud using the rotation matrix
    pcd.rotate(rotation_matrix, center=(0, 0, 0))

    # Convert the point cloud back to a NumPy array
    result_points = np.asarray(pcd.points)

    # Reattach intensity column if provided
    if points3d.shape[1] == 4:
        result_points = np.column_stack((result_points, points3d[:, 3]))

    return result_points

# def rotate_3D_np(points3d, rotation_axis, rotation_degrees):
#     """ Rotate a vector v about axis by taking the component of v perpendicular to axis,
#     rotating it theta in the plane perpendicular to axis, 
#     then add the component of v parallel to axis.

#     Let a be a unit vector along an axis axis. Then a = axis/norm(axis).
#     Let A = I x a, the cross product of a with an identity matrix I.
#     Then exp(theta,A) is the rotation matrix.
#     Finally, dotting the rotation matrix with the vector will rotate the vector.

#     https://www.kite.com/python/answers/how-to-rotate-a-3d-vector-about-an-axis-in-python
#     https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.html
#     """
#     rotation_radians = np.radians(rotation_degrees)
#     rotation_vector = rotation_radians * rotation_axis
#     rotation = R.from_rotvec(rotation_vector)

#     # rotation_matrix = rotation.as_matrix()
#     # return np.dot(points3d[:, 0:3], rotation_matrix.T)

#     result_points = points3d.copy()
#     if points3d.shape[1] == 4:
#         # remove intensity column
#         points3d = np.delete(points3d, -1, axis=1)
#     # apply rotation to each point
#     for i, point in enumerate(points3d):
#         result_points[i][0:3] = rotation.apply(point)
#     return result_points

# extract translation and rotation vectors from transformation matrix
def get_transform_vectors(transform_M):
    # Extract translation (top-right 3x1 sub-matrix)
    translation = transform_M[:3, 3]

    # Extract rotation (top-left 3x3 sub-matrix), make a copy to avoid read only error
    rotation_M = np.array(transform_M[:3, :3])
    # Convert rotation matrix to Euler angles
    r = R.from_matrix(rotation_M)
    euler_angles = r.as_euler('xyz', degrees=True)

    return translation, euler_angles

# apply translation and rotation to open3d point cloud
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


if __name__ == "__main__":
    from file_utils import list_files
    from visualization import opengl_fallback, visualize

    opengl_fallback()

    filepaths = list_files("data", type='npy')
    pcd = merge_data(filepaths, angle_step=5.5, offset=(0, 10, 0), up_axis="Z", columns="XZI", as_pcd=True)

    export_pointcloud([pcd], "export/3d-scan", type="e57")
    visualize([pcd], unlit=True)
