import numpy as np
import open3d as o3d
import pye57
import copy
import cv2
import matplotlib
import threading

from os import path
from scipy.spatial.transform import Rotation as R


# set verbosity level to suppress Open3D debug messages
def set_verbosity():
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)  # .Debug

# colorize pointcloud using matplotlib colormap
def colormap_pcd(pcd, cmap="viridis", gamma=2.2):
    r = np.asarray(pcd.colors)[:, 0]
    r_norm = (r - r.min()) / (r.max() - r.min())
    r_corrected = r_norm**(gamma)
    colors = matplotlib.colormaps[cmap](r_corrected)    # Map the normalized color channel to the color map
    if colors.shape[1] == 4:
        colors = colors[:, :3]                          # Remove the alpha channel if present
    pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd

# load point cloud from file (3D: pcd, ply, e57 | 2D: csv, npy), return as pcd object or numpy table
# columns parameter: "XYZ" for 3D, "XZ" for 2D vertical, "I" for intensity or "RGB" for color
def import_pointcloud(filepath, columns="XYZI", csv_delimiter=",", as_array=False):
    type = path.splitext(filepath)[1][1:]

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
def export_pointcloud(pcd, filepath, ply_ascii=False, csv_delimiter=","):
    format = path.splitext(filepath)[1][1:]

    if format == "pcd":
        o3d.io.write_point_cloud(filename=filepath, pointcloud=pcd)
    
    elif format == "ply":
        o3d.io.write_point_cloud(filename=filepath, pointcloud=pcd, write_ascii=ply_ascii, compressed=True)

    # TODO: add support for exporting intensity and RGB colors
    elif type == "csv":
        if not isinstance(pcd, np.ndarray):
            array = np.asarray(pcd.points)
        np.savetxt(filepath, array, delimiter=csv_delimiter)

    elif type == "e57":
        # if a single point cloud is provided, convert it to a list
        if isinstance(pcd, list): 
            pcd_list = pcd
        elif isinstance(pcd, o3d.geometry.PointCloud):
            pcd_list = [pcd]

        # Create an E57 object with write mode
        e57 = pye57.E57(filepath, mode='w')

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
    
    print("Export completed.")

def export_pointcloud_threaded(pcd, output_path, ply_ascii=False, csv_delimiter=","):
    export_thread = threading.Thread(target=export_pointcloud, args=(pcd, output_path, ply_ascii, csv_delimiter))
    export_thread.start()

# Remove rows with NaN values from a numpy array
def remove_NaN(array):
    return array[~np.isnan(array).any(axis=1)]

# estimate normals for a point cloud
def estimate_point_normals(pcd, radius=1, max_nn=30, center=(0,0,0)):
    KD_search_param = o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn)
    pcd.estimate_normals(search_param=KD_search_param)
    pcd.orient_normals_towards_camera_location(camera_location=center)
    return pcd

# load list of point cloud files and merge them by angle, return numpy array
def merge_2D_points(filepaths, angle_step=0, ccw=False, offset=(0,0,0), up_vector=(0,0,1), columns="XZI", csv_delimiter=","):
    # init result object with (X,Y,Z, intensity)
    pointcloud = np.zeros((1, 4))
    angle = 0

    for filepath in filepaths:
        points2d = import_pointcloud(filepath, columns, csv_delimiter, as_array=True)

        # insert 3D Y=0 after column 0 so 2D-Y becomes 3D-Z (Z-up: image is now vertical)
        points3d = np.insert(points2d, 1, values=0, axis=1)

        points3d = rotate_3D(points3d, angle, translation_vector=offset, rotation_axis=np.array(up_vector))
        pointcloud = np.append(pointcloud, points3d, axis=0)

        if ccw:
            angle += angle_step
        else:
            angle -= angle_step
    
    # Remove rows with NaN values
    pointcloud = remove_NaN(pointcloud)

    return pointcloud

# convert numpy array to open3d point cloud
# supports 2D and 3D points, intensity and RGB colors
def pcd_from_np(array, columns="XYZI", estimate_normals=True):
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
    
    if estimate_normals:
        pcd = estimate_point_normals(pcd, radius=10, max_nn=30)

    # colors
    if "I" in columns:  # type == "XYZI" -> array.shape[1] == 4:
        intensities = array[:, color_i] / 255  # normalize intensity values
        pcd.colors = o3d.utility.Vector3dVector(np.column_stack([intensities, intensities, intensities]))

    elif "RGB" in columns:
        pcd.colors = o3d.utility.Vector3dVector(array[:, color_i:color_i+3] / 255)

    return pcd


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
def transform(pcd, transformation=None, translate=None, scale=None, euler_rotate_deg=None, pivot=(0,0,0)):
    pcd_temp = copy.deepcopy(pcd)
    
    if transformation is not None:
        pcd_temp.transform(transformation)
    
    if translate is not None:
        pcd_temp.translate(translate)

    if euler_rotate_deg is not None:
        euler_rotate_rad = np.deg2rad(euler_rotate_deg)
        rotation_matrix = pcd_temp.get_rotation_matrix_from_xyz(euler_rotate_rad)
        pcd_temp.rotate(rotation_matrix, center=pivot)
    
    if scale is not None:
        pcd_temp.scale(scale, center=pivot)

    return pcd_temp


###########################################
# pointcloud Lookup


def cartesian(point_cloud, degrees=False):
    if degrees:
        point_cloud = np.deg2rad(point_cloud)  # degrees to radians
        
    # Convert the point cloud from angular to cartesian coordinates
    # using the formula x = r * sin(theta) * cos(phi), y = r * sin(theta) * sin(phi), z = r * cos(theta)
    # where theta is the rotationX, phi is the rotationZ, and r is the distance
    x = point_cloud[:, 1] * np.sin(point_cloud[:, 0]) * np.cos(point_cloud[:, 2])
    y = point_cloud[:, 1] * np.sin(point_cloud[:, 0]) * np.sin(point_cloud[:, 2])
    z = point_cloud[:, 1] * np.cos(point_cloud[:, 0])
    point_cloud = np.stack([x, y, z], axis=1) # stack the x, y, z arrays into a [n, 3] array
    return point_cloud
    
    
def pcd_from_array(point_cloud, colors=None):
    # Create a dictionary of tensors for the point cloud attributes
    map_to_tensors = {}
    map_to_tensors["positions"] = o3d.core.Tensor(point_cloud, dtype=o3d.core.float32)
    if colors:
        map_to_tensors["colors"] = o3d.core.Tensor(colors, dtype=o3d.core.uint8)

    # Create a point cloud object with open3d
    pcd = o3d.t.geometry.PointCloud(map_to_tensors)

    # Return the point cloud object
    return pcd


def get_point_colors(point_cloud, spherical_map, interpolate=1, degrees=True):
    # Resize the spherical map image with cv2.INTER_AREA interpolation for anti-aliasing
    image_height, image_width, _ = spherical_map.shape
    
    if interpolate != 1:
        image_height = int(image_height * interpolate)
        image_width = int(image_height * 2)  # spherical map aspect ratio is 2:1
        spherical_map = cv2.resize(spherical_map, (image_width, image_height), interpolation=cv2.INTER_AREA)

    if degrees:
        point_cloud = np.deg2rad(point_cloud)  # degrees to radians
    
    # Convert the point cloud from rotationX and rotationZ to longitude and latitude
    longitude = np.mod(point_cloud[:, 2], 2 * np.pi) # theta = rotationZ mod 2 * pi
    #longitude = point_cloud[:, 2]  #  theta = rotationZ
    latitude = np.pi / 2 - point_cloud[:, 0]  # phi =  pi / 2 - rotationX
    #latitude = np.mod(np.pi / 2 - point_cloud[:, 0], 2 * np.pi) # phi = (pi / 2 - rotationX) mod 2 * pi

    # Convert the point cloud from longitude and latitude to pixel coordinates
    image_x = np.floor((longitude / (2 * np.pi)) * image_width).astype(int) # image_x = floor(longitude / (2 * pi) * image_width)
    #image_x = (longitude + np.pi) / (2 * np.pi) * image_width
    image_y = np.floor(((np.pi / 2 - latitude) / np.pi) * image_height).astype(int) # image_y = floor((pi / 2 - latitude) / pi * image_height)
    #image_y = (np.pi / 2 - latitude) / np.pi * image_height


    # Create a map of pixel coordinates from the point cloud
    map_x, map_y = np.meshgrid(np.arange(point_cloud.shape[0]), np.arange(1)) # create a grid of shape (1, n)
    map_x = image_x.reshape(1, -1) # reshape the image_x array to (1, n)
    map_y = image_y.reshape(1, -1) # reshape the image_y array to (1, n)

    # # Round the pixel coordinates to the nearest integer and clip them to the image boundaries
    # image_x = np.round(image_x).astype(int)
    # image_y = np.round(image_y).astype(int)
    # image_x = np.clip(image_x, 0, image_width - 1)
    # image_y = np.clip(image_y, 0, image_height - 1)


    # Sample the color values from the resized spherical map image using the map of pixel coordinates
    color_values = cv2.remap(spherical_map, map_x, map_y, cv2.INTER_LINEAR) # use linear interpolation
    color_values = color_values.reshape(-1, 3) # reshape the color_values array to (n, 3)

    # # Sample the color values from the resized spherical map image using the pixel coordinates
    # color_values = spherical_map[image_y, image_x]
    # # Convert the color values to a numpy array of dtype uint8
    # color_values = np.array(color_values, dtype=np.uint8)
        
    return color_values


def pointcloud_lookup(angular_points, spherical_map, degrees=True, interpolate=2):
    
     # degrees to radians
    if degrees:
        angular_points = np.deg2rad(angular_points)
    
    colors = get_point_colors(angular_points, spherical_map, interpolate=interpolate, degrees=False)
    
    # convert angular coordinates (radians) to cartesian coordinates
    cartesian_points = cartesian(angular_points, degrees=False)
    
    # create open3D pointcloud
    pcd = pcd_from_array(cartesian_points, colors=colors)
    
    return pcd



###########################################

if __name__ == "__main__":
    from file_utils import list_files
    from visualization import opengl_fallback, visualize

    opengl_fallback()

    scan_id = "scan_01"
    output_type = "ply" # ply or e57

    filepaths = list_files(f"data/{scan_id}", type='npy')
    array_3D = merge_2D_points(filepaths, angle_step=0.48464451, offset=(0, -0.374, -0.102), up_vector=(0,0,1), columns="XZI")
    pcd = pcd_from_np(array_3D, estimate_normals=True)
    
    export_pointcloud(pcd, f"export/{scan_id}.{output_type}")
    visualize([pcd], unlit=True)


    # -----------------------------------
    # pcd = pointcloud_lookup(angular_points, spherical_map, degrees=True, interpolate=2)
    