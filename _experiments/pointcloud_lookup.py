import cv2
import numpy as np
import open3d as o3d


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

