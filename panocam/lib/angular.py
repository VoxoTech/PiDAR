"""
http://paulbourke.net/dome/dualfish2sphere/diagram.pdf
"""
import math
import numpy as np


def vector_to_longlat(point3d):
    Px, Py, Pz = point3d
    longitude = math.atan2(-Py, Px)
    latitude = math.atan2(Pz, math.sqrt(Px**2 + Py**2))
    return longitude, latitude  # CW


def longlat_to_vector(longlat):
    longitude, latitude = longlat
    Px = math.cos(latitude) * math.cos(longitude)
    Py = math.cos(latitude) * math.sin(longitude)
    Pz = math.sin(latitude)
    return Px, Py, Pz


def longlat_to_spherical(longlat):  # horizontal, vertical
    longitude, latitude = longlat
    u = longitude / math.pi / 2
    v = 2 * latitude / math.pi
    return u, v


def spherical_to_longlat(uv):
    u, v = uv
    longitude = u * math.pi
    latitude = v * math.pi / 2
    return longitude, latitude


# radians to degrees
def deg(rad_list):
    # deg_list = [math.degrees(item) for item in rad_list]  # list comprehension
    return tuple(map(math.degrees, rad_list))


# degrees to radians
def rad(deg_list):
    return tuple(map(math.radians, deg_list))


# % 360
def mod(angles, mod=math.pi*2):
    angles = list(map(math.fmod, angles, [mod] * len(angles)))
    return angles


# LIST CONVERSIONS

# https://stackoverflow.com/questions/20924085/python-conversion-between-coordinates
def polar2cartesian(angles, distances, offset_angle=math.pi/2):
    """
    converts list of polar coordinates into cartesian (x/y)
    :param angles: list of radians
    :param distances: list of distances
    :param offset_angle: offset radian
    :return: two lists of x and y coordinates
    """
    angles = list(np.array(angles) + offset_angle)
    x_list = distances * -np.cos(angles)
    y_list = distances * np.sin(angles)
    return x_list, y_list


def cartesian2polar(x_list, y_list, offset_angle=math.pi/2):
    distances = np.sqrt(x_list**2 + y_list**2)
    angles = -np.arctan2(y_list, x_list)
    angles = list(np.array(angles) + offset_angle)
    return angles, distances





if __name__ == "__main__":
    longlat = vector_to_longlat((-1, 0, 1))
    # longlat_to_spherical
    print(deg(longlat))
    # math.degrees(longlat[0]), math.degrees(longlat[1])

