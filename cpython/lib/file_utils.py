import os
import time
import csv
import numpy as np

from lib.open3d_utils import rotate_3D


def save_thread(save_dir, x_list, y_list, color, delimiter):
    filename = f"{save_dir}/{time.time()}.csv"
    
    data = list(zip(x_list, y_list, color))

    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f, delimiter=delimiter)
        writer.writerows(data)


def list_files(dir, extension=None):
    filepaths = list()
    for root, dirs, files in os.walk(dir, topdown=False):
        for file in files:
            if extension is None or os.path.splitext(file)[1] == extension:
                filepath = os.path.join(root, file)
                filepaths.append(filepath)
    return filepaths


def merge_csv_dir(csv_dir, angle_step=0, up_axis="Z"):
    if up_axis == "Z":
        rotation_axis = np.array([0, 0, 1])

    # init result object with (X,y,Z, intensity)
    pointcloud = np.zeros((1, 4))
    angle = 0

    filepaths = list_files(csv_dir, extension=".csv")

    for filepath in filepaths:
        points2d = np.loadtxt(filepath, delimiter=";")

        # insert 3D Y=0 after column 0 so 2D-Y becomes 3D-Z (Z-up: image is now vertical)
        points3d = np.insert(points2d, 1, values=0, axis=1)

        points3d = rotate_3D(points3d, rotation_axis, angle)
        pointcloud = np.append(pointcloud, points3d, axis=0)

        angle += angle_step
    return pointcloud
