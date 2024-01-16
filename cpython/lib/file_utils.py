import os
import numpy as np
import csv
import time
import threading

try:
    from lib.open3d_utils import rotate_3D 
except:
    from open3d_utils import rotate_3D


def save_data(data_dir, np_array, format='npy'):
    if format == 'npy':
        t = threading.Thread(target=save_npy, args=(data_dir, np_array))
    else:  # format == 'csv':
        t = threading.Thread(target=save_csv, args=(data_dir, np_array))
    t.start()


def save_csv(save_dir, points_2d, delimiter=',', filename=None):
    if filename is None:
        filename = f"{save_dir}/{time.time()}.csv"

    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f, delimiter=delimiter)
        writer.writerows(points_2d)


def save_npy(save_dir, points_2d, filename=None):
    if filename is None:
        filename = f"{save_dir}/{time.time()}.npy"

    np.save(filename, points_2d)


def convert_npy_to_csv(dir):
    npy_files = list_files(dir, '.npy')
    for npy_file in npy_files:
        data = np.load(npy_file)
        csv_file = os.path.splitext(npy_file)[0] + '.csv'
        save_csv(os.path.dirname(csv_file), data, filename=csv_file)


def list_files(dir, extension=None):
    filepaths = list()
    for root, dirs, files in os.walk(dir, topdown=False):
        for file in files:
            if extension is None or os.path.splitext(file)[1] == extension:
                filepath = os.path.join(root, file)
                filepaths.append(filepath)
    return filepaths


def merge_data(path, angle_step=0, up_axis="Z", format='npy', delimiter=","):
    if up_axis.upper() == "Z":
        rotation_axis = np.array([0, 0, 1])

    # init result object with (X,Y,Z, intensity)
    pointcloud = np.zeros((1, 4))
    angle = 0

    filepaths = list_files(path, extension= "." + format.lower() )

    for filepath in filepaths:
        if format == 'npy':
            points2d = np.load(filepath)
        elif format == 'csv':
            points2d = np.loadtxt(filepath, delimiter=delimiter)

        # insert 3D Y=0 after column 0 so 2D-Y becomes 3D-Z (Z-up: image is now vertical)
        points3d = np.insert(points2d, 1, values=0, axis=1)

        points3d = rotate_3D(points3d, rotation_axis, angle)
        pointcloud = np.append(pointcloud, points3d, axis=0)

        angle += angle_step
    return pointcloud


if __name__ == "__main__":
    # convert all npy to csv files
    DATA_DIR = "cpython/data"
    convert_npy_to_csv(DATA_DIR)
