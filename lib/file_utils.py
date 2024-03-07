import os
import time
import csv
import re
import math
import threading
import numpy as np
from glob import glob


def save_data(filepath, np_array):
    dirname, filename = os.path.split(filepath)
    basename, format = os.path.splitext(filename)
    format = format.lstrip('.')

    # if filename is *.npy, create a new filename with a timestamp
    if basename == '*':
        filename = f"{dirname}/{time.time()}.{format}"
    else:
        filename = filepath

    if format == 'npy':
        t = threading.Thread(target=save_npy, args=(filename, np_array))
    else:  # format == 'csv':
        t = threading.Thread(target=save_csv, args=(filename, np_array))
    t.start()

def save_csv(filepath, points_2d, csv_delimiter=','):
    with open(filepath, 'w', newline='') as f:
        writer = csv.writer(f, delimiter=csv_delimiter)
        writer.writerows(points_2d)

def save_npy(filepath, points_2d):
    np.save(filepath, points_2d)

def csv_from_npy_dir(dir):
    npy_files = list_files(dir, type='npy')
    for npy_file in npy_files:
        data = np.load(npy_file)
        csv_file = os.path.splitext(npy_file)[0] + '.csv'
        save_csv(csv_file, data)

def list_files(dir, type=None, recursive=False):
    # from glob import glob
    # return sorted(glob(os.path.join(dir, '*.'+type)))

    filepaths = list()
    for root, dirs, files in os.walk(dir, topdown=True):
        files = sorted(files)

        for file in files:
            if type is None or os.path.splitext(file)[1][1:] == type:  # [1:] just removes the dot in ".jpg"
                filepath = os.path.join(root, file)
                filepaths.append(filepath)
            
        if not recursive:
            break
    print(f"found: {len(filepaths)} {type} files ({len(files)-len(filepaths)} other)")
    return filepaths

def make_dir(dir):
    # if not os.path.exists(dir):
    os.makedirs(dir, exist_ok=True)

def angles_from_filenames(data_dir, name="image", ext = "jpg"):
    # create a list of files
    files = glob(os.path.join(data_dir, f"*.{ext}"))
    files = sorted(files)

    angles = []
    for file in files:
        fname = os.path.basename(file)
        # use a regular expression to find the angle value
        angle = re.search(rf"{name}_(-?\d+\.\d+)", fname)
        if angle:
            angles.append(float(angle.group(1)))
        else:
            angles.append(math.nan)

    angles = np.array(angles)

    # sort both lists by the angles list
    files, angles = zip(*sorted(zip(files, angles), key=lambda pair: pair[1]))

    # filter out the lines that contain a NaN value in the angles list
    files, angles = zip(*filter(lambda pair: not math.isnan(pair[1]), zip(files, angles)))
    return files, angles


if __name__ == "__main__":

    DATA_DIR = "data/scan_02"

    # # convert all npy to csv files
    # csv_from_npy_dir(DATA_DIR)

    # extract angles from filenames
    files, angles = angles_from_filenames(DATA_DIR, name="image", ext="npy")
    for i, (file, angle) in enumerate(zip(files, angles)):
        print(f"{i:03d}: {file} -> {angle:.2f}")
    