import os

import numpy as np
import csv
import time
import threading


def save_data(data_dir, np_array, format='npy'):
    if format == 'npy':
        t = threading.Thread(target=save_npy, args=(data_dir, np_array))
    else:  # format == 'csv':
        t = threading.Thread(target=save_csv, args=(data_dir, np_array))
    t.start()

def save_csv(save_dir, points_2d, csv_delimiter=',', filename=None):
    if filename is None:
        filename = f"{save_dir}/{time.time()}.csv"

    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f, delimiter=csv_delimiter)
        writer.writerows(points_2d)

def save_npy(save_dir, points_2d, filename=None):
    if filename is None:
        filename = f"{save_dir}/{time.time()}.npy"

    np.save(filename, points_2d)

def csv_from_npy_dir(dir):
    npy_files = list_files(dir, type='npy')
    for npy_file in npy_files:
        data = np.load(npy_file)
        csv_file = os.path.splitext(npy_file)[0] + '.csv'
        save_csv(os.path.dirname(csv_file), data, filename=csv_file)

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


if __name__ == "__main__":
    # convert all npy to csv files
    DATA_DIR = "data"
    csv_from_npy_dir(DATA_DIR)
