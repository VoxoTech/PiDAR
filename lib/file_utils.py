import os
import time
import threading
import csv
import numpy as np


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


if __name__ == "__main__":
    # convert all npy to csv files
    DATA_DIR = "data"
    csv_from_npy_dir(DATA_DIR)
