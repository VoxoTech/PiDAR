import os
import re
import numpy as np
import math
from glob import glob


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


data_dir = "data/scan_02"
files, angles = angles_from_filenames(data_dir, name="image", ext="npy")

for i, (file, angle) in enumerate(zip(files, angles)):
    print(f"{i:03d}: {file} -> {angle:.2f}")
