"""
http://paulbourke.net/dome/dualfish2sphere/diagram.pdf


Hugin CLI parameters from StereoPi.sh :
    pto_gen --projection=2 --fov=360 -o ./tmp/project.pto $1 $2
    pto_template --output=./tmp/project.pto --template=template.pto ./tmp/project.pto
    hugin_executor --stitching --prefix=$RESULT_NAME ./tmp/project.pto

additional parameters:
    pto_lensstack -o project1.pto --new-lens i1 project.pto
    cpfind -o project.pto --multirow project.pto
    cpclean -o project.pto project.pto
    linefind -o project.pto project.pto
    pto_var -o project.pto --opt TrX,TrY,TrZ,r,Eev,Ra,Rb,Rc,Rd,Re,!TrX0,!TrY0,!TrZ0,!r0,!Eev0,!Ra1,!Rb1,!Rc1,!Rd1,!Re1 project.pto
    autooptimiser -n -o project.pto project.pto
    pano_modify  --projection=1 --fov=AUTO --center --canvas=AUTO --crop=AUTO -o project.pto project.pto
"""

import math
import numpy as np
import subprocess
import shutil
import os


def hugin_modify(project_path, output_path, width=6800):
    # Load the project file(project_path)
    with open(project_path, 'r') as file:
        lines = file.readlines()

    # search for first line that starts with 'p ' -> 'p f2 w6000 h3000 v360 ...'
    for i, line in enumerate(lines):

        if line.startswith("p "):
            # replace w and h arguments
            words = line.split(" ")
            for j, word in enumerate(words):
                if word.startswith("w"):
                    words[j] = f"w{width}"
                elif word.startswith("h"):
                    words[j] = f"h{width//2}"

            lines[i] = " ".join(words)
            break
    
    # Save the project file at output_path
    with open(output_path, 'w') as file:
        file.writelines(lines)

def hugin_stitch(files, template=None, width=None, output_path="export/pano.jpg", tmp_dir="panocam/tmp"):
    # Create a temporary directory
    os.makedirs(tmp_dir, exist_ok=True)

    project_path = f'./{tmp_dir}/temp.pto'

    # create Hugin project match to template
    calls = [['pto_gen', '--projection=2', '--fov=360', '-o', project_path, *files],  # *files -> unpack list
             ['pto_template', f'--output={project_path}', f'--template={template}', project_path]]
    for call in calls:
        subprocess.run(call)
    
    # modify resolution of the temporary project file
    if width is not None:
        hugin_modify(project_path, project_path, width=width)

    # stitch the panorama
    subprocess.run(['hugin_executor', '--stitching', f'--prefix={output_path}', project_path])

    # remove temporary directory
    shutil.rmtree(tmp_dir)

def sample_color(img, uv, normalize_color=False):
    longitude, latitude = uv

    if isinstance(longitude, float):
        h, w, c = img.shape
        longitude = int(longitude * w)
        latitude = int(latitude * h)

    # print(longitude, latitude)
    color = img[latitude, longitude]

    if normalize_color:  # convert uint8 to float
        color = color / 255

    return tuple(color)

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
