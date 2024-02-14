import open3d as o3d
import numpy as np
import os

from lib.file_utils import merge_data
from lib.pointcloud import pcd_from_np
from lib.visualization import visualize
from lib.platform import get_platform


os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1' if get_platform() == 'RaspberryPi' else '0'  # disable OpenGL

path = "data"
pointcloud = merge_data(path, angle_step=0.3, format='npy')
pcd = pcd_from_np(pointcloud)
visualize([pcd], unlit=True)
