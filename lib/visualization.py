"""
# check opengl version
sudo apt-get install mesa-utils libgl1-mesa-dri
glxinfo | grep "OpenGL version"
>> OpenGL version string: 3.1 Mesa 23.2.1-1~bpo12+rpt2
"""

import open3d as o3d
import copy
import os
import colorsys
import numpy as np

try:
    from lib.pointcloud import transform
    from lib.platform_utils import get_platform
except:
    from pointcloud import transform
    from platform_utils import get_platform


def opengl_fallback(check=True):
    # suppress OpenGL warnings
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)  # .Debug

    # disable OpenGL on Raspberry Pi
    if not check or get_platform() == 'RaspberryPi':
        use_software_rendering = '1'
    else:
        use_software_rendering = '0'
    os.environ['LIBGL_ALWAYS_SOFTWARE'] = use_software_rendering


def __generate_colors__(n, lightness=0.5, saturation=0.8, float=False):
    start_hue, _, _ = colorsys.rgb_to_hls(1, 0.706, 0)  # yellow
    end_hue, _, _ = colorsys.rgb_to_hls(0, 0.651, 0.929)  # blue
    print(start_hue, end_hue)

    colors = []
    for i in range(n):
        hue = start_hue + i * (end_hue - start_hue) / (n-1) if n > 1 else end_hue
        r, g, b = colorsys.hls_to_rgb(hue, lightness, saturation)
        if float:
            colors.append((r, g, b))
        else:
            colors.append((int(r * 255), int(g * 255), int(b * 255)))
    return colors


def visualize(object_list, title="PiDAR", transformation=None, width=1800, height=1000, left=0, view="front", point_size=1, unlit=False, backface=True, point_colors="color", bgcolor=(0.15,0.15,0.15)):
    if point_colors.lower() == "uniform":
        point_colors = "color"
        uniform_colors = True
    else:
        uniform_colors = False
    point_color_option = {"x"       : o3d.visualization.PointColorOption.XCoordinate, 
                          "y"       : o3d.visualization.PointColorOption.YCoordinate, 
                          "z"       : o3d.visualization.PointColorOption.ZCoordinate, 
                          "normal"  : o3d.visualization.PointColorOption.Normal, 
                          "color"   : o3d.visualization.PointColorOption.Color
                          }[point_colors.lower()]
    
    # Deepcopy to avoid modifying the original data
    if transformation is not None or point_color_option == 5:
        object_list = copy.deepcopy(object_list)

    if transformation is not None:
        # np.identity(4)
        object_list[0] = transform(object_list[0], transformation=transformation)
    
    if uniform_colors:
        object_list = copy.deepcopy(object_list)
        
        colors = __generate_colors__(len(object_list), float=True)
        for i, object in enumerate(object_list):
            object_list[i].paint_uniform_color(colors[i])

    views = {"top":   {"zoom": 0.5, "front": (0, 0, 10), "lookat": (1, 0, 0), "up": (0, -1, 0)},
             "front": {"zoom": 0.25, "front": (-1, 4, 1), "lookat": (0, 0, 0), "up": (0, 0, 1)}}
    
    if isinstance(view, dict):
        # custom view
        v = view
    elif isinstance(view, str) and view in views.keys():
        # predefined view
        v = views[view]
    else:
        print("[Error] Invalid view. defaulting to top view")
        v = views["top"]
    
    # Create a visualizer
    vis = o3d.visualization.Visualizer()
    vis.create_window(width=width, height=height, left=left, window_name=title)
    
    # Add the geometries to the visualizer
    for obj in object_list:
        vis.add_geometry(obj)
    
    # Set the view parameters
    vis.get_view_control().set_zoom(v["zoom"])
    vis.get_view_control().set_front(v["front"])
    vis.get_view_control().set_lookat(v["lookat"])
    vis.get_view_control().set_up(v["up"])
    
    # Set the background color to dark gray
    render_option = vis.get_render_option()
    render_option.background_color = np.asarray(bgcolor)
    render_option.point_size = point_size
    render_option.light_on = False if unlit else True
    render_option.mesh_show_back_face = backface

    render_option.point_color_option = point_color_option

    # Run the visualizer
    vis.run()
    
    # Destroy the window before exiting
    vis.destroy_window()


# def visualize_simple(object_list, uniform_colors=False, view="front"):
#     if uniform_colors:
#         object_list = copy.deepcopy(object_list)
        
#         colors = get_color_list(len(object_list), float=True)
#         for i, object in enumerate(object_list):
#             object_list[i].paint_uniform_color(colors[i])
    

#     views = {"top":   {"zoom": 0.5, "front": (0, 0, 10), "lookat": (1, 0, 0), "up": (0, -1, 0)},
#              "front": {"zoom": 0.25, "front": (-1, 4, 1), "lookat": (0, 0, 0), "up": (0, 0, 1)}}
    
#     if isinstance(view, dict):
#         # custom view
#         v = view
#     elif isinstance(view, str) and view in views.keys():
#         # predefined view
#         v = views[view]
#     else:
#         print("[Error] Invalid view. defaulting to top view")
#         v = views["top"]
        
#     o3d.visualization.draw_geometries(object_list, mesh_show_back_face=True, zoom=v["zoom"], front=v["front"], lookat=v["lookat"], up=v["up"])


if __name__ == "__main__":
    from pointcloud import estimate_point_normals

    # download demo data
    DemoICPPointClouds = o3d.data.DemoICPPointClouds()
    path0, path1, path2 = DemoICPPointClouds.paths

    pcd = o3d.io.read_point_cloud(path0)
    pcd_down = estimate_point_normals(pcd, radius=1, max_nn=30)
    
    view={"zoom": 0.5, "front": (0, 0, -1), "lookat": (2, 2, 1.5), "up": (0, -1, 0)}
    visualize([pcd_down], view=view, point_colors="normal")
