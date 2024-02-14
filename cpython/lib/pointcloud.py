import numpy as np
import open3d as o3d
import pye57
import copy


def set_verbosity():
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)  # .Debug

def __KD_search_param__(radius, max_nn):
    return o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn)

def estimate_point_normals(pcd, radius=0.1, max_nn=30):
    KD_search_param = __KD_search_param__(radius, max_nn)
    pcd.estimate_normals(search_param=KD_search_param)
    return pcd

def export_pointcloud(pcd, savepath, type="pcd", write_ascii=True, compressed=True):
    if type == "pcd" or type == "ply":
        if write_ascii:
            compressed = False
        o3d.io.write_point_cloud(savepath+"."+type, pcd, write_ascii=write_ascii, compressed=compressed),

    elif type == "csv":
        if not isinstance(pcd, np.ndarray):
            array = np.asarray(pcd.points)
        np.savetxt(savepath+"."+type, array, delimiter=",")

    elif type == "e57":
        # Convert open3d point cloud to numpy array
        points = np.asarray(pcd.points)
        colors = np.asarray(pcd.colors) * 255

        # Create a dictionary with keys for each coordinate and color
        data_raw = {
            "cartesianX": points[:, 0],
            "cartesianY": points[:, 1],
            "cartesianZ": points[:, 2],
            "colorRed":   colors[:, 0],
            "colorGreen": colors[:, 1],
            "colorBlue":  colors[:, 2]}

        # Create an E57 object with write mode
        e57 = pye57.E57(savepath+"."+type, mode='w')

        # Write the data to the E57 file
        e57.write_scan_raw(data_raw)
        e57.close()

def fpfh_from_pointcloud(pcd, radius=0.25, max_nn=100):
    ''' Fast Point Feature Histograms (FPFH) descriptor'''
    KD_search_param = __KD_search_param__(radius, max_nn)
    return o3d.pipelines.registration.compute_fpfh_feature(pcd, KD_search_param)

def preprocess_point_cloud(pcd, voxel_size=0, compute_fpfh=True):
    if voxel_size > 0:
        # downsample
        pcd_down = pcd.voxel_down_sample(voxel_size)
    else:
        pcd_down = copy.deepcopy(pcd)
        
    # estimate normals
    pcd_down = estimate_point_normals(pcd_down, radius=voxel_size*2, max_nn=30)

    if compute_fpfh:
        # compute FPFH feature
        pcd_fpfh = fpfh_from_pointcloud(pcd_down, radius=voxel_size*5, max_nn=100)
        return pcd_down, pcd_fpfh
    else:
        return pcd_down

def pcd_from_np(array):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(array[:, 0:3])
    
    if array.shape[1] == 4:
        # normalize intensity values to [0, 1]
        intensities = array[:, 3] / np.max(array[:, 3])

        # map intensities to a grayscale color map
        colors = np.column_stack([intensities, intensities, intensities])
        pcd.colors = o3d.utility.Vector3dVector(colors)

    return pcd


if __name__ == "__main__":
    from visualization import visualize

    # download demo data
    DemoICPPointClouds = o3d.data.DemoICPPointClouds()
    path0, path1, path2 = DemoICPPointClouds.paths

    pcd = o3d.io.read_point_cloud(path0)
    pcd_down = preprocess_point_cloud(pcd, voxel_size=0, compute_fpfh=False)
    visualize([pcd_down])
