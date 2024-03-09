"""
http://www.open3d.org/docs/release/tutorial/visualization/non_blocking_visualization.html
http://www.open3d.org/docs/latest/tutorial/Basic/transformation.html

https://www.open3d.org/docs/latest/tutorial/Advanced/global_registration.html
https://www.open3d.org/docs/latest/tutorial/Basic/icp_registration.html
"""

import open3d as o3d
import numpy as np
import copy


# compute Fast Point Feature Histograms (FPFH) descriptor for a point cloud
def fpfh_from_pointcloud(pcd, radius=0.25, max_nn=100):
    KD_search_param = o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn)
    return o3d.pipelines.registration.compute_fpfh_feature(pcd, KD_search_param)

# RANSAC or FAST global registration using FPFH features
def global_registration(source_down, target_down, source_fpfh, target_fpfh, distance_threshold, use_fast=False, max_iteration=4000000, confidence=0.9):
    if use_fast:
        result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh, 
            o3d.pipelines.registration.FastGlobalRegistrationOption(
                maximum_correspondence_distance=distance_threshold))

    else: # RANSAC
        result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh, True, distance_threshold,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(False), 4, [
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)], 
                o3d.pipelines.registration.RANSACConvergenceCriteria(max_iteration, confidence))
        
    return result

# point-to-plane or point-to-point ICP registration
def ICP_registration(source, target, distance_threshold, transformation, use_p2l=True, p2p_max_iteration=200):
    if use_p2l: # point-to-plane ICP
        icp_method = o3d.pipelines.registration.TransformationEstimationPointToPlane()
        
        return o3d.pipelines.registration.registration_icp(
            source, target, distance_threshold, transformation, icp_method)
    
    else: # point-to-point ICP
        icp_method = o3d.pipelines.registration.TransformationEstimationPointToPoint()
        convergence_criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=p2p_max_iteration)

        return o3d.pipelines.registration.registration_icp(
            source, target, distance_threshold, transformation, icp_method, convergence_criteria)

def evaluate_registration(source, target, threshold, transformation=None):
    source_temp = copy.deepcopy(source)

    if transformation is None:
        transformation = np.identity(4)
        source_temp.transform(transformation)

    return o3d.pipelines.registration.evaluate_registration(source_temp, target, threshold, transformation)


if __name__ == "__main__":
    import time

    from pointcloud import export_pointcloud, get_transform_vectors, transform, estimate_point_normals
    from registration import fpfh_from_pointcloud, global_registration, ICP_registration
    from visualization import opengl_fallback, visualize


    opengl_fallback()

    voxel_size = 0.05  # meter units
    gr_max_iteration = 1000000
    gr_confidence = 0.9

    icp_threshold = voxel_size * 0.4
    p2p_max_iteration = 200

    verbose = False


    # download demo data (cloud_bin_0.pcd, cloud_bin_1.pcd, cloud_bin_2.pcd)
    DemoICPPointClouds = o3d.data.DemoICPPointClouds()
    path0, path1, path2 = DemoICPPointClouds.paths


    # TODO: 0 <> 2 not working with P2L
    source = o3d.io.read_point_cloud(path2)
    target = o3d.io.read_point_cloud(path0)

    # downsample, compute normals, and compute FPFH feature
    source_down = estimate_point_normals(source.voxel_down_sample(voxel_size), radius=voxel_size*2, max_nn=30)
    source_fpfh = fpfh_from_pointcloud(source_down, radius=voxel_size*5, max_nn=100)

    target_down = estimate_point_normals(target.voxel_down_sample(voxel_size), radius=voxel_size*2, max_nn=30)
    target_fpfh = fpfh_from_pointcloud(target_down, radius=voxel_size*5, max_nn=100)


    view={"zoom": 0.5, "front": (0, 0, -1), "lookat": (2, 2, 1.5), "up": (0, -1, 0)}
    visualize([source, target], point_colors="uniform", view=view)
    visualize([source_down, target_down], point_colors="uniform", view=view)



    ########################################
    # FAST GLOBAL REGISTRATION 

    # start = time.time()
    # distance_threshold = voxel_size * 0.5
    # reg_fast = global_registration(source_down, target_down, source_fpfh, target_fpfh, distance_threshold, 
    #                                   use_fast=True, max_iteration=gr_max_iteration, confidence=gr_confidence)

    # print(f"FAST global registration took {time.time() - start:.3f} sec.")
    # # print(reg_fast)
    # visualize([source_down, target_down], transformation=reg_fast.transformation, point_colors="uniform", view=view)

    # # print(evaluate_registration(source, target, icp_threshold, transform=reg_ransac.transformation))
    # visualize([source, target], transformation=reg_fast.transformation, point_colors="uniform", view=view)


    # ########################################
    # RANSAC GLOBAL REGISTRATION 

    start = time.time()
    distance_threshold = voxel_size * 1.5
    reg_ransac = global_registration(source_down, target_down, source_fpfh, target_fpfh, distance_threshold, 
                                        use_fast=False, max_iteration=gr_max_iteration, confidence=gr_confidence)

    print(f"\nRANSAC global registration took {time.time() - start:.3f} sec.")
    # print(reg_ransac)

    ransac_translation, ransac_euler = get_transform_vectors(reg_ransac.transformation)
    print(f"[RANSAC] translate:\t{ransac_translation})")
    print(f"[RANSAC] rotate:\t{ransac_euler})")

    visualize([source, target], transformation=reg_ransac.transformation, point_colors="uniform", view=view)


    ########################################
    # # P2P ICP

    # start = time.time()
    # reg_p2p = ICP_registration(source, target, icp_threshold, 
    #                               reg_ransac.transformation, use_p2l=False, p2p_max_iteration=p2p_max_iteration)

    # print(f"P2P ICP took {time.time() - start:.3f} sec.")
    # # print(reg_p2p)

    # visualize([source, target], transformation=reg_p2p.transformation, point_colors="uniform", view=view)


    # ########################################
    # P2L ICP

    start = time.time()
    reg_p2l = ICP_registration(source, target, icp_threshold, reg_ransac.transformation, use_p2l=True)

    print(f"\nP2L ICP took {time.time() - start:.3f} sec.")
    # print(reg_p2l)

    icp_translation, icp_euler = get_transform_vectors(reg_p2l.transformation)
    print(f"[P2L ICP] translate:\t{icp_translation}")
    print(f"[P2L ICP] rotate:\t{icp_euler}")

    visualize([source, target], transformation=reg_p2l.transformation, point_colors="uniform", view=view)


    ########################################
    # EXPORT TRANSFORMED POINT CLOUDS
    transformed_source = transform(source, transformation=reg_p2l.transformation)
    export_pointcloud([transformed_source, target], "export/registration_test.e57")
