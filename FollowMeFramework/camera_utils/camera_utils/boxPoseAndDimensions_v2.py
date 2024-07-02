'''----------------------------------------------------------------------------------------------------------------------------------
# Copyright (C) 2022
#
# author: Fabio Amadio, Federico Rollo
#
# Institute: Leonardo Labs (Leonardo S.p.a - Istituto Italiano di tecnologia)
#
# This file is part of camera_utils. <https://github.com/IASRobolab/camera_utils>
#
# camera_utils is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# camera_utils is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License. If not, see http://www.gnu.org/licenses/
---------------------------------------------------------------------------------------------------------------------------------'''
import numpy as np
import open3d as o3d
import math
import cv2
import sys
import copy
import time
from scipy.spatial.transform import Rotation as R
from termcolor import colored


def display_inlier_outlier(cloud, ind, window_name):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)

    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud],
                                        window_name = window_name + ' [Inliers (gray) - Outliers (red)]')



def plot_mask_img_with_vertices(mask, vertices):
    rect_mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

    # Plot the mask with vertices coloured vertices
    rect_mask = cv2.circle(rect_mask, (vertices['UL'][0], vertices['UL'][1]),
                           radius=3, color=(0, 0, 255), thickness=2)  # red
    rect_mask = cv2.circle(rect_mask, (vertices['UR'][0], vertices['UR'][1]),
                           radius=3, color=(0, 255, 0), thickness=2)  # green
    rect_mask = cv2.circle(rect_mask, (vertices['DL'][0], vertices['DL'][1]),
                           radius=3, color=(255, 0, 0), thickness=2)  # blue
    rect_mask = cv2.circle(rect_mask, (vertices['DR'][0], vertices['DR'][1]),
                           radius=3, color=(0, 255, 255), thickness=2)  # yellow
    # add text to vertices
    text_shift = 15
    cv2.putText(rect_mask, text='UR', org=(vertices['UR'][0], vertices['UR'][1] - text_shift),
                fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.8, color=(255, 255, 255),
                thickness=2, lineType=cv2.LINE_AA)
    cv2.putText(rect_mask, text='UL', org=(vertices['UL'][0] - text_shift, vertices['UL'][1] - text_shift),
                fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.8, color=(255, 255, 255),
                thickness=2, lineType=cv2.LINE_AA)
    cv2.putText(rect_mask, text='DR', org=(vertices['DR'][0], vertices['DR'][1] + text_shift * 2),
                fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.8, color=(255, 255, 255),
                thickness=2, lineType=cv2.LINE_AA)
    cv2.putText(rect_mask, text='DL', org=(vertices['DL'][0] - text_shift, vertices['DL'][1] + text_shift * 2),
                fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.8, color=(255, 255, 255),
                thickness=2, lineType=cv2.LINE_AA)

    cv2.namedWindow('Rect Mask', cv2.WINDOW_NORMAL)
    cv2.imshow('Rect Mask', rect_mask)



def compute_2dvector_angle(vector):
    # turn the vector when it overcome 90 degrees
    if vector[0] < 0:
        vector = -vector
    vec_norm = np.linalg.norm(vector)
    x_norm = vector[0] / vec_norm
    y_norm = vector[1] / vec_norm
    # - sign is due to the opencv coordinate change (y points downward)
    angle = math.atan2(-y_norm, x_norm)  
    return angle


def find_vertices_from_box(box):
    """
    Find the vertices of the rectangle containing the object in the mask

    @param box: the box used to find the vertices

    @return: A dictionary containing the X,Y image position of the four vertices
    {"UL": ul, "DL": dl, "UR": ur, "DR": dr}
    """
    box = box[np.argsort(box[:, 1])]  # sort box vertices with respect y position

    if box[0, 0] < box[1, 0]:
        ul = box[0, :]
        ur = box[1, :]
    else:
        ur = box[0, :]
        ul = box[1, :]
    if box[2, 0] < box[3, 0]:
        dl = box[2, :]
        dr = box[3, :]
    else:
        dl = box[3, :]
        dr = box[2, :]

    vertices = {"UL": ul, "DL": dl, "UR": ur, "DR": dr}
    return vertices


def compute_angle_from_vertices(vertices):
    '''
    Compute the angle using the vertices of the mask

    @param vertices:   the four box vertices

    @return:
         the angle [rad] of the objects
    '''

    up_vector = vertices['UR'] - vertices['UL']
    right_vector = vertices['UR'] - vertices['DR']

    # use longer side to compute x angle
    if np.linalg.norm(up_vector) < np.linalg.norm(right_vector):
        angle = compute_2dvector_angle(right_vector)
    else:
        angle = compute_2dvector_angle(up_vector)

    return angle


def compute_box_pose_and_dimensions(rgb, depth, mask, intrinsics, cam2plane_distance,
                                    display=True, only_pose=False, voxel_size = 0.002,
                                    plane_table_angle_treshold = 15, max_box_height = 0.5):
    rgb = np.asarray(rgb)
    depth = np.asarray(depth, np.uint16)
    mask = np.asarray(mask, np.uint8)

    rgb_scene = o3d.geometry.Image(rgb.copy())
    depth_scene = o3d.geometry.Image(depth.copy())

    new_rgb = rgb.copy()

    # assert rgb and depth are the same resolution
    if not rgb[:, :, 0].shape == depth.shape:
        sys.exit("\nfrom2Dto3D/compute_dimensions function: rgb and depth shapes are different."
                 " They should have equal dimensions.\n")

    # assert a real mask is given not only noise or empty
    assert not(mask is None or mask.shape[0] == 0 or np.sum(mask) < 500)


    # set intrinsics for open3d
    width = max(depth.shape[0], depth.shape[1])
    height = min(depth.shape[0], depth.shape[1])
    intrinsic = o3d.camera.PinholeCameraIntrinsic()
    intrinsic.set_intrinsics(width, height, intrinsics['fx'],
                                            intrinsics['fy'], 
                                            intrinsics['px'],
                                            intrinsics['py'])

    # extract rough mask contours
    cnt, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # search for bigger contour (probably the box)
    max_area = 0
    max_cnt = cnt[0]
    for cont in cnt:
        if cv2.contourArea(cont) > max_area:
            max_area = cv2.contourArea(cont)
            max_cnt = cont

    # find the min enclosing rectagle of the mask to have a smoother mask
    rect = cv2.minAreaRect(max_cnt)
    box = cv2.boxPoints(rect)
    box = np.int0(box)  # turn into ints

    mask = np.zeros(mask.shape, dtype=np.uint8)
    cv2.fillPoly(mask, pts=[box], color=(255, 255, 255))
    rect_mask = np.array(mask, dtype=np.uint8)

    # order the vertices
    vertices = find_vertices_from_box(box)

    # plot the mask with vertices MASK PLOTTING
    if display:
        plot_mask_img_with_vertices(rect_mask, vertices)

    # ------------- BOX POINT CLOUD ---------------

    # mask binarization
    rect_mask[rect_mask > 254] = 1

    # extract mask from rgb
    for i in range(3):
        rgb[:, :, i] = np.multiply(rgb[:, :, i], rect_mask)
    # extract mask from depth
    depth = np.multiply(depth, rect_mask)

    # create open3d point cloud of the box
    depth = o3d.geometry.Image(depth)
    rgb = o3d.geometry.Image(rgb)
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb, depth)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)
    pcd = pcd.voxel_down_sample(voxel_size=voxel_size)

    
    
    # print("Remove points outside of allowed z-range")
    # # points that are too far
    # ind1 = np.where(np.asarray(pcd.points)[:,2] < cam2plane_distance)[0]
    # cl = pcd.select_by_index(ind1)
    # if display:
    #     display_inlier_outlier(pcd, ind1, window_name = 'Cut below')
    # pcd = cl

    # # points that are too close
    # ind2 = np.where(np.asarray(pcd.points)[:,2] > cam2plane_distance - max_box_height)[0]
    # cl = pcd.select_by_index(ind2)
    # if display:
    #     display_inlier_outlier(pcd, ind2, window_name = 'Cut above')
    # pcd = cl

    # -------------PLANE SEGMENTATION TO REMOVE NOISY OUTLIERS----------------

    plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                             ransac_n=3,
                                             num_iterations=1000)

    table_normal = np.array([0., 0., 1.])
    plane_normal  = plane_model[:3]
    plane_normal_norm = np.linalg.norm(plane_normal)
    cos_plane_table_angle= np.dot(table_normal, plane_normal.T) / (1 * plane_normal_norm)
    plane_table_angle = np.arccos(cos_plane_table_angle) * 180 / np.pi

    print(colored("\nPlane-Table angle = "+str(plane_table_angle)+" [deg]", 'yellow'))

    if o3d.__version__ == '0.9.0.0':
        inlier_cloud = pcd.select_down_sample(inliers)
        outlier_cloud = pcd.select_down_sample(inliers, invert=True)
    else:
        inlier_cloud = pcd.select_by_index(inliers)
        outlier_cloud = pcd.select_by_index(inliers, invert=True)

    inlier_cloud = inlier_cloud.voxel_down_sample(voxel_size=voxel_size)
    outlier_cloud = outlier_cloud.voxel_down_sample(voxel_size=voxel_size)

    if display:
        o3d.visualization.draw_geometries([inlier_cloud.paint_uniform_color([0, 1., 0]),
                                          outlier_cloud.paint_uniform_color([1., 0, 0])],
                                          window_name = 'Plane (green) - Outliers (red)')
    
    # -------------- CENTROID and ANGLE-------------------
    angle = -compute_angle_from_vertices(vertices)
    if angle > 0:
        # change angle to have x-axis towards - camera y-axis
        angle = -np.pi + angle
    centroid = inlier_cloud.get_center()

    if abs(plane_table_angle) <= plane_table_angle_treshold:    
        z = centroid[2]
    else:
        print(colored("Plane inclination threshold not met", 'red'))
        inlier_points = np.asarray(inlier_cloud.points)
        z = np.min(inlier_points[:,2])

    vertex_3d = {}
    for vertex_id, point in vertices.items():
        x = ((point[0] - intrinsics['px']) * z) / intrinsics['fx']
        y = ((point[1] - intrinsics['py']) * z) / intrinsics['fy']
        vertex_3d[vertex_id] = np.array([x, y, z])


    diag1 = vertex_3d['UL'] - vertex_3d['DR']
    centroid = vertex_3d['DR'] + diag1 / 2
    # diag2 = vertex_3d['UR'] - vertex_3d['DL']
    # centroid = vertex_3d['DL'] + diag2 / 2



    if display:
        cam_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
        quat = (0, 0, np.sin(angle / 2), np.cos(angle / 2)) # aligned with the camera (z-axis downards)
        camera_R_box = R.from_quat(quat) # [qx, qy, qz, qw]
        camera_H_box = np.eye(4)
        camera_H_box[:3,:3] = camera_R_box.as_matrix()
        camera_H_box[:3, 3] = centroid
        centroid_frame = copy.deepcopy(cam_frame).transform(camera_H_box)

        cam_frame_v = o3d.geometry.TriangleMesh.create_coordinate_frame(size = 0.1)
        camera_H_vert = np.eye(4)
        camera_H_vert[:3,:3] = camera_R_box.as_matrix()
        camera_H_vert[:3, 3] = vertex_3d['UL']
        UL_frame =  copy.deepcopy(cam_frame_v).transform(camera_H_vert)

        camera_H_vert[:3, 3] = vertex_3d['DL']
        DL_frame =  copy.deepcopy(cam_frame_v).transform(camera_H_vert)

        camera_H_vert[:3, 3] = vertex_3d['UR']
        UR_frame =  copy.deepcopy(cam_frame_v).transform(camera_H_vert)

        camera_H_vert[:3, 3] = vertex_3d['DR']
        DR_frame =  copy.deepcopy(cam_frame_v).transform(camera_H_vert)


        o3d.visualization.draw_geometries([pcd, inlier_cloud.paint_uniform_color([0, 1., 0]),
                                        centroid_frame, UL_frame, DL_frame, UR_frame, DR_frame],
                                        window_name = 'Fitted plane, centroid and vertices')

    # --------------- DIMENSIONS ------------------------

    if only_pose:  # if you want only posisiton and orientation
        box_pose_and_dim = {'dim': [0, 0, 0], 'centroid': centroid, 'angle': angle}

    else: 
        # ---------------------COMPUTE DIMENSIONS-----------------------
        left_side = np.linalg.norm(vertex_3d['DL'] - vertex_3d['UL'])
        down_side = np.linalg.norm(vertex_3d['DL'] - vertex_3d['DR'])
        up_side = np.linalg.norm(vertex_3d['UL'] - vertex_3d['UR'])
        right_side = np.linalg.norm(vertex_3d['UR'] - vertex_3d['DR'])

        side1 = np.mean([left_side, right_side])
        side2 = np.mean([down_side, up_side])

        min_z = centroid[2]

        dim_x = max(side1, side2)
        dim_y = min(side1, side2)
        dim_z = cam2plane_distance - min_z

        dim = [dim_x, dim_y, dim_z]

        # create a return structure
        box_pose_and_dim = {'dim': dim, 'centroid': centroid, 'angle': angle}

    # -------- Print plane segmented with real 3d vertices plus original pcd and vertices----------
    if display:
        rgbd_scene = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_scene, depth_scene)
        pcd_scene = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_scene, intrinsic)
        pcd_scene.voxel_down_sample(voxel_size=voxel_size)
        cam_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size = 0.3)
        centroid_frame = copy.deepcopy(cam_frame).transform(camera_H_box)

        o3d.visualization.draw_geometries([pcd, inlier_cloud,  pcd_scene, cam_frame,
                                            centroid_frame],
                                            window_name = 'Scene - Fitted plane - Centroid')

    # --------------------------- print and return values -----------------------------------
    # print('Length: %.2f cm, Width: %.2f cm, Height: %.2f cm, ' % (dim[0] * 100, dim[1] * 100, dim[2] * 100))
    # print('X: %.2f m, Y: %.2f m, Z: %.2f m' % (centroid[0], centroid[1], centroid[2]))
    # print('th: %.2f rad = %.2f°' % (angle, angle * 180 / 3.14))

    return box_pose_and_dim

