'''----------------------------------------------------------------------------------------------------------------------------------
# Copyright (C) 2022
#
# author: Federico Rollo
# mail: rollo.f96@gmail.com
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
import pdb

import numpy as np
import open3d as o3d
import math
import cv2
import sys


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


def plot_3dpcd_with_vertices(vertices_pcd, real_vertices, inlier_cloud, pcd, centroid, dim_z):
    pdb.set_trace()  # used otherwise closing a launch file while plotting a pointcloud with open3d could be painful
    vertices_pcd['DL'].paint_uniform_color([0, 0, 1.0])  # blue
    vertices_pcd['DR'].paint_uniform_color([1.0, 1.0, 0])  # yellow
    vertices_pcd['UL'].paint_uniform_color([1.0, 0, 0])  # red
    vertices_pcd['UR'].paint_uniform_color([0, 1.0, 0])  # green
    refined_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector([real_vertices['UL'], real_vertices['UR'],
                                                                      real_vertices['DL'], real_vertices['DR']]))
    upper_centroid = o3d.geometry.PointCloud(o3d.utility.Vector3dVector([centroid]))
    upper_centroid.paint_uniform_color([1.0, 0, 0])
    upper_centroid.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    centroid[2] += dim_z/2
    centroid = o3d.geometry.PointCloud(o3d.utility.Vector3dVector([centroid]))
    centroid.paint_uniform_color([1.0, 0, 0])
    centroid.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    for point in vertices_pcd.values():
        point.paint_uniform_color([0, 0, 1.0])
        point.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    inlier_cloud.paint_uniform_color([0, 1.0, 0])
    inlier_cloud.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    # inlier_cloud = inlier_cloud.voxel_down_sample(voxel_size=0.005)
    refined_pcd.paint_uniform_color([1.0, 0, 0])
    refined_pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    pcd.paint_uniform_color([0.5, 0.5, 0.5])
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    # pcd = pcd.voxel_down_sample(voxel_size=0.005)
    o3d.visualization.draw_geometries([pcd, inlier_cloud,  vertices_pcd['UL'], vertices_pcd['UR'],
                                       vertices_pcd['DL'], vertices_pcd['DR'], refined_pcd, upper_centroid, centroid])


def compute_2dvector_angle(vector):
    # turn the vector when it overcome 90 degrees
    if vector[0] < 0:
        vector = -vector
    vec_norm = np.linalg.norm(vector)
    x_norm = vector[0] / vec_norm
    y_norm = vector[1] / vec_norm
    angle = math.atan2(-y_norm, x_norm)  # the minus in y is due to the coordinate changes of opencv (y points downward)
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


def compute_box_pose_and_dimensions(rgb, depth, mask, intrinsics, cam2plane_distance, display=True, only_pose=False):

    # convert image to np array todo: control if is useless
    rgb = np.asarray(rgb)
    depth = np.asarray(depth, np.uint16)
    mask = np.asarray(mask, np.uint8)

    new_rgb = rgb.copy()

    # assert rgb and depth are the same resolution
    if not rgb[:, :, 0].shape == depth.shape:
        sys.exit("\nfrom2Dto3D/compute_dimensions function: rgb and depth shapes are different."
                 " They should have equal dimensions.\n")

    # assert a real mask is given not only noise ore empty
    assert not(mask is None or mask.shape[0] == 0 or np.sum(mask) < 500)


    # set intrinsics for open3d
    width = max(depth.shape[0], depth.shape[1])
    height = min(depth.shape[0], depth.shape[1])
    intrinsic = o3d.camera.PinholeCameraIntrinsic()
    intrinsic.set_intrinsics(width, height, intrinsics['fx'], intrinsics['fy'], intrinsics['px'], intrinsics['py'])

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
    pcd = pcd.voxel_down_sample(voxel_size=0.002)

    # -------------PLANE SEGMENTATION TO REMOVE NOISY OUTLIERS----------------

    try: 
        plane_model, inliers = pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
    except Exception:
        print('\033[91mError during plane generation\033[0m')
        raise AssertionError  # to warn the main that the function has not worked
    if o3d.__version__ == '0.9.0.0':
        inlier_cloud = pcd.select_down_sample(inliers)
        # outlier_cloud = pcd.select_down_sample(inliers, invert=True)
    else:
        inlier_cloud = pcd.select_by_index(inliers)
        # outlier_cloud = pcd.select_by_index(inliers, invert=True)

    inlier_cloud = inlier_cloud.voxel_down_sample(voxel_size=0.002)

    # -------------- CENTROID and ANGLE-------------------
    # box upper surface centroid
    centroid = inlier_cloud.get_center()
    angle = compute_angle_from_vertices(vertices)

    # --------------- DIMENSIONS ------------------------

    if only_pose:  # if you want only posisiton and orientation
        box_pose_and_dim = {'dim': [0, 0, 0], 'centroid': centroid, 'angle': angle}

    else:  # if you want also dimensions
        # 3D vertices searching
        vertices_pcd = {}
        for vertex_id, point in vertices.items():

            # initialize mask and rgb for points
            point_mask = np.zeros(mask.shape)
            point_rgb = np.zeros(new_rgb.shape)

            if point[1] < 0 or point[1] >= height or point[0] < 0 or point[0] >= width:
                   print("\033[91mMask out of field of view\033[0m")
                   raise AssertionError

            # change a single value of the mask to one in order to have only the vertex position on the mask
            point_mask[point[1], point[0]] = 1

            # lets extract the point position
            point_depth = np.multiply(depth, point_mask)

            # If the point found on the depth has a zero value we need to found another value different from zero around
            # the point found
            counter = 0
            while np.sum(point_depth) == 0:
                # print('entering while with %s' % key)
                counter += 1
                if counter == 6:
                    print("\033[91mNo good vertex has been found.\033[0m")
                    raise AssertionError
                # search around the vertex point for a depth value different from zero
                for i in range(-counter, counter + 1):
                    for j in range(-counter, counter + 1):
                        if point[1] + i < 0 or point[1] + i >= height or point[0] + j < 0 or point[0] + j >= width:
                            continue
                        point_mask = np.zeros(mask.shape)
                        point_mask[point[1] + i, point[0] + j] = 1
                        point_depth = np.multiply(depth, point_mask)
                        if np.sum(point_depth) != 0:
                            break
                    else:
                        continue
                    break

            # extract rgb
            for j in range(3):
                point_rgb[:, :, j] = np.multiply(new_rgb[:, :, j], point_mask)

            point_rgb = np.array(point_rgb, dtype=np.uint8)
            point_depth = np.array(point_depth, dtype=np.uint16)

            point_rgb = o3d.geometry.Image(point_rgb)
            point_depth = o3d.geometry.Image(point_depth)

            point_rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(point_rgb, point_depth)
            point_cloud = o3d.geometry.PointCloud.create_from_rgbd_image(point_rgbd, intrinsic)

            vertices_pcd[vertex_id] = point_cloud

        # ---------------FIND THE NEAREST POINT TO THE 3D VERTICES IN THE SEGMENTED PLANE---------------

        real_vertices = {}
        min_distance = {}
        for vertex_id in vertices_pcd.keys():
            min_distance[vertex_id] = 1000000

        for point in inlier_cloud.points:
            for vertex_id, pcd_point in vertices_pcd.items():
                # pdb.set_trace()
                pcd_point = pcd_point.points[0][:2]
                distance = np.linalg.norm(pcd_point - point[:2])

                if distance < min_distance[vertex_id]:
                    min_distance[vertex_id] = distance
                    real_vertices[vertex_id] = point

        # ---------------------COMPUTE DIMENSIONS-----------------------
        left_side = np.linalg.norm(real_vertices['DL'] - real_vertices['UL'])
        down_side = np.linalg.norm(real_vertices['DL'] - real_vertices['DR'])
        up_side = np.linalg.norm(real_vertices['UL'] - real_vertices['UR'])
        right_side = np.linalg.norm(real_vertices['UR'] - real_vertices['DR'])

        side1 = np.mean([left_side, right_side])
        side2 = np.mean([down_side, up_side])

        # min_z = np.min(np.asarray(inlier_cloud.points)[:, 2])
        min_z = centroid[2]

        dim_x = max(side1, side2)
        dim_y = min(side1, side2)
        # # dimZ = max_z - min_z
        dim_z = cam2plane_distance - min_z

        dim = [dim_x, dim_y, dim_z]

        # create a return structure
        box_pose_and_dim = {'dim': dim, 'centroid': centroid, 'angle': angle}

        # -------- Print plane segmented with real 3d vertices plus original pcd and vertices----------
        # plot_3dpcd_with_vertices(vertices_pcd, real_vertices, inlier_cloud, pcd, centroid, dim_z)

        # --------------------------- print and return values -----------------------------------
        # print('Length: %.2f cm, Width: %.2f cm, Height: %.2f cm, ' % (dim[0] * 100, dim[1] * 100, dim[2] * 100))
        # print('X: %.2f m, Y: %.2f m, Z: %.2f m' % (centroid[0], centroid[1], centroid[2]))
        # print('th: %.2f rad = %.2f°' % (angle, angle * 180 / 3.14))

    return box_pose_and_dim

