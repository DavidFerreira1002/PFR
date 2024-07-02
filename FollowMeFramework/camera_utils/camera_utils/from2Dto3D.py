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
import sys

import open3d as o3d
import cv2
import numpy as np
import math

_LARGE_MASK_AREA_THRESH = 120000

def extract_contours(rgb):
    """
    Extract the contours from a rgb image.

    @param rgb: the image

    @return:
        A list with contours inside
    """
    gray_cluster = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
    ret, tresh = cv2.threshold(gray_cluster, 127, 255, 0)
    cv2.imshow("grey image for contour",tresh)
    cv2.waitKey(1)
    cnt, _ = cv2.findContours(tresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    return cnt


def compute_mu(cnt, rgb=None, depth=None):
    '''
    compute the rgb centroid from contours
    rgb and depth are needed for their shape to scale the point

    @param cnt:   the contours
    @param rgb:   (None) the image
    @param depth: (None) the depth image


    @return:

        A list containing position [X, Y] on image plane
    '''

    x_sum, y_sum = 0, 0
    if len(cnt) > 0:
        for cont in cnt[0]:
            x_sum += cont[0, 0]
            y_sum += cont[0, 1]

        mu = [int(x_sum / cnt[0].shape[0]), int(y_sum / cnt[0].shape[0])]
        if rgb is not None and depth is not None:
            new_x = (float(mu[0]) / rgb.shape[0]) * depth.shape[0]
            new_y = (float(mu[1]) / rgb.shape[1]) * depth.shape[1]
        else:
            new_x = mu[0]
            new_y = mu[1]

        mu = [round(new_x), round(new_y)]
    else:
        mu = [0,0]
        print("No contours found, setting mu to 0")
        cv2.imshow("No contours rgb", rgb)
        cv2.waitKey(0)
        if rgb is not None and depth is not None:
            new_x = (float(mu[0]) / rgb.shape[0]) * depth.shape[0]
            new_y = (float(mu[1]) / rgb.shape[1]) * depth.shape[1]
        else:
            new_x = mu[0]
            new_y = mu[1]

    return mu


def compute_centroid_from_mask(mask):
    """
    Compute centroids given a binary mask.

    @param mask: binary mask (only 0 and 1) in numpy array
    """
    _num_cc, cc_labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
    if stats[1:, -1].size == 0:
        return
    largest_component_id = np.argmax(stats[1:, -1]) + 1

    # draw text on the largest component, as well as other very large components.
    for cid in range(1, _num_cc):
        if cid == largest_component_id or stats[cid, -1] > _LARGE_MASK_AREA_THRESH:
            # median is more stable than centroid
            center = np.median((cc_labels == cid).nonzero(), axis=1)[::-1]

    return center


def compute_angle(mu, cnt):
    '''
    Compute the angle of the contoured object using image inertia

    @param mu:      the image plane centroid [X, Y]
    @param cnt:     object contours

    @return:
        the contours angle [rad] extracted from inertia
    '''
    Ixx, Ixy, Iyy = 0, 0, 0
    for cont in cnt[0]:
        Ixx += pow(cont[0, 0] - mu[0], 2)
        Ixy += (cont[0, 0] - mu[0]) * (cont[0, 1] - mu[1])
        Iyy += pow(cont[0, 1] - mu[1], 2)

    alpha = (0.5 * math.atan2((2 * Ixy), (Ixx - Iyy)))

    return alpha


def compute_angle_from_rgb(rgb, depth):
    '''
    Compute the angle of the object from the rgb and depth information


    @param rgb:    the rgb image
    @param depth:  the depth image

    @return:
        the angle [rad] of the object
    '''
    cnt = extract_contours(rgb)
    mu = compute_mu(cnt, rgb, depth)
    alpha = compute_angle(mu, cnt)

    return alpha


def compute_2dvector_angle(vector):
    # turn the vector when it overcome 90 degrees
    if vector[0] < 0:
        vector = -vector
    vec_norm = np.linalg.norm(vector)
    x_norm = vector[0]/vec_norm
    y_norm = vector[1]/vec_norm
    angle = math.atan2(-y_norm, x_norm)  # the minus in y is due to the coordinate changes of opencv (y points downward)
    return angle


def find_vertices_from_mask(box):
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


def compute_angle_from_mask(mask):
    '''
    Compute the angle using the vertices of the mask

    @param mask:   the object(s) mask(s)

    @return:
         the angle [rad] of the objects
    '''

    cnt, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    max_area = 0
    max_cnt = cnt[0]

    for cont in cnt:
        if cv2.contourArea(cont) > max_area:
            max_area = cv2.contourArea(cont)
            max_cnt = cont

    rect = cv2.minAreaRect(max_cnt)
    box = cv2.boxPoints(rect)
    box = np.int0(box)  # turn into ints

    vertices = find_vertices_from_mask(box)

    up_vector = vertices['UR'] - vertices['UL']
    right_vector = vertices['UR'] - vertices['DR']

    # use longer side to compute x angle
    if np.linalg.norm(up_vector) < np.linalg.norm(right_vector):
        angle = compute_2dvector_angle(right_vector)
    else:
        angle = compute_2dvector_angle(up_vector)

    return angle


def compute_centroids(rgb, depth, mask, intrinsics, use_pcd=True):
    '''
    compute the 3D centroid(s) and angle(s) from mask(s)

    @param rgb:        rgb image
    @param depth:      depth image
    @param mask:       the object mask (could be also a list of masks [mask1, mask2, ...]
    @param intrinsics: the camera intrinsic parameters
    @param use_pcd:    [bool] if true use pcd to compute centroids otherwise use only depth

    @return:
        A list of the form [[x, y, z], theta], [x, y, z], theta], ...] which contains the 3D
        centroids and angles of the objects present in the masks
    '''

    if not rgb[:, :, 0].shape == depth.shape:
        sys.exit("\nfrom2Dto3D/compute_dimensions function: rgb and depth shapes are different."
                 " They should have equal dimensions.\n")
    focal_length = [intrinsics['fx'], intrinsics['fy']]
    principal_point = [intrinsics['px'], intrinsics['py']]

    if mask is None or mask.shape[0] == 0:
        points_and_angles = [[[0, 0, 0], 0]]
        return points_and_angles
    # convert image to np array
    rgb = np.asarray(rgb)
    depth = np.asarray(depth, np.uint16)
    mask = np.asarray(mask, np.uint8)

    points_and_angles = []

    if mask.shape == depth.shape:
        mask = np.expand_dims(mask, axis=0)

    if use_pcd:

        intrinsic = o3d.camera.PinholeCameraIntrinsic()
        intrinsic.set_intrinsics(depth.shape[1], depth.shape[0], focal_length[0], focal_length[1],
                                 principal_point[0], principal_point[1])

        for j in range(mask.shape[0]):
            rgb_new = rgb.copy()
            depth_new = depth.copy()
            # extract mask from rgb and depth
            for i in range(3):
                rgb_new[:, :, i] = np.multiply(rgb[:, :, i], mask[j, :, :])
            depth_new = np.multiply(depth_new, mask[j, :, :])

            rgb_new = rgb_new.astype(np.uint8)
            depth_new = depth_new.astype(np.uint16)

            # compute angle
            # Find indices of non-zero elements
            non_zero_indices = np.nonzero(mask[j])

            # Extract non-zero elements using the indices
            non_zero_elements = mask[j][non_zero_indices]
            #print(non_zero_elements)
            if len(non_zero_elements) > 0:
                alpha = compute_angle_from_mask(mask[j])
            else:
                alpha = 0.0
                print("Mask was all 0, alpha set as 0.0")

            # compute center
            rgb_o3d = o3d.geometry.Image(rgb_new)
            depth_o3d = o3d.geometry.Image(depth_new)

            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_o3d, depth_o3d, depth_trunc=12)
            pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)

            center = pcd.get_center()

            points_and_angles.append([center, alpha])

    else:

        for j in range(mask.shape[0]):
            rgb_new = rgb.copy()
            # extract mask from rgb
            for i in range(3):
                rgb_new[:, :, i] = np.multiply(rgb[:, :, i], mask[j, :, :])

            cnt = extract_contours(rgb_new)
            mu = compute_mu(cnt, rgb_new, depth)

            # compute 3D spatial coordinates with respect to the camera reference frame
            z = depth[mu[1], mu[0]] * 0.001
            x = ((mu[0] - principal_point[0]) * z) / focal_length[0]
            y = ((mu[1] - principal_point[1]) * z) / focal_length[1]

            center = [x, y, z]

            # Find indices of non-zero elements
            non_zero_indices = np.nonzero(mask[j])

            # Extract non-zero elements using the indices
            non_zero_elements = mask[j][non_zero_indices]
            #print(non_zero_elements)
            if len(non_zero_elements) > 0:
                alpha = compute_angle_from_mask(mask[j])
            else:
                alpha = 0.0
                print("Mask was all 0, alpha set as 0.0")

            # append the centroid and the angle of the considered mask in the list.
            points_and_angles.append([center, alpha])

    return points_and_angles
