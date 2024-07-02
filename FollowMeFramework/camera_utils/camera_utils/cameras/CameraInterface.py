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
from enum import Enum
import open3d as o3d

class Camera:

    class Resolution(Enum):
        HD = 0
        FullHD = 1
        QHD = 2
        LOW = 3

    camera_name = ""
    camera_brand = ""
    intr = []
    pipeline = 0
    camera_resolution = 0
    fps = 0
    serial_number = ""

    o3d_intr = 0

    def __init__(self, camera_resolution=Resolution.HD,  fps=30, serial_number=""):
        self.camera_resolution = camera_resolution
        self.fps = fps
        self.serial_number = serial_number
        

        print("%s initialization" % self.camera_brand)

    def get_intrinsics(self):
        '''
        :return: camera intrinsics
        '''
        return self.intr

    def get_name(self):
        '''
        :return: camera name
        '''
        return self.camera_name

    def get_brand(self):
        '''
        :return: camera camera_brand
        '''
        return self.camera_brand

    def get_serial_number(self):
        '''
        :return: camera serial number
        '''
        return self.serial_number

    def get_fps(self):
        '''
        :return: camera frames per second
        '''
        return self.fps

    def get_rgb(self):
        raise NotImplementedError

    def get_depth(self):
        raise NotImplementedError

    def get_frames(self):
        raise NotImplementedError

    def get_aligned_frames(self):
        raise NotImplementedError
    
    def set_option(self, option, value):
        raise NotImplementedError

    def get_option(self, option):
        raise NotImplementedError

    def get_pcd(self, depth_truncation=5.0, display_pcd=False):
        '''
        :param depth_truncation: [m] only depth values smaller than this distance will be considered
        :param display_pcd: [bool] if true display the pcd in a open3d plot window
        
        :return: open3d pcd 
        '''
        rgb, depth = self.get_frames()

        depth = o3d.geometry.Image(depth)
        rgb = o3d.geometry.Image(rgb)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb, depth, depth_trunc=depth_truncation, convert_rgb_to_intensity=False)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, self.o3d_intr)

        if display_pcd:
            pcd_display = o3d.geometry.PointCloud(pcd)
            pcd_display.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
            o3d.visualization.draw_geometries([pcd_display])

        return pcd