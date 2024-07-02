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
import numpy as np
from camera_utils.cameras.CameraInterface import Camera
import pyrealsense2 as rs
import open3d as o3d

class IntelRealsense(Camera):

    def __init__(self, camera_resolution=Camera.Resolution.HD, fps=30,
                 serial_number="", depth_in_meters=False):

        self.camera_brand = "Intel Realsense"
        Camera.__init__(self, camera_resolution, fps, serial_number)

        # start camera
        self.pipeline = rs.pipeline()
        config = rs.config()

        if self.serial_number != "":
            config.enable_device(self.serial_number)

        # set resolutions
        # RGB max 1920x1080 at 30 fps
        # DEPTH max 1280x720 at 90 fps
        if self.camera_resolution == Camera.Resolution.LOW:
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, self.fps) 
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, self.fps)     
        elif self.camera_resolution == Camera.Resolution.HD:
            config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, self.fps)  
            config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, self.fps)         
        elif self.camera_resolution == Camera.Resolution.FullHD:
            config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, self.fps)  
            config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, self.fps)     
            print(
                "\033[93mWARNING! RGB resolution is set to FullHD while Depth is set to HD."
                "\nIf you want equal resoultions you need to set camera_resolution to HD or LOW.\033[0m"
            ) 
        else:
            print("\033[91mERROR: Wrong resolution set for camera. Choose between LOW, HD, FullHD.\033[0m")
            exit()   


        # Start streaming
        try:
            cfg = self.pipeline.start(config)
        except RuntimeError:
            print("\n\033[1;31;40mError during camera initialization.\nMake sure to have set the right RGB camera resolution. Some cameras doesn't have FullHD resolution (e.g. Intel Realsense D455).\nIf you have connected more cameras make sure to insert the serial numbers to distinguish cameras during initialization.\033[0m\n")
            exit(1)

        # setting camera name from camera info
        name_profile = config.resolve(self.pipeline)
        device = name_profile.get_device()
        self.camera_name = device.get_info(rs.camera_info.name)
        self.serial_number = device.get_info(rs.camera_info.serial_number)

        profile = cfg.get_stream(rs.stream.color)
        intr = profile.as_video_stream_profile().get_intrinsics()
        self.intr = {'fx': intr.fx, 'fy': intr.fy, 'px': intr.ppx, 'py': intr.ppy, 'width': intr.width, 'height': intr.height}
        
        self.o3d_intr = o3d.camera.PinholeCameraIntrinsic()
        self.o3d_intr.set_intrinsics(self.intr["width"], self.intr["height"], self.intr['fx'], self.intr['fy'], self.intr['px'], self.intr['py'])

        if depth_in_meters:
            self.mm2m_conversion = 1000
        else:
            self.mm2m_conversion = 1

        print("%s (S/N: %s) camera configured.\n" % (self.camera_name, self.serial_number))

    def __del__(self):
        try:
            self.pipeline.stop()
            print("%s (S/N: %s) camera closed" % (self.camera_name, self.serial_number))
        except RuntimeError as ex:
            print("\033[0;33;40mException (%s): %s\033[0m" % (type(ex).__name__, ex))
        

    def get_rgb(self):
        '''
        :return: An rgb image as numpy array
        '''
        color_frame = None
        while not color_frame:
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()

        color_frame = np.asanyarray(color_frame.get_data())
        return color_frame

    def get_depth(self):
        '''
        :return: A depth image (1 channel) as numpy array
        '''
        depth_frame = None
        while not depth_frame:
            frames = self.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
        depth_frame = np.asanyarray(depth_frame.get_data(), dtype=np.uint16) 
        return np.asanyarray(depth_frame / self.mm2m_conversion, dtype=np.uint16)

    def get_frames(self):
        '''
        :return: rgb, depth images as numpy arrays
        '''
        depth_frame_cam, color_frame_cam = None, None
        while not color_frame_cam or not depth_frame_cam:
            frames = self.pipeline.wait_for_frames()
            depth_frame_cam = frames.get_depth_frame()
            color_frame_cam = frames.get_color_frame()
        depth_frame = np.asanyarray(depth_frame_cam.get_data()) / self.mm2m_conversion
        color_frame = np.asanyarray(color_frame_cam.get_data())

        return color_frame, np.asanyarray(depth_frame / self.mm2m_conversion, dtype=np.uint16)

    def get_aligned_frames(self):
        '''
        :return: rgb, depth images aligned with post-processing as numpy arrays
        '''
        depth_frame_cam, color_frame_cam = None, None
        while not color_frame_cam or not depth_frame_cam:
            frames = self.pipeline.wait_for_frames()
            align = rs.align(rs.stream.color)
            aligned_frames = align.process(frames)
            color_frame_cam = aligned_frames.first(rs.stream.color)
            depth_frame_cam = aligned_frames.get_depth_frame()
        depth_frame = np.asanyarray(depth_frame_cam.get_data()) / self.mm2m_conversion
        color_frame = np.asanyarray(color_frame_cam.get_data())

        return color_frame, np.asanyarray(depth_frame / self.mm2m_conversion, dtype=np.uint16)

    def get_pcd(self, depth_truncation=5.0):
        '''
        :param depth_truncation: [m] only depth values smaller than this distance will be considered
        
        :return: open3d pcd 
        '''
        rgb, depth = self.get_frames()

        depth = o3d.geometry.Image(depth)
        rgb = o3d.geometry.Image(rgb)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb, depth, depth_trunc=depth_truncation)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, self.o3d_intr)

        return pcd
        
    def set_option(self, option, value):
        '''
        :param option: the option to be set (rs.option.OPTION_NAME)
        :param value: the value of the option
        '''
        option_name = str(option)
        try:
            sensor = self.pipeline.get_active_profile().get_device().query_sensors()[1]
            sensor.set_option(option, value)
            option_name = str(option).replace('option.', '').upper()
            print("Option %s changed to value: %d" % (option_name, int(value)))
        except TypeError as ex:
            print("\033[0;33;40m Exception (%s): the option %s has NOT been set." % (type(ex).__name__, option_name))

    def get_option(self, option):
        '''
        :param option: the option to be got (rs.option.OPTION_NAME)
        '''
        option_name = str(option).replace('option.', '').upper()
        try:
            sensor = self.pipeline.get_active_profile().get_device().query_sensors()[1]
            value = sensor.get_option(option)
            print("Option %s value: %d" % (option_name, int(value)))
        except TypeError as ex:
            print("\033[1;33;40m Exception (%s): the option %s has NOT been set." % (type(ex).__name__, option_name))
