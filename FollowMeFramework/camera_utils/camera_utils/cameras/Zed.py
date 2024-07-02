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
import open3d as o3d

try:
    import pyzed.sl as sl
except ImportError:
    print("\033[91mpyzed.sl module not found. Please install the ZED SDK to use this module.\033[0m")
    exit(1)


class Zed(Camera):

    def __init__(
        self, single_camera_mode=True, camera_resolution=Camera.Resolution.FullHD,
        fps=30, serial_number=""):
        '''
        :param single_camera_mode: boolean to choose if using the camera as a single camera or a dual camera (two images ...)
        :param camera_resolution: the rgb resolution of the camera (e.g., Zed.Resolution.HD)
        :param depth_resolution: the depth resolution of the camera (e.g., Zed.Resolution.HD)
        :param fps: the camera frames per second
        :param serial_number: the camera serial number
        '''
        self.camera_brand = "ZED"
        Camera.__init__(self, camera_resolution, fps, serial_number)

        self.single_camera_mode = single_camera_mode
        self.pipeline = sl.Camera()
        # set initial parameters
        init_params = sl.InitParameters()

        if self.serial_number != "":
            init_params.set_from_serial_number(self.serial_number)

        # set camera resolution
        if self.camera_resolution == Camera.Resolution.LOW:
            init_params.camera_resolution = sl.RESOLUTION.VGA
        elif self.camera_resolution == Camera.Resolution.HD:
            init_params.camera_resolution = sl.RESOLUTION.HD720
        elif self.camera_resolution == Camera.Resolution.FullHD:
            init_params.camera_resolution = sl.RESOLUTION.HD1080
        elif self.camera_resolution == Camera.Resolution.QHD:
            init_params.camera_resolution = sl.RESOLUTION.HD2K
            if self.fps > 15:
                self.fps = 15
        else:
            print("\033[91mERROR: Wrong resolution set for camera. Choose between LOW, HD, FullHD, QHD.\033[0m")
            exit()

        init_params.camera_fps = self.fps

        # start the camera
        err = self.pipeline.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            print("ERROR: problem starting the camera. Check connection and initialization parameters.")
            exit()

        # get intrinsics
        zed_params = self.pipeline.get_camera_information().calibration_parameters
        left_intr = zed_params.left_cam
        
        # set camera name and serial number
        self.camera_name = self.pipeline.get_camera_information().camera_model
        self.serial_number = self.pipeline.get_camera_information().serial_number


        if self.single_camera_mode:
            self.intr = {
                'fx': left_intr.fx, 'fy': left_intr.fy, 
                'px': left_intr.cx, 'py': left_intr.cy, 
                'width': left_intr.image_size.width, 'height': left_intr.image_size.height
            }
            self.o3d_intr = o3d.camera.PinholeCameraIntrinsic()
            self.o3d_intr.set_intrinsics(self.intr["width"], self.intr["height"], self.intr['fx'], self.intr['fy'], self.intr['px'], self.intr['py'])
        else:
            right_intr = zed_params.right_cam
            self.intr = {
                'fx': [left_intr.fx, right_intr.fx],
                'fy': [left_intr.fy, right_intr.fy],
                'px': [left_intr.cx, right_intr.cx],
                'py': [left_intr.cy, right_intr.cy],
                'width': [left_intr.image_size.width, right_intr.image_size.width],
                'height': [left_intr.image_size.height, right_intr.image_size.height]
            }
            self.o3d_intr = o3d.camera.PinholeCameraIntrinsic()
            self.o3d_intr.set_intrinsics(self.intr["width"][0], self.intr["height"][0], self.intr['fx'][0], self.intr['fy'][0], self.intr['px'][0], self.intr['py'][0])


        print("%s (S/N: %s) camera configured.\n" % (self.camera_name, self.serial_number))

    def __del__(self):
        self.pipeline.close()
        print("%s (S/N: %s) camera closed" % (self.camera_name, self.serial_number))

    def get_rgb(self):
        '''
        :return: An rgb image as numpy array or [left, right] rgb images depending on camera mode (single or double)
        '''
        color_frame_left = sl.Mat()
        runtime_parameters = sl.RuntimeParameters()
        if self.pipeline.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            self.pipeline.retrieve_image(color_frame_left, sl.VIEW.LEFT)
        color_frame_left = np.array(color_frame_left.get_data())[:, :, :3]
        if self.single_camera_mode:
            return color_frame_left
        else:
            color_frame_right = sl.Mat()
            if self.pipeline.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                self.pipeline.retrieve_image(color_frame_right, sl.VIEW.RIGHT)
            color_frame_right = np.array(color_frame_right.get_data())[:, :, :3]
            return color_frame_left, color_frame_right

    def get_depth(self):
        '''
        :return: A depth image as numpy array or [left, right] depth images depending on camera mode (single or double)
        '''
        depth_frame = sl.Mat()
        runtime_parameters = sl.RuntimeParameters()
        if self.pipeline.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            self.pipeline.retrieve_measure(depth_frame, sl.MEASURE.DEPTH)
        depth_frame = np.array(depth_frame.get_data())
        depth_frame = np.nan_to_num(depth_frame)
        return depth_frame

    def get_frames(self):
        '''
        :return: rgb, depth image as numpy array or [left rgb, right rgb], depth images depending on camera mode
        '''
        color_frame_left = sl.Mat()
        depth_frame = sl.Mat()
        runtime_parameters = sl.RuntimeParameters()
        if self.pipeline.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            self.pipeline.retrieve_image(color_frame_left, sl.VIEW.LEFT)
            self.pipeline.retrieve_measure(depth_frame, sl.MEASURE.DEPTH)
        depth_frame = np.array(depth_frame.get_data())
        depth_frame = np.nan_to_num(depth_frame)
        color_frame_left = np.array(color_frame_left.get_data())[:, :, :3]
        if self.single_camera_mode:
            return color_frame_left, depth_frame
        else:
            color_frame_right = sl.Mat()
            if self.pipeline.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                self.pipeline.retrieve_image(color_frame_right, sl.VIEW.RIGHT)
            color_frame_right = np.array(color_frame_right.get_data())[:, :, :3]
            return [color_frame_left, color_frame_right], depth_frame

    def get_aligned_frames(self):
        return self.get_frames()

    def set_option(self, option, value):
        '''
        :param option: the option to be set (sl.)
        :param value: the value of the option
        '''
        option_name = str(option).replace('VIDEO_SETTINGS.', '').upper()
        try:
            self.pipeline.set_camera_settings(option, value)
            print("Option %s changed to value: %d" % (option_name, int(value)))
        except TypeError as ex:
            print("\033[0;33;40m Exception (%s): the option %s has NOT been set." % (type(ex).__name__, option_name))

    def get_option(self, option):
        '''
        :param option: the option to be got (sl.)
        '''
        option_name = str(option).replace('VIDEO_SETTINGS.', '').upper()
        try:
            value = self.pipeline.get_camera_settings(option)
            print("Option %s value: %d" % (option_name, int(value)))
        except TypeError as ex:
            print("\033[0;33;40m Exception (%s): the option %s has NOT been set." % (type(ex).__name__, option_name))
