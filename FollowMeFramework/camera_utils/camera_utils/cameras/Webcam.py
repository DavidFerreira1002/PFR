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

from camera_utils.cameras.CameraInterface import Camera
import cv2
import numpy as np

class Webcam(Camera):

    def __init__(self, device_idx = 0):

        self.camera_brand = "Computer Webcam"
        self.camera_name = "Webcam"

        self.camera = cv2.VideoCapture(device_idx)

        if not self.camera.isOpened():
            print("Cannot open Webcam camera")
            exit()

        Camera.__init__(self)

        width  = self.camera.get(cv2.CAP_PROP_FRAME_WIDTH) 
        height = self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT)

        self.intr = {'width': width, 'height': height}

        print("%s initialized" % self.camera_brand)




    def __del__(self):
        self.camera.release()


    def get_rgb(self):
        '''
        :return: An rgb image as numpy array
        '''
        ret, frame = self.camera.read()

        if ret:
            return np.asanyarray(frame)
        return None

    