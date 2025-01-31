'''----------------------------------------------------------------------------------------------------------------------------------
# Copyright (C) 2022
#
# author: Federico Rollo, Andrea Zunino
# mail: rollo.f96@gmail.com
#
# Institute: Leonardo Labs (Leonardo S.p.a - Istituto Italiano di tecnologia)
#
# This file is part of ai_utils. <https://github.com/IASRobolab/ai_utils>
#
# ai_utils is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# ai_utils is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License. If not, see http://www.gnu.org/licenses/
---------------------------------------------------------------------------------------------------------------------------------'''
import cv2
import numpy as np
import torch
from torch.nn import functional as F
import sys
from mmt import models
from mmt.utils.serialization import load_checkpoint, copy_state_dict
import matplotlib.pyplot as plt
from sklearn.utils import shuffle

#import json
import pickle
import io

class FastBaseTransform(torch.nn.Module):
    """
    Transform that does all operations on the GPU* (not anymore) for super speed.
    This doesn't support a lot of config settings and should only be used for production.
    Maintain this as necessary.
    """
    MEANS = (103.94, 116.78, 123.68)
    STD = (57.38, 57.12, 58.40)

    def __init__(self):
        super().__init__()
        self.mean = torch.Tensor(self.MEANS).float()[None, :, None, None]
        self.std = torch.Tensor(self.STD).float()[None, :, None, None]

    def forward(self, img):
        self.mean = self.mean.to(img.device)
        self.std = self.std.to(img.device)
        # img assumed to be a pytorch BGR image with channel order [n, h, w, c]
        img_size = (256, 128)
        img = img.permute(0, 3, 1, 2).contiguous()
        img = F.interpolate(img, img_size, mode='bilinear', align_corners=False)
        img = (img - self.mean) / self.std
        img = img[:, (2, 1, 0), :, :].contiguous()
        # Return value is in channel order [n, c, h, w] and RGB
        return img


class Reidentificator:
    '''
    Reidentify an object on an image using the inference output of another AI algorithm and calibration
    '''

    def __init__(self, class_target, model_weights, calibration_path, display_img=False):
        '''
        initialize the Re-identificator object
        :param class_target: the class of the object you want to track
        :param display_img: boolean value to return an image which create a bounding box around the
        re-identified object
        '''

        self.class_target = class_target
        self.display_img = display_img

        #it parallels the FastBaseTransform class that is on the gpu
        self.transform = torch.nn.DataParallel(FastBaseTransform())
        print('Loading REID model...', end='')
        self.model_REID = models.create('resnet_ibn50a', pretrained=False,
                                        num_features=256, dropout=0,
                                        num_classes=0, norm=True)
        self.model_REID
        self.model_REID = torch.nn.DataParallel(self.model_REID)
        try:
            checkpoint = load_checkpoint(model_weights)
        except ValueError:
            print('\n\033[91mWeights not found in ' + model_weights + ". You must download them "
                                                                      "in that directory.\033[0m")
            exit(1)
        copy_state_dict(checkpoint['state_dict'], self.model_REID)
        self.model_REID.eval()
        print('Done.')

        self.iteration_number = 0
        self.required_calibration_measures = 500
        self.std_dev_confidence = 2
        self.calibrated = False

        self.feats = []
        self.feats_distances = []

        self.feature_threshold = float
        self.mahalanobis_deviation_const = float
        self.std_pers = float
        self.mean_pers = float
        self.calibration_path = calibration_path

    def calibrate_person(self, rgb, inference_output):
        '''
        Function used to calibrate the reidentificator with the object image. This function should be called iteratively
        until it returns True (i.e., when the object is calibrated)
        :param rgb: the image in which there is the object
        :param inference_output: a dictionary containing the inferences obtained by an instance segmentation algorithm
        (e.g., Yolact++)
        :return: A boolean which confirm if the object has been correctly calibrated or not
        '''
        try:
            boxes = inference_output[self.class_target]['boxes']
            masks = inference_output[self.class_target]['masks']
        except KeyError:
            return self.calibrated

        if len(boxes) > 1:
            print('WARNING: MORE THAN ONE PERSON DETECTED DURING CALIBRATION!')
            self.iteration_number = 0
            self.feats = []
        else:
            for i in range(3):
                rgb[:, :, i] = rgb[:, :, i] * masks[0]
            percentage = self.iteration_number / self.required_calibration_measures * 100
            if percentage % 10 == 0:
                print("CALIBRATING ", int(percentage), "%")
            img_person = self.transform(torch.from_numpy(rgb[boxes[0][1]:boxes[0][3], boxes[0][0]:boxes[0][2], :]).unsqueeze(0).float())
            self.iteration_number += 1
            self.feats.append(self.model_REID(img_person).data.cpu()[0].numpy())

            # after meas_init measures we terminate the initialization
            if self.iteration_number > self.required_calibration_measures:
                print('\nCALIBRATION FINISHED')

                self.calibrated = True
                # shuffle features
                self.feats = shuffle(self.feats)
                calibration_threshold_slice = int(len(self.feats)*2/3)
                # compute mean, std and mahalanobis
                feat_calibrate = self.feats[:calibration_threshold_slice]
                self.mean_pers = np.mean(np.array(feat_calibrate), axis=0)
                self.std_pers = np.std(np.array(feat_calibrate), axis=0)
                self.mahalanobis_deviation_const = np.sqrt(self.mean_pers.shape[0])
                # compute threshold
                feat_threshold = self.feats[calibration_threshold_slice:]
                person_mean_feat = np.tile(self.mean_pers, (len(feat_threshold), 1))
                person_std_feat = np.tile(self.std_pers, (len(feat_threshold), 1))
                dist_threshold = np.linalg.norm((feat_threshold - person_mean_feat) / (self.mahalanobis_deviation_const * person_std_feat), axis=1)

                # plt.plot(np.arange(len(dist_threshold)), np.array(dist_threshold))
                # plt.plot(np.ones(len(dist_threshold)) * np.mean(np.array(dist_threshold)))
                # plt.plot(np.ones(len(dist_threshold)) * (np.mean(np.array(dist_threshold)) + self.std_dev_confidence * np.std(np.array(dist_threshold))))
                # plt.legend(["Thresholds", "Mean", "2.2 std"])
                # plt.grid(True)
                # plt.show()

                self.feature_threshold = np.mean(np.array(dist_threshold)) + self.std_dev_confidence * np.std(np.array(dist_threshold))
                print("\nTHRESHOLD: %.4f" % self.feature_threshold)
                self.serialize_to_pickle(self.calibration_path)

        return self.calibrated

    def reidentify(self, rgb, inference_output):
        '''
        Used to reidentify the calibrated object on the image (if present)
        :param rgb: the image in which there should be the object to reidentify
        :param inference_output: a dictionary containing the inferences obtained by an instance segmentation algorithm
        (e.g., Yolact++)
        :return: the mask of the targeted object if reidentified
        '''
        rgb = rgb.copy()
        if not self.calibrated:
            sys.exit("Error: Reidentificator not calibrated!")

        try:
            boxes = inference_output[self.class_target]['boxes']
            masks = inference_output[self.class_target]['masks']

            img_persons = []
            # COPY THE FEATURES TEMPLATE ACCORDINGLY TO NUMBER OF DETECTED PERSONS FOR FAST DISTANCE COMPUTATION
            person_mean_feat = np.tile(self.mean_pers, (len(boxes), 1))
            person_std_feat = np.tile(self.std_pers, (len(boxes), 1))
            # CUT THE BOUNDING BOXES OF THE DETECTED PERSONS OVER THE IMAGE
            for id in range(len(boxes)):
                rgb_new = rgb.copy()
                for i in range(3):
                    rgb_new[:, :, i] = rgb_new[:, :, i] * masks[id]
                person_bb = self.transform(
                    torch.from_numpy(rgb_new[boxes[id][1]:boxes[id][3], boxes[id][0]:boxes[id][2], :]).unsqueeze(
                        0).float())
                img_persons.append(person_bb[0])
            img_persons = [img_person.float() for img_person in img_persons]
            img_persons = torch.stack(img_persons, 0)
            # PASS THE IMAGES INSIDE THE EXTERNAL NETWORK
            feat_pers = self.model_REID(img_persons).data.cpu()
            # COMPUTE FEATURES DISTANCES
            dist = np.linalg.norm((feat_pers - person_mean_feat) / (self.mahalanobis_deviation_const * person_std_feat),
                                  axis=1)
            # print(dist)
            # RETURN REIDENTIFIED CLASS IFF THE DISTANCE BETWEEN FEATURE IS NO MORE THAN A CALIBRATED THRESHOLD
            if np.min(dist) > self.feature_threshold:
                reidentified_class = None
            else:
                target_idx = np.argmin(dist)
                reidentified_class = {"class": self.class_target,
                                      "boxes": boxes[target_idx],
                                      "masks": masks[target_idx]}

                # DISPLAY REIDENTIFIED IMAGE
                if self.display_img:
                    x1, y1, x2, y2 = boxes[target_idx]
                    cv2.rectangle(rgb, (x1, y1), (x2, y2), (255, 255, 255), 5)

                    text_str = 'TARGET'

                    font_face = cv2.FONT_HERSHEY_DUPLEX
                    font_scale = 0.6
                    font_thickness = 1

                    text_w, text_h = cv2.getTextSize(text_str, font_face, font_scale, font_thickness)[0]

                    text_pt = (x1, y1 - 3)
                    text_color = [255, 255, 255]

                    cv2.rectangle(rgb, (x1, y1), (x1 + text_w, y1 - text_h - 4), (0, 0, 0), -1)
                    cv2.putText(rgb, text_str, text_pt, font_face, font_scale, text_color, font_thickness,
                                cv2.LINE_AA)

                    cv2.namedWindow("Reidentificator", cv2.WINDOW_NORMAL)
                    cv2.imshow("Reidentificator", rgb)

                    if cv2.waitKey(1) == ord('q'):
                        print("Closed Reidentificator Image Viewer.")
                        exit(0)

        # KeyError exception rise up if no boxes and masks are found in inference input
        except KeyError:
            reidentified_class = None

        return reidentified_class

    # ------------ONLY USED FOR STATISTICS-----------------
    def calibrate_person_statistics(self, rgb):
        '''
        Function used to calibrate the reidentificator with the object image. This function should be called iteratively
        until it returns True (i.e., when the object is calibrated)
        :param rgb: the image in which there is the object
        :return: A boolean which confirm if the object has been correctly calibrated or not
        '''

        percentage = self.iteration_number / self.required_calibration_measures * 100
        if percentage % 10 == 0:
            print("CALIBRATING ", int(percentage), "%")
        img_person = self.transform(torch.from_numpy(rgb).unsqueeze(0).float())
        self.iteration_number += 1
        self.feats.append(self.model_REID(img_person).data.cpu()[0].numpy())

        if self.iteration_number >= self.required_calibration_measures:
            print('\nCALIBRATION FINISHED\n')
            self.calibrated = True

            self.feats = shuffle(self.feats)
            calibration_threshold_slice = int(len(self.feats) * 2 / 3)

            feat_calibrate = self.feats[:calibration_threshold_slice]
            self.mean_pers = np.mean(np.array(feat_calibrate), axis=0)
            self.std_pers = np.std(np.array(feat_calibrate), axis=0)
            self.mahalanobis_deviation_const = np.sqrt(self.mean_pers.shape[0])

            feat_threshold = self.feats[calibration_threshold_slice:]
            person_mean_feat = np.tile(self.mean_pers, (len(feat_threshold), 1))
            person_std_feat = np.tile(self.std_pers, (len(feat_threshold), 1))
            dist_threshold = np.linalg.norm(
                (feat_threshold - person_mean_feat) / (self.mahalanobis_deviation_const * person_std_feat), axis=1)

            # plt.plot(np.arange(len(dist_threshold)), np.array(dist_threshold))
            # plt.plot(np.ones(len(dist_threshold)) * np.mean(np.array(dist_threshold)))
            # plt.plot(np.ones(len(dist_threshold)) * (np.mean(np.array(dist_threshold)) + self.std_dev_confidence * np.std(np.array(dist_threshold))))
            # plt.legend(["Distances", "Mean", "Threshold (Mean + 2 std)"])
            # plt.grid(True)
            # plt.show()

            self.feature_threshold = np.mean(np.array(dist_threshold)) + self.std_dev_confidence * np.std(
                np.array(dist_threshold))
            print("\nTHRESHOLD: %.4f" % self.feature_threshold)

        return self.calibrated

    def reidentify_statistics(self, rgb):
        '''
        Used to reidentify the calibrated object on the image (if present)
        :param rgb: the image in which there should be the object to reidentify
        :return: the mask of the targeted object if reidentified
        '''
        rgb = rgb.copy()
        if not self.calibrated:
            sys.exit("Error: Reidentificator not calibrated!")

        img_persons = self.transform(torch.from_numpy(rgb).unsqueeze(0).float())
        # PASS THE IMAGES INSIDE THE EXTERNAL NETWORK
        feat_pers = self.model_REID(img_persons).data.cpu()
        # COMPUTE FEATURES DISTANCES
        dist = np.linalg.norm((feat_pers - self.mean_pers) / (self.mahalanobis_deviation_const * self.std_pers),
                              axis=1)
        # print(dist)
        # RETURN REIDENTIFIED CLASS IFF THE DISTANCE BETWEEN FEATURE IS NO MORE THAN A THRESHOLD
        if dist > self.feature_threshold:
            reidentified_class = False
        else:
            reidentified_class = True

        return reidentified_class

    def reidentify_statistics_in_images(self, images: dict) -> str:
        '''
        Used to reidentify the calibrated object on the image (if present)
        :param images: the image in which there should be the object to reidentify
        :return: the mask of the targeted object if reidentified
        '''
        min_dist = 2000000000
        reidentified_class = None
        for person, image in images.items():
            if not self.calibrated:
                sys.exit("Error: Reidentificator not calibrated!")

            img_persons = self.transform(torch.from_numpy(image).unsqueeze(0).float())
            # PASS THE IMAGES INSIDE THE EXTERNAL NETWORK
            feat_pers = self.model_REID(img_persons).data.cpu()
            # COMPUTE FEATURES DISTANCES
            dist = np.linalg.norm((feat_pers - self.mean_pers) / (self.mahalanobis_deviation_const * self.std_pers),
                                  axis=1)
            if dist < min_dist:
                min_dist = dist
                reidentified_class = person
            # print(dist)
            # RETURN REIDENTIFIED CLASS IFF THE DISTANCE BETWEEN FEATURE IS NO MORE THAN A THRESHOLD
        if min_dist > self.feature_threshold:
            reidentified_class = None

        return reidentified_class
    
    """ def serialize_to_json(self, filename):
        # Convert feats to a nested list
        feats_serializable = [feat.tolist() for feat in self.feats]


        data = {
            "class_target": self.class_target,
            "display_img": self.display_img,
            "iteration_number": self.iteration_number,
            "required_calibration_measures": self.required_calibration_measures,
            "std_dev_confidence": self.std_dev_confidence,
            "calibrated": self.calibrated,
            "feats": feats_serializable,
            "feats_distance": self.feats_distances,
            "feature_treshold": self.feature_threshold,
            "mahalanobis_deviation_const": self.mahalanobis_deviation_const,
            "std_pers": self.std_pers,
            "mean_pers": self.mean_pers,
            "calibration_path": self.calibration_path
        }

        with open(filename, 'w') as file:
            json.dump(data,file)

    @classmethod
    def deserialize_from_json(cls, filename,model_weights):
        with open(filename, 'r') as file:
            data = json.load(file)

        # Convert feats back to NumPy arrays
        feats = [np.array(feat) for feat in data["feats"]]

        transform = torch.nn.DataParallel(FastBaseTransform())
        print('Loading REID model...', end='')
        model_REID = models.create('resnet_ibn50a', pretrained=False,
                                        num_features=256, dropout=0,
                                        num_classes=0, norm=True)
        model_REID
        model_REID = torch.nn.DataParallel(model_REID)
        try:
            checkpoint = load_checkpoint(model_weights)
        except ValueError:
            print('\n\033[91mWeights not found in ' + model_weights + ". You must download them "
                                                                      "in that directory.\033[0m")
            exit(1)
        copy_state_dict(checkpoint['state_dict'], model_REID)
        model_REID.eval()
        print('Done.')
        
        return cls(data["class_target"], data["display_img"], transform, model_REID, data["iteration_number"], data["required_calibration_measures"], data["std_dev_confidence"], data["calibrated"], feats, data["feats_distance"], data["feature_treshold"], data["mahalanobis_deviation_const"], data["std_pers"], data["mean_pers"], data["calibration_path"]) """
    
    def serialize_to_pickle(self, filename):
        with open(filename, 'wb') as file:
            pickle.dump(self, file)

    @classmethod
    def deserialize_from_pickle(cls, filename):
        with open(filename, 'rb') as file:
            obj = pickle.load(file)
        return obj

