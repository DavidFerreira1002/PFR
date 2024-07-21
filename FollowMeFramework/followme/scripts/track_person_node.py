#!/usr/bin/env python

'''----------------------------------------------------------------------------------------------------------------------------------
# Copyright (C) 2022, Federico Rollo
#
# Institute: Leonardo Labs (Leonardo S.p.a)
#
# This file is part of FollowMe. <https://github.com/FedericoRollo/followme>
#
# FollowMe is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# FollowMe is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License. If not, see http://www.gnu.org/licenses/
---------------------------------------------------------------------------------------------------------------------------------'''

# personal imports
from camera_utils.from2Dto3D import compute_centroids
#from ai_utils.detectors.YolactInference import YolactInference
import cv2
from followme_py_pkg.Reidentificator import Reidentificator
from followme_py_pkg.HandPoseInference import HandPoseInference
from followme_py_pkg.mp_segment import MediapipeInstanceSegmentation, is_valid_contour
from followme_py_pkg.PersonManager import PersonBuffer, publish_location

# system imports
import pickle
import geometry_msgs.msg
import tf
import sys
import select
import time
import numpy as np
import os

# ros imports
import rospy
import tf.transformations
import tf2_ros
import geometry_msgs
import tf2_geometry_msgs
from std_msgs.msg import Int32
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import people_msgs.msg
from std_msgs.msg import String
#from pfr_ws.src.FollowMeFramework.followme.scripts.mp_segment import MediapipeInstanceSegmentation

# import matplotlib.pyplot as plt
# from sklearn.utils import shuffle

#------------------------------------------------------------------

class StateHandler:
    def __init__(self, topic_name):
        self.current_state = False
        
        # Initialize the subscriber
        self.bool_subscriber_ = rospy.Subscriber(topic_name, String, self.stateCallback)
        
    def stateCallback(self, msg):
        # rospy.loginfo("Callback triggered with message: %s", "true" if msg.data else "false")
        self.current_state = msg.data
        
    def getCurrentState(self):
        return self.current_state

class FrameSynchronizer():

    rgb_image : np.array
    depth_image : np.array

    bridge = CvBridge()
    camera_info = {}

    def __init__(self, rgb_topic, depth_topic, camera_info_topic) -> None:

        self.rgb_topic = rgb_topic
        self.depth_topic = depth_topic
        self.camera_info_topic = camera_info_topic

        rospy.wait_for_message(self.camera_info_topic, CameraInfo, 2)
        self.camera_info_sub = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_callback)

        rospy.wait_for_message(self.rgb_topic, Image, 3)
        rospy.wait_for_message(self.depth_topic, Image, 3)
        rgb_sub = message_filters.Subscriber(self.rgb_topic, Image)
        depth_sub = message_filters.Subscriber(self.depth_topic, Image)

        self.ts = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], 10, 0.5)
        self.ts.registerCallback(self.synchronized_camera_callback)
        
    def synchronized_camera_callback(self, rgb_image: Image, depth_image: Image):
        self.rgb_image = self.bridge.imgmsg_to_cv2(rgb_image)
        self.depth_image = self.bridge.imgmsg_to_cv2(depth_image)

    def camera_info_callback(self, camera_info: CameraInfo):
        self.camera_info['fx'] = camera_info.K[0]
        self.camera_info['fy'] = camera_info.K[4]
        self.camera_info['px'] = camera_info.K[2]
        self.camera_info['py'] = camera_info.K[5]

        self.camera_info_sub.unregister()

    def get_rgb(self):
        rgb = self.rgb_image.copy()
        return rgb

    def get_depth(self):
        depth_image = self.depth_image.copy()
        return depth_image

    def get_camera_info(self):
        return self.camera_info

if __name__ == '__main__':

    #------GETTING PARAMS------
    sys.argv = rospy.myargv(argv=sys.argv)

    rospy.init_node('track_person')
    br = tf.TransformBroadcaster()

    # load display ros params
    display_img_results = rospy.get_param('track_person/display_img', False)
    display_fps = rospy.get_param('track_person/display_fps', False)

    # find the weight path
    pkg_dir_name = '/' + os.path.join(*__file__.split('/')[:-2])
    weights_dir = pkg_dir_name + "/weights"
    weights_dir_mp = pkg_dir_name + "/weights_mp" 

    # Load detector
    #yolact_weights = os.path.join(weights_dir, "yolact_plus_resnet50_54_800000.pth")
    #yolact = YolactInference(model_weights=yolact_weights, display_img=display_img_results)
    #mediapipe_weights = os.path.join(weights_dir_mp,"efficientdet_lite0.tflite")
    mediapipe_weights = os.path.join(weights_dir_mp,"ssd_mobilenet_v2.tflite")
    mediapipe_model = MediapipeInstanceSegmentation(mediapipe_weights,display=display_img_results)

    # Load reidentificator ros params
    calibration_bool = rospy.get_param('reidentificator/calibration_bool', True)
    calibration_filename = rospy.get_param('reidentificator/calibration_filename', "calibration_default.pkl")
    calibration_dir = pkg_dir_name + "/calibrations"
    calibration_path = os.path.join(calibration_dir, calibration_filename)
    mmt_weights = os.path.join(weights_dir, "old_pytorch_resnet_ibn_REID_feat256_train_msmt17.pth")

    # Load reidentificator 
    # If calibration_bool True, creates a new reident
    if(calibration_bool):
        reident = Reidentificator(class_target="person", display_img=display_img_results, model_weights=mmt_weights, calibration_path = calibration_path)
    # Instance a previous object from a pkl file
    else:
        reident = Reidentificator.deserialize_from_pickle(calibration_path)
    
    # create target publisher
    target_publisher = rospy.Publisher("target_position", geometry_msgs.msg.PoseStamped, queue_size=5)

    # load gesture detection model
    hand_pose = HandPoseInference(display_img=display_img_results)
    hand_weights = os.path.join(weights_dir, "right_hand_model.sav")
    hand_classifier = pickle.load(open(hand_weights, 'rb'))

    # create gesture publisher
    gesture_publisher = rospy.Publisher("hand_gesture", Int32, queue_size=10)
    gesture_msg = Int32()

    # load ros params for topics
    rgb_topic = rospy.get_param('track_person/rgb_topic', None)
    depth_topic = rospy.get_param('track_person/depth_topic', None)
    camera_info_topic = rospy.get_param('track_person/camera_info_topic', None)
    camera_depth_reference_frame = rospy.get_param("track_person/camera_depth_optical_frame", None) 

    # load person_buffer params
    buffer_size = rospy.get_param('person_buffer/buffer_size', 5) 
    base_distance = rospy.get_param('person_buffer/base_distance', 0.4)
    velocity_influence = rospy.get_param('person_buffer/velocity_influence', 0.25)
    side_bias = rospy.get_param('person_buffer/side_bias', 1.8) 
    activity_treshold = rospy.get_param('person_buffer/activity_treshold', 15)

    # check if ros params have been loaded
    if rgb_topic is None or depth_topic is None or camera_info_topic is None or camera_depth_reference_frame is None:
        rospy.logerr("Topic names loaded by ros param server are None..\n"
        "Check if you have set correctly your params in the yaml file! See documentation for more.")
        exit(1)
    #----------END OF GETTING PARAMS---------------
        
    #----------INITIALIZING FrameSync--------------
    # initialize synchronized subscriber object
    camera_synchronizer = FrameSynchronizer(rgb_topic, depth_topic, camera_info_topic)

    #stop_char = 'q'  # char used to stop the infinite loops

    # wait for subscriptions to load
    time.sleep(2)

    #----------END OF FrameSync-------------------

    #----------INITIALIZE THE BUFFER AND A PUBLISHER----------
    person_buffer = PersonBuffer(buffer_size, base_distance, velocity_influence, side_bias, activity_treshold) 
    person_pub = rospy.Publisher('/people',people_msgs.msg.People,queue_size=5)

    #----------ReID AND Tracker Variables------------
    control_counter = 0
    kcf_success = False

    #----------State Subscriber-------------------
    state_topic_name = rospy.get_param('state/topic_name')
    current_state = StateHandler(state_topic_name)
    
    #----------Person Screen Side Publisher-------
    screen_side_pub = rospy.Publisher('/target_side',String, queue_size=5)
    screen_side = ""
    previous_screen_side = ""
    screen_side_msg = String()

    #----------START OF LOOPS---------------------
    #Needs to be 1 loop
    #Check if calibration is done, if TRUE, jump to the REID,
    #If FALSE do reident.calibrate_person(rgb, yolact_infer) and then dont do the REID
    #At the end there should be like the following block:
    #------------
    #r = rospy.Rate(30)#30hz
    #while not rospy.is_shutdown():
    #    print("TEST")
    #    r.sleep()
    #-------------

    tf_buffer = tf2_ros.Buffer(rospy.Duration(2.0))
    tf2_ros.TransformListener(tf_buffer)

    seq_n = 0  # incremented for successive timestamps in tf
    rate = rospy.Rate(30) #30hz
    while not rospy.is_shutdown():
        if(current_state.getCurrentState() == "GENERAL"):
            continue
        else:

            #Check if calibration is done:
            if not reident.calibrated:
                rgb = camera_synchronizer.get_rgb()
                #yolact_infer = yolact.img_inference(rgb)
                mp_infer, _ = mediapipe_model.run(rgb)
                #print(yolact_infer)
                #print(mp_infer)
                #reident.calibrate_person(rgb, yolact_infer)
                reident.calibrate_person(rgb, mp_infer)
            elif reident.calibrated:
                if display_fps:
                    start_time = time.time()

                color_frame = camera_synchronizer.get_rgb()
                depth_frame = camera_synchronizer.get_depth()
                ros_time = rospy.Time.now()

                # Person detection
                #yolact_infer = yolact.img_inference(color_frame)

                mp_infer, persons_mask = mediapipe_model.run(color_frame)
                previous_persons_mask = persons_mask
                #print(mp_infer)

                #Publish the locations of the persons found to /people with a msg type people_msgs/People
                #publish_location(color_frame, depth_frame, yolact_infer, camera_synchronizer.get_camera_info(), person_buffer, person_pub, tf_buffer)
                publish_location(color_frame, depth_frame, mp_infer, camera_synchronizer.get_camera_info(), person_buffer, person_pub, tf_buffer)
                
                #IF statement to choose between ReID and Tracker
                # Use ReID
                
                filtered_color_frame = cv2.bitwise_and(color_frame,color_frame,mask=persons_mask)
                if control_counter == 0 and not kcf_success:
                    
                    re_id_frame = color_frame.copy()

                    # Person reidentification
                    #reidentified_person = reident.reidentify(color_frame, yolact_infer)
                    reidentified_person = reident.reidentify(re_id_frame, mp_infer)

                    # if no person re-identified restart detection step
                    if reidentified_person is None:
                        #print("Target not found.")
                        control_counter = 0
                        kcf_success = False
                        continue
                    
                    control_counter += 1
                    reidentified_mask = reidentified_person["masks"]
                    reidentified_box = reidentified_person["boxes"]

                else:

                    #Check if current person_mask is valid, else use the previous one and dont save this one
                    if is_valid_contour(color_frame, persons_mask[:]):
                        previous_persons_mask = persons_mask
                    else:
                        persons_mask = previous_persons_mask
                    
                    # if kcf_success is False, restart the kcf tracker
                    if(not kcf_success):
                        kcf_tracker = cv2.TrackerKCF_create()
                        #Note, the bounding box that the kcf tracker is expecting is one of the type (height,width,x,y), 
                        # what comes out of the ReID is (xmin,ymin,xmax,ymax)
                        xmin = reidentified_box[0]
                        ymin = reidentified_box[1]
                        xmax = reidentified_box[2]
                        ymax = reidentified_box[3]
                        
                        # Make the starting bouding box much smaller and on the chest of the target
                        # midpoint_x = int(xmin + ((xmax - xmin)/2))
                        # midpoint_y = int(ymin + ((ymax - ymin)/2))
                        # new_height = int((ymax - ymin))
                        # new_width = int((xmax - xmin)/1.4)
                        # new_x = int(midpoint_x - (new_width/2))
                        # new_y = int(midpoint_y - (new_height/2))
                        # tracker_bbox = [ new_x, new_y, new_width, new_height]
                        tracker_bbox = [ xmin, ymin, xmax-xmin, ymax-ymin]
                        #Pass the bounding box and the frame where that bouding box was gotten from
                        filtered_re_id_frame = cv2.bitwise_and(re_id_frame,re_id_frame,mask=persons_mask)
                        kcf_tracker.init(filtered_re_id_frame, tracker_bbox)
                        previous_persons_mask = persons_mask

                    kcf_success, tracker_bbox = kcf_tracker.update(filtered_color_frame)
                    control_counter += 1
                    #Send it back to reidentified_box
                    reidentified_box[0] = tracker_bbox[0]
                    reidentified_box[1] = tracker_bbox[1]
                    reidentified_box[2] = tracker_bbox[2] + tracker_bbox[0]
                    reidentified_box[3] = tracker_bbox[3] + tracker_bbox[1]

                    # If target lost
                    if not kcf_success:
                        #print("Target not found.")
                        control_counter = 0
                        continue
                    
                    # Maximum amount of frames for the tracker to process without correction
                    if control_counter >= 50:
                        control_counter = 0
                        kcf_success = False

                    

                    #Get the reidentified_mask, using the reidentified_box and the persons_mask
                    temp_mask = np.zeros(persons_mask.shape[:], dtype=np.uint8)
                    x1 = reidentified_box[0]
                    y1 = reidentified_box[1] 
                    x2 = reidentified_box[2]
                    y2 = reidentified_box[3]
                    temp_mask[y1:y2,x1:x2] = 255
                    reidentified_mask = cv2.bitwise_and(persons_mask, persons_mask, mask=temp_mask)
                

                # Publish the side the person last was
                midpoint_x = reidentified_box[0] + (reidentified_box[2] - reidentified_box[0])/2
                image_midpoint = color_frame.shape[1]/2
                
                if midpoint_x > image_midpoint:
                    #left
                    screen_side = "RIGHT"
                else:
                    #right
                    screen_side = "LEFT"

                if not screen_side == "" and screen_side != previous_screen_side:
                    screen_side_msg.data = screen_side
                    previous_screen_side = screen_side
                    screen_side_pub.publish(screen_side_msg)
                

                # Debug only, in production keep it False
                show = True
                if show:
                    # Draw the mask on the image
                    image_t = filtered_color_frame.copy()
                    # Ensure the mask has the same dimensions as the image
                    if reidentified_mask.shape[:2] != image_t.shape[:2]:
                        reidentified_mask = cv2.resize(reidentified_mask, (image_t.shape[1], image_t.shape[0]))

                    mask = reidentified_mask
                    image_t[mask == 255] = [0, 255, 0]  # Color the mask region (green)

                    # Draw the bounding box on the image
            
                    x1 = reidentified_box[0]
                    y1 = reidentified_box[1] 
                    x2 = reidentified_box[2]
                    y2 = reidentified_box[3]

                    cv2.rectangle(image_t, (x1, y1), (x2, y2), (255, 0, 0), 2)  # Blue box

                    # Show the image with masks and boxes
                    if kcf_success:
                        cv2.imshow("Tracked target",cv2.cvtColor(image_t, cv2.COLOR_BGR2RGB))
                        cv2.waitKey(1)
                    else:
                        cv2.imshow("ReID target",cv2.cvtColor(image_t, cv2.COLOR_BGR2RGB))
                        cv2.waitKey(1)

                    # Clear the image for the next person
                    image_t = np.zeros((512, 512, 3), dtype=np.uint8)


                hand_img = color_frame.copy()
                hand_img = hand_img[reidentified_box[1]:reidentified_box[3], reidentified_box[0]:reidentified_box[2], :]

                # initialize the prediction class as the last class is the predictor
                gesture_prediction = hand_classifier.n_support_.shape[0] - 1

                if hand_img is not None:
                    hand_results = hand_pose.get_hand_pose(hand_img)
                    if hand_results is not None:
                        for hand_label in hand_results.keys():
                            if hand_label == "Left":
                                continue
                            else:
                                gesture_prediction = hand_classifier.predict([hand_results[hand_label]])

                    gesture_msg.data = int(gesture_prediction)
                    gesture_publisher.publish(gesture_msg)

                # compute centroid
                points_and_angles = compute_centroids(color_frame, depth_frame, reidentified_mask, camera_synchronizer.get_camera_info(),use_pcd=False)
                
                # total_time = time.time()-start_time
                # curr_time.append(total_time)     
                # i+=1
                # print(i)
                # if i >= 50:
                #     i = 0
                #     final_time.append(sum(curr_time)/len(curr_time))
                #     input("press to continue")     
                # if len(final_time) == 5:
                #     print(final_time)
                #     exit(0)
                # print(total_time)
                
                if points_and_angles[0][0][0] != 0 and points_and_angles[0][0][1] != 0 and \
                        points_and_angles[0][0][2] != 0:

                    x = points_and_angles[0][0][0]
                    y = points_and_angles[0][0][1]
                    z = points_and_angles[0][0][2]
                    theta = 0  # points_and_angles[target_idx][1]

                    # create the tf msg
                    m = geometry_msgs.msg.TransformStamped()

                    # set header info
                    m.header.frame_id = camera_depth_reference_frame
                    m.header.stamp = ros_time
                    m.header.seq = seq_n

                    # set child name (target name)
                    m.child_frame_id = f"target_0"

                    # set rototranslation of tf
                    m.transform.translation = geometry_msgs.msg.Vector3(x, y, z)
                    angle = tf.transformations.quaternion_from_euler(0, 0, theta)
                    # pdb.set_trace()
                    m.transform.rotation.x = angle[0]
                    m.transform.rotation.y = angle[1]
                    m.transform.rotation.z = angle[2]
                    m.transform.rotation.w = angle[3]

                    # send tf as a tf message
                    br.sendTransformMessage(m)
                    seq_n += 1

                    # create the msg for the topic \target_position and publish it 

                    target_position = geometry_msgs.msg.PoseStamped()
                    target_position.header.stamp = m.header.stamp
                    target_position.header.frame_id = camera_depth_reference_frame
                    target_position.pose.position.x = x
                    target_position.pose.position.y = y
                    target_position.pose.position.z = z

                    target_position.pose.orientation.x = 0.0
                    target_position.pose.orientation.y = 0.0
                    target_position.pose.orientation.z = 0.0
                    target_position.pose.orientation.w = 1.0
                    
                    target_publisher.publish(target_position)


                if display_fps:
                    print(1/(time.time()-start_time))
            rate.sleep()

    cv2.destroyAllWindows()
#---------END OF LOOPS--------------


#------------- OLD CODE PARTS FROM FEDERICO ROLLO ------------------------
    # Person calibration loop
    # start_time = time.time()
    #while True and not reident.calibrated:
    #    rgb = camera_synchronizer.get_rgb()
    #    yolact_infer = yolact.img_inference(rgb)
    #    if reident.calibrate_person(rgb, yolact_infer) or stop_loop(stop_char):
    #        break
    # input("Press enter to continue")

    # impostor_distances = []
    # i = 0
    # while i < 201:
    #     rgb = camera_synchronizer.get_rgb()
    #     yolact_infer = yolact.img_inference(rgb)
    #     dist, found = reident.compute_dist(rgb, yolact_infer)
    #     if found:
    #         i += 1
    #         impostor_distances.append(dist)

    # impostor_distances = shuffle(impostor_distances)

    # plt.plot(np.arange(len(reident.dist_threshold)), np.array(reident.dist_threshold), 'g', linewidth=3)
    # plt.plot(np.arange(len(impostor_distances)), np.array(impostor_distances), 'r', linewidth=3)
    # plt.plot(np.ones(len(reident.dist_threshold)) * np.mean(np.array(reident.dist_threshold)), 'c', linewidth=3)
    # plt.plot(np.ones(len(reident.dist_threshold)) * (np.mean(np.array(reident.dist_threshold)) + reident.std_dev_confidence * np.std(np.array(reident.dist_threshold))), 'm', linewidth=3)
    # plt.legend(["Calibrated Person Distances", "Distractor Person Distances", r'Calibrated Person Distances Mean $\mu$' , r"Threshold $\mu$+2$\sigma$"], prop={'size': 18})
    # plt.grid(True)
    # plt.ylim((0,2.5))
    # plt.ylabel('Feature Distance', size=18)
    # plt.xlabel('# Samples', size=18)
    # plt.xticks(fontsize=18)
    # plt.yticks(fontsize=18)
    # plt.show()    


    #seq_n = 0  # incremented for successive timestamps in tf
    # i = 0
    # curr_time = []
    # final_time = []
    # tracking loop with re-identification
    #while not stop_loop(stop_char):
        # if display_fps:
        #     start_time = time.time()

        # # start_time = time.time()

        # color_frame = camera_synchronizer.get_rgb()
        # depth_frame = camera_synchronizer.get_depth()
        # ros_time = rospy.Time.now()

        # # Person detection
        # yolact_infer = yolact.img_inference(color_frame)

        # # Person reidentification
        # reidentified_person = reident.reidentify(color_frame, yolact_infer)

        # # if no person re-identified restart detection step
        # if reidentified_person is None:
        #     print("Target not found.")
        #     rospy.spin()
        #     continue

        # reidentified_mask = reidentified_person["masks"]
        # reidentified_box = reidentified_person["boxes"]

        # hand_img = color_frame.copy()
        # hand_img = hand_img[reidentified_box[1]:reidentified_box[3], reidentified_box[0]:reidentified_box[2], :]

        # # initialize the prediction class as the last class is the predictor
        # gesture_prediction = hand_classifier.n_support_.shape[0] - 1

        # hand_results = hand_pose.get_hand_pose(hand_img)
        # if hand_results is not None:
        #     for hand_label in hand_results.keys():
        #         if hand_label == "Left":
        #             continue
        #         else:
        #             gesture_prediction = hand_classifier.predict([hand_results[hand_label]])

        # gesture_msg.data = int(gesture_prediction)
        # gesture_publisher.publish(gesture_msg)

        # # compute centroid
        # points_and_angles = compute_centroids(color_frame, depth_frame, reidentified_mask, camera_synchronizer.get_camera_info())
        
        # # total_time = time.time()-start_time
        # # curr_time.append(total_time)
        
        # # i+=1
        # # print(i)

        # # if i >= 50:
        # #     i = 0
        # #     final_time.append(sum(curr_time)/len(curr_time))
        # #     input("press to continue")
            
        # # if len(final_time) == 5:
        # #     print(final_time)
        # #     exit(0)

        # # print(total_time)
        
        # if points_and_angles[0][0][0] != 0 and points_and_angles[0][0][1] != 0 and \
        #         points_and_angles[0][0][2] != 0:

        #     x = points_and_angles[0][0][0]
        #     y = points_and_angles[0][0][1]
        #     z = points_and_angles[0][0][2]
        #     theta = 0  # points_and_angles[target_idx][1]

        #     # create the tf msg
        #     m = geometry_msgs.msg.TransformStamped()

        #     # set header info
        #     m.header.frame_id = camera_depth_reference_frame
        #     m.header.stamp = ros_time
        #     m.header.seq = seq_n

        #     # set child name (target name)
        #     m.child_frame_id = f"target_0"

        #     # set rototranslation of tf
        #     m.transform.translation = geometry_msgs.msg.Vector3(x, y, z)
        #     angle = tf.transformations.quaternion_from_euler(0, 0, theta)
        #     # pdb.set_trace()
        #     m.transform.rotation.x = angle[0]
        #     m.transform.rotation.y = angle[1]
        #     m.transform.rotation.z = angle[2]
        #     m.transform.rotation.w = angle[3]

        #     # send tf as a tf message
        #     br.sendTransformMessage(m)
        #     seq_n += 1

        # if display_fps:
        #     print(1/(time.time()-start_time))
        # rospy.spin()