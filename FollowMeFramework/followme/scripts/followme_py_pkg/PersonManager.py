# personal imports
from camera_utils.from2Dto3D import compute_centroids

# system imports

import geometry_msgs.msg
import time
import numpy as np

# ros imports
import rospy
import tf2_ros
import geometry_msgs
import tf2_geometry_msgs
import people_msgs.msg

#Holds the positions and id of a person
class PersonStats:
    def __init__(self,id,buffer_size, activity_treshold):
        self.positions = [] #expecting geometry_msgs.msg.PointStamped
        self.id = id
        self.vel = geometry_msgs.msg.Point()
        #self.kalman = KalmanFilter3D()
        self.buffer_size = buffer_size
        self.activity_treshold = activity_treshold
        self.last_updated = time.time()

    def getPositions(self):
        return self.positions
    
    def getLastPosition(self):
        return self.positions[len(self.positions) - 1]

    def getSizePositions(self):
        return len(self.positions)
    
    def addPosition(self,pos):
        self.positions.append(pos)
        if self.getSizePositions() > self.buffer_size:
            self.deletePosition(0)

    def deletePosition(self,n):
        if(n < 0 or n > (len(self.positions)-1)):
            print("n is outside the size of positions.")
        else:
            del self.positions[n]
    
    def getId(self):
        return self.id
    
    def getVel(self):
        return self.vel
    
    def setVel(self,x,y,z):
        self.vel.x = x
        self.vel.y = y
        self.vel.z = z

    def update(self):
        self.last_updated = time.time()

    def is_idle(self):
        return time.time() - self.last_updated > self.activity_treshold

    

#Manages the persons objects
class PersonBuffer:
    def __init__(self,buffer_size, base_distance, velocity_influence, side_bias, activity_treshold):
        self.persons = []
        self.buffer_size = buffer_size
        self.base_distance = base_distance #base distance added on the tolerance to check whose pos it is
        self.velocity_influence = velocity_influence #the influence velocity has on the tolerance
        self.side_bias = side_bias #the bias toward a side, bias of 1 is no bias.
        self.activity_treshold = activity_treshold #seconds that the object will be deleted after no activity

    def addPerson(self, person):
        self.persons.append(person)

    def getPersons(self):
        return self.persons
    
    def getPersonById(self, id):
        for person in self.persons:
            if person.getId() == id:
                return person

    def delPersonById(self, id):
        for person in self.persons:
            if person.getId() == id:
                self.persons.remove(person)
        
    def time_diff_seconds(self, header_stamp1, header_stamp2):
        """
        Calculate the time difference in seconds between two ROS header stamps.

        Args:
        - header_stamp1: The header.stamp of the first ROS message (rospy.Time).
        - header_stamp2: The header.stamp of the second ROS message (rospy.Time).

        Returns:
        - The time difference in seconds (float).
        """
        # Convert header stamps to seconds
        time1 = header_stamp1.to_sec()
        time2 = header_stamp2.to_sec()

        # Calculate the time difference in seconds
        time_difference = abs(time1 - time2)

        return time_difference

    def checkIfSamePerson(self, new_pos : geometry_msgs.msg.PointStamped):
        #flag to check if the position is from someone known
        pos_is_known_person = False
        #Iterate over all the persons
        for person in self.persons:
            #get the last position of the person and check to see if its close based on the last position and the velocity of the person
            lastPos_x = person.getLastPosition().point.x
            lastPos_y = person.getLastPosition().point.y
            lastPos_z = person.getLastPosition().point.z
            #lastPos_time = person.getLastPostion().header.stamp
            
            vel_x = person.getVel().x
            vel_y = person.getVel().y
            vel_z = person.getVel().z

            new_pos_x = new_pos.point.x
            new_pos_y = new_pos.point.y
            new_pos_z = new_pos.point.z
            #new_pos_time = new_pos.header.stamp

            #calculate the tolerance based on the velocity
            # X axis
            if vel_x > 0:
                
                tolerance_x_upper = lastPos_x + self.base_distance * (1 + (self.velocity_influence * vel_x))
                tolerance_x_lower = lastPos_x - (self.base_distance/self.side_bias) * (1 + (self.velocity_influence * vel_x))

            elif vel_x < 0:
                tolerance_x_upper = lastPos_x + (self.base_distance/self.side_bias) * (1 - (self.velocity_influence * vel_x))
                tolerance_x_lower = lastPos_x - self.base_distance * (1 - (self.velocity_influence * vel_x))
            
            elif vel_x == 0:
                tolerance_x_upper = lastPos_x + self.base_distance
                tolerance_x_lower = lastPos_x - self.base_distance

            #Y axis
            if vel_y > 0:
                
                tolerance_y_upper = lastPos_y + self.base_distance * (1 + (self.velocity_influence * vel_y))
                tolerance_y_lower = lastPos_y - (self.base_distance/self.side_bias) * (1 + (self.velocity_influence * vel_y))

            elif vel_y < 0:
                tolerance_y_upper = lastPos_y + (self.base_distance/self.side_bias) * (1 - (self.velocity_influence * vel_y))
                tolerance_y_lower = lastPos_y - self.base_distance * (1 - (self.velocity_influence * vel_y))
            
            elif vel_y == 0:
                tolerance_y_upper = lastPos_y + self.base_distance
                tolerance_y_lower = lastPos_y - self.base_distance
            #Z axis
            if vel_z > 0:
                
                tolerance_z_upper = lastPos_z + self.base_distance * (1 + (self.velocity_influence * vel_z))
                tolerance_z_lower = lastPos_z - (self.base_distance/self.side_bias) * (1 + (self.velocity_influence * vel_z))

            elif vel_z < 0:
                tolerance_z_upper = lastPos_z + (self.base_distance/self.side_bias) * (1 - (self.velocity_influence * vel_z))
                tolerance_z_lower = lastPos_z - self.base_distance * (1 - (self.velocity_influence * vel_z))

            elif vel_z == 0:
                tolerance_z_upper = lastPos_z + self.base_distance
                tolerance_z_lower = lastPos_z - self.base_distance

            #Check if the pos are inside the tolerance
            if (tolerance_x_lower < new_pos_x) and (tolerance_x_upper > new_pos_x) and (tolerance_y_lower < new_pos_y) and (tolerance_y_upper > new_pos_y) and (tolerance_z_lower < new_pos_z) and (tolerance_z_upper > new_pos_z):
                #if so, add the pos to the person
                person.addPosition(new_pos)
                person.update()
                #turn the flag true
                pos_is_known_person = True
                #find out the persons id
                id = person.getId()
                #and cut the rest of the for
                break
        
        if pos_is_known_person == False:
            #print("New Person Found")
            #print(new_pos.point.x,new_pos.point.y)
            #check the last id used and +1, create the object to be introduced in the buffer
            if(len(self.persons) == 0):
                id = 1
            else:
                id = self.persons[len(self.persons) - 1].getId() + 1
            new_person = PersonStats(id,self.buffer_size,self.activity_treshold)
            new_person.addPosition(new_pos)
            self.addPerson(new_person)
        
        return id

    def calculateVelocity(self,id):
        person = self.getPersonById(id)
        
        positions = person.getPositions()
        positions_size = person.getSizePositions()
        
        sum_x = 0
        sum_y = 0
        sum_z = 0
        if positions_size < 2:
            vel_x = 0
            vel_y = 0
            vel_z = 0
        else:
            for i in range(positions_size - 2):
                if self.time_diff_seconds(positions[i + 1].header.stamp, positions[i].header.stamp) != 0:
                    sum_x = sum_x + ((positions[i + 1].point.x - positions[i].point.x)/(self.time_diff_seconds(positions[i + 1].header.stamp, positions[i].header.stamp)))

                    sum_y = sum_y + ((positions[i + 1].point.y - positions[i].point.y)/(self.time_diff_seconds(positions[i + 1].header.stamp, positions[i].header.stamp)))
                    
                    sum_z = sum_z + ((positions[i + 1].point.z - positions[i].point.z)/(self.time_diff_seconds(positions[i + 1].header.stamp, positions[i].header.stamp)))

            vel_x = sum_x/positions_size
            vel_y = sum_y/positions_size
            vel_z = sum_z/positions_size

        vel = geometry_msgs.msg.Point()
        vel.x = vel_x
        vel.y = vel_y
        vel.z = vel_z
        person.setVel(vel_x,vel_y,vel_z)

        #print(vel_x,vel_y)
        return vel

    def cleanup(self):
            for person in self.persons:
                if person.is_idle():
                    del person


# function used to publish the position of every person identified by the yolact inference
def publish_location(rgb, depth, inference, intrinsics, person_buffer : PersonBuffer, person_pub : rospy.Publisher, tf_buffer : tf2_ros.Buffer, use_pcd=True):
    '''
    Used to publish the position of a list of persons
    @param rgb: rgb image
    @param depth: depth image
    @param inference: a dictionary containing the inferences obtained by an instance segmentation algorithm (e.g. Yolact++)
    @param intrinsics: the camera intrinsic parameters
    @param person_buffer: object of the class PersonBuffer to hold the positions of the identified persons
    @param person_pub: ros publisher to publish the message 
    @param tf_buffer: tf buffer to check the tranforms
    @param use_pcd: [bool] if true use pcd to compute centroids otherwise use only depth
    '''
    #clean the buffer from unactive people
    person_buffer.cleanup()

    try:
        masks = np.array(inference["person"]["masks"])
        reliability = np.array(inference["person"]["scores"])
    except KeyError:
        #print("No persons found")
        return False
    #tf_buffer = tf2_ros.Buffer(rospy.Duration(2.0))
    #tf2_ros.TransformListener(tf_buffer)
    

    try:
        tf_map_cameralink = tf_buffer.lookup_transform("map","camera_depth_optical_frame", rospy.Time(),rospy.Duration(1.0))
    except (tf2_ros.LookupException,tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e: 
        rospy.logwarn(f"Failed to lookup transform from 'camera_depth_optical_frame' to 'map'. Exception: {type(e).__name__}")
        tf_map_cameralink = None

    #compute the center and angle of the person
    ppl_pos_ang = compute_centroids(rgb,depth,masks,intrinsics,use_pcd=False)
    for person, rel in zip(ppl_pos_ang,reliability):
        if person[0][0] != 0 and person[0][1] != 0 and person[0][2] != 0:

            x = person[0][0]
            y = person[0][1]
            z = person[0][2]
            #theta = person[1]  
            
            if tf_map_cameralink:
                tf_camera = tf2_geometry_msgs.PointStamped()
                tf_camera.header.frame_id = "camera_depth_optical_frame"
                tf_camera.header.stamp = rospy.Time(0)
                tf_camera.point.x = x
                tf_camera.point.y = y
                tf_camera.point.z = z

                #orientation_camera = tf.transformations.quaternion_from_euler(0,0,theta)
                
                #Transform the coordinates of the person from the camera to the map
                try:
                    tf_map = tf_buffer.transform(tf_camera, "map")
                    transformed_x = tf_map.point.x
                    transformed_y = tf_map.point.y
                    transformed_z = tf_map.point.z
                    #transformed_orientation = tf.transformations.quaternion_multiply(tf_map_cameralink.trasform.rotation,orientation_camera)
                
                    pos_map = geometry_msgs.msg.PointStamped()

                    pos_map.header.frame_id = "map"
                    pos_map.header.stamp = tf_map.header.stamp
                    pos_map.point.x = transformed_x
                    pos_map.point.y = transformed_y
                    pos_map.point.z = transformed_z
                
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.logwarn("Failed to transform point from 'camera_depth_optical_frame' to 'map'.")   
                    
                #Check whose position it belongs to, if its someone new it adds the person, return the id
                person_id = person_buffer.checkIfSamePerson(pos_map)
                #Get the persons Velocity
                person_vel = person_buffer.calculateVelocity(person_id)

                #Create the message to publish        
                people_msg = people_msgs.msg.People()
                people_msg.header.frame_id = "map"
                people_msg.header.stamp = pos_map.header.stamp
                person_msg = people_msgs.msg.Person()
                person_msg.name = "person" + str(person_id)
                person_msg.position = pos_map.point
                person_msg.velocity = person_vel
                person_msg.reliability = rel
                
                people_msg.people.append(person_msg)

                #Publish the message
                person_pub.publish(people_msg)
