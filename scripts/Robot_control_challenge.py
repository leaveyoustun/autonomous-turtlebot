#!/usr/bin/env python3
# −*− coding : utf −8 −*−

"""
ROS Project

Author: Liviu Stan and Guillaume Oudet

Master ISI - Sorbonne Universite


Robot_control_challenge.py

This code has been made to pass the challenges of the project

"""

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import time
import math
from scipy.ndimage import gaussian_filter1d
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import tf
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion
from ObstacleManager import ObstacleManager


class RobotController:
    def __init__(self):
        # Initialization of node
        rospy.init_node('robot_control_node', anonymous=True)
        
        
        # Initialization of publisher and subcribers
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub_laser = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.sub_image = rospy.Subscriber('/camera/image', Image, self.image_callback)
        self.sub_odom = rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # Attribute which knoiw what challenge the robot is doing
        self.compteur = 0

        # initialzation of the odometer
        self.robot_position = [0,0]
        self.robot_orientation = 0
        
        self.tab_error = []
        self.calcul = True
        


        # Distance thresholds
        self.minimal_obstacle_distance = 0.5
        self.distance_emergency = 0.2

        # Booleans
        self.obstacles_done = False
        self.obstacle_phase = False
        self.obstacle_emergency=False


        # Sensors data
        self.laser_data = {}
        self.image_data = {}

        # Time varables
        self.last_detection_time = 0
        self.detection_interval = 15

        # Line following parameters
        self.speed_line = 0.15
        self.white_offset = 10
        self.yellow_offset = 0
        self.line_Kp = 8
        self.min_speed_coef_line = 0.5

        # Obstacles parameteres
        self.speed_obstacles = 0.075
        self.obstacles_Kp = 0.75
        self.min_speed_coef_obstacles = 0.1
        self.desired_angle = 9
        self.distance_for_turn = 0.20
        self.center_found = False
        self.centre_entre_obstacles = None
        self.ob_manager = None

        # Corridor parameters
        self.speed_corridor = 0.15
        self.corridor_Kp = 1.2
        self.min_speed_coef_corridor = 1
        self.max_distance_corridor = 0.45


    # Method that will calculate the robot's position and orientation using the /odom topic
    def odom_callback(self, data):
        self.robot_position = [data.pose.pose.position.x, data.pose.pose.position.y]
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w)
        euler_angles = euler_from_quaternion(quaternion)
        self.robot_orientation = math.degrees(euler_angles[2])


    # Method that will store the lidar that we need in the laser_data attribute    
    def process_laser_data(self, msg):
        
        # Getting the data from the lidar and putting it in arrays
        ######################################################################
        #--------------------------------------------------------------------#
        
        # We get the actual data from the lidar
        front_small = np.concatenate((msg.ranges[-20:], msg.ranges[:20]))
        left = np.array(msg.ranges[30:90])
        right = np.array(msg.ranges[270:330])
        front_big = np.concatenate((msg.ranges[-50:], msg.ranges[:50]))

        # We create masks to filter the infinity values 
        front_small_mask = np.isfinite(front_small)
        left_mask = np.isfinite(left)
        right_mask = np.isfinite(right)

        # We apply the masks to the arrays
        front_small_finite = front_small[front_small_mask]
        left_finite = left[left_mask]
        right_finite = right[right_mask]

        # If the arrays are empty it means there are no objects detected, so the
        # arrays will be empty, in order for it to not creat problems for the 
        # mean value we will calculate, we will add an infinity value in the array
        if len(front_small_finite) == 0:
            front_small_finite = [np.inf]
        if len(left_finite) == 0:
            left_finite = [np.inf]
        if len(right_finite) == 0:
            right_finite = [np.inf]

        # We filter the left and right distances
        sigma = 10  
        filtered_left = gaussian_filter1d(left_finite, sigma)
        filtered_right = gaussian_filter1d(right_finite, sigma)

        # We calculate the mean values of the distances
        front_small = np.mean(front_small_finite)
        left = np.mean(filtered_left)
        right = np.mean(filtered_right)

        #---------------------------------------------------------------------#
        #######################################################################


        # Booleans modifications
        #######################################################################
        #----------------------------------------------------------------------#
        
        # If we detect an object in front of the robot, the obstacle_phase boolean is
        # set to True, but we only have an obstacle phase in the track so we want it 
        # to be set to True a single time in the script. For this we created another
        # boolean, obstacles_done, that will make sure we only do it once. Also, the
        # obstacle phase will begin only when the counter is set to 2.
        if front_small < self.minimal_obstacle_distance and not self.obstacles_done and self.compteur == 2:
            self.obstacle_phase = True
            self.obstacles_done = False
    
        # If there's no object in the front area of the robot, it means the obstacle phase
        # is over

        if len(front_big[np.isfinite(front_big)]) == 0:
            self.obstacle_phase = False

        # Boolean that will be used for the emergency stop
        if np.mean(front_small) < self.distance_emergency :
            self.obstacle_emergency=True
        else:
            self.obstacle_emergency=False

        #-----------------------------------------------------------------------#
        #########################################################################
            
        
        # Creation of the ObstacleManager object that will create the obstacles map
        #########################################################################
        #-----------------------------------------------------------------------#
        self.ob_manager = ObstacleManager()
           
        self.ob_manager.update_robot_position(self.robot_position, self.robot_orientation)
        self.ob_manager.update_obstacles(msg)

        #-----------------------------------------------------------------------#
        #########################################################################
            


        # We calculate the minimum distance to the robot and the relative angle this
        # distance makes with the robot
        #########################################################################
        #-----------------------------------------------------------------------#

        # We decided to use slices of ten degrees to take in account the noise of 
        # the lidar and random values it gives

        # Slicing of the lidar data
        distances_tab = []
        for i in range(36):
            distances_temp = np.array(msg.ranges[i*10:i*10+10])
            distances_mask = np.isfinite(distances_temp)
            distances_finite = distances_temp[distances_mask]
            if len(distances_finite) == 0:
                distances_finite = [np.inf]
            
            distances_tab.append(np.mean(distances_finite))
        distances = distances_tab

        # We calculate the minimum angle on the left and right side
        
        angle_min_left = np.argmin(distances[0:18])
        angle_min_right = -np.argmin(distances[:-18:-1])

        #-----------------------------------------------------------------------#
        #########################################################################
        

        # We store the data we collected in an attribute to be able to use it later
        self.laser_data = {
            "front distance small": front_small,
            "front distance big": front_big,
            "left distance": left,
            "right distance": right,
            "minimum angle right": angle_min_right,
            "minimum angle left": angle_min_left
        }


    def laser_callback(self, data):

        # We process the data with the method we createdd
        self.process_laser_data(data)

        # Data from the lidar
        front_small = self.laser_data["front distance small"]
        front_big = self.laser_data["front distance big"]
        left = self.laser_data["left distance"]
        right = self.laser_data["right distance"]
        ob_manager = self.ob_manager
        angle_min_left = self.laser_data["minimum angle left"]
        angle_min_right = self.laser_data["minimum angle right"]

        # We create the Twist object
        twist = Twist()


        # Obstacle avoidance algorithm
        if self.compteur == 2 and self.obstacle_phase == True:

            # Speed and proportional gain
            speed = self.speed_obstacles
            Kp = self.obstacles_Kp
            
            # We calculate the center between the first two obstacles
            if ob_manager:
                if len(ob_manager.obstacles) > 1 and not self.center_found:
                    self.centre_between_obstacles = ob_manager.get_milieu_entre_obstacles()
            
            # We calculate the side of the first obstacle using the center
            if self.centre_between_obstacles:
                cote_obstacle = ob_manager.get_cote_obstacle(self.centre_between_obstacles)
            else:
                cote_obstacle = "we don't know"

            # Error calculation for left and right (we will use a way higher Kp if the angle
            # is higher than the desired angle because we don't want the robot to understeer
            # when overtaking an obstacle and a lower Kp when the angle is smaller to avoid
            # oversteering and making sure the robot sticks to the obstacle in front of it )
            if cote_obstacle == "left":
                if np.abs(angle_min_left)>self.desired_angle:
                    Kp = 8
                error = (angle_min_left - self.desired_angle) / self.desired_angle

            elif cote_obstacle == "right":
                if np.abs(angle_min_right)>self.desired_angle:
                    Kp = 8
                error = (angle_min_right + self.desired_angle) / self.desired_angle

            else:
                error = 0

            # We calculate the command
            command = (Kp * error) 

            # We enter the command in the Twist object
            twist.angular.z = command
            twist.linear.x = (speed)*max((1-np.abs(error)), self.min_speed_coef_obstacles)
            
            self.pub.publish(twist)


        # Corridor algorithm          
        if self.compteur == 3 :
            
            # Error calculation
            error = (left - right)/self.max_distance_corridor
            # Passing the command to the robot
            command = self.corridor_Kp * error
            twist.linear.x = (self.speed_corridor)*max((1-np.abs(error)), self.min_speed_coef_corridor)
            twist.angular.z = command
            self.pub.publish(twist)

            

    # Method that will store the image data we need in the image_data attribute
    def process_image_data(self, data):

        cvBridge = CvBridge()

        # We transform the image to openCV format , data is the original image from ROS
        cvImage = cvBridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

        # We get the dimensions of the image
        height, width = cvImage.shape[:2]

        # We calculate the slicing indexes
        mid_x, mid_y = width // 2, 3* height // 4

        # We change color represtation from BGR to HSV
        hsv = cv2.cvtColor(cvImage , cv2.COLOR_BGR2HSV)


        # Mask Red 
        red_min = np.array([160, 100, 100])
        red_max = np.array([179, 255, 255])
        mask_red = cv2.inRange(hsv[mid_y+height//8:, width//3:2*width//3], red_min, red_max)
        
        # Mask Yellow 
        yellow_min = np.array([20, 100, 100])
        yellow_max = np.array([40, 255, 255])
        mask_yellow = cv2.inRange(hsv[mid_y:, :mid_x], yellow_min, yellow_max)

        # Mask White
        lower_white = np.array([0, 0, 200])  
        upper_white = np.array([180, 50, 255])  
        mask_white = cv2.inRange(hsv[mid_y:, mid_x:], lower_white, upper_white)
        
    
        # We calculate the coordinates of the centers of each line
        ###########################################################
        #---------------------------------------------------------#

        # Yellow line
        M_yellow = cv2.moments(mask_yellow)
        cY_yellow = 0
        cX_yellow = 0
        yellow_line_detected = False
        if M_yellow['m00'] > 0:
            yellow_line_detected = True
            cX_yellow = int(M_yellow['m10'] / M_yellow['m00'])
            cY_yellow = int(M_yellow['m01'] / M_yellow['m00'])

        # White line
        M_white = cv2.moments(mask_white)
        cY_white = 0
        cX_white = 0
        white_line_detected = False
        if M_white['m00'] > 0:
            white_line_detected = True
            cX_white = int(M_white['m10'] / M_white['m00'])
            cY_white = int(M_white['m01'] / M_white['m00'])

        #----------------------------------------------------------------------#
        ########################################################################

        # We calculate the dimensions of the masks
        height_mask, width_mask = mask_yellow.shape[:2]
        
        
        # If we detect the red line, we increment the compteur attribute
        M_red = cv2.moments(mask_red)
        if M_red['m00'] > 0:
            if (time.time() - self.last_detection_time) > self.detection_interval:
                self.compteur +=1
                self.last_detection_time = time.time()
                #position_reference = self.robot_position
                #orientation_reference = self.robot_orientation

            
    
        # We store the data we need in the image_data attribute so we can use it later
        self.image_data = {
            "white line detected": white_line_detected,
            "yellow line detected": yellow_line_detected,
            "white x": cX_white,
            "white y": cY_white,
            "yellow x": cX_yellow,
            "yellow y": cY_yellow,
            "mask height": height_mask,
            "mask width": width_mask
        }
        

    def image_callback(self, data):

        # We update the image data
        self.process_image_data(data)

        # We store the image data in local variables for easier use
        white_line_detected = self.image_data["white line detected"]
        yellow_line_detected = self.image_data["yellow line detected"]
        cX_white = self.image_data["white x"]
        cY_white = self.image_data["white y"]
        cX_yellow = self.image_data["yellow x"]
        cX_yellow = self.image_data["yellow y"]
        height_mask = self.image_data["mask height"]
        widht_mask = self.image_data["mask width"]

        # We will follow the lines only in certain conditions decided by compteur
        if (self.compteur < 2) or (self.compteur == 2 and self.obstacle_phase == False) or (3 > self.compteur > 4):
            
            if white_line_detected:
                error = (widht_mask//2 + self.white_offset - cX_white)/(widht_mask + self.white_offset)
                speed = self.speed_line              
                
            elif yellow_line_detected:
                error = (widht_mask//2 + self.yellow_offset - cX_yellow)/(widht_mask//2 + self.yellow_offset)
                speed = self.speed_line

            else:    
                error = 0
                speed = 0.05

            command = self.line_Kp * error
            
            twist = Twist()
            twist.angular.z = command
            twist.linear.x = speed*max((1-np.abs(error)), self.min_speed_coef_line)
            self.pub.publish(twist)
            

        # For the beginning of the corridor, we will only follow the white line
        if (self.compteur > 2 and white_line_detected):
            
            if white_line_detected:
                error = (widht_mask//2 + self.white_offset - cX_white)/(widht_mask + self.white_offset)
                speed = self.speed_line
            else:    
                error = 0
                speed = 0.05

            command = self.line_Kp * error

            twist = Twist()
            twist.angular.z = command
            twist.linear.x = speed*max((1-np.abs(error)), self.min_speed_coef_line)
            self.pub.publish(twist)
        
            
        # at the last challenge, the robot stop, because the last challenge has not been treated
        # or  stop if the obstacle_emergency has been activated
        if self.compteur >= 5 or self.obstacle_emergency:
            twist = Twist()
            twist.angular.z = 0
            twist.linear.x = 0
            self.pub.publish(twist)
            

            

    def run(self):
        rospy.spin()


done = False

if __name__ == '__main__':

    if not done:

        controller = RobotController()
        done = True
    

    controller.run()