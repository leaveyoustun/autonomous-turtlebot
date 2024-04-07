#!/usr/bin/env python3
# −*− coding : utf −8 −*−

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
#import click


keys = {'\x1b[A':'up', '\x1b[B':'down', '\x1b[C':'right', '\x1b[D':'left', 's':'stop', 'q':'quit'}
# Callback function for reading turtlesim node output
def laser_callback(msg):
    front = np.concatenate((msg.ranges[-20:], msg.ranges[:20]))
    left = np.array(msg.ranges[70:110])
    back = np.array(msg.ranges[160:200])
    right = np.array(msg.ranges[250:290])

    front_mask = np.isfinite(front)
    left_mask = np.isfinite(left)
    back_mask = np.isfinite(back)
    right_mask = np.isfinite(right)

    front_finite = front[front_mask]
    left_finite = left[left_mask]
    back_finite = back[back_mask]
    right_finite = right[right_mask]

    if len(front_finite) == 0:
        front_finite = [np.inf]
    if len(left_finite) == 0:
        left_finite = [np.inf]
    if len(right_finite) == 0:
        right_finite = [np.inf]
    if len(back_finite) == 0:
        back_finite = [np.inf]

    front = np.mean(front_finite)
    left = np.mean(left_finite)
    back = np.mean(back_finite)
    right = np.mean(right_finite)

    info = Float32MultiArray()
    
    info.data = [front, left, right, back]

    pub.publish(info)

    #mykey = click.getchar()
    #if mykey in keys.keys():
        #char=keys[mykey]


def image_callback(data):
    try:
        cvBridge = CvBridge()

        # Transform the image to openCV format , msg is the original image from ROS
        cvImage = cvBridge.imgmsg_to_cv2(data, desired_encoding='bgr8')


        # Get the dimensions of the image
        height, width = cvImage.shape[:2]

        # Calculate the slicing indexes
        mid_x, mid_y = width // 2, 3* height // 4

        # Change color represtation from BGR to HSV
        hsv = cv2.cvtColor( cvImage , cv2.COLOR_BGR2HSV)

        # Mask Yellow 
        yellow_min = np.array([20, 100, 100])
        yellow_max = np.array([40, 255, 255])
        mask_yellow = cv2.inRange(hsv[mid_y:, :mid_x], yellow_min, yellow_max)

        # Mask Red 
        red_min = np.array([160, 100, 100])
        red_max = np.array([179, 255, 255])
        mask_red = cv2.inRange(hsv[mid_y+height//8:, width//3:2*width//3], red_min, red_max)



        # Mask White
        lower_white = np.array([0, 0, 200])  
        upper_white = np.array([180, 50, 255])  
        mask_white = cv2.inRange(hsv[mid_y:, mid_x:], lower_white, upper_white)
        

        # Cropped images
        #cropped_yellow = mask_yellow[mid_y:, :mid_x]
        #cropped_white = mask_white[mid_y:, mid_x:]

        # Trouvez le centre des lignes
        M_yellow = cv2.moments(mask_yellow)
        if M_yellow['m00'] > 0:
            cX_yellow = int(M_yellow['m10'] / M_yellow['m00'])
            cY_yellow = int(M_yellow['m01'] / M_yellow['m00'])
            # Utilisez cX et cY pour guider le robot

        # Trouvez le centre des lignes
        M_red = cv2.moments(mask_red)
        if M_red['m00'] > 0:
            twist.linear.x = 0
            twist.angular.z = 0
            

        # Trouvez le centre des lignes
        M_white = cv2.moments(mask_white)
        if M_white['m00'] > 0:
            cX_white = int(M_white['m10'] / M_white['m00'])
            cY_white = int(M_white['m01'] / M_white['m00'])
            # Utilisez cX et cY pour guider le robot

        cX_yellow_img = cX_yellow
        cX_white_img = width//2 + cX_white
        cX = 100
        cY = (4*cY_white + 4*cY_yellow)//2

        
        # Publiez des commandes de vitesse basées sur le centre de la ligne
        twist = Twist()
        if cX_yellow_img < width // 4-5:
            twist.linear.x = 0.2
            twist.angular.z = 1
        elif cX_yellow_img > width // 4+5:
            twist.linear.x = 0.2
            twist.angular.z = -1
        else:
            twist.linear.x = 0.2
            twist.angular.z = 0
        
        pub_move.publish(twist)

        #pixel_value = mask_yellow[214, 70 ]
        #print(pixel_value)
        #print(mask)
        #rospy.loginfo("Pixel value left: {}".format(pixel_value))
        
        #print(cX,)
        cv2.circle(mask_white, (cX_white, cY_white), 1, (0, 0, 0), thickness=2)
        cv2.circle(mask_yellow, (cX_yellow, cY_yellow), 1, (0, 0, 0), thickness=2)
        cv2.circle(cvImage, (cX, cY), 1, (255, 0, 0), thickness=2)
        cv2.imshow("image", cvImage)
        cv2.imshow("Mask yellow", mask_yellow)
        cv2.imshow("Mask white", mask_white)
        cv2.waitKey(1)
    
        



    except CvBridgeError as e:
        rospy.logerr(e)

if __name__ == '__main__':
    try:

        rospy.init_node("line_follower", anonymous=True)

        image_sub=rospy.Subscriber("/camera/image",Image, image_callback)
        pub = rospy.Publisher("/presence_objets", Float32MultiArray, queue_size=10)
        pub_move = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        laser_sub =rospy.Subscriber("/scan",LaserScan, laser_callback )
        
        # And then ... wait for the node to be terminated
        rospy.spin()
        cv2.destroyAllWindows()


    except rospy.ROSInterruptException:
        pass

