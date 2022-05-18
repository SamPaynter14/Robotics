#!/usr/bin/env python2.7

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
import cv_bridge
import numpy as np
#importing relavant modules



class robot_control:

    def turn_left(self):
        t = Twist()
        t.angular.z = 0.6
        t.linear.x = 0.0
        return t
        #turns the robot left

    def turn_right(self):
        t = Twist()
        t.angular.z = -0.6
        t.linear.x = 0.0
        return t
        #turns robot right
    
    def forwards(self):
        t = Twist()
        t.linear.x = 0.2
        return t
        #moves robot forwards

    def stop(self):
        t = Twist()
        t.linear.x = 0.0
        t.angular.z = 0.0
        return t
        #stops robot
class Chatter:

    def __init__(self):
        rospy.init_node('chatter')
        self.publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.laser_cb)
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        #publishes to the velocity topic and subscribes to the scan and image raw topcs
        
        self.bridge = cv_bridge.CvBridge()

        self.redFlag = False
        self.greenFlag = False
        #variables used to store whether the robot sees red, blue, or green
        
        self.error = None
        #stores the error 

        



    def laser_cb(self, laser_msg):

        if laser_msg.ranges[0] < 0.75:
            self.publisher.publish(rc.turn_left())
            #if the laser scan detcts a wall to the right it will turn left
        elif laser_msg.ranges[639] < 0.75:
            self.publisher.publish(rc.turn_right()) 
            #if the laserscan detects a waal to the left it turns right 
        elif self.redFlag == True:
            self.publisher.publish(rc.turn_left())
            #if red is detected, the robot turns left
        elif self.greenFlag == True:
            self.publisher.publish(rc.stop())
            #if the robot detects green, the robot stops
        else:
            self.publisher.publish(rc.forwards())
            #otherwise ther obot moves forwards


    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        h, w, d = image.shape
        #gets image dimentions
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        #gets image from robot and connects to and opencv image

        lower_green = np.array([36, 100, 100])
        upper_green = np.array([70, 255, 255])
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        #gets the ranges for the colour green

        lower_red = np.array([0, 60, 60])
        upper_red = np.array([15, 255, 255])
        mask_red = cv2.inRange(hsv, lower_red, upper_red)
        #gets the ranges for  the red

        
        greenM = cv2.moments(mask_green)
        redM = cv2.moments(mask_red)
        #checks for an y of these colours in the image

        
        if redM['m00'] > 0:
            cx = int(redM['m10']/redM['m00'])
            self.error = cx - w/2
            self.redFlag = True
            # redflag set to true if there is any red
        else:
            self.redFlag = False

        if greenM['m00'] > 0:
            cx = int(greenM['m10']/greenM['m00'])
            self.error = cx - w/2
            self.greenFlag = True
            #greenflag set to true if there is any green
        else:
            self.greenFlag = False


    
    def run(self):
        rospy.spin()

rc = robot_control()
c = Chatter()
c.run()
