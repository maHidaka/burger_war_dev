#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import random


from geometry_msgs.msg import Twist
from obstacle_detector.msg import Obstacles
import tf


import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs


import yaml
import os
import roslib.packages

# Ref: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/

#from std_msgs.msg import String
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError
import cv2




class NaviBot():
    def __init__(self):
        path = roslib.packages.get_pkg_dir('burger_war_dev') + '/scripts/waypoints.yaml'
        waypoint_file = rospy.get_param('/waypoint_file_path',path)

        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.obstacles = rospy.Subscriber("obstacles", Obstacles, self.detect)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.waypoints = self.waypoint_load(waypoint_file)


    def waypoint_load(self, file_path):
        with open(file_path) as f:
            waypoints = yaml.safe_load(f)
            print(waypoints)
            print("-" *30)

            return waypoints
#            print(yml['waypoints'][0])

    def detect(self, data):
        for c in data.circles:


            if -0.3 < c.center.x < 0.3 :
                if -0.3 < c.center.y < 0.3 :
                    print("center")
                    print("-"*5)
                    continue
            if -0.7 < c.center.x < -0.4 :
                if -0.7 < c.center.y < -0.3 :
                    print("r_back")
                    print("-"*5)
                    continue
            if -0.7 < c.center.x < -0.4 :
                if 0.4 < c.center.y < 0.7 :
                    print("l_back")
                    print("-"*5)
                    continue
            if 0.3 < c.center.x < 0.6 :
                if -0.7 < c.center.y < -0.4 :
                    print("r_front")
                    print("-"*5)
                    continue
            if 0.3 < c.center.x < 0.7 :
                if 0.3 < c.center.y < 0.7 :
                    print("l_front")
                    print("-"*5)
                    continue

            if -0.8 < c.center.x*0.707 < 0.8 :
                if -0.8 < c.center.y*0.707 < 0.8 :
                    print("in range")
                else :
                    print("out")
                    continue
            else :
                print("out")
                continue
            
            print(c.center)
            print("-"*5)
        print("-"*20)
        if not c == "":
            self.setGoal(c.center.x,c.center.y,0)


    def setGoal(self,x,y,yaw):
        self.client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        # Euler to Quartanion
        q=tf.transformations.quaternion_from_euler(0,0,yaw)        
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()        


    def strategy(self):
        r = rospy.Rate(1) # change speed 5fps

        while not rospy.is_shutdown():
            for pos in self.waypoints['waypoints']:
                print(pos['x'])
                #self.setGoal(pos['x'],pos['y'],pos['th'])




if __name__ == '__main__':
    rospy.init_node('navirun')
    bot = NaviBot()
    bot.strategy()
