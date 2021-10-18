#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random

from geometry_msgs.msg import Twist

import tf


import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs

import yaml
import os

from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image
import numpy as np



class NaviBot():
    def __init__(self):
        waypoint_file = rospy.get_param('/waypoint_file_path','/home/handaru/catkin_ws/src/burger_war_dev/burger_war_dev/scripts/waypoints.yaml')

        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.waypoints = self.waypoint_load(waypoint_file)
        
        self.img = None
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('image_raw', Image, self.imageCallback)

    def imageCallback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        lim = []
        lim.append([340, 18, 230, 140])  # 赤
        circles, scale = self.find_target(self.img, lim)
        cv2.imshow("Image window", scale)
        cv2.waitKey(1)

    def find_target(self,image, lim):
        # hsv空間の作成
        limit = lim[0]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV_FULL)
        h = hsv[:, :, 0]
        s = hsv[:, :, 1]
        # 取得画像のサイズでhsv空間の配列を作成
        mask = np.zeros(h.shape, dtype=np.uint8)
        # 表示用にmaskと同じサイズの配列を作成
        hsv_scale = np.copy(mask)
        # hsvのしきい値で二値化
        if limit[0] > limit[1]:
            mask[((h > limit[0] * (255/360)) | (h < limit[1] * (255/360)))
                & ((limit[3] < s) & (s < limit[2]))] = 255
        else:
            mask[((h > limit[0] * (255/360)) & (h < limit[1] * (255/360)))
                & ((limit[3] < s) & (s < limit[2]))] = 255
        # 輪郭計算
        _, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        circles = []
        # 計算した輪郭をなめらかにマルっと包み込む
        for contour in contours:
            approx = cv2.convexHull(contour)
            area = cv2.contourArea(approx)
            # デカすぎor小さすぎな領域はスキップ
            if area < 1e2 or 1e5 < area:
                continue
            # 最小外接円で近似
            (x, y), radius = cv2.minEnclosingCircle(approx)
            # 返り値準備
            center = (int(x), int(y))
            radius = int(radius)
            circles.append(np.array((center, radius)))
            hsv_scale = mask
        return circles, hsv_scale



    def waypoint_load(self, file_path):
        with open(file_path) as f:
            waypoints = yaml.safe_load(f)
            print(waypoints)
            print("-" *30)

            return waypoints
#            print(yml['waypoints'][0])


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
        r = rospy.Rate(5) # change speed 5fps

        while not rospy.is_shutdown():
            for pos in self.waypoints['waypoints']:
                print(pos['x'])
                self.setGoal(pos['x'],pos['y'],pos['th'])



if __name__ == '__main__':
    rospy.init_node('navirun')
    bot = NaviBot()
    bot.strategy()
