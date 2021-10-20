#!/usr/bin/env python

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math

import yaml
import tf
import roslib.packages

path = roslib.packages.get_pkg_dir('burger_war_dev') + '/scripts/waypoints.yaml'
waypoint_file = rospy.get_param('/waypoint_file_path',path)

topic = 'visualization_marker_array'
publisher = rospy.Publisher(topic, MarkerArray)

rospy.init_node('register')

markerArray = MarkerArray()

count = 0
MARKERS_MAX = 100

def waypoint_load(file_path):
    with open(file_path) as f:
        waypoints = yaml.safe_load(f)
        print(waypoints)
        print("-" *30)

        return waypoints


points = waypoint_load(waypoint_file)

while not rospy.is_shutdown():

    for pos in points['waypoints']:
        q=tf.transformations.quaternion_from_euler(0,0,-pos['th'])        

        marker = Marker()
        marker.header.frame_id = "/map"
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0


        marker.pose.position.x = -pos['x']
        marker.pose.position.y = -pos['y']
        marker.pose.position.z = 0
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.w = q[2]
        marker.pose.orientation.z = q[3]
        markerArray.markers.append(marker)

        # Renumber the marker IDs
        id = 0
        for m in markerArray.markers:
            m.id = id
            id += 1

        # Publish the MarkerArray
        publisher.publish(markerArray)


    rospy.sleep(1)
