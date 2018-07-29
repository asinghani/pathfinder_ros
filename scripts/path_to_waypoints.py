#!/usr/bin/env python

"""
Script for taking in a Path message (outputted by a global planner such as Navfn or VoronoiPlanner) and converting it to a set of waypoints (interpolating angles between each point on the path) that can be used with pathfinder_node to create a spline path
"""

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Path
from time import sleep
import tf
import math

everyN = 1
pub = None

def newPath(path):
    print("Recieved path")
    global pub

    arr = PoseArray()
    arr.header = path.header

    for poseStamped in path.poses[::everyN]:
        arr.poses.append(poseStamped.pose)

    if len(path.poses) % everyN != 0:
        arr.poses.append(path.poses[-1].pose)

    for i in range(len(arr.poses) - 2):
        yaw1 = math.atan2(arr.poses[i + 2].position.y - arr.poses[i + 1].position.y, arr.poses[i + 2].position.x - arr.poses[i + 1].position.x)
        yaw2 = math.atan2(arr.poses[i + 1].position.y - arr.poses[i].position.y, arr.poses[i + 1].position.x - arr.poses[i].position.x)
        yaw = (yaw1 + yaw2) / 2.0


        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        arr.poses[i].orientation.x = quaternion[0]
        arr.poses[i].orientation.y = quaternion[1]
        arr.poses[i].orientation.z = quaternion[2]
        arr.poses[i].orientation.w = quaternion[3]

    arr.poses[-2].orientation = arr.poses[-3].orientation
    arr.poses[-1].orientation = arr.poses[-2].orientation

    pub.publish(arr)

if __name__ == "__main__":
    rospy.init_node("publish_waypoints", anonymous=True)

    pathTopic = rospy.get_param("~path_topic", "/move_base/VoronoiPlanner/plan")
    everyN = rospy.get_param("~every_n", 30)

    print("Started")
    pub = rospy.Publisher("/pathfinder_ros/waypoints", PoseArray, queue_size=10)

    rospy.Subscriber(pathTopic, Path, newPath, queue_size=1)

    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(0.05)
