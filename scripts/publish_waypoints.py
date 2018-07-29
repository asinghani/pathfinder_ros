#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from tf import TransformListener
from time import sleep

pub = None
robotTf = None
odomTf = None
tfListener = None

def newPoint(pose):
    print("Recieved point")
    global pub
    arr = PoseArray()
    arr.header.frame_id = odomTf

    robotPos = Pose()

    t = tfListener.getLatestCommonTime(robotTf, odomTf)
    position, rotation = tfListener.lookupTransform(odomTf, robotTf, t)
    robotPos.position.x = position[0]
    robotPos.position.y = position[1]
    robotPos.position.z = 0.0

    robotPos.orientation.x = rotation[0]
    robotPos.orientation.y = rotation[1]
    robotPos.orientation.z = rotation[2]
    robotPos.orientation.w = rotation[3]

    arr.poses.append(robotPos)
    arr.poses.append(pose.pose)

    pub.publish(arr)

if __name__ == "__main__":
    rospy.init_node("publish_waypoints", anonymous=True)

    robotTf = rospy.get_param("~robot_tf", "/base_link")
    odomTf = rospy.get_param("~odom_tf", "/odom")
    tfListener = TransformListener()

    print("Started")
    pub = rospy.Publisher("/pathfinder_ros/waypoints", PoseArray, queue_size=10)

    rospy.Subscriber("/move_base_simple/goal", PoseStamped, newPoint, queue_size=1)

    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(0.05)
