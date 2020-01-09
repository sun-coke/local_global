#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

rospy.init_node('pathtest')
pathpub = rospy.Publisher('path', Path, queue_size=1)
path = Path()
path.header.frame_id = "odom"
for i in range(10):
    pose = PoseStamped()
    pose.header.frame_id = "odom"
    pose.pose.position.x = i
    pose.pose.position.y = i
    path.poses.append(pose)
while not rospy.is_shutdown():
    pathpub.publish(path)
