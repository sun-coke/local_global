#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
class Vel_Cal():
    def __init__(self):
        self.joint = []
        self.odom = []
        self.info = Twist()
        self.sub = rospy.Subscriber('/joint_states',JointState, self.subcallback,queue_size=3)
        self.pub = rospy.Publisher('/vel_cal',Twist, queue_size=1)

    def subcallback(self,msg):
        self.joint.append(msg.position)
        vel = 30.0 * (self.joint[-1][0] - self.joint[0][0] + self.joint[-1][1] - self.joint[0][1]) * 0.22 / 4.0
        self.info.linear.x = vel
        ## raidius error
        if len(self.joint)>3:
            self.joint.pop(0)

    def main(self):
        self.pub.publish(self.info)
if __name__ == '__main__':

    rospy.init_node('vel_cal')
    rate = rospy.Rate(10) # 10hz
    main = Vel_Cal()
    while not rospy.is_shutdown():
        main.main()
        print main.info.linear.x
        rate.sleep()