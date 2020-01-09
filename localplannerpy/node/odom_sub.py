#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,PoseStamped
class Info():
    def __init__(self):
        self.cmd = [[0, 0]]
        self.odom = []
        self.state = [[0., 0., 0., 0., 0.]]
        self.logv = open('v_w.txt', 'w')
        self.logt = open('odom.txt', 'w')
        self.logs = open('ref.txt', 'w')
        self.subodom = rospy.Subscriber('odom', Odometry, self.cb, queue_size=1)
        self.sub_cmd = rospy.Subscriber('cmd_vel', Twist, self.cb_cmd, queue_size=1)
        self.sub_ref = rospy.Subscriber('state_ref', PoseStamped, self.cbstate, queue_size=1)



    def cb(self,msg=Odometry()):
        x =msg.pose.pose.position.x
        y =msg.pose.pose.position.y
        v_x = msg.twist.twist.linear.x
        w_z = msg.twist.twist.angular.z
        e_x = self.state[-1][3] - x
        e_y = self.state[-1][3] - y
        self.logt.write(str([x, y, e_x, e_y]))
        self.logs.write(str(self.state[-1]))
        self.logv.write(str([self.cmd[-1][0], self.cmd[-1][1], v_x, w_z]))


    def cb_cmd(self,msg=Twist()):
        cmd_v =msg.linear.x
        cmd_w =msg.angular.z
        self.cmd.append([cmd_v,cmd_w])

    def cbstate(self, msg = PoseStamped()):
        self.state.append( [msg.pose.position.x,msg.pose.position.y,msg.pose.position.z,msg.pose.orientation.x,msg.pose.orientation.y])

    def close(self):
        self.logs.close()
        self.logt.close()
        self.logv.close()
    #def write(self):



if __name__ == '__main__':
    rospy.init_node('sub_info')
    I = Info()
    while not rospy.is_shutdown():
        try:
            pass
        except rospy.ROSInterruptException,KeyboardInterrupt:
            I.close()
    if rospy.core.is_shutdown():
        raise rospy.exceptions.ROSInterruptException("ROS shutdown request")