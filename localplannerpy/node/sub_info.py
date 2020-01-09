#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,PoseStamped
cmd=[[0,0]]
odom = []
state = [[0., 0., 0., 0., 0.]]

def cb(msg=Odometry()):
    x =msg.pose.pose.position.x
    y =msg.pose.pose.position.y
    v_x = msg.twist.twist.linear.x
    w_z = msg.twist.twist.angular.z
    e_x = state[-1][3] - x
    e_y = state[-1][4] - y
    logv.write(str([cmd[-1][0],cmd[-1][1],v_x,w_z]))
    logt.write(str([x,y,e_x,e_y]))
    logs.write(str(state[-1]))
def cb_cmd(msg=Twist()):
    cmd_v =msg.linear.x
    cmd_w =msg.angular.z
    cmd.append([cmd_v,cmd_w])

def cbstate(msg = PoseStamped()):
    state.append( [msg.pose.position.x,msg.pose.position.y,msg.pose.position.z,msg.pose.orientation.x,msg.pose.orientation.y])



logv = open('v_w.txt','w')
logv.write('cmd_real,real_vel')
logt = open('odom.txt','w')
logs = open('ref.txt','w')

rospy.init_node('sub_info')
subodom = rospy.Subscriber('odom',Odometry,cb,queue_size=1)
sub_cmd = rospy.Subscriber('cmd_vel',Twist,cb_cmd,queue_size=1)
sub_ref = rospy.Subscriber('state_ref',PoseStamped,cbstate,queue_size=1)
while not rospy.is_shutdown():
    try:
        pass
    except rospy.ROSInterruptException:
        pass

logv.close()
logt.close()
logs.close()
