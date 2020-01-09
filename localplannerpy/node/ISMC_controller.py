#!/usr/bin/env python
import rospy, numpy
from geometry_msgs.msg import Twist,Pose2D,PoseStamped
from nav_msgs.msg import Odometry
from tf import transformations
class Point():
    def __init__(self):
        self.x = 0
        self.y = 0
        self.angle = 0
        self.v = 0
        self.w = 0

    def theta(self):
        self.cos = numpy.cos(self.angle)
        self.sin = numpy.sin(self.angle)

class SMC():
    def __init__(self):
        self.cmd_smc = Twist()
        self.refer = Point()
        self.refer.theta()
        self.real = Point()
        self.real.theta()
        self.init_err = 0.
        #self.subsmd = rospy.Subscriber('cmd_vel', Twist, self.cbcmd, queue_size=1)
        self.substate = rospy.Subscriber('state_ref', PoseStamped, self.cbstate, queue_size=1)
        self.subodom = rospy.Subscriber('odom', Odometry, self.cbodom, queue_size=1)
        self.pubsmd = rospy.Publisher('cmd_vel', Twist,queue_size=1)

    def cbstate(self, msg=PoseStamped()):
        self.refer.v= msg.pose.position.x
        self.refer.w = msg.pose.position.y
        self.refer.angle = msg.pose.position.z
        self.refer.theta()
        self.refer.x = msg.pose.orientation.x
        self.refer.y = msg.pose.orientation.y
        self.main()

    def cbodom(self, msg=Odometry()):
        self.real.angle = self.orientation_to_theta(msg.pose.pose.orientation)
        self.real.theta()
        self.real.x = msg.pose.pose.position.x
        self.real.y = msg.pose.pose.position.y
        self.real.v = msg.twist.twist.linear.x
        self.real.w = msg.twist.twist.angular.z

    def orientation_to_theta(self, orientation):
        ang = transformations.euler_from_quaternion((orientation.x,
                                                     orientation.y,
                                                     orientation.z,
                                                     orientation.w))
        return ang[2]


    def sat(self,s):
        delta = 0.3
        if s > delta:
            return 1.0
        elif s < -delta:
            return -1.0
        else:
            return s/delta

    def main(self):
        e_robot_d_ref_add = self.init_err
        maxv = 0.2
        maxw = 0.2
        refer = self.refer
        real = self.real
        ex = refer.x - real.x
        ey = refer.y - real.y
        ea = refer.angle - real.angle
        e_global = numpy.array([[ex],[ey],[ea]])
        #cosa = numpy.cos(ea)
        #sina = numpy.sin(ea)
        trans_global_robot= numpy.matrix([[real.cos, real.sin, 0.],[real.sin, real.cos, 0.],[0.,0.,1.]])
        e_robot = trans_global_robot * numpy.matrix(e_global)
        u_ref = numpy.array([[refer.v], [refer.w]])
        #trans_ref = numpy.matrix([[refer.cos,0], [refer.sin,0], [0,1]])
        f_1 = numpy.matrix([[refer.v * numpy.cos(e_robot[2,0])],[refer.v * numpy.sin(e_robot[2,0])], [refer.w]])
        f_2 = numpy.matrix([ [-1.0, e_robot[1]], [0., -e_robot[0]], [0., -1.0] ])
        e_robot_d_ref = f_1 + f_2 * numpy.matrix(u_ref)
        e_robot_d_ref_add += e_robot_d_ref
        #print e_robot_d_ref_add
        s_C = numpy.matrix([[-1.,0.,0.],[0.,0.,-1.]])
        e_robot_init = numpy.array([[0.],[0.],[0.]])
        s = s_C*(e_robot - e_robot_init - e_robot_d_ref_add)
        s.tolist()
        u_d_v = - maxv * self.sat(s[0])
        u_d_w = - maxw * self.sat(-e_robot[1]*s[0] + s[1])
        #u_real = u_ref+ numpy.matrix([[u_d_v],[u_d_w]])
        x = u_ref[0] + u_d_v
        z = u_ref[1] + u_d_w
        self.init_err = e_robot_d_ref_add
        self.cmd_smc.linear.x = x
        self.cmd_smc.angular.z = z
        if not refer.v == 0.:
            self.pubsmd.publish(self.cmd_smc)
        else:
            stop = Twist()
            stop.linear.x = 0.0
            stop.angular.z = 0.
            self.pubsmd.publish(stop)

#todo: debug for the s function
#todo: reading the matlab code
def main():
    rospy.init_node('ISMC_controller')
    try:
        S = SMC()
    except rospy.ROSInterruptException:
        pass
    #rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            pass
        except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
    main()
