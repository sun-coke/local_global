#!/usr/bin/env python
#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import  Twist,Pose2D ,Pose
import Bezier6D_v1_1

class Subgoal_scan():

    def __init__(self):
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.cbOdom, queue_size = 1)
        self.pub_path = rospy.Publisher('/sub_goal', Path, queue_size=1)

        self.goal = Pose()
        self.odom = Odometry()
        self.goal_pose_2d = Pose2D()
        self.twist = Twist()
        self.front_obstacle = []
        self.is_free_to_go = True
        self.safe = 0.3

        # done: clarify the class of path points

    def cbOdom(self, msg):
        self.odom = msg


    def main(self):
        goal = Pose()
        goal.position.x = 0.215663174186
        goal.position.x =0.282741111194
        goal.orientation.z =0.408837880109
        goal.orientation.w = 0.912607028128
        B = Bezier6D_v1_1.Bezier(self.odom, goal)
        B.main()
        state_ = B.state
        rate = rospy.Rate(10)
        cmd_ = []
        for i in state_:
            cmd = Twist()
            cmd.linear.x = i.v
            cmd.angular.z = i.w
            cmd_.append(cmd)
        B.pubpath()
        B.pubControl()
        for i in range(len(state_)):
            B.pubcmd(cmd_[i])
            B.pubstate(i)
            i += 1
            rate.sleep()
        #self.path.poses.pop(0)
        return

if __name__ == '__main__':
    rospy.init_node('test')
    S = Subgoal_scan()
    S.main()