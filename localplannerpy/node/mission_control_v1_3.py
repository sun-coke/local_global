#!/usr/bin/env python
import rospy, numpy, math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist,Pose2D ,Pose
from tf import transformations
import Bezier6D_v1_1

class Subgoal_scan():

    def __init__(self):
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.cbOdom, queue_size = 1)
        self.sub_laser = rospy.Subscriber('/scan', LaserScan, self.cbLaser, queue_size=1)
        self.sub_goal = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.cbGoal, queue_size=1)
        self.pub_path = rospy.Publisher('/sub_goal', Path, queue_size=1)
        #self.pub_cmd = rospy.Publisher('cmd_vel',Twist,queue_size=1)
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
        #self.link_angle()

    def cbLaser(self, msg= LaserScan()):
        self.point_set = msg.ranges
        self.max_range = msg.range_max
        self.min_range = msg.range_min

    def cbGoal(self, msg):
        self.goal = msg.pose
        self.link_angle()
        self.path = Path()
        self.goal_pose_2d.x = self.goal.position.x
        self.goal_pose_2d.y = self.goal.position.y
        self.goal_pose_2d.theta = self.goal_yaw
        self.list_H = [[self.goal_pose_2d,[0.0, 0.0]]]
        self.list_T = [[self.goal_pose_2d,[0.0, 0.0]]]
        self.detect_block()

    def cost_part(self,side_part_list,v_g):
        end  = side_part_list[0]
        pos = self.pol2cart(end)
        p_goal_cost = abs(v_g[0]-pos[0])+abs(v_g[1]-pos[1])
        cost = end[0] + p_goal_cost
        return cost

    def check_path_across(self,left_part,right_part):
        right_part.reverse()
        v_g= [self.goal_pos.x,self.goal_pos.y]
        cost_left = self.cost_part(left_part,v_g)
        cost_right = self.cost_part(right_part,v_g)
        direction = cmp(cost_left,cost_right)

        if direction == -1:# left
            end_ = Pose2D()
            end_.x,end_.y= self.pol2cart(self.front_obstacle[4])
            end = Pose2D()
            end.x, end.y = self.pol2cart(left_part[0])
            turnpoint = self.turnpoint_g(end,end_)
            self.path.poses.append(turnpoint)

        elif direction == 1:# right
            end_ = Pose2D()
            end_.x, end_.y = self.pol2cart(self.front_obstacle[-4])
            end = Pose2D()
            end.x, end.y=  self.pol2cart(right_part[0])
            turnpoint = self.turnpoint_g(end,end_)
            self.path.poses.append(turnpoint)
        return

    def turnpoint_g(self,end=Pose2D(),end_= Pose2D()):
        s = 0.6
        ang = numpy.arctan2(end.y-end_.y, end.x-end_.x)
        x = end.x + s * numpy.cos(ang)
        y = end.y + s * numpy.sin(ang)
        theta = self.o_g_theta
        turnpoint = Pose()
        turnpoint.position.x = x
        turnpoint.position.y = y
        turnpoint.orientation.z = numpy.sin( theta/2.0)
        turnpoint.orientation.w = numpy.cos( theta/2.0)
        # todo fix
        return turnpoint

    def arr_to_pose2D(self, arr, ang):
        pose = Pose2D()
        pose.x = arr[0] + self.odom_pos.x
        pose.y = arr[1] + self.odom_pos.y
        pose.theta = ang
        return pose

    def sum_dis(self, list_O):
        if len(list_O) > 2:
            i_sum = sum(list_O[1][1]) + list_O[-1][1][1]
        else:
            i_sum = sum(list_O[1][1])
        return i_sum

    def pol2cart(self, i):
        dis = i[0]
        ang =self.bean_to_angle(i[1])
        x = dis*numpy.cos(ang) + self.odom_pos.x
        y = dis*numpy.sin(ang) + self.odom_pos.y
        return [x, y]

    def angle_to_bean(self, ang):
        if ang < 0.0:
            bean = - round(ang*180.0/numpy.pi)
        else:
            bean = 359 - round(ang*180.0/numpy.pi)
        return bean

    def bean_to_angle(self,bean):
            angle = (360. - bean)/180.0 * numpy.pi
            return angle


    def point_resample(self, item,link_id):
        # change a more sufficient way to resample depending on the subjective distance
        item_position = item[1]
        n_i = len(item_position)
        front_dis = self.point_set[link_id]
        ang_step = max(1,round(45/(numpy.pi*front_dis)))
        #add the radius in the fomular
        my_slice =  slice( 0, n_i, min(n_i / 4,int(ang_step)) )
        if n_i > 4:
            i_resample = item_position[my_slice]
        else:
            i_resample = [item_position[0], item_position[-1]]
        return i_resample

    def break2(self,link_id, front):
        p_list = [j[1] for j in front]
        p_list.append(link_id+0.1)
        p_list.sort()
        index_link = p_list.index(link_id+0.1)
        left = front[0:index_link]
        right = front[index_link:]
        return left,right

    def point_distinguish(self, points,freeid,ifcontinue):
        link_a, odom_a, = self.link_angle()
        link_id = int(self.angle_to_bean(link_a))
        if link_id in freeid:
            self.is_free_to_go = True
            return
        mean_dis = self.min_range
        left =[]
        right = []
        if self.point_set[link_id] < self.o_g_distance:
            self.is_free_to_go = False
            for i in points.items():
                p_list =  [j[1] for j in i[1]]
                if link_id in p_list:
                    relist = self.point_resample(i, link_id)
                    self.front_obstacle = relist
                    left, right = self.break2(link_id,relist)
                elif link_id+ifcontinue*360 in p_list:
                    relist = self.point_resample(i, link_id)
                    self.front_obstacle = relist
                    left, right = self.break2(link_id+360, relist)
        if not self.front_obstacle:
            self.is_free_to_go = True
        elif self.point_set[link_id] > self.o_g_distance:
            self.is_free_to_go = True
        else:
            self.is_free_to_go = False
            self.check_path_across(left,right)

    def check_continue_fcn(self,alist,blist):
        if alist[0][1] < 2 and blist[-1][1]> 357:
            for i in alist:
                i[1] +=360
            blist.extend(alist)
            return True,blist
        else:
            return False,None

    def detect_block(self):
        cutscan = Cut(self.point_set)
        result = cutscan.main()
        lastobs_i = result[cutscan.group-1]
        firstobs_1 =result[0]

        ifcontinue, front = self.check_continue_fcn(firstobs_1,lastobs_i)
        if front:
            result[cutscan.group - 1] = front
            result.pop(0)
            # front continue
        freeid = cutscan.free
        self.point_distinguish(result,freeid,ifcontinue)

        # todo : fix the Bezier track prob 3_22
        if self.is_free_to_go:
            B = Bezier6D_v1_1.Bezier(self.odom, self.goal)
            #B.d_last = self.o_g_distance/5
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
                B.pubstate(i)
                i += 1
                rate.sleep()

        else:
            B = Bezier6D_v1_1.Bezier(self.odom, self.path.poses[0])
            B.main()
            state_ = B.state
            rate = rospy.Rate(10)
            cmd_ = []
            for i in state_:
                cmd = Twist()
                cmd.linear.x = i.v
                cmd.angular.z = i.w
                cmd_.append(cmd)
                #self.pub_cmd.publish(cmd)
                #rate.sleep()
            B.pubpath()
            B.pubControl()
            for i in range(len(state_)):
                B.pubstate(i)
                i += 1
                rate.sleep()
            self.path.poses.pop(0)
            return

    def link_angle(self):
        self.goal_pos = self.goal.position
        self.odom_pos = self.odom.pose.pose.position
        self.o_g_theta = numpy.arctan2(self.goal_pos.y - self.odom_pos.y, self.goal_pos.x - self.odom_pos.x)

        self.odom_pos_2d = Pose2D()
        self.odom_pos_2d.x = self.odom_pos.x
        self.odom_pos_2d.y = self.odom_pos.y
        self.odom_pos_2d.theta = self.orientation_to_theta(self.odom.pose.pose.orientation)

        self.goal_yaw = self.orientation_to_theta(self.goal.orientation)
        self.o_g_distance = numpy.linalg.norm(numpy.array([self.goal_pos.x, self.goal_pos.y]) - numpy.array([self.odom_pos.x , self.odom_pos.y]))

        return [self.o_g_theta, self.odom_pos_2d.theta]

    def orientation_to_theta(self, orientation):
        ang = transformations.euler_from_quaternion((orientation.x,
                                                     orientation.y,
                                                     orientation.z,
                                                     orientation.w))
        return ang[2]

def main():
    rospy.init_node('mission_control')
    while not rospy.is_shutdown():
        try:
            control = Subgoal_scan()
            pass
        except rospy.ROSInterruptException:
            print 'Ros Interruption'

class Cut():
    def __init__(self, scan):
        self.free = []
        self.obs_index = []
        self.scan = scan
        count = 0
        for i in self.scan:
            if math.isinf(i):
                self.free.append(count)
            else:
                self.obs_index.append(count)
            count += 1

    def cut(self, indexlist):
        cutpoint = numpy.diff(indexlist)
        cutlist = cutpoint.tolist()
        count = 0
        group = 0
        results = {}
        temp = []
        for i in range(len(indexlist) - 1):
            temp.append(indexlist[count])
            count += 1
            if not cutlist[i] == 1:
                results[group] = temp
                temp = []
                group += 1
            results[group] = temp
        return results

    def obscut_fcn(self, obs_cut={}):
        tolerrence = 0.5
        results = {}
        self.group = 0
        for item in obs_cut:

            scan_cut = self.scan[obs_cut[item][0]:obs_cut[item][-1] + 1]
            item_diff = numpy.diff(scan_cut)
            item_diff_list = item_diff.tolist()
            count = 0
            temp = [[scan_cut[0], obs_cut[item][0]]]
            for i in range(len(scan_cut) - 1):
                if abs(item_diff_list[i]) > tolerrence:
                    results[self.group] = temp
                    temp = []
                    self.group += 1
                count += 1
                temp.append([scan_cut[count], obs_cut[item][count]])
            results[self.group] = temp
            self.group += 1
        return results

    def main(self):
        obs_cut = self.cut(self.obs_index)
        self.freecut = self.cut(self.free)
        res = self.obscut_fcn(obs_cut)
        return res


if __name__ == '__main__':
    main()