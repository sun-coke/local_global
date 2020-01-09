# !/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist, Pose2D, PoseStamped,Pose
from nav_msgs.msg import Path, Odometry

class Pn():
    def __init__(self, x, y):
        self.x = x
        self.y = y

class Robot_state():
    def __init__(self,v=0,w = 0,theta = 0):
        self.v = v
        self.w = w
        self.theta = theta


class Bezier():
    def __init__(self,odom,goal):
        self.numSteps = 100
        self.ctrlpx = []
        self.ctrlpy = []
        self.state = []
        self.odom = odom
        self.goal = goal
        self.odom2D = self.Pose_Pose2D(self.odom.pose.pose)
        self.goalv = 0.05
        self.goalw = 0.0
        self.d_last = 0.2
        self.ctrlp = Path()
        self.pathpub = rospy.Publisher('path', Path, queue_size=1)
        #self.pubtrack_r = rospy.Publisher('track_ref',PoseStamped,queue_size=1)
        self.cmd_v = rospy.Publisher("cmd_vel_ref", Twist, queue_size=1)
        self.point(self.odom.pose.pose,self.goal)
        self.ctrlpub = rospy.Publisher('ctrlpoint',Path,queue_size=1)
        self.pub_state = rospy.Publisher('state_ref', PoseStamped, queue_size=1)

    def B(self,coorArr, i, j, t):
        if j == 0:
            return coorArr[i]
        return self.B(coorArr, i, j - 1, t) * (1 - t) + self.B(coorArr, i + 1, j - 1, t) * t

    def Pose_Pose2D(self,o = Pose()):
        pose = Pose2D()
        pose.x = o.position.x
        pose.y = o.position.y
        pose.theta = math.atan2(2*(o.orientation.z * o.orientation.w+ o.orientation.x*o.orientation.y ), 1.0- 2*(o.orientation.y*o.orientation.y+o.orientation.z*o.orientation.z))
        return pose

    def point(self, odom=Pose(), goal=Pose()):
        global n
        pose1 = self.Pose_Pose2D(odom)
        pose2 = self.Pose_Pose2D(goal)
        n = 6
        w = self.odom.twist.twist.angular.z
        v = self.odom.twist.twist.linear.x
        if v<0.05:
            k = 0.1
        else:
            k= w/v
        vg = self.goalv
        wg = self.goalw
        if not vg:
            kg = 0.1
        else:
            kg = wg / vg
        P = range(n)
     # mid of the distance
        d1 = max(0.1,v*2.0) ## define by the initial velocity
        d2 = v*3.0
        d5 = vg*2.0 # define by the demanding velocity of the subgoal
        d4 = max(self.d_last , vg*3.0)
        coso= math.cos(pose1.theta)
        sino= math.sin(pose1.theta)
        P[5] = Pn((pose2.x-pose1.x)*coso+(pose2.y- pose1.y)*sino  , -(pose2.x-pose1.x)*sino + (pose2.y- pose1.y)*coso )
        psi = pose2.theta - pose1.theta  ## the goal yaw rate
        cosp = math.cos(psi)
        sinp = math.sin(psi)
        P[0] = Pn(0, 0)
        P[1] = Pn(d1, 0)
        P[4] = Pn(P[5].x - d5 * math.cos(psi), P[5].y - d5 * math.sin(psi))
        P[2] = Pn(d1+d2, 5.0 * k * d1 * d1 / 4.0)
        P[3] = Pn(P[4].x + 5.0 * kg * d5 * d5 * sinp / 4.0 - d4 * cosp,
                  P[4].y - 5.0 * kg * d5 * d5 * cosp / 4.0 - d4 * sinp)

        self.ctrlp.header.frame_id = "odom"
        for k in range(n):
            self.ctrlpx.append(P[k].x)
            self.ctrlpy.append(P[k].y)
            pose = PoseStamped()
            pose.header.frame_id = "odom"
            pose.pose.position.x = P[k].x*coso - sino*P[k].y + pose1.x # offset to current pos initial
            pose.pose.position.y = P[k].y*coso + sino*P[k].x +pose1.y
            self.ctrlp.poses.append(pose)

    def xy_U(self,x, y):
        n = len(x)
        dx = []
        dy = []
        ddx = []
        ddy = []

        state_line = []
        v_line = []
        w_line = []
        h = 0.1  # the meaning of step h ralation to time  h = 0.1s = '10Hz publish rate '
        for i in range(n - 2):
            state = Robot_state()
            dx.append((x[i + 2] - x[i]) / (2 * h))
            dy.append((y[i + 2] - y[i]) / (2 * h))
            ddx.append((x[i + 2] - 2 * x[i + 1] + x[i]) / (h * h))
            ddy.append((y[i + 2] - 2 * y[i + 1] + y[i]) / (h * h))
            state.v = math.sqrt(math.pow(dx[i], 2) + math.pow(dy[i], 2))
            state.w = (ddy[i] * dx[i] - ddx[i] * dy[i]) / (math.pow(dx[i], 2) + math.pow(dy[i], 2))
            state.theta = math.atan2(dy[i], dx[i])
            state_line.append(state)
            v_line.append(state.v)
            w_line.append(state.w)
            i += 1
        return state_line, v_line


    def track_out(self):
        dotx = []
        doty = []
        for k in range(self.numSteps):
            t = float(k) / (self.numSteps - 1)
            x_f = self.B(self.ctrlpx, 0, n - 1, t)
            y_f = self.B(self.ctrlpy, 0, n - 1, t)
            try:
                dotx.append(x_f)
                doty.append(y_f)
            except:
                pass
        return dotx, doty


    def main(self):
        v_lim = 0.2  # the max vel of robot
        self.numSteps = int((abs(self.odom.pose.pose.position.x- self.goal.position.x) + abs(self.odom.pose.pose.position.y- self.goal.position.y)) * 50)
        dotx, doty = self.track_out()
        state_, v_line = self.xy_U(dotx, doty)
        for i in range(1, 30):
            if max(v_line) > v_lim:
                self.numSteps += 1
                dotx, doty = self.track_out()
                state_, v_line = self.xy_U(dotx, doty)
            elif max(v_line) < (v_lim - 0.03):
                self.numSteps -= 1
                dotx, doty = self.track_out()
                state_, v_line = self.xy_U(dotx, doty)
            else:
                break
        v_acc = []
        for i in range(self.numSteps - 3):
            acc = (v_line[i + 1] - v_line[i]) / 0.1
            v_acc.append(acc)
        v_acc.insert(0, v_acc[0])
        v_acc.insert(-1, v_acc[-1])
        w_line = [i.w for i in state_]
        sino = math.sin(self.odom2D.theta)
        coso = math.cos(self.odom2D.theta)

        self.path = Path()
        self.path.header.frame_id = "odom"
        log = open('track.txt','w')
        log.write(str(v_line))
        log.write(str(w_line))
        for i in range(self.numSteps):
            pose = PoseStamped()
            pose.header.frame_id = "odom"
            pose.pose.position.x = dotx[i]*coso - doty[i]*sino + self.odom2D.x # offset to current pos initial
            pose.pose.position.y = doty[i]*coso + dotx[i]*sino + self.odom2D.y
            self.path.poses.append(pose)
            log.write(str([pose.pose.position.x,pose.pose.position.y]))
        #self.path.poses.insert(0,self.odom.pose)
        #self.path.poses.append(self.goal)
        log.close()
        state_.append(Robot_state(0,0,0))
        self.state = state_

    def pubpath(self):
        self.pathpub.publish(self.path)
    def pubControl(self):
        self.ctrlpub.publish(self.ctrlp)
    def pubstate(self,i):
        pose = PoseStamped()
        pose.pose.position.x = self.state[i].v
        pose.pose.position.y = self.state[i].w
        pose.pose.position.z = self.state[i].theta
        pose.pose.orientation.x= self.path.poses[i].pose.position.x
        pose.pose.orientation.y = self.path.poses[i].pose.position.y
        self.pub_state.publish(pose)

## turn into Class style fuction