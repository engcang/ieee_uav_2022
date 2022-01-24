#!/usr/bin/env python

import numpy as np
from scipy.optimize import minimize
from math import pow, sqrt, cos, sin

import rospy
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import Bool
from yolo_ros_simple.msg import bboxes

import time
import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


def rpy_saturation(angle):
    if angle>np.pi:
        angle=angle-2*np.pi
    if angle<-np.pi:
        angle=angle+2*np.pi
    return angle


class mpc_ctrl():
    def __init__(self):

        ### ROS Things
        rospy.init_node('mpc_controlelr', anonymous=True)
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb)
        self.traj_sub = rospy.Subscriber('/best_path', Path, self.traj_cb)
        self.bbox_sub = rospy.Subscriber('/bboxes', bboxes, self.bbox_cb)
        self.control_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=2)
        self.rate = rospy.Rate(15)
        
        self.pose_in = False
        self.traj_in = False
        self.bbox_in = False

        ### MPC setup
        self.horizon = 10
        self.dt = 0.07
        self.num_inputs = 4 # linear_x, linear_y, linear_z, angular_z

        self.vx_min = -1.0
        self.vx_max = 1.0
        self.vy_min = -1.0
        self.vy_max = 1.0
        self.vz_min = -0.2
        self.vz_max = 0.2
        self.w_min = -1.2
        self.w_max = 1.2
        self.bounds = []
        for i in range(self.horizon):
            self.bounds += [[self.vx_min, self.vx_max]]
            self.bounds += [[self.vy_min, self.vy_max]]
            self.bounds += [[self.vz_min, self.vz_max]]
            self.bounds += [[self.w_min, self.w_max]]
        self.bounds = np.array(self.bounds)

        self.u = np.zeros(self.horizon*self.num_inputs)

        self.position_weight = 3.0
        self.yaw_weight = 1.0
        self.input_weight = 0.5
        self.input_smoothness_weight = 2.0

    def pose_cb(self, msg):
        curr_x = msg.pose.position.x
        curr_y = msg.pose.position.y
        curr_z = msg.pose.position.z
        _, _, curr_yaw = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        self.current_state = np.array([curr_x, curr_y, curr_z, rpy_saturation(curr_yaw)])
        self.pose_in = True
        return

    def traj_cb(self, msg):
        if len(msg.poses) > 1 :
            last_idx=len(msg.poses)-1

            ref_x = msg.poses[last_idx].pose.position.x
            ref_y = msg.poses[last_idx].pose.position.y
            ref_z = msg.poses[last_idx].pose.position.z
            _, _, ref_yaw = euler_from_quaternion([msg.poses[last_idx].pose.orientation.x, msg.poses[last_idx].pose.orientation.y, msg.poses[last_idx].pose.orientation.z, msg.poses[last_idx].pose.orientation.w])
            self.traj_ref = np.array([ref_x, ref_y, ref_z, rpy_saturation(ref_yaw)])
            self.traj_in = True
            return


    def cost_function(self,u, *args):
        curr_state = args[0]
        ref = args[1]
        cost = 0.0
        for i in range(self.horizon):
            prev_state = curr_state
            curr_state = self.plant(prev_state, self.dt, u[i*self.num_inputs], u[i*self.num_inputs+1], u[i*self.num_inputs+2], u[i*self.num_inputs+3])

            #tracking cost
            distance = pow(curr_state[0]-ref[0], 2) + pow(curr_state[1]-ref[1], 2) + pow(curr_state[2]-ref[2], 2)
            if distance > 5:
                cost += self.position_weight * (distance-5)
            elif distance <= 5:
                cost += self.position_weight * (5-distance)
            # cost += self.position_weight * distance
            cost += self.yaw_weight * pow(rpy_saturation(curr_state[3]-ref[3]), 2)

            #input cost
            cost += self.input_weight * pow(u[i*self.num_inputs], 2)
            cost += self.input_weight * pow(u[i*self.num_inputs+1], 2)
            cost += self.input_weight * pow(u[i*self.num_inputs+2], 2)
            cost += self.input_weight * pow(u[i*self.num_inputs+3], 2)

            #input smoothness
            if i>0:
                cost += self.input_smoothness_weight * pow(u[i*self.num_inputs]-u[(i-1)*self.num_inputs], 2)
                cost += self.input_smoothness_weight * pow(u[i*self.num_inputs+1]-u[(i-1)*self.num_inputs+1], 2)
                cost += self.input_smoothness_weight * pow(u[i*self.num_inputs+2]-u[(i-1)*self.num_inputs+2], 2)  
                cost += self.input_smoothness_weight * pow(u[i*self.num_inputs+3]-u[(i-1)*self.num_inputs+3], 2)  
        return cost


    def plant(self, prev_state, dt, vx, vy, vz, w):
        x_t = prev_state[0]
        y_t = prev_state[1]
        z_t = prev_state[2]
        yaw_t = prev_state[3]

        x_t_1 = x_t + vx*dt
        y_t_1 = y_t + vy*dt
        z_t_1 = z_t + vz*dt
        yaw_t_1 = rpy_saturation(yaw_t + w*dt)
        return [x_t_1, y_t_1, z_t_1, yaw_t_1]

    def bbox_cb(self, msg):
        if len(msg.bboxes)>0:
            box=msg.bboxes[0]
            self.ibvs_yaw = (320-(box.x+box.width)/2) / 640*1.02974
            self.bbox_in=True


#######################################################################
if __name__ == '__main__':
    mpc_ = mpc_ctrl()
    time.sleep(0.5)

    while 1:
        try:
            if mpc_.pose_in and mpc_.traj_in:

                ### mpc start
                tic = time.time()

                for i in range(mpc_.num_inputs):
                    mpc_.u = np.delete(mpc_.u, 0) #remove first input
                    mpc_.u = np.append(mpc_.u, mpc_.u[-mpc_.num_inputs]) #copy last input

                u_solution = minimize(mpc_.cost_function, mpc_.u, (mpc_.current_state, mpc_.traj_ref),
                                      method='SLSQP', bounds=mpc_.bounds, tol=1e-4, options = {'disp': False}) #disp - debugging
                mpc_.u = u_solution.x
                # print(u_solution.x) #solution is stored in "x"
                # print(u_solution.success)
                # print(u_solution.message)

                solved_input = TwistStamped()
                solved_input.header.stamp = rospy.Time.now()
                # solved_input.twist.linear.x = cos(curr_yaw) * u_solution.x[0] + sin(curr_yaw) * u_solution.x[1]
                # solved_input.twist.linear.y = -sin(curr_yaw) * u_solution.x[0] + cos(curr_yaw) * u_solution.x[1]
                solved_input.twist.linear.x = u_solution.x[0]
                solved_input.twist.linear.y = u_solution.x[1]
                solved_input.twist.linear.z = u_solution.x[2]
                if mpc_.bbox_in:
                    solved_input.twist.angular.z = rpy_saturation(u_solution.x[3] + mpc_.ibvs_yaw*3)
                    mpc_.bbox_in=False
                else:
                    solved_input.twist.angular.z = u_solution.x[3]
                mpc_.control_pub.publish(solved_input)

                toc = time.time()
                print('MPC time spent: %.5f, solved: %s' %(toc - tic, u_solution.success))

            mpc_.rate.sleep()
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)