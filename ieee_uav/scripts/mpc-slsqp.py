#!/usr/bin/env python

import numpy as np
from scipy.optimize import minimize
from math import pow, sqrt, cos, sin, atan2, exp

import rospy
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from gazebo_msgs.msg import ModelStates
from yolo_ros_simple.msg import bboxes
from ieee_uav.msg import odom_array

import time
import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

def val_saturation(val, thres):
    if val>thres:
        val=thres
    if val<-thres:
        val=-thres
    return val

def rpy_saturation(angle):
    if angle>np.pi:
        angle=angle-2*np.pi
    if angle<-np.pi:
        angle=angle+2*np.pi
    return angle

def sign(val):
    if val>=0:
        return 1
    elif val<0:
        return -1


class mpc_ctrl():
    def __init__(self):

        ### ROS Things
        rospy.init_node('mpc_controlelr', anonymous=True)
        self.drone_name = rospy.get_param("/drone_name", "iris")
        self.altitude_fixed = rospy.get_param("/altitude_fixed", 0.3)
        self.pose_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.pose_cb)
        self.target_pose_sub = rospy.Subscriber('/goal_pose', odom_array, self.target_pose_cb)
        self.bbox_sub = rospy.Subscriber('/bboxes', bboxes, self.bbox_cb)
        self.control_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=2)
        
        self.rate = rospy.Rate(15)

        self.pose_in = False
        self.goal_in = False
        self.bbox_in = False

        ### MPC setup
        self.horizon = 10
        self.dt = 0.07
        self.num_inputs = 3 # linear_x, linear_y, linear_z

        self.vx_min = -3.5
        self.vx_max = 3.5
        self.vy_min = -3.5
        self.vy_max = 3.5
        self.vz_min = -0.7
        self.vz_max = 0.7
        self.bounds = []
        for i in range(self.horizon):
            self.bounds += [[self.vx_min, self.vx_max]]
            self.bounds += [[self.vy_min, self.vy_max]]
            self.bounds += [[self.vz_min, self.vz_max]]
        self.bounds = np.array(self.bounds)

        self.u = np.zeros(self.horizon*self.num_inputs)

        self.position_weight = 3.0
        self.velocity_weight = 3.0
        self.input_weight = 1.5
        self.input_smoothness_weight = 2.0

    def pose_cb(self, msg):
        for i in range(len(msg.name)):
            if msg.name[i]==self.drone_name:
                curr_x = msg.pose[i].position.x
                curr_y = msg.pose[i].position.y
                curr_z = msg.pose[i].position.z
                curr_vx = msg.twist[i].linear.x
                curr_vy = msg.twist[i].linear.y
                curr_vz = msg.twist[i].linear.z
                _, _, self.curr_yaw = euler_from_quaternion([msg.pose[i].orientation.x, msg.pose[i].orientation.y, msg.pose[i].orientation.z, msg.pose[i].orientation.w])
                self.current_state = np.array([curr_x, curr_y, curr_z, curr_vx, curr_vy, curr_vz])
                self.pose_in = True
        return

    def target_pose_cb(self, msg):
        ref_list = []
        for i in range(len(msg.array)):
            ref_x = msg.array[i].pose.pose.position.x
            ref_y = msg.array[i].pose.pose.position.y
            ref_z = 0.9
            ref_vx = msg.array[i].twist.twist.linear.x
            ref_vy = msg.array[i].twist.twist.linear.y
            ref_vz = msg.array[i].twist.twist.linear.z
            ref_list.append([ref_x, ref_y, ref_z, ref_vx, ref_vy, ref_vz])
        self.goal_ref = np.array(ref_list)
        self.goal_in = True
        return


    def cost_function(self, u, *args):
        curr_state = args[0]
        ref = args[1][0] #seg
        cost = 0.0
        for i in range(self.horizon):
            prev_state = curr_state
            curr_state = self.plant(prev_state, self.dt, u[i*self.num_inputs], u[i*self.num_inputs+1], u[i*self.num_inputs+2])

            distance = pow(curr_state[0]-ref[0], 2) + pow(curr_state[1]-ref[1], 2)
            if distance > 5:
                # tracking cost
                cost += self.position_weight *1.5 * (distance-2.5)
                # velocity tracking cost
                cost += self.velocity_weight * ( pow(curr_state[3]-ref[3], 2) + pow(curr_state[4]-ref[4], 2) + pow(curr_state[5]-ref[5], 2) )
            elif distance > 2.5:
                # tracking cost
                cost += self.position_weight *0.7 * (distance-3.0)
                # velocity tracking cost
                cost += self.velocity_weight *1.5 * ( pow(curr_state[3]-ref[3], 2) + pow(curr_state[4]-ref[4], 2) + pow(curr_state[5]-ref[5], 2) )
            else:
                # velocity tracking cost
                cost += self.velocity_weight * ( pow(curr_state[3]-ref[3], 2) + pow(curr_state[4]-ref[4], 2) + pow(curr_state[5]-ref[5], 2) )

            cost += self.position_weight * pow(curr_state[2]-ref[2], 2)

            #input cost
            cost += self.input_weight * pow(u[i*self.num_inputs], 2)
            cost += self.input_weight * pow(u[i*self.num_inputs+1], 2)
            cost += self.input_weight * pow(u[i*self.num_inputs+2], 2)

            #input smoothness
            if i>0:
                cost += self.input_smoothness_weight * pow(u[i*self.num_inputs]-u[(i-1)*self.num_inputs], 2)
                cost += self.input_smoothness_weight * pow(u[i*self.num_inputs+1]-u[(i-1)*self.num_inputs+1], 2)
                cost += self.input_smoothness_weight * pow(u[i*self.num_inputs+2]-u[(i-1)*self.num_inputs+2], 2)  
        return cost


    def plant(self, prev_state, dt, vx, vy, vz):
        x_t = prev_state[0]
        y_t = prev_state[1]
        z_t = prev_state[2]
        vx_t = prev_state[3]
        vy_t = prev_state[4]
        vz_t = prev_state[5]

        x_t_1 = x_t + vx_t*dt
        y_t_1 = y_t + vy_t*dt
        z_t_1 = z_t + vz_t*dt
        vx_t_1 = vx
        vy_t_1 = vy
        vz_t_1 = vz
        return [x_t_1, y_t_1, z_t_1, vx_t_1, vy_t_1, vz_t_1]

    def bbox_cb(self, msg):
        if len(msg.bboxes)>0:
            box=msg.bboxes[0]
            self.ibvs_yaw = (320-(box.x+box.width)/2) / 640*1.5708
            self.bbox_in=True


#######################################################################
if __name__ == '__main__':
    mpc_ = mpc_ctrl()
    time.sleep(0.3)

    while 1:
        try:
            if mpc_.pose_in and mpc_.goal_in:

                ### mpc start
                tic = time.time()

                for i in range(mpc_.num_inputs):
                    mpc_.u = np.delete(mpc_.u, 0) #remove first input
                    mpc_.u = np.append(mpc_.u, mpc_.u[-mpc_.num_inputs]) #copy last input

                u_solution = minimize(mpc_.cost_function, mpc_.u, (mpc_.current_state, mpc_.goal_ref),
                                      method='SLSQP', bounds=mpc_.bounds, tol=1e-4, options = {'disp': False}) #disp - debugging
                mpc_.u = u_solution.x
                # print(u_solution.x) #solution is stored in "x"
                # print(u_solution.success)
                # print(u_solution.message)

                solved_input = TwistStamped()
                solved_input.header.stamp = rospy.Time.now()
                solved_input.twist.linear.x = u_solution.x[0]
                solved_input.twist.linear.y = u_solution.x[1]
                solved_input.twist.linear.z = u_solution.x[2]

                target_forward_yaw = atan2(mpc_.goal_ref[-1][1]-mpc_.goal_ref[0][1], mpc_.goal_ref[-1][0]-mpc_.goal_ref[0][0])
                forward_facing_yaw = atan2(mpc_.goal_ref[-1][1]-mpc_.current_state[1], mpc_.goal_ref[-1][0]-mpc_.current_state[0])
                yaw_Err=rpy_saturation(rpy_saturation(target_forward_yaw - mpc_.curr_yaw))
                if mpc_.bbox_in:
                    solved_input.twist.angular.z = sign(yaw_Err) * val_saturation(exp(abs(yaw_Err)), 1.0)
                    # solved_input.twist.angular.z = val_saturation(yaw_Err, 1.0)
                    mpc_.bbox_in=False
                # else:
                    # solved_input.twist.angular.z = val_saturation(forward_facing_yaw, 3.0)
                # forward_yaw = atan2(mpc_.goal_ref[1]-mpc_.current_state[1], mpc_.goal_ref[0]-mpc_.current_state[0])
                # if mpc_.bbox_in:
                #     solved_input.twist.angular.z = rpy_saturation(rpy_saturation(forward_yaw - mpc_.curr_yaw) + mpc_.ibvs_yaw*2)
                #     mpc_.bbox_in=False
                # else:
                #     solved_input.twist.angular.z = val_saturation(forward_yaw, 3.0)
                mpc_.control_pub.publish(solved_input)

                toc = time.time()
                # print('MPC time spent: %.5f, solved: %s' %(toc - tic, u_solution.success))

            mpc_.rate.sleep()
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)