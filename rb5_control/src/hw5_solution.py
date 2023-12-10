#!/usr/bin/env python
import sys
import roslib
import rospy
import geometry_msgs.msg
import math
import tf
import tf2_ros
import time
import numpy as np
from math import cos, sin, pi, pow
from geometry_msgs.msg import Twist
from tf.transformations import quaternion_matrix, euler_from_quaternion
from utils import get_H, calculate_Kalman_gain_coeff_K, utilize_Kalman_gain_coeff_K, \
    calculate_observation_residuals, update_state_covariance

# Global
step = 0
step_1 = 0
step_2 = 0
MAX_I = 4
I = 0
Obstacle_DIS = 0.15
waypoint = np.array([0.0, 0.0, 0.0])

Q_m = np.diag([0.05 ** 2, 0.05 ** 2, 0.15 ** 2])
R_m = np.diag([pow(0.05, 2), pow(0.05, 2)])
State_cov = np.diag([1, 1, 1]) 
Cov_init = 1

class PIDcontroller:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.target = None
        self.I = 0
        self.lastError = 0
        self.timestep = 0.1
        # self.timestep = 1

        self.maximumValue = 1
        self.angular_tolerance = 0.9
        self.last_action_type = "move"
        self.update_value = np.array([0.0,0.0,0.0])

        #TODO: which match the real velocity
        self.v_straight = 0.085
        self.v_rotate = 0.85

    def setTarget(self, state):
        self.I = 0
        self.lastError = 0
        self.target = np.array(state)

    def getError(self, currentState, targetState):
        result = targetState - currentState
        result[2] = (result[2] + np.pi) % (2 * np.pi) - np.pi
        return result

    def update_x(self, e):
        P = self.Kp * e

        self.I = self.I + self.Ki * e * self.timestep 
        I = self.I
        D = self.Kd * (e - self.lastError)
        result = P + I + D

        self.lastError = e

        if(result!=0):
            # scale down the twist if its norm is more than the maximum value. 
            flag = result / abs(result)
            # if(result > self.maximumValue):
            #     result = self.maximumValue * flag
            #     self.I = 0.0

            result = self.v_straight * flag
        return result
    
    def update_z(self, e):
        P = self.Kp * e

        self.I = self.I + self.Ki * e * self.timestep 
        I = self.I
        D = self.Kd * (e - self.lastError)
        result = 2 * (P + I + D)

        self.lastError = e

        if(result!=0):
            flag = result / abs(result)
            # scale down the twist if its norm is more than the maximum value. 
            # if(result > self.maximumValue):
            #     result = self.maximumValue * flag
            #     self.I = 0.0

            result = self.v_rotate * flag
        return result
    
    def determine_angle_details(self, current_state):
        # Calculate target angle
        z_target_axis = math.atan2(self.target[1] - current_state[1], self.target[0] - current_state[0])
        # Normalize the result to between -pi and pi
        while z_target_axis > math.pi:
            z_target_axis -= 2 * math.pi
        while z_target_axis < -math.pi:
            z_target_axis += 2 * math.pi
        # Calculate angle difference
        angle_difference_robot_target = z_target_axis - current_state[2]
        # Normalize the result to between -pi and pi
        while angle_difference_robot_target > math.pi:
            angle_difference_robot_target -= 2 * math.pi
        while angle_difference_robot_target < -math.pi:
            angle_difference_robot_target += 2 * math.pi
        # Append the details to the list
        angle_details = {"current_state": current_state, "target": self.target, "z_target_axis": z_target_axis, "angle_difference_robot_target": angle_difference_robot_target}
        return angle_details

    def calculate_distance(self, point1, point2):
        return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)

    def detailed_movement_information(self, angle_details):

        angle_diff = round(angle_details["angle_difference_robot_target"], 2)
        # Check if the rounded angle difference is close to any of the four angles for 'move' actions
        distance = self.calculate_distance((angle_details["current_state"][0], angle_details["current_state"][1]), (angle_details["target"][0], angle_details["target"][1]))
        #TODO:check whether can reduce 0.05
        if (abs(angle_diff - 0) <= self.angular_tolerance) and distance > 0.05:
            result = ['move', distance]
            return result
        elif (abs(angle_diff - math.pi) <= self.angular_tolerance or abs(angle_diff + math.pi) <= self.angular_tolerance) and distance > 0.05:
            result = ['move', -distance]
            return result
        # For 'rotate' actions
        #TODO:check whether can reduce 0.05
        elif distance > 0.05:
            if -math.pi/2 < angle_diff or angle_diff < math.pi/2:
                rotation_before_move = angle_diff
            elif angle_diff > 0:
                rotation_before_move = angle_diff - math.pi
            else:
                rotation_before_move = angle_diff + math.pi
            result = ['rotate', rotation_before_move]
            return result
        else:
            rotation_before_move = angle_details["target"][2] - angle_details["current_state"][2]
            result = ['rotate', rotation_before_move]
            return result

    def generate_control_command(self, detailed_info):

        action_type = detailed_info[0]

        # check whether the action type change
        if self.last_action_type != action_type:
            self.I = 0
            self.lastError = 0
            # print("===Reset===")

        # For 'move' actions
        if action_type == 'move':
            distance = detailed_info[1]
            # Calculate the speed
            v_x = self.update_x(distance)
            self.update_value = np.array([v_x, 0.0, 0.0])
            control_command = {"command": "move", "rb5_speed": self.update_value}
            self.last_action_type = 'move'

        # For 'rotate' actions
        elif action_type == 'rotate':
            rotation_before_move = detailed_info[1]
            # Calculate rotation times and move time
            v_z = self.update_z(rotation_before_move)
            self.update_value = np.array([0.0, 0.0, v_z])
            control_command = {"command": "rotate","rb5_speed": self.update_value}
            self.last_action_type = 'rotate'

        return control_command

def getMarkerPos(l):
    global step_1, step_2
    step_1 += 1
    print("step_1:",step_1)
    br = tf.TransformBroadcaster()
    result = None
    foundMarker = False

    # print("Call getCurrentPos")
    for i in range(0, 9):
        camera_name = "camera_" + str(i)
        marker_name = "marker_" + str(i)
        if l.frameExists(camera_name):
            step_2 += 1
            print("step_2:",step_2)
            print("Found {}".format(camera_name))
            try:
                now = rospy.Time()
                # wait for the transform ready from the map to the camera for 1 second.
                l.waitForTransform(camera_name, marker_name, now, rospy.Duration(0.1))
                # extract the transform camera pose in the map coordinate.
                (trans, rot) = l.lookupTransform(camera_name, marker_name, now)
                # convert the rotate matrix to theta angle in 2d
                # matrix = quaternion_matrix(rot)
                # angle = math.atan2(matrix[1][2], matrix[0][2])
                # euler = euler_from_quaternion(rot)

                # this is not required, I just used this for debug in RVIZ
                # br.sendTransform((trans[0], trans[1], 0), tf.transformations.quaternion_from_euler(0,0,angle), rospy.Time.now(), "base_link", "map")

                # result = np.array([trans[0], trans[1], angle])
                result = [(trans[2], -1*trans[0]), i]
                print("marker result: ", result)
                # print("***marker_id={}***\n, trans[0]={:.3f}, trans[1]={:.3f}, trans[2]={:.3f}, euler={}"
                #         .format(i, trans[0], trans[1], trans[2], euler))
                foundMarker = True
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf2_ros.TransformException):
                print("meet error")
    listener.clear()
    return foundMarker, result

def genTwistMsg(desired_twist):
    """
    Convert the twist to twist msg.
    """
    twist_msg = Twist()
    twist_msg.linear.x = desired_twist[0] 
    twist_msg.linear.y = desired_twist[1] 
    twist_msg.linear.z = 0
    twist_msg.angular.x = 0
    twist_msg.angular.y = 0
    twist_msg.angular.z = desired_twist[2]
    return twist_msg

def local_to_global_velocity(local_velocity, global_orientation):
    # Create a rotation matrix based on the global orientation
    J = np.array([[np.cos(global_orientation), -np.sin(global_orientation), 0.0],
                  [np.sin(global_orientation), np.cos(global_orientation), 0.0],
                  [0.0, 0.0, 1.0]])

    # Convert the local velocity to global velocity
    global_velocity = np.dot(J, local_velocity)
    return global_velocity

def calculate_global_marker_position(x_c, y_c, theta_c, x_obs, y_obs):
    x_m = x_c + x_obs * np.cos(theta_c) - y_obs * np.sin(theta_c)
    y_m = y_c + x_obs * np.sin(theta_c) + y_obs * np.cos(theta_c)
    return x_m, y_m

def expand_diag_matrix(original_matrix, n):
    diag_elements = np.diag(original_matrix)
    
    expanded_matrix = np.zeros((n, n))

    for i in range(min(len(diag_elements), n)):
        expanded_matrix[i, i] = diag_elements[i]
    
    return expanded_matrix

def expand_and_fill_diag_matrix(original_matrix, n, x):
    original_size = original_matrix.shape[0]
    
    expanded_matrix = np.zeros((n, n))

    for i in range(original_size):
        expanded_matrix[i, i] = original_matrix[i, i]

    for i in range(original_size, n):
        expanded_matrix[i, i] = x
    
    return expanded_matrix

def expand_X_update(original_array, new_length):

    if len(original_array) != 3:
        raise ValueError("Input must be a 1D array of length 3.")

    if new_length < 3:
        raise ValueError("New length must be at least 3.")

    # Create an array of zeros with the new length
    expanded_array = np.zeros(new_length)

    # Copy the original array into the expanded array
    expanded_array[:3] = original_array

    return expanded_array

def execute_control_step(pid, pub_twist, listener, X_k, State_cov, Q_m, R_m, marker_dic, Cov_init, step):
    #! pid calculate
    angle_details = pid.determine_angle_details(X_k[:3])
    # print(angle_details,'\n')
    action_details = pid.detailed_movement_information(angle_details)
    # print(action_details,'\n') 
    control_command = pid.generate_control_command(action_details)
    # print(control_command)

    #! Check Obstacle
    found_marker, marker_info = getMarkerPos(listener)
    while(Obstacle_DIS >= math.sqrt(marker_info[0][0]**2 + marker_info[0][1]**2)) :
        pub_twist.publish(genTwistMsg(np.array([0.0,0.0,0.0])))
        time.sleep(pid.timestep*10)

    #! publish the twist
    pub_twist.publish(genTwistMsg(pid.update_value))
    time.sleep(pid.timestep)

    #! Calculate X_k
    X_update = expand_X_update(local_to_global_velocity(pid.update_value, X_k[2]) * pid.timestep, len(X_k))
    X_k += X_update

    # Normalize the result to between -pi and pi
    if X_k[2] > math.pi:
        X_k[2] -= 2 * math.pi
    if X_k[2] < -math.pi:
        X_k[2] += 2 * math.pi

    #! Calculate State_cov
    State_cov = State_cov + expand_diag_matrix(Q_m, len(X_k))
    # print(State_cov)

    #! Check whether we observe a marker
    found_marker, marker_info = getMarkerPos(listener)
    if found_marker:
        marker_name = "marker_" + str(marker_info[1])
        #! If it is a new marker
        if marker_name not in marker_dic:
            #! Add it to the marker_dic
            marker_dic[marker_name] = len(marker_dic)
            print("marker_dic: ", marker_dic)
            #! Add it to X_k
            x_new, y_new = calculate_global_marker_position(X_k[0], X_k[1], X_k[2], marker_info[0][0], marker_info[0][1])
            X_k = np.append(X_k, x_new)
            X_k = np.append(X_k, y_new)
            # print(X_k)
            #! Expend State_cov
            State_cov = expand_and_fill_diag_matrix(State_cov, len(X_k), Cov_init)
            # print(State_cov)

    #! when observe a marker
    if found_marker:
        H_m = get_H(X_k[2])

        #! gain K
        Kalman_gain_K = calculate_Kalman_gain_coeff_K(State_cov, H_m, R_m, marker_name, marker_dic)
        # print("Kalman_gain_K: ", Kalman_gain_K)

        #! update X_k based on K
        X_k = utilize_Kalman_gain_coeff_K(State_cov, H_m, R_m, X_k, marker_info[0], marker_name, marker_dic)
        # print("X_k_new: ", X_k)

        #! update state cov
        State_cov = update_state_covariance(State_cov, H_m, Kalman_gain_K, marker_name, marker_dic)
        # print("Updated State_cov: ", State_cov)
    
    step += 1 
    # X_k = np.round_(X_k, 4)
    print("X_k:",step,X_k)

if __name__ == "__main__":

    print("===start init===\n")
    step = 0
    step_1 = -1
    step_2 = 0
    MAX_I = 4
    I = 0
    Obstacle_DIS = 0.15

    rospy.init_node("hw3")
    pub_twist = rospy.Publisher("/twist", Twist, queue_size=1)

    listener = tf.TransformListener()

    # init pid controller
    pid = PIDcontroller(0.02,0.005,0.005)

    marker_dic = {}

    #! init X_k
    X_k = np.array([0.0, 0.0, 0.0])

    print("X:",step, X_k)

    # ! active 
    found_marker, marker_info = getMarkerPos(listener)
    pub_twist.publish(genTwistMsg(pid.update_value))
    time.sleep(1)

    print("===start generate boundaries===\n")

    while(len(X_k) <= 25 and I <= MAX_I):
    #square
        I += 1
        waypoint = np.array([
                            #! one point
                            [0.4, 0.0, math.pi/2],
                            [0.4, 0.4, math.pi],
                            [0.0, 0.4, -math.pi/2]
                            [0.0, 0.0, 0.0],
                            ])         

        for wp in waypoint:
            # print("move to way point", wp)
            # ! set wp as the target point
            pid.setTarget(wp)

            X_k, State_cov, step = execute_control_step(pid, pub_twist, listener, X_k, State_cov, Q_m, R_m, marker_dic, Cov_init, step)

            while(np.linalg.norm(pid.getError(X_k[:3], wp)) > 0.11): # check the error between current state and current way point

                X_k, State_cov, step = execute_control_step(pid, pub_twist, listener, X_k, State_cov, Q_m, R_m, marker_dic, Cov_init, step)

    boundaries = X_k[-24:]

    print("===start generate waypoints===\n")

    print("===start sweep===\n")
    for wp in waypoint:
        # print("move to way point", wp)
        # ! set wp as the target point
        pid.setTarget(wp)

        X_k, State_cov, step = execute_control_step(pid, pub_twist, listener, X_k, State_cov, Q_m, R_m, marker_dic, Cov_init, step)

        while(np.linalg.norm(pid.getError(X_k[:3], wp)) > 0.11): # check the error between current state and current way point

            X_k, State_cov, step = execute_control_step(pid, pub_twist, listener, X_k, State_cov, Q_m, R_m, marker_dic, Cov_init, step)

    #! stop the car and exit
    pub_twist.publish(genTwistMsg(np.array([0.0,0.0,0.0])))
    print("===done===!\n")

    