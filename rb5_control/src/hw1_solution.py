#!/usr/bin/env python

import sys
import rospy
import time
import math
import numpy as np
from geometry_msgs.msg import Twist

# Global 
pid = None
pub_twist = None
current_waypoint_index = 0
waypoint = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]) 
current_state = np.array([0.0, 0.0, 0.0]) 
step_num = 0

class PIDcontroller:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.target = None
        self.I = 0
        self.lastError = 0
        self.timestep = 0.1

        self.maximumValue = 0.1
        self.angular_tolerance = 0.05
        self.last_action_type = "move"
        self.update_value = np.array([0.0,0.0,0.0])

    def setTarget(self, state):
        """
        set the target pose.
        """
        self.I = 0
        self.lastError = 0
        self.target = np.array(state)

    def update(self, e):
        P = self.Kp * e

        self.I = self.I + self.Ki * e * self.timestep 
        I = self.I
        D = self.Kd * (e - self.lastError)
        result = P + I + D

        self.lastError = e

        # scale down the twist if its norm is more than the maximum value. 
        if(result > self.maximumValue):
            result = self.maximumValue
            self.I = 0.0

        return result

    ###TODO: change points to point_1(state) and point_2(targrt)
    def determine_angle_details(self, current_state):
        """
        Determines the target angle and the angle difference for the robot for each pair of points.
        """
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
        """
        Calculates the Euclidean distance between two 2D points.
        """
        return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)

    ###TODO: change to only one action
    def detailed_movement_information(self, angle_details):

        angle_diff = round(angle_details["angle_difference_robot_target"], 2)
        # Check if the rounded angle difference is close to any of the four angles for 'move' actions
        ###TODO: change to only forward and backward, need to focus on tolerance
        ###TODO: no more need 1.57 and -1.57
        if abs(angle_diff - 0) <= self.angular_tolerance:
            distance = self.calculate_distance((angle_details["current_state"][0], angle_details["current_state"][1]), (angle_details["target"][0], angle_details["target"][1]))
            result = ['move', distance]
            return result
        elif abs(angle_diff - math.pi) <= self.angular_tolerance or abs(angle_diff + math.pi) <= self.angular_tolerance:
            distance = -self.calculate_distance((angle_details["current_state"][0], angle_details["current_state"][1]), (angle_details["target"][0], angle_details["target"][1]))
            ###TODO: distance used to input into pid control, no more need rotation_after_move
            result = ['move', distance]
            return result
        # For 'rotate' actions
        else:
            ###TODO: notice!!!
            if -math.pi/2 < angle_diff or angle_diff < math.pi/2:
                rotation_before_move = angle_diff
            elif angle_diff > 0:
                rotation_before_move = angle_diff - math.pi
            else:
                rotation_before_move = angle_diff + math.pi
            ###TODO: distance used to input into pid control, no more need rotation_after_move and distance
            result = ['rotate', rotation_before_move]
            return result

    ###TODO: need to output right/left/forward/backward
    def generate_control_command(self, detailed_info):

        action_type = detailed_info[0]

        # check whether the action type change
        if self.last_action_type != action_type:
            self.I = 0
            self.lastError = 0

        # For 'move' actions
        if action_type == 'move':
            distance = detailed_info[1]
            ###TODO: need to add pid control to calculate the x_speed
            # Calculate the speed
            v_x = self.update(distance)
            self.update_value = np.array([v_x, 0.0, 0.0])
            ###TODO: no more need to calculate the rotation time
            control_command = {"command": "move", "rb5_speed": self.update_value}

        # For 'rotate' actions
        elif action_type == 'rotate':
            rotation_before_move = detailed_info[1]
            # Calculate rotation times and move time
            ###TODO: need to add pid control to calculate the z_speed
            v_z = self.update(rotation_before_move)
            self.update_value = np.array([0.0, 0.0, v_z])
            ###TODO: no more need
            control_command = {"command": "rotate","rb5_speed": self.update_value}

        return control_command

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

if __name__ == "__main__":

    print("===start===\n")

    rospy.init_node("hw1")
    pub_twist = rospy.Publisher("/twist", Twist, queue_size=1)

    waypoint = np.array([
                        [0.0,0.0,0.0], 
                        [1.0,0.0,0.0],
                        [1.0,2.0,np.pi],
                        [0.0,0.0,0.0],
                        ])         

    # init pid controller
    pid = PIDcontroller(0.02,0.005,0.005)

    # init current state
    current_state = np.array([0.0,0.0,0.0])

    for wp in waypoint:
        print("move to way point", wp)
        # set wp as the target point
        pid.setTarget(wp)

        angle_details = pid.determine_angle_details(current_state)
        print(angle_details,'\n')

        action_details = pid.detailed_movement_information(angle_details)
        print(action_details,'\n') 

        control_command = pid.generate_control_command(action_details)
        print(control_command)

        # used to active
        # publish the twist
        pub_twist.publish(genTwistMsg())
        #print(coord(update_value, current_state))
        time.sleep(0.05)

        # update the current state
        current_state += pid.update_value
        while(np.linalg.norm(pid.getError(current_state, wp)) > 0.05): # check the error between current state and current way point
            # calculate the current twist
            angle_details = pid.determine_angle_details(current_state)
            print(angle_details,'\n')

            action_details = pid.detailed_movement_information(angle_details)
            print(action_details,'\n') 

            control_command = pid.generate_control_command(action_details)
            print(control_command)
            # publish the twist
            pub_twist.publish(genTwistMsg(pid.update_value))
            #print(coord(update_value, current_state))
            time.sleep(0.05)
            # update the current state
            current_state += pid.update_value
    # stop the car and exit
    pub_twist.publish(genTwistMsg(np.array([0.0,0.0,0.0])))
    print("===done===\n")

    