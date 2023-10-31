#!/usr/bin/env python

import sys
import rospy
import time
import math
import numpy as np
from geometry_msgs.msg import Twist

from sensor_msgs.msg import Joy
from key_parser import get_key, save_terminal_settings, restore_terminal_settings

# Global 
pid = None
pub_twist = None
current_waypoint_index = 0
waypoint = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]) 
current_state = np.array([0.0, 0.0, 0.0]) 
step_num = 0

class KeyJoyNode:
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
    
    def run(self):

        print("===start===\n")

        ###TODO: change to waypoint
        # Load all the coordinates from file
        all_coordinates = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]])

        # Do angle calulate for subsequent calculation
        angle_details = self.determine_angle_details(all_coordinates)

        # Make action decision based on the angle details
        detailed_move_rotate_info = self.detailed_movement_information_simplified(angle_details)

        ###TODO: no more need this 
        # linear_velocity(linear_velocity_1 represents straight velocity while linear_velocity_2 represents slide velocity) and angular_velocity
        # linear_velocity_1 = 0.21 * 2   # m/s
        # linear_velocity_2 = 0.145 * 2   # m/s
        # angular_velocity = 1.815  # rad/s

        ###TODO:
        # Generate control commands based on action decision
        control_commands = self.generate_control_commands(detailed_move_rotate_info, linear_velocity_1, linear_velocity_2, angular_velocity)

        # Activate the control node
        joy_msg = Joy()
        joy_msg.axes = [0.0 ,1.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0]
        joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
        time.sleep(1)

        # Used to debug
        # self.i = self.i + 1
        # print(self.i)
        # print('\n')

        # Execute
        for cmd in control_commands:
            # Print cmd
            print(cmd)
            print('\n')

            joy_msg = Joy()
            joy_msg.axes = [0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0]
            joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
            joy_msg, command_time = self.execute_command(cmd, joy_msg)

            # Used to debug
            # self.i = self.i + 1
            # print(self.i)
            # print('\n')
            # print(joy_msg.axes)
            # print('\n')

            # Publish joy
            self.pub_joy.publish(joy_msg)
            print('Command Sent!\n')   
            time.sleep(command_time)
            print('Command Completed!\n')

            # Used to debug
            # self.i = self.i + 1
            # print(self.i)
            # print('\n')

            # Stop the robot
            joy_msg = Joy()
            joy_msg.axes = [0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0]
            joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
            self.pub_joy.publish(joy_msg)

            time.sleep(1)  # Waits for 1 seconds before moving to the next iteration

        # Used to debug
        # self.i = self.i + 1
        # print(self.i)
        # print('\n')

        # Stop the robot
        print('===All the commands done===')
        joy_msg = Joy()
        joy_msg.axes = [0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0]
        joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
        self.pub_joy.publish(joy_msg)

        self.stop()

    #Reads the nth line from the given file path and returns the values as x, y, and z.
    def read_nth_line(self, file_path, n):
        with open(file_path, "r") as file:
            lines = file.readlines()
            if n <= len(lines):
                x, y, z = map(float, lines[n-1].split(','))  # Indexing starts from 0
                return x, y, z

    ###TODO: change points to point_1(state) and point_2(targrt)
    def determine_angle_details(self, points):
        """
        Determines the target angle and the angle difference for the robot for each pair of points.
        """
        angle_details = []
        for i in range(len(points) - 1):
            point1, point2 = points[i], points[i+1]
            # Calculate target angle
            z_target_axis = math.atan2(point2[1] - point1[1], point2[0] - point1[0])
            # Normalize the result to between -pi and pi
            while z_target_axis > math.pi:
                z_target_axis -= 2 * math.pi
            while z_target_axis < -math.pi:
                z_target_axis += 2 * math.pi
            # Calculate angle difference
            angle_difference_robot_target = z_target_axis - point1[2]
            # Normalize the result to between -pi and pi
            while angle_difference_robot_target > math.pi:
                angle_difference_robot_target -= 2 * math.pi
            while angle_difference_robot_target < -math.pi:
                angle_difference_robot_target += 2 * math.pi
            # Append the details to the list
            angle_details.append({"point1": point1, "point2": point2, "z_target_axis": z_target_axis, "angle_difference_robot_target": angle_difference_robot_target})
        return angle_details

    def calculate_distance(self, point1, point2):
        """
        Calculates the Euclidean distance between two 2D points.
        """
        return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)

    ###TODO: change to only one action
    def detailed_movement_information_simplified(self, angle_details):
        """
        Provides detailed movement information for both 'rotate' and 'move' actions, with simplified calculations.
        Returns a list of actions and their respective details.
        """
        results = []
        for detail in angle_details:
            angle_diff = round(detail["angle_difference_robot_target"], 2)
            # Check if the rounded angle difference is close to any of the four angles for 'move' actions
            ###TODO: change to only forward and backward, need to focus on tolerance
            ###TODO: no more need 1.57 and -1.57
            if abs(angle_diff - 0) <= self.angular_tolerance:
                distance = self.calculate_distance((detail["point1"][0], detail["point1"][1]), (detail["point2"][0], detail["point2"][1]))
            elif abs(angle_diff - math.pi) <= self.angular_tolerance or abs(angle_diff + math.pi) <= self.angular_tolerance:
                distance = -self.calculate_distance((detail["point1"][0], detail["point1"][1]), (detail["point2"][0], detail["point2"][1]))
                ###TODO: distance used to input into pid control, no more need rotation_after_move
                results.append(['move', distance])
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
                results.append(['rotate', rotation_before_move])
        return results

    ###TODO: need to output right/left/forward/backward
    def generate_control_commands(self, detailed_info):
        """
        Converts the detailed movement information into specific control commands and required times.
        """
        control_commands = []

        for action_info in detailed_info:
            action_type = action_info[0]

            # check whether the action type change
            if self.last_action_type != action_type:
                self.I = 0
                self.lastError = 0

            # For 'move' actions
            if action_type == 'move':
                _, distance = action_info
                ###TODO: need to add pid control to calculate the x_speed
                # Calculate the speed
                v_x = self.update(distance)
                self.update_value = np.array([v_x,0.0,0.0])
                ###TODO: no more need to calculate the rotation time
                control_commands.append({"command": "move", "rb5_speed": self.update_value})

            # For 'rotate' actions
            elif action_type == 'rotate':
                _, rotation_before_move = action_info
                # Calculate rotation times and move time
                ###TODO: need to add pid control to calculate the z_speed
                v_z = self.update(rotation_before_move)
                self.update_value = np.array([0,0.0,v_z])
                ###TODO: no more need
                control_commands.append({"command": "rotate","rb5_speed": self.update_value})

        return control_commands

if __name__ == "__main__":
    key_joy_node = KeyJoyNode()
    rospy.init_node("key_joy")
    key_joy_node.run()