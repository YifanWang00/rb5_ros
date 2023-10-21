#!/usr/bin/env python
"""
Copyright 2023, UC San Diego, Contextual Robotics Institute

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""
import sys
import rospy
import time
import math

from sensor_msgs.msg import Joy
from key_parser import get_key, save_terminal_settings, restore_terminal_settings

class KeyJoyNode:
    def __init__(self):
        self.pub_joy = rospy.Publisher("/joy", Joy, queue_size=1)
        self.settings = save_terminal_settings()
        # Used to debug
        # self.i = 0

    def run(self):

        print("===start===\n")

        # Load all the coordinates from file
        all_coordinates = []

        # Get the num of points
        num_lines = sum(1 for line in open('/root/rb5_ws/src/rb5_ros/key_joy/src/waypoints.txt'))
        print("There are",num_lines, "points\n")

        # Load points
        for i in range(1, num_lines + 1):
            x, y, z = self.read_nth_line('/root/rb5_ws/src/rb5_ros/key_joy/src/waypoints.txt', i)
            all_coordinates.append((x, y, z))
            print((x, y, z))
            print('\n')

        # Do angle calulate for subsequent calculation
        angle_details = self.determine_angle_details(all_coordinates)

        # Make action decision based on the angle details
        detailed_move_rotate_info = self.detailed_movement_information_simplified(angle_details)

        # linear_velocity(linear_velocity_1 represents straight velocity while linear_velocity_2 represents slide velocity) and angular_velocity
        linear_velocity_1 = 0.21 * 2   # m/s
        linear_velocity_2 = 0.145 * 2   # m/s
        angular_velocity = 1.815  # rad/s

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

    def detailed_movement_information_simplified(self, angle_details):
        """
        Provides detailed movement information for both 'rotate' and 'move' actions, with simplified calculations.
        Returns a list of actions and their respective details.
        """
        results = []
        for detail in angle_details:
            angle_diff = round(detail["angle_difference_robot_target"], 2)
            # Check if the rounded angle difference is close to any of the four angles for 'move' actions
            if any(abs(angle_diff - val) <= 0.05 for val in [0, 3.14, 1.57, -1.57]):
                distance = self.calculate_distance((detail["point1"][0], detail["point1"][1]), (detail["point2"][0], detail["point2"][1]))
                rotation_after_move = detail["point2"][2] - detail["point1"][2]
                while rotation_after_move > math.pi:
                    rotation_after_move -= 2 * math.pi
                while rotation_after_move < -math.pi:
                    rotation_after_move += 2 * math.pi
                rotation_after_move = round(rotation_after_move, 2)
                results.append(['move', angle_diff, distance, rotation_after_move])
            # For 'rotate' actions
            else:
                if angle_diff < 1.57:
                    rotation_before_move = angle_diff
                    angle_difference_robot_target_1 = 0
                else:
                    rotation_before_move = 3.14 - angle_diff
                    angle_difference_robot_target_1 = 3.14
                distance = self.calculate_distance((detail["point1"][0], detail["point1"][1]), (detail["point2"][0], detail["point2"][1]))
                # Calculate the rotation angle required after reaching point2
                rotation_after_move = detail["point2"][2] - (detail["point1"][2] + rotation_before_move)
                # Normalize rotation_after_move to between -pi and pi
                while rotation_after_move > math.pi:
                    rotation_after_move -= 2 * math.pi
                while rotation_after_move < -math.pi:
                    rotation_after_move += 2 * math.pi
                rotation_after_move = round(rotation_after_move, 2)
                results.append(['rotate', rotation_before_move, angle_difference_robot_target_1, distance, rotation_after_move])
        return results

    def generate_control_commands(self, detailed_info, linear_velocity_1, linear_velocity_2, angular_velocity):
        """
        Converts the detailed movement information into specific control commands and required times.
        """
        control_commands = []

        for action_info in detailed_info:
            action_type = action_info[0]

            # For 'move' actions
            if action_type == 'move':
                _, direction, distance, rotation_after_move = action_info
                # Calculate move time and rotation time
                if abs(direction - 0) < 0.1 or abs(direction - 3.14) < 0.1:
                   move_time = distance / linear_velocity_1
                else:
                    move_time = distance / linear_velocity_2
                rotation_time = abs(rotation_after_move) / angular_velocity
                control_commands.append({"command": "move", "direction": direction, "time": move_time})
                if rotation_time > 0.1:  # Only add a rotate command if there's a need to rotate after moving
                    control_commands.append({"command": "rotate", "angle": rotation_after_move, "time": rotation_time})

            # For 'rotate' actions
            elif action_type == 'rotate':
                _, rotation_before_move, direction, distance, rotation_after_move = action_info
                # Calculate rotation times and move time
                rotation_time_1 = abs(rotation_before_move) / angular_velocity
                move_time = distance / linear_velocity_1
                rotation_time_2 = abs(rotation_after_move) / angular_velocity
                control_commands.append({"command": "rotate", "angle": rotation_before_move, "time": rotation_time_1})
                control_commands.append({"command": "move", "direction": direction, "time": move_time})
                if rotation_time_2 > 0.1:  # Only add a rotate command if there's a need to rotate after moving
                    control_commands.append({"command": "rotate", "angle": rotation_after_move, "time": rotation_time_2})

        return control_commands

    def execute_command(self,command_detail, joy_msg):
        """
        Executes the given command based on the details provided.
        """
        if command_detail["command"] == "move":
            if abs(command_detail["direction"] - 0) <= 0.05:
                # Execute move in the 0 direction
                print("Moving in direction: 0, time:",command_detail["time"])
                joy_msg.axes[1] = 1.0
            elif abs(command_detail["direction"] - 3.14) <= 0.05:
                # Execute move in the 3.14 direction
                print("Moving in direction: 3.14",command_detail["time"])
                joy_msg.axes[1] = -1.0
            elif abs(command_detail["direction"] - 1.57) <= 0.05:
                # Execute move in the 1.57 direction
                print("Moving in direction: 1.57",command_detail["time"])
                joy_msg.axes[0] = -1.0
            elif abs(command_detail["direction"] + 1.57) <= 0.05:
                # Execute move in the -1.57 direction
                print("Moving in direction: -1.57",command_detail["time"])
                joy_msg.axes[0] = 1.0

        elif command_detail["command"] == "rotate":
            if command_detail["angle"] > 0:
                # Execute clockwise rotation
                print("Rotating clockwise",command_detail["time"])
                joy_msg.axes[2] = 1.0
            else:
                # Execute counter-clockwise rotation
                print("Rotating counter-clockwise",command_detail["time"])
                joy_msg.axes[2] = -1.0
        command_time = command_detail["time"]
        return joy_msg, command_time
    
    def stop(self):
        restore_terminal_settings(self.settings)

if __name__ == "__main__":
    key_joy_node = KeyJoyNode()
    rospy.init_node("key_joy")
    key_joy_node.run()