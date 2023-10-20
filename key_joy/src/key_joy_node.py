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
import math

from sensor_msgs.msg import Joy
from key_parser import get_key, save_terminal_settings, restore_terminal_settings

class KeyJoyNode:
    def __init__(self):
        self.pub_joy = rospy.Publisher("/joy", Joy, queue_size=1)
        self.settings = save_terminal_settings()

    #Reads the nth line from the given file path and returns the values as x, y, and z.
    def read_nth_line(self, file_path, n):
        with open(file_path, "r") as file:
            lines = file.readlines()
            if n <= len(lines):
                x, y, z = map(float, lines[n-1].split(','))  # Indexing starts from 0
                return x, y, z
######
    def calculate_distance(self, point1, point2):
        """
        Calculates the Euclidean distance between two 2D points.
        """
        return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)

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
                results.append(['move', angle_diff, distance, rotation_after_move])
            # For 'rotate' actions
            else:
                if angle_diff < 1.57:
                    rotation_angle_change_1 = angle_diff
                    angle_difference_robot_target_1 = 0
                else:
                    rotation_angle_change_1 = 3.14 - angle_diff
                    angle_difference_robot_target_1 = 3.14
                distance = self.calculate_distance((detail["point1"][0], detail["point1"][1]), (detail["point2"][0], detail["point2"][1]))
                # Calculate the rotation angle required after reaching point2
                rotation_angle_change_2 = detail["point2"][2] - (detail["point1"][2] + rotation_angle_change_1)
                # Normalize rotation_angle_change_2 to between -pi and pi
                while rotation_angle_change_2 > math.pi:
                    rotation_angle_change_2 -= 2 * math.pi
                while rotation_angle_change_2 < -math.pi:
                    rotation_angle_change_2 += 2 * math.pi
                results.append(['rotate', rotation_angle_change_1, angle_difference_robot_target_1, distance, rotation_angle_change_2])
        return results

#####

    def run(self):
        # while True:
        #     # parse keyboard control
        #     key = get_key(self.settings, timeout=0.1)

        #     # interpret keyboard control as joy
        #     joy_msg, flag = self.key_to_joy(key)
        #     if flag is False:
        #         break

        #     # publish joy
        #     self.pub_joy.publish(joy_msg)
        print("start")

        all_coordinates = []

        num_lines = sum(1 for line in open('/root/rb5_ws/src/rb5_ros/key_joy/src/waypoints.txt'))
        #
        print(num_lines)
        #

        for i in range(1, num_lines + 1):
            x, y, z = self.read_nth_line('/root/rb5_ws/src/rb5_ros/key_joy/src/waypoints.txt', i)
            all_coordinates.append((x, y, z))
        
        #
        print(all_coordinates)
        #

        angle_details = self.determine_angle_details(all_coordinates)

        #
        print(angle_details)
        #

        self.detailed_move_rotate_info = self.detailed_movement_information_simplified(angle_details)

        #
        print(angle_details)
        #

        self.stop()

    # constant
    def key_to_joy(self, key):
        flag = True
        joy_msg = Joy()
        joy_msg.axes = [0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0]
        joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]

        # joy_msg.axes only change one index => carMixed unable
        if key == 'w':
            joy_msg.axes[1] = 1.0
        elif key == 's':
            joy_msg.axes[1] = -1.0

        elif key == 'a':
            joy_msg.axes[0] = -1.0
        elif key == 'd':
            joy_msg.axes[0] = 1.0

        elif key == 'q':
            joy_msg.axes[2] = -1.0
        elif key == 'e':
            joy_msg.axes[2] = 1.0

        elif (len(key) > 0 and ord(key) == 27) or (key == '\x03'):
            flag = False

        return joy_msg, flag
    

    def stop(self):
        restore_terminal_settings(self.settings)


if __name__ == "__main__":
    key_joy_node = KeyJoyNode()
    rospy.init_node("key_joy")
    key_joy_node.run()