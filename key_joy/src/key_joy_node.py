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
        print(num_lines)
        for i in range(1, num_lines + 1):
            x, y, z = self.read_nth_line('/root/rb5_ws/src/rb5_ros/key_joy/src/waypoints.txt', i)
            all_coordinates.append((x, y, z))
        
        print(all_coordinates)
    
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