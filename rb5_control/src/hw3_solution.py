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
from geometry_msgs.msg import Twist
from tf.transformations import quaternion_matrix

class PIDcontroller:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.target = None
        self.I = 0
        self.lastError = 0
        self.timestep = 0.1

        self.maximumValue = 1
        self.angular_tolerance = 0.9
        self.last_action_type = "move"
        self.update_value = np.array([0.0,0.0,0.0])

        self.v_straight = 0.085
        self.v_rotate = 0.85

    def setTarget(self, state):
        """
        set the target pose.
        """
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
            if(result > self.maximumValue):
                result = self.maximumValue * flag
                self.I = 0.0

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
            if(result > self.maximumValue):
                result = self.maximumValue * flag
                self.I = 0.0

            result = self.v_rotate * flag
        return result
    
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

    def detailed_movement_information(self, angle_details):

        angle_diff = round(angle_details["angle_difference_robot_target"], 2)
        # Check if the rounded angle difference is close to any of the four angles for 'move' actions
        distance = self.calculate_distance((angle_details["current_state"][0], angle_details["current_state"][1]), (angle_details["target"][0], angle_details["target"][1]))
        
        if (abs(angle_diff - 0) <= self.angular_tolerance) and distance > 0.05:
            result = ['move', distance]
            return result
        elif (abs(angle_diff - math.pi) <= self.angular_tolerance or abs(angle_diff + math.pi) <= self.angular_tolerance) and distance > 0.05:
            result = ['move', -distance]
            return result
        # For 'rotate' actions
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

def getCurrentPos(l):
    """
    Given the tf listener, we consider the camera's z-axis is the header of the car
    """
    br = tf.TransformBroadcaster()
    result = None
    foundSolution = False

    for i in range(0, 9):
        camera_name = "camera_" + str(i)
        if l.frameExists(camera_name):
            try:
                now = rospy.Time()
                # wait for the transform ready from the map to the camera for 1 second.
                l.waitForTransform("map", camera_name, now, rospy.Duration(1.0))
                # extract the transform camera pose in the map coordinate.
                (trans, rot) = l.lookupTransform("map", camera_name, now)
                # convert the rotate matrix to theta angle in 2d
                matrix = quaternion_matrix(rot)
                angle = math.atan2(matrix[1][2], matrix[0][2])
                # this is not required, I just used this for debug in RVIZ
                br.sendTransform((trans[0], trans[1], 0), tf.transformations.quaternion_from_euler(0,0,angle), rospy.Time.now(), "base_link", "map")
                result = np.array([trans[0], trans[1], angle])
                foundSolution = True
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf2_ros.TransformException):
                print("meet error")
    listener.clear()
    return foundSolution, result

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

if __name__ == "__main__":

    print("===start===\n")

    rospy.init_node("hw3")
    pub_twist = rospy.Publisher("/twist", Twist, queue_size=1)

    listener = tf.TransformListener()

    #square
    waypoint = np.array([
                        [0.0,0.0,0.0], 
                        [0.8,0.0, math.pi/2],
                        [0.8,0.8,math.pi], 
                        [0.0,0.8, -math.pi/2],
                        [0.0,0.0,0.0]
                        ])         

    # init pid controller
    pid = PIDcontroller(0.02,0.005,0.005)

    # init current state
    current_state = np.array([0.0,0.0,0.0])
    print(current_state)

    # ! active 
    found_state, estimated_state = getCurrentPos(listener)
    pub_twist.publish(genTwistMsg(pid.update_value))
    time.sleep(1)

    for wp in waypoint:
        # print("move to way point", wp)

        # set wp as the target point
        pid.setTarget(wp)

        angle_details = pid.determine_angle_details(current_state)
        # print(angle_details,'\n')

        action_details = pid.detailed_movement_information(angle_details)
        # print(action_details,'\n') 

        control_command = pid.generate_control_command(action_details)
        # print(control_command)

        # publish the twist
        pub_twist.publish(genTwistMsg(pid.update_value))
        time.sleep(pid.timestep)

        # update the current state
        # noise = np.random.uniform(low=-0.001, high=0.001, size=(3, ))
        # current_state += local_to_global_velocity(pid.update_value, current_state[2]) * 10 + noise
        current_state += local_to_global_velocity(pid.update_value, current_state[2]) * pid.timestep
        
        found_state, estimated_state = getCurrentPos(listener)

        if found_state: # if the tag is detected, we can use it to update current state.
            current_state = estimated_state

        # Normalize the result to between -pi and pi
        if current_state[2] > math.pi:
            current_state[2] -= 2 * math.pi
        if current_state[2] < -math.pi:
            current_state[2] += 2 * math.pi
        print(current_state)
    
        while(np.linalg.norm(pid.getError(current_state, wp)) > 0.11): # check the error between current state and current way point
            # calculate the current twist
            angle_details = pid.determine_angle_details(current_state)
            # print(angle_details,'\n')

            action_details = pid.detailed_movement_information(angle_details)
            # print(action_details,'\n') 

            control_command = pid.generate_control_command(action_details)
            # print(control_command)

            # publish the twist
            pub_twist.publish(genTwistMsg(pid.update_value))
            time.sleep(pid.timestep)

            # update the current state
            # noise = np.random.uniform(low=-0.001, high=0.001, size=(3, ))
            # current_state += local_to_global_velocity(pid.update_value, current_state[2]) * 10 + noise
            current_state += local_to_global_velocity(pid.update_value, current_state[2]) * pid.timestep

            found_state, estimated_state = getCurrentPos(listener)
            if found_state:
                current_state = estimated_state

            # Normalize the result to between -pi and pi
            if current_state[2] > math.pi:
                current_state[2] -= 2 * math.pi
            if current_state[2] < -math.pi:
                current_state[2] += 2 * math.pi
            # print("=====")
            print(current_state)

    # stop the car and exit
    pub_twist.publish(genTwistMsg(np.array([0.0,0.0,0.0])))
    print("===done===!\n")

    