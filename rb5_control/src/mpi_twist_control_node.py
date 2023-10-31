#!/usr/bin/env python
""" MegaPi Controller ROS Wrapper"""
import rospy

from geometry_msgs.msg import Twist
from mpi_control import MegaPiController
import numpy as np


class MegaPiControllerNode:
    def __init__(self, verbose=False, debug=False):
        self.mpi_ctrl = MegaPiController(port='/dev/ttyUSB0', verbose=verbose)
        self.r = 0.025 # radius of the wheel
        self.lx = 0.055 # half of the distance between front wheel and back wheel
        self.ly = 0.07 # half of the distance between left wheel and right wheel

        """
        1:
            single
        2:
            xz
        3:
            yz
        4:
            yx
        """

        ###point 1 (1, 0, 0):
        # self.calibration_x = -5.0
        # self.calibration_y = -90.0
        # self.calibration_z = 190.0 #0.031-0.022 pi/2

        # self.calibration_z = 120.0 #0.063-0.041 pi

        ###point 2 (1, 2, pi):
        # self.calibration_x = 70.0
        # self.calibration_y = -170.0
        # self.calibration_z = 95.0 #0.063-0.041 pi y_up z_down based on y = 2, pi = pi 

        # self.calibration_x = 80.0 #0.2-0.13
        # self.calibration_y = -150.0 #0.04-0.027
        # self.calibration_z = 85.0 #based on y = -2 x = -1 

        ###point 0 (0, 0, 0):
        # self.calibration_x = 100.0 #0.2-0.13
        # self.calibration_y = -130.0 #0.04-0.027
        # self.calibration_z = 90.0 #based on y = -2 x = -1 

        # default:
        self.calibration_x = 50.0 
        self.calibration_y = -50.0 
        self.calibration_z = 50.0 

    def twist_callback(self, twist_cmd):
        # if(
        #     (abs(twist_cmd.linear.x - 0.014) + abs(twist_cmd.linear.y - 0.032) + abs(twist_cmd.angular.z + 0.049)) < 0.01\
        #     and (abs(self.calibration_x - 100.0) + abs(self.calibration_y + 130.0) + abs(self.calibration_z - 90.0)) > 1
        #    ):
        #     self.calibration_x = 90.0 
        #     self.calibration_y = -120.0
        #     self.calibration_z = 90.0 
        # elif(
        #     (abs(twist_cmd.linear.x - 0.020) + abs(twist_cmd.linear.y - 0.0) + abs(twist_cmd.angular.z + 0.0)) < 0.01
        #     ):
        #     self.calibration_x = 87.0 
        # elif(
        #     (abs(twist_cmd.linear.x + 0.016) + abs(twist_cmd.linear.y - 0.032) + abs(twist_cmd.angular.z + 0.051)) < 0.01\
        #     and (abs(self.calibration_x - 70) + abs(self.calibration_y + 170.0) + abs(self.calibration_z - 95.0)) > 1       
        #     ):
        #     self.calibration_x = 80.0
        #     self.calibration_y = -150.0 
        #     self.calibration_z = 85.0 #0.063-0.041 pi y_up z_down based on y = 2, pi = pi 
        desired_twist = np.array([[self.calibration_x * twist_cmd.linear.x], [self.calibration_y * twist_cmd.linear.y], [self.calibration_z * twist_cmd.angular.z]])
        # calculate the jacobian matrix
        jacobian_matrix = np.array([[1, -1, -(self.lx + self.ly)],
                                     [1, 1, (self.lx + self.ly)],
                                     [1, 1, -(self.lx + self.ly)],
                                     [1, -1, (self.lx + self.ly)]]) / self.r
        # calculate the desired wheel velocity
        result = np.dot(jacobian_matrix, desired_twist)

        # send command to each wheel
        self.mpi_ctrl.setFourMotors(result[0][0], result[1][0], result[2][0], result[3][0])
        

if __name__ == "__main__":
    mpi_ctrl_node = MegaPiControllerNode()
    rospy.init_node('megapi_controller')
    rospy.Subscriber('/twist', Twist, mpi_ctrl_node.twist_callback, queue_size=1) 
    
    rospy.spin()
