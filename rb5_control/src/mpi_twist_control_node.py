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

        ###1:
        # self.calibration_x = 70.0
        # self.calibration_y = -90.0
        # self.calibration_z = 190.0 #0.031-0.022 pi/2
        # self.calibration_z = 120.0 #0.063-0.041 pi

        ###2:
        # self.calibration_x = 90.0
        # self.calibration_y = -90.0
        # self.calibration_z = 80.0 #0.031-0.022 pi/2 0.063-0.041 pi

        ##3:
        # self.calibration_x = 90.0
        # self.calibration_y = -90.0
        # self.calibration_z = 80.0 #0.063-0.041 pi y_up z_down based on y = 2, pi = pi 

        ##4:
        # self.calibration_x = 70.0 #0.2-0.13
        # self.calibration_y = -40.0 #0.04-0.027
        # self.calibration_z = 80.0 #based on y = -2 x = -1 

        #5:
        self.calibration_x = 70.0 #0.014-0.009
        self.calibration_y = -40.0 #0.031-0.016
        self.calibration_z = 80.0 #0.48-0.26 ----- differ from +++++

        # self.calibration_x = 70.0
        # self.calibration_x = 100.0
        # self.calibration_y = -100.0
        # self.calibration_z = 180d.0
        # self.calibration_z = 100.0

    def twist_callback(self, twist_cmd):
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
