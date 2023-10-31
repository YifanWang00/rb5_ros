#!/usr/bin/env python
import sys
import rospy
import time
import math
from geometry_msgs.msg import Twist
import numpy as np
from rb5_message.msg import rb5_message

# Global 
pid = None
pub_twist = None
current_waypoint_index = 0
waypoint = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]) 
current_state = np.array([0.0, 0.0, 0.0]) 
step_num = 0

"""
The class of the pid controller.
"""
class PIDcontroller:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target = None
        self.I = np.array([0.0,0.0,0.0])
        self.lastError = np.array([0.0,0.0,0.0])
        self.timestep = 0.1
        self.maximumValue = 0.06

    def setTarget(self, targetx, targety, targetw):
        """
        set the target pose.
        """
        self.I = np.array([0.0,0.0,0.0]) 
        self.lastError = np.array([0.0,0.0,0.0])
        self.target = np.array([targetx, targety, targetw])

    def setTarget(self, state):
        """
        set the target pose.
        """
        self.I = np.array([0.0,0.0,0.0]) 
        self.lastError = np.array([0.0,0.0,0.0])
        self.target = np.array(state)
        print("move to way point", self.target)

    def getError(self, currentState, targetState):
        """
        return the different between two states
        """
        result = targetState - currentState
        result[2] = (result[2] + np.pi) % (2 * np.pi) - np.pi
        return result 

    def setMaximumUpdate(self, mv):
        """
        set maximum velocity for stability.
        """
        self.maximumValue = mv

    def update(self, currentState):
        """
        calculate the update value on the state based on the error between current state and target state with PID.
        """
        e = self.getError(currentState, self.target)

        P = self.Kp * e

        self.I += self.Ki * e
        I = self.I
        # if math.isnan(I[0]):
        #     I = np.array([0.0,0.0,0.0])

        D = self.Kd * (e - self.lastError)
        # if math.isnan(self.D[0]):
        #     D = np.array([0.0,0.0,0.0])

        result = P + I + D

        self.lastError = e

        # scale down the twist if its norm is more than the maximum value. 
        resultNorm = np.linalg.norm(result)
        if(resultNorm > self.maximumValue):
            result = (result / resultNorm) * self.maximumValue
            self.I = 0.0

        print("World speed:" ,result)

        return result

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

def coord(twist, current_state):
    J = np.array([[np.cos(current_state[2]), np.sin(current_state[2]), 0.0],
                  [-np.sin(current_state[2]), np.cos(current_state[2]), 0.0],
                  [0.0,0.0,1.0]])
    return np.dot(J, twist)
    
def rb5_message_callback(data):
    global current_state, current_waypoint_index, pub_twist, pid, waypoint, step_num

    current_state = np.array([round(data.data[0], 2),round(data.data[1], 2),round(data.data[2], 2)])

    if math.isnan(current_state[0]):
        print("BAD MESSAGE!")
        return

    step_num = step_num + 1
    print("====",step_num,"====")
    print("Current state:", current_state, type(current_state))
    print("Target state:", np.array(waypoint[current_waypoint_index]), type(waypoint[current_waypoint_index]))

    target_point  = np.array(waypoint[current_waypoint_index])

    if np.linalg.norm(pid.getError(current_state, target_point)) <= 0.05:
        current_waypoint_index += 1
        if current_waypoint_index >= len(waypoint):
            # pub_twist.publish(genTwistMsg(np.array([0.0,0.0,0.0]))) 
            return

        pid.setTarget(target_point) 
        step_num = 0

    update_value = pid.update(current_state)
    print("Car speed:",coord(update_value, current_state))
    # pub_twist.publish(genTwistMsg(coord(update_value, current_state)))

def stop_motors():
    pub_twist.publish(genTwistMsg(np.array([0.0, 0.0, 0.0]))) 

def main():
    global pid, pub_twist, current_state

    rospy.init_node("hw1")
    pub_twist = rospy.Publisher("/twist", Twist, queue_size=1)

    pid = PIDcontroller(0.02, 0.005, 0.005)
    pid.setTarget(waypoint[0]) 

    rospy.Subscriber("/rb5_state_topic", rb5_message, rb5_message_callback)

    rospy.on_shutdown(stop_motors)

    rospy.spin()
    

if __name__ == "__main__":
    main()

    # rospy.init_node("hw1")
    # pub_twist = rospy.Publisher("/twist", Twist, queue_size=1)

    # waypoint = np.array([[0.0,0.0,0.0], 
    #                      [1.0,2.0,-np.pi/2],
    #                     #  [1.0,1.0,0.0],
    #                     #  [-1.0,-1.0,np.pi/2.0],

    #                     #  [-1.0,1.0,np.pi/2.0],
    #                     #  [-2.0,1.0,0.0],
    #                     #  [-2.0,2.0,-np.pi/2.0],
    #                     #  [-1.0,1.0,-np.pi/4.0],
    #                     #  [0.0,0.0,0.0]
    #                      ]) 

    # # init pid controller
    # pid = PIDcontroller(0.02,0.005,0.005)

    # # init current state
    # current_state = np.array([0.0,0.0,0.0])

    # # in this loop we will go through each way point.
    # # once error between the current state and the current way point is small enough, 
    # # the current way point will be updated with a new point.
    # for wp in waypoint:
    #     print("move to way point", wp)
    #     # set wp as the target point
    #     pid.setTarget(wp)

    #     # calculate the current twist
    #     update_value = pid.update(current_state)
    #     # publish the twist
    #     pub_twist.publish(genTwistMsg(coord(update_value, current_state)))
    #     #print(coord(update_value, current_state))
    #     time.sleep(0.05)
    #     # update the current state
    #     current_state += update_value
    #     while(np.linalg.norm(pid.getError(current_state, wp)) > 0.05): # check the error between current state and current way point
    #         # calculate the current twist
    #         update_value = pid.update(current_state)
    #         # publish the twist
    #         pub_twist.publish(genTwistMsg(coord(update_value, current_state)))
    #         # print(coord(update_value, current_state))
    #         time.sleep(0.05)
    #         # update the current state
    #         current_state += update_value
    # # stop the car and exit
    # pub_twist.publish(genTwistMsg(np.array([0.0,0.0,0.0])))

