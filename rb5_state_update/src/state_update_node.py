#!/usr/bin/env python
import rospy
from rb5_message.msg import rb5_message
import numpy as np
import time

def publisher_node():
    print("1")
    rospy.init_node('state_publisher')
    pub = rospy.Publisher('rb5_state_topic', rb5_message, queue_size=1)
    print("1")

    #active
    state_msg = rb5_message()
    state_msg.data = np.array([0.0,0.0,0.0])  # For testing purposes, we'll use random values
    pub.publish(state_msg)
    time.sleep(1)

    state_msg = rb5_message()
    state_msg.data = np.array([0.0,0.0,0.0])  # For testing purposes, we'll use random values
    pub.publish(state_msg)
    time.sleep(1)
    state_msg = rb5_message()
    state_msg.data = np.array([0.5,0.0,0.0])  # For testing purposes, we'll use random values
    pub.publish(state_msg)
    time.sleep(1)
    state_msg = rb5_message()
    state_msg.data = np.array([1.0,0.0,0.0])  # For testing purposes, we'll use random values
    pub.publish(state_msg)
    time.sleep(1)
    state_msg = rb5_message()
    state_msg.data = np.array([1.5,0.0,0.0])  # For testing purposes, we'll use random values
    pub.publish(state_msg)
    time.sleep(1)
    state_msg = rb5_message()
    state_msg.data = np.array([2.0,0.0,0.0])  # For testing purposes, we'll use random values
    pub.publish(state_msg)
    time.sleep(1)

if __name__ == '__main__':

    publisher_node()


