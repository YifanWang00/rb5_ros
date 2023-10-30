#!/usr/bin/env python
import rospy
from rb5_message.msg import rb5_message
import numpy as np

def callback(data):
    rospy.loginfo("Received rb5_message: [%f, %f, %f]", data.data[0], data.data[1], data.data[2])

if __name__ == '__main__':
    rospy.init_node('state_subscriber')
    rospy.Subscriber('rb5_state_topic', rb5_message, callback, queue_size=1)
    rospy.spin()


