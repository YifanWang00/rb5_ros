#!/bin/bash

rosrun tf static_transform_publisher 1.625 0 0 0 0 0 0 map marker_0 &
rosrun tf static_transform_publisher 1.625 2 0 0 0 0 0 map marker_1 &
rosrun tf static_transform_publisher -0.625 2.5 0 0 0 0 0 map marker_2 &
rosrun tf static_transform_publisher -0.625 1 0 0 0 0 0 map marker_3 &
rosrun tf static_transform_publisher -0.1875 -0.75 0 0 0 0 0 map marker_4 &
