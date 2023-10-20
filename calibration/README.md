### A ROS node for robot calibration

This is a ros node used to calibrate the robot for UCSD CSE-276A-23Fall. You can launch calibration by:

`$ ./setup_calibration.sh`

By running the script above you create a tmux session called *calibration*. Then you can start calibrating the robot in the lower right window! 

Here's some function we have implemented / plan to implement:
- Velocities measurement
    - [x] Press 'n' to start / stop the car from going forward.
    - [x] Press 'm' to start / stop the car from spinning in place.
    - [ ] Record the interval time of each run.
        - Todo: record what kind of run (straight, spin) it is.
- Speed adjustment
    - [x] Press 'r' to reset speed
    - [x] Press 't'/'g' to increse / decrease slide speed.
    - [x] Press 'y'/'h' to increse / decrease straight speed.
    - [x] Press 'u'/'j' to increse / decrease rotate speed.
