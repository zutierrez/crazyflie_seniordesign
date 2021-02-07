#!/bin/bash
screen -d -m -S launch bash -c 'cd ~/catkin_ws/src/CrazyS/rotors_gazebo/launch && roslaunch crazyflie2_internal_model_controller.launch'
cd ~/catkin_ws/scripts/crazyflie_seniordesign
python crazyflie_trajectory.py