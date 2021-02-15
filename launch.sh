#!/bin/bash
screen -d -m -S launch bash -c 'cd ~/catkin_ws/src/CrazyS/rotors_gazebo/launch && roslaunch crazyflie2_internal_model_controller.launch'
cd ~/catkin_ws/scripts/crazyflie_seniordesign
PID=$(pgrep "$BASECMD")
if [ "$?" -eq "0" ]; then
    echo "at least one instance of "$BASECMD" found, killing all instances"
    kill $PID
else
    echo "no running instances of "$BASECMD" found, starting one"
    $1
 fi
python crazyflie_trajectory.py