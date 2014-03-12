#!/bin/bash

./convert_xacro.sh

rosnode kill metralabs_ros schunk_state_publisher

sleep 1

#file=$(find scitos_description)/models/scitos_haw+schunk.urdf
#setting ros params
#rosparam set robot_description $(cat $(rospack find scitos_description)/models/scitos_haw+arm_5dof.urdf)
#rosparam set schunk_description $(cat $(find scitos_description)/models/scitos_haw+arm_5dof.urdf)

roslaunch metralabs_ros scitos_haw_schunk_start.launch &

