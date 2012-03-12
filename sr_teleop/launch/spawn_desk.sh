#!/bin/bash
rosparam set -t `rospack find sr_teleop`/robot/desk.urdf desk_description
rosrun gazebo spawn_model -param desk_description -urdf -x 0.8 -y -0.1 -z 0.001 -model desk
rosrun tf static_transform_publisher 0 0 0 0 0 0 world desk_plate 100





