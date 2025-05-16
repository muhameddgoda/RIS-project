#!/bin/bash
source /environment.sh
dt-launchfile-init
roslaunch my_package line_follower.launch
dt-launchfile-join
