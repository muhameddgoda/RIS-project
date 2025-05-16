#!/bin/bash
source /environment.sh
dt-launchfile-init
roslaunch my_package tag_action.launch
dt-launchfile-join
