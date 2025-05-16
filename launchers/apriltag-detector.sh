#!/bin/bash
source /environment.sh
dt-launchfile-init
roslaunch my_package apriltag_detector.launch
dt-launchfile-join
