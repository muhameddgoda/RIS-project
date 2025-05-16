#!/bin/bash
source /environment.sh
dt-launchfile-init

rosrun my_package apriltag_detector_node.py

dt-launchfile-join
