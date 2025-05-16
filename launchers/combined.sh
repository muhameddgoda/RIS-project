#!/bin/bash
source /environment.sh
dt-launchfile-init
roslaunch my_package combined.launch
dt-launchfile-join
