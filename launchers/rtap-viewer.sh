#!/bin/bash

source /environment.sh
dt-launchfile-init

# Launch the RTAP Viewer node
rosrun my_package rtap_viewer_node.py

dt-launchfile-join
