#!/bin/bash
source /environment.sh
dt-launchfile-init

rosrun my_package rtap_mapper_node.py

dt-launchfile-join
