#!/bin/bash
source /environment.sh
dt-launchfile-init

# Keyboard teleop; adjust topic if needed
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/cmd_vel

dt-launchfile-join
