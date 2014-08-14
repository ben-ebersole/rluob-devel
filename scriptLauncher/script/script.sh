#!/bin/sh

echo "The script starts now."

echo "Isn't this fun!?"

gnome-terminal -e "/opt/ros/hydro/bin/roscore"

gnome-terminal -e "/opt/ros/hydro/bin/rosrun rviz rviz"

