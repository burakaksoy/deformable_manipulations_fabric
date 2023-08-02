#!/bin/bash
sleep 1s;

gnome-terminal --tab --title="ROSCORE" --command "bash -c \"source ~/.bashrc; killall gzclient && killall gzserver; roscore; exec bash\"";
sleep 1s;

gnome-terminal --tab --title="All" --command "bash -c \"source ~/.bashrc; roslaunch deformable_manipulations_fabric test.launch; exec bash\"";
sleep 1s;

gnome-terminal --tab --title="Keyboard Teleop" --command "bash -c \"source ~/.bashrc; roslaunch deformable_manipulations_fabric keyboard.launch; exec bash\"";
sleep 1s;






