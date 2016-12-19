#!/bin/bash
sudo cp libs/process.hpp /usr/include/boost
sudo cp -r libs/process /usr/include/boost
cd
echo "export GAZEBO_PLUGIN_PATH=\$GAZEBO_PLUGIN_PATH:~/catkin_ws/src/minho_team_simulation_tools/plugins" >> .profile
echo "export GAZEBO_PLUGIN_PATH=\$GAZEBO_PLUGIN_PATH:~/catkin_ws/src/minho_team_simulation_tools/plugins" >> .bashrc
echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:~/catkin_ws/src/minho_team_simulation_tools/models" >> .profile
echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:~/catkin_ws/src/minho_team_simulation_tools/models" >> .bashrc
echo "export GAZEBO_RESOURCE_PATH=\$GAZEBO_RESOURCE_PATH:~/catkin_ws/src/minho_team_simulation_tools/worlds" >> .profile
echo "export GAZEBO_RESOURCE_PATH=\$GAZEBO_RESOURCE_PATH:~/catkin_ws/src/minho_team_simulation_tools/worlds" >> .bashrc
echo "export GAZEBO_RESOURCE_PATH=\$GAZEBO_RESOURCE_PATH:/usr/share/gazebo-7" >> .profile
echo "export GAZEBO_RESOURCE_PATH=\$GAZEBO_RESOURCE_PATH:/usr/share/gazebo-7" >> .bashrc
. .profile
echo "Installation of libs and paths completed!"
