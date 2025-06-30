#!/bin/bash

#export GZ_SIM_RESOURCE_PATH=$HOME/Documents/ap-sim/ardupilot_gazebo/models:$HOME/Documents/ap-sim/ardupilot_gazebo/worlds:
#export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/Documents/ap-sim/ardupilot_gazebo/build:

#export GZ_SIM_RESOURCE_PATH=$HOME/Documents/ap-sim/ac-dev/ap-gz/models:$HOME/Documents/ap-sim/ac-dev/ap-gz/worlds:
#export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/Documents/ap-sim/ac-dev/ap-gz/build:

export GZ_SIM_RESOURCE_PATH=$(pwd)/ap-gz/models:$(pwd)/ap-gz/worlds:
export GZ_SIM_SYSTEM_PLUGIN_PATH=$(pwd)/ap-gz/build:


