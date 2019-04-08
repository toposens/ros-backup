#!/bin/bash

# Inspired by Gitlab CI script for ROS by @VictorLamoine
# https://gitlab.com/VictorLamoine/ros_gitlab_ci/blob/master/gitlab-ros.bash


# Display system information
#---------------------------
echo "##############################################"
uname -a
lsb_release -a
gcc --version
echo "CXXFLAGS = ${CXXFLAGS}"
cmake --version
echo "##############################################"


# Setup ROS
#----------
export CCACHE_DIR=${CI_PROJECT_DIR}/ccache
source /opt/ros/$(ls /opt/ros/)/setup.bash


# Prepare workspace
#------------------
PROJECT_NAME=$(basename ${CI_PROJECT_DIR})
rm -rf src && mkdir -p src/${PROJECT_NAME}
mv $(ls -a | grep -Ev '^.$|^..$|^ccache$|^src$') src/${PROJECT_NAME}


# Initialize git submodules
#--------------------------
cd ${CI_PROJECT_DIR}/src/${PROJECT_NAME}
git submodule update --init --recursive
cd ${CI_PROJECT_DIR}
