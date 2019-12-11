#!/bin/bash

# Build and test the tf_service package from source.
# Intended for use in Docker builds only.

set -e

test -d catkin_ws/src/tf_service

source "/opt/ros/${ROS_DISTRO}/setup.bash"
apt-get update

pushd catkin_ws/
pushd src/
catkin_init_workspace
rosdep install -y --from-paths .
popd
catkin_make
catkin_make run_tests
catkin_test_results --all --verbose
popd

# https://docs.docker.com/develop/develop-images/dockerfile_best-practices/
rm -rf /var/lib/apt/lists/*
