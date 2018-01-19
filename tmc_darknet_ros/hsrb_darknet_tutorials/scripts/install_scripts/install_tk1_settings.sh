#!/bin/bash
# install CUDA6.5
CUDA_PATH=http://developer.download.nvidia.com/embedded/L4T/r21_Release_v3.0/cuda-repo-l4t-r21.3-6-5-prod_6.5-42_armhf.deb
wget $CUDA_PATH
sudo dpkg --force-all -i ./cuda-repo-l4t-r21.3-6-5-prod_6.5-42_armhf.deb
rm cuda-repo-l4t-r21.3-6-5-prod_6.5-42_armhf.deb
sudo apt-get -y --force-yes install cuda-toolkit-6-5
# install necessary ROS packages
sudo apt-get -y --force-yes install ros-indigo-image-transport ros-indigo-cv-bridge
