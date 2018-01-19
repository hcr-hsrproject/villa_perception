#!/bin/sh
PACKAGE_PATH=`rospack find hsrb_darknet_tutorials`
if [ $# -ge 1 ]; then
    DOWNLOAD_PATH=$1
elif [ -z $ROS_HOME ]; then
    DOWNLOAD_PATH=~/.ros/darknet
else
    DOWNLOAD_PATH=$ROS_HOME/darknet
fi
echo Download to $DOWNLOAD_PATH
mkdir -p $DOWNLOAD_PATH
(cd $DOWNLOAD_PATH; wget https://raw.githubusercontent.com/pjreddie/darknet/master/data/coco.names --no-check-certificate)
(cd $DOWNLOAD_PATH; wget https://raw.githubusercontent.com/pjreddie/darknet/master/cfg/yolo.cfg --no-check-certificate)
(cd $DOWNLOAD_PATH; wget http://pjreddie.com/media/files/yolo.weights --no-check-certificate)
