#!/bin/sh
PACKAGE_PATH=`rospack find hsrb_darknet_tutorials`
BBOX_LABEL_TOOL_COMMIT_VERSION=090f1dcf4ed0b2e5744656d32731596ce9685197
DNN_TOOLS_COMMIT_VERSION=b26c388fdcef1d8a35c4be3b3ca6b421eee5a001

if [ $# -ge 1 ]; then
    DOWNLOAD_PATH=$1
else
    DOWNLOAD_PATH=$PACKAGE_PATH/scripts
fi
mkdir -p $DOWNLOAD_PATH

cancel_handler() {
    rm -rf $MY_TMP_DIR
    exit
}

MY_TMP_DIR=`mktemp -d`
trap "rm -rf $MY_TMP_DIR" EXIT
trap cancel_handler SIGINT
# install necessary package
sudo apt-get -y install python-pil.imagetk
# install annotation tool
(cd $MY_TMP_DIR; git clone https://github.com/puzzledqs/BBox-Label-Tool.git)
(cd $MY_TMP_DIR/BBox-Label-Tool; git checkout $BBOX_LABEL_COMMIT_VERSION)
(cd $MY_TMP_DIR/BBox-Label-Tool; patch -u main.py < $PACKAGE_PATH/config/BBox-Label-Tool.patch)
(cd $DOWNLOAD_PATH; cp $MY_TMP_DIR/BBox-Label-Tool/main.py ./BBox-Label-Tool)
(cd $DOWNLOAD_PATH; chmod a+x BBox-Label-Tool)
# install inflate images tool
(cd $MY_TMP_DIR; git clone https://github.com/bohemian916/deeplearning_tool.git)
(cd $MY_TMP_DIR/deeplearning_tool; git checkout $DNN_TOOLS_COMMIT_VERSION)
(cd $MY_TMP_DIR/deeplearning_tool; patch -u increase_picture.py < $PACKAGE_PATH/config/increase_picture.patch)
(cd $DOWNLOAD_PATH; cp $MY_TMP_DIR/deeplearning_tool/increase_picture.py ./inflate_images)
(cd $DOWNLOAD_PATH; chmod a+x inflate_images)
