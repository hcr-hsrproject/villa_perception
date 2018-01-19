# Villa Perception [![Build Status](https://travis-ci.com/AustinVillaatHome/villa_perception.svg?token=1o9Avy4mGixFuRg9knBP&branch=master)](https://travis-ci.com/AustinVillaatHome/villa_perception)

* 3D object extraction with GrabCut
* Age and gender identification with RudeCarnie
* Face recognition
* Object recognition with YOLO
* Plane detection

## Usage

See package readmes for usage.

## Compiling

If you have a GPU with CUDA configured, set the `GPU_ENABLED` environment variable to true in your bashrc

    export GPU_ENABLED=true
    
Regardless, you can compile as usual with

    catkin build

## Installing RTAB map
### Make rtabmap_ros visible
````
cd ~/robocup_home_ws/src/villa_perception/rtabmap_ros
cp ignore_pacakge.xml package.xml
````
### Navigate to robocup_home_ws and Install the dependencies
````
cd ~/robocup_home_ws
rosdep install --from-paths src/villa_perception/rtabmap_ros --ignore-src --rosdistro=${ROS_DISTRO} -y
````

### Run RTAB with the robot
````
roslaunch rtabmap_ros hsrb_rtabmap.launch rtabmap_args:="--delete_db_on_start" rviz:=true rtabmapviz:=false
````

### Run with the kinect
````
roslaunch freenect_launch freenect.launch depth_registration:=true
roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start" rviz:=true rtabmapviz:=false
````


