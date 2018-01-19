# villa_3d_object_extract

YOLO 2D output to 3D Point cloud

This repo has a ROSified version of YOLO which extracts 2D bounding boxes from a 2D image. We take these bounding boxes and perform a simple segmentation to extract the corresponding object point cloud by returning the closest cluster of the 3d points within the 2D bounding box.


### HSRB on board testing

#### Launch YOLO on the TK-1

Follow the instructions in `villa_yolo` to install and launch a model on TK1.

#### Launch the detector node on the main HSRB computer
While we can launch the detector node anyhwere, it is best to launch this on the robot:
    
    ssh zilker@hsrb.local
    rosrun villa_3d_object_extract tmc_yolo_3D_extractor
    

For cluster based extraction:

    rosrun villa_3d_object_extract tmc_cluster_based_yolo_3D_extractor


To visualize, run

    roslaunch villa_3d_object_extract visualize.launch


###  Kinect v1 Testing
You can also test the segmentation capability by plugging in a Kinect v1 On your computer.

Install Kinect v1 Drivers if you don't have it

    sudo apt-get install libfreenect-dev
    sudo apt-get install ros-indigo-freenect-launch

Launch the kinect:

    roslaunch freenect_launch freenect.launch 

Launch RViz with

    roslaunch villa_3d_object_extract visualize.launch kinect:=true

Rviz should launch and you should see an RGB-D point cloud of the world. Then on a new terminal:

    rosrun villa_3d_object_extract tmc_yolo_3D_extractor_kinect1

In Rviz, if an object is detected (see below for types objects that can be detected), green points would be overlayed on the object and a purple bounding box will surround the objects

If you are using the CPU to perform classification, it will take around 10-20 seconds to classify an image.

