
#include <ros/ros.h>
#include <villa_octomap_server/ObjectOctomapServer.h>

#define USAGE "\nUSAGE: octomap_server <map.[bt|ot]>\n" \
        "  map.bt: inital octomap 3D map file to read\n"

using namespace villa_octomap_server;

int main(int argc, char** argv){
  ros::init(argc, argv, "octomap_server");
  const ros::NodeHandle& private_nh = ros::NodeHandle("~");
  std::string mapFilename(""), mapFilenameParam("");

  if (argc > 2 || (argc == 2 && std::string(argv[1]) == "-h")){
    ROS_ERROR("%s", USAGE);
    exit(-1);
  }

  ObjectOctomapServer server;
  ros::spinOnce();

  try{
    ros::spin();
  }catch(std::runtime_error& e){
    ROS_ERROR("octomap_server exception: %s", e.what());
    return -1;
  }

  return 0;
}
