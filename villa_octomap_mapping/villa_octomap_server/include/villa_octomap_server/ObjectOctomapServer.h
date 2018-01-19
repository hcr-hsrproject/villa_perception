
#ifndef OBJECT_OCTOMAP_SERVER_H
#define OCJECT_OCTOMAP_SERVER_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/ColorRGBA.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <dynamic_reconfigure/server.h>
#include <villa_octomap_server/OctomapServerConfig.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>

#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>

#include <villa_perception_msgs/SegmentedPointCloud.h>
#include <villa_perception_msgs/SegmentedMarkerArray.h>

#include <villa_octomap_server/VillaOctomap.h>
#include <villa_octomap_server/Node.h>

//#include <unorderd_map>

namespace villa_octomap_server {
    class ObjectOctomapServer {
        public:
            typedef pcl::PointXYZ PCLPoint;
            typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;

            typedef octomap_msgs::GetOctomap OctomapSrv;
            typedef octomap_msgs::BoundingBoxQuery BBXSrv;

            typedef octomap::OcTree OcTreeT;
            typedef octomap::point3d Point;
            typedef std::vector<Node> Nodes;

            typedef villa_octomap::VillaOctomap VillaOctomap;
            typedef std::map<std::string, VillaOctomap> Objects;

            ObjectOctomapServer(ros::NodeHandle private_nh_ = ros::NodeHandle("~"));
            virtual ~ObjectOctomapServer();

            //bool getSceneSrv();
            //bool getObjectSrv(); // returns cloud for single object
            //bool getBBXSrv(BBXSrv::Request& req, BBXSrv::Response& resp);
            //bool clearBBXSrv(BBXSrv::Request& req, BBXSrv::Response& resp);
            //bool resetSrv(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);
            
            void insertSegmentedCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
            //void insertSegmentedCloudCallback(const villa_perception_msgs::SegmentedPointCloud::ConstPtr& segCloud); 
            void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud, VillaOctomap& map);
            void publishAll(const ros::Time& rostime, VillaOctomap& map);
            void publishAll();

        protected:
            //void ocTreeToPointCloud(octomap::OcTree octree, pcl::PointCloud<pcl::PointXYZ>& pc);

            void filterGroundPlane(const PCLPointCloud& pc, PCLPointCloud& ground, PCLPointCloud& nonground);
            void insertScan(const tf::Point& sensorOrigin, const PCLPointCloud& pc, VillaOctomap& map);

            visualization_msgs::MarkerArray getFreeMarkerArray(const ros::Time& rostime, VillaOctomap& map, Nodes& free);
            visualization_msgs::MarkerArray getOccupiedMarkerArray(const ros::Time& rostime, VillaOctomap& map, Nodes& occupied);
            sensor_msgs::PointCloud2 getCentersPointCloud(const ros::Time& rostime, Nodes& occupied);
            void fillPointsAndMarkers(const ros::Time& rostime, VillaOctomap& map, 
                                      sensor_msgs::PointCloud2& centers,  
                                      visualization_msgs::MarkerArray& occupiedMarkers, 
                                      visualization_msgs::MarkerArray& freeMarkers);

            static std_msgs::ColorRGBA heightMapColor(double h);

            VillaOctomap* scene;
            Objects objects;

            ros::NodeHandle m_nh;

            ros::Publisher  m_markerPub, 
                            //m_binaryMapPub, 
                            //m_fullMapPub, 
                            m_pointCloudPub,
                            m_fmarkerPub;

            message_filters::Subscriber<sensor_msgs::PointCloud2>* m_pointCloudSub;
            tf::MessageFilter<sensor_msgs::PointCloud2>* m_tfPointCloudSub;
            /*
            ros::ServiceServer m_octomapBinaryService, 
                               m_octomapFullService, 
                               m_clearBBXService, 
                               m_resetService;
            */
            tf::TransformListener m_tfListener;

            double m_probHit, 
                   m_probMiss, 
                   m_thresMin,
                   m_thresMax;

            double m_maxRange;
            std::string m_worldFrameId; // the map frame
            std::string m_baseFrameId; // base of the robot for ground plane filtering
            bool m_useHeightMap;
            std_msgs::ColorRGBA m_color;
            std_msgs::ColorRGBA m_colorFree;
            double m_colorFactor;

            bool m_latchedTopics;
            bool m_publishFreeSpace;

            double m_res;

            double m_pointcloudMinX;
            double m_pointcloudMaxX;
            double m_pointcloudMinY;
            double m_pointcloudMaxY;
            double m_pointcloudMinZ;
            double m_pointcloudMaxZ;
            double m_occupancyMinZ;
            double m_occupancyMaxZ;
            double m_minSizeX;
            double m_minSizeY;
            bool m_filterSpeckles;

            bool m_filterGroundPlane;
            double m_groundFilterDistance;
            double m_groundFilterAngle;
            double m_groundFilterPlaneDistance;

            bool m_compressMap;
    };
}

#endif

