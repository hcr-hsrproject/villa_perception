
#include <villa_octomap_server/ObjectOctomapServer.h>

using namespace octomap;

namespace villa_octomap_server {
    ObjectOctomapServer::ObjectOctomapServer(ros::NodeHandle private_nh_)
    : m_nh(),
      m_pointCloudSub(NULL),
      m_tfPointCloudSub(NULL),
      m_maxRange(-1.0),
      m_worldFrameId("/map"), m_baseFrameId("base_footprint"),
      m_useHeightMap(true),
      m_colorFactor(0.8),
      m_latchedTopics(true),
      m_publishFreeSpace(false),
      m_res(0.05),
      m_pointcloudMinX(-std::numeric_limits<double>::max()),
      m_pointcloudMaxX(std::numeric_limits<double>::max()),
      m_pointcloudMinY(-std::numeric_limits<double>::max()),
      m_pointcloudMaxY(std::numeric_limits<double>::max()),
      m_pointcloudMinZ(-std::numeric_limits<double>::max()),
      m_pointcloudMaxZ(std::numeric_limits<double>::max()),
      m_occupancyMinZ(-std::numeric_limits<double>::max()),
      m_occupancyMaxZ(std::numeric_limits<double>::max()),
      m_minSizeX(0.0), m_minSizeY(0.0),
      m_filterSpeckles(false), m_filterGroundPlane(false),
      m_groundFilterDistance(0.04), m_groundFilterAngle(0.15), m_groundFilterPlaneDistance(0.07),
      m_compressMap(true)
    {
        double probHit, probMiss, thresMin, thresMax;

        ros::NodeHandle private_nh(private_nh_);
        private_nh.param("frame_id", m_worldFrameId, m_worldFrameId);
        private_nh.param("base_frame_id", m_baseFrameId, m_baseFrameId);
        private_nh.param("height_map", m_useHeightMap, m_useHeightMap);
        private_nh.param("color_factor", m_colorFactor, m_colorFactor);

        private_nh.param("pointcloud_min_x", m_pointcloudMinX,m_pointcloudMinX);
        private_nh.param("pointcloud_max_x", m_pointcloudMaxX,m_pointcloudMaxX);
        private_nh.param("pointcloud_min_y", m_pointcloudMinY,m_pointcloudMinY);
        private_nh.param("pointcloud_max_y", m_pointcloudMaxY,m_pointcloudMaxY);
        private_nh.param("pointcloud_min_z", m_pointcloudMinZ,m_pointcloudMinZ);
        private_nh.param("pointcloud_max_z", m_pointcloudMaxZ,m_pointcloudMaxZ);
        private_nh.param("occupancy_min_z", m_occupancyMinZ,m_occupancyMinZ);
        private_nh.param("occupancy_max_z", m_occupancyMaxZ,m_occupancyMaxZ);
        private_nh.param("min_x_size", m_minSizeX,m_minSizeX);
        private_nh.param("min_y_size", m_minSizeY,m_minSizeY);

        private_nh.param("filter_speckles", m_filterSpeckles, m_filterSpeckles);
        private_nh.param("filter_ground", m_filterGroundPlane, m_filterGroundPlane);
        // distance of points from plane for RANSAC
        private_nh.param("ground_filter/distance", m_groundFilterDistance, m_groundFilterDistance);
        // angular derivation of found plane:
        private_nh.param("ground_filter/angle", m_groundFilterAngle, m_groundFilterAngle);
        // distance of found plane from z=0 to be detected as ground (e.g. to exclude tables)
        private_nh.param("ground_filter/plane_distance", m_groundFilterPlaneDistance, m_groundFilterPlaneDistance);

        private_nh.param("sensor_model/max_range", m_maxRange, m_maxRange);

        private_nh.param("resolution", m_res, m_res);
        private_nh.param("sensor_model/hit", m_probHit, 0.7);
        private_nh.param("sensor_model/miss", m_probMiss, 0.4);
        private_nh.param("sensor_model/min", m_thresMin, 0.12);
        private_nh.param("sensor_model/max", m_thresMax, 0.97);
        private_nh.param("compress_map", m_compressMap, m_compressMap);

        if (m_filterGroundPlane && (m_pointcloudMinZ > 0.0 || m_pointcloudMaxZ < 0.0)){
            ROS_WARN_STREAM("You enabled ground filtering but incoming pointclouds will be pre-filtered in ["
                            << m_pointcloudMinZ <<", "<< m_pointcloudMaxZ << "], excluding the ground level z=0. "
                            << "This will not work.");
        }

        double r, g, b, a;
        private_nh.param("color/r", r, 0.0);
        private_nh.param("color/g", g, 0.0);
        private_nh.param("color/b", b, 1.0);
        private_nh.param("color/a", a, 1.0);
        m_color.r = r;
        m_color.g = g;
        m_color.b = b;
        m_color.a = a;

        private_nh.param("color_free/r", r, 0.0);
        private_nh.param("color_free/g", g, 1.0);
        private_nh.param("color_free/b", b, 0.0);
        private_nh.param("color_free/a", a, 1.0);
        m_colorFree.r = r;
        m_colorFree.g = g;
        m_colorFree.b = b;
        m_colorFree.a = a;

        private_nh.param("publish_free_space", m_publishFreeSpace, m_publishFreeSpace);

        private_nh.param("latch", m_latchedTopics, m_latchedTopics);
        if (m_latchedTopics)
            ROS_INFO("Publishing latched (single publish will take longer, all topics are prepared)");
        else
            ROS_INFO("Publishing non-latched (topics are only prepared as needed, will only be re-published on map change");

        m_markerPub = m_nh.advertise<visualization_msgs::MarkerArray>("occupied_cells_vis_array", 1, m_latchedTopics);
        //m_binaryMapPub = m_nh.advertise<Octomap>("octomap_binary", 1, m_latchedTopics);
        //m_fullMapPub = m_nh.advertise<Octomap>("octomap_full", 1, m_latchedTopics);
        m_pointCloudPub = m_nh.advertise<sensor_msgs::PointCloud2>("octomap_point_cloud_centers", 1, m_latchedTopics);
        m_fmarkerPub = m_nh.advertise<visualization_msgs::MarkerArray>("free_cells_vis_array", 1, m_latchedTopics);

        m_pointCloudSub = new message_filters::Subscriber<sensor_msgs::PointCloud2> (m_nh, "cloud_in", 5);
        m_tfPointCloudSub = new tf::MessageFilter<sensor_msgs::PointCloud2> (*m_pointCloudSub, m_tfListener, m_worldFrameId, 5);
        m_tfPointCloudSub->registerCallback(boost::bind(&ObjectOctomapServer::insertSegmentedCloudCallback, this, _1));

        //m_octomapBinaryService = m_nh.advertiseService("octomap_binary", &OctomapServer::octomapBinarySrv, this);
        //m_octomapFullService = m_nh.advertiseService("octomap_full", &OctomapServer::octomapFullSrv, this);
        //m_clearBBXService = private_nh.advertiseService("clear_bbx", &OctomapServer::clearBBXSrv, this);
        //m_resetService = private_nh.advertiseService("reset", &OctomapServer::resetSrv, this);

        scene = new VillaOctomap(m_maxRange, m_res, 
                                 m_probHit, m_probMiss, 
                                 m_thresMin, m_thresMax,
                                 m_occupancyMinZ, m_occupancyMaxZ, 
                                 m_filterSpeckles, m_compressMap);
    }

    ObjectOctomapServer::~ObjectOctomapServer() {
        if (m_tfPointCloudSub){
            delete m_tfPointCloudSub;
            m_tfPointCloudSub = NULL;
        }

        if (m_pointCloudSub){
            delete m_pointCloudSub;
            m_pointCloudSub = NULL;
        }

        if (scene) {
            delete scene;
            scene = NULL;
        }
/*
        if (objects) {
            delete objects;
            objects = NULL;
        }
*/
    }

    void ObjectOctomapServer::insertSegmentedCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud) {
        insertCloudCallback(cloud, *scene);
        publishAll(cloud->header.stamp, *scene);
/*
        insertCloudCallback(*segCloud->scene, *scene);
        for (unsigned i= 0; i < segCloud->object_names.size(); ++i) {
            // if object does not exist
            if (objects.find(segCloud->object_names[i]) != objects.end()) {
                objects[segCloud->object_names[i]]] = VillaOctomap(m_maxRange, m_res, 
                                                                   m_probHit, m_probMiss, 
                                                                   m_thresMin, m_thresMax,
                                                                   m_occupancyMinZ, m_occupancyMaxZ, 
                                                                   m_filterSpeckles, m_compressMap);
            }

            insertCloudCallback(*segCloud->objects[i], objects[segCloud->object_names[i]]]);
        }
    
        publishAll();
*/
    }

    void ObjectOctomapServer::insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud, VillaOctomap& map) {
        ros::WallTime startTime = ros::WallTime::now();

        // ground filtering in base frame
        PCLPointCloud pc; // input cloud for filtering and ground-detection
        PCLPointCloud pc_ground; // segmented ground plane
        PCLPointCloud pc_nonground; // everything else

        pcl::fromROSMsg(*cloud, pc);

        tf::StampedTransform sensorToWorldTf;
        try {
            m_tfListener.lookupTransform(m_worldFrameId, cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);
        } catch(tf::TransformException& ex) {
            ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
            return;
        }

        Eigen::Matrix4f sensorToWorld;
        pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);

        // set up filter for height range, also removes NANs:
        pcl::PassThrough<pcl::PointXYZ> pass_x;
        pcl::PassThrough<pcl::PointXYZ> pass_y;
        pcl::PassThrough<pcl::PointXYZ> pass_z;

        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(m_pointcloudMinX, m_pointcloudMaxX);

        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(m_pointcloudMinY, m_pointcloudMaxY);

        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(m_pointcloudMinZ, m_pointcloudMaxZ);

        if (m_filterGroundPlane) {
            tf::StampedTransform sensorToBaseTf, baseToWorldTf;
            try {
                m_tfListener.waitForTransform(m_baseFrameId, cloud->header.frame_id, cloud->header.stamp, ros::Duration(0.2));
                m_tfListener.lookupTransform(m_baseFrameId, cloud->header.frame_id, cloud->header.stamp, sensorToBaseTf);
                m_tfListener.lookupTransform(m_worldFrameId, m_baseFrameId, cloud->header.stamp, baseToWorldTf);
            } catch(tf::TransformException& ex) {
                ROS_ERROR_STREAM("Transform error for ground plane filter: " << ex.what() << ", quitting callback.\n"
                                 "You need to set the base_frame_id or disable filter_ground.");
            }

            Eigen::Matrix4f sensorToBase, baseToWorld;
            pcl_ros::transformAsMatrix(sensorToBaseTf, sensorToBase);
            pcl_ros::transformAsMatrix(baseToWorldTf, baseToWorld);

            // transform pointcloud from sensor frame to fixed robot frame
            pcl::transformPointCloud(pc, pc, sensorToBase);
            pass_x.setInputCloud(pc.makeShared());
            pass_x.filter(pc);
            pass_y.setInputCloud(pc.makeShared());
            pass_y.filter(pc);
            pass_z.setInputCloud(pc.makeShared());
            pass_z.filter(pc);

            filterGroundPlane(pc, pc_ground, pc_nonground);

            // transform clouds to world frame for insertion
            pcl::transformPointCloud(pc_ground, pc_ground, baseToWorld);
            pcl::transformPointCloud(pc_nonground, pc_nonground, baseToWorld);
        } else {
            // directly transform to map frame:
            pcl::transformPointCloud(pc, pc, sensorToWorld);

            // just filter height range:
            pass_x.setInputCloud(pc.makeShared());
            pass_x.filter(pc);
            pass_y.setInputCloud(pc.makeShared());
            pass_y.filter(pc);
            pass_z.setInputCloud(pc.makeShared());
            pass_z.filter(pc);

            pc_nonground = pc;
            // pc_nonground is empty without ground segmentation
            pc_ground.header = pc.header;
            pc_nonground.header = pc.header;
        }

        insertScan(sensorToWorldTf.getOrigin(), pc_nonground, map);

        double total_elapsed = (ros::WallTime::now() - startTime).toSec();
        ROS_DEBUG("Pointcloud insertion in ObjectOctomapServer done (%zu+%zu pts (ground/nonground), %f sec)", pc_ground.size(), pc_nonground.size(), total_elapsed);
    }

    void ObjectOctomapServer::publishAll(const ros::Time& rostime, VillaOctomap& map) {
        ros::Time startTime = ros::Time::now();

        octomap::OcTree* octree = map.getOctree();
        size_t octomapSize = octree->size();

        if (octomapSize <= 1){
            ROS_WARN("Nothing to publish, octree is empty");
            return;
        }

        bool publishFreeMarkerArray = m_publishFreeSpace && (m_latchedTopics || m_fmarkerPub.getNumSubscribers() > 0);
        bool publishMarkerArray = (m_latchedTopics || m_markerPub.getNumSubscribers() > 0);
        bool publishPointCloud = (m_latchedTopics || m_pointCloudPub.getNumSubscribers() > 0);

        visualization_msgs::MarkerArray occupiedMarkers;
        visualization_msgs::MarkerArray freeMarkers;
        sensor_msgs::PointCloud2 centers;

        fillPointsAndMarkers(rostime, map, centers, occupiedMarkers, freeMarkers);

        if (publishFreeMarkerArray)
            m_fmarkerPub.publish(freeMarkers);

        if (publishMarkerArray)
            m_markerPub.publish(occupiedMarkers);

        if (publishPointCloud)
            m_pointCloudPub.publish(centers);

        double total_elapsed = (ros::Time::now() - startTime).toSec();
        ROS_DEBUG("Map publishing in ObjectOctomapServer took %f sec", total_elapsed);
    }

    void ObjectOctomapServer::publishAll() {
        ros::Time startTime = ros::Time::now();

        bool publishFreeMarkerArray = m_publishFreeSpace && (m_latchedTopics || m_fmarkerPub.getNumSubscribers() > 0);
        bool publishMarkerArray = (m_latchedTopics || m_markerPub.getNumSubscribers() > 0);
        bool publishPointCloud = (m_latchedTopics || m_pointCloudPub.getNumSubscribers() > 0);

        // segmented messages
        villa_perception_msgs::SegmentedMarkerArray segOccupied;
        villa_perception_msgs::SegmentedMarkerArray segFree;
        villa_perception_msgs::SegmentedPointCloud segCenters;

        // scene messages
        visualization_msgs::MarkerArray occupiedMarkers;
        visualization_msgs::MarkerArray freeMarkers;
        sensor_msgs::PointCloud2 centers;

        fillPointsAndMarkers(startTime, *scene, centers, occupiedMarkers, freeMarkers);

        segOccupied.scene = occupiedMarkers;
        segFree.scene = freeMarkers;
        segCenters.scene = centers;

        // object messages
        for (Objects::iterator it = objects.begin(); it != objects.end(); ++it) {
            visualization_msgs::MarkerArray occupiedMarkers;
            visualization_msgs::MarkerArray freeMarkers;
            sensor_msgs::PointCloud2 centers;

            fillPointsAndMarkers(startTime, it->second, centers, occupiedMarkers, freeMarkers);

            segOccupied.object_names.push_back(it->first);
            segOccupied.objects.push_back(occupiedMarkers);

            segFree.object_names.push_back(it->first);
            segFree.objects.push_back(freeMarkers);

            segCenters.object_names.push_back(it->first);
            segCenters.objects.push_back(centers);
        }

        /*
        if (publishFreeMarkerArray)
            m_fmarkerPub.publish(freeMarkers);

        if (publishMarkerArray)
            m_markerPub.publish(occupiedMarkers);

        if (publishPointCloud)
            m_pointCloudPub.publish(centers);
        */

        double total_elapsed = (ros::Time::now() - startTime).toSec();
        ROS_DEBUG("Map publishing in ObjectOctomapServer took %f sec", total_elapsed);
    }

    void ObjectOctomapServer::filterGroundPlane(const PCLPointCloud& pc, PCLPointCloud& ground, PCLPointCloud& nonground) {
        ground.header = pc.header;
        nonground.header = pc.header;

        if (pc.size() < 50) {
            ROS_WARN("Pointcloud in OctomapServer too small, skipping ground plane extraction");
            nonground = pc;
        } else {
            // plane detection for ground plane removal:
            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

            // Create the segmentation object and set up:
            pcl::SACSegmentation<PCLPoint> seg;
            seg.setOptimizeCoefficients (true);
            // TODO: maybe a filtering based on the surface normals might be more robust / accurate?
            seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setMaxIterations(200);
            seg.setDistanceThreshold (m_groundFilterDistance);
            seg.setAxis(Eigen::Vector3f(0,0,1));
            seg.setEpsAngle(m_groundFilterAngle);

            PCLPointCloud cloud_filtered(pc);
            // Create the filtering object
            pcl::ExtractIndices<PCLPoint> extract;
            bool groundPlaneFound = false;

            while(cloud_filtered.size() > 10 && !groundPlaneFound) {
                seg.setInputCloud(cloud_filtered.makeShared());
                seg.segment (*inliers, *coefficients);
                if (inliers->indices.size () == 0) {
                    ROS_INFO("PCL segmentation did not find any plane.");
                    break;
                }

                extract.setInputCloud(cloud_filtered.makeShared());
                extract.setIndices(inliers);

                if (std::abs(coefficients->values.at(3)) < m_groundFilterPlaneDistance) {
                    ROS_DEBUG("Ground plane found: %zu/%zu inliers. Coeff: %f %f %f %f", inliers->indices.size(), cloud_filtered.size(), coefficients->values.at(0), coefficients->values.at(1), coefficients->values.at(2), coefficients->values.at(3));
                    extract.setNegative (false);
                    extract.filter (ground);

                    // remove ground points from full pointcloud:
                    // workaround for PCL bug:
                    if(inliers->indices.size() != cloud_filtered.size()) {
                        extract.setNegative(true);
                        PCLPointCloud cloud_out;
                        extract.filter(cloud_out);
                        nonground += cloud_out;
                        cloud_filtered = cloud_out;
                    }

                    groundPlaneFound = true;
                } else {
                    ROS_DEBUG("Horizontal plane (not ground) found: %zu/%zu inliers. Coeff: %f %f %f %f", inliers->indices.size(), cloud_filtered.size(), coefficients->values.at(0), coefficients->values.at(1), coefficients->values.at(2), coefficients->values.at(3));
                    pcl::PointCloud<PCLPoint> cloud_out;
                    extract.setNegative (false);
                    extract.filter(cloud_out);
                    nonground +=cloud_out;

                    // remove current plane from scan for next iteration:
                    // workaround for PCL bug:
                    if(inliers->indices.size() != cloud_filtered.size()){
                        extract.setNegative(true);
                        cloud_out.points.clear();
                        extract.filter(cloud_out);
                        cloud_filtered = cloud_out;
                    } else {
                        cloud_filtered.points.clear();
                    }
                }
            }
            // TODO: also do this if overall starting pointcloud too small?
            if (!groundPlaneFound){ // no plane found or remaining points too small
                ROS_WARN("No ground plane found in scan");

                // do a rough fitlering on height to prevent spurious obstacles
                pcl::PassThrough<PCLPoint> second_pass;
                second_pass.setFilterFieldName("z");
                second_pass.setFilterLimits(-m_groundFilterPlaneDistance, m_groundFilterPlaneDistance);
                second_pass.setInputCloud(pc.makeShared());
                second_pass.filter(ground);

                second_pass.setFilterLimitsNegative (true);
                second_pass.filter(nonground);
            }
        }
    }

    void ObjectOctomapServer::insertScan(const tf::Point& sensorOriginTf, const PCLPointCloud& pc, VillaOctomap& map) {
        point3d sensorOrigin = pointTfToOctomap(sensorOriginTf);
        bool inserted = map.insertPointCloud(sensorOrigin, pc);
        if (!inserted)
            ROS_ERROR_STREAM("Could not generate Key");
    }

    visualization_msgs::MarkerArray ObjectOctomapServer::getFreeMarkerArray(const ros::Time& rostime, VillaOctomap& map, Nodes& free) {
        octomap::OcTree* octree = map.getOctree();
        unsigned treeDepth = octree->getTreeDepth();

        // init markers for free space
        visualization_msgs::MarkerArray freeNodesVis;
        // each array stores all cubes of a different size, one for each depth level
        freeNodesVis.markers.resize(treeDepth+1);

        for (Nodes::iterator it = free.begin(); it != free.end(); ++it) {
            unsigned depth = it->getDepth();
            assert(depth < freeNodesVis.markers.size());

            double x = it->getX();
            double y = it->getY();
            double z = it->getZ();

            geometry_msgs::Point cubeCenter;
            cubeCenter.x = x;
            cubeCenter.y = y;
            cubeCenter.z = z;

            freeNodesVis.markers[depth].points.push_back(cubeCenter);
        }

        // finish FreeMarkerArray
        for (unsigned i= 0; i < freeNodesVis.markers.size(); ++i) {
            double size = octree->getNodeSize(i);

            freeNodesVis.markers[i].header.frame_id = m_worldFrameId;
            freeNodesVis.markers[i].header.stamp = rostime;
            freeNodesVis.markers[i].ns = "map";
            freeNodesVis.markers[i].id = i;
            freeNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
            freeNodesVis.markers[i].scale.x = size;
            freeNodesVis.markers[i].scale.y = size;
            freeNodesVis.markers[i].scale.z = size;
            freeNodesVis.markers[i].color = m_colorFree;

            if (freeNodesVis.markers[i].points.size() > 0)
                freeNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
            else
                freeNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
        }

        return freeNodesVis;
    }

    visualization_msgs::MarkerArray ObjectOctomapServer::getOccupiedMarkerArray(const ros::Time& rostime, VillaOctomap& map, Nodes& occupied) {
        octomap::OcTree* octree = map.getOctree();
        unsigned treeDepth = octree->getTreeDepth();

        // init markers:
        visualization_msgs::MarkerArray occupiedNodesVis;
        // each array stores all cubes of a different size, one for each depth level:
        occupiedNodesVis.markers.resize(treeDepth+1);

        for (Nodes::iterator it = occupied.begin(); it != occupied.end(); ++it) {
            double x = it->getX();
            double y = it->getY();
            double z = it->getZ();
            unsigned depth = it->getDepth();
            assert(depth < occupiedNodesVis.markers.size());

            geometry_msgs::Point cubeCenter;
            cubeCenter.x = x;
            cubeCenter.y = y;
            cubeCenter.z = z;

            occupiedNodesVis.markers[depth].points.push_back(cubeCenter);
            if (m_useHeightMap) {
                double minX, minY, minZ, maxX, maxY, maxZ;
                octree->getMetricMin(minX, minY, minZ);
                octree->getMetricMax(maxX, maxY, maxZ);

                double h = (1.0 - std::min(std::max((cubeCenter.z-minZ)/ (maxZ - minZ), 0.0), 1.0)) *m_colorFactor;
                occupiedNodesVis.markers[depth].colors.push_back(heightMapColor(h));
            }
        }

        // finish MarkerArray
        for (unsigned i = 0; i < occupiedNodesVis.markers.size(); ++i) {
            double size = octree->getNodeSize(i);

            occupiedNodesVis.markers[i].header.frame_id = m_worldFrameId;
            occupiedNodesVis.markers[i].header.stamp = rostime;
            occupiedNodesVis.markers[i].ns = "map";
            occupiedNodesVis.markers[i].id = i;
            occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
            occupiedNodesVis.markers[i].scale.x = size;
            occupiedNodesVis.markers[i].scale.y = size;
            occupiedNodesVis.markers[i].scale.z = size;
            occupiedNodesVis.markers[i].color = m_color;

            if (occupiedNodesVis.markers[i].points.size() > 0)
                occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
            else
                occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
        }

        return occupiedNodesVis;
    }

    sensor_msgs::PointCloud2 ObjectOctomapServer::getCentersPointCloud(const ros::Time& rostime, Nodes& occupied) {
        // init pointcloud:
        pcl::PointCloud<PCLPoint> pclCloud;
        sensor_msgs::PointCloud2 cloud;

        for (Nodes::iterator it = occupied.begin(); it != occupied.end(); ++it) {
            double x = it->getX();
            double y = it->getY();
            double z = it->getZ();

            pclCloud.push_back(PCLPoint(x, y, z));
        }

        // finish pointcloud
        pcl::toROSMsg (pclCloud, cloud);
        cloud.header.frame_id = m_worldFrameId;
        cloud.header.stamp = rostime;

        return cloud;
    }
    
    void ObjectOctomapServer::fillPointsAndMarkers(const ros::Time& rostime, VillaOctomap& map, 
                                                   sensor_msgs::PointCloud2& centers, 
                                                   visualization_msgs::MarkerArray& occupiedMarkers,
                                                   visualization_msgs::MarkerArray& freeMarkers) {
        octomap::OcTree* octree = map.getOctree();
        size_t octomapSize = octree->size();
        if (octomapSize <= 1){
            ROS_WARN("Nothing to publish, octree is empty");
            return;
        }

        bool publishFreeMarkerArray = m_publishFreeSpace && (m_latchedTopics || m_fmarkerPub.getNumSubscribers() > 0);
        bool publishMarkerArray = (m_latchedTopics || m_markerPub.getNumSubscribers() > 0);
        bool publishPointCloud = (m_latchedTopics || m_pointCloudPub.getNumSubscribers() > 0);

        Nodes free;
        Nodes occupied;
        
        if (publishFreeMarkerArray || publishMarkerArray || publishPointCloud)
            map.getNodes(free, occupied);

        if (publishFreeMarkerArray)
            freeMarkers = getFreeMarkerArray(rostime, map, free);

        if (publishMarkerArray)
            occupiedMarkers = getOccupiedMarkerArray(rostime, map, occupied);

        if (publishPointCloud)
            centers = getCentersPointCloud(rostime, occupied);
    }

    std_msgs::ColorRGBA ObjectOctomapServer::heightMapColor(double h) {
        std_msgs::ColorRGBA color;
        color.a = 1.0;
        // blend over HSV-values (more colors)

        double s = 1.0;
        double v = 1.0;

        h -= floor(h);
        h *= 6;
        int i;
        double m, n, f;

        i = floor(h);
        f = h - i;
        if (!(i & 1))
        f = 1 - f; // if i is even
        m = v * (1 - s);
        n = v * (1 - s * f);

        switch (i) {
            case 6:
            case 0:
                color.r = v; color.g = n; color.b = m;
                break;
            case 1:
                color.r = n; color.g = v; color.b = m;
                break;
            case 2:
                color.r = m; color.g = v; color.b = n;
                break;
            case 3:
                color.r = m; color.g = n; color.b = v;
                break;
            case 4:
                color.r = n; color.g = m; color.b = v;
                break;
            case 5:
                color.r = v; color.g = m; color.b = n;
                break;
            default:
                color.r = 1; color.g = 0.5; color.b = 0.5;
                break;
        }

            return color;
    }

/*
    void ObjectOctomapServer::fillPointsAndMarkers(const octomap::OcTree& octree, const pcl::PointCloud<pcl::PointXYZ>& centers, const visualization_msgs::MarkerArray& occupied, const visualization_msgs::MarkerArray& free) {
        
    }
*/
}

