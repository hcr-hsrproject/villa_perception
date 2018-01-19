
#ifndef VILLAOCTOMAP_H
#define VILLAOCTOMAP_H

#include <pcl/point_types.h>
#include <pcl/conversions.h>

#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>

#include <villa_octomap_server/Node.h>

namespace villa_octomap {
    class VillaOctomap {
        public:
            typedef pcl::PointXYZ PCLPoint;
            typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;
            typedef octomap::OcTree OcTreeT;
            typedef octomap::point3d Point;
            typedef std::vector<Node> Nodes;

            VillaOctomap();
            VillaOctomap(double maxRange, double res, 
                         double probHit, double probMiss,
                         double thresMin, double thresMax,
                         double occupancyMinZ, double occupancyMaxZ,
                         bool filterSpeckles, bool compressMap);
            virtual ~VillaOctomap();

            octomap::OcTree* getOctree();

            bool insertPointCloud(const Point& sensorOrigin, const PCLPointCloud& pc);
            bool clearBBX(const Point& min, const Point& max);
            bool reset();

            void getNodes(Nodes& free, Nodes& occupied);

        protected:
            OcTreeT* m_octree;
            octomap::KeyRay m_keyRay;  // temp storage for ray casting
            octomap::OcTreeKey m_updateBBXMin;
            octomap::OcTreeKey m_updateBBXMax;

            double m_maxRange;
            double m_res;

            unsigned m_treeDepth;
            unsigned m_maxTreeDepth;

            double m_occupancyMinZ;
            double m_occupancyMaxZ;

            bool m_filterSpeckles;
            bool m_compressMap;

            bool isSpeckleNode(const octomap::OcTreeKey& key);

            inline static void updateMinKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& min) {
                for (unsigned i = 0; i < 3; ++i)
                    min[i] = std::min(in[i], min[i]);
            };

            inline static void updateMaxKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& max) {
                for (unsigned i = 0; i < 3; ++i)
                    max[i] = std::max(in[i], max[i]);
            };
    };
}

#endif

