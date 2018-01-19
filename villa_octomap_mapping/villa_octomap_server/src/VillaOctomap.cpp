
#include <villa_octomap_server/VillaOctomap.h>

using namespace octomap;

namespace villa_octomap {
    VillaOctomap::VillaOctomap() {}
    
    VillaOctomap::VillaOctomap(double maxRange, double res, 
                               double probHit, double probMiss,
                               double thresMin, double thresMax,
                               double occupancyMinZ, double occupancyMaxZ,
                               bool filterSpeckles, bool compressMap) {
        m_maxRange = maxRange;
        m_res = res;
        m_occupancyMinZ = occupancyMinZ;
        m_occupancyMaxZ = occupancyMaxZ;
        m_filterSpeckles = filterSpeckles;
        m_compressMap = compressMap;

        // initialize octomap object & params
        m_octree = new OcTreeT(m_res);
        m_octree->setProbHit(probHit);
        m_octree->setProbMiss(probMiss);
        m_octree->setClampingThresMin(thresMin);
        m_octree->setClampingThresMax(thresMax);
        m_treeDepth = m_octree->getTreeDepth();
        m_maxTreeDepth = m_treeDepth;
    }

    VillaOctomap::~VillaOctomap() {
        if (m_octree) {
            delete m_octree;
            m_octree = NULL;
        }
    }

    octomap::OcTree* VillaOctomap::getOctree() {
        return m_octree;
    }

    bool VillaOctomap::insertPointCloud(const Point& sensorOrigin, const PCLPointCloud& pc) {
        if (!m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMin) 
            || !m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMax))
            return false;

        KeySet free_cells, occupied_cells;

        // all other points: free on ray, occupied on endpoint:
        for (PCLPointCloud::const_iterator it = pc.begin(); it != pc.end(); ++it) {
            Point point(it->x, it->y, it->z);
            // maxrange check
            if ((m_maxRange < 0.0) || ((point - sensorOrigin).norm() <= m_maxRange)) {
                // free cells
                if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay))
                    free_cells.insert(m_keyRay.begin(), m_keyRay.end());
                // occupied endpoint
                OcTreeKey key;
                if (m_octree->coordToKeyChecked(point, key)) {
                    occupied_cells.insert(key);

                    updateMinKey(key, m_updateBBXMin);
                    updateMaxKey(key, m_updateBBXMax);
                }
            } 
            else { // ray longer than maxrange:;
                Point new_end = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
                if (m_octree->computeRayKeys(sensorOrigin, new_end, m_keyRay)) {
                    free_cells.insert(m_keyRay.begin(), m_keyRay.end());

                    octomap::OcTreeKey endKey;
                    if (m_octree->coordToKeyChecked(new_end, endKey)) {
                        updateMinKey(endKey, m_updateBBXMin);
                        updateMaxKey(endKey, m_updateBBXMax);
                    } 
                    else {
                        return false;
                    }
                }
            }
        }

        // mark free cells only if not seen occupied in this cloud
        for(KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ++it) {
            if (occupied_cells.find(*it) == occupied_cells.end()) {
                m_octree->updateNode(*it, false);
            }
        }

        // now mark all occupied cells:
        for (KeySet::iterator it = occupied_cells.begin(), end=occupied_cells.end(); it!= end; it++) {
            m_octree->updateNode(*it, true);
        }

        if (m_compressMap)
            m_octree->prune();

        return true;
    }

    bool VillaOctomap::clearBBX(const Point& min, const Point& max) {
        double thresMin = m_octree->getClampingThresMin();
        for(OcTreeT::leaf_bbx_iterator it = m_octree->begin_leafs_bbx(min,max),
            end=m_octree->end_leafs_bbx(); it!= end; ++it) {

            it->setLogOdds(octomap::logodds(thresMin));
        }

        m_octree->updateInnerOccupancy();

        return true;
    }

    bool VillaOctomap::reset() {
        m_octree->clear();
        return true;
    }

    void VillaOctomap::getNodes(Nodes& free, Nodes& occupied) {
        for (OcTreeT::iterator it = m_octree->begin(m_maxTreeDepth), end = m_octree->end(); it != end; ++it) {
            if (m_octree->isNodeOccupied(*it)) {
                double z = it.getZ();
                if (z > m_occupancyMinZ && z < m_occupancyMaxZ) {
                    unsigned depth = it.getDepth();
                    double x = it.getX();
                    double y = it.getY();

                    // Ignore speckles in the map:
                    if (m_filterSpeckles && (depth == m_treeDepth+1) && isSpeckleNode(it.getKey())) {
                        continue;
                    } // else: current octree node is no speckle, send it out

                    occupied.push_back(Node(depth, x, y, z));
                }
            } else { // node not occupied => mark as free in 2D map if unknown so far
                double z = it.getZ();
                if (z > m_occupancyMinZ && z < m_occupancyMaxZ) {
                    unsigned depth = it.getDepth();
                    double x = it.getX();
                    double y = it.getY();

                    free.push_back(Node(depth, x, y, z));
                }
            }
        }
    }

    bool VillaOctomap::isSpeckleNode(const OcTreeKey& nKey) {
        OcTreeKey key;
        bool neighborFound = false;
        for (key[2] = nKey[2] - 1; !neighborFound && key[2] <= nKey[2] + 1; ++key[2]) {
            for (key[1] = nKey[1] - 1; !neighborFound && key[1] <= nKey[1] + 1; ++key[1]) {
                for (key[0] = nKey[0] - 1; !neighborFound && key[0] <= nKey[0] + 1; ++key[0]) {
                    if (key != nKey){
                        OcTreeNode* node = m_octree->search(key);
                        if (node && m_octree->isNodeOccupied(node)) {
                            // we have a neighbor => break!
                            neighborFound = true;
                        }
                    }
                }
            }
        }

        return neighborFound;
    }
}

