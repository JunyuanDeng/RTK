#ifndef POINTCLOUD
#define POINTCLOUD

#include "transform.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;

class pointCloud
{
private:
    
public:

    PointCloudPtr FinalCloudPtr;
    pointCloud():FinalCloudPtr(new PointCloud){};
    PointCloudPtr readPCD(const std::string& pcdFileName);
    //void readGroundtruth(std::string groudTruthFile);
    void PCDsFileTrans(std::vector<std::string> PCDsFileAddrs, std::string groundTruthFile);
    void FinalCloudPtrVisualize();
};

#endif
