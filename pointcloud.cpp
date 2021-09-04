#include "pointcloud.hpp"

PointCloudPtr pointCloud::readPCD(const std::string& pcdFileName)
{
    PointCloudPtr cloudPtr(new PointCloud);
    std::cout<<"readFile"<<pcdFileName<<std::endl;
    if( pcl::io::loadPCDFile(pcdFileName, *cloudPtr) == -1)
    {
        PCL_ERROR("Couldn't read pcd file");
        exit(1);
    }
    std::cout << "Loaded:" << cloudPtr->width*cloudPtr->height<<"data points from test_pcd.pcd with the following fields:"<< std::endl;
    return cloudPtr;
}
void pointCloud::PCDsFileTrans(std::vector<std::string> PCDsFileAddrs, std::string groundTruthFile)
{
    std::ifstream groundTruth(groundTruthFile);
    if(!groundTruth)
    {
        std::cout << "cannot find trajectory file at " << groundTruthFile << std::endl;
        return;
    }

    auto fileNameIte = PCDsFileAddrs.begin();
    while(!groundTruth.eof()&&fileNameIte != PCDsFileAddrs.end())
    {
        // float x, y, z, q1, q2, q3, w;
        // groundTruth >> x >> y >> z >> q1 >> q2 >> q3 >> w;
        float x, y, z, r1, r2, r3, r4, r5, r6, r7, r8, r9;
        groundTruth >> x >> y >> z >> r1 >> r2 >> r3 >> r4 >> r5 >> r6 >> r7 >> r8 >> r9;
        // Eigen::Affine3f T(Eigen::Quaternionf(w, q1, q2, q3));

        Eigen::Matrix3f R;
        R << r1, r2, r3, r4, r5, r6, r7, r8, r9;
        
        Eigen::Affine3f T(R);

        T.translation()<< x, y, z;
        
        //  Eigen::Isometry3f T =  Eigen::Isometry3f::Identity();;
        //  T.rotate(R);
        //  T.pretranslate( Eigen::Vector3f(x, y, z));

        auto PCDfile = *fileNameIte;
        fileNameIte++;
        PointCloudPtr onePointCloud = readPCD(PCDfile);
        PointCloudPtr transformedPointCloud(new PointCloud);
        
        // for (size_t i = 0; i < onePointCloud->points.size(); ++i) 
        // {
        //     Eigen::Vector3f pointbefore(onePointCloud->points[i].x, onePointCloud->points[i].y, onePointCloud->points[i].z);
        //     auto pointafter = T * pointbefore;
        //     onePointCloud->points[i].x = pointafter(0,0);
        //     onePointCloud->points[i].y = pointafter(1,0);
        //     onePointCloud->points[i].z = pointafter(2,0);
        // }
        // *FinalCloudPtr += *onePointCloud;
        // onePointCloud->clear();
        // transformedPointCloud->clear();

        pcl::transformPointCloud(*onePointCloud, *transformedPointCloud, T);
        *FinalCloudPtr += *transformedPointCloud;
        onePointCloud->clear();
        transformedPointCloud->clear();
        
        
    }
}


void pointCloud::FinalCloudPtrVisualize()
{
    pcl::visualization::CloudViewer viewer("pcd viewer");
    viewer.showCloud(FinalCloudPtr);
    while (!viewer.wasStopped())
    {
    }
}