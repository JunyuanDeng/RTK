#include "transform.hpp"
#include "readFileName.hpp"
#include "pointcloud.hpp"

int main()
{
    RTKInfos temp("reference.csv");
    temp.run();

    std::vector<std::string> file_name;
    std::string path = "/media/juneyoung/Samsung_T5/dataset/UrbanNav/pcd_rtk";
    readFileName file(path);
    file.GetFileNames();
    file_name = file.fileNames();


    pointCloud pointcloud;
    pointcloud.PCDsFileTrans(file_name, "groundTruth.txt");
    pointcloud.FinalCloudPtrVisualize();
    pcl::io::savePCDFile("finalePCD.pcd", *pointcloud.FinalCloudPtr);
}