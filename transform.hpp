#ifndef TRANSFORM
#define TRANSFORM

#include <fstream>
#include <string>
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <sstream>

struct RTKpoint
{
    double latitude, longitude, altitude;
    double roll, pitch, heading;
    //double x, y, z;
    //Eigen::Matrix3d Rotation; 
};

class RTKInfos
{
private:
    Eigen::Matrix3d RotXsens2Lidar;
    Eigen::Matrix3d RotXsens2SPAN;

    Eigen::Vector3d TransXsens2Lidar;
    Eigen::Vector3d TransXsens2SPAN;

    Eigen::Isometry3d Xsens2Lidar;
    Eigen::Isometry3d Xsens2SPAN;

    Eigen::Isometry3d SPAN2Lidar;

    std::vector<Eigen::Vector3d> position;
    std::vector<Eigen::Matrix3d> rotation;
    std::string filename;


    std::vector<RTKpoint> readFromCsv(std::string file);
    void transformation(std::vector<RTKpoint>& RTKpoints);

    Eigen::Vector3d BLH2XYZ(Eigen::Vector3d& BLH);
    Eigen::Matrix3d RYH2RotMat(Eigen::Vector3d& HPR);
    Eigen::Matrix3d RotMatXYZ2ENU(Eigen::Vector3d& BLH);
    Eigen::Vector3d BLH2ENU(Eigen::Vector3d& Ref_XYZ, Eigen::Vector3d& Ref_BLH, Eigen::Vector3d& Cur_XYZ, Eigen::Vector3d& Cur_BLH);

    void write2csv();
    void write2txt();
     
public:
    RTKInfos();
    RTKInfos(std::string fileaddress);
    void run();
};


#endif