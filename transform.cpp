//这个程序的目的是为了：
//给定连续帧的相关位置，旋转信息（reference.csv）
//算出所有帧的Lidar雷达到第一帧东北天坐标系下转换矩阵（旋转与平移），以方便合并后续所有雷达文件（.pcd）。形成一个以第一帧东北天为坐标系的包含所有雷达的pcd文件。
//其中reference里的数据是SPAN（我看这个链接里表述的应该是这个意思https://github.com/weisongwen/UrbanNavDataset/blob/master/UrbanNav-HK-Data20190428/extrinsic.yaml）


#include "transform.hpp"
#include <iostream>
#include <set>



/**
 * @brief  初始化相机外参
 * 
 * 分别有三个设备：Lidar、XSens、SPAN。
 * 外参的地址：https://github.com/weisongwen/UrbanNavDataset/blob/master/UrbanNav-HK-Data20190428/extrinsic.yaml。
 * 三个设备的位置关系图示：https://github.com/weisongwen/UrbanNavDataset/blob/master/img/hongkong_sensor.png。
 * 
 * @param fileaddress csv文件地址
 */
RTKInfos::RTKInfos(std::string fileaddress):filename(fileaddress)
{
    RotXsens2Lidar << 2.67949e-08, -1, 0, 1, 2.67949e-08, 0, 0, 0, 1;
    RotXsens2SPAN = Eigen::Matrix3d::Identity();

    TransXsens2Lidar << 0, 0, -0.28;
    TransXsens2SPAN = Eigen::Vector3d::Zero();

    Xsens2Lidar = Eigen::Isometry3d::Identity();
    Xsens2Lidar.rotate(RotXsens2Lidar);
    Xsens2Lidar.pretranslate(TransXsens2Lidar);

    Xsens2SPAN = Eigen::Isometry3d::Identity();
    Xsens2SPAN.rotate(RotXsens2SPAN);
    Xsens2SPAN.pretranslate(TransXsens2SPAN);

    SPAN2Lidar = Xsens2Lidar * Xsens2SPAN;

}

/**
 * @brief 从csv文件读取数据到包含RTK点vector
 * @param file csv文件名
 */
std::vector<RTKpoint> RTKInfos::readFromCsv(std::string file)
{
    std::vector<RTKpoint> readResult;
    std::ifstream csvFile(file, std::ios::in);
    std::set<int> colonneToSkip{0, 1, 2, 6, 7, 8};
    std::string data;
    bool firstLine = true;
    while (std::getline(csvFile, data))
    {
        if (firstLine)
        {
            firstLine =false;
            continue;
        }


        std::string valueString;
        std::stringstream ssData(data);
        int colonne = 0;
        RTKpoint point;
        while (getline(ssData, valueString, ','))
        {
            if (colonneToSkip.find(colonne) != colonneToSkip.end())
            {
                colonne++;
                continue;
            }
                
            else
            {
                double value = std::stod(valueString);
                switch (colonne)
                {
                case 3:
                    point.latitude = value / 180 * M_PI;
                    break;
                case 4:
                    point.longitude = value/ 180 * M_PI;
                    break;
                case 5:
                    point.altitude = value;
                    break;
                case 9:
                    point.roll = value / 180 * M_PI;
                    break;
                case 10:
                    point.pitch = value / 180 * M_PI;
                    break;
                case 11:
                    //if (value > 180)
                    //   value -= 360;
                    point.heading = (value) / 180 * M_PI;
                    break;
                default:
                    break;
                }
                colonne++;
            }
        }
        readResult.push_back(point);
    }
    return readResult;
}

/**
 * @brief 经纬度到地心坐标系的转换
 * @param 经度、纬度、高度
 * @return 地心坐标系
 */
Eigen::Vector3d RTKInfos::BLH2XYZ(Eigen::Vector3d& BLH)
{
    Eigen::Vector3d XYZ;
    double earth_a = 6378137.0000;
    double earth_e = 0.081819190928906327;
    double N = earth_a / (sqrt(1 - earth_e * earth_e * sin(BLH(0)) * sin(BLH(0))));
    XYZ(0) = (N + BLH(2)) * cos(BLH(0)) * cos(BLH(1));
    XYZ(1) = (N + BLH(2)) * cos(BLH(0)) * sin(BLH(1));
    XYZ(2) = (N * (1 - earth_e * earth_e) + BLH(2))*sin(BLH(0));
    return XYZ;
}

/**
 * @brief 欧拉角到旋转矩阵
 * @param 欧拉角（heading，pitch，roll）
 * @return 旋转矩阵
 */
Eigen::Matrix3d RTKInfos::RYH2RotMat(Eigen::Vector3d& HPR)
{
    Eigen::Matrix3d rotation_matrix3;
    rotation_matrix3 = Eigen::AngleAxisd(HPR(0), Eigen::Vector3d::UnitZ()) * 
                       Eigen::AngleAxisd(HPR(1), Eigen::Vector3d::UnitY()) * 
                       Eigen::AngleAxisd(HPR(2), Eigen::Vector3d::UnitX());
    return rotation_matrix3;
}

/**
 * @brief 将包含RTK点的Vector中的每个RTK点转换成第一帧东北天坐标系下的点
 */
void RTKInfos::transformation(std::vector<RTKpoint>& RTKpoints)
{
    for (auto pointIter = RTKpoints.begin(); pointIter != RTKpoints.end(); pointIter++)
    {
        Eigen::Vector3d Lidar2ENU;         //雷达到ENU的位移
        Eigen::Matrix3d RotMatLidar2ENU; //雷达到ENU的旋转矩阵

        auto point = *pointIter; 
        Eigen::Vector3d BLH(point.latitude, point.longitude, point.altitude);
        Eigen::Vector3d HPR(point.heading, point.pitch, point.roll);
        

        Eigen::Matrix3d FirstRotMatXYZ2ENU;
        Eigen::Vector3d FirstXYZ;
        Eigen::Vector3d FirstBLH;

        if (pointIter == RTKpoints.begin())  //如果是第一帧真值传感器接受的数据
        {
            FirstXYZ = BLH2XYZ(BLH);
            FirstBLH = BLH;

            auto RotMatSPAN2ENU = RYH2RotMat(HPR);            // 欧拉角转换成旋转矩阵（这个应该是在SPAN坐标系到ENU坐标系的旋转矩阵，李涛学长是这么说的）
            FirstRotMatXYZ2ENU = RotMatXYZ2ENU(BLH);          //地心坐标系到第一帧ENU的转换
            RotMatLidar2ENU =  RotMatSPAN2ENU * RotXsens2SPAN * RotXsens2Lidar.inverse();  //得到雷达到ENU的旋转矩阵

            auto RotMatXsens2ENU = RotMatSPAN2ENU * RotXsens2SPAN;
            Lidar2ENU = RotMatXsens2ENU * (-TransXsens2Lidar); //得到雷达在第一帧ENU为参考系的点的坐标，这里加负号是因为Xsens到雷达的4*4矩阵T中的最右边一列表示的是雷达到Xsens的位移
        }
        else
        {
            //Lidar2ENU = BLH2ENU(FirstXYZ, FirstBLH, XYZ, BLH);  //首先得到后续帧真值传感器在第一帧下的坐标
            //Lidar2ENU = SPAN2Lidar.inverse() * Lidar2ENU;
    
            //当前帧雷达到第一帧ENU的旋转矩阵的计算
            auto RotMatSPAN2curENU = RYH2RotMat(HPR);            // 欧拉角转换成旋转矩阵（这个应该是在SPAN坐标系到当前ENU坐标系的旋转矩阵）
            auto RotcurLidar2curENU =  RotMatSPAN2curENU * RotXsens2SPAN * RotXsens2Lidar.inverse(); //得到雷达到当前帧ENU坐标的变换 
            auto RotMatXYZ2curENU = RotMatXYZ2ENU(BLH);       //地心坐标系到当前ENU的转换
            auto RotcurENU2refENU =  FirstRotMatXYZ2ENU * RotMatXYZ2curENU.inverse();   //得到当前帧ENU到第一帧ENU的转换
            RotMatLidar2ENU = RotcurENU2refENU * RotcurLidar2curENU; //得到当前帧雷达到第一帧ENU的变换


            //当前帧雷达到第一帧ENU的位移的计算
            auto curXYZ = BLH2XYZ(BLH);                         //地心坐标系下当前帧的XYZ
            auto refXYZ2curXYZ = curXYZ - FirstXYZ;             //地心坐标系下两个SPAN的位移
            auto RotMatXsens2XYZ = RotMatXYZ2curENU.inverse()  * RotMatSPAN2curENU * RotXsens2SPAN; //算出Xsens到XYZ的旋转矩阵变换
            auto refXYZ2curLidar = -(RotMatXsens2XYZ*TransXsens2Lidar) +refXYZ2curXYZ; //地心坐标系下第一个SPAN到当前帧雷达的位移
                                                                                       //原本需要+ TransXsens2SPAN，但是是0向量就忽略了
            Lidar2ENU = FirstRotMatXYZ2ENU * refXYZ2curLidar;   //转到第一帧ENU坐标系下SPAN到当前帧雷达的位移

            
        }
        position.push_back(Lidar2ENU);
        rotation.push_back(RotMatLidar2ENU);
    }
}

/**
 * @brief 得到一个当前BLH下，从地心坐标系到ENU坐标系的旋转矩阵
 * @param 经度纬度高度
 * @return 地心坐标系到ENU坐标系的旋转矩阵
 */
Eigen::Matrix3d RTKInfos::RotMatXYZ2ENU(Eigen::Vector3d& BLH)
{
    Eigen::Matrix3d M;
    M(0, 0) = -sin(BLH(1));
    M(0, 1) = cos(BLH(1));
    M(0, 2) = 0;
    M(1, 0) = -sin(BLH(0))*cos(BLH(1));
    M(1, 1) = -sin(BLH(0))*sin(BLH(1));
    M(1, 2) = cos(BLH(0));
    M(2, 0) = cos(BLH(0))*cos(BLH(1));
    M(2, 1) = cos(BLH(0))*sin(BLH(1));
    M(2, 2) = sin(BLH(0));
    return M;
}

/**
 * @brief 给定一个地心坐标系点，转换到以第一帧为参考系的坐标
 * @param Ref(Cur)_XYZ 第一帧(当前帧)地心坐标系点坐标
 * @param Ref(Cur)_BLH 第一帧(当前帧)经度纬度高度
 * @return 当前帧在第一帧下的坐标
 */
Eigen::Vector3d RTKInfos::BLH2ENU(Eigen::Vector3d& Ref_XYZ, Eigen::Vector3d& Ref_BLH, Eigen::Vector3d& Cur_XYZ, Eigen::Vector3d& Cur_BLH)
{
    Eigen::Matrix3d M = RotMatXYZ2ENU(Ref_BLH);
    
    Eigen::Vector3d delta_A1_A2;
    delta_A1_A2(0) = Cur_XYZ(0) - Ref_XYZ(0);
    delta_A1_A2(1) = Cur_XYZ(1) - Ref_XYZ(1);
    delta_A1_A2(2) = Cur_XYZ(2) - Ref_XYZ(2);

    Eigen::Vector3d T = M * delta_A1_A2;

    return T;
}

void RTKInfos::run()
{
    auto RTKpoints = readFromCsv(filename);
    transformation(RTKpoints);
    write2csv();
    write2txt();
}

/**
 * @brief 将以第一帧为参考的所有点坐标写到csv文件，其中用四元数表示旋转
 */

void RTKInfos::write2csv()
{
    std::ofstream outFile;
    outFile.open("groundTruth.csv", std::ios::out);

    outFile<<'x'<<','<<'y'<<','<<'z'<<','<<"q1"<<','<<"q2"<<','<<"q3"<<','<<'w'<<std::endl;
    for (int i = 0; i != position.size(); i++)
    {
        auto t = position[i];
        outFile<<t(0)<<','<<t(1)<<','<<t(2)<<',';
        Eigen::Quaterniond q(rotation[i]);
        auto coeff = q.coeffs();
        outFile<<coeff(0)<<','<<coeff(1)<<','<<coeff(2)<<','<<coeff(3)<<std::endl;
    }
}

/**
 * 
 * @brief 将以第一帧为参考的所有点坐标写到txt文件，其中用旋转矩阵表示旋转 
 * 
 * 
 */
void RTKInfos::write2txt()
{
    std::ofstream outFile;
    outFile.open("groundTruth.txt", std::ios::out);

    for (int i = 0; i != position.size(); i++)
    {
        auto t = position[i];
        outFile<<t(0)<<' '<<t(1)<<' '<<t(2)<<' ';

        auto R = rotation[i];
        outFile << R(0, 0) << " " << R(0,1) << " " << R(0, 2) << " "
                << R(1, 0) << " " << R(1,1) << " " << R(1, 2) << " "
                << R(2, 0) << " " << R(2,1) << " " << R(2, 2) << std::endl;
        // Eigen::Quaterniond q(rotation[i]);
        // auto coeff = q.coeffs();
        // outFile<<coeff(0)<<' '<<coeff(1)<<' '<<coeff(2)<<' '<<coeff(3)<<std::endl;
    }
}
