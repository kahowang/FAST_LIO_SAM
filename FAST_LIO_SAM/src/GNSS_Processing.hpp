#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <ros/ros.h>
#include <so3_math.h>
#include <Eigen/Eigen>
#include <common_lib.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>
#include "use-ikfom.hpp"

#include <GeographicLib/LocalCartesian.hpp>                 //  调用GeographicLib库

class GnssProcess
{
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double time ;
    double latitude ;
    double longitude ;
    double altitude ;
    double local_E ;
    double local_N ;
    double local_U ;
    int status ;
    int service ;

    double origin_longitude;
    double origin_latitude;
    double origin_altitude;

    V3D pose_cov ;

    GnssProcess();
    ~GnssProcess();

    void InitOriginPosition(double latitude, double longitude, double altitude);
    void UpdateXYZ(double latitude, double longitude, double altitude);

    void Reverse(
      const double &local_E, const double &local_N, const double &local_U,
      double &lat, double &lon, double &alt
    );

    void set_extrinsic(const V3D &transl, const M3D &rot);
    void set_extrinsic(const V3D &transl);
    void set_extrinsic(const MD(4,4) &T);

  private:
    GeographicLib::LocalCartesian geo_converter;

    M3D Gnss_R_wrt_Lidar ;
    V3D Gnss_T_wrt_Lidar;
};

GnssProcess::GnssProcess()
{
    time = 0.0;
    local_E = 0.0;
    local_N = 0.0;
    local_U = 0.0;
    status = 0;
    service = 0;

    origin_longitude = 0 ;
    origin_latitude = 0;
    origin_altitude = 0;
    pose_cov = Zero3d;
    Gnss_T_wrt_Lidar = Zero3d;  
    Gnss_R_wrt_Lidar = Eye3d;
}

GnssProcess::~GnssProcess() {}

// 初始化原点， WGS84 -> ENU   ???  调试结果好像是 NED 北东地
void GnssProcess::InitOriginPosition(double latitude, double longitude, double altitude)
{
    geo_converter.Reset(latitude, longitude, altitude);
    ROS_INFO("Init    Gnss  OriginPosition");   
    origin_latitude = latitude;
    origin_longitude = longitude;
    origin_altitude = altitude;
}

// 获取更新后的ENU坐标
void GnssProcess::UpdateXYZ(double latitude, double longitude, double altitude) {
    geo_converter.Forward(latitude, longitude, altitude, local_E, local_N, local_U);
}

void GnssProcess::Reverse(
    const double &local_E, const double &local_N, const double &local_U,
    double &lat, double &lon, double &alt
) {
    geo_converter.Reverse(local_E, local_N, local_U, lat, lon, alt);
}

void GnssProcess::set_extrinsic(const MD(4,4) &T)
{
  Gnss_T_wrt_Lidar = T.block<3,1>(0,3);
  Gnss_R_wrt_Lidar = T.block<3,3>(0,0);
}

void GnssProcess::set_extrinsic(const V3D &transl)
{
  Gnss_T_wrt_Lidar = transl;
  Gnss_R_wrt_Lidar.setIdentity();
}

void GnssProcess::set_extrinsic(const V3D &transl, const M3D &rot)
{
  Gnss_T_wrt_Lidar = transl;
  Gnss_R_wrt_Lidar = rot;
}


