#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <signal.h>
#include <vector>
#include <numeric>
#include <algorithm>
#include <iomanip>
#include <cmath>
#include <ublox_msgs/NavPVT.h>
#include <ublox_msgs/NavATT.h>
#include <novatel_gps_msgs/Inspva.h>
#include <gps_common/GPSFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <autoware_msgs/NDTStat.h>
#include <novatel_gps_msgs/Inspva.h>
#include <geometry_msgs/PoseStamped.h>
#include <ddos_msf_localizer/Msf_state.h>
#include <ddos_msf_localizer/Position.h>

const double pi = 3.14159265358979323846;
std::ofstream fout;

typedef struct imu_raw {
    long double linear_e;
    long double linear_n;
    long double linear_u;
    long double angular_e;
    long double angular_n;
    long double angular_u;
    long double time;
    long double pre_time;
}imu_raw_t;

typedef struct imu_nav {
    long double lon;
    long double lat;
    double height;
    double roll;
    double pitch;
    double yaw;
    double velE;
    double velN;
    double velU;
    double covariance;
}imu_nav_t;
typedef struct pos_nav {
    long double lon;
    long double lat;
    double height;
    double east;
    double north;
    double up;
    double covariance;
}pos_nav_t;

imu_raw_t imu;
imu_nav_t nav;
imu_nav_t novatel;
pos_nav_t ndt;
pos_nav_t msf;
ros::Time start_time;



void getFix(const sensor_msgs::NavSatFix& msg){
    nav.lat = msg.latitude;
    nav.lon = msg.longitude;
    nav.height = msg.altitude;
    nav.covariance = msg.position_covariance[0];
}

void getFixVelocity(const geometry_msgs::TwistWithCovarianceStamped& msg){
    // enu frame velocity (m)
    nav.velE = msg.twist.twist.linear.x;
    nav.velN = msg.twist.twist.linear.y;
    nav.velU = -msg.twist.twist.linear.z;
}

void getNavatt(const ublox_msgs::NavATT& msg){
    // unit (deg)
    nav.roll = msg.roll * 1e-5;
    nav.pitch = msg.pitch * 1e-5;
    double yaw = msg.heading * 1e-5;
    if(yaw <= 180)
        nav.yaw = -yaw;
    else if(yaw > 180 && yaw <= 360)
        nav.yaw = -(yaw - 360);
}
void getNovatel(const novatel_gps_msgs::Inspva& msg){
    // novatel.lat = msg.latitude;
    // novatel.lon = msg.longitude;
    // novatel.height = msg.height;
    double novatel_yaw = msg.azimuth;
    
    novatel.roll = msg.roll * pi / 180;
    novatel.pitch = msg.pitch * pi / 180;
    // yaw angle between enu and ned frame
    if(novatel_yaw > 0 && novatel_yaw <= 180)
        novatel.yaw = -novatel_yaw;
    else if(novatel_yaw > 180 && novatel_yaw <= 360)
        novatel.yaw = -(novatel_yaw - 360);
    novatel.yaw = novatel.yaw * pi / 180;
}
void getImuMeas(const sensor_msgs::Imu& msg){
    // ublox f9k base to local(ENU) frame
    imu.linear_e = -msg.linear_acceleration.y;
    imu.linear_n = msg.linear_acceleration.x;
    imu.linear_u = msg.linear_acceleration.z;
    imu.angular_e = -msg.angular_velocity.y;
    imu.angular_n = msg.angular_velocity.x;
    imu.angular_u = msg.angular_velocity.z;
    imu.time = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9;
    
    std::cout << "Date Writing" << std::endl;
    fout << nav.lat << "," << nav.lon << "," << nav.height << "," 
        << nav.velE << "," << nav.velN << "," << nav.velU << "," << nav.yaw << "," << nav.covariance << ","
        << novatel.lat << "," << novatel.lon  << "," << novatel.height  << "," 
        << novatel.roll << "," << novatel.pitch << "," << novatel.yaw << ","
        << ndt.lat << "," << ndt.lon << "," << ndt.height << ","
        << ndt.east << "," << ndt.north << "," << ndt.up << ","
        << msf.lat << "," << msf.lon << "," << msf.height << ","
        << msf.east << "," << msf.north << "," << msf.up << "," << msf.covariance << ","
        << "\n";
        
}

void getNdtFix(const sensor_msgs::NavSatFix& msg){
    novatel.lat = msg.latitude;
    novatel.lon = msg.longitude;
    novatel.height = msg.altitude;
}
void getNdtPose(const geometry_msgs::PoseStamped::ConstPtr& msg){
    ndt.east = msg->pose.position.x;
    ndt.north = msg->pose.position.y;
    ndt.up = -msg->pose.position.z;
}
void getMsfFix(const sensor_msgs::NavSatFix& msg){
    msf.lat = msg.latitude;
    msf.lon = msg.longitude;
    msf.height = msg.altitude;
    msf.covariance = msg.position_covariance[1];
}
void getMsfPose(const geometry_msgs::PoseStamped::ConstPtr& msg){
    msf.east = msg->pose.position.x;
    msf.north = msg->pose.position.y;
    msf.up = -msg->pose.position.z;
}
void Siginthandler(int sig){
    fout.close();
    ros::shutdown();
    std::cout << "END Data_Collect" << std::endl;
}

int main(int argc,char **argv){
    signal(SIGINT, Siginthandler);
    ros::init(argc, argv, "data_collect");
    ros::NodeHandle n;
    
    std::string filePath;
    n.getParam("data_collect/file_path", filePath);
    
    ros::Subscriber sub[9];
    // sub[0] = n.subscribe("/ublox_f9k/fix",1,getFix);
    // sub[2] = n.subscribe("/ublox_f9k/fix_velocity",1,getFixVelocity);
    // sub[1] = n.subscribe("/ublox_f9k/imu_meas",1,getImuMeas);
    // sub[3] = n.subscribe("/ublox_f9k/navatt",1,getNavatt);
    sub[0] = n.subscribe("/ublox_gps/fix",1,getFix);
    sub[2] = n.subscribe("/ublox_gps/fix_velocity",1,getFixVelocity);
    sub[1] = n.subscribe("/ublox_gps/imu_meas",1,getImuMeas);
    sub[3] = n.subscribe("/ublox_gps/navatt",1,getNavatt);

    sub[4] = n.subscribe("/novatel/inspva",1,getNovatel);
    sub[5] = n.subscribe("/novatel/fix",1,getNdtFix);
    sub[6] = n.subscribe("/ndt_pose",1,getNdtPose);
    sub[7] = n.subscribe("/ddos_msf_localizer/msf_fix",1,getMsfFix);
    sub[8] = n.subscribe("/ddos_msf_localizer/msf_pose",1,getMsfPose);

    // ros::Rate loop_rate(50);
    std::cout << filePath << std::endl;
    fout.open(filePath,std::ios::out | std::ios::app);
    fout.precision(12);
    fout << "lat_ublox" << "," << "lon_ublox" << "," << "alt_ublox" << "," 
        << "velE_ublox" << "," << "velN_ublox" << "," << "velU_ublox" << "," << "yaw_unlox" << "," << "cov_ublox" << ","
        << "lat_novatel" << "," << "lon_novatel" << "," << "alt_novatel"  << "," 
        << "roll_novatel" << "," << "pitch_novatel" << "," << "yaw_novatel" << ","
        << "lat_ndt" << "," << "lon_ndt" << "," << "alt_ndt" << ","
        << "e_ndt" << "," << "n_ndt" << "," << "u_ndt" << ","
         << "lat_msf" << "," << "lon_msf" << "," << "alt_msf" << ","
        << "e_msf" << "," << "n_msf" << "," << "u_msf" << "," << "cov_msf" << ","
        << "\n";
    start_time = ros::Time::now(); 
    std::cout << "Start to Data_Collect" << std::endl;
    ros::spin();
    
    return 0;
}