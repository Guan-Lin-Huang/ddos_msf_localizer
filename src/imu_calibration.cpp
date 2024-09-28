#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <sensor_msgs/Imu.h>
#include <signal.h>
#include <vector>
#include <numeric>

std::ofstream fout;
ros::Time start_time;

void getImuMeas(const sensor_msgs::Imu& msg) {
    double linear_x = msg.linear_acceleration.x;
    double linear_y = msg.linear_acceleration.y;
    double linear_z = msg.linear_acceleration.z;
    double angular_x = msg.angular_velocity.x;
    double angular_y = msg.angular_velocity.y;
    double angular_z = msg.angular_velocity.z;

    fout << linear_x << ", " << linear_y << ", " << linear_z << ", " 
         << angular_x << ", " << angular_y << ", " << angular_z << "\n";

    std::cout << "Wait "<< (180 - (ros::Time::now() - start_time).toSec()) <<" seconds" << std::endl;
}

void Siginthandler(int sig) {
    fout.close();
    ros::shutdown();
}

int main(int argc, char **argv) {
    signal(SIGINT, Siginthandler);
    ros::init(argc, argv, "Imu_calibration");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/ublox_gps/imu_meas", 1, getImuMeas);
    
    std::string filePath = "/home/mec/mecware/catkin_ws/src/ddos_rtk/src/z1.csv";
    fout.open(filePath, std::ios::out | std::ios::app);
    fout.precision(12);
    fout << "linear_x" << ", " << "linear_y" << ", " << "linear_z" << ", "
         << "angular_x" << ", " << "angular_y" << ", " << "angular_z" << "\n";

    start_time = ros::Time::now();
    std::cout << "Don't move ublox, Start to Imu_calibration" << std::endl;
    ros::spin();

    std::ifstream fin(filePath);
    std::string line;
    std::vector<double> linear_x_data, linear_y_data, linear_z_data, angular_x_data, angular_y_data, angular_z_data;


    std::getline(fin, line);

    while (std::getline(fin, line)) {
        std::istringstream iss(line);
        double linear_x, linear_y, linear_z, angular_x, angular_y, angular_z;
        char comma;

        if (iss >> linear_x >> comma >> linear_y >> comma >> linear_z >> comma >> angular_x >> comma >> angular_y >> comma >> angular_z) {
            linear_x_data.push_back(linear_x);
            linear_y_data.push_back(linear_y);
            linear_z_data.push_back(linear_z);
            angular_x_data.push_back(angular_x);
            angular_y_data.push_back(angular_y);
            angular_z_data.push_back(angular_z);
        }
    }

    double mean_linear_x = std::accumulate(linear_x_data.begin(), linear_x_data.end(), 0.0) / linear_x_data.size();
    double mean_linear_y = std::accumulate(linear_y_data.begin(), linear_y_data.end(), 0.0) / linear_y_data.size();
    double mean_linear_z = std::accumulate(linear_z_data.begin(), linear_z_data.end(), 0.0) / linear_z_data.size();
    double mean_angular_x = std::accumulate(angular_x_data.begin(), angular_x_data.end(), 0.0) / angular_x_data.size();
    double mean_angular_y = std::accumulate(angular_y_data.begin(), angular_y_data.end(), 0.0) / angular_y_data.size();
    double mean_angular_z = std::accumulate(angular_z_data.begin(), angular_z_data.end(), 0.0) / angular_z_data.size();

    std::cout << "Mean Linear Acceleration (m/s^2): " << "x: " << mean_linear_x << ", y: " << mean_linear_y << ", z: " << mean_linear_z << std::endl;
    std::cout << "Mean Angular Velocity (rad/s): " << "x: " << mean_angular_x << ", y: " << mean_angular_y << ", z: " << mean_angular_z << std::endl;

    // Mean Linear Acceleration (m/s^2): x: -0.122802, y: -0.0175636, z: 10.0725
    // Mean Angular Velocity (rad/s): x: -0.0016567, y: -0.00523431, z: -0.000366345

    return 0;
}