#include <msf_localizer/msf_localizer.h>

std::mutex ekf_mutex;

int main(int argc, char* argv[]){

    ros::init(argc,argv,"ddos_msf_localizer");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    Extended_Kalman_filter EKF(nh,private_nh);
    EKF.init();
    EKF.run();

    ros::spin();
    return 0;
}