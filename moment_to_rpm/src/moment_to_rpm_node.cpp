#include "moment_to_rpm/moment_to_rpm.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moment_to_rpm");

    ros::NodeHandle nh;

    MomentToRPM moment_to_rpm(nh);

    moment_to_rpm.ros_run();

    return 0;
}