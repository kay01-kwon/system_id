#include "moment_control/moment_control.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moment_control");

    ros::NodeHandle nh;

    MomentController moment_control(nh);

    moment_control.ros_run();

    return 0;
}