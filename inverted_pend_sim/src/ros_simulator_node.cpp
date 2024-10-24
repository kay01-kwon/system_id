#include "ros_wrapper/ros_wrapper_simulator.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_simulator_node");
    ros::NodeHandle nh;

    RosWrapperSimulator ros_wrapper_simulator(nh);

    ros_wrapper_simulator.run();

    return 0;
}