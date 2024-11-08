#include "trajectory_generation/trajectory_generator.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_trajectory_node");
    ros::NodeHandle nh;

    TrajectoryGeneration trajectory_generation(nh, 1, M_PI/2);

    trajectory_generation.ros_run();

    return 0;
}