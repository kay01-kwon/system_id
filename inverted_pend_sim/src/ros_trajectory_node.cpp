#include "trajectory_generation/trajectory_generator.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_trajectory_node");
    ros::NodeHandle nh;

    double freq;
    double ampl;

    nh.param("freq", freq);
    nh.param("ampl", ampl);

    TrajectoryGeneration trajectory_generation(nh, 1.0, M_PI/12);

    trajectory_generation.ros_run();

    return 0;
}