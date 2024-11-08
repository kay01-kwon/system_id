#ifndef TRAJECTORY_GENERATION_HPP
#define TRAJECTORY_GENERATION_HPP

#include <ros/ros.h>
#include <std_msgs/Float64.h>

// Generate roll, pitch, yaw trajectories
// for system identification
// It generates a sinusoidal trajectory
class TrajectoryGeneration
{
    public:
        TrajectoryGeneration(ros::NodeHandle &nh, double freq, double ampl);
        
        ~TrajectoryGeneration();

        void ros_run();

    private:

        void update_reference(const double &t);

        ros::Publisher reference_pub_;

        ros::Rate loop_rate{200};

        double ref_;
        double current_time_;
        double last_time_;
        double dt_;

        double ampl_;
        double freq_;

};


#endif