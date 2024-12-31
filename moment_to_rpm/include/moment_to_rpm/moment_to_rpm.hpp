#ifndef MOMENT_TO_RPM_HPP
#define MOMNET_TO_RPM_HPP
#include "variable_type.hpp"
#include <ros/ros.h>
#include <ros_libcanard/cmd_raw.h>
#include <geometry_msgs/Wrench.h>

using ros_libcanard::cmd_raw;
using geometry_msgs::Wrench;

class MomentToRPM
{
    public:

        MomentToRPM() = delete;

        MomentToRPM(const ros::NodeHandle &nh);

        void ros_run();

        ~MomentToRPM();

    private:

        ros::NodeHandle nh_;
        ros::Subscriber sub_moment_;
        ros::Publisher pub_cmd_raw_;
    
        void moment_callback(const Wrench::ConstPtr &msg);

        void moment_to_rpm(const Vector4d &f_z_moment, Vector4d &rpm);

        double C_T_{0.};
        double C_M_{2e-9};
        double l_{0.340};

        double max_bit_{8191};
        double max_rpm_{9800};
        double min_rpm_{2000};

        Matrix4x4d K_inverse_;

};


#endif