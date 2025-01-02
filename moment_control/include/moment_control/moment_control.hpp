#ifndef MOMENT_CONTROL_HPP
#define MOMENT_CONTROL_HPP
#include "variable_type_moment_control.hpp"
#include <ros/ros.h>
#include <ros_libcanard/cmd_raw.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>

using ros_libcanard::cmd_raw;
using std_msgs::Float64;
using sensor_msgs::Imu;

class MomentController
{
    public:

        MomentController() = delete;

        MomentController(const ros::NodeHandle &nh);

        void ros_run();

        ~MomentController();


    private:

        ros::NodeHandle nh_;

        ros::Subscriber sub_ref_;
        ros::Subscriber sub_imu_;

        ros::Publisher pub_cmd_raw_;


        void ref_callback(const Float64::ConstPtr &msg);

        void imu_callback(const Imu::ConstPtr &msg);

        double C_T_{0.};
        double C_M_{2e-9};
        double l_{0.340};

        double max_bit_{8191};
        double max_rpm_{9800};
        double min_rpm_{2000};

        double K_p_{1.0};
        double ref_{0.0};

        double f_z_{3.0};

        Matrix4x4d K_inverse_;

};


#endif