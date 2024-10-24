#ifndef ROS_WRAPPER_SIMULATOR_HPP
#define ROS_WRAPPER_SIMULATOR_HPP

#include "inverted_pend_sim/system_dynamics.hpp"
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <ros_libcanard/cmd_raw.h>

using sensor_msgs::Imu;
using ros_libcanard::cmd_rawConstPtr;

class RosWrapperSimulator
{
    public:

    RosWrapperSimulator(ros::NodeHandle &nh);

    ~RosWrapperSimulator();

    void cmd_raw_callback(const cmd_rawConstPtr &msg);

    private:


};




#endif // ROS_WRAPPER_SIMULATOR_HPP