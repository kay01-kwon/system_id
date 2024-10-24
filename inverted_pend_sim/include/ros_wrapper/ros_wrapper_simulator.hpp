#ifndef ROS_WRAPPER_SIMULATOR_HPP
#define ROS_WRAPPER_SIMULATOR_HPP

#include "inverted_pend_sim/system_dynamics.hpp"
#include "yaml_read/yaml_read.hpp"
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

    void update_state();

    private:

    std::unique_ptr<SystemDynamics> system_dynamics_;

    ros::Subscriber cmd_raw_sub;
    ros::Publisher imu_pub;

    ros::Rate loop_rate{200};

};


#endif // ROS_WRAPPER_SIMULATOR_HPP