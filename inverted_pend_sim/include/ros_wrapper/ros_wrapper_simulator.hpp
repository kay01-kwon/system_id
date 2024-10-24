#ifndef ROS_WRAPPER_SIMULATOR_HPP
#define ROS_WRAPPER_SIMULATOR_HPP

#include "inverted_pend_sim/system_dynamics.hpp"
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/external/eigen/eigen.hpp>
#include "yaml_read/yaml_read.hpp"
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <ros_libcanard/cmd_raw.h>

using boost::numeric::odeint::runge_kutta4;

using sensor_msgs::Imu;
using ros_libcanard::cmd_rawConstPtr;

class RosWrapperSimulator
{
    public:

    RosWrapperSimulator(ros::NodeHandle &nh);

    ~RosWrapperSimulator();

    void cmd_raw_callback(const cmd_rawConstPtr &msg);

    void run();
    
    private:

    void update_state();

    std::unique_ptr<SystemDynamics> system_dynamics_;

    runge_kutta4<Vector2d> stepper_;

    Vector2d s_;
    Quaternionf quatf_;
    Vector3d w_;
    double current_time_, last_time_;

    ros::Subscriber cmd_raw_sub_;
    ros::Publisher imu_pub_;

    ros::Rate loop_rate{200};

};


#endif // ROS_WRAPPER_SIMULATOR_HPP