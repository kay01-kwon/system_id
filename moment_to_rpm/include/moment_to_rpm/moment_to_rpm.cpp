#include "moment_to_rpm.hpp"

MomentToRPM::MomentToRPM(const ros::NodeHandle &nh)
:nh_(nh)
{
    // Get the parameters from the parameter server
    nh_.param("C_T", C_T_, 0.);
    nh_.param("C_m", C_M_, 2e-9);
    nh_.param("l", l_, 0.340);

    // Subscribe to the moment topic
    sub_moment_ = nh_.subscribe("/moment", 1, 
    &MomentToRPM::moment_callback, this);

    // Publish the rpm to the cmd_raw topic
    pub_cmd_raw_ = nh_.advertise<cmd_raw>("/cmd_raw", 1);
}

void MomentToRPM::ros_run()
{
    while(ros::ok())
    {
        ros::spinOnce();
    }
}

void MomentToRPM::moment_callback(const Wrench::ConstPtr &msg)
{
    Vector3d moment;
    Vector4d rpm;

    // Get the moment from the message
    moment << msg->torque.x, msg->torque.y, msg->torque.z;

    // Convert the moment to rpm
    moment_to_rpm(moment, rpm);


    cmd_raw cmd;

    for(size_t i = 0; i < 4; i++)
    {
        cmd.raw[i] = rpm(i);
    }

    // Publish the cmd raw
    pub_cmd_raw_.publish(cmd);
}

void MomentToRPM::moment_to_rpm(const Vector3d &moment, Vector4d &rpm)
{
}