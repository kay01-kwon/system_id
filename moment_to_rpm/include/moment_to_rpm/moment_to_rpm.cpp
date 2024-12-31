#include "moment_to_rpm.hpp"

MomentToRPM::MomentToRPM(const ros::NodeHandle &nh)
:nh_(nh)
{
    // Get the parameters from the parameter server
    nh_.param("C_T", C_T_, 148e-9);
    nh_.param("C_M", C_M_, 2e-9);
    nh_.param("l", l_, 0.340);

    l_ = l_/2;

    Matrix4x4d K_forward;

    // Calculate the K matrix (f_z, moment -> rpm2)
    // K_forward << C_T_, C_T_, C_T_, C_T_,
    // C_T_*l_, C_T_*l_, -C_T_*l_, -C_T_*l_,
    // -C_T_*l_, -C_T_*l_, C_T_*l_, C_T_*l_,
    // -C_M_, C_M_, C_M_, -C_M_;

    // cout << K_forward << endl;

    // Calculate the inverse of the K matrix (rpm2 -> f_z, moment)
    // K_inverse_ = K_forward.inverse();

    K_inverse_ << 1/4.0/C_T_, 1/4.0/C_T_/l_, -1/4.0/C_T_/l_, -1/4.0/C_M_,
    1/4.0/C_T_, -1/4.0/C_T_/l_, -1/4.0/C_T_/l_, 1/4.0/C_M_,
    1/4.0/C_T_, -1/4.0/C_T_/l_, 1/4.0/C_T_/l_, -1/4.0/C_M_,
    1/4.0/C_T_, 1/4.0/C_T_/l_, 1/4.0/C_T_/l_, 1/4.0/C_M_;

    cout << K_inverse_ << endl;

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

MomentToRPM::~MomentToRPM()
{
    // Do nothing
}

void MomentToRPM::moment_callback(const Wrench::ConstPtr &msg)
{
    Vector3d moment;
    Vector4d f_z_moment;
    Vector4d rpm;

    // Get the moment from the message
    f_z_moment << msg->force.z, msg->torque.x, 
    msg->torque.y, msg->torque.z;

    // Convert the moment to rpm
    moment_to_rpm(f_z_moment, rpm);


    cmd_raw cmd;

    for(size_t i = 0; i < 4; i++)
    {
        if(rpm(i) >= max_rpm_)
            rpm(i) = max_rpm_;
        else if(rpm(i) <= 0)
            rpm(i) = min_rpm_;

        cmd.raw[i] = (int16_t) max_bit_/max_rpm_*rpm(i);
    }

    // Publish the cmd raw
    pub_cmd_raw_.publish(cmd);
}

void MomentToRPM::moment_to_rpm(const Vector4d &f_z_moment, Vector4d &rpm)
{
    Vector4d rpm2;
    rpm2 = K_inverse_ * f_z_moment;

    for(size_t i = 0; i < 4; i++)
    {
        if(rpm2(i) < min_rpm_*min_rpm_)
            rpm2(i) = min_rpm_*min_rpm_;
        rpm(i) = sqrt(rpm2(i));
    }

    cout << "rpm: " << rpm.transpose() << endl;
}