#include "moment_control.hpp"

MomentController::MomentController(const ros::NodeHandle &nh)
:nh_(nh)
{
    // Get the parameters from the parameter server
    nh_.param("C_T", C_T_, 148e-9);
    nh_.param("C_M", C_M_, 2e-9);
    nh_.param("l", l_, 0.340);
    nh_.param("K_p", K_p_, 1.0);
    nh_.param("f_z", f_z_, 3.0);

    l_ = l_/2;

    K_inverse_ << 1/4.0/C_T_, 1/4.0/C_T_/l_, -1/4.0/C_T_/l_, 1/4.0/C_M_,
    1/4.0/C_T_, -1/4.0/C_T_/l_, -1/4.0/C_T_/l_, -1/4.0/C_M_,
    1/4.0/C_T_, -1/4.0/C_T_/l_, 1/4.0/C_T_/l_, 1/4.0/C_M_,
    1/4.0/C_T_, 1/4.0/C_T_/l_, 1/4.0/C_T_/l_, -1/4.0/C_M_;

    cout << K_inverse_ << endl;

    // Subscribe to the reference topic
    sub_ref_ = nh_.subscribe("/ref", 1, 
    &MomentController::ref_callback, this);

    // Subscribe to the imu topic
    sub_imu_ = nh_.subscribe("/imu/data", 1,
    &MomentController::imu_callback, this);

    // Publish the cmd_raw topic
    pub_cmd_raw_ = nh_.advertise<cmd_raw>("/cmd_raw", 1);

}

void MomentController::ros_run()
{
    while(ros::ok())
    {
        ros::spinOnce();
    }
}

MomentController::~MomentController()
{
    // Do nothing
}

void MomentController::ref_callback(const Float64::ConstPtr &msg)
{
    ref_ = msg->data;
    ref_ = ref_*M_PI/180;

}

void MomentController::imu_callback(const Imu::ConstPtr &msg)
{
    Vector3d moment;
    Vector4d f_z_moment;
    Vector4d rpm2;
    Vector4d rpm;

    double qx, qy, qz, qw;

    double psi;

    // Get the quaternion from the message
    qx = msg->orientation.x;
    qy = msg->orientation.y;
    qz = msg->orientation.z;
    qw = msg->orientation.w;

    // Calculate the psi angle from the quaternion
    psi = atan2(2*(qw*qz + qx*qy), 
    1 - 2*(qy*qy + qz*qz));

    // Only P control for the angle psi
    f_z_moment << f_z_, 0, 0, K_p_*(ref_ - psi);
    
    rpm2 = K_inverse_*f_z_moment;

    for(size_t i = 0; i < 4; i++)
    {
        if(rpm2(i) < min_rpm_*min_rpm_)
            rpm2(i) = min_rpm_*min_rpm_;
        rpm(i) = sqrt(rpm2(i));
    }

    cmd_raw cmd;

    for(size_t i = 0; i < 4; i++)
    {
        if(rpm(i) >= max_rpm_)
            rpm(i) = max_rpm_;
        else if(rpm(i) <= 0)
            rpm(i) = min_rpm_;
        cmd.raw[i] = (int16_t) max_bit_/max_rpm_*rpm(i);
    }

    pub_cmd_raw_.publish(cmd);

}