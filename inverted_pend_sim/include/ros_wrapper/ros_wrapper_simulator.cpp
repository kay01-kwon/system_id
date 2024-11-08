#include "ros_wrapper_simulator.hpp"

RosWrapperSimulator::RosWrapperSimulator(ros::NodeHandle &nh)
:s_(Vector2d::Zero()), quatf_(Quaternionf::Identity()), w_(Vector3d::Zero())
{
    // Subscribe to the raw command topic
    cmd_raw_sub_ = nh.subscribe("cmd_raw", 1, &RosWrapperSimulator::cmd_raw_callback, this);
    imu_pub_ = nh.advertise<Imu>("/mavros/imu/data", 1);

    string config_file, system_type_str;
    InertialParams_t inertial_params;
    AeroCoeffs_t aero_coeffs;
    SystemType system_type;
    double c;
    double perturb_state;

    nh.getParam("config_dir", config_file);
    nh.getParam("system_type", system_type_str);
    nh.getParam("perturb_state", perturb_state);
    nh.getParam("c", c);

    YAMLRead yaml_reader(config_file);
    yaml_reader.get_inertial_params(inertial_params);
    yaml_reader.get_aero_coeffs(aero_coeffs);

    if(system_type_str == "SYSTEM_XX")
    {
        system_type = SystemType::SYSTEM_XX;
    }
    else if(system_type_str == "SYSTEM_YY")
    {
        system_type = SystemType::SYSTEM_YY;
    }
    else if(system_type_str == "SYSTEM_ZZ")
    {
        system_type = SystemType::SYSTEM_ZZ;
    }
    else
    {
        ROS_ERROR("Invalid system type");
        ros::shutdown();
    }

    s_ << perturb_state, 0.0;

    system_dynamics_ = SystemDynamics::createSystem(system_type);

    system_dynamics_->set_params(inertial_params, aero_coeffs, c);

    current_time_ = ros::Time::now().toSec();
    last_time_ = current_time_;

}

RosWrapperSimulator::~RosWrapperSimulator()
{
    // Empty destructor
}

void RosWrapperSimulator::cmd_raw_callback(const cmd_rawConstPtr &msg)
{
    int16_t cmd_raw[4];

    for(size_t i = 0; i < 4; i++)
    {
        cmd_raw[i] = msg->raw[i];
    }

    system_dynamics_->set_cmd_raw(cmd_raw);
    
}

void RosWrapperSimulator::run()
{
    while(ros::ok())
    {
        update_state();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void RosWrapperSimulator::update_state()
{
    current_time_ = ros::Time::now().toSec();
    double dt_ = current_time_ - last_time_;

    stepper_.do_step([this](const Vector2d &s, Vector2d &dsdt, const double &t)
    {
        system_dynamics_->system_dynamics(s, dsdt, t);
    }, s_, last_time_, dt_);

    last_time_ = current_time_;

    system_dynamics_->state_to_quatf_w(s_,quatf_,w_);

    Imu imu_msg;

    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = "body_frame";

    imu_msg.orientation.w = quatf_.w();
    imu_msg.orientation.x = quatf_.x();
    imu_msg.orientation.y = quatf_.y();
    imu_msg.orientation.z = quatf_.z();

    imu_msg.angular_velocity.x = w_(0);
    imu_msg.angular_velocity.y = w_(1);
    imu_msg.angular_velocity.z = w_(2);

    imu_pub_.publish(imu_msg);
}
