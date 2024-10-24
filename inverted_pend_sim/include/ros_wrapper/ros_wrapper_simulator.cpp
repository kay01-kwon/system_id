#include "ros_wrapper_simulator.hpp"

RosWrapperSimulator::RosWrapperSimulator(ros::NodeHandle &nh)
{
    // Subscribe to the raw command topic
    cmd_raw_sub = nh.subscribe("cmd_raw", 1, &RosWrapperSimulator::cmd_raw_callback, this);
    imu_pub = nh.advertise<Imu>("imu", 1);

    string config_file, system_type_str;

    SystemType system_type;

    InertialParams_t inertial_params;
    AeroCoeffs_t aero_coeffs;

    nh.getParam("config_dir", config_file);
    nh.getParam("system_type", system_type_str);
    
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

    system_dynamics_ = SystemDynamics::createSystem(system_type);

}

RosWrapperSimulator::~RosWrapperSimulator()
{
    // Empty destructor
}

void RosWrapperSimulator::cmd_raw_callback(const cmd_rawConstPtr &msg)
{
    Vector4i16 cmd_raw;

    cmd_raw << msg->raw[0],
               msg->raw[1],
               msg->raw[2],
               msg->raw[3];

    system_dynamics_->set_cmd_raw(cmd_raw);
    
}

void RosWrapperSimulator::update_state()
{
    
}
