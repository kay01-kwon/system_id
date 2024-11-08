#include "trajectory_generator.hpp"

TrajectoryGeneration::TrajectoryGeneration(ros::NodeHandle &nh, 
double freq, double ampl)
:ref_(0.0), current_time_(0.0), last_time_(0.0), 
dt_(0.0), freq_(freq), ampl_(ampl)
{
    // Initialize the publisher
    reference_pub_ = nh.advertise<std_msgs::Float64>("/ref", 1);

    // Initalize the time
    current_time_ = ros::Time::now().toSec();
    last_time_ = ros::Time::now().toSec();

    // Initialize the time step
    dt_ = 1.0/freq_;
}

TrajectoryGeneration::~TrajectoryGeneration()
{
    // Empty destructor
}

void TrajectoryGeneration::ros_run()
{
    bool is_time_initialized = false;

    double t;

    while(ros::ok())
    {
        current_time_ = ros::Time::now().toSec();

        if(!is_time_initialized)
        {
            // Wait for 2 seconds to initialize the time
            if(current_time_ - last_time_ >= 2.0)
            {
                last_time_ = current_time_;
                is_time_initialized = true;
            }
        }

        if(is_time_initialized)
        {
            t = current_time_ - last_time_;
        }else
        {
            t = 0.0;
        }

        update_reference(t);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void TrajectoryGeneration::update_reference(const double &t)
{

    std_msgs::Float64 ref_msg;

    ref_msg.data = ampl_*sin(2*M_PI*freq_*t)*180.0/M_PI;
    ROS_INFO("Time: %f", t);

    reference_pub_.publish(ref_msg);
}