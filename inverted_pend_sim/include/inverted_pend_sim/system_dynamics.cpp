#include "system_dynamics.hpp"
#include "system_xx.hpp"
#include "system_yy.hpp"
#include "system_zz.hpp"

SystemDynamics::SystemDynamics()
:cmd_raw_(Vector4d::Zero()), quatf_(Quaternionf::Identity()), w_(Vector3d::Zero())
{
    g_<<0, 0, -9.81;
    raw_to_rpm_factor_ = double (MAX_RPM) / double (MAX_BIT);
}

SystemDynamics::~SystemDynamics()
{
}

void SystemDynamics::set_params(const InertialParams_t &inertial_params, 
const AeroCoeffs_t &aero_coeffs)
{
    inertial_params_ = inertial_params;
    aero_coeffs_ = aero_coeffs;
}

void SystemDynamics::set_cmd_raw(const Vector4i16 &cmd_raw)
{
    cmd_raw_ = cmd_raw;
}

void SystemDynamics::get_state(Quaternionf &quatf, Vector3d &w) const
{
    quatf = quatf_;
    w = w_;
}

unique_ptr<SystemDynamics> SystemDynamics::createSystem(SystemType system_type)
{

    switch(system_type)
    {
        case SystemType::SYSTEM_XX:
            return std::make_unique<SystemXX>();
        
        case SystemType::SYSTEM_YY:
            return std::make_unique<SystemYY>();

        case SystemType::SYSTEM_ZZ:
            return std::make_unique<SystemZZ>();

        default:
            std::cout<<"Invalid system type"<<std::endl;
            return nullptr;
    }
}

void SystemDynamics::raw_to_rpm(const Vector4i16 &cmd_raw, Vector4d &cmd_rpm)
{
    for(size_t i = 0; i < 4; i++)
    {
        cmd_rpm(i) = raw_to_rpm_factor_* cmd_raw(i);
    }
}

void SystemDynamics::rpm_to_moment(const Vector4d &cmd_rpm, Vector3d &moment)
{

    Vector4d rpm_squared = cmd_rpm.cwiseProduct(cmd_rpm);

    moment(0) = aero_coeffs_.l/2 * aero_coeffs_.C_T * 
    (rpm_squared(0) - rpm_squared(1) 
    - rpm_squared(2) + rpm_squared(3));

    moment(1) = aero_coeffs_.l/2 * 
    (-rpm_squared(0) - rpm_squared(1)
    + rpm_squared(2) + rpm_squared(3));

    moment(2) = aero_coeffs_.C_M *
    (rpm_squared(0) - rpm_squared(1)
    + rpm_squared(2) - rpm_squared(3));
    
}