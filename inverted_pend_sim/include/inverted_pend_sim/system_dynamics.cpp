#include "system_dynamics.hpp"
#include "system_xx.hpp"
#include "system_yy.hpp"
#include "system_zz.hpp"

SystemDynamics::SystemDynamics()
{
    raw_to_rpm_factor_ = double (MAX_RPM) / double (MAX_BIT);
}

SystemDynamics::~SystemDynamics()
{
}

SystemDynamics *SystemDynamics::createSystem(SystemType system_type, 
const InertialParams_t &inertial_params, 
const AeroCoeffs_t &aero_coeffs)
{
    
    switch(system_type)
    {
        case SystemType::SYSTEM_XX:
            // return boost::make_unique<SystemXX>(inertial_params, aero_coeffs);
        case SystemType::SYSTEM_YY:
            // return boost::make_unique<Quadrotor>(inertial_params, aero_coeffs);
        case SystemType::SYSTEM_ZZ:

        default:
            break;
    }

    return nullptr;
}

void SystemDynamics::raw_to_rpm(const Vector4i16 &cmd_raw, Vector4d &cmd_rpm)
{
    for(size_t i = 0; i < 4; i++)
    {
        cmd_rpm(i) = raw_to_rpm_factor_* cmd_raw(i);
    }
}
