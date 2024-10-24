#include "system_xx.hpp"

SystemXX::SystemXX(const InertialParams_t &inertial_params, 
const AeroCoeffs_t &aero_coeffs)
{
    inertial_params_ = inertial_params;
    aero_coeffs_ = aero_coeffs;
}

void SystemXX::system_dynamics(const Vector2d &state, 
Vector2d &dsdt, 
const double &t)
{
    double theta = state(0);
    double theta_dot = state(1);

}
