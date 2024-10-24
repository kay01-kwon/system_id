#ifndef SYSTEM_YY_HPP
#define SYSTEM_YY_HPP
#include "system_dynamics.hpp"

class SystemYY : public SystemDynamics
{
    public:

    SystemYY();

    SystemYY(const InertialParams_t &inertial_params,
             const AeroCoeffs_t &aero_coeffs);

    void system_dynamics(const Vector2d &state,
                         Vector2d &dsdt,
                         const double &dt) override;
};

#endif // SYSTEM_YY_HPP