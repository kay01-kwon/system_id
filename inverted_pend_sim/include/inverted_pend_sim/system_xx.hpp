#ifndef SYSTEM_XX_HPP
#define SYSTEM_XX_HPP
#include "system_dynamics.hpp"

class SystemXX: public SystemDynamics
{
    public:

    SystemXX();

    SystemXX(const InertialParams_t &inertial_params, 
             const AeroCoeffs_t &aero_coeffs);

    void system_dynamics(const Vector2d &state, 
                         Vector2d &dsdt, 
                         const double &t) override;

};


#endif // SYSTEM_XX_HPP