#ifndef SYSTEM_ZZ_HPP
#define SYSTEM_ZZ_HPP
#include "system_dynamics.hpp"

class SystemZZ : public SystemDynamics
{
    public:

    SystemZZ();

    SystemZZ(const InertialParams_t &inertial_params, 
             const AeroCoeffs_t &aero_coeffs);
};


#endif // SYSTEM_ZZ_HPP