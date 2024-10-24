#ifndef SYSTEM_ZZ_HPP
#define SYSTEM_ZZ_HPP
#include "system_dynamics.hpp"

class SystemZZ : public SystemDynamics
{
    public:

    SystemZZ();

    void system_dynamics(const Vector2d& state,
                         Vector2d& dsdt,
                         const double &t) override;

};


#endif // SYSTEM_ZZ_HPP