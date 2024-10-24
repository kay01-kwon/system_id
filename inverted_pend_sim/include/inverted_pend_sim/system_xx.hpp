#ifndef SYSTEM_XX_HPP
#define SYSTEM_XX_HPP
#include "system_dynamics.hpp"

class SystemXX: public SystemDynamics
{
    public:

    SystemXX();

    void system_dynamics(const Vector2d &state, 
                         Vector2d &dsdt, 
                         const double &t) override;
    
    void state_to_quatf_w(const Vector2d &state,
                          Quaternionf &quatf,
                          Vector3d &w) override;

};


#endif // SYSTEM_XX_HPP