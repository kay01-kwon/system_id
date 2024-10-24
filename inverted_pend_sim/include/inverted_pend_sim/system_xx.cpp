#include "system_xx.hpp"

SystemXX::SystemXX()
{
}

void SystemXX::system_dynamics(const Vector2d &state, 
Vector2d &dsdt, 
const double &t)
{
    double theta = state(0);
    double theta_dot = state(1);

}
