#include "system_xx.hpp"

SystemXX::SystemXX()
{
}

void SystemXX::system_dynamics(const Vector2d &state, 
Vector2d &dsdt, 
const double &t)
{

    Vector4d cmd_rpm;
    Vector3d moment;

    double m = inertial_params_.m;
    double J_xx = inertial_params_.J(0,0);
    double y_CM = inertial_params_.CG_r_CM(1);
    double z_CM = inertial_params_.CG_r_CM(2);


    double theta = state(0);
    double theta_dot = state(1);

    raw_to_rpm(cmd_raw_, cmd_rpm);
    rpm_to_moment(cmd_rpm, moment);

    double tau_x = moment(0);

    dsdt(0) = theta_dot;
    dsdt(1) = 1/J_xx*(tau_x + m*g_(2)*(y_CM*cos(theta) + z_CM*sin(theta)));
}

void state_to_quatf_w(const Vector2d &state,
Quaternionf &quatf,
Vector3d &w)
{
    double theta = state(0);
    double theta_dot = state(1);

    double qw = cos(theta/2);
    double qx = sin(theta/2);
    double qy = 0;
    double qz = 0;

    quatf = Quaternionf(qw, qx, qy, qz);
    w = Vector3d(theta_dot, 0, 0);

}
