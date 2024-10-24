#include "system_yy.hpp"

SystemYY::SystemYY()
{
}

void SystemYY::system_dynamics(const Vector2d &state, 
Vector2d &dsdt, 
const double &dt)
{
    Vector4d cmd_rpm;
    Vector3d moment;

    double m = inertial_params_.m;
    double J_yy = inertial_params_.J(1,1);
    double x_CM = inertial_params_.CG_r_CM(0);
    double z_CM = inertial_params_.CG_r_CM(2);


    double phi = state(0);
    double phi_dot = state(1);

    raw_to_rpm(cmd_raw_, cmd_rpm);
    rpm_to_moment(cmd_rpm, moment);

    double tau_y = moment(1);

    dsdt(0) = phi_dot;
    dsdt(1) = 1/J_yy*(tau_y + m*g_(2)*(x_CM*cos(phi) + z_CM*sin(phi)));
}

void SystemYY::state_to_quatf_w(const Vector2d &state, 
Quaternionf &quatf, 
Vector3d &w)
{
    double phi = state(0);
    double phi_dot = state(1);

    double qw = cos(phi/2);
    double qx = 0;
    double qy = sin(phi/2);
    double qz = 0;

    quatf = Quaternionf(qw, qx, qy, qz);
    w = Vector3d(0, phi_dot, 0);
}
