#include "system_zz.hpp"

SystemZZ::SystemZZ()
{
}

void SystemZZ::system_dynamics(const Vector2d &state, 
Vector2d &dsdt, 
const double &t)
{
    Vector4d cmd_rpm;
    Vector3d moment;

    double m = inertial_params_.m;
    double J_zz = inertial_params_.J(2,2);

    double psi = state(0);
    double psi_dot = state(1);

    raw_to_rpm(cmd_raw_, cmd_rpm);
    rpm_to_moment(cmd_rpm, moment);

    double tau_z = moment(2);

    dsdt(0) = psi_dot;
    dsdt(1) = 1/J_zz*(tau_z - c_*psi_dot);
}

void SystemZZ::state_to_quatf_w(const Vector2d &state, 
Quaternionf &quatf, 
Vector3d &w)
{
    double psi = state(0);
    double psi_dot = state(1);

    double qw = cos(psi/2);
    double qx = 0;
    double qy = 0;
    double qz = sin(psi/2);

    quatf = Quaternionf(qw, qx, qy, qz);
    w << 0, 0, psi_dot;
}
