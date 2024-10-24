#ifndef SYSTEM_DYNAMICS_HPP
#define SYSTEM_DYNAMICS_HPP
#include "definitions/simulation_param_def.hpp"
#include <memory>

using std::unique_ptr;

class SystemDynamics
{
    public:

    SystemDynamics();

    virtual ~SystemDynamics();

    void set_params(const InertialParams_t& inertial_params,
                           const AeroCoeffs_t& aero_coeffs);

    virtual void system_dynamics(const Vector2d& state,
                                 Vector2d& dsdt,
                                 const double &t) = 0;

    static unique_ptr<SystemDynamics> createSystem(SystemType system_type);
    
    protected:

    void raw_to_rpm(const Vector4i16& cmd_raw, Vector4d& cmd_rpm);

    void rpm_to_moment(const Vector4d& cmd_rpm, Vector3d& moment);

    InertialParams_t inertial_params_;
    AeroCoeffs_t aero_coeffs_;

    Vector4d cmd_rpm_;
    Vector3d g_;

    double l_;

    Quaternionf quatf_;
    Vector3d w_;

    double raw_to_rpm_factor_;

};



#endif // SYSTEM_DYNAMICS_HPP