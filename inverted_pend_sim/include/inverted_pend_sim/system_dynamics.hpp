#ifndef SYSTEM_DYNAMICS_HPP
#define SYSTEM_DYNAMICS_HPP
#include "definitions/simulation_param_def.hpp"
#include <boost/make_unique.hpp>

class SystemDynamics
{
    public:

    SystemDynamics();

    virtual ~SystemDynamics();

    virtual void system_dynamics(const Vector2d& state,
                                 Vector2d& dsdt,
                                 const double &t) = 0;

    static SystemDynamics* createSystem(SystemType system_type,
                                        const InertialParams_t& inertial_params,
                                        const AeroCoeffs_t& aero_coeffs);
    
    protected:

    void raw_to_rpm(const Vector4i16& cmd_raw, Vector4d& cmd_rpm);

    private:

    InertialParams_t inertial_params_;
    AeroCoeffs_t aero_coeffs_;

    Vector4d cmd_rpm_;
    Quaternionf quatf_;

    double raw_to_rpm_factor_;

};



#endif // SYSTEM_DYNAMICS_HPP