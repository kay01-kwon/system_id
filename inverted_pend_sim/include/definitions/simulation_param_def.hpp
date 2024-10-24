#ifndef SIMULATION_PARAM_DEF_HPP
#define SIMULATION_PARAM_DEF_HPP

#include "eigen_type_def.hpp"

#define MAX_BIT 8191
#define MAX_RPM 9800


enum class SystemType
{
    SYSTEM_XX,
    SYSTEM_YY,
    SYSTEM_ZZ
};

typedef struct _InertialParams
{
    double m;
    Matrix3d J;
    Vector3d CG_r_CM;
} InertialParams_t;

// Aerodynamic coefficients
// C_T: Thrust coefficient
// C_M: Moment coefficient
// l: Distance from the center of geometry to the center of thrust
typedef struct _AeroCoffs
{
    double C_T;
    double C_M;
    double s;
} AeroCoeffs_t;

#endif // SIMULATION_PARAM_DEF_HPP