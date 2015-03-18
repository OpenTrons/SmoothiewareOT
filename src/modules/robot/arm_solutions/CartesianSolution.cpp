#include "CartesianSolution.h"
#include "Kernel.h"
#include "StreamOutputPool.h"
#include <math.h>

void CartesianSolution::cartesian_to_actuator( float cartesian_mm[], float actuator_mm[] ){
    actuator_mm[ALPHA_STEPPER] = cartesian_mm[X_AXIS];
    actuator_mm[BETA_STEPPER ] = cartesian_mm[Y_AXIS];
    actuator_mm[GAMMA_STEPPER] = cartesian_mm[Z_AXIS];

	actuator_mm[CHI_STEPPER  ] = cartesian_mm[A_AXIS];
	actuator_mm[PSI_STEPPER  ] = cartesian_mm[B_AXIS];
	actuator_mm[OMEGA_STEPPER] = cartesian_mm[C_AXIS];
}

void CartesianSolution::actuator_to_cartesian( float actuator_mm[], float cartesian_mm[] ){
    cartesian_mm[ALPHA_STEPPER] = actuator_mm[X_AXIS];
    cartesian_mm[BETA_STEPPER ] = actuator_mm[Y_AXIS];
    cartesian_mm[GAMMA_STEPPER] = actuator_mm[Z_AXIS];

	cartesian_mm[CHI_STEPPER  ] = actuator_mm[A_AXIS];
	cartesian_mm[PSI_STEPPER  ] = actuator_mm[B_AXIS];
	cartesian_mm[OMEGA_STEPPER] = actuator_mm[C_AXIS];

}
