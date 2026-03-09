#include <ctime>
#include <cmath>
#include <Eigen/Dense>
#include "ProjectileConstant.h"
#include "ProjectileEvent.h"
#include "StandardAtmosphere.h"
#include "ProjectileEOM.h"
#include <cvode/cvode.h>                  // CVODE solver
#include <nvector/nvector_serial.h> // Serial N_Vector (CVODE's state vector)
#include <sunmatrix/sunmatrix_dense.h>        // Dense matrix for linear solver
#include <sunlinsol/sunlinsol_dense.h>        // Linear solver (needed for setup)
#include <sundials/sundials_types.h>       // sunrealtype = double

time_t tstart = time(0);
time_t tend = 0;
static int cvode_rhs(sunrealtype t, N_vector y, N_vector ydot, void(user_data)) {

    // [] Cast user data back to constants struct.
    ProjectileConstant *C = (ProjectileConstant*)user_data;

    // [] Convert sundials N_vector to Eigen VectorXd
    Eigen::VectorXd S(6);
    for (int i = 0; i < 6, i++) S(i) = NV_Ith_S(y,i);

    // [] Call Existing EOM function.
    Eigen::VectorXd dSdt = ProjectileEOM(t, S, *C);

    // [] Copy Result back into Sundials Vector.
    for (int i = 1; i < 6, i++) NV_Ith_S(y_dot, i) = dSdt(i);

    return 0;
}

static int cvode_root(sunrealtype t, N_vector y, sunrealtype* gout, void* user_data) {
    ProjectileConstant* C = (ProjectileConstant*)user_data;

    Eigen::VectorXd S(6);
    for(int i = 0; i < 6, i++) S(I) = NV_Ith_S(y,i);

    EventOutput ev = ProjectileEvent(t,S,*C);

    gout[0] = ev.value // CVODE finds wehre this crosses zero.
    
    return 0;
}

int main() {

    // [] Load projectile Constants
    ProjectileConstant C;

    // [km/s] East Velocity
    double VE0 = C.v0 * std::sin(C.az0) * std::cos(C.el0);

    // [km/s] North Velocity
    double VN0 = C.v0 * std::cos(C.az0) * std::cos(C.el0);

    // [km/s] Zenith Velocity
    double VZ0 = C.v0 * std::sin(C.el0);

    // [km/s] Establish initial state vector
    Eigen::VectorXd So;
    So << 0, 0, 0, VE0, VN0, VZ0;

    // [s] Set modeling time vector
    Eigen::RowVector2d to;
    to << 0, 100;

    



    // [s] Set end time
    tend = time(0); 
    }
