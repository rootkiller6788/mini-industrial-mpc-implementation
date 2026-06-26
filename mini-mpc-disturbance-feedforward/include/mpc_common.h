/**
 * @file mpc_common.h
 * @brief MPC Common Data Structures and Definitions
 *
 * Defines foundational data types for Model Predictive Control (MPC)
 * with disturbance feedforward. Covers L1 (Definitions) and L2 (Core Concepts).
 *
 * Reference: Rawlings, Mayne, Diehl, "Model Predictive Control: Theory,
 * Computation, and Design", 2nd Ed. (2017), Chapters 1-4.
 *
 * Course Mappings:
 * - MIT 6.302: State-space representations, optimal control structure
 * - Stanford ENGR205: MPC formulation fundamentals
 * - ETH: Constrained optimal control
 * - CMU 24-677: Advanced MPC with disturbance models
 *
 * @knowledge L1: MPC parameters, horizons, weighting matrices
 * @knowledge L2: Receding horizon, cost function, constraints
 */

#ifndef MPC_COMMON_H
#define MPC_COMMON_H

#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>

/* Flat-array accessor macros for 2D static arrays */
#define FLAT2D(M) ((const double*)(M))
#define FLAT2D_MUT(M) ((double*)(M))

/* Dimension limits: tuned to balance capability vs stack usage */
#define MPC_MAX_NX  20
#define MPC_MAX_NU  5
#define MPC_MAX_NY  10
#define MPC_MAX_ND  5
#define MPC_MAX_NP  20
#define MPC_MAX_NC  5

typedef struct {
    int     rows, cols;
    double  data[MPC_MAX_NX + MPC_MAX_NU + MPC_MAX_ND][MPC_MAX_NX + MPC_MAX_NU + MPC_MAX_ND];
} mpc_matrix_t;

typedef enum {
    MPC_SOLVE_SUCCESS         = 0,
    MPC_SOLVE_MAX_ITER        = 1,
    MPC_SOLVE_INFEASIBLE      = 2,
    MPC_SOLVE_UNBOUNDED       = 3,
    MPC_SOLVE_NUMERICAL_ERROR = 4,
    MPC_SOLVE_NOT_INITIALIZED = 5
} mpc_solve_status_t;

typedef enum {
    MPC_MODE_FEEDBACK_ONLY          = 0,
    MPC_MODE_OUTPUT_DISTURBANCE     = 1,
    MPC_MODE_INPUT_DISTURBANCE      = 2,
    MPC_MODE_STATE_DISTURBANCE      = 3,
    MPC_MODE_MEASURED_FEEDFORWARD   = 4,
    MPC_MODE_COMBINED_FF_DOB        = 5
} mpc_mode_t;

typedef struct {
    int     np, nc;
    double  ts;
    double  q_weight[MPC_MAX_NY];
    double  r_weight[MPC_MAX_NU];
    double  s_weight[MPC_MAX_NY];
    double  rdelta_weight[MPC_MAX_NU];
    double  rho_soft_constraint;
    int     enable_terminal_constraint;
    int     enable_rate_constraint;
} mpc_tuning_t;

typedef struct {
    int     nx, nu, ny, nd;
    double  A[MPC_MAX_NX][MPC_MAX_NX];
    double  B[MPC_MAX_NX][MPC_MAX_NU];
    double  C[MPC_MAX_NY][MPC_MAX_NX];
    double  D[MPC_MAX_NY][MPC_MAX_NU];
    double  Bd[MPC_MAX_NX][MPC_MAX_ND];
    double  Cd[MPC_MAX_NY][MPC_MAX_ND];
} mpc_ss_model_t;

typedef struct {
    double  u_min[MPC_MAX_NU];
    double  u_max[MPC_MAX_NU];
    double  du_min[MPC_MAX_NU];
    double  du_max[MPC_MAX_NU];
    double  y_min[MPC_MAX_NY];
    double  y_max[MPC_MAX_NY];
    double  x_min[MPC_MAX_NX];
    double  x_max[MPC_MAX_NX];
    int     enable_u, enable_du, enable_y, enable_x;
    int     use_soft_y, use_soft_x;
} mpc_constraints_t;

typedef struct {
    double  x[MPC_MAX_NX];
    double  x_aug[MPC_MAX_NX + MPC_MAX_ND];
    double  u_prev[MPC_MAX_NU];
    double  d_meas[MPC_MAX_ND];
    double  d_est[MPC_MAX_ND];
    double  y_ref[MPC_MAX_NY];
    double  u_ff[MPC_MAX_NU];
    double  u_opt[MPC_MAX_NC * MPC_MAX_NU];
    int     k;
} mpc_state_t;

typedef struct {
    int     nz, n_ineq, n_eq;
    double  H[MPC_MAX_NC * MPC_MAX_NU][MPC_MAX_NC * MPC_MAX_NU];
    double  f[MPC_MAX_NC * MPC_MAX_NU];
    double  A_ineq[MPC_MAX_NC * (MPC_MAX_NX + MPC_MAX_NU)][MPC_MAX_NC * MPC_MAX_NU];
    double  b_ineq[MPC_MAX_NC * (MPC_MAX_NX + MPC_MAX_NU)];
    double  A_eq[MPC_MAX_NC * MPC_MAX_NX][MPC_MAX_NC * MPC_MAX_NU];
    double  b_eq[MPC_MAX_NC * MPC_MAX_NX];
    double  z_lb[MPC_MAX_NC * MPC_MAX_NU];
    double  z_ub[MPC_MAX_NC * MPC_MAX_NU];
    double  z_opt[MPC_MAX_NC * MPC_MAX_NU];
    double  lambda_opt[MPC_MAX_NC * (MPC_MAX_NX + MPC_MAX_NU)];
    double  nu_opt[MPC_MAX_NC * MPC_MAX_NX];
    double  obj_value;
    int     max_iter;
    double  tol_opt, tol_feas;
} mpc_qp_t;

typedef struct {
    double  u_apply[MPC_MAX_NU];
    double  x_pred[MPC_MAX_NP][MPC_MAX_NX];
    double  y_pred[MPC_MAX_NP][MPC_MAX_NY];
    double  u_seq[MPC_MAX_NC][MPC_MAX_NU];
    double  cost;
    mpc_solve_status_t status;
    int     iterations;
} mpc_solution_t;

int mpc_validate_tuning(const mpc_tuning_t *tuning);
int mpc_validate_model(const mpc_ss_model_t *model);
void mpc_tuning_init_default(mpc_tuning_t *tuning);
void mpc_model_init(mpc_ss_model_t *model, int nx, int nu, int ny, int nd);
void mpc_constraints_init_default(mpc_constraints_t *cons, int nu, int ny, int nx);

#endif /* MPC_COMMON_H */
