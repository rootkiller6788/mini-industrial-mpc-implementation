/**
 * @file mpc_model.h
 * @brief MPC Model Operations: discretization, augmentation, prediction
 *
 * Implements state-space model transformations essential for MPC:
 * discretization via matrix exponential, disturbance augmentation,
 * prediction matrix construction, and step response models.
 *
 * Reference: Astrom and Wittenmark, "Computer-Controlled Systems" (1997)
 * Reference: Camacho and Bordons, "Model Predictive Control", 2nd Ed. (2007)
 *
 * Course Mappings:
 * - MIT 2.171: Digital control, discretization methods
 * - Stanford ENGR205: Model structures for MPC
 * - Berkeley ME233: Discrete-time LTI systems
 * - CMU 24-677: Model augmentation techniques
 *
 * @knowledge L1: State-space model definition
 * @knowledge L2: Model augmentation for offset-free control
 * @knowledge L3: Discretization structure (ZOH, Tustin)
 * @knowledge L4: Zero-order hold discretization theorem
 * @knowledge L5: Prediction matrix construction algorithm
 */

#ifndef MPC_MODEL_H
#define MPC_MODEL_H

#include "mpc_common.h"

/* Discretization method enumeration */
typedef enum {
    MPC_DISCRETIZE_ZOH    = 0,
    MPC_DISCRETIZE_FOH    = 1,
    MPC_DISCRETIZE_TUSTIN = 2
} mpc_discretize_method_t;

/**
 * Augmented model structure for offset-free MPC.
 * xa = [x; d]
 * Theorem (Internal Model Principle, Francis and Wonham 1976):
 * For zero steady-state offset, the controller must contain an
 * integrator for each output with nonzero reference/disturbance.
 */
typedef struct {
    int     nx_aug, nu, ny;
    double  Aa[MPC_MAX_NX][MPC_MAX_NX];
    double  Ba[MPC_MAX_NX][MPC_MAX_NU];
    double  Ca[MPC_MAX_NY][MPC_MAX_NX];
    double  Da[MPC_MAX_NY][MPC_MAX_NU];
} mpc_aug_model_t;

/**
 * Prediction matrices for MPC.
 * Y = Phi * x[k] + Gamma * DeltaU
 * Reference: Rawlings et al. (2017), Section 1.4
 */
typedef struct {
    int     np, nc, nx, nu, ny, nd;
    double  Phi[MPC_MAX_NP * MPC_MAX_NY][MPC_MAX_NX];
    double  Gamma[MPC_MAX_NP * MPC_MAX_NY][MPC_MAX_NC * MPC_MAX_NU];
    double  Phi_d[MPC_MAX_NP * MPC_MAX_NY][MPC_MAX_ND];
    double  Gamma_d[MPC_MAX_NP * MPC_MAX_NY][MPC_MAX_NC * MPC_MAX_ND];
    double  Psi[MPC_MAX_NP * MPC_MAX_NX][MPC_MAX_NC * MPC_MAX_NU];
    double  A_pow[MPC_MAX_NP][MPC_MAX_NX][MPC_MAX_NX];
} mpc_prediction_t;

/**
 * Step response model for DMC (Dynamic Matrix Control).
 * Reference: Cutler and Ramaker, "Dynamic Matrix Control" (1979)
 */
typedef struct {
    int     n, nu, ny;
    double  S[MPC_MAX_NP][MPC_MAX_NY][MPC_MAX_NU];
} mpc_step_model_t;

/* Matrix exponential via Pade approximation + scaling-and-squaring */
int mpc_matrix_expm(const double A[MPC_MAX_NX][MPC_MAX_NX], int n,
                    double E[MPC_MAX_NX][MPC_MAX_NX]);

/* Discretize continuous-time SS model to discrete-time */
int mpc_discretize_model(const double Ac[MPC_MAX_NX][MPC_MAX_NX],
                          const double Bc[MPC_MAX_NX][MPC_MAX_NU],
                          const double Cc[MPC_MAX_NY][MPC_MAX_NX],
                          const double Dc[MPC_MAX_NY][MPC_MAX_NU],
                          int nx, int nu, int ny, double Ts,
                          mpc_discretize_method_t method,
                          mpc_ss_model_t *model);

/* Build augmented model for offset-free MPC */
int mpc_build_augmented_model(const mpc_ss_model_t *plant,
                               mpc_mode_t mode,
                               mpc_aug_model_t *aug);

/* Construct MPC prediction matrices */
int mpc_build_prediction_matrices(const mpc_aug_model_t *model,
                                   const mpc_tuning_t *tuning,
                                   mpc_prediction_t *pred);

/* Compute steady-state targets for a given reference */
int mpc_compute_steady_state_target(const mpc_ss_model_t *model,
                                     const double y_ref[MPC_MAX_NY],
                                     double x_ss[MPC_MAX_NX],
                                     double u_ss[MPC_MAX_NU]);

/* Build step response model from state-space model */
int mpc_build_step_model(const mpc_ss_model_t *model, int n_points,
                          mpc_step_model_t *step);

/* Validate observability of augmented model (Kalman duality) */
int mpc_check_observability(const mpc_aug_model_t *aug);

/* Validate controllability of augmented model */
int mpc_check_controllability(const mpc_aug_model_t *aug);

/* Solve Discrete-time Algebraic Riccati Equation (DARE) */
int mpc_solve_dare(const double A[MPC_MAX_NX][MPC_MAX_NX],
                    const double B[MPC_MAX_NX][MPC_MAX_NU],
                    const double Q[MPC_MAX_NX][MPC_MAX_NX],
                    const double R[MPC_MAX_NU][MPC_MAX_NU],
                    int n, int nu,
                    double P[MPC_MAX_NX][MPC_MAX_NX],
                    int max_iter, double tol);

/* Compute infinity-norm condition number */
double mpc_matrix_cond(const double A[MPC_MAX_NX][MPC_MAX_NX], int n);

/* Model factory functions: pre-built canonical models */
void mpc_model_first_order(mpc_ss_model_t *model, double K, double tau, double Ts);
void mpc_model_second_order(mpc_ss_model_t *model, double K, double wn, double zeta, double Ts);
void mpc_model_integrator(mpc_ss_model_t *model, double K, double Ts);
void mpc_model_fopdt(mpc_ss_model_t *model, double K, double tau, double theta, double Ts);

/* Constraint management functions (declared here due to dependency on mpc_aug_model_t) */
void mpc_constraints_set_input(mpc_constraints_t *cons,
                                const double u_min[MPC_MAX_NU],
                                const double u_max[MPC_MAX_NU], int nu);
void mpc_constraints_set_rate(mpc_constraints_t *cons,
                               const double du_min[MPC_MAX_NU],
                               const double du_max[MPC_MAX_NU], int nu);
void mpc_constraints_set_output(mpc_constraints_t *cons,
                                 const double y_min[MPC_MAX_NY],
                                 const double y_max[MPC_MAX_NY],
                                 int ny, int use_soft);
void mpc_constraints_set_state(mpc_constraints_t *cons,
                                const double x_min[MPC_MAX_NX],
                                const double x_max[MPC_MAX_NX],
                                int nx, int use_soft);
int mpc_constraints_check_input(const mpc_constraints_t *cons,
                                 const double u[MPC_MAX_NU], int nu);
int mpc_constraints_check_rate(const mpc_constraints_t *cons,
                                const double u[MPC_MAX_NU],
                                const double u_prev[MPC_MAX_NU], int nu);
void mpc_constraints_project_input(const mpc_constraints_t *cons,
                                    double u[MPC_MAX_NU], int nu);
double mpc_constraints_violation_input(const mpc_constraints_t *cons,
                                        const double u[MPC_MAX_NU], int nu);
int mpc_constraints_terminal_set(const mpc_aug_model_t *aug,
                                  const mpc_tuning_t *tuning,
                                  double terminal_bound[MPC_MAX_NX + MPC_MAX_ND]);
int mpc_constraints_in_terminal_set(const mpc_constraints_t *cons,
                                     const double x[MPC_MAX_NX + MPC_MAX_ND],
                                     int nx);
double mpc_constraints_max_violation(const mpc_constraints_t *cons,
                                      const mpc_solution_t *sol,
                                      int nu, int ny, int nx);

#endif /* MPC_MODEL_H */
