/** @file mpc_advanced.h
 * @brief Advanced MPC Features: NMPC, Robust MPC, Adaptive MPC, MHE (L8)
 * Ref: Rawlings/Mayne/Diehl (2017) Ch.7-9, Astrom and Wittenmark (2008)
 */
#ifndef MPC_ADVANCED_H
#define MPC_ADVANCED_H
#include "mpc_common.h"
#ifdef __cplusplus
extern "C" {
#endif

/* Nonlinear MPC: successive linearization */
typedef struct {
    int nx, nu, ny;
    double *A_lin, *B_lin, *C_lin;
    double *x_op, *u_op, *y_op;
    int max_linearizations;
} mpc_nmpc_config_t;

/* Robust MPC: scenario-based constraint tightening */
typedef struct {
    int n_scenarios;
    double confidence_level;
    double *scenario_perturbations;
    double constraint_backoff;
} mpc_robust_config_t;

/* Adaptive MPC: online model update trigger */
typedef struct {
    double model_error_threshold;
    int min_samples_before_update;
    double adaptation_rate;
    mpc_rls_estimator_t *rls;
    int needs_update;
} mpc_adaptive_config_t;

/* Moving Horizon Estimation (MHE) */
typedef struct {
    int N_horizon, nx, ny, nu, nd;
    double *x_est, *d_est;
    double *y_buffer, *u_buffer;
} mpc_mhe_config_t;

int mpc_nmpc_config_init(mpc_nmpc_config_t *cfg, int nx, int nu, int ny);
void mpc_nmpc_config_free(mpc_nmpc_config_t *cfg);
int mpc_nmpc_linearize(mpc_nmpc_config_t *cfg, const double *x_curr, const double *u_curr);
int mpc_robust_config_init(mpc_robust_config_t *cfg, int n_scenarios, double confidence);
void mpc_robust_config_free(mpc_robust_config_t *cfg);
int mpc_robust_tighten_constraints(mpc_robust_config_t *cfg, double *b_ineq, int n_constraints);
int mpc_adaptive_config_init(mpc_adaptive_config_t *cfg, int n_params);
void mpc_adaptive_config_free(mpc_adaptive_config_t *cfg);
int mpc_adaptive_check_update(mpc_adaptive_config_t *cfg, double current_rmse);
int mpc_mhe_config_init(mpc_mhe_config_t *cfg, int N, int nx, int ny, int nu, int nd);
void mpc_mhe_config_free(mpc_mhe_config_t *cfg);
int mpc_mhe_estimate(mpc_mhe_config_t *cfg, const double *y_meas, const double *u_applied);

#ifdef __cplusplus
}
#endif
#endif
