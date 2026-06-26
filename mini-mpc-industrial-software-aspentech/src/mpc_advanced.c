/** @file mpc_advanced.c
 * @brief Advanced MPC: NMPC, Robust MPC, Adaptive MPC, MHE (L8)
 *
 * L8 Topics Implemented:
 *   1. Nonlinear MPC via successive linearization
 *   2. Robust MPC with scenario-based constraint tightening
 *   3. Adaptive MPC with online model update triggering
 *   4. Moving Horizon Estimation for state/disturbance estimation
 *
 * Ref:
 *   Rawlings/Mayne/Diehl (2017) Ch.7-9
 *   Mayne et al. (2000) - Constrained MPC: Stability and Optimality
 *   Bemporad & Morari (1999) - Robust MPC
 *   Rao & Rawlings (2002) - Moving Horizon Estimation
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include "mpc_common.h"
#include "mpc_advanced.h"

/* === Nonlinear MPC: Successive Linearization === */

int mpc_nmpc_config_init(mpc_nmpc_config_t *cfg, int nx, int nu, int ny)
{
    if (!cfg || nx < 1 || nu < 1 || ny < 1) return -1;
    cfg->nx = nx; cfg->nu = nu; cfg->ny = ny;
    cfg->max_linearizations = 10;
    cfg->A_lin = (double*)calloc(nx * nx, sizeof(double));
    cfg->B_lin = (double*)calloc(nx * nu, sizeof(double));
    cfg->C_lin = (double*)calloc(ny * nx, sizeof(double));
    cfg->x_op = (double*)calloc(nx, sizeof(double));
    cfg->u_op = (double*)calloc(nu, sizeof(double));
    cfg->y_op = (double*)calloc(ny, sizeof(double));
    if (!cfg->A_lin || !cfg->B_lin || !cfg->C_lin || !cfg->x_op || !cfg->u_op || !cfg->y_op) {
        mpc_nmpc_config_free(cfg); return -2;
    }
    for (int i = 0; i < nx; i++) cfg->A_lin[i*nx+i] = 1.0;
    for (int i = 0; i < ny && i < nx; i++) cfg->C_lin[i*nx+i] = 1.0;
    return 0;
}

void mpc_nmpc_config_free(mpc_nmpc_config_t *cfg)
{
    if (cfg) {
        free(cfg->A_lin); free(cfg->B_lin); free(cfg->C_lin);
        free(cfg->x_op); free(cfg->u_op); free(cfg->y_op);
        memset(cfg, 0, sizeof(mpc_nmpc_config_t));
    }
}

/* Linearize nonlinear model at (x_curr, u_curr) using finite differences.
 * A_ij = df_i/dx_j |_(x_curr,u_curr) approx (f_i(x+dx) - f_i(x-dx)) / (2*dx)
 *
 * For a user-defined nonlinear model, the user provides the
 * state transition and output functions. Here we use an
 * identity model for demonstration.
 */

int mpc_nmpc_linearize(mpc_nmpc_config_t *cfg,
    const double *x_curr, const double *u_curr)
{
    if (!cfg || !x_curr || !u_curr) return -1;
    int nx = cfg->nx, nu = cfg->nu, ny = cfg->ny;
    double eps = 1e-6;

    memcpy(cfg->x_op, x_curr, nx * sizeof(double));
    memcpy(cfg->u_op, u_curr, nu * sizeof(double));

    double *x_plus = (double*)calloc(nx, sizeof(double));
    double *x_minus = (double*)calloc(nx, sizeof(double));
    double *y_plus = (double*)calloc(ny, sizeof(double));
    double *y_minus = (double*)calloc(ny, sizeof(double));
    if (!x_plus || !x_minus || !y_plus || !y_minus) {
        free(x_plus); free(x_minus); free(y_plus); free(y_minus);
        return -2;
    }

    for (int j = 0; j < nx; j++) {
        memcpy(x_plus, x_curr, nx * sizeof(double));
        memcpy(x_minus, x_curr, nx * sizeof(double));
        x_plus[j] += eps; x_minus[j] -= eps;

        for (int i = 0; i < nx; i++) {
            double f_plus = 0.0, f_minus = 0.0;
            for (int k = 0; k < nx; k++) {
                f_plus += cfg->A_lin[i*nx+k] * x_plus[k];
                f_minus += cfg->A_lin[i*nx+k] * x_minus[k];
            }
            for (int k = 0; k < nu; k++) {
                f_plus += cfg->B_lin[i*nu+k] * u_curr[k];
                f_minus += cfg->B_lin[i*nu+k] * u_curr[k];
            }
            cfg->A_lin[i*nx+j] = (f_plus - f_minus) / (2.0 * eps);
        }
    }

    free(x_plus); free(x_minus); free(y_plus); free(y_minus);
    return 0;
}

/* === Robust MPC: Constraint Tightening === */

int mpc_robust_config_init(mpc_robust_config_t *cfg, int n_scenarios, double confidence)
{
    if (!cfg || n_scenarios < 1 || confidence <= 0.0 || confidence > 1.0) return -1;
    cfg->n_scenarios = n_scenarios;
    cfg->confidence_level = confidence;
    cfg->constraint_backoff = 0.05;
    cfg->scenario_perturbations = (double*)calloc(n_scenarios, sizeof(double));
    if (!cfg->scenario_perturbations) return -2;

    for (int i = 0; i < n_scenarios; i++) {
        double quantile = (i + 0.5) / n_scenarios;
        double z = 4.91 * (pow(quantile, 0.14) - pow(1.0 - quantile, 0.14));
        cfg->scenario_perturbations[i] = z;
    }
    return 0;
}

void mpc_robust_config_free(mpc_robust_config_t *cfg)
{
    if (cfg) {
        free(cfg->scenario_perturbations);
        memset(cfg, 0, sizeof(mpc_robust_config_t));
    }
}

/* Tighten inequality constraints by backoff margin.
 * b_tightened = b_lo + backoff, b_tightened = b_hi - backoff
 * This ensures constraint satisfaction under uncertainty.
 *
 * Theorem (Campi & Garatti 2008):
 *   With N >= (1/eps)*(n/(1-eta)-1) scenarios,
 *   probabilistic constraint satisfaction holds with confidence eta.
 */

int mpc_robust_tighten_constraints(mpc_robust_config_t *cfg,
    double *b_ineq, int n_constraints)
{
    if (!cfg || !b_ineq || n_constraints < 1) return -1;
    double backoff = cfg->constraint_backoff;
    double worst_case = 0.0;
    for (int i = 0; i < cfg->n_scenarios; i++) {
        if (fabs(cfg->scenario_perturbations[i]) > worst_case)
            worst_case = fabs(cfg->scenario_perturbations[i]);
    }
    backoff *= worst_case;
    for (int i = 0; i < n_constraints; i++) {
        b_ineq[i] -= backoff;
    }
    return 0;
}

/* === Adaptive MPC: Online Model Update Trigger === */

int mpc_adaptive_config_init(mpc_adaptive_config_t *cfg, int n_params)
{
    if (!cfg || n_params < 1) return -1;
    cfg->model_error_threshold = 0.1;
    cfg->min_samples_before_update = 50;
    cfg->adaptation_rate = 0.1;
    cfg->needs_update = 0;
    cfg->rls = mpc_rls_alloc(n_params, 0.99, 1.0);
    if (!cfg->rls) return -2;
    return 0;
}

void mpc_adaptive_config_free(mpc_adaptive_config_t *cfg)
{
    if (cfg) {
        mpc_rls_free(cfg->rls);
        memset(cfg, 0, sizeof(mpc_adaptive_config_t));
    }
}

int mpc_adaptive_check_update(mpc_adaptive_config_t *cfg, double current_rmse)
{
    if (!cfg) return -1;
    if (current_rmse > cfg->model_error_threshold) {
        cfg->needs_update = 1;
        return 1;
    }
    cfg->needs_update = 0;
    return 0;
}

/* === Moving Horizon Estimation (MHE) === */

int mpc_mhe_config_init(mpc_mhe_config_t *cfg, int N, int nx, int ny, int nu, int nd)
{
    if (!cfg || N < 2 || nx < 1 || ny < 1) return -1;
    cfg->N_horizon = N; cfg->nx = nx; cfg->ny = ny; cfg->nu = nu; cfg->nd = nd;
    cfg->x_est = (double*)calloc(nx, sizeof(double));
    cfg->d_est = (double*)calloc(nd, sizeof(double));
    cfg->y_buffer = (double*)calloc(N * ny, sizeof(double));
    cfg->u_buffer = (double*)calloc(N * nu, sizeof(double));
    if (!cfg->x_est || !cfg->y_buffer || !cfg->u_buffer) {
        mpc_mhe_config_free(cfg); return -2;
    }
    return 0;
}

void mpc_mhe_config_free(mpc_mhe_config_t *cfg)
{
    if (cfg) {
        free(cfg->x_est); free(cfg->d_est);
        free(cfg->y_buffer); free(cfg->u_buffer);
        memset(cfg, 0, sizeof(mpc_mhe_config_t));
    }
}

/* Estimate states and disturbances using moving horizon of past measurements.
 * MHE solves: min sum_{k=0}^{N-1} ||y_meas(k) - Cx(k)||^2_R + ||w(k)||^2_Q
 * subject to system dynamics x(k+1) = Ax(k) + Bu(k) + w(k)
 *
 * Simplified: uses averaging filter over recent errors as disturbance estimate.
 */

int mpc_mhe_estimate(mpc_mhe_config_t *cfg,
    const double *y_meas, const double *u_applied)
{
    if (!cfg || !y_meas) return -1;
    int N = cfg->N_horizon, ny = cfg->ny, nu = cfg->nu, nd = cfg->nd;

    memmove(cfg->y_buffer, cfg->y_buffer + ny, (N - 1) * ny * sizeof(double));
    memcpy(cfg->y_buffer + (N - 1) * ny, y_meas, ny * sizeof(double));

    if (u_applied && nu > 0) {
        memmove(cfg->u_buffer, cfg->u_buffer + nu, (N - 1) * nu * sizeof(double));
        memcpy(cfg->u_buffer + (N - 1) * nu, u_applied, nu * sizeof(double));
    }

    memset(cfg->x_est, 0, cfg->nx * sizeof(double));
    for (int i = 0; i < ny && i < cfg->nx; i++) {
        double avg = 0.0;
        for (int k = 0; k < N; k++) avg += cfg->y_buffer[k * ny + i];
        cfg->x_est[i] = avg / N;
    }

    if (nd > 0) {
        for (int i = 0; i < nd && i < ny; i++) {
            cfg->d_est[i] = y_meas[i] - cfg->x_est[i];
        }
    }
    return 0;
}
