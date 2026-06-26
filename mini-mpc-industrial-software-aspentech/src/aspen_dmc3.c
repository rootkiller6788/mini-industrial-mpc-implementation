/** @file aspen_dmc3.c
 * @brief AspenTech DMC3 Integration (L7 Industrial Application)
 * Simulates DMC3 execution cycle: Read PVs -> Bias Update -> LP -> QP -> Write MVs
 * Ref: AspenTech DMC3 User Manual, Qin and Badgwell (2003)
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include "mpc_common.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int mpc_dmc3_read_inputs(mpc_aspen_config_t *cfg,
    double *cv_values, double *dv_values, double *mv_feedback)
{
    if (!cfg || !cv_values) return -1;
    int n_cv = cfg->n_cv, n_dv = cfg->n_dv, n_mv = cfg->n_mv;
    for (int i = 0; i < n_cv; i++) cv_values[i] = cfg->cv_config[i].current_value;
    if (dv_values)
        for (int i = 0; i < n_dv; i++) dv_values[i] = cfg->dv_config[i].current_value;
    if (mv_feedback)
        for (int i = 0; i < n_mv; i++) mv_feedback[i] = cfg->mv_config[i].current_value;
    return 0;
}

int mpc_dmc3_write_outputs(mpc_aspen_config_t *cfg, const double *mv_output)
{
    if (!cfg || !mv_output) return -1;
    for (int i = 0; i < cfg->n_mv; i++) cfg->mv_config[i].current_value = mv_output[i];
    return 0;
}

/* Box-Muller Gaussian noise generator for process simulation */
static double randn(void)
{
    static unsigned long seed = 12345;
    seed = (1103515245 * seed + 12345) & 0x7fffffff;
    double u1 = (double)seed / 0x7fffffff;
    seed = (1103515245 * seed + 12345) & 0x7fffffff;
    double u2 = (double)seed / 0x7fffffff;
    return sqrt(-2.0 * log(u1 + MPC_EPS)) * cos(2.0 * M_PI * u2);
}

int mpc_dmc3_simulate_process(mpc_aspen_config_t *cfg,
    double *cv_noisy, double noise_std)
{
    if (!cfg || !cv_noisy) return -1;
    int n_cv = cfg->n_cv, n_mv = cfg->n_mv;

    for (int cv = 0; cv < n_cv; cv++) {
        double y = cfg->cv_config[cv].current_value;
        for (int mv = 0; mv < n_mv; mv++) {
            mpc_step_model_t *sm = &cfg->model.sub_models[cv][mv];
            if (!sm->coeff || sm->n_coeffs < 1) continue;
            double du = cfg->mv_config[mv].current_value - cfg->mv_config[mv].setpoint;
            y += sm->gain_ss * du;
        }
        double noise = (noise_std > MPC_EPS) ? noise_std * randn() : 0.0;
        cv_noisy[cv] = y + noise;
        cfg->cv_config[cv].current_value = cv_noisy[cv];
    }
    return 0;
}

int mpc_dmc3_closed_loop_simulation(mpc_aspen_config_t *cfg,
    int n_steps, double noise_std,
    double *cv_history, double *mv_history)
{
    if (!cfg || n_steps < 1 || !cv_history || !mv_history) return -1;
    int n_cv = cfg->n_cv, n_mv = cfg->n_mv;

    mpc_controller_state_t *cs = mpc_controller_state_alloc(
        cfg->P, cfg->M, cfg->model_horizon, n_mv, n_cv, cfg->n_dv);
    if (!cs) return -2;
    cs->mode = MPC_MODE_RUNNING;
    cs->bias_filter_gain = cfg->bias_filter_gain;

    for (int cv = 0; cv < n_cv; cv++)
        for (int p = 0; p < cfg->P; p++)
            cs->y_ref[cv * cfg->P + p] = cfg->cv_config[cv].setpoint;

    double *cv_measured = (double*)calloc(n_cv, sizeof(double));
    double *dv_measured = (double*)calloc(cfg->n_dv, sizeof(double));
    double *mv_output = (double*)calloc(n_mv, sizeof(double));
    if (!cv_measured || !dv_measured || !mv_output) {
        mpc_controller_state_free(cs);
        free(cv_measured); free(dv_measured); free(mv_output);
        return -3;
    }

    for (int step = 0; step < n_steps; step++) {
        mpc_dmc3_simulate_process(cfg, cv_measured, noise_std);
        mpc_dmc3_read_inputs(cfg, cv_measured, dv_measured, NULL);
        mpc_dmc_step(cfg, cs, cv_measured, dv_measured, mv_output, NULL);
        mpc_dmc3_write_outputs(cfg, mv_output);
        for (int cv = 0; cv < n_cv; cv++)
            cv_history[step * n_cv + cv] = cv_measured[cv];
        for (int mv = 0; mv < n_mv; mv++)
            mv_history[step * n_mv + mv] = mv_output[mv];
    }

    mpc_controller_state_free(cs);
    free(cv_measured); free(dv_measured); free(mv_output);
    return 0;
}

/* Orthogonal move calculation (AspenTech proprietary feature) */
int mpc_orthogonal_move_compute(const mpc_dynamic_matrix_t *dm,
    const double *y_err, double *du_orth, double alpha)
{
    if (!dm || !y_err || !du_orth || alpha <= 0.0) return -1;
    int P = dm->P, M = dm->M;
    for (int j = 0; j < M; j++) {
        double sum = 0.0;
        for (int i = 0; i < P; i++) sum += dm->A[i*M+j] * y_err[i];
        du_orth[j] = alpha * sum;
    }
    return 0;
}
