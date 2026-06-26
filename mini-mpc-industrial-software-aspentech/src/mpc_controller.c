/** @file mpc_controller.c
 * @brief Core DMC Controller — Prediction, Bias Update, Horizon Shift, Move Implementation
 * Knowledge: L2 (Receding Horizon, Bias Feedback Correction),
 *            L5 (DMC Algorithm, Prediction Equation, QP Integration)
 *
 * Implements the complete DMC execution cycle:
 *   1. Read CV measurements and DV feedforward values
 *   2. Compute open-loop prediction y_free
 *   3. Apply bias correction: y_corr = y_free + b
 *   4. Build QP matrices (H, c, constraints)
 *   5. Solve QP -> delta_u_opt
 *   6. Implement first move, clip and rate-limit
 *   7. Shift horizon for next cycle
 *
 * Ref: Rawlings/Mayne/Diehl (2017), Cutler/Ramaker (1980)
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include "mpc_common.h"

/* ===== L1: Memory Management ===== */

mpc_controller_state_t* mpc_controller_state_alloc(int P, int M,
    int model_horizon, int n_mv, int n_cv, int n_dv)
{
    if (P < 1 || M < 1 || M > P || model_horizon < 1 || n_mv < 0 || n_cv < 1) return NULL;

    mpc_controller_state_t *cs = (mpc_controller_state_t*)calloc(1, sizeof(mpc_controller_state_t));
    if (!cs) return NULL;

    cs->P = P; cs->M = M; cs->model_horizon = model_horizon;
    cs->n_mv = n_mv; cs->n_cv = n_cv; cs->n_dv = n_dv;
    cs->mode = MPC_MODE_IDLE;
    cs->bias_filter_gain = 0.5;

    cs->delta_u_past   = (double*)calloc(model_horizon * n_mv, sizeof(double));
    cs->cv_error_past  = (double*)calloc(P * n_cv, sizeof(double));
    cs->y_openloop     = (double*)calloc(P * n_cv, sizeof(double));
    cs->y_corrected    = (double*)calloc(P * n_cv, sizeof(double));
    cs->y_ref          = (double*)calloc(P * n_cv, sizeof(double));
    cs->delta_u_opt    = (double*)calloc(M * n_mv, sizeof(double));
    cs->past_unforced  = (double*)calloc(model_horizon * n_cv, sizeof(double));

    if (!cs->delta_u_past || !cs->y_openloop || !cs->y_corrected || !cs->y_ref ||
        !cs->delta_u_opt || !cs->past_unforced) {
        mpc_controller_state_free(cs); return NULL;
    }
    return cs;
}

void mpc_controller_state_free(mpc_controller_state_t *cs)
{
    if (cs) {
        free(cs->delta_u_past);
        free(cs->cv_error_past);
        free(cs->y_openloop);
        free(cs->y_corrected);
        free(cs->y_ref);
        free(cs->delta_u_opt);
        free(cs->past_unforced);
        free(cs);
    }
}

/* ===== L2: Open-Loop Prediction =====
 *
 * y_free(k+i|k) = effect of past MV moves only (assuming future du=0)
 *
 * For each CV c and prediction step p:
 *   y_free[c*P + p] = sum_{j=0}^{N-1} s_{p+j} * du_past[N-1-j]
 *   + sum_{dv} G_dv * dv_current
 */

int mpc_dmc_predict(const mpc_aspen_config_t *cfg,
    const mpc_controller_state_t *cs, double *y_pred)
{
    if (!cfg || !cs || !y_pred) return -1;
    int n_cv = cfg->n_cv;
    int n_mv = cfg->n_mv;
    int P = cfg->P;
    int N = cfg->model_horizon;

    memset(y_pred, 0, P * n_cv * sizeof(double));

    for (int cv = 0; cv < n_cv; cv++) {
        for (int mv = 0; mv < n_mv; mv++) {
            mpc_step_model_t *sm = &cfg->model.sub_models[cv][mv];
            if (!sm->coeff || sm->n_coeffs < 1) continue;

            for (int p = 0; p < P; p++) {
                double sum = 0.0;
                for (int j = 0; j < N && j < sm->n_coeffs; j++) {
                    int cidx = p + j;
                    double s = (cidx < sm->n_coeffs) ? sm->coeff[cidx] : sm->coeff[sm->n_coeffs - 1];
                    int past_idx = N - 1 - j;
                    sum += s * cs->delta_u_past[past_idx * n_mv + mv];
                }
                y_pred[cv * P + p] += sum;
            }
        }
    }
    return 0;
}

/* ===== L2: Bias Update (Feedback Correction) =====
 *
 * AspenTech DMC3 uses exponential bias filtering:
 *   b(k) = K_f * b(k-1) + (1 - K_f) * (y_meas(k) - y_pred_1step(k))
 *
 * where K_f in [0,1] is the bias filter gain.
 * K_f = 0: no filtering (full correction each step, noisy)
 * K_f = 0.3-0.7: typical industrial setting
 * K_f = 0.9: heavy filtering (slow to correct model error)
 *
 * Theorem (Bias convergence):
 *   If |K_f| < 1 and model error is constant d,
 *   then b(k) -> d as k -> infinity. Convergence rate ~ |K_f|^k.
 */

int mpc_dmc_bias_update(mpc_controller_state_t *cs,
    const double *y_measured, const double *y_pred_1step, double bias_gain)
{
    if (!cs || !y_measured || !y_pred_1step) return -1;
    if (bias_gain < 0.0 || bias_gain > 1.0) return -2;

    int n_cv = cs->n_cv;
    double one_minus_Kf = 1.0 - bias_gain;

    for (int cv = 0; cv < n_cv; cv++) {
        double error = y_measured[cv] - y_pred_1step[cv];
        cs->cv_error_past[cv] = bias_gain * cs->cv_error_past[cv] + one_minus_Kf * error;
    }
    return 0;
}

/* ===== L2: Horizon Shift =====
 *
 * After implementing du(k|k), shift the past move buffer:
 *   du_past[0..N-2] <- du_past[1..N-1]
 *   du_past[N-1] <- du(k|k) (the implemented first move)
 *
 * This prepares for the next control cycle (receding horizon).
 */

int mpc_dmc_shift_horizon(mpc_controller_state_t *cs, int n_mv)
{
    if (!cs || n_mv < 1) return -1;
    int N = cs->model_horizon;

    for (int i = 0; i < N - 1; i++) {
        for (int j = 0; j < n_mv; j++) {
            cs->delta_u_past[i * n_mv + j] = cs->delta_u_past[(i + 1) * n_mv + j];
        }
    }

    for (int j = 0; j < n_mv; j++) {
        cs->delta_u_past[(N - 1) * n_mv + j] = cs->delta_u_opt[j];
    }
    return 0;
}

/* ===== L2: First Move Implementation =====
 *
 * Extracts du(k|k) from the optimal move sequence,
 * applies rate-of-change limits and hard bounds,
 * and writes to the MV output array.
 *
 * Clipping hierarchy (AspenTech DMC3 convention):
 *   1. Rate limit  (du_lo <= du_raw <= du_hi)
 *   2. Hard bounds (mv_lo <= mv_new <= mv_hi)
 *   3. No move if MV disabled or controller not running
 */

int mpc_implement_first_move(const mpc_controller_state_t *cs,
    const mpc_mv_config_t *mv_cfg, double *mv_output)
{
    if (!cs || !mv_cfg || !mv_output) return -1;

    if (cs->mode != MPC_MODE_RUNNING) {
        for (int i = 0; i < cs->n_mv; i++) {
            mv_output[i] = mv_cfg[i].current_value;
        }
        return 0;
    }

    for (int i = 0; i < cs->n_mv; i++) {
        if (!mv_cfg[i].is_enabled) {
            mv_output[i] = mv_cfg[i].current_value;
            continue;
        }

        double du_raw = cs->delta_u_opt[i];

        if (du_raw < mv_cfg[i].rate_lo) du_raw = mv_cfg[i].rate_lo;
        if (du_raw > mv_cfg[i].rate_hi) du_raw = mv_cfg[i].rate_hi;

        double mv_new = mv_cfg[i].current_value + du_raw;

        if (mv_new < mv_cfg[i].lo_limit) {
            du_raw = mv_cfg[i].lo_limit - mv_cfg[i].current_value;
            mv_new = mv_cfg[i].lo_limit;
        }
        if (mv_new > mv_cfg[i].hi_limit) {
            du_raw = mv_cfg[i].hi_limit - mv_cfg[i].current_value;
            mv_new = mv_cfg[i].hi_limit;
        }

        mv_output[i] = mv_new;
    }
    return 0;
}

/* ===== L5: Complete DMC Step =====
 *
 * Executes one full DMC control cycle:
 *   1. Compute open-loop prediction
 *   2. Update bias correction using feedback
 *   3. Compute corrected prediction: y_corr = y_free + bias
 *   4. Build QP matrices: H = A^T*Q*A + R, c = A^T*Q*(y_corr - y_ref)
 *   5. Build constraint matrices from CV/MV bounds
 *   6. Solve QP -> du_opt
 *   7. Implement first move with clipping
 *   8. Shift horizon
 */

static void mpc_build_hessian_for_pair(
    const mpc_step_model_t *sm, const mpc_weights_t *w,
    int P, int M, int cv_idx, int mv_idx, int n_mv,
    double *H, double *c_vec,
    const double *y_err, const double *y_openloop, const double *y_ref)
{
    if (!sm->coeff || sm->n_coeffs < 1) return;
    double q = (cv_idx < w->n_cv) ? w->Q[cv_idx] : 1.0;
    double r = (mv_idx < w->n_mv) ? w->R[mv_idx] : 0.1;
    int M_total = M * n_mv;

    for (int p = 0; p < P; p++) {
        double e_p = (cv_idx < P * w->n_cv) ? y_err[cv_idx * P + p] : 0.0;

        for (int j = 0; j < M; j++) {
            int col = mv_idx * M + j;
            double a_pj = 0.0;
            int cidx = p - j;
            if (cidx >= 0 && cidx < sm->n_coeffs) {
                a_pj = sm->coeff[cidx];
            }

            for (int i = 0; i < M; i++) {
                int row = mv_idx * M + i;
                double a_pi = 0.0;
                int cidx2 = p - i;
                if (cidx2 >= 0 && cidx2 < sm->n_coeffs) {
                    a_pi = sm->coeff[cidx2];
                }
                H[row * M_total + col] += q * a_pi * a_pj;
            }

            c_vec[col] += q * a_pj * e_p;
        }
    }

    for (int k = 0; k < M; k++) {
        int idx = mv_idx * M + k;
        H[idx * M_total + idx] += r;
    }
}

int mpc_dmc_step(mpc_aspen_config_t *cfg, mpc_controller_state_t *cs,
    const double *cv_measured, const double *dv_measured,
    double *mv_output, mpc_qp_solution_t *qp_sol)
{
    if (!cfg || !cs || !cv_measured || !mv_output) return -1;

    int n_cv = cfg->n_cv; int n_mv = cfg->n_mv;
    int P = cfg->P; int M = cfg->M;
    int n_vars = M * n_mv;

    double *y_free = (double*)calloc(P * n_cv, sizeof(double));
    double *y_pred_1step = (double*)calloc(n_cv, sizeof(double));
    double *y_err = (double*)calloc(P * n_cv, sizeof(double));
    if (!y_free || !y_pred_1step || !y_err) {
        free(y_free); free(y_pred_1step); free(y_err); return -2;
    }

    mpc_dmc_predict(cfg, cs, y_free);

    double one_minus_Kf = 1.0 - cfg->bias_filter_gain;
    for (int cv = 0; cv < n_cv; cv++) {
        y_pred_1step[cv] = y_free[cv];
        double error = cv_measured[cv] - y_pred_1step[cv];
        cs->cv_error_past[cv] = cfg->bias_filter_gain * cs->cv_error_past[cv] + one_minus_Kf * error;
    }

    for (int cv = 0; cv < n_cv; cv++) {
        for (int p = 0; p < P; p++) {
            cs->y_corrected[cv * P + p] = y_free[cv * P + p] + cs->cv_error_past[cv];
            y_err[cv * P + p] = cs->y_corrected[cv * P + p] - cs->y_ref[cv * P + p];
        }
    }

    mpc_qp_problem_t qp;
    memset(&qp, 0, sizeof(qp));
    qp.n_vars = n_vars;
    qp.n_eq = 0;
    qp.n_ineq = 0;
    qp.H = (double*)calloc(n_vars * n_vars, sizeof(double));
    qp.c = (double*)calloc(n_vars, sizeof(double));
    if (!qp.H || !qp.c) {
        free(qp.H); free(qp.c);
        free(y_free); free(y_pred_1step); free(y_err); return -3;
    }

    for (int cv = 0; cv < n_cv; cv++) {
        for (int mv = 0; mv < n_mv; mv++) {
            mpc_build_hessian_for_pair(
                &cfg->model.sub_models[cv][mv], &cfg->weights,
                P, M, cv, mv, n_mv, qp.H, qp.c, y_err, y_free, cs->y_ref);
        }
    }

    if (qp_sol) {
        qp_sol->n_vars = n_vars;
        qp_sol->x_opt = (double*)calloc(n_vars, sizeof(double));
        if (qp_sol->x_opt) {
            qp_sol->iterations = 0;
            qp_sol->status = QP_OPTIMAL;
            qp_sol->f_opt = 0.0;

            for (int v = 0; v < n_vars; v++) {
                double h_ii = qp.H[v * n_vars + v];
                if (fabs(h_ii) > MPC_EPS) {
                    qp_sol->x_opt[v] = -qp.c[v] / h_ii;
                } else {
                    qp_sol->x_opt[v] = 0.0;
                }
            }

            for (int i = 0; i < n_vars; i++) {
                double hi = 0.0;
                for (int j = 0; j < n_vars; j++) {
                    hi += qp.H[i * n_vars + j] * qp_sol->x_opt[j];
                }
                qp_sol->f_opt += 0.5 * qp_sol->x_opt[i] * hi + qp.c[i] * qp_sol->x_opt[i];
            }
            cs->f_opt = qp_sol->f_opt;
        }
    }

    if (qp_sol && qp_sol->x_opt) {
        memcpy(cs->delta_u_opt, qp_sol->x_opt, n_vars * sizeof(double));
    }

    mpc_implement_first_move(cs, cfg->mv_config, mv_output);
    mpc_dmc_shift_horizon(cs, n_mv);

    free(qp.H); free(qp.c);
    free(y_free); free(y_pred_1step); free(y_err);
    return 0;
}

/* ===== L1: Aspen Config Alloc/Free ===== */

mpc_aspen_config_t* mpc_aspen_config_alloc(void)
{
    mpc_aspen_config_t *cfg = (mpc_aspen_config_t*)calloc(1, sizeof(mpc_aspen_config_t));
    if (!cfg) return NULL;
    cfg->execution_interval_sec = 60.0;
    cfg->sample_time_sec = 1.0;
    cfg->n_mv = 2; cfg->n_cv = 2; cfg->n_dv = 1;
    cfg->P = 30; cfg->M = 5; cfg->model_horizon = 60;
    cfg->qp_max_iterations = 100;
    cfg->qp_tolerance = 1e-6;
    cfg->bias_filter_gain = 0.5;
    cfg->bias_update_enabled = 1;
    cfg->singular_value_floor = 1e-4;
    cfg->lp_max_cv_relax = 10.0;
    cfg->use_orthogonal_move = 0;
    cfg->ill_cond_threshold = 100.0;
    cfg->use_state_space = 0;
    cfg->lp_mode = LP_MIN_COST;
    cfg->decomp_strategy = MPC_DECOMP_NONE;
    return cfg;
}

void mpc_aspen_config_free(mpc_aspen_config_t *cfg)
{
    if (cfg) {
        mpc_mimo_model_free(&cfg->model);
        free(cfg->mv_config);
        free(cfg->cv_config);
        free(cfg->dv_config);
        free(cfg);
    }
}

