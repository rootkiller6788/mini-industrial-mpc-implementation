/** @file dmc_model.c
 * @brief Step Response Model — FIR construction, FOPDT fitting, prediction, validation
 * Knowledge: L1 Definitions (FIR coefficients, dead time, gain),
 *            L5 Algorithms (convolution prediction, FOPDT parameterization)
 *
 * Theory: DMC uses truncated FIR (step response) models.
 *   y(k) = sum_{i=1}^{N} s_i * du(k-i) + s_N * u(k-N-1) + d(k)
 *   For FOPDT: s_i = K * (1 - exp(-(i*Ts - theta)/tau)) for i*Ts > theta
 *
 * Ref: Cutler & Ramaker (1980), Seborg/Edgar/Mellichamp (2016) Ch.20
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include "mpc_common.h"

/* ================================================================
 * L1: Memory Management
 * ================================================================ */

mpc_step_model_t* mpc_step_model_alloc(int n_coeffs)
{
    if (n_coeffs < 1 || n_coeffs > MPC_MAX_MODEL_HORIZON) return NULL;
    mpc_step_model_t *m = (mpc_step_model_t*)calloc(1, sizeof(mpc_step_model_t));
    if (!m) return NULL;
    m->n_coeffs = n_coeffs;
    m->coeff = (double*)calloc(n_coeffs, sizeof(double));
    if (!m->coeff) { free(m); return NULL; }
    m->gain_ss = 0.0;
    m->time_constant_sec = 0.0;
    m->dead_time_samples = 0.0;
    m->sample_time_sec = 1.0;
    m->mv_index = -1;
    m->cv_index = -1;
    return m;
}

void mpc_step_model_free(mpc_step_model_t *m)
{
    if (m) {
        free(m->coeff);
        free(m);
    }
}

mpc_mimo_model_t* mpc_mimo_model_alloc(int n_mv, int n_cv, int n_dv, int horizon)
{
    if (n_mv < 0 || n_cv < 1 || horizon < 1 || horizon > MPC_MAX_MODEL_HORIZON) return NULL;
    mpc_mimo_model_t *m = (mpc_mimo_model_t*)calloc(1, sizeof(mpc_mimo_model_t));
    if (!m) return NULL;
    m->n_mv = n_mv; m->n_cv = n_cv; m->n_dv = n_dv;
    m->model_horizon = horizon;
    m->sample_time_sec = 1.0;

    m->sub_models = (mpc_step_model_t**)calloc(n_cv, sizeof(mpc_step_model_t*));
    if (!m->sub_models) { free(m); return NULL; }
    for (int i = 0; i < n_cv; i++) {
        m->sub_models[i] = (mpc_step_model_t*)calloc(n_mv, sizeof(mpc_step_model_t));
        if (!m->sub_models[i]) {
            for (int j = 0; j < i; j++) free(m->sub_models[j]);
            free(m->sub_models); free(m); return NULL;
        }
    }

    m->dv_models = (mpc_step_model_t**)calloc(n_cv, sizeof(mpc_step_model_t*));
    if (!m->dv_models) {
        for (int i = 0; i < n_cv; i++) free(m->sub_models[i]);
        free(m->sub_models); free(m); return NULL;
    }
    for (int i = 0; i < n_cv; i++) {
        m->dv_models[i] = (mpc_step_model_t*)calloc(n_dv, sizeof(mpc_step_model_t));
        if (!m->dv_models[i]) {
            for (int j = 0; j < i; j++) free(m->dv_models[j]);
            free(m->dv_models);
            for (int j = 0; j < n_cv; j++) free(m->sub_models[j]);
            free(m->sub_models); free(m); return NULL;
        }
    }
    return m;
}

void mpc_mimo_model_free(mpc_mimo_model_t *m)
{
    if (m) {
        if (m->sub_models) {
            for (int i = 0; i < m->n_cv; i++) {
                if (m->sub_models[i]) free(m->sub_models[i]);
            }
            free(m->sub_models);
            m->sub_models = NULL;
        }
        if (m->dv_models) {
            for (int i = 0; i < m->n_cv; i++) {
                if (m->dv_models[i]) free(m->dv_models[i]);
            }
            free(m->dv_models);
            m->dv_models = NULL;
        }
    }
}

/* ================================================================
 * L5: FOPDT to Step-Response Model Conversion
 *
 * First-Order Plus Dead Time (FOPDT):
 *   G(s) = K * exp(-theta*s) / (tau*s + 1)
 *
 * Step response (continuous):
 *   y(t) = 0                           for t < theta
 *   y(t) = K * (1 - exp(-(t-theta)/tau)) for t >= theta
 *
 * Discretized at sample time Ts:
 *   s_i = K * (1 - exp(-(i*Ts - theta)/tau))   for i*Ts > theta
 *   s_i = 0                                     for i*Ts <= theta
 *
 * Theorem (FOPDT truncation bound):
 *   |y(N*Ts) - K| = K * exp(-(N*Ts - theta)/tau)
 *   => N needed for 99% settling: N > (theta + 4.6*tau)/Ts
 * ================================================================ */

int mpc_step_model_from_fopdt(mpc_step_model_t *m,
    double gain, double tau, double dead_time, double sample_time)
{
    if (!m || m->n_coeffs < 1 || tau <= 0.0 || sample_time <= 0.0) return -1;
    if (dead_time < 0.0) return -2;

    m->gain_ss = gain;
    m->time_constant_sec = tau;
    m->dead_time_samples = dead_time / sample_time;
    m->sample_time_sec = sample_time;

    double dead_samples = dead_time / sample_time;
    for (int i = 0; i < m->n_coeffs; i++) {
        double t = (i + 1) * sample_time;
        if (t <= dead_time) {
            m->coeff[i] = 0.0;
        } else {
            m->coeff[i] = gain * (1.0 - exp(-(t - dead_time) / tau));
        }
    }
    return 0;
}

/* ================================================================
 * L1: FIR Coefficient Import
 *
 * Direct import of step-response coefficients from plant step-test data.
 * In AspenTech DMC3, this is the output of the Model Identification step.
 * ================================================================ */

int mpc_step_model_from_fir(mpc_step_model_t *m,
    const double *fir_coeffs, int n, double sample_time)
{
    if (!m || !fir_coeffs || n < 1 || sample_time <= 0.0) return -1;
    if (n > m->n_coeffs) return -2;

    m->sample_time_sec = sample_time;
    memcpy(m->coeff, fir_coeffs, n * sizeof(double));

    m->gain_ss = fir_coeffs[n - 1];

    int dead_idx = 0;
    while (dead_idx < n && fabs(fir_coeffs[dead_idx]) < MPC_EPS) dead_idx++;
    m->dead_time_samples = (double)dead_idx;

    double gain63 = 0.632 * m->gain_ss;
    for (int i = dead_idx; i < n; i++) {
        if (fabs(fir_coeffs[i]) >= fabs(gain63)) {
            m->time_constant_sec = (i + 1 - dead_idx) * sample_time;
            break;
        }
    }
    return 0;
}

/* ================================================================
 * L5: Step-Response Prediction (Discrete Convolution)
 *
 * Given past MV moves du(k), du(k-1), ..., du(k-N+1)
 * Predict CV at future steps: y_pred(k+1), ..., y_pred(k+n_pred)
 *
 * Algorithm: For each prediction step p (1..n_pred):
 *   y_pred[p-1] = sum_{j=1}^{min(N, p+n_past)} s_{j+p-1} * du_past[n_past-1-(j-1)]
 *
 * Complexity: O(N * n_pred) time, O(1) extra space
 * ================================================================ */

int mpc_step_model_predict(const mpc_step_model_t *m,
    const double *delta_u_past, int n_past, double *y_pred, int n_pred)
{
    if (!m || !delta_u_past || !y_pred) return -1;
    if (n_past < 1 || n_pred < 1) return -2;
    if (n_past > m->n_coeffs) return -3;

    int N = m->n_coeffs;

    for (int p = 0; p < n_pred; p++) {
        double y = 0.0;
        for (int j = 0; j < n_past; j++) {
            int coeff_idx = p + j;
            if (coeff_idx >= N) {
                y += m->coeff[N - 1] * delta_u_past[n_past - 1 - j];
            } else {
                y += m->coeff[coeff_idx] * delta_u_past[n_past - 1 - j];
            }
        }
        y_pred[p] = y;
    }
    return 0;
}

/* ================================================================
 * L4: Truncation Error Estimation
 *
 * For an FIR model truncated at N coefficients, the truncation
 * error bounds the difference between N-coeff and infinite-horizon
 * predictions. Uses the monotone decreasing property of stable
 * step responses.
 *
 * Formula: error <= |s_N - s_inf| * max|du|
 * ================================================================ */

double mpc_step_model_truncation_error(const mpc_step_model_t *m,
    double error_tolerance)
{
    if (!m || m->n_coeffs < 2) return MPC_INF;

    int N = m->n_coeffs;
    double s_final = m->coeff[N - 1];
    double s_prev  = m->coeff[N - 2];
    double delta_s = fabs(s_final - s_prev);

    if (delta_s <= error_tolerance) return 0.0;

    double tau_est = m->time_constant_sec;
    int extra_needed = 0;
    if (tau_est > 0.0) {
        double remaining = fabs(m->gain_ss - s_final);
        extra_needed = (int)ceil(-tau_est / m->sample_time_sec * log(error_tolerance / (remaining + MPC_EPS)));
        if (extra_needed < 0) extra_needed = 0;
    }
    return (double)extra_needed;
}

/* ================================================================
 * L1: Model Validation
 * ================================================================ */

int mpc_step_model_validate(const mpc_step_model_t *m)
{
    if (!m) return -1;
    if (m->n_coeffs < 2 || m->n_coeffs > MPC_MAX_MODEL_HORIZON) return -2;
    if (m->sample_time_sec <= 0.0) return -3;
    if (!m->coeff) return -4;

    int has_nonzero = 0;
    for (int i = 0; i < m->n_coeffs; i++) {
        if (!isfinite(m->coeff[i])) return -5;
        if (fabs(m->coeff[i]) > MPC_EPS) has_nonzero = 1;
    }
    if (!has_nonzero) return -6;

    double s_last = fabs(m->coeff[m->n_coeffs - 1]);
    double s_last_minus_1 = fabs(m->coeff[m->n_coeffs - 2]);
    if (s_last > s_last_minus_1 * 1.1 && s_last > MPC_EPS) {
        return -7;
    }
    return 0;
}

/* ================================================================
 * L8: Step Response to Discrete State-Space Conversion
 *
 * Converts FIR step-response model to state-space (A,B,C).
 * Uses Ho-Kalman realization:
 *   x(k+1) = A*x(k) + B*du(k)
 *   y(k)   = C*x(k)
 * where A is shift matrix, B = [1,0,...,0]^T, C = [s_1,...,s_N]
 * ================================================================ */

int mpc_step_model_to_ss(const mpc_step_model_t *m,
    double *a, double *b, double *c)
{
    if (!m || !a || !b || !c) return -1;
    int n = m->n_coeffs;
    if (n < 2) return -2;

    memset(a, 0, n * n * sizeof(double));
    for (int i = 1; i < n; i++) {
        a[(i-1)*n + (i-1)] = 0.0;
        a[(i-1)*n + (i)] = 1.0;
    }
    a[(n-1)*n + (n-1)] = 1.0;

    memset(b, 0, n * sizeof(double));
    b[0] = 1.0;

    for (int i = 0; i < n; i++) {
        c[i] = m->coeff[i];
    }
    return 0;
}

