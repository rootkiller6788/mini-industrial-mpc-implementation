/** @file mpc_disturbance_model.c
 * @brief Disturbance Modeling and Feedforward for MPC (L5, L7)
 *
 * Industrial MPC controllers handle various disturbance types:
 *   1. Measured disturbances (DV feedforward) — e.g., feed rate changes
 *   2. Unmeasured step disturbances — bias update handles these
 *   3. Ramp disturbances — integrator model needed
 *   4. Periodic disturbances — repetitive control or harmonic models
 *
 * Disturbance Model Integration (Rawlings/Mayne/Diehl Ch.4):
 *   Augmented state: [x; d] where d is the disturbance state.
 *   Augmented model predicts disturbance propagation:
 *     d(k+1) = A_d * d(k)  (autonomous disturbance dynamics)
 *     y(k) = C*x(k) + C_d*d(k)
 *
 * Ref:
 *   Rawlings/Mayne/Diehl (2017) Ch.4 - Disturbance Models
 *   Muske & Badgwell (2002) - Disturbance modeling for MPC
 *   Qin & Badgwell (2003) - Industrial MPC survey, Section 4
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "mpc_common.h"

/* ===== L5: Step Disturbance Model =====
 *
 * Most common unmeasured disturbance model (type 1).
 * d(k+1) = d(k)  (integrated white noise)
 *
 * In DMC: the bias term b(k) = y_meas(k) - y_pred(k|k-1)
 * acts as a step disturbance estimator.
 *
 * Theorem (Integral action):
 *   A controller with step disturbance model achieves
 *   zero steady-state offset for constant disturbances
 *   and setpoint changes. (Internal Model Principle)
 */

int mpc_disturbance_step_predict(const double *disturbance_current,
    int n_d, int n_steps, double *disturbance_forecast)
{
    if (!disturbance_current || !disturbance_forecast || n_d < 1 || n_steps < 1)
        return -1;

    /* Step disturbance: d(k+i|k) = d(k) for all i */
    for (int i = 0; i < n_steps; i++) {
        for (int j = 0; j < n_d; j++) {
            disturbance_forecast[i * n_d + j] = disturbance_current[j];
        }
    }
    return 0;
}

/* ===== L5: Ramp Disturbance Model =====
 *
 * d(k+1) = d(k) + delta_d(k)
 * delta_d(k+1) = delta_d(k)  (constant ramp rate)
 *
 * Augmented state: [x; d; delta_d] where delta_d is the ramp slope.
 * Prediction: d(k+i|k) = d(k) + i * delta_d(k)
 *
 * Industrial example: catalyst deactivation, fouling, ambient temperature drift.
 */

int mpc_disturbance_ramp_predict(const double *disturbance_current,
    const double *ramp_rate, int n_d, int n_steps,
    double *disturbance_forecast)
{
    if (!disturbance_current || !ramp_rate || !disturbance_forecast || n_d < 1 || n_steps < 1)
        return -1;

    for (int i = 0; i < n_steps; i++) {
        for (int j = 0; j < n_d; j++) {
            disturbance_forecast[i * n_d + j] = disturbance_current[j] + (i + 1.0) * ramp_rate[j];
        }
    }
    return 0;
}

/* ===== L5: Exponential Decay Disturbance Model =====
 *
 * d(k+1) = alpha * d(k)  where 0 < alpha < 1
 *
 * Prediction: d(k+i|k) = alpha^i * d(k)
 *
 * Useful for: first-order process disturbances (leaks, thermal decay).
 *   alpha = exp(-Ts/tau_d) where tau_d is the disturbance time constant.
 */

int mpc_disturbance_exp_predict(const double *disturbance_current,
    int n_d, int n_steps, double alpha, double *disturbance_forecast)
{
    if (!disturbance_current || !disturbance_forecast || n_d < 1 || n_steps < 1)
        return -1;
    if (alpha <= 0.0 || alpha >= 1.0) return -2;

    for (int i = 0; i < n_steps; i++) {
        double factor = pow(alpha, (double)(i + 1));
        for (int j = 0; j < n_d; j++) {
            disturbance_forecast[i * n_d + j] = disturbance_current[j] * factor;
        }
    }
    return 0;
}

/* ===== L5: Periodic (Sinusoidal) Disturbance Model =====
 *
 * d(k) = A * sin(omega*k*Ts + phi)
 *
 * State-space representation:
 *   [d1(k+1)] = [ cos(w*Ts)  sin(w*Ts)] [d1(k)]
 *   [d2(k+1)]   [-sin(w*Ts)  cos(w*Ts)] [d2(k)]
 *   d(k) = [1 0] * [d1(k); d2(k)]
 *
 * Industrial examples: compressor pulsation (60 Hz), day/night temperature cycle,
 *   rolling mill eccentricity, tide effects in marine systems.
 */

int mpc_disturbance_periodic_predict(double amplitude, double frequency_hz,
    double phase_rad, double sample_time_sec, int n_steps,
    double *disturbance_forecast)
{
    if (!disturbance_forecast || n_steps < 1 || sample_time_sec <= 0.0) return -1;
    if (frequency_hz < 0.0) return -2;

    double omega = 2.0 * 3.14159265358979323846 * frequency_hz;
    for (int i = 0; i < n_steps; i++) {
        double t = (i + 1.0) * sample_time_sec;
        disturbance_forecast[i] = amplitude * sin(omega * t + phase_rad);
    }
    return 0;
}

/* ===== L7: Disturbance Model Selection (AspenTech DMC3) =====
 *
 * In DMC3, the user selects a disturbance model per CV:
 *   - Type 0: No disturbance model (use bias only)
 *   - Type 1: Step (integrated white noise) — default
 *   - Type 2: Ramp (double integrator)
 *   - Type 3: First-order (exponential decay)
 *
 * This function predicts the disturbance contribution
 * over the MPC prediction horizon for the selected model type.
 */

#define MPC_DIST_TYPE_NONE    0
#define MPC_DIST_TYPE_STEP    1
#define MPC_DIST_TYPE_RAMP    2

int mpc_disturbance_predict_by_type(int dist_type,
    const double *dist_current, const double *dist_param,
    int n_d, int n_steps, double *dist_forecast)
{
    if (!dist_current || !dist_forecast || n_d < 1 || n_steps < 1) return -1;

    switch (dist_type) {
    case MPC_DIST_TYPE_NONE:
        memset(dist_forecast, 0, n_steps * n_d * sizeof(double));
        break;
    case MPC_DIST_TYPE_STEP:
        mpc_disturbance_step_predict(dist_current, n_d, n_steps, dist_forecast);
        break;
    case MPC_DIST_TYPE_RAMP:
        if (!dist_param) return -3;
        mpc_disturbance_ramp_predict(dist_current, dist_param, n_d, n_steps, dist_forecast);
        break;
    default:
        return -2;
    }
    return 0;
}

/* ===== L7: DV Feedforward Prediction =====
 *
 * Measured disturbance variables (DVs) enter the prediction through
 * their step-response models, similar to MVs but without optimization.
 *
 * y_cv_pred += G_dv * dv_future (known future DV trajectory)
 *
 * In AspenTech DMC3, DV future trajectory can be:
 *   - Constant: dv(k+i) = dv(k) for all i
 *   - Scheduled: dv(k+i) from operator-entered schedule
 *   - Predicted: dv(k+i) from upstream optimizer
 */

int mpc_dv_feedforward_predict(const mpc_mimo_model_t *mimo,
    const double *dv_current, const double *dv_future,
    int n_pred, double *y_dv_contrib)
{
    if (!mimo || !dv_current || !y_dv_contrib || n_pred < 1) return -1;
    int n_cv = mimo->n_cv, n_dv = mimo->n_dv;

    memset(y_dv_contrib, 0, n_pred * n_cv * sizeof(double));

    for (int cv = 0; cv < n_cv; cv++) {
        for (int dv = 0; dv < n_dv; dv++) {
            mpc_step_model_t *dm = &mimo->dv_models[cv][dv];
            if (!dm->coeff || dm->n_coeffs < 1) continue;

            int N = dm->n_coeffs;
            for (int p = 0; p < n_pred; p++) {
                double y_dv = 0.0;
                for (int j = 0; j <= p; j++) {
                    double dv_val = (j < n_pred && dv_future) ? dv_future[j * n_dv + dv] : dv_current[dv];
                    int coeff_idx = p - j;
                    if (coeff_idx < N) {
                        y_dv += dm->coeff[coeff_idx] * dv_val;
                    } else {
                        y_dv += dm->coeff[N - 1] * dv_val;
                    }
                }
                y_dv_contrib[cv * n_pred + p] += y_dv;
            }
        }
    }
    return 0;
}

/* ===== L7: Disturbance Rejection Ratio (DRR) =====
 *
 * DRR = 1 - var(y_cl_with_dist) / var(y_ol_with_dist)
 *
 * Measures how well the MPC controller rejects disturbances
 * compared to open-loop operation.
 *   DRR ~ 1: Excellent disturbance rejection
 *   DRR ~ 0: No improvement over open-loop
 *   DRR < 0: Control makes things worse
 */

double mpc_disturbance_rejection_ratio(const double *y_closed_loop,
    const double *y_open_loop, int n_samples, int n_cv)
{
    if (!y_closed_loop || !y_open_loop || n_samples < 2 || n_cv < 1) return -1.0;

    double var_cl = 0.0, var_ol = 0.0;

    /* Compute mean for each */
    double mean_cl = 0.0, mean_ol = 0.0;
    for (int i = 0; i < n_samples * n_cv; i++) {
        mean_cl += y_closed_loop[i];
        mean_ol += y_open_loop[i];
    }
    mean_cl /= (n_samples * n_cv);
    mean_ol /= (n_samples * n_cv);

    /* Compute variance */
    for (int i = 0; i < n_samples * n_cv; i++) {
        double d_cl = y_closed_loop[i] - mean_cl;
        double d_ol = y_open_loop[i] - mean_ol;
        var_cl += d_cl * d_cl;
        var_ol += d_ol * d_ol;
    }

    if (var_ol < MPC_EPS) return 1.0;
    return 1.0 - var_cl / var_ol;
}
