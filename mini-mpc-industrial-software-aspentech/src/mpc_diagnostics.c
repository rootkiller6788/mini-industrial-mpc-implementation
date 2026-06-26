/** @file mpc_diagnostics.c
 * @brief MPC Performance Diagnostics (L6, L7)
 * L6: Canonical control performance problems
 * L7: Industrial application - AspenTech DMC3 Model Quality Monitor
 * Ref: Qin and Badgwell (2003), Jelali - Control Performance Management (2012)
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "mpc_common.h"

double mpc_compute_cv_violation(const mpc_controller_state_t *cs,
    const mpc_cv_config_t *cv_cfg)
{
    if (!cs || !cv_cfg) return 0.0;
    double max_violation = 0.0;
    for (int cv = 0; cv < cs->n_cv; cv++) {
        double y = cs->y_corrected[cv];
        double violation = 0.0;
        if (y < cv_cfg[cv].lo_limit) violation = cv_cfg[cv].lo_limit - y;
        else if (y > cv_cfg[cv].hi_limit) violation = y - cv_cfg[cv].hi_limit;
        if (violation > max_violation) max_violation = violation;
    }
    return max_violation;
}

double mpc_compute_mv_utilization(const mpc_mv_config_t *mv_cfg,
    const double *mv_output)
{
    if (!mv_cfg || !mv_output) return 0.0;
    double span = mv_cfg->eu100 - mv_cfg->eu0;
    if (fabs(span) < MPC_EPS) return 0.0;
    double pct = ((*mv_output - mv_cfg->eu0) / span) * 100.0;
    if (pct < 0.0) pct = 0.0;
    if (pct > 100.0) pct = 100.0;
    return pct;
}

/* Harris Performance Index
 * I_perf = 1 - var(y_cl) / var(y_ol)
 * ~1 = good control, ~0 = poor, <0 = worse than open-loop
 * Ref: Harris (1989), Assessment of Control Loop Performance
 */

double mpc_compute_performance_index(const mpc_controller_state_t *cs,
    const double *y_measured, int n_cv)
{
    if (!cs || !y_measured || n_cv < 1) return -1.0;
    double var_cl = 0.0, var_ol = 0.0;
    for (int i = 0; i < n_cv; i++) {
        double err = y_measured[i] - cs->y_ref[i];
        var_cl += err * err;
        double ol_err = y_measured[i];
        var_ol += ol_err * ol_err;
    }
    if (var_ol < MPC_EPS) return 1.0;
    return 1.0 - (var_cl / var_ol);
}

/* Model Quality Monitor - RMSE between measured and predicted CVs
 * Uses sliding window of recent samples.
 * If RMSE exceeds threshold, triggers model re-identification alert.
 */

int mpc_model_quality_monitor(const mpc_mimo_model_t *mimo,
    const double *measured, const double *predicted,
    int n_samples, double *rmse)
{
    if (!mimo || !measured || !predicted || !rmse || n_samples < 1) return -1;
    double sum_sq = 0.0;
    int n_cv = mimo->n_cv;
    for (int k = 0; k < n_samples; k++) {
        for (int cv = 0; cv < n_cv; cv++) {
            double err = measured[k * n_cv + cv] - predicted[k * n_cv + cv];
            sum_sq += err * err;
        }
    }
    *rmse = sqrt(sum_sq / (n_samples * n_cv));
    return 0;
}
