#ifndef MPC_KPI_METRICS_H
#define MPC_KPI_METRICS_H
#include "mpc_kpi_defs.h"
#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Harris Performance Index (Minimum Variance Benchmark)
 * ========================================================================= */

mpc_kpi_status_t mpc_kpi_compute_harris_index(
    const double *output_data, uint64_t n_samples,
    int max_lag, mpc_kpi_harris_t *result);

mpc_kpi_status_t mpc_kpi_estimate_impulse_response(
    const double *data, uint64_t n, int num_lags,
    double *impulse, int *num_impulse);

mpc_kpi_status_t mpc_kpi_yule_walker_ar(
    const double *data, uint64_t n, int ar_order,
    double *phi, double *sigma2, double *aic);

/* =========================================================================
 * Autocorrelation and Whiteness Tests
 * ========================================================================= */

mpc_kpi_status_t mpc_kpi_autocorrelation(
    const double *data, uint64_t n, int max_lag,
    mpc_kpi_autocorr_t *result);

mpc_kpi_status_t mpc_kpi_partial_autocorrelation(
    const double *acf, int max_lag, double *pacf);

mpc_kpi_status_t mpc_kpi_ljung_box_test(
    const double *residuals, uint64_t n, int num_lags,
    double *q_stat, double *p_value, int *dof, bool *is_white);

mpc_kpi_status_t mpc_kpi_ljung_box_from_acf(
    const double *acf, uint64_t n, int num_lags,
    double *q_stat, double *p_value, int *dof, bool *is_white);

/* =========================================================================
 * Model-Plant Mismatch Detection
 * ========================================================================= */

mpc_kpi_status_t mpc_kpi_detect_model_mismatch(
    const double *prediction_error, const double *mv_moves,
    uint64_t n, int max_corr_lag,
    mpc_kpi_mismatch_t *result);

mpc_kpi_status_t mpc_kpi_cross_correlation(
    const double *x, const double *y, uint64_t n, int max_lag,
    double *ccf);

mpc_mismatch_type_t mpc_kpi_classify_mismatch(
    const double *corr, int length, double peak, int lag,
    double *confidence);

mpc_kpi_status_t mpc_kpi_f_test_mismatch(
    double baseline_var, double current_var,
    uint64_t n_baseline, uint64_t n_current,
    double *f_stat, double *p_value, bool *significant,
    double alpha);

/* =========================================================================
 * Constraint Monitoring KPIs
 * ========================================================================= */

double mpc_kpi_constraint_satisfaction_rate(
    const bool *violation_flags, uint64_t n);

mpc_kpi_status_t mpc_kpi_constraint_violation_stats(
    const double *violation_magnitudes, uint64_t n,
    double *mean_viol, double *max_viol, double *rms_viol,
    uint64_t *count_violated);

double mpc_kpi_mv_saturation_fraction(
    const double *mv_values, const double *mv_lower, const double *mv_upper,
    uint64_t n_steps, int mv_idx, double tolerance);

/* =========================================================================
 * Statistical Utilities for KPIs
 * ========================================================================= */

mpc_kpi_status_t mpc_kpi_linear_trend(
    const double *y, uint64_t n,
    double *slope, double *intercept, double *r_squared);

double mpc_kpi_mad(const double *data, uint64_t n);

mpc_kpi_status_t mpc_kpi_normality_test(
    const double *data, uint64_t n,
    double *w_statistic, double *p_value, bool *is_normal);

double mpc_kpi_quantile(const double *sorted_data, uint64_t n, double p);

double mpc_kpi_economic_benefit(double baseline_cost, double current_cost);

double mpc_kpi_variance_reduction(double var_baseline, double var_current);

double mpc_kpi_controller_utilization(const bool *is_auto, uint64_t n);

double mpc_kpi_chi2_cdf_approx(double x, int dof);

double mpc_kpi_chi2_survival(double x, int dof);

double mpc_kpi_covariance(const double *x, const double *y, uint64_t n);

double mpc_kpi_correlation(const double *x, const double *y, uint64_t n);

mpc_kpi_status_t mpc_kpi_detect_outliers(
    const double *data, uint64_t n,
    bool *outlier_flags, uint64_t *num_outliers, double k);

/* Additional time series and robust statistics */
mpc_kpi_status_t mpc_kpi_adf_test(const double *d, uint64_t n, double *test_stat, double *p_value, bool *is_stationary);
mpc_kpi_status_t mpc_kpi_bootstrap_ci(const double *d, uint64_t n, int n_bootstrap, double *ci_lower, double *ci_upper, double alpha);
double mpc_kpi_signal_to_noise(const double *d, uint64_t n);
double mpc_kpi_stability_index(const double *d, uint64_t n);
mpc_kpi_status_t mpc_kpi_recursive_residual_cusum(const double *residuals, uint64_t n, int start_k, double *cusum_values, uint64_t *n_values, double *max_deviation);
double mpc_kpi_mutual_information(const double *x, const double *y, uint64_t n, int n_bins);
double mpc_kpi_durbin_watson(const double *residuals, uint64_t n);
mpc_kpi_status_t mpc_kpi_runs_test(const double *data, uint64_t n, double median, int *n_runs, double *z_statistic, bool *is_random);
mpc_kpi_status_t mpc_kpi_theil_sen_trend(const double *y, uint64_t n, double *slope, double *intercept);
mpc_kpi_status_t mpc_kpi_granger_causality(const double *x, const double *y, uint64_t n, int max_lag, double *f_statistic, bool *x_causes_y);

#ifdef __cplusplus
}
#endif
#endif
