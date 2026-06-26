#ifndef MPC_KPI_MONITORING_H
#define MPC_KPI_MONITORING_H
#include "mpc_kpi_defs.h"
#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Baseline Estimation
 * ========================================================================= */

mpc_kpi_status_t mpc_kpi_baseline_estimate(
    const double *data, uint64_t n, double outlier_threshold,
    mpc_kpi_baseline_t *result);

mpc_kpi_status_t mpc_kpi_baseline_update(
    mpc_kpi_baseline_t *baseline, double new_value, double learning_rate);

mpc_kpi_status_t mpc_kpi_baseline_validate(
    const mpc_kpi_baseline_t *baseline);

/* =========================================================================
 * Continuous Monitoring Engine
 * ========================================================================= */

mpc_kpi_status_t mpc_kpi_monitor_init(
    mpc_kpi_dashboard_t *db, mpc_kpi_mode_t mode, uint64_t report_interval);

mpc_kpi_status_t mpc_kpi_monitor_step(
    mpc_kpi_dashboard_t *db, const double *cv_values, int num_cv,
    const double *mv_values, int num_mv,
    const double *prediction_errors, int num_pe,
    uint64_t cycle);

mpc_kpi_status_t mpc_kpi_monitor_compute_all(
    mpc_kpi_dashboard_t *db);

mpc_kpi_status_t mpc_kpi_monitor_check_alarms(
    mpc_kpi_dashboard_t *db, mpc_kpi_alarm_rule_t *rules,
    int num_rules);

mpc_kpi_status_t mpc_kpi_monitor_generate_report(
    const mpc_kpi_dashboard_t *db, mpc_kpi_report_t *report);

/* =========================================================================
 * Health Score Computation
 * ========================================================================= */

mpc_kpi_status_t mpc_kpi_compute_category_score(
    const mpc_kpi_dashboard_t *db, mpc_kpi_category_t cat,
    double *score);

double mpc_kpi_weighted_health_score(
    const double *scores, const double *weights, int n);

mpc_kpi_tier_t mpc_kpi_health_score_to_tier(double score);

/* =========================================================================
 * KPI Baselining Period Management
 * ========================================================================= */

mpc_kpi_status_t mpc_kpi_start_baselining(
    mpc_kpi_dashboard_t *db, uint64_t duration_cycles);

mpc_kpi_status_t mpc_kpi_end_baselining(
    mpc_kpi_dashboard_t *db);

bool mpc_kpi_is_baselining_complete(const mpc_kpi_dashboard_t *db);

/* =========================================================================
 * Environmental Impact KPIs (L7: ISO 50001 energy management)
 * ========================================================================= */

double mpc_kpi_co2_reduction_estimate(
    double energy_savings_kwh, double grid_emission_factor);

double mpc_kpi_energy_intensity_index(
    double energy_consumed, double production_rate, double baseline_eii);

/* Advanced monitoring features */
mpc_kpi_status_t mpc_kpi_moving_range(const double *d, uint64_t n, double *avg_mr, double *ucl_mr, double *lcl_mr);
mpc_kpi_status_t mpc_kpi_process_capability(const double *d, uint64_t n, double usl, double lsl, double *cp, double *cpk, double *ppm_defective);
mpc_kpi_status_t mpc_kpi_holt_smoothing(const double *d, uint64_t n, double alpha, double beta, double *level, double *trend, double *forecast, int f_horizon);
mpc_kpi_status_t mpc_kpi_correlation_matrix(const double **kpi_data, int n_kpis, uint64_t n_samples, double *corr_matrix);
mpc_kpi_status_t mpc_kpi_collinearity_check(const double *corr, int n_kpis, double *condition_number, bool *has_collinearity);
mpc_kpi_status_t mpc_kpi_cumulative_economic_benefit(const double *per_cycle_benefit, uint64_t n, double *cumulative, double *annualized, double cycles_per_year);
mpc_kpi_status_t mpc_kpi_alarm_flood_detect(const bool *alarm_states, uint64_t n, int flood_threshold_per_minute, double *flood_ratio, bool *in_flood);
mpc_kpi_status_t mpc_kpi_throughput_monitor(const double *throughput, uint64_t n, double target_rate, double *avg_throughput, double *throughput_loss, double *target_compliance, double *oee_component);
mpc_kpi_status_t mpc_kpi_data_completeness(const bool *data_valid_flags, uint64_t n, uint64_t expected_count, double *completeness, double *gap_ratio, uint64_t *longest_gap);
mpc_kpi_status_t mpc_kpi_operator_intervention_rate(const bool *manual_mode_flags, uint64_t n, double *intervention_rate, double *mean_intervention_duration, uint64_t *num_interventions);
mpc_kpi_status_t mpc_kpi_batch_cycle_time(const double *batch_durations, uint64_t n, double target_duration, double *avg_cycle_time, double *cycle_time_cv, double *target_adherence, double *cycle_time_improvement);
mpc_kpi_status_t mpc_kpi_supplier_quality_impact(const double *feed_quality, const double *cv_quality, uint64_t n, double *correlation_strength, double *feed_quality_index, double *impact_score);

#ifdef __cplusplus
}
#endif
#endif
