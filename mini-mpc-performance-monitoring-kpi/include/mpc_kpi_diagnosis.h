#ifndef MPC_KPI_DIAGNOSIS_H
#define MPC_KPI_DIAGNOSIS_H
#include "mpc_kpi_defs.h"
#include "mpc_kpi_metrics.h"
#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Performance Degradation Root Cause Analysis
 * ========================================================================= */

mpc_kpi_status_t mpc_kpi_diagnose_degradation(
    const mpc_kpi_dashboard_t *db,
    const double *prediction_errors, uint64_t n_pe,
    const double *mv_moves, uint64_t n_mv,
    int *primary_cause, double *confidence);

mpc_kpi_status_t mpc_kpi_diagnose_oscillation(
    const double *data, uint64_t n,
    double *oscillation_period, double *oscillation_amplitude,
    double *regularity_index, bool *is_oscillating);

mpc_kpi_status_t mpc_kpi_diagnose_sluggishness(
    const double *setpoint, const double *cv, uint64_t n,
    double *settling_time_estimate, double *rise_time,
    double *overshoot_pct);

mpc_kpi_status_t mpc_kpi_diagnose_stiction(
    const double *mv, const double *cv, uint64_t n,
    double *stiction_band, bool *has_stiction,
    double *slip_jump_ratio);

/* =========================================================================
 * Mismatch Isolation (L5: subspace-based residual analysis)
 * ========================================================================= */

mpc_kpi_status_t mpc_kpi_isolate_mismatch_channel(
    const double *prediction_errors, uint64_t n_pe,
    const double *mv_moves, uint64_t n_mv,
    int num_cv, int num_mv,
    int *mismatch_cv_idx, int *mismatch_mv_idx,
    mpc_mismatch_type_t *types);

mpc_kpi_status_t mpc_kpi_residual_whiteness_test(
    const double *residuals, uint64_t n,
    int max_lag, double significance,
    bool *is_white, double *test_stat, double *critical_val);

/* =========================================================================
 * Performance Forecasting (L8: Bayesian trend prediction)
 * ========================================================================= */

mpc_kpi_status_t mpc_kpi_forecast_kpi(
    const double *kpi_history, uint64_t n,
    int forecast_horizon, double *forecast_values,
    double *prediction_interval_lo, double *prediction_interval_hi,
    double confidence_level);

mpc_kpi_status_t mpc_kpi_forecast_degradation_time(
    const double *kpi_history, uint64_t n,
    double alarm_threshold, double *estimated_cycles_until_alarm,
    double *confidence);

/* =========================================================================
 * Recommendation Engine
 * ========================================================================= */

mpc_kpi_status_t mpc_kpi_generate_recommendations(
    const mpc_kpi_dashboard_t *db,
    const mpc_kpi_mismatch_t *mismatch_info,
    char *recommendations, int max_chars);

mpc_kpi_status_t mpc_kpi_model_update_urgency(
    const mpc_kpi_mismatch_t *mismatch, double economic_impact_per_day,
    double *urgency_score, char *urgency_label, int label_max);

#ifdef __cplusplus
}
#endif
#endif

/* =========================================================================
 * L7/L8: Industrial vendor and advanced KPI functions (declared here for test access)
 * ========================================================================= */

/* Advanced diagnosis functions */
mpc_kpi_status_t mpc_kpi_diagnose_ratio_violation(const double *pv, const double *sv, uint64_t n, double ratio_target, double tolerance_pct, double *actual_ratio_mean, double *ratio_violation_rate, bool *ratio_maintained);
mpc_kpi_status_t mpc_kpi_valve_travel_histogram(const double *mv, uint64_t n, double *travel_total, double *travel_mean, double *travel_max, double *reversal_count, double *stiction_indicator);
mpc_kpi_status_t mpc_kpi_constraint_activity_analysis(const double *lagrange_multipliers, uint64_t n, int n_constraints, double *activity_fraction, int *most_active_idx, double *avg_multiplier);
mpc_kpi_status_t mpc_kpi_setpoint_change_analysis(const double *sp, uint64_t n, double *changes_per_cycle, double *mean_change_magnitude, uint64_t *num_changes);
mpc_kpi_status_t mpc_kpi_degradation_rate(const double *health_history, uint64_t n, double *degradation_per_cycle, double *cycles_to_critical, double critical_threshold);
mpc_kpi_status_t mpc_kpi_improvement_scenario(const mpc_kpi_dashboard_t *db, double improvement_pct, int target_category, double *projected_health, double *roi_estimate, double cost_per_point);
mpc_kpi_status_t mpc_kpi_detect_nonlinearity(const double *data, uint64_t n, double *nonlinearity_index, bool *is_nonlinear, double *bicoherence_peak);
mpc_kpi_status_t mpc_kpi_tuning_aggressiveness(const double *mv_moves, uint64_t n, const double *mv_bounds, double *aggressiveness, double *move_smoothness, bool *overly_aggressive);
mpc_kpi_status_t mpc_kpi_idle_time_analysis(const bool *running_flags, uint64_t n, uint64_t cycle_time_sec, double *utilization_pct, double *mttf, double *mttr, uint64_t *num_stops);
mpc_kpi_status_t mpc_kpi_gain_schedule_check(const double *cv, const double *sp, const double *operating_region, uint64_t n, int n_regions, double *per_region_rmse, double *worst_region, int *worst_region_idx);
mpc_kpi_status_t mpc_kpi_seasonal_decompose(const double *data, uint64_t n, int season_length, double *trend, double *seasonal, double *residual);

/* Industrial vendor KPIs */
mpc_kpi_status_t mpc_kpi_aspentech_aspenwatch_kpi(const mpc_kpi_dashboard_t *db, double *health_index, double *service_factor);
mpc_kpi_status_t mpc_kpi_honeywell_profit_sensor(const double *pred_errors, uint64_t n, const double *mv_costs, int nmv, double *profit_impact, double *model_quality);
mpc_kpi_status_t mpc_kpi_yokogawa_md_diagnostic(const double *cv_values, int ncv, const double *cv_targets, double *process_capability, double *robustness_index);
mpc_kpi_status_t mpc_kpi_shell_mv_monitor(const double *mv_utilization, int nmv, const double *cv_variances, int ncv, double *overall_health, double *bottleneck_index);
mpc_kpi_status_t mpc_kpi_iso50001_enpi(double energy_baseline, double energy_current, double production_baseline, double production_current, double *enpi, double *improvement_pct);
mpc_kpi_status_t mpc_kpi_autonomous_health_assessment(const mpc_kpi_dashboard_t *db, double *autonomy_readiness, double *human_intervention_frequency);
mpc_kpi_status_t mpc_kpi_bayesian_change_point(const double *kpi_history, uint64_t n, double *change_probability, uint64_t *change_point_cycle, double *pre_change_mean, double *post_change_mean);
mpc_kpi_status_t mpc_kpi_subspace_model_validation(const double *input_data, const double *output_data, uint64_t n, int input_dim, int output_dim, int past_horizon, double *subspace_angle, bool *model_degraded);
mpc_kpi_status_t mpc_kpi_pareto_analysis(const double *economic_kpis, const double *quality_kpis, int n, int *pareto_front_indices, int *num_pareto, int max_pareto);
mpc_kpi_status_t mpc_kpi_time_varying_kpi(const double *kpi_stream, uint64_t n, double forgetting_factor, double *tv_mean, double *tv_variance, double *adaptation_rate);

/* Extended industrial vendor KPIs */
mpc_kpi_status_t mpc_kpi_rockwell_pavilion_model_quality(const double *pred_errors, uint64_t n, int num_models, double *model_scores, int *best_model_idx, double *ensemble_score);
mpc_kpi_status_t mpc_kpi_siemens_apc_monitor(const mpc_kpi_dashboard_t *db, const double *qp_times, uint64_t n_qp, double *apc_availability, double *apc_efficiency);
mpc_kpi_status_t mpc_kpi_kalman_innovation_monitor(const double *innovations, uint64_t n, const double *innovation_cov, double *whiteness_stat, double *normalized_innovation_mean, bool *filter_healthy);
mpc_kpi_status_t mpc_kpi_monte_carlo_robustness(const double *kpi_distribution, uint64_t n, int n_simulations, double *robustness_score, double *worst_case, double *best_case, double *expected_value);
mpc_kpi_status_t mpc_kpi_multirate_aggregation(const double *fast_kpi, uint64_t n_fast, int decimation_factor, double *slow_kpi, uint64_t *n_slow, double *aliasing_indicator);
mpc_kpi_status_t mpc_kpi_digital_twin_sync(const double *physical_kpis, const double *digital_twin_kpis, uint64_t n, double *sync_error, double *model_fidelity, bool *resync_needed);
mpc_kpi_status_t mpc_kpi_emerson_deltav_mpc_health(const mpc_kpi_dashboard_t *db, double *mpc_health_index, double *inferred_property_accuracy, double *loop_reliability);
mpc_kpi_status_t mpc_kpi_yokogawa_exapilot_transition(const double *transition_durations, uint64_t n_transitions, double target_duration, double *avg_transition_time, double *transition_success_rate, double *grade_change_efficiency);
mpc_kpi_status_t mpc_kpi_abb_800xa_apc_kpi(const mpc_kpi_dashboard_t *db, const double *mv_costs, int n_mv, double *apc_benefit_index, double *constraint_push_index);
mpc_kpi_status_t mpc_kpi_osisoft_pi_af_kpi(const mpc_kpi_dashboard_t *db, const double *pi_point_values, uint64_t n_points, double *asset_health_index, double *event_frame_rate, char *af_template_name, int name_max);
mpc_kpi_status_t mpc_kpi_it_ot_convergence_readiness(const mpc_kpi_dashboard_t *db, double *data_integration_score, double *cybersecurity_readiness, double *cloud_readiness);
