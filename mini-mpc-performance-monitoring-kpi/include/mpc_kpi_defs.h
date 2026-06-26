#ifndef MPC_KPI_DEFS_H
#define MPC_KPI_DEFS_H

#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <stdbool.h>
#include <float.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MPC_KPI_MAX_HISTORY_LENGTH   86400
#define MPC_KPI_MAX_WINDOW_SIZE       7200
#define MPC_KPI_MAX_CV_COUNT           128
#define MPC_KPI_MAX_MV_COUNT            64
#define MPC_KPI_NAME_MAX                64
#define MPC_KPI_DESCRIPTION_MAX        256
#define MPC_KPI_EPS                  1e-10
#define MPC_KPI_EWMA_LAMBDA_DEFAULT   0.2
#define MPC_KPI_CUSUM_H_DEFAULT        5.0
#define MPC_KPI_HARRIS_MAX_LAG         30
#define MPC_KPI_MAX_ALARM_RULES         8

typedef enum {
    MPC_KPI_CAT_AVAILABILITY = 0,
    MPC_KPI_CAT_PERFORMANCE  = 1,
    MPC_KPI_CAT_QUALITY      = 2,
    MPC_KPI_CAT_ECONOMIC     = 3,
    MPC_KPI_CAT_CONSTRAINT   = 4,
    MPC_KPI_CAT_DIAGNOSTIC   = 5,
    MPC_KPI_CAT_ROBUSTNESS   = 6
} mpc_kpi_category_t;

typedef enum {
    MPC_KPI_HARRIS_INDEX           = 0,
    MPC_KPI_CONTROLLER_UTILIZATION = 1,
    MPC_KPI_CONSTRAINT_SATISFACTION= 2,
    MPC_KPI_MODEL_PLANT_MISMATCH   = 3,
    MPC_KPI_PREDICTION_ERROR_RMS   = 4,
    MPC_KPI_SETPOINT_TRACKING_RMSE = 5,
    MPC_KPI_ECONOMIC_BENEFIT       = 6,
    MPC_KPI_ENERGY_SAVINGS         = 7,
    MPC_KPI_VARIANCE_REDUCTION     = 8,
    MPC_KPI_SETTLING_TIME_RATIO    = 9,
    MPC_KPI_OVERSHOOT_STATISTIC    = 10,
    MPC_KPI_QP_SOLVE_TIME          = 11,
    MPC_KPI_QP_ITERATIONS          = 12,
    MPC_KPI_ACTIVE_CONSTRAINTS     = 13,
    MPC_KPI_MV_TRAVEL              = 14,
    MPC_KPI_MV_SATURATION_FRACTION = 15,
    MPC_KPI_INPUT_OUTPUT_BALANCE   = 16,
    MPC_KPI_CV_CORRELATION_INDEX   = 17,
    MPC_KPI_THROUGHPUT_MAXIMIZATION= 18,
    MPC_KPI_YIELD_IMPROVEMENT      = 19
} mpc_kpi_id_t;

typedef enum {
    MPC_KPI_TIER_EXCELLENT = 0,
    MPC_KPI_TIER_GOOD      = 1,
    MPC_KPI_TIER_FAIR      = 2,
    MPC_KPI_TIER_POOR      = 3,
    MPC_KPI_TIER_CRITICAL  = 4
} mpc_kpi_tier_t;

typedef enum {
    MPC_KPI_OK                 = 0,
    MPC_KPI_ERR_NULL_POINTER   = -1,
    MPC_KPI_ERR_INVALID_PARAM  = -2,
    MPC_KPI_ERR_NOT_ENOUGH_DATA= -3,
    MPC_KPI_ERR_MEMORY         = -4,
    MPC_KPI_ERR_DIVISION_BY_ZERO=-5,
    MPC_KPI_ERR_BUFFER_FULL    = -6,
    MPC_KPI_ERR_BUFFER_EMPTY   = -7,
    MPC_KPI_ERR_INVALID_WINDOW = -8,
    MPC_KPI_ERR_CONVERGENCE    = -9,
    MPC_KPI_ERR_NUMERICAL      = -10,
    MPC_KPI_ERR_NOT_MONOTONIC  = -11
} mpc_kpi_status_t;

typedef enum {
    MPC_KPI_DATA_VALID       = 0x00,
    MPC_KPI_DATA_SUSPECT     = 0x01,
    MPC_KPI_DATA_STALE       = 0x02,
    MPC_KPI_DATA_OUTLIER     = 0x04,
    MPC_KPI_DATA_LIMITED     = 0x08,
    MPC_KPI_DATA_INTERPOLATED= 0x10,
    MPC_KPI_DATA_FLATLINE    = 0x20,
    MPC_KPI_DATA_INSUFFICIENT= 0x40
} mpc_kpi_data_quality_t;

typedef enum {
    MPC_KPI_MODE_IDLE        = 0,
    MPC_KPI_MODE_BASELINING  = 1,
    MPC_KPI_MODE_MONITORING  = 2,
    MPC_KPI_MODE_ALARM       = 3,
    MPC_KPI_MODE_DIAGNOSING  = 4,
    MPC_KPI_MODE_REPORTING   = 5
} mpc_kpi_mode_t;

typedef enum {
    MPC_KPI_SEVERITY_INFO     = 0,
    MPC_KPI_SEVERITY_WARNING  = 1,
    MPC_KPI_SEVERITY_ALERT    = 2,
    MPC_KPI_SEVERITY_CRITICAL = 3
} mpc_kpi_severity_t;

typedef enum {
    MPC_MISMATCH_NONE         = 0,
    MPC_MISMATCH_GAIN_BIAS    = 1,
    MPC_MISMATCH_GAIN_SLOPE   = 2,
    MPC_MISMATCH_DEADTIME     = 3,
    MPC_MISMATCH_TIME_CONSTANT= 4,
    MPC_MISMATCH_INVERSE_RESPONSE=5,
    MPC_MISMATCH_NONLINEARITY = 6,
    MPC_MISMATCH_DISTURBANCE  = 7
} mpc_mismatch_type_t;

typedef struct {
    mpc_kpi_id_t        kpi_id;
    mpc_kpi_category_t  category;
    char                name[MPC_KPI_NAME_MAX];
    char                description[MPC_KPI_DESCRIPTION_MAX];
    double              current_value, target_value, baseline_mean, baseline_std;
    double              min_seen, max_seen, mean_rolling, std_rolling;
    double              skew_rolling, kurtosis_rolling, ewma_value, ewma_lambda;
    double              trend_slope, trend_r_squared;
    mpc_kpi_tier_t      tier;
    mpc_kpi_data_quality_t quality;
    uint64_t            last_update_cycle, num_samples, num_outliers;
    bool                is_alarming;
    mpc_kpi_severity_t  alarm_severity;
    double              alarm_high, alarm_low, alarm_high_high, alarm_low_low, hysteresis;
} mpc_kpi_value_t;

typedef struct {
    uint64_t    window_size, step_size, current_start, current_end;
    bool        is_full;
    uint64_t    num_windows_completed;
} mpc_kpi_window_t;

typedef struct {
    double     *data;
    uint64_t   *timestamps;
    uint64_t    capacity, head, count;
    double      sum, sum_sq, sum_cube, sum_quart, min_val, max_val;
    uint64_t    min_idx, max_idx;
} mpc_kpi_ringbuffer_t;

typedef struct {
    double   lambda, current_ewma, current_ewmvar, current_ewmvol;
    bool     initialized;
    uint64_t count;
} mpc_kpi_ewma_t;

typedef struct {
    double   decision_interval_h, sigma_process, target_mean;
    double   cusum_positive, cusum_negative, min_detectable_shift;
    bool     positive_alarm, negative_alarm;
    uint64_t samples_since_reset;
} mpc_kpi_cusum_t;

typedef struct {
    double  *autocorr, *partial_autocorr;
    int      num_lags;
    double   ljung_box_statistic, ljung_box_pvalue;
    int      ljung_box_dof;
    bool     is_white_noise;
    double   significance_level, critical_value;
} mpc_kpi_autocorr_t;

typedef struct {
    double   harris_index, min_variance_achievable, actual_variance;
    double   variance_reduction_pct, closed_loop_time_constant_est, deadtime_est;
    mpc_kpi_tier_t tier;
    double  *impulse_response_coeffs;
    int      num_impulse_coeffs;
    bool     converged;
    double   normalized_index;
} mpc_kpi_harris_t;

typedef struct {
    mpc_mismatch_type_t primary_type;
    double   mismatch_magnitude, correlation_peak;
    int      correlation_lag;
    double   gain_error_pct, deadtime_error_steps, tau_error_pct, confidence_level;
    double   f_test_statistic, f_critical_value;
    bool     is_significant;
    double  *correlation_function;
    int      correlation_length;
    bool     model_update_recommended;
} mpc_kpi_mismatch_t;

typedef struct {
    mpc_kpi_value_t  *kpi_values;
    int               num_kpi, capacity;
    mpc_kpi_mode_t    mode;
    uint64_t          cycle;
    double            overall_health_score, availability_score, performance_score;
    double            quality_score, economic_score, constraint_score;
    mpc_kpi_tier_t    overall_tier;
    int               num_alarming, num_in_critical;
    uint64_t          last_report_cycle, report_interval;
} mpc_kpi_dashboard_t;

typedef struct {
    mpc_kpi_id_t       kpi_id;
    mpc_kpi_severity_t severity;
    double              high_limit, low_limit, high_high_limit, low_low_limit;
    double              rate_of_change_limit;
    uint64_t            min_sustain_cycles, alarm_deadband_cycles;
    bool                enabled;
    char                rule_name[MPC_KPI_NAME_MAX];
} mpc_kpi_alarm_rule_t;

typedef struct {
    uint64_t           cycle;
    mpc_kpi_value_t   *kpi_snapshot;
    int                num_kpi;
    double             overall_health;
    mpc_kpi_tier_t     tier;
    double             time_weighted_availability, cumulative_economic_benefit;
    uint64_t           report_generation_time;
} mpc_kpi_report_t;

typedef struct {
    bool      baseline_valid;
    uint64_t  baseline_start_cycle, baseline_end_cycle, baseline_duration;
    double    baseline_mean, baseline_std, baseline_median, baseline_mad;
    double    baseline_p5, baseline_p95;
    double    normality_sw_statistic, normality_pvalue;
    bool      is_normal;
    double    cv_statistic;
    int       num_samples_used, num_outliers_removed;
} mpc_kpi_baseline_t;

/* === Enum to String === */
const char *mpc_kpi_category_string(mpc_kpi_category_t cat);
const char *mpc_kpi_id_string(mpc_kpi_id_t id);
const char *mpc_kpi_tier_string(mpc_kpi_tier_t tier);
const char *mpc_kpi_status_string(mpc_kpi_status_t status);
const char *mpc_kpi_mode_string(mpc_kpi_mode_t mode);
const char *mpc_kpi_severity_string(mpc_kpi_severity_t sev);
const char *mpc_mismatch_type_string(mpc_mismatch_type_t mt);

/* === KPI Value Lifecycle === */
mpc_kpi_status_t mpc_kpi_value_init(mpc_kpi_value_t *kv, mpc_kpi_id_t id, mpc_kpi_category_t cat);
void mpc_kpi_value_reset(mpc_kpi_value_t *kv);
mpc_kpi_status_t mpc_kpi_value_update(mpc_kpi_value_t *kv, double new_value, uint64_t cycle);
mpc_kpi_tier_t mpc_kpi_classify_value(double value, double target, double std);
mpc_kpi_status_t mpc_kpi_value_copy(mpc_kpi_value_t *dst, const mpc_kpi_value_t *src);

/* === Ring Buffer === */
mpc_kpi_status_t mpc_kpi_ringbuffer_init(mpc_kpi_ringbuffer_t *rb, uint64_t capacity);
void mpc_kpi_ringbuffer_free(mpc_kpi_ringbuffer_t *rb);
mpc_kpi_status_t mpc_kpi_ringbuffer_push(mpc_kpi_ringbuffer_t *rb, double value, uint64_t ts);
double mpc_kpi_ringbuffer_mean(const mpc_kpi_ringbuffer_t *rb);
double mpc_kpi_ringbuffer_variance(const mpc_kpi_ringbuffer_t *rb);
double mpc_kpi_ringbuffer_std(const mpc_kpi_ringbuffer_t *rb);
double mpc_kpi_ringbuffer_skewness(const mpc_kpi_ringbuffer_t *rb);
double mpc_kpi_ringbuffer_kurtosis(const mpc_kpi_ringbuffer_t *rb);
double mpc_kpi_ringbuffer_percentile(const mpc_kpi_ringbuffer_t *rb, double p);
double mpc_kpi_ringbuffer_min(const mpc_kpi_ringbuffer_t *rb);
double mpc_kpi_ringbuffer_max(const mpc_kpi_ringbuffer_t *rb);
uint64_t mpc_kpi_ringbuffer_count(const mpc_kpi_ringbuffer_t *rb);
mpc_kpi_status_t mpc_kpi_ringbuffer_get_window(const mpc_kpi_ringbuffer_t *rb, uint64_t start, uint64_t end, double *out, uint64_t *count);

/* === EWMA === */
mpc_kpi_status_t mpc_kpi_ewma_init(mpc_kpi_ewma_t *ewma, double lambda);
mpc_kpi_status_t mpc_kpi_ewma_update(mpc_kpi_ewma_t *ewma, double new_value);
double mpc_kpi_ewma_get_mean(const mpc_kpi_ewma_t *ewma);
double mpc_kpi_ewma_get_variance(const mpc_kpi_ewma_t *ewma);
double mpc_kpi_ewma_get_volatility(const mpc_kpi_ewma_t *ewma);
void mpc_kpi_ewma_reset(mpc_kpi_ewma_t *ewma);

/* === CUSUM === */
mpc_kpi_status_t mpc_kpi_cusum_init(mpc_kpi_cusum_t *cs, double target, double sigma, double h, double delta);
mpc_kpi_status_t mpc_kpi_cusum_update(mpc_kpi_cusum_t *cs, double value);
void mpc_kpi_cusum_reset(mpc_kpi_cusum_t *cs);
double mpc_kpi_cusum_positive_stat(const mpc_kpi_cusum_t *cs);
double mpc_kpi_cusum_negative_stat(const mpc_kpi_cusum_t *cs);
bool mpc_kpi_cusum_is_alarm(const mpc_kpi_cusum_t *cs);

/* === Dashboard === */
mpc_kpi_status_t mpc_kpi_dashboard_init(mpc_kpi_dashboard_t *db, int capacity);
void mpc_kpi_dashboard_free(mpc_kpi_dashboard_t *db);
mpc_kpi_status_t mpc_kpi_dashboard_register_kpi(mpc_kpi_dashboard_t *db, mpc_kpi_id_t id, mpc_kpi_category_t cat);
mpc_kpi_status_t mpc_kpi_dashboard_update(mpc_kpi_dashboard_t *db, mpc_kpi_id_t id, double value, uint64_t cycle);
mpc_kpi_status_t mpc_kpi_dashboard_compute_health(mpc_kpi_dashboard_t *db);
int mpc_kpi_dashboard_count_alarming(const mpc_kpi_dashboard_t *db);
int mpc_kpi_dashboard_count_tier(const mpc_kpi_dashboard_t *db, mpc_kpi_tier_t tier);
mpc_kpi_value_t *mpc_kpi_dashboard_find(mpc_kpi_dashboard_t *db, mpc_kpi_id_t id);

/* === Alarms & Reports === */
mpc_kpi_status_t mpc_kpi_alarm_init(mpc_kpi_alarm_rule_t *ar, mpc_kpi_id_t id, mpc_kpi_severity_t sev);
mpc_kpi_severity_t mpc_kpi_alarm_evaluate(const mpc_kpi_alarm_rule_t *ar, double current, double prev, uint64_t cycles_since_change);
bool mpc_kpi_alarm_is_active(const mpc_kpi_alarm_rule_t *ar, double current);
mpc_kpi_status_t mpc_kpi_report_init(mpc_kpi_report_t *rpt, int num_kpi);
void mpc_kpi_report_free(mpc_kpi_report_t *rpt);
mpc_kpi_status_t mpc_kpi_report_snapshot(mpc_kpi_report_t *rpt, const mpc_kpi_dashboard_t *db, uint64_t cycle);

/* === Utility Functions === */
double mpc_kpi_weighted_health_score(const double *scores, const double *weights, int n);
mpc_kpi_tier_t mpc_kpi_health_score_to_tier(double score);
double mpc_kpi_co2_reduction_estimate(double energy_savings_kwh, double grid_emission_factor);
double mpc_kpi_energy_intensity_index(double energy_consumed, double production_rate, double baseline_eii);

#ifdef __cplusplus
}
#endif

#endif
