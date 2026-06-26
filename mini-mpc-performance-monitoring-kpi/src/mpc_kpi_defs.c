/**
 * @file mpc_kpi_defs.c
 * @brief Core MPC KPI data structure implementations.
 *
 * Implements KPI value lifecycle, ring buffer statistics, EWMA filtering,
 * CUSUM change detection, dashboard management, alarm rules, and reporting.
 *
 * Knowledge points:
 *   L1: KPI struct initialization and validation
 *   L2: Performance tier classification
 *   L3: Ring buffer with O(1) moment updates via running sums
 *   L4: EWMA variance = (1-lambda)*prev_var + lambda*(x-mu)^2
 *   L5: CUSUM: S+ = max(0, S+ + (x-mu0)/sigma - k)
 *   L6: Dashboard health aggregation with weighted scoring
 *
 * Reference:
 *   Hunter (1986) J. Quality Technology 18(4), 203-210 (EWMA control)
 *   Page (1954) Biometrika 41(1/2), 100-115 (CUSUM)
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../include/mpc_kpi_defs.h"

/* =========================================================================
 * L1: Enum-to-string conversion functions
 * ========================================================================= */

const char *mpc_kpi_category_string(mpc_kpi_category_t cat)
{
    switch (cat) {
    case MPC_KPI_CAT_AVAILABILITY: return "Availability";
    case MPC_KPI_CAT_PERFORMANCE:  return "Performance";
    case MPC_KPI_CAT_QUALITY:      return "Quality";
    case MPC_KPI_CAT_ECONOMIC:     return "Economic";
    case MPC_KPI_CAT_CONSTRAINT:   return "Constraint";
    case MPC_KPI_CAT_DIAGNOSTIC:   return "Diagnostic";
    case MPC_KPI_CAT_ROBUSTNESS:   return "Robustness";
    default: return "Unknown";
    }
}

const char *mpc_kpi_id_string(mpc_kpi_id_t id)
{
    switch (id) {
    case MPC_KPI_HARRIS_INDEX:            return "Harris Index";
    case MPC_KPI_CONTROLLER_UTILIZATION:  return "Controller Utilization";
    case MPC_KPI_CONSTRAINT_SATISFACTION: return "Constraint Satisfaction";
    case MPC_KPI_MODEL_PLANT_MISMATCH:    return "Model-Plant Mismatch";
    case MPC_KPI_PREDICTION_ERROR_RMS:    return "Prediction Error RMS";
    case MPC_KPI_SETPOINT_TRACKING_RMSE:  return "Setpoint Tracking RMSE";
    case MPC_KPI_ECONOMIC_BENEFIT:        return "Economic Benefit";
    case MPC_KPI_ENERGY_SAVINGS:          return "Energy Savings";
    case MPC_KPI_VARIANCE_REDUCTION:      return "Variance Reduction";
    case MPC_KPI_SETTLING_TIME_RATIO:     return "Settling Time Ratio";
    case MPC_KPI_OVERSHOOT_STATISTIC:     return "Overshoot Statistic";
    case MPC_KPI_QP_SOLVE_TIME:           return "QP Solve Time";
    case MPC_KPI_QP_ITERATIONS:           return "QP Iterations";
    case MPC_KPI_ACTIVE_CONSTRAINTS:      return "Active Constraints";
    case MPC_KPI_MV_TRAVEL:               return "MV Travel";
    case MPC_KPI_MV_SATURATION_FRACTION:  return "MV Saturation Fraction";
    case MPC_KPI_INPUT_OUTPUT_BALANCE:    return "I/O Balance";
    case MPC_KPI_CV_CORRELATION_INDEX:    return "CV Correlation Index";
    case MPC_KPI_THROUGHPUT_MAXIMIZATION: return "Throughput Maximization";
    case MPC_KPI_YIELD_IMPROVEMENT:       return "Yield Improvement";
    default: return "Unknown KPI";
    }
}

const char *mpc_kpi_tier_string(mpc_kpi_tier_t tier)
{
    switch (tier) {
    case MPC_KPI_TIER_EXCELLENT: return "Excellent";
    case MPC_KPI_TIER_GOOD:      return "Good";
    case MPC_KPI_TIER_FAIR:      return "Fair";
    case MPC_KPI_TIER_POOR:      return "Poor";
    case MPC_KPI_TIER_CRITICAL:  return "Critical";
    default: return "Unknown";
    }
}

const char *mpc_kpi_status_string(mpc_kpi_status_t status)
{
    switch (status) {
    case MPC_KPI_OK:                  return "OK";
    case MPC_KPI_ERR_NULL_POINTER:    return "Null Pointer";
    case MPC_KPI_ERR_INVALID_PARAM:   return "Invalid Parameter";
    case MPC_KPI_ERR_NOT_ENOUGH_DATA: return "Not Enough Data";
    case MPC_KPI_ERR_MEMORY:          return "Memory Error";
    case MPC_KPI_ERR_DIVISION_BY_ZERO:return "Division by Zero";
    case MPC_KPI_ERR_BUFFER_FULL:     return "Buffer Full";
    case MPC_KPI_ERR_BUFFER_EMPTY:    return "Buffer Empty";
    case MPC_KPI_ERR_INVALID_WINDOW:  return "Invalid Window";
    case MPC_KPI_ERR_CONVERGENCE:     return "Convergence Error";
    case MPC_KPI_ERR_NUMERICAL:       return "Numerical Error";
    case MPC_KPI_ERR_NOT_MONOTONIC:   return "Not Monotonic";
    default: return "Unknown Status";
    }
}

const char *mpc_kpi_mode_string(mpc_kpi_mode_t mode)
{
    switch (mode) {
    case MPC_KPI_MODE_IDLE:       return "Idle";
    case MPC_KPI_MODE_BASELINING: return "Baselining";
    case MPC_KPI_MODE_MONITORING: return "Monitoring";
    case MPC_KPI_MODE_ALARM:      return "Alarm";
    case MPC_KPI_MODE_DIAGNOSING: return "Diagnosing";
    case MPC_KPI_MODE_REPORTING:  return "Reporting";
    default: return "Unknown";
    }
}

const char *mpc_kpi_severity_string(mpc_kpi_severity_t sev)
{
    switch (sev) {
    case MPC_KPI_SEVERITY_INFO:     return "Info";
    case MPC_KPI_SEVERITY_WARNING:  return "Warning";
    case MPC_KPI_SEVERITY_ALERT:    return "Alert";
    case MPC_KPI_SEVERITY_CRITICAL: return "Critical";
    default: return "Unknown";
    }
}

const char *mpc_mismatch_type_string(mpc_mismatch_type_t mt)
{
    switch (mt) {
    case MPC_MISMATCH_NONE:           return "None";
    case MPC_MISMATCH_GAIN_BIAS:      return "Gain Bias";
    case MPC_MISMATCH_GAIN_SLOPE:     return "Gain Slope";
    case MPC_MISMATCH_DEADTIME:       return "Deadtime";
    case MPC_MISMATCH_TIME_CONSTANT:  return "Time Constant";
    case MPC_MISMATCH_INVERSE_RESPONSE:return "Inverse Response";
    case MPC_MISMATCH_NONLINEARITY:   return "Nonlinearity";
    case MPC_MISMATCH_DISTURBANCE:    return "Disturbance";
    default: return "Unknown";
    }
}

/* =========================================================================
 * L1: KPI value lifecycle management
 * ========================================================================= */

mpc_kpi_status_t mpc_kpi_value_init(mpc_kpi_value_t *kv, mpc_kpi_id_t id, mpc_kpi_category_t cat)
{
    if (!kv) return MPC_KPI_ERR_NULL_POINTER;
    memset(kv, 0, sizeof(mpc_kpi_value_t));
    kv->kpi_id = id;
    kv->category = cat;
    snprintf(kv->name, MPC_KPI_NAME_MAX, "%s", mpc_kpi_id_string(id));
    snprintf(kv->description, MPC_KPI_DESCRIPTION_MAX, "%s KPI", mpc_kpi_category_string(cat));
    kv->current_value = 0.0;
    kv->target_value = 1.0;
    kv->baseline_mean = 0.0;
    kv->baseline_std = 1.0;
    kv->min_seen = 1e30;
    kv->max_seen = -1e30;
    kv->mean_rolling = 0.0;
    kv->std_rolling = 0.0;
    kv->ewma_lambda = MPC_KPI_EWMA_LAMBDA_DEFAULT;
    kv->tier = MPC_KPI_TIER_FAIR;
    kv->quality = MPC_KPI_DATA_VALID;
    kv->last_update_cycle = 0;
    kv->num_samples = 0;
    kv->num_outliers = 0;
    kv->is_alarming = false;
    kv->alarm_severity = MPC_KPI_SEVERITY_INFO;
    kv->alarm_high = 1e30;
    kv->alarm_low = -1e30;
    kv->alarm_high_high = 1e30;
    kv->alarm_low_low = -1e30;
    kv->hysteresis = 0.01;
    return MPC_KPI_OK;
}

void mpc_kpi_value_reset(mpc_kpi_value_t *kv)
{
    if (!kv) return;
    kv->current_value = 0.0;
    kv->min_seen = 1e30;
    kv->max_seen = -1e30;
    kv->mean_rolling = 0.0;
    kv->std_rolling = 0.0;
    kv->skew_rolling = 0.0;
    kv->kurtosis_rolling = 0.0;
    kv->ewma_value = 0.0;
    kv->trend_slope = 0.0;
    kv->trend_r_squared = 0.0;
    kv->num_samples = 0;
    kv->num_outliers = 0;
    kv->is_alarming = false;
}

mpc_kpi_status_t mpc_kpi_value_update(mpc_kpi_value_t *kv, double new_value, uint64_t cycle)
{
    if (!kv) return MPC_KPI_ERR_NULL_POINTER;

    kv->current_value = new_value;
    kv->last_update_cycle = cycle;
    kv->num_samples++;

    if (new_value < kv->min_seen) kv->min_seen = new_value;
    if (new_value > kv->max_seen) kv->max_seen = new_value;

    double old_mean = kv->mean_rolling;
    kv->mean_rolling += (new_value - old_mean) / (double)kv->num_samples;
    if (kv->num_samples >= 2) {
        double delta = new_value - old_mean;
        double delta2 = new_value - kv->mean_rolling;
        double m2 = kv->std_rolling * kv->std_rolling * (double)(kv->num_samples - 1);
        m2 += delta * delta2;
        kv->std_rolling = sqrt(m2 / (double)(kv->num_samples - 1));
    }

    if (kv->num_samples == 1) {
        kv->ewma_value = new_value;
    } else {
        kv->ewma_value = kv->ewma_lambda * new_value + (1.0 - kv->ewma_lambda) * kv->ewma_value;
    }

    kv->tier = mpc_kpi_classify_value(new_value, kv->target_value, kv->baseline_std);

    if (kv->num_samples > 10 && kv->std_rolling < MPC_KPI_EPS) {
        kv->quality = (mpc_kpi_data_quality_t)((int)kv->quality | MPC_KPI_DATA_FLATLINE);
    }

    return MPC_KPI_OK;
}

mpc_kpi_tier_t mpc_kpi_classify_value(double value, double target, double std)
{
    if (std < MPC_KPI_EPS) std = 1.0;
    double normalized_error = fabs(value - target) / std;
    if (normalized_error < 0.5)  return MPC_KPI_TIER_EXCELLENT;
    if (normalized_error < 1.0)  return MPC_KPI_TIER_GOOD;
    if (normalized_error < 2.0)  return MPC_KPI_TIER_FAIR;
    if (normalized_error < 3.0)  return MPC_KPI_TIER_POOR;
    return MPC_KPI_TIER_CRITICAL;
}

mpc_kpi_status_t mpc_kpi_value_copy(mpc_kpi_value_t *dst, const mpc_kpi_value_t *src)
{
    if (!dst || !src) return MPC_KPI_ERR_NULL_POINTER;
    memcpy(dst, src, sizeof(mpc_kpi_value_t));
    return MPC_KPI_OK;
}

/* =========================================================================
 * L2: Ring buffer with O(1) running statistics
 * ========================================================================= */

mpc_kpi_status_t mpc_kpi_ringbuffer_init(mpc_kpi_ringbuffer_t *rb, uint64_t capacity)
{
    if (!rb) return MPC_KPI_ERR_NULL_POINTER;
    if (capacity == 0) return MPC_KPI_ERR_INVALID_PARAM;
    memset(rb, 0, sizeof(mpc_kpi_ringbuffer_t));
    rb->data = (double *)calloc((size_t)capacity, sizeof(double));
    rb->timestamps = (uint64_t *)calloc((size_t)capacity, sizeof(uint64_t));
    if (!rb->data || !rb->timestamps) {
        free(rb->data); rb->data = NULL;
        free(rb->timestamps); rb->timestamps = NULL;
        return MPC_KPI_ERR_MEMORY;
    }
    rb->capacity = capacity;
    rb->head = 0;
    rb->count = 0;
    rb->sum = 0.0;
    rb->sum_sq = 0.0;
    rb->sum_cube = 0.0;
    rb->sum_quart = 0.0;
    rb->min_val = 1e30;
    rb->max_val = -1e30;
    return MPC_KPI_OK;
}

void mpc_kpi_ringbuffer_free(mpc_kpi_ringbuffer_t *rb)
{
    if (!rb) return;
    free(rb->data); rb->data = NULL;
    free(rb->timestamps); rb->timestamps = NULL;
    rb->capacity = 0; rb->count = 0;
}

mpc_kpi_status_t mpc_kpi_ringbuffer_push(mpc_kpi_ringbuffer_t *rb, double value, uint64_t ts)
{
    if (!rb || !rb->data) return MPC_KPI_ERR_NULL_POINTER;

    if (rb->count == rb->capacity) {
        double old_val = rb->data[rb->head];
        rb->sum -= old_val;
        rb->sum_sq -= old_val * old_val;
        rb->sum_cube -= old_val * old_val * old_val;
        rb->sum_quart -= old_val * old_val * old_val * old_val;
    } else {
        rb->count++;
    }

    rb->data[rb->head] = value;
    rb->timestamps[rb->head] = ts;
    rb->sum += value;
    rb->sum_sq += value * value;
    rb->sum_cube += value * value * value;
    rb->sum_quart += value * value * value * value;

    if (value < rb->min_val) { rb->min_val = value; rb->min_idx = rb->head; }
    if (value > rb->max_val) { rb->max_val = value; rb->max_idx = rb->head; }

    rb->head = (rb->head + 1) % rb->capacity;
    return MPC_KPI_OK;
}

double mpc_kpi_ringbuffer_mean(const mpc_kpi_ringbuffer_t *rb)
{
    if (!rb || rb->count == 0) return 0.0;
    return rb->sum / (double)rb->count;
}

double mpc_kpi_ringbuffer_variance(const mpc_kpi_ringbuffer_t *rb)
{
    if (!rb || rb->count < 2) return 0.0;
    double mean = rb->sum / (double)rb->count;
    double var = (rb->sum_sq / (double)rb->count) - (mean * mean);
    return var > 0.0 ? var : 0.0;
}

double mpc_kpi_ringbuffer_std(const mpc_kpi_ringbuffer_t *rb)
{
    return sqrt(mpc_kpi_ringbuffer_variance(rb));
}

double mpc_kpi_ringbuffer_skewness(const mpc_kpi_ringbuffer_t *rb)
{
    if (!rb || rb->count < 3) return 0.0;
    double n = (double)rb->count;
    double m1 = rb->sum / n;
    double m2 = (rb->sum_sq / n) - (m1 * m1);
    double m3 = (rb->sum_cube / n) - 3.0*m1*(rb->sum_sq/n) + 2.0*m1*m1*m1;
    if (m2 < MPC_KPI_EPS) return 0.0;
    return m3 / pow(m2, 1.5);
}

double mpc_kpi_ringbuffer_kurtosis(const mpc_kpi_ringbuffer_t *rb)
{
    if (!rb || rb->count < 4) return 0.0;
    double n = (double)rb->count;
    double m1 = rb->sum / n;
    double m2 = (rb->sum_sq / n) - (m1 * m1);
    double m4 = (rb->sum_quart/n) - 4.0*m1*(rb->sum_cube/n) + 6.0*m1*m1*(rb->sum_sq/n) - 3.0*m1*m1*m1*m1;
    if (m2 < MPC_KPI_EPS) return 0.0;
    return (m4 / (m2 * m2)) - 3.0;
}

double mpc_kpi_ringbuffer_percentile(const mpc_kpi_ringbuffer_t *rb, double p)
{
    if (!rb || rb->count == 0) return 0.0;
    if (p < 0.0) p = 0.0;
    if (p > 1.0) p = 1.0;
    double *sorted = (double *)malloc((size_t)rb->count * sizeof(double));
    if (!sorted) return 0.0;
    for (uint64_t i = 0; i < rb->count; i++) {
        uint64_t idx = (rb->head + rb->capacity - 1 - i) % rb->capacity;
        sorted[i] = rb->data[idx];
    }
    for (uint64_t i = 1; i < rb->count; i++) {
        double key = sorted[i]; int64_t j = (int64_t)i - 1;
        while (j >= 0 && sorted[j] > key) { sorted[j+1] = sorted[j]; j--; }
        sorted[j+1] = key;
    }
    double idx = p * (double)(rb->count - 1);
    uint64_t lo = (uint64_t)idx, hi = (uint64_t)ceil(idx);
    double result = sorted[lo];
    if (lo != hi) result += (sorted[hi] - sorted[lo]) * (idx - (double)lo);
    free(sorted);
    return result;
}

double mpc_kpi_ringbuffer_min(const mpc_kpi_ringbuffer_t *rb) { return rb ? rb->min_val : 0.0; }
double mpc_kpi_ringbuffer_max(const mpc_kpi_ringbuffer_t *rb) { return rb ? rb->max_val : 0.0; }
uint64_t mpc_kpi_ringbuffer_count(const mpc_kpi_ringbuffer_t *rb) { return rb ? rb->count : 0; }

mpc_kpi_status_t mpc_kpi_ringbuffer_get_window(const mpc_kpi_ringbuffer_t *rb,
    uint64_t start, uint64_t end, double *out, uint64_t *count)
{
    if (!rb || !out || !count) return MPC_KPI_ERR_NULL_POINTER;
    if (start >= end || end > rb->count) return MPC_KPI_ERR_INVALID_WINDOW;
    *count = end - start;
    for (uint64_t i = start; i < end; i++) {
        uint64_t idx = (rb->head + rb->capacity - 1 - i) % rb->capacity;
        out[i - start] = rb->data[idx];
    }
    return MPC_KPI_OK;
}

/* =========================================================================
 * L3: EWMA filtering for noisy KPI smoothing
 * ========================================================================= */

mpc_kpi_status_t mpc_kpi_ewma_init(mpc_kpi_ewma_t *ewma, double lambda)
{
    if (!ewma) return MPC_KPI_ERR_NULL_POINTER;
    if (lambda <= 0.0 || lambda > 1.0) return MPC_KPI_ERR_INVALID_PARAM;
    memset(ewma, 0, sizeof(mpc_kpi_ewma_t));
    ewma->lambda = lambda;
    ewma->initialized = false;
    ewma->count = 0;
    return MPC_KPI_OK;
}

mpc_kpi_status_t mpc_kpi_ewma_update(mpc_kpi_ewma_t *ewma, double new_value)
{
    if (!ewma) return MPC_KPI_ERR_NULL_POINTER;
    if (ewma->initialized) {
        double prev_mean = ewma->current_ewma;
        ewma->current_ewma = ewma->lambda * new_value + (1.0 - ewma->lambda) * prev_mean;
        double delta = new_value - prev_mean;
        ewma->current_ewmvar = (1.0 - ewma->lambda) * ewma->current_ewmvar +
                                ewma->lambda * delta * delta;
    } else {
        ewma->current_ewma = new_value;
        ewma->current_ewmvar = 0.0;
        ewma->initialized = true;
    }
    ewma->current_ewmvol = sqrt(ewma->current_ewmvar);
    ewma->count++;
    return MPC_KPI_OK;
}

double mpc_kpi_ewma_get_mean(const mpc_kpi_ewma_t *ewma)  { return ewma ? ewma->current_ewma : 0.0; }
double mpc_kpi_ewma_get_variance(const mpc_kpi_ewma_t *ewma) { return ewma ? ewma->current_ewmvar : 0.0; }
double mpc_kpi_ewma_get_volatility(const mpc_kpi_ewma_t *ewma) { return ewma ? ewma->current_ewmvol : 0.0; }

void mpc_kpi_ewma_reset(mpc_kpi_ewma_t *ewma)
{
    if (!ewma) return;
    ewma->current_ewma = 0.0; ewma->current_ewmvar = 0.0;
    ewma->current_ewmvol = 0.0; ewma->initialized = false; ewma->count = 0;
}

/* =========================================================================
 * L4: CUSUM accumulator for persistent shift detection
 * ========================================================================= */

mpc_kpi_status_t mpc_kpi_cusum_init(mpc_kpi_cusum_t *cs, double target, double sigma, double h, double delta)
{
    if (!cs) return MPC_KPI_ERR_NULL_POINTER;
    if (sigma <= 0.0 || h <= 0.0) return MPC_KPI_ERR_INVALID_PARAM;
    memset(cs, 0, sizeof(mpc_kpi_cusum_t));
    cs->decision_interval_h = h;
    cs->sigma_process = sigma;
    cs->target_mean = target;
    cs->min_detectable_shift = delta;
    return MPC_KPI_OK;
}

mpc_kpi_status_t mpc_kpi_cusum_update(mpc_kpi_cusum_t *cs, double value)
{
    if (!cs) return MPC_KPI_ERR_NULL_POINTER;
    if (cs->sigma_process < MPC_KPI_EPS) return MPC_KPI_ERR_DIVISION_BY_ZERO;

    double stdz = (value - cs->target_mean) / cs->sigma_process;
    double k = cs->min_detectable_shift / (2.0 * cs->sigma_process);
    if (k < MPC_KPI_EPS) k = 0.5;

    cs->cusum_positive = fmax(0.0, cs->cusum_positive + stdz - k);
    cs->cusum_negative = fmax(0.0, cs->cusum_negative - stdz - k);
    cs->positive_alarm = (cs->cusum_positive > cs->decision_interval_h);
    cs->negative_alarm = (cs->cusum_negative > cs->decision_interval_h);
    cs->samples_since_reset++;
    return MPC_KPI_OK;
}

void mpc_kpi_cusum_reset(mpc_kpi_cusum_t *cs)
{
    if (!cs) return;
    cs->cusum_positive = 0.0; cs->cusum_negative = 0.0;
    cs->positive_alarm = false; cs->negative_alarm = false;
    cs->samples_since_reset = 0;
}

double mpc_kpi_cusum_positive_stat(const mpc_kpi_cusum_t *cs) { return cs ? cs->cusum_positive : 0.0; }
double mpc_kpi_cusum_negative_stat(const mpc_kpi_cusum_t *cs) { return cs ? cs->cusum_negative : 0.0; }
bool mpc_kpi_cusum_is_alarm(const mpc_kpi_cusum_t *cs)
{
    return cs ? (cs->positive_alarm || cs->negative_alarm) : false;
}

/* =========================================================================
 * L5: Dashboard management
 * ========================================================================= */

mpc_kpi_status_t mpc_kpi_dashboard_init(mpc_kpi_dashboard_t *db, int capacity)
{
    if (!db) return MPC_KPI_ERR_NULL_POINTER;
    if (capacity <= 0) return MPC_KPI_ERR_INVALID_PARAM;
    memset(db, 0, sizeof(mpc_kpi_dashboard_t));
    db->kpi_values = (mpc_kpi_value_t *)calloc((size_t)capacity, sizeof(mpc_kpi_value_t));
    if (!db->kpi_values) return MPC_KPI_ERR_MEMORY;
    db->capacity = capacity;
    db->mode = MPC_KPI_MODE_IDLE;
    db->report_interval = 3600;
    return MPC_KPI_OK;
}

void mpc_kpi_dashboard_free(mpc_kpi_dashboard_t *db)
{
    if (!db) return;
    free(db->kpi_values);
    db->kpi_values = NULL; db->num_kpi = 0; db->capacity = 0;
}

mpc_kpi_status_t mpc_kpi_dashboard_register_kpi(mpc_kpi_dashboard_t *db, mpc_kpi_id_t id, mpc_kpi_category_t cat)
{
    if (!db) return MPC_KPI_ERR_NULL_POINTER;
    if (db->num_kpi >= db->capacity) return MPC_KPI_ERR_BUFFER_FULL;
    return mpc_kpi_value_init(&db->kpi_values[db->num_kpi++], id, cat);
}

mpc_kpi_status_t mpc_kpi_dashboard_update(mpc_kpi_dashboard_t *db, mpc_kpi_id_t id, double value, uint64_t cycle)
{
    if (!db) return MPC_KPI_ERR_NULL_POINTER;
    mpc_kpi_value_t *kv = mpc_kpi_dashboard_find(db, id);
    if (!kv) return MPC_KPI_ERR_INVALID_PARAM;
    db->cycle = cycle;
    return mpc_kpi_value_update(kv, value, cycle);
}

mpc_kpi_status_t mpc_kpi_dashboard_compute_health(mpc_kpi_dashboard_t *db)
{
    if (!db) return MPC_KPI_ERR_NULL_POINTER;
    if (db->num_kpi == 0) return MPC_KPI_ERR_NOT_ENOUGH_DATA;

    double avail_sum=0, perf_sum=0, qual_sum=0, econ_sum=0, constr_sum=0;
    int avail_cnt=0, perf_cnt=0, qual_cnt=0, econ_cnt=0, constr_cnt=0;
    db->num_alarming = 0; db->num_in_critical = 0;

    for (int i = 0; i < db->num_kpi; i++) {
        mpc_kpi_value_t *kv = &db->kpi_values[i];
        double score = 1.0 - (double)kv->tier / 4.0;
        switch (kv->category) {
        case MPC_KPI_CAT_AVAILABILITY: avail_sum+=score; avail_cnt++; break;
        case MPC_KPI_CAT_PERFORMANCE:  perf_sum+=score;  perf_cnt++;  break;
        case MPC_KPI_CAT_QUALITY:      qual_sum+=score;  qual_cnt++;  break;
        case MPC_KPI_CAT_ECONOMIC:     econ_sum+=score;  econ_cnt++;  break;
        case MPC_KPI_CAT_CONSTRAINT:   constr_sum+=score; constr_cnt++; break;
        default: break;
        }
        if (kv->is_alarming) db->num_alarming++;
        if (kv->tier == MPC_KPI_TIER_CRITICAL) db->num_in_critical++;
    }

    db->availability_score = avail_cnt>0 ? avail_sum/avail_cnt : 0.5;
    db->performance_score  = perf_cnt>0  ? perf_sum/perf_cnt   : 0.5;
    db->quality_score      = qual_cnt>0  ? qual_sum/qual_cnt   : 0.5;
    db->economic_score     = econ_cnt>0  ? econ_sum/econ_cnt   : 0.5;
    db->constraint_score   = constr_cnt>0? constr_sum/constr_cnt : 0.5;

    double scores[] = {db->availability_score, db->performance_score,
                       db->quality_score, db->economic_score, db->constraint_score};
    double weights[] = {0.25, 0.25, 0.15, 0.20, 0.15};
    db->overall_health_score = mpc_kpi_weighted_health_score(scores, weights, 5);
    db->overall_tier = mpc_kpi_health_score_to_tier(db->overall_health_score);
    return MPC_KPI_OK;
}

int mpc_kpi_dashboard_count_alarming(const mpc_kpi_dashboard_t *db)
{
    if (!db) return 0;
    int count = 0;
    for (int i = 0; i < db->num_kpi; i++)
        if (db->kpi_values[i].is_alarming) count++;
    return count;
}

int mpc_kpi_dashboard_count_tier(const mpc_kpi_dashboard_t *db, mpc_kpi_tier_t tier)
{
    if (!db) return 0;
    int count = 0;
    for (int i = 0; i < db->num_kpi; i++)
        if (db->kpi_values[i].tier == tier) count++;
    return count;
}

mpc_kpi_value_t *mpc_kpi_dashboard_find(mpc_kpi_dashboard_t *db, mpc_kpi_id_t id)
{
    if (!db) return NULL;
    for (int i = 0; i < db->num_kpi; i++)
        if (db->kpi_values[i].kpi_id == id) return &db->kpi_values[i];
    return NULL;
}

/* =========================================================================
 * L6: Alarm and reporting operations
 * ========================================================================= */

mpc_kpi_status_t mpc_kpi_alarm_init(mpc_kpi_alarm_rule_t *ar, mpc_kpi_id_t id, mpc_kpi_severity_t sev)
{
    if (!ar) return MPC_KPI_ERR_NULL_POINTER;
    memset(ar, 0, sizeof(mpc_kpi_alarm_rule_t));
    ar->kpi_id = id; ar->severity = sev;
    ar->high_limit = 1e30; ar->low_limit = -1e30;
    ar->high_high_limit = 1e30; ar->low_low_limit = -1e30;
    ar->rate_of_change_limit = 1e30;
    ar->min_sustain_cycles = 1; ar->alarm_deadband_cycles = 5;
    ar->enabled = true;
    snprintf(ar->rule_name, MPC_KPI_NAME_MAX, "%s_alarm", mpc_kpi_id_string(id));
    return MPC_KPI_OK;
}

mpc_kpi_severity_t mpc_kpi_alarm_evaluate(const mpc_kpi_alarm_rule_t *ar, double current, double prev, uint64_t cycles_since_change)
{
    if (!ar || !ar->enabled) return MPC_KPI_SEVERITY_INFO;
    if (cycles_since_change < ar->min_sustain_cycles) return MPC_KPI_SEVERITY_INFO;
    (void)prev;
    if (current > ar->high_high_limit || current < ar->low_low_limit)
        return MPC_KPI_SEVERITY_CRITICAL;
    if (current > ar->high_limit || current < ar->low_limit)
        return ar->severity;
    return MPC_KPI_SEVERITY_INFO;
}

bool mpc_kpi_alarm_is_active(const mpc_kpi_alarm_rule_t *ar, double current)
{
    if (!ar || !ar->enabled) return false;
    return (current > ar->high_limit || current < ar->low_limit);
}

mpc_kpi_status_t mpc_kpi_report_init(mpc_kpi_report_t *rpt, int num_kpi)
{
    if (!rpt) return MPC_KPI_ERR_NULL_POINTER;
    if (num_kpi <= 0) return MPC_KPI_ERR_INVALID_PARAM;
    memset(rpt, 0, sizeof(mpc_kpi_report_t));
    rpt->kpi_snapshot = (mpc_kpi_value_t *)calloc((size_t)num_kpi, sizeof(mpc_kpi_value_t));
    if (!rpt->kpi_snapshot) return MPC_KPI_ERR_MEMORY;
    rpt->num_kpi = num_kpi;
    return MPC_KPI_OK;
}

void mpc_kpi_report_free(mpc_kpi_report_t *rpt)
{
    if (!rpt) return;
    free(rpt->kpi_snapshot); rpt->kpi_snapshot = NULL; rpt->num_kpi = 0;
}

mpc_kpi_status_t mpc_kpi_report_snapshot(mpc_kpi_report_t *rpt, const mpc_kpi_dashboard_t *db, uint64_t cycle)
{
    if (!rpt || !db) return MPC_KPI_ERR_NULL_POINTER;
    int num_copy = (db->num_kpi < rpt->num_kpi) ? db->num_kpi : rpt->num_kpi;
    for (int i = 0; i < num_copy; i++)
        memcpy(&rpt->kpi_snapshot[i], &db->kpi_values[i], sizeof(mpc_kpi_value_t));
    rpt->cycle = cycle; rpt->overall_health = db->overall_health_score;
    rpt->tier = db->overall_tier; rpt->time_weighted_availability = db->availability_score;
    return MPC_KPI_OK;
}

/* =========================================================================
 * L7: Environmental KPI (ISO 50001), Health score utility
 * ========================================================================= */

double mpc_kpi_co2_reduction_estimate(double energy_savings_kwh, double grid_emission_factor)
{
    if (energy_savings_kwh < 0.0 || grid_emission_factor < 0.0) return 0.0;
    return energy_savings_kwh * grid_emission_factor;
}

double mpc_kpi_energy_intensity_index(double energy_consumed, double production_rate, double baseline_eii)
{
    if (production_rate < MPC_KPI_EPS || baseline_eii < MPC_KPI_EPS) return 1.0;
    return (energy_consumed / production_rate) / baseline_eii;
}

double mpc_kpi_weighted_health_score(const double *scores, const double *weights, int n)
{
    if (!scores || !weights || n <= 0) return 0.5;
    double total = 0.0, weight_sum = 0.0;
    for (int i = 0; i < n; i++) { total += scores[i]*weights[i]; weight_sum += weights[i]; }
    return (weight_sum > MPC_KPI_EPS) ? (total/weight_sum) : 0.5;
}

mpc_kpi_tier_t mpc_kpi_health_score_to_tier(double score)
{
    if (score >= 0.90) return MPC_KPI_TIER_EXCELLENT;
    if (score >= 0.75) return MPC_KPI_TIER_GOOD;
    if (score >= 0.50) return MPC_KPI_TIER_FAIR;
    if (score >= 0.25) return MPC_KPI_TIER_POOR;
    return MPC_KPI_TIER_CRITICAL;
}

/* =========================================================================
 * Additional utility: KPI string formatting for HMI display
 * ========================================================================= */

/** Format a KPI value for operator HMI display with tier color code */
int mpc_kpi_format_hmi_string(const mpc_kpi_value_t *kv, char *buf, int buf_size)
{
    if (!kv || !buf || buf_size <= 0) return -1;
    const char *tier_str = mpc_kpi_tier_string(kv->tier);
    return snprintf(buf, (size_t)buf_size, "[%s] %s = %.3f (target: %.3f)",
                    tier_str, kv->name, kv->current_value, kv->target_value);
}

/** Validate that all pointer fields in a dashboard are non-null */
bool mpc_kpi_validate_dashboard_pointers(const mpc_kpi_dashboard_t *db)
{
    if (!db) return false;
    if (!db->kpi_values && db->num_kpi > 0) return false;
    return true;
}
