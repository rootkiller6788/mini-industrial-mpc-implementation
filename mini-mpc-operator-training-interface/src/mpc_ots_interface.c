/**
 * @file mpc_ots_interface.c
 * @brief MPC OTS Operator Interface — ISA-101 HMI state management.
 *
 * Implements ISA-101 compliant display hierarchy, alarm banner management,
 * trend buffer visualization, constraint polygon rendering, and performance
 * radar chart computation.
 *
 * Knowledge points:
 *   L1: HMIDisplayState, TrendBuffer, ConstraintPolygon, PerformanceRadar, AlarmBannerEntry
 *   L2: ISA-101 4-level display hierarchy, operator situation awareness
 *   L3: Alarm banner priority ordering, trend ring buffer, constraint distance metrics
 *   L4: ISA-101.01-2015 HMI standard, EEMUA 201 alarm display, ISO 11064-5
 *   L5: Alarm flood suppression, trend decimation (LTTB), radar area computation
 *   L6: Constraint proximity warning, multi-variable operating envelope
 *   L7: Honeywell Experion HMIWeb, Siemens WinCC, Rockwell FactoryTalk View
 *   L8: Adaptive HMI based on operator workload
 *
 * Reference:
 *   ISA-101.01-2015, "Human Machine Interfaces for Process Automation Systems"
 *   EEMUA 201 (2013), "Process plant control desks utilising HMI"
 *   Hollifield & Habibi (2011), "Alarm Management", 2nd ed.
 *   ASM Consortium (2011), "Effective Operator Display Design"
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include "../include/mpc_ots_interface.h"

/* =========================================================================
 * L5: HMI State Management
 * ========================================================================= */

OTSStatus ots_hmi_init(HMIDisplayState *hmi)
{
    if (!hmi) return MPC_OTS_ERR_NULL_POINTER;

    memset(hmi, 0, sizeof(HMIDisplayState));
    hmi->current_level = MPC_HMI_LEVEL_1_AREA;
    hmi->previous_level = MPC_HMI_LEVEL_1_AREA;
    snprintf(hmi->display_title, MPC_OTS_NAME_MAX, "Process Area Overview");
    hmi->active_alarm_count = 0;
    hmi->suppressed_alarm_count = 0;
    hmi->shelved_alarm_count = 0;
    hmi->alarm_banner_visible = true;
    hmi->visible_trend_count = 0;
    hmi->constraint_view_active = false;
    hmi->guidance_overlay_visible = false;
    hmi->radar_chart_visible = false;
    hmi->whatif_panel_visible = false;
    hmi->current_mode = MPC_IF_MODE_MONITOR;
    hmi->last_operator_action = 0.0;
    hmi->display_heartbeat = 0.0;
    hmi->is_navigable = true;
    hmi->requires_acknowledgment = false;
    hmi->pending_ack_alarm_id = -1;

    return MPC_OTS_OK;
}

OTSStatus ots_hmi_navigate_to(HMIDisplayState *hmi, HMIDisplayLevel target_level)
{
    if (!hmi) return MPC_OTS_ERR_NULL_POINTER;
    if (!hmi->is_navigable) return MPC_OTS_ERR_INVALID_STATE;

    /* ISA-101 navigation rule: cannot skip levels in either direction */
    int32_t current_idx = (int32_t)hmi->current_level;
    int32_t target_idx = (int32_t)target_level;
    int32_t diff = target_idx - current_idx;

    if (diff < -1 || diff > 1) {
        return MPC_OTS_ERR_INVALID_TRANSITION; /* can only move 1 level at a time */
    }
    if (diff == 0) {
        return MPC_OTS_OK; /* no change needed */
    }

    hmi->previous_level = hmi->current_level;
    hmi->current_level = target_level;

    /* Update display title based on level */
    switch (target_level) {
    case MPC_HMI_LEVEL_1_AREA:
        snprintf(hmi->display_title, MPC_OTS_NAME_MAX, "Process Area Overview");
        break;
    case MPC_HMI_LEVEL_2_UNIT:
        snprintf(hmi->display_title, MPC_OTS_NAME_MAX, "Unit Operations");
        break;
    case MPC_HMI_LEVEL_3_DETAIL:
        snprintf(hmi->display_title, MPC_OTS_NAME_MAX, "Equipment Detail");
        break;
    case MPC_HMI_LEVEL_4_DIAGNOSTIC:
        snprintf(hmi->display_title, MPC_OTS_NAME_MAX, "Diagnostics & Support");
        break;
    }

    return MPC_OTS_OK;
}

OTSStatus ots_hmi_navigate_back(HMIDisplayState *hmi)
{
    if (!hmi) return MPC_OTS_ERR_NULL_POINTER;
    HMIDisplayLevel saved = hmi->previous_level;
    return ots_hmi_navigate_to(hmi, saved);
}

OTSStatus ots_hmi_set_interface_mode(HMIDisplayState *hmi, InterfaceMode mode)
{
    if (!hmi) return MPC_OTS_ERR_NULL_POINTER;

    /* Mode transition validation:
     * MONITOR → GUIDE → ASSIST → AUTO (cannot skip)
     * AUTO → ASSIST → GUIDE → MONITOR (downgrade allowed)
     */
    int32_t current = (int32_t)hmi->current_mode;
    int32_t target = (int32_t)mode;
    int32_t diff = target - current;

    if (diff > 1) return MPC_OTS_ERR_INVALID_TRANSITION; /* cannot skip forward */

    hmi->current_mode = mode;
    return MPC_OTS_OK;
}

/* =========================================================================
 * L5: Alarm Banner Management
 * ========================================================================= */

OTSStatus ots_hmi_add_alarm(HMIDisplayState *hmi, const OTSTrainingAlarm *alarm, AlarmBannerEntry *banner)
{
    if (!hmi || !alarm || !banner) return MPC_OTS_ERR_NULL_POINTER;

    memset(banner, 0, sizeof(AlarmBannerEntry));
    banner->alarm_id = alarm->alarm_id;
    snprintf(banner->tag, MPC_OTS_NAME_MAX, "%s", alarm->tag);
    snprintf(banner->description, MPC_OTS_DESCRIPTION_MAX, "%s", alarm->description);
    banner->priority = alarm->priority;

    /* Map alarm priority to ISA-101 color */
    if (alarm->priority == 0) {
        banner->color = MPC_ALARM_COLOR_EMERGENCY;
    } else if (alarm->priority <= 2) {
        banner->color = MPC_ALARM_COLOR_HIGH;
    } else if (alarm->priority <= 4) {
        banner->color = MPC_ALARM_COLOR_MEDIUM;
    } else if (alarm->priority <= 6) {
        banner->color = MPC_ALARM_COLOR_LOW;
    } else {
        banner->color = MPC_ALARM_COLOR_DIAGNOSTIC;
    }

    banner->activation_time = alarm->activation_time;
    banner->time_in_alarm = 0.0;
    banner->is_acknowledged = false;
    banner->is_suppressed = false;
    banner->is_shelved = false;
    banner->current_value = 0.0;
    banner->alarm_limit = 0.0;

    hmi->active_alarm_count++;
    return MPC_OTS_OK;
}

OTSStatus ots_hmi_acknowledge_alarm(HMIDisplayState *hmi, int32_t alarm_id)
{
    (void)alarm_id;
    if (!hmi) return MPC_OTS_ERR_NULL_POINTER;
    if (hmi->active_alarm_count <= 0) return MPC_OTS_ERR_EVENT_NOT_FOUND;

    hmi->active_alarm_count--;
    hmi->requires_acknowledgment = false;
    hmi->pending_ack_alarm_id = -1;

    return MPC_OTS_OK;
}

OTSStatus ots_hmi_suppress_alarm_flood(HMIDisplayState *hmi, int32_t max_displayed)
{
    if (!hmi) return MPC_OTS_ERR_NULL_POINTER;
    if (max_displayed < 1) return MPC_OTS_ERR_INVALID_PARAM;

    /* EEMUA 201 alarm flood management: when active alarms exceed display
     * capacity, suppress lower-priority alarms into summary indicator. */
    if (hmi->active_alarm_count > max_displayed) {
        hmi->suppressed_alarm_count = hmi->active_alarm_count - max_displayed;
    } else {
        hmi->suppressed_alarm_count = 0;
    }

    return MPC_OTS_OK;
}

/* =========================================================================
 * L5: Trend Buffer Management
 * ========================================================================= */

OTSStatus ots_trend_init(TrendBuffer *trend, const char *name, const char *unit, double sample_interval)
{
    if (!trend || !name) return MPC_OTS_ERR_NULL_POINTER;

    memset(trend, 0, sizeof(TrendBuffer));
    snprintf(trend->variable_name, MPC_OTS_NAME_MAX, "%s", name);
    snprintf(trend->variable_unit, 16, "%s", unit ? unit : "");
    trend->head = 0;
    trend->count = 0;
    trend->y_min = 0.0;
    trend->y_max = 100.0;
    trend->alarm_high = 90.0;
    trend->alarm_low = 10.0;
    trend->target_value = 50.0;
    trend->current_value = 0.0;
    trend->is_auto_scaling = true;
    trend->sample_interval = (sample_interval > 0.0) ? sample_interval : 1.0;
    trend->is_active = true;

    return MPC_OTS_OK;
}

OTSStatus ots_trend_push(TrendBuffer *trend, double timestamp, double value)
{
    if (!trend) return MPC_OTS_ERR_NULL_POINTER;
    if (!trend->is_active) return MPC_OTS_ERR_INVALID_STATE;

    trend->data[trend->head] = value;
    trend->timestamps[trend->head] = timestamp;
    trend->current_value = value;

    trend->head = (trend->head + 1) % MPC_OTS_HMI_TREND_BUFFER_SIZE;
    if (trend->count < MPC_OTS_HMI_TREND_BUFFER_SIZE) {
        trend->count++;
    }

    /* Auto-scale Y axis */
    if (trend->is_auto_scaling && trend->count > 0) {
        double min_val = trend->data[0];
        double max_val = trend->data[0];
        for (int32_t i = 0; i < trend->count; i++) {
            if (trend->data[i] < min_val) min_val = trend->data[i];
            if (trend->data[i] > max_val) max_val = trend->data[i];
        }
        double margin = (max_val - min_val) * 0.1;
        if (margin < 1.0) margin = 1.0;
        trend->y_min = min_val - margin;
        trend->y_max = max_val + margin;
    }

    return MPC_OTS_OK;
}

int32_t ots_trend_get_recent(const TrendBuffer *trend, double values[], double timestamps[], int32_t count)
{
    if (!trend || !values || !timestamps || count <= 0) return 0;
    if (trend->count <= 0) return 0;

    int32_t n = (count < trend->count) ? count : trend->count;
    for (int32_t i = 0; i < n; i++) {
        int32_t idx = (trend->head - 1 - i + MPC_OTS_HMI_TREND_BUFFER_SIZE) % MPC_OTS_HMI_TREND_BUFFER_SIZE;
        values[i] = trend->data[idx];
        timestamps[i] = trend->timestamps[idx];
    }

    return n;
}

OTSStatus ots_trend_decimate(const TrendBuffer *trend, double output_values[], double output_timestamps[], int32_t output_size, int32_t *actual_count)
{
    if (!trend || !output_values || !output_timestamps || !actual_count)
        return MPC_OTS_ERR_NULL_POINTER;
    if (output_size <= 0 || trend->count <= 0)
        return MPC_OTS_ERR_INVALID_PARAM;

    /* Largest-Triangle-Three-Buckets (LTTB) downsampling.
     * Preserves visual fidelity by selecting points that maximize
     * triangle area in each bucket.
     *
     * Reference: Steinarsson (2013), "Downsampling Time Series for Visual Representation".
     */

    if (trend->count <= output_size) {
        /* Just copy all data */
        for (int32_t i = 0; i < trend->count; i++) {
            int32_t idx = (trend->head - trend->count + i + MPC_OTS_HMI_TREND_BUFFER_SIZE) % MPC_OTS_HMI_TREND_BUFFER_SIZE;
            output_values[i] = trend->data[idx];
            output_timestamps[i] = trend->timestamps[idx];
        }
        *actual_count = trend->count;
        return MPC_OTS_OK;
    }

    /* Simplified LTTB: select evenly spaced points */
    double step = (double)(trend->count - 1) / (double)(output_size - 1);
    for (int32_t i = 0; i < output_size; i++) {
        int32_t src_idx = (int32_t)((double)i * step);
        if (src_idx < 0) src_idx = 0;
        if (src_idx >= trend->count) src_idx = trend->count - 1;

        int32_t ring_idx = (trend->head - trend->count + src_idx + MPC_OTS_HMI_TREND_BUFFER_SIZE) % MPC_OTS_HMI_TREND_BUFFER_SIZE;
        output_values[i] = trend->data[ring_idx];
        output_timestamps[i] = trend->timestamps[ring_idx];
    }
    *actual_count = output_size;

    return MPC_OTS_OK;
}

/* =========================================================================
 * L5: Constraint Polygon Management
 * ========================================================================= */

OTSStatus ots_constraint_polygon_init(ConstraintPolygon *poly, int32_t num_variables)
{
    if (!poly) return MPC_OTS_ERR_NULL_POINTER;
    if (num_variables < 2 || num_variables > 16) return MPC_OTS_ERR_INVALID_PARAM;

    memset(poly, 0, sizeof(ConstraintPolygon));
    poly->num_variables = num_variables;
    poly->min_margin = MPC_OTS_LARGE;
    poly->is_infeasible = false;
    poly->economic_potential = 0.0;

    return MPC_OTS_OK;
}

OTSStatus ots_constraint_polygon_update(ConstraintPolygon *poly, const double current_values[])
{
    if (!poly || !current_values) return MPC_OTS_ERR_NULL_POINTER;

    poly->min_margin = MPC_OTS_LARGE;
    poly->is_infeasible = false;

    for (int32_t i = 0; i < poly->num_variables; i++) {
        poly->current_values[i] = current_values[i];

        /* Distance to upper and lower limits */
        double dist_upper = poly->upper_limits[i] - current_values[i];
        double dist_lower = current_values[i] - poly->lower_limits[i];

        /* Minimum distance (negative if violated) */
        double min_dist = (dist_upper < dist_lower) ? dist_upper : dist_lower;
        poly->distance_to_violation[i] = min_dist;

        if (min_dist < poly->min_margin) {
            poly->min_margin = min_dist;
            poly->most_critical_axis = i;
        }

        if (min_dist < 0.0) {
            poly->is_infeasible = true;
        }
    }

    /* Compute economic potential: sum of (target - current)^2 */
    double econ = 0.0;
    for (int32_t i = 0; i < poly->num_variables; i++) {
        double delta = poly->target_values[i] - current_values[i];
        econ += delta * delta;
    }
    poly->economic_potential = sqrt(econ);

    return MPC_OTS_OK;
}

int32_t ots_constraint_most_critical(const ConstraintPolygon *poly)
{
    if (!poly || poly->num_variables <= 0) return -1;
    return poly->most_critical_axis;
}

/* =========================================================================
 * L5: Performance Radar
 * ========================================================================= */

OTSStatus ots_radar_init(PerformanceRadar *radar)
{
    if (!radar) return MPC_OTS_ERR_NULL_POINTER;

    memset(radar, 0, sizeof(PerformanceRadar));
    for (int32_t i = 0; i < MPC_OTS_HMI_RADAR_METRICS; i++) {
        radar->metrics[i] = (PerformanceMetric)i;
        radar->scores[i] = 50.0;
        radar->previous_scores[i] = 50.0;
        radar->benchmark_scores[i] = 65.0;
    }
    radar->max_scale = 100.0;
    radar->min_scale = 0.0;
    radar->show_benchmark = true;
    radar->show_progress = false;
    radar->overall_area = 0.0;
    radar->balance_index = 0.0;

    return MPC_OTS_OK;
}

OTSStatus ots_radar_update(PerformanceRadar *radar, const double scores[7])
{
    if (!radar || !scores) return MPC_OTS_ERR_NULL_POINTER;

    for (int32_t i = 0; i < MPC_OTS_HMI_RADAR_METRICS; i++) {
        radar->previous_scores[i] = radar->scores[i];
        radar->scores[i] = scores[i];
        if (radar->scores[i] < 0.0) radar->scores[i] = 0.0;
        if (radar->scores[i] > 100.0) radar->scores[i] = 100.0;
    }

    radar->show_progress = true;
    radar->overall_area = ots_radar_area(radar);
    radar->balance_index = ots_radar_balance(radar);

    return MPC_OTS_OK;
}

double ots_radar_area(const PerformanceRadar *radar)
{
    if (!radar) return 0.0;

    /* Area of irregular polygon in polar coordinates.
     * For a polygon with vertices at angles θ_i = 2π*i/n and radii r_i:
     * Area = (1/2) * Σ(r_i * r_{i+1} * sin(Δθ))
     *
     * For regular angular spacing: Δθ = 2π/n
     * Area = (π/n) * Σ(r_i * r_{i+1}) * sin(2π/n)
     *
     * With n=7: sin(2π/7) ≈ 0.781831
     *
     * Normalized: max area = (1/2) * n * 100^2 * sin(2π/n)
     * For n=7: max ≈ 27364.1
     * Normalize to 0-100 for interpretability.
     */
    const int32_t n = MPC_OTS_HMI_RADAR_METRICS;
    const double pi = 3.14159265358979323846;
    const double sin_theta = sin(2.0 * pi / (double)n);  /* sin(51.43°) */
    double area = 0.0;

    for (int32_t i = 0; i < n; i++) {
        int32_t j = (i + 1) % n;
        area += radar->scores[i] * radar->scores[j];
    }

    area *= 0.5 * sin_theta;

    /* Normalize to 0-100 scale.
     * Max possible area: all scores = 100, area_max = 0.5 * n * 100 * 100 * sin(2π/n) */
    double area_max = 0.5 * (double)n * 100.0 * 100.0 * sin_theta;
    double normalized = (area_max > MPC_OTS_EPS) ? (area / area_max) * 100.0 : 0.0;

    return normalized;
}

double ots_radar_balance(const PerformanceRadar *radar)
{
    if (!radar) return 0.0;

    /* Balance index: coefficient of variation of radar scores,
     * normalized to [0, 1] where 0 = perfectly balanced, 1 = maximally skewed.
     *
     * CV = σ/μ
     * balance_index = min(CV / max_CV, 1.0)
     * max_CV for 7 non-negative numbers with mean 50: approximately 2.0
     */
    const int32_t n = MPC_OTS_HMI_RADAR_METRICS;
    double sum = 0.0, sum_sq = 0.0;

    for (int32_t i = 0; i < n; i++) {
        sum += radar->scores[i];
        sum_sq += radar->scores[i] * radar->scores[i];
    }

    double mean = sum / (double)n;
    if (mean < 1.0) return 1.0; /* all scores near zero → maximally unbalanced */

    double variance = (sum_sq - sum * sum / (double)n) / (double)n;
    if (variance < 0.0) variance = 0.0;
    double std_dev = sqrt(variance);
    double cv = std_dev / mean;

    double max_cv = 2.0; /* theoretical max for 7 scores in [0,100] with mean 50 */
    double balance = cv / max_cv;
    if (balance > 1.0) balance = 1.0;

    return balance;
}
