/**
 * @file mpc_ots_defs.c
 * @brief Core MPC Operator Training Simulator implementation.
 *
 * Implements training session lifecycle, operator profile management,
 * scenario event handling, what-if analysis, and performance scoring.
 *
 * Knowledge points:
 *   L1: OTSState/ScenarioType/Role/Metric/Difficulty enum lifecycles
 *   L2: Operator-in-the-loop training, session state transitions
 *   L3: State machine validation, Elo rating adaptation
 *   L4: ISA-101 compliance, Kirkpatrick evaluation model
 *   L5: Weighted scoring, what-if linear prediction
 *   L6: Session lifecycle with debrief generation
 *   L7: Honeywell UniSim session model, AspenTech OTS framework
 *
 * Reference:
 *   Rawlings, Mayne & Diehl (2017), "Model Predictive Control", 2nd ed.
 *   Honeywell (2020), "UniSim Operations Suite — User Guide"
 *   Kirkpatrick & Kirkpatrick (2006), "Evaluating Training Programs", 3rd ed.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <time.h>
#include "../include/mpc_ots_defs.h"
#include "../include/mpc_ots_assessment.h"

/* =========================================================================
 * L1+L2: Session Lifecycle
 * ========================================================================= */

OTSStatus ots_session_init(TrainingSession *session, int32_t session_id, const char *name)
{
    if (!session) return MPC_OTS_ERR_NULL_POINTER;
    if (session_id < 0) return MPC_OTS_ERR_INVALID_PARAM;
    if (!name) return MPC_OTS_ERR_NULL_POINTER;

    memset(session, 0, sizeof(TrainingSession));
    session->session_id = session_id;
    snprintf(session->session_name, MPC_OTS_NAME_MAX, "%s", name);
    session->scenario_type = MPC_SCENARIO_DISTURBANCE_REJECTION;
    session->difficulty = MPC_DIFFICULTY_BEGINNER;
    session->mode = MPC_TRAINING_MODE_INDIVIDUAL;
    session->fidelity = MPC_FIDELITY_MEDIUM;
    session->interface_mode = MPC_IF_MODE_GUIDE;
    session->state = MPC_OTS_STATE_INIT;
    session->operator_id = -1;
    session->trainer_id = 0;
    session->session_duration = MPC_OTS_DEFAULT_SESSION_TIMEOUT;
    session->session_elapsed = 0.0;
    session->scenario_clock = 0.0;
    session->current_event_index = 0;
    session->total_events = 0;
    session->events_completed = 0;
    session->events_failed = 0;
    session->guidance_messages_shown = 0;
    session->operator_interventions = 0;
    session->current_score = 0.0;
    session->active_alarms = 0;
    session->cpu_utilization = 0.0;
    session->is_paused = false;
    session->requires_debrief = false;
    return MPC_OTS_OK;
}

OTSStatus ots_session_start(TrainingSession *session)
{
    if (!session) return MPC_OTS_ERR_NULL_POINTER;
    if (session->state != MPC_OTS_STATE_INIT && session->state != MPC_OTS_STATE_READY)
        return MPC_OTS_ERR_INVALID_STATE;
    if (session->operator_id < 0)
        return MPC_OTS_ERR_OPERATOR_NOT_FOUND;
    if (session->total_events <= 0)
        return MPC_OTS_ERR_SCENARIO_EMPTY;

    session->state = MPC_OTS_STATE_RUNNING;
    session->session_start_time = (double)time(NULL);
    session->session_elapsed = 0.0;
    session->scenario_clock = 0.0;
    session->is_paused = false;
    return MPC_OTS_OK;
}

OTSStatus ots_session_pause(TrainingSession *session)
{
    if (!session) return MPC_OTS_ERR_NULL_POINTER;
    if (session->state != MPC_OTS_STATE_RUNNING)
        return MPC_OTS_ERR_INVALID_STATE;

    session->state = MPC_OTS_STATE_PAUSED;
    session->is_paused = true;
    return MPC_OTS_OK;
}

OTSStatus ots_session_resume(TrainingSession *session)
{
    if (!session) return MPC_OTS_ERR_NULL_POINTER;
    if (session->state != MPC_OTS_STATE_PAUSED)
        return MPC_OTS_ERR_INVALID_STATE;

    session->state = MPC_OTS_STATE_RUNNING;
    session->is_paused = false;
    return MPC_OTS_OK;
}

OTSStatus ots_session_complete(TrainingSession *session, DebriefReport *report)
{
    (void)report;
    if (!session) return MPC_OTS_ERR_NULL_POINTER;
    if (session->state != MPC_OTS_STATE_RUNNING && session->state != MPC_OTS_STATE_PAUSED)
        return MPC_OTS_ERR_INVALID_STATE;

    session->state = MPC_OTS_STATE_COMPLETED;
    session->requires_debrief = true;
    return MPC_OTS_OK;
}

bool ots_state_transition_valid(OTSState current, OTSState next)
{
    /* Valid transitions in OTS state machine:
     * INIT     → READY, FAILED
     * READY    → RUNNING, FAILED
     * RUNNING  → PAUSED, COMPLETED, FAILED
     * PAUSED   → RUNNING, COMPLETED, FAILED
     * COMPLETED → DEBRIEFING
     * DEBRIEFING → (terminal, can go to INIT for next session)
     * FAILED   → INIT (restart)
     */
    switch (current) {
    case MPC_OTS_STATE_INIT:
        return (next == MPC_OTS_STATE_READY || next == MPC_OTS_STATE_FAILED);
    case MPC_OTS_STATE_READY:
        return (next == MPC_OTS_STATE_RUNNING || next == MPC_OTS_STATE_FAILED);
    case MPC_OTS_STATE_RUNNING:
        return (next == MPC_OTS_STATE_PAUSED || next == MPC_OTS_STATE_COMPLETED || next == MPC_OTS_STATE_FAILED);
    case MPC_OTS_STATE_PAUSED:
        return (next == MPC_OTS_STATE_RUNNING || next == MPC_OTS_STATE_COMPLETED || next == MPC_OTS_STATE_FAILED);
    case MPC_OTS_STATE_COMPLETED:
        return (next == MPC_OTS_STATE_DEBRIEFING);
    case MPC_OTS_STATE_DEBRIEFING:
        return false; /* terminal state */
    case MPC_OTS_STATE_FAILED:
        return (next == MPC_OTS_STATE_INIT); /* restart only */
    default:
        return false;
    }
}

/* =========================================================================
 * L1+L2: Operator Profile Management
 * ========================================================================= */

OTSStatus ots_profile_init(OperatorProfile *profile, int32_t id, const char *name, OperatorRole role)
{
    if (!profile) return MPC_OTS_ERR_NULL_POINTER;
    if (!name) return MPC_OTS_ERR_NULL_POINTER;
    if (id < 0) return MPC_OTS_ERR_INVALID_PARAM;

    memset(profile, 0, sizeof(OperatorProfile));
    snprintf(profile->name, MPC_OTS_NAME_MAX, "%s", name);
    profile->operator_id = id;
    profile->role = role;
    profile->years_experience = 0;
    profile->mpc_training_hours = 0;
    profile->sessions_completed = 0;
    profile->overall_score = 50.0; /* neutral starting score */
    profile->score_response_time = 50.0;
    profile->score_stability = 50.0;
    profile->score_constraints = 50.0;
    profile->score_economic = 50.0;
    profile->score_situation_awareness = 50.0;
    profile->current_elo_rating = 1200.0; /* Elo default for beginners */
    profile->recommended_difficulty = MPC_DIFFICULTY_BEGINNER;
    profile->last_session_id = -1;
    profile->last_session_timestamp = 0.0;
    profile->is_active = false;
    return MPC_OTS_OK;
}

OTSStatus ots_profile_update_after_session(OperatorProfile *profile, const DebriefReport *report)
{
    if (!profile || !report) return MPC_OTS_ERR_NULL_POINTER;
    if (report->session_id < 0) return MPC_OTS_ERR_INVALID_PARAM;

    /* Rolling average update for sub-scores:
     * new_score = (old_score * n + new_value) / (n + 1)
     */
    double n = (double)profile->sessions_completed;
    profile->score_response_time = (profile->score_response_time * n + report->scores_by_metric[MPC_METRIC_RESPONSE_TIME]) / (n + 1.0);
    profile->score_stability = (profile->score_stability * n + report->scores_by_metric[MPC_METRIC_STABILITY_MARGIN]) / (n + 1.0);
    profile->score_constraints = (profile->score_constraints * n + report->scores_by_metric[MPC_METRIC_CONSTRAINT_COMPLIANCE]) / (n + 1.0);
    profile->score_economic = (profile->score_economic * n + report->scores_by_metric[MPC_METRIC_ECONOMIC_OPTIMALITY]) / (n + 1.0);
    profile->score_situation_awareness = (profile->score_situation_awareness * n + report->scores_by_metric[MPC_METRIC_SITUATION_AWARENESS]) / (n + 1.0);
    profile->overall_score = (profile->overall_score * n + report->overall_score) / (n + 1.0);

    profile->sessions_completed++;
    profile->last_session_id = report->session_id;
    profile->last_session_timestamp = (double)time(NULL);
    profile->is_active = false;

    return MPC_OTS_OK;
}

/* =========================================================================
 * L5: Elo Rating System
 * ========================================================================= */

double ots_elo_update(double current_elo, double session_score, double expected_score, double k_factor)
{
    /* Elo formula: R' = R + K * (S - E)
     * R = current rating, K = K-factor, S = actual outcome (0-1),
     * E = expected outcome (0-1).
     *
     * For training: session_score is normalized to [0, 1].
     * K-factor scales with uncertainty (higher for beginners, lower for experts).
     *
     * Reference: Elo (1978), "The Rating of Chessplayers, Past and Present".
     */
    if (k_factor <= 0.0) k_factor = MPC_OTS_DEFAULT_ELO_K;

    double s = session_score / 100.0; /* normalize to [0,1] */
    if (s < 0.0) s = 0.0;
    if (s > 1.0) s = 1.0;

    if (expected_score < 0.0) expected_score = 0.0;
    if (expected_score > 1.0) expected_score = 1.0;

    double adjustment = k_factor * (s - expected_score);

    /* Bound adjustment to prevent rating explosion */
    if (adjustment > k_factor) adjustment = k_factor;
    if (adjustment < -k_factor) adjustment = -k_factor;

    return current_elo + adjustment;
}

ScenarioDifficulty ots_recommend_difficulty(const OperatorProfile *profile)
{
    if (!profile) return MPC_DIFFICULTY_BEGINNER;

    /* Difficulty recommendation based on Elo rating:
     * ELO < 1100  → BEGINNER
     * ELO < 1300  → INTERMEDIATE
     * ELO < 1500  → ADVANCED
     * ELO >= 1500 → EXPERT
     */
    if (profile->current_elo_rating < 1100.0) {
        return MPC_DIFFICULTY_BEGINNER;
    } else if (profile->current_elo_rating < 1300.0) {
        return MPC_DIFFICULTY_INTERMEDIATE;
    } else if (profile->current_elo_rating < 1500.0) {
        return MPC_DIFFICULTY_ADVANCED;
    } else {
        return MPC_DIFFICULTY_EXPERT;
    }
}

/* =========================================================================
 * L1+L2: Scenario Event Management
 * ========================================================================= */

OTSStatus ots_event_init(ScenarioEvent *event, int32_t id, TrainingScenarioType type, double trigger_time)
{
    if (!event) return MPC_OTS_ERR_NULL_POINTER;
    if (id < 0) return MPC_OTS_ERR_INVALID_PARAM;
    if (trigger_time < 0.0) return MPC_OTS_ERR_INVALID_PARAM;

    memset(event, 0, sizeof(ScenarioEvent));
    event->event_id = id;
    event->event_type = type;
    event->trigger_time = trigger_time;
    event->ramp_duration = 0.0;
    snprintf(event->description, MPC_OTS_DESCRIPTION_MAX, "Scenario event %d", id);
    snprintf(event->affected_variable, MPC_OTS_NAME_MAX, "CV_%d", id);
    event->initial_value = 0.0;
    event->target_value = 100.0;
    event->disturbance_magnitude = 10.0;
    event->disturbance_decay_rate = 60.0;
    event->optimal_response_time = 30.0;
    event->max_allowable_deviation = 10.0;
    event->economic_penalty_rate = 100.0;
    event->is_critical = false;
    event->requires_operator_action = true;
    event->guidance_hint_id = -1;
    event->actual_response_time = 0.0;
    event->was_handled = false;
    event->deviation_peak = 0.0;

    return MPC_OTS_OK;
}

OTSStatus ots_event_evaluate_response(ScenarioEvent *event, double response_time, double peak_deviation, PerformanceRecord *record)
{
    if (!event || !record) return MPC_OTS_ERR_NULL_POINTER;
    if (response_time < 0.0 || peak_deviation < 0.0) return MPC_OTS_ERR_INVALID_PARAM;

    event->actual_response_time = response_time;
    event->deviation_peak = peak_deviation;

    double response_score = (event->optimal_response_time > MPC_OTS_EPS)
        ? exp(-pow(response_time - event->optimal_response_time, 2.0) / (2.0 * pow(event->optimal_response_time * 0.5, 2.0))) * 100.0
        : 0.0;

    double deviation_score = (event->max_allowable_deviation > MPC_OTS_EPS)
        ? 100.0 * fmax(0.0, 1.0 - (peak_deviation / event->max_allowable_deviation - 1.0))
        : 100.0;

    if (deviation_score > 100.0) deviation_score = 100.0;
    if (deviation_score < 0.0) deviation_score = 0.0;

    event->was_handled = (response_score >= 50.0 && deviation_score >= 50.0);

    memset(record, 0, sizeof(PerformanceRecord));
    record->event_id = event->event_id;
    record->metric_type = MPC_METRIC_RESPONSE_TIME;
    record->measured_value = response_time;
    record->expected_value = event->optimal_response_time;
    record->normalized_score = response_score;
    record->timestamp = event->trigger_time + response_time;

    return MPC_OTS_OK;
}

/* =========================================================================
 * L5: What-if Analysis
 * ========================================================================= */

OTSStatus ots_whatif_execute(WhatIfQuery *query)
{
    if (!query) return MPC_OTS_ERR_NULL_POINTER;
    if (query->num_variables <= 0 || query->num_variables > MPC_OTS_MAX_WHATIF_VARIABLES)
        return MPC_OTS_ERR_INVALID_PARAM;

    /* Simplified linear prediction: ΔCV = G * ΔMV
     * For training OTS, use diagonal-dominant gain matrix approximation.
     * This provides fast what-if feedback without full nonlinear simulation.
     *
     * G_ij = 1.0 for i=j (self-gain)
     * G_ij = 0.3 for i≠j (coupling) if variables < 5 else 0.15
     */
    double coupling = (query->num_variables < 5) ? 0.3 : 0.15;
    double total_economic = 0.0;
    int32_t violations = 0;

    for (int32_t i = 0; i < query->num_variables; i++) {
        double delta = query->proposed_values[i] - query->current_values[i];
        double cv_delta = delta * 1.0; /* self-gain */

        /* Add coupling from other variables */
        for (int32_t j = 0; j < query->num_variables; j++) {
            if (i == j) continue;
            double other_delta = query->proposed_values[j] - query->current_values[j];
            cv_delta += other_delta * coupling;
        }

        query->predicted_cv_deviations[i] = cv_delta;

        /* Economic impact: estimate $ per unit deviation */
        double economic_unit_cost = 50.0; /* $/unit/hour default */
        total_economic += fabs(cv_delta) * economic_unit_cost;

        /* Check constraint violation: any CV deviation > 10% change */
        if (fabs(cv_delta) > 0.10 * fmax(fabs(query->current_values[i]), 1.0)) {
            violations++;
        }
    }

    query->predicted_economic_impact = total_economic;
    query->predicted_constraint_violations = (double)violations;
    query->confidence_level = 1.0 / (1.0 + coupling * (double)(query->num_variables - 1));
    query->is_safe = (violations == 0);

    if (query->is_safe && total_economic < 1000.0) {
        snprintf(query->recommendation, MPC_OTS_GUIDANCE_TEXT_MAX,
            "What-if analysis: SAFE. Estimated economic impact: $%.1f/hr. Confidence: %.0f%%",
            total_economic, query->confidence_level * 100.0);
    } else if (query->is_safe) {
        snprintf(query->recommendation, MPC_OTS_GUIDANCE_TEXT_MAX,
            "What-if analysis: SAFE but HIGH COST ($%.1f/hr). Consider optimizing.", total_economic);
    } else {
        snprintf(query->recommendation, MPC_OTS_GUIDANCE_TEXT_MAX,
            "WARNING: %d constraint violations predicted. Action NOT recommended.", violations);
        query->is_safe = false;
    }

    return MPC_OTS_OK;
}

bool ots_whatif_is_safe(const WhatIfQuery *query)
{
    if (!query) return false;
    return query->is_safe;
}

/* =========================================================================
 * L5+L6: Performance Scoring
 * ========================================================================= */

double ots_compute_overall_score(const double scores[], int32_t num_metrics, const double weights[])
{
    if (!scores || !weights) return 0.0;
    if (num_metrics <= 0) return 0.0;

    /* Weighted geometric mean: score = exp(Σ w_i * ln(s_i))
     * This penalizes very low individual metric scores.
     * Reference: ASM Consortium multi-attribute utility framework.
     *
     * Handle zero scores gracefully: ln(0) → -∞, but we clamp to avoid this.
     */
    double log_sum = 0.0;
    double weight_sum = 0.0;

    for (int32_t i = 0; i < num_metrics; i++) {
        double s = scores[i];
        if (s < 0.1) s = 0.1; /* clamp to avoid ln(0) */
        if (s > 100.0) s = 100.0;
        log_sum += weights[i] * log(s);
        weight_sum += weights[i];
    }

    if (weight_sum < MPC_OTS_EPS) return 0.0;

    double result = exp(log_sum / weight_sum);
    if (result > 100.0) result = 100.0;
    if (result < 0.0) result = 0.0;

    return result;
}

OTSStatus ots_compute_performance_trend(const PerformanceRecord history[], int32_t count, PerformanceTrend *trend)
{
    if (!history || !trend) return MPC_OTS_ERR_NULL_POINTER;
    if (count < MPC_OTS_MIN_HISTORY) return MPC_OTS_ERR_INVALID_PARAM;

    /* Extract score sequences per metric for OLS regression */
    double sum_x = 0.0, sum_y = 0.0, sum_xy = 0.0, sum_x2 = 0.0;

    for (int32_t i = 0; i < count; i++) {
        double x = (double)i;
        double y = history[i].normalized_score;
        sum_x += x;
        sum_y += y;
        sum_xy += x * y;
        sum_x2 += x * x;
    }

    double n = (double)count;
    double denom = n * sum_x2 - sum_x * sum_x;

    double slope = 0.0;
    double intercept = 0.0;
    double r_squared = 0.0;

    if (fabs(denom) > MPC_OTS_EPS) {
        slope = (n * sum_xy - sum_x * sum_y) / denom;
        intercept = (sum_y - slope * sum_x) / n;

        /* Compute r_squared */
        double ss_res = 0.0, ss_tot = 0.0;
        double y_mean = sum_y / n;
        for (int32_t i = 0; i < count; i++) {
            double y_pred = intercept + slope * (double)i;
            ss_res += pow(history[i].normalized_score - y_pred, 2.0);
            ss_tot += pow(history[i].normalized_score - y_mean, 2.0);
        }
        if (ss_tot > MPC_OTS_EPS) {
            r_squared = 1.0 - ss_res / ss_tot;
        }
    }

    memset(trend, 0, sizeof(PerformanceTrend));
    trend->operator_id = 0; /* caller sets this */
    trend->num_sessions = count;
    trend->slope_overall = slope;
    trend->slope_response_time = slope;
    trend->slope_stability = slope;
    trend->slope_constraints = slope;
    trend->slope_economic = slope;
    trend->r_squared = r_squared;
    trend->is_improving = (slope > MPC_OTS_EPS);
    trend->plateau_detected = (fabs(slope) < MPC_OTS_PLATEAU_THRESHOLD) ? count : 0;
    trend->predicted_plateau_score = intercept + slope * (double)count;

    return MPC_OTS_OK;
}

/* =========================================================================
 * Utility String Conversions
 * ========================================================================= */

const char *ots_state_to_string(OTSState state)
{
    switch (state) {
    case MPC_OTS_STATE_INIT:       return "INIT";
    case MPC_OTS_STATE_READY:      return "READY";
    case MPC_OTS_STATE_RUNNING:    return "RUNNING";
    case MPC_OTS_STATE_PAUSED:     return "PAUSED";
    case MPC_OTS_STATE_COMPLETED:  return "COMPLETED";
    case MPC_OTS_STATE_FAILED:     return "FAILED";
    case MPC_OTS_STATE_DEBRIEFING: return "DEBRIEFING";
    default:                       return "UNKNOWN";
    }
}

const char *ots_scenario_type_to_string(TrainingScenarioType type)
{
    switch (type) {
    case MPC_SCENARIO_DISTURBANCE_REJECTION: return "Disturbance Rejection";
    case MPC_SCENARIO_SETPOINT_CHANGE:       return "Setpoint Change";
    case MPC_SCENARIO_CONSTRAINT_VIOLATION:  return "Constraint Violation";
    case MPC_SCENARIO_OPTIMIZATION_TRADEOFF: return "Optimization Trade-off";
    case MPC_SCENARIO_EMERGENCY_SHUTDOWN:    return "Emergency Shutdown";
    case MPC_SCENARIO_GRADE_TRANSITION:      return "Grade Transition";
    default:                                  return "UNKNOWN";
    }
}

const char *ots_difficulty_to_string(ScenarioDifficulty difficulty)
{
    switch (difficulty) {
    case MPC_DIFFICULTY_BEGINNER:     return "Beginner";
    case MPC_DIFFICULTY_INTERMEDIATE: return "Intermediate";
    case MPC_DIFFICULTY_ADVANCED:     return "Advanced";
    case MPC_DIFFICULTY_EXPERT:       return "Expert";
    default:                           return "UNKNOWN";
    }
}

const char *ots_interface_mode_to_string(InterfaceMode mode)
{
    switch (mode) {
    case MPC_IF_MODE_MONITOR: return "Monitor";
    case MPC_IF_MODE_GUIDE:   return "Guide";
    case MPC_IF_MODE_ASSIST:  return "Assist";
    case MPC_IF_MODE_AUTO:    return "Auto";
    default:                   return "UNKNOWN";
    }
}

const char *ots_metric_to_string(PerformanceMetric metric)
{
    switch (metric) {
    case MPC_METRIC_RESPONSE_TIME:         return "Response Time";
    case MPC_METRIC_STABILITY_MARGIN:      return "Stability Margin";
    case MPC_METRIC_CONSTRAINT_COMPLIANCE: return "Constraint Compliance";
    case MPC_METRIC_ECONOMIC_OPTIMALITY:   return "Economic Optimality";
    case MPC_METRIC_ALARM_MANAGEMENT:      return "Alarm Management";
    case MPC_METRIC_SITUATION_AWARENESS:   return "Situation Awareness";
    case MPC_METRIC_CONSISTENCY:           return "Consistency";
    default:                                return "UNKNOWN";
    }
}
