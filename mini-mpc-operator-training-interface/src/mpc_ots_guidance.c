/**
 * @file mpc_ots_guidance.c
 * @brief Operator guidance and decision support system for MPC OTS.
 *
 * Implements context-sensitive guidance generation, priority-based
 * message queuing, workload estimation (NASA-TLX adapted),
 * what-if prediction, and operator decision logging.
 *
 * Knowledge points:
 *   L1: GuidanceEntry, GuidanceQueue, WorkloadEstimate, DecisionContext, OperatorDecision
 *   L2: Progressive guidance escalation (Hint→Advice→Warning→Intervention)
 *   L3: Priority queue with expiry, what-if prediction pipeline
 *   L4: NASA-TLX workload model, ISO 9241-210 human-centered design
 *   L5: Context-weighted guidance selection, workload estimation, what-if linear prediction
 *   L6: Operator hesitation detection, decision accuracy assessment
 *   L7: Honeywell Alarm Guidance, AspenTech Decision Support
 *   L8: Trust-calibrated automation (Lee & See 2004)
 *
 * Reference:
 *   Bainbridge (1983), "Ironies of automation", Automatica 19(6)
 *   Endsley (1995), "Toward a theory of situation awareness in dynamic systems"
 *   Hart & Staveland (1988), "NASA-TLX development"
 *   Lee & See (2004), "Trust in automation", Human Factors 46(1)
 *   Parasuraman & Riley (1997), "Humans and automation", Human Factors 39(2)
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include "../include/mpc_ots_guidance.h"

/* =========================================================================
 * L5: Guidance Queue Management
 * ========================================================================= */

OTSStatus ots_guidance_queue_init(GuidanceQueue *queue)
{
    if (!queue) return MPC_OTS_ERR_NULL_POINTER;

    memset(queue, 0, sizeof(GuidanceQueue));
    queue->count = 0;
    queue->total_delivered = 0;
    queue->total_acted = 0;
    queue->total_expired = 0;
    queue->last_delivery_time = 0.0;
    queue->cooldown_remaining = 0.0;
    queue->current_displayed = -1;
    queue->is_suppressed = false;

    return MPC_OTS_OK;
}

OTSStatus ots_guidance_enqueue(GuidanceQueue *queue, const GuidanceEntry *entry)
{
    if (!queue || !entry) return MPC_OTS_ERR_NULL_POINTER;

    /* Handle full queue: displace lowest-priority message */
    if (queue->count >= MPC_OTS_GUIDANCE_MAX_QUEUE) {
        /* Find lowest priority (highest number) */
        int32_t lowest_idx = 0;
        int32_t lowest_priority = queue->entries[0].priority;

        for (int32_t i = 1; i < queue->count; i++) {
            if (queue->entries[i].priority > lowest_priority) {
                lowest_priority = queue->entries[i].priority;
                lowest_idx = i;
            }
        }

        /* If new entry has higher priority, replace lowest */
        if (entry->priority < lowest_priority) {
            queue->entries[lowest_idx] = *entry;
            return MPC_OTS_OK;
        } else {
            return MPC_OTS_ERR_GUIDANCE_QUEUE_FULL;
        }
    }

    /* Insert in priority order */
    int32_t insert_pos = queue->count;
    for (int32_t i = 0; i < queue->count; i++) {
        if (entry->priority < queue->entries[i].priority) {
            insert_pos = i;
            break;
        }
    }

    /* Shift higher priority messages down */
    for (int32_t i = queue->count; i > insert_pos; i--) {
        queue->entries[i] = queue->entries[i - 1];
    }

    queue->entries[insert_pos] = *entry;
    queue->count++;

    return MPC_OTS_OK;
}

OTSStatus ots_guidance_dequeue_next(GuidanceQueue *queue, GuidanceEntry *entry)
{
    if (!queue || !entry) return MPC_OTS_ERR_NULL_POINTER;
    if (queue->count <= 0) return MPC_OTS_ERR_EVENT_NOT_FOUND;
    if (queue->is_suppressed) return MPC_OTS_ERR_INVALID_STATE;

    /* Check cooldown: don't overwhelm operator */
    if (queue->cooldown_remaining > MPC_OTS_EPS) {
        return MPC_OTS_ERR_TIMEOUT; /* not ready yet */
    }

    /* Find first non-expired, not-yet-displayed message */
    for (int32_t i = 0; i < queue->count; i++) {
        GuidanceEntry *candidate = &queue->entries[i];
        if (!candidate->was_displayed && !candidate->is_expired) {
            *entry = *candidate;
            candidate->was_displayed = true;

            queue->total_delivered++;
            queue->current_displayed = candidate->message_id;
            queue->last_delivery_time = entry->generation_time;
            queue->cooldown_remaining = MPC_OTS_GUIDANCE_COOLDOWN;

            return MPC_OTS_OK;
        }
    }

    return MPC_OTS_ERR_EVENT_NOT_FOUND; /* no undisplayed messages */
}

OTSStatus ots_guidance_mark_acted(GuidanceQueue *queue, int32_t message_id)
{
    if (!queue) return MPC_OTS_ERR_NULL_POINTER;

    for (int32_t i = 0; i < queue->count; i++) {
        if (queue->entries[i].message_id == message_id) {
            queue->entries[i].was_acted_upon = true;
            queue->total_acted++;
            return MPC_OTS_OK;
        }
    }
    return MPC_OTS_ERR_EVENT_NOT_FOUND;
}

OTSStatus ots_guidance_expire_stale(GuidanceQueue *queue, double current_time)
{
    if (!queue) return MPC_OTS_ERR_NULL_POINTER;

    int32_t expired = 0;
    for (int32_t i = 0; i < queue->count; i++) {
        GuidanceEntry *entry = &queue->entries[i];
        if (!entry->was_displayed && !entry->is_expired
            && entry->expiry_time > MPC_OTS_EPS
            && current_time > entry->expiry_time) {
            entry->is_expired = true;
            expired++;
        }
    }

    queue->total_expired += expired;

    /* Update cooldown */
    if (queue->cooldown_remaining > MPC_OTS_EPS) {
        /* cooldown decreases with time (caller provides updates) */
    }

    return MPC_OTS_OK;
}

/* =========================================================================
 * L5: Guidance Generation
 * ========================================================================= */

OTSStatus ots_guidance_generate(const DecisionContext *ctx, const WorkloadEstimate *wl, GuidanceEntry *entry)
{
    if (!ctx || !entry) return MPC_OTS_ERR_NULL_POINTER;
    (void)wl;

    memset(entry, 0, sizeof(GuidanceEntry));
    entry->priority = 5; /* medium default */

    /* Determine guidance based on most critical deviation */
    double max_margin_violation = 0.0;
    int32_t worst_var = -1;

    for (int32_t i = 0; i < ctx->num_variables; i++) {
        if (ctx->constraint_margins[i] < max_margin_violation) {
            max_margin_violation = ctx->constraint_margins[i];
            worst_var = i;
        }
    }

    /* Set guidance level based on violation severity */
    if (max_margin_violation < -10.0) {
        entry->level = MPC_GUIDANCE_INTERVENTION;
        entry->trigger = MPC_TRIGGER_SAFETY_CRITICAL;
        entry->priority = 0;
        snprintf(entry->title, MPC_OTS_NAME_MAX, "CRITICAL: Safety intervention required");
        snprintf(entry->message, MPC_OTS_GUIDANCE_TEXT_MAX,
            "Variable %d has critically violated constraints. Immediate action required.", worst_var);
        snprintf(entry->action, MPC_OTS_GUIDANCE_TEXT_MAX,
            "Reduce MV to bring CV back within safe operating limits.");
    } else if (max_margin_violation < 0.0) {
        entry->level = MPC_GUIDANCE_WARNING;
        entry->trigger = MPC_TRIGGER_CONSTRAINT_VIOLATED;
        entry->priority = 1;
        snprintf(entry->title, MPC_OTS_NAME_MAX, "WARNING: Constraint violation detected");
        snprintf(entry->message, MPC_OTS_GUIDANCE_TEXT_MAX,
            "Variable %d has exceeded operating limits. Adjust MV promptly.", worst_var);
        snprintf(entry->action, MPC_OTS_GUIDANCE_TEXT_MAX,
            "Review related MV and reduce to restore CV within limits.");
    } else if (max_margin_violation < 5.0) {
        entry->level = MPC_GUIDANCE_ADVICE;
        entry->trigger = MPC_TRIGGER_CONSTRAINT_APPROACH;
        entry->priority = 3;
        snprintf(entry->title, MPC_OTS_NAME_MAX, "Attention: Approaching constraint boundary");
        snprintf(entry->message, MPC_OTS_GUIDANCE_TEXT_MAX,
            "Variable %d is within 5 units of constraint. Consider preemptive action.", worst_var);
        snprintf(entry->action, MPC_OTS_GUIDANCE_TEXT_MAX,
            "Monitor trend and prepare to adjust if deviation continues.");
    } else {
        entry->level = MPC_GUIDANCE_HINT;
        entry->trigger = MPC_TRIGGER_OPTIMAL_DEVIATION;
        entry->priority = 5;
        snprintf(entry->title, MPC_OTS_NAME_MAX, "Optimization opportunity");
        snprintf(entry->message, MPC_OTS_GUIDANCE_TEXT_MAX,
            "Process is stable. Consider moving toward economic optimum.");
        snprintf(entry->action, MPC_OTS_GUIDANCE_TEXT_MAX,
            "Review economic targets and adjust MV to maximize profit.");
    }

    /* Compute delay based on operator experience */
    entry->delay_before_display = ots_guidance_compute_delay(
        ctx->operator_level, ctx->difficulty, MPC_OTS_GUIDANCE_DELAY_BASE);

    entry->relevance_score = ots_guidance_relevance(ctx, entry);

    return MPC_OTS_OK;
}

double ots_guidance_relevance(const DecisionContext *ctx, const GuidanceEntry *entry)
{
    if (!ctx || !entry) return 0.0;
    (void)entry;

    /* Relevance = 1 - min_margin_normalized
     * If all margins are large, relevance is low.
     * If any margin is small/negative, relevance is high.
     */
    double min_margin = MPC_OTS_LARGE;
    for (int32_t i = 0; i < ctx->num_variables; i++) {
        if (ctx->constraint_margins[i] < min_margin) {
            min_margin = ctx->constraint_margins[i];
        }
    }

    /* Normalize margin: positive margin → low relevance */
    if (min_margin < 0.0) return 1.0; /* violation → maximum relevance */
    double relevance = 1.0 / (1.0 + min_margin / 5.0); /* decay with margin */
    if (relevance < 0.0) relevance = 0.0;
    if (relevance > 1.0) relevance = 1.0;

    return relevance;
}

double ots_guidance_compute_delay(OperatorRole role, ScenarioDifficulty difficulty, double base_delay)
{
    /* More experienced operators get longer delays to encourage
     * independent problem-solving.
     *
     * Experience multiplier (based on role):
     *   TRAINEE: 0.5  (help arrives faster)
     *   JUNIOR:  0.7
     *   SENIOR:  1.0
     *   SHIFT_LEAD: 1.3
     *   TRAINER: 1.5  (longest delay, most independence)
     *
     * Difficulty multiplier:
     *   BEGINNER:     0.6 (lower difficulty = less delay penalty)
     *   INTERMEDIATE: 0.8
     *   ADVANCED:     1.0
     *   EXPERT:       1.2 (expert difficulty, but also expert operators)
     */
    double role_mult;
    switch (role) {
    case MPC_ROLE_TRAINEE:         role_mult = 0.5; break;
    case MPC_ROLE_JUNIOR_OPERATOR: role_mult = 0.7; break;
    case MPC_ROLE_SENIOR_OPERATOR: role_mult = 1.0; break;
    case MPC_ROLE_SHIFT_LEAD:      role_mult = 1.3; break;
    case MPC_ROLE_TRAINER:         role_mult = 1.5; break;
    default:                       role_mult = 1.0; break;
    }

    double diff_mult;
    switch (difficulty) {
    case MPC_DIFFICULTY_BEGINNER:     diff_mult = 0.6; break;
    case MPC_DIFFICULTY_INTERMEDIATE: diff_mult = 0.8; break;
    case MPC_DIFFICULTY_ADVANCED:     diff_mult = 1.0; break;
    case MPC_DIFFICULTY_EXPERT:       diff_mult = 1.2; break;
    default:                          diff_mult = 1.0; break;
    }

    double delay = base_delay * role_mult * diff_mult;
    if (delay < MPC_OTS_GUIDANCE_DELAY_MIN) delay = MPC_OTS_GUIDANCE_DELAY_MIN;
    if (delay > MPC_OTS_GUIDANCE_DELAY_MAX) delay = MPC_OTS_GUIDANCE_DELAY_MAX;

    return delay;
}

/* =========================================================================
 * L5: Workload Estimation (NASA-TLX adapted)
 * ========================================================================= */

OTSStatus ots_workload_estimate(const DecisionContext *ctx, WorkloadEstimate *wl)
{
    if (!ctx || !wl) return MPC_OTS_ERR_NULL_POINTER;

    memset(wl, 0, sizeof(WorkloadEstimate));

    /* Mental demand: based on active events and difficulty */
    wl->mental_demand = 0.15 * (double)ctx->active_event_count
                       + 0.10 * (double)ctx->difficulty
                       + 0.05 * (double)ctx->num_variables;
    if (wl->mental_demand > 1.0) wl->mental_demand = 1.0;

    /* Physical demand: minimal for panel operator (keyboard/mouse) */
    wl->physical_demand = 0.05 + 0.02 * (double)ctx->active_event_count;

    /* Temporal demand: based on constraint margins (pressure) */
    double min_margin = MPC_OTS_LARGE;
    for (int32_t i = 0; i < ctx->num_variables; i++) {
        if (ctx->constraint_margins[i] < min_margin) {
            min_margin = ctx->constraint_margins[i];
        }
    }
    if (min_margin < 0.0) {
        wl->temporal_demand = fmin(1.0, -min_margin / 20.0 + 0.3);
    } else {
        wl->temporal_demand = fmax(0.1, 0.3 - min_margin / 50.0);
    }

    /* Performance: inverse of current score */
    wl->performance_self = 1.0 - (ctx->current_score / 100.0);

    /* Effort: composite of mental and temporal */
    wl->effort = 0.6 * wl->mental_demand + 0.4 * wl->temporal_demand;

    /* Frustration: increases with difficulty and poor score */
    wl->frustration = 0.3 * ((double)ctx->difficulty / 3.0)
                     + 0.3 * wl->performance_self
                     + 0.1 * (double)ctx->active_event_count / 10.0;
    if (wl->frustration > 1.0) wl->frustration = 1.0;

    /* Overall weighted sum (NASA-TLX weights):
     * The original NASA-TLX uses pairwise comparisons; here we use
     * standardized weights from Hart & Staveland (1988) meta-analysis. */
    wl->overall_workload = 0.25 * wl->mental_demand
                         + 0.05 * wl->physical_demand
                         + 0.25 * wl->temporal_demand
                         + 0.10 * wl->performance_self
                         + 0.15 * wl->effort
                         + 0.20 * wl->frustration;

    wl->active_alarms = ctx->active_event_count;
    wl->is_overloaded = ots_workload_is_critical(wl);

    return MPC_OTS_OK;
}

bool ots_workload_is_critical(const WorkloadEstimate *wl)
{
    if (!wl) return false;
    return wl->overall_workload > MPC_OTS_WORKLOAD_CRITICAL;
}

/* =========================================================================
 * L5: What-if Decision Support
 * ========================================================================= */

OTSStatus ots_whatif_predict(const DecisionContext *ctx, const double proposed_mv[], int32_t num_mv, double predicted_cv[], double uncertainty[])
{
    if (!ctx || !proposed_mv || !predicted_cv) return MPC_OTS_ERR_NULL_POINTER;
    if (num_mv <= 0 || num_mv > ctx->num_variables) return MPC_OTS_ERR_INVALID_PARAM;

    /* Simplified steady-state gain model for training OTS.
     *
     * DeltaCV = G * DeltaMV where G is the process gain matrix.
     *
     * For training purposes, we use a simplified model:
     * - Diagonal elements: G_ii = 1.0 (direct effect)
     * - Off-diagonal elements: G_ij = 0.25 for |i-j|=1 (adjacent coupling)
     * - G_ij = 0.05 for |i-j|>1 (weak coupling)
     *
     * Uncertainty increases with coupling complexity:
     *   u_i = 0.05 + 0.02 * (number of non-zero off-diagonal contributions)
     *
     * Reference: Seborg et al. (2016), "Process Dynamics and Control", Ch.16.
     */
    for (int32_t i = 0; i < ctx->num_variables && i < num_mv; i++) {
        double delta = proposed_mv[i] - ctx->current_values[i];
        predicted_cv[i] = delta * 1.0; /* self-gain */

        /* Coupling from adjacent variables */
        if (i > 0 && i - 1 < num_mv) {
            double adj_delta = proposed_mv[i - 1] - ctx->current_values[i - 1];
            predicted_cv[i] += adj_delta * 0.25;
        }
        if (i + 1 < ctx->num_variables && i + 1 < num_mv) {
            double adj_delta = proposed_mv[i + 1] - ctx->current_values[i + 1];
            predicted_cv[i] += adj_delta * 0.25;
        }

        /* Weak coupling from non-adjacent */
        for (int32_t j = 0; j < num_mv; j++) {
            if (j == i || j == i - 1 || j == i + 1) continue;
            double weak_delta = proposed_mv[j] - ctx->current_values[j];
            predicted_cv[i] += weak_delta * 0.05;
        }

        if (uncertainty) {
            uncertainty[i] = 0.05 + 0.02 * (double)(num_mv - 1);
            if (uncertainty[i] > 0.50) uncertainty[i] = 0.50;
        }
    }

    return MPC_OTS_OK;
}

double ots_whatif_economic_impact(const double predicted_cv[], const double economic_coefficients[], int32_t num_cv)
{
    if (!predicted_cv || !economic_coefficients || num_cv <= 0) return 0.0;

    double impact = 0.0;
    for (int32_t i = 0; i < num_cv; i++) {
        impact += fabs(predicted_cv[i]) * economic_coefficients[i];
    }
    return impact;
}

/* =========================================================================
 * L5: Decision Logging
 * ========================================================================= */

OTSStatus ots_decision_log(OperatorDecision *log, int32_t id, int32_t event_id, const char *action, double magnitude, double predicted, bool followed_guidance)
{
    if (!log || !action) return MPC_OTS_ERR_NULL_POINTER;

    memset(log, 0, sizeof(OperatorDecision));
    log->decision_id = id;
    log->event_id = event_id;
    snprintf(log->action_description, MPC_OTS_DESCRIPTION_MAX, "%s", action);
    log->action_magnitude = magnitude;
    log->predicted_outcome = predicted;
    log->actual_outcome = 0.0; /* unknown at decision time */
    log->prediction_error = 0.0;
    log->was_correct = false; /* unknown at decision time */
    log->followed_guidance = followed_guidance;
    log->confidence = 0.7; /* base confidence */
    log->time_to_decide = 0.0; /* set by caller */

    return MPC_OTS_OK;
}

double ots_compute_hesitation(double event_trigger_time, double first_action_time)
{
    if (first_action_time < event_trigger_time) return 0.0;
    return first_action_time - event_trigger_time;
}

OTSStatus ots_decision_summary(const OperatorDecision decisions[], int32_t count, double *guidance_follow_rate, double *mean_hesitation, double *decision_accuracy)
{
    if (!decisions || !guidance_follow_rate || !mean_hesitation || !decision_accuracy) {
        return MPC_OTS_ERR_NULL_POINTER;
    }
    if (count <= 0) return MPC_OTS_ERR_INVALID_PARAM;

    int32_t followed = 0;
    int32_t correct = 0;
    double total_hesitation = 0.0;

    for (int32_t i = 0; i < count; i++) {
        if (decisions[i].followed_guidance) followed++;
        if (decisions[i].was_correct) correct++;
        total_hesitation += decisions[i].time_to_decide;
    }

    *guidance_follow_rate = (double)followed / (double)count;
    *mean_hesitation = total_hesitation / (double)count;
    *decision_accuracy = (double)correct / (double)count;

    return MPC_OTS_OK;
}
