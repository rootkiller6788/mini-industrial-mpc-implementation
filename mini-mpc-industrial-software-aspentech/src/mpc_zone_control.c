/** @file mpc_zone_control.c
 * @brief Zone Control for Industrial MPC (L5, L6, L7)
 *
 * In industrial MPC, many CVs do not require tight setpoint control.
 * Zone control allows CVs to float within a target range, activating
 * MV movements only when the CV approaches the zone boundary.
 *
 * Zone Control Cost Function (Maciejowski 2002, Ch.6):
 *   J_zone(y) = 0                          for y_lo <= y <= y_hi
 *   J_zone(y) = q_lo * (y - y_lo)^2       for y < y_lo
 *   J_zone(y) = q_hi * (y - y_hi)^2       for y > y_hi
 *
 * This is a piecewise-quadratic penalty that creates a "dead zone"
 * where no control action is taken. It is fundamentally different
 * from setpoint tracking where J = q * (y - y_ref)^2.
 *
 * Funnel Constraints (Qin & Badgwell 2003):
 *   Zone boundaries can widen over the prediction horizon:
 *     y_lo(k+i|k) = y_lo_ss - funnel_slope * i  (wider at far future)
 *     y_hi(k+i|k) = y_hi_ss + funnel_slope * i
 *
 * Ref:
 *   Maciejowski "Predictive Control with Constraints" (2002) Ch.6
 *   Qin & Badgwell "A survey of industrial MPC technology" (2003)
 *   AspenTech DMC3 Zone Control Configuration
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "mpc_common.h"

/* ===== L5: Zone Control Penalty Computation =====
 *
 * For a CV with zone [lo, hi], compute the penalty contribution
 * to the QP cost function gradient vector c.
 *
 * If y_pred is inside the zone: gradient contribution = 0
 * If y_pred is below lo:      gradient contribution = 2*q_lo*(y_pred - lo)
 * If y_pred is above hi:      gradient contribution = 2*q_hi*(y_pred - hi)
 *
 * Key insight: Zone control reduces unnecessary MV movement
 * compared to setpoint control, improving actuator life and
 * reducing energy consumption.
 */

int mpc_zone_control_gradient(const double *y_pred, int n_pred,
    double y_lo, double y_hi, double q_lo, double q_hi,
    double *gradient_out, double *cost_out)
{
    if (!y_pred || !gradient_out || n_pred < 1) return -1;
    if (q_lo < 0.0) q_lo = 0.0;
    if (q_hi < 0.0) q_hi = 0.0;

    double total_cost = 0.0;

    for (int k = 0; k < n_pred; k++) {
        double y = y_pred[k];

        if (y < y_lo) {
            double error = y - y_lo;
            gradient_out[k] = 2.0 * q_lo * error;
            total_cost += q_lo * error * error;
        } else if (y > y_hi) {
            double error = y - y_hi;
            gradient_out[k] = 2.0 * q_hi * error;
            total_cost += q_hi * error * error;
        } else {
            gradient_out[k] = 0.0;
        }
    }

    if (cost_out) *cost_out = total_cost;
    return 0;
}

/* ===== L5: Funnel Constraint Generation =====
 *
 * Funnel constraints widen the zone over the prediction horizon:
 *   y_lo(k+i) = y_lo_ss - funnel_slope * i
 *   y_hi(k+i) = y_hi_ss + funnel_slope * i
 *
 * This prevents the controller from making aggressive moves
 * for violations predicted far in the future, reflecting
 * the increasing uncertainty of long-range predictions.
 */

int mpc_generate_funnel_constraints(double y_lo_ss, double y_hi_ss,
    double funnel_slope, int n_pred,
    double *y_lo_profile, double *y_hi_profile)
{
    if (!y_lo_profile || !y_hi_profile || n_pred < 1) return -1;
    if (funnel_slope < 0.0) funnel_slope = 0.0;

    for (int i = 0; i < n_pred; i++) {
        y_lo_profile[i] = y_lo_ss - funnel_slope * (double)i;
        y_hi_profile[i] = y_hi_ss + funnel_slope * (double)i;
    }
    return 0;
}

/* ===== L6: Zone Control with Setpoint Preference =====
 *
 * In AspenTech DMC3, each CV can be configured as:
 *   - Setpoint control: J = q*(y - y_sp)^2
 *   - Zone control: J = 0 inside [lo, hi], quadratic outside
 *   - Ideal Resting Value (IRV): J = q*(y - y_irv)^2 but only when
 *     other controlled CVs are satisfied. IRV is a "preferred" value
 *     within the zone.
 *
 * This function computes the CV mode-specific tracking error
 * for inclusion in the QP gradient.
 */

int mpc_cv_tracking_error(const double *y_pred, int n_pred,
    double y_setpoint, double y_zone_lo, double y_zone_hi,
    double y_irv, int zone_enabled, int irv_enabled,
    double *error_profile)
{
    if (!y_pred || !error_profile || n_pred < 1) return -1;

    for (int i = 0; i < n_pred; i++) {
        double y = y_pred[i];

        if (zone_enabled) {
            /* Inside zone: zero error (unless IRV) */
            if (y >= y_zone_lo && y <= y_zone_hi) {
                if (irv_enabled) {
                    /* IRV: penalize deviation from ideal resting value */
                    error_profile[i] = y - y_irv;
                } else {
                    error_profile[i] = 0.0;
                }
            } else if (y < y_zone_lo) {
                error_profile[i] = y - y_zone_lo;
            } else {
                error_profile[i] = y - y_zone_hi;
            }
        } else {
            /* Standard setpoint tracking */
            error_profile[i] = y - y_setpoint;
        }
    }
    return 0;
}

/* ===== L7: CV Priority Ranking for DMC3 =====
 *
 * AspenTech DMC3 supports CV priority ranking:
 *   Critical CVs: Must stay within limits at all costs (hard constraints)
 *   Primary CVs: Track setpoint/zone with high weight
 *   Secondary CVs: Track setpoint/zone with low weight
 *   Auxiliary CVs: Information-only, not used in control
 *
 * Priority-based constraint softening:
 *   Slack variables are penalized with priority-dependent weights.
 *   Higher priority = higher slack penalty, so low-priority
 *   constraints are relaxed first when infeasible.
 */

int mpc_assign_cv_priority(double *slack_penalties, int n_cv,
    const int *cv_priorities, double base_penalty)
{
    if (!slack_penalties || !cv_priorities || n_cv < 1) return -1;
    if (base_penalty <= 0.0) base_penalty = 100.0;

    /* Priority 0 = critical, 1 = primary, 2 = secondary, 3 = auxiliary */
    for (int i = 0; i < n_cv; i++) {
        int prio = cv_priorities[i];
        if (prio < 0) prio = 0;
        /* Exponential decay: higher priority number = lower penalty */
        slack_penalties[i] = base_penalty * exp(-0.5 * (double)prio);
    }
    return 0;
}

/* ===== L6: Constraint Softening with L1 and L2 Penalties =====
 *
 * When QP is infeasible, soften constraints by adding slack variables s:
 *
 *   min  J(x) + rho_L1 * ||s||_1 + rho_L2 * ||s||_2^2
 *   s.t. lb - s <= x <= ub + s,  s >= 0
 *
 * L1 penalty (||s||_1): exact penalty, prefers sparse constraint
 *   violations (only violates the minimum number of constraints).
 *
 * L2 penalty (||s||_2^2): smooth penalty, distributes violations
 *   evenly across constraints.
 *
 * Ref: Nocedal & Wright (2006) Ch.17, penalty methods.
 */

int mpc_soften_constraints(double *lb, double *ub, int n,
    const double *slack_penalties, double *slack_vars_out,
    double *cost_penalty_out, int use_l1)
{
    if (!lb || !ub || !slack_penalties || n < 1) return -1;

    double total_penalty = 0.0;
    for (int i = 0; i < n; i++) {
        slack_vars_out[i] = 0.0;

        /* Check if lower bound is active and needs softening */
        if (lb[i] > -MPC_INF * 0.5) {
            slack_vars_out[i] = 0.0;  /* Initial slack is zero */
        }

        if (slack_penalties[i] > MPC_EPS) {
            if (use_l1) {
                total_penalty += slack_penalties[i] * slack_vars_out[i];
            } else {
                total_penalty += slack_penalties[i] * slack_vars_out[i] * slack_vars_out[i];
            }
        }
    }

    if (cost_penalty_out) *cost_penalty_out = total_penalty;
    return 0;
}

/* ===== L6: Zone Crossing Detection and Count =====
 *
 * Monitors how often each CV crosses its zone boundaries.
 * High crossing frequency indicates:
 *   - Zone is too narrow (need wider zone)
 *   - Model mismatch (predicted vs actual divergence)
 *   - Excessive disturbance (need better DV feedforward)
 */

int mpc_count_zone_crossings(const double *cv_history, int n_cv,
    int n_steps, const double *zone_lo, const double *zone_hi,
    int *crossing_counts)
{
    if (!cv_history || !zone_lo || !zone_hi || !crossing_counts || n_cv < 1 || n_steps < 2)
        return -1;

    for (int cv = 0; cv < n_cv; cv++) crossing_counts[cv] = 0;

    for (int cv = 0; cv < n_cv; cv++) {
        int prev_state = 0;  /* -1=below, 0=inside, 1=above */
        double y0 = cv_history[cv];
        if (y0 < zone_lo[cv]) prev_state = -1;
        else if (y0 > zone_hi[cv]) prev_state = 1;

        for (int step = 1; step < n_steps; step++) {
            double y = cv_history[step * n_cv + cv];
            int curr_state = 0;
            if (y < zone_lo[cv]) curr_state = -1;
            else if (y > zone_hi[cv]) curr_state = 1;

            if (curr_state != 0 && prev_state == 0) crossing_counts[cv]++;
            if (curr_state == 0 && prev_state != 0) crossing_counts[cv]++;
            prev_state = curr_state;
        }
    }
    return 0;
}
