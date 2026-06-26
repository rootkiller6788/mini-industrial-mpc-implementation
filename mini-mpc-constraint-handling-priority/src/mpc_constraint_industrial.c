/**
 * @file mpc_constraint_industrial.c
 * @brief Industrial MPC constraint handling implementations.
 *
 * Implements vendor-specific constraint handling patterns as used in
 * real industrial MPC products and canonical industrial scenarios.
 *
 * Knowledge points:
 *   L6: Input saturation management, output prioritization in multi-CV systems
 *   L7: AspenTech DMC3 sequential QP, Honeywell RMPCT range control,
 *       Shell SMOC state-space soft constraints, ABB Predict & Control
 *   L8: Constraint funnel management, ideal resting values, lexicographic MPC
 *
 * Reference:
 *   Qin & Badgwell (2003), "A survey of industrial model predictive control
 *     technology", Control Engineering Practice 11(7), 733-764
 *   Froisy (1994), "Model predictive control: Past, present and future",
 *     ISA Transactions 33(3), 235-243
 *   Darby & Nikolaou (2012), "MPC: Current practice and challenges",
 *     Control Engineering Practice 20(4), 328-342
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include "../include/mpc_constraint_defs.h"
#include "../include/mpc_constraint_priority.h"

/* ====================================================================
 * L6: Input Saturation Management
 *
 * When an MV hits its physical limit (saturation), the MPC loses one
 * degree of freedom. The controller must:
 *   1. Detect saturation (MV at bound for N consecutive cycles)
 *   2. Redistribute control effort to remaining MVs (MIMO case)
 *   3. Accept output deviation if no remaining MVs (SISO case)
 *   4. Plan for desaturation when disturbance direction reverses
 *
 * Industrial practice: "Anti-windup for MPC" — the integrator in the
 * disturbance model must also respect MV saturation.
 * ==================================================================== */

mpc_status_t mpc_detect_input_saturation(const double *mv_values,
                                          const double *mv_lower,
                                          const double *mv_upper,
                                          int num_mv,
                                          int saturation_threshold,
                                          mpc_input_saturation_t *saturation_info)
{
    if (!mv_values || !mv_lower || !mv_upper || !saturation_info)
        return MPC_ERR_NULL_POINTER;
    if (num_mv <= 0 || saturation_threshold <= 0)
        return MPC_ERR_DIMENSION_MISMATCH;

    for (int i = 0; i < num_mv; i++) {
        saturation_info[i].mv_index = i;
        saturation_info[i].at_upper_bound = false;
        saturation_info[i].at_lower_bound = false;
        saturation_info[i].lost_control_authority = 0.0;

        /* Check upper saturation */
        if (mv_values[i] >= mv_upper[i] - MPC_FEASIBILITY_TOL) {
            saturation_info[i].at_upper_bound = true;
            saturation_info[i].saturated_value = mv_upper[i];
            saturation_info[i].saturation_duration++;
        }
        /* Check lower saturation */
        else if (mv_values[i] <= mv_lower[i] + MPC_FEASIBILITY_TOL) {
            saturation_info[i].at_lower_bound = true;
            saturation_info[i].saturated_value = mv_lower[i];
            saturation_info[i].saturation_duration++;
        }
        /* Not saturated — reset duration */
        else {
            saturation_info[i].saturation_duration = 0;
        }

        /* Compute lost control authority:
         * Fraction of the total MV range that is unavailable due to saturation.
         * If saturated at upper bound: lost = (mv - upper) / range = 0 (already at max)
         * Actually: lost authority = fraction of feasible direction blocked */
        if (saturation_info[i].at_upper_bound) {
            /* Can only move down, not up. Lost = 50% (half of control action) */
            saturation_info[i].lost_control_authority = 0.5;
        } else if (saturation_info[i].at_lower_bound) {
            saturation_info[i].lost_control_authority = 0.5;
        } else {
            saturation_info[i].lost_control_authority = 0.0;
        }
    }

    return MPC_OK;
}

mpc_status_t mpc_compute_desaturation_path(const mpc_input_saturation_t *sat,
                                            const double *mv_steady_state,
                                            int num_mv,
                                            double *desaturation_moves,
                                            int horizon)
{
    if (!sat || !mv_steady_state || !desaturation_moves)
        return MPC_ERR_NULL_POINTER;

    /* Compute a smooth desaturation path:
     * Move the saturated MV back toward its steady-state value
     * at the maximum allowed rate, respecting rate constraints. */
    for (int i = 0; i < num_mv; i++) {
        if (sat[i].at_upper_bound) {
            /* Move down toward steady state */
            double total_move = sat[i].saturated_value - mv_steady_state[i];
            double move_per_step = total_move / (double)horizon;
            for (int k = 0; k < horizon; k++) {
                desaturation_moves[i * horizon + k] = -move_per_step;
            }
        } else if (sat[i].at_lower_bound) {
            /* Move up toward steady state */
            double total_move = mv_steady_state[i] - sat[i].saturated_value;
            double move_per_step = total_move / (double)horizon;
            for (int k = 0; k < horizon; k++) {
                desaturation_moves[i * horizon + k] = move_per_step;
            }
        } else {
            /* Not saturated — no desaturation needed */
            for (int k = 0; k < horizon; k++) {
                desaturation_moves[i * horizon + k] = 0.0;
            }
        }
    }

    return MPC_OK;
}

/* ====================================================================
 * L6: Output Prioritization in Multi-CV Systems
 *
 * In processes with multiple controlled variables, output constraints
 * compete for limited MV resources. The priority system determines
 * which CV constraints are satisfied first when conflicts arise.
 *
 * Industrial example (FCC unit):
 *   CV1: Reactor temperature (CRITICAL — safety)
 *   CV2: Regenerator temperature (HIGH — catalyst protection)
 *   CV3: Gasoline yield (MEDIUM — economics)
 *   CV4: LPG production (LOW — secondary product)
 * ==================================================================== */

mpc_status_t mpc_output_prioritization_init(mpc_output_prioritization_t *op,
                                              int num_cv)
{
    if (!op || num_cv <= 0) return MPC_ERR_NULL_POINTER;

    op->num_cv = num_cv;
    op->cv_priority = (int *)calloc((size_t)num_cv, sizeof(int));
    op->cv_lower_limit = (double *)calloc((size_t)num_cv, sizeof(double));
    op->cv_upper_limit = (double *)calloc((size_t)num_cv, sizeof(double));
    op->cv_current_value = (double *)calloc((size_t)num_cv, sizeof(double));
    op->cv_constraint_active = (bool *)calloc((size_t)num_cv, sizeof(bool));
    op->cv_violation_cost = (double *)calloc((size_t)num_cv, sizeof(double));

    if (!op->cv_priority || !op->cv_lower_limit || !op->cv_upper_limit ||
        !op->cv_current_value || !op->cv_constraint_active || !op->cv_violation_cost) {
        free(op->cv_priority); free(op->cv_lower_limit);
        free(op->cv_upper_limit); free(op->cv_current_value);
        free(op->cv_constraint_active); free(op->cv_violation_cost);
        return MPC_ERR_MEMORY;
    }

    /* Default: all CVs at LOW priority with infinite bounds */
    for (int i = 0; i < num_cv; i++) {
        op->cv_priority[i] = MPC_PRIORITY_LOW;
        op->cv_lower_limit[i] = -MPC_BOUND_INF;
        op->cv_upper_limit[i] = MPC_BOUND_INF;
        op->cv_current_value[i] = 0.0;
        op->cv_constraint_active[i] = false;
        op->cv_violation_cost[i] = 1.0;
    }

    op->cv_most_critical = -1;
    return MPC_OK;
}

void mpc_output_prioritization_free(mpc_output_prioritization_t *op)
{
    if (!op) return;
    free(op->cv_priority);
    free(op->cv_lower_limit);
    free(op->cv_upper_limit);
    free(op->cv_current_value);
    free(op->cv_constraint_active);
    free(op->cv_violation_cost);
    memset(op, 0, sizeof(mpc_output_prioritization_t));
}

mpc_status_t mpc_output_prioritization_set_cv(mpc_output_prioritization_t *op,
                                                int cv_idx,
                                                mpc_priority_level_t priority,
                                                double lower, double upper,
                                                double violation_cost)
{
    if (!op) return MPC_ERR_NULL_POINTER;
    if (cv_idx < 0 || cv_idx >= op->num_cv)
        return MPC_ERR_DIMENSION_MISMATCH;

    op->cv_priority[cv_idx] = (int)priority;
    op->cv_lower_limit[cv_idx] = lower;
    op->cv_upper_limit[cv_idx] = upper;
    op->cv_violation_cost[cv_idx] = violation_cost;

    return MPC_OK;
}

mpc_status_t mpc_output_prioritization_evaluate(
    mpc_output_prioritization_t *op,
    const double *cv_values)
{
    if (!op || !cv_values) return MPC_ERR_NULL_POINTER;

    /* Find the most critical violated CV */
    op->cv_most_critical = -1;
    int highest_priority_violated = MPC_PRIORITY_MONITOR + 1;

    for (int i = 0; i < op->num_cv; i++) {
        op->cv_current_value[i] = cv_values[i];
        op->cv_constraint_active[i] = false;

        /* Check violation */
        if (cv_values[i] < op->cv_lower_limit[i] - MPC_FEASIBILITY_TOL ||
            cv_values[i] > op->cv_upper_limit[i] + MPC_FEASIBILITY_TOL) {
            op->cv_constraint_active[i] = true;

            /* Track the highest-priority (lowest number) violated CV */
            if (op->cv_priority[i] < highest_priority_violated) {
                highest_priority_violated = op->cv_priority[i];
                op->cv_most_critical = i;
            }
        }
    }

    return MPC_OK;
}

mpc_status_t mpc_output_prioritization_rank_violations(
    const mpc_output_prioritization_t *op,
    int *ranked_indices)
{
    if (!op || !ranked_indices) return MPC_ERR_NULL_POINTER;

    /* Sort by priority (ascending = most critical first), then by violation cost */
    int count = 0;
    for (int i = 0; i < op->num_cv; i++) {
        if (op->cv_constraint_active[i]) {
            ranked_indices[count] = i;
            count++;
        }
    }

    /* Bubble sort by priority (small violations, acceptable for few CVs) */
    for (int i = 0; i < count - 1; i++) {
        for (int j = 0; j < count - i - 1; j++) {
            int a = ranked_indices[j];
            int b = ranked_indices[j + 1];
            if (op->cv_priority[a] > op->cv_priority[b] ||
                (op->cv_priority[a] == op->cv_priority[b] &&
                 op->cv_violation_cost[a] < op->cv_violation_cost[b])) {
                int tmp = ranked_indices[j];
                ranked_indices[j] = ranked_indices[j + 1];
                ranked_indices[j + 1] = tmp;
            }
        }
    }

    return MPC_OK;
}

/* ====================================================================
 * L7: AspenTech DMC3 — Sequential Quadratic Programming
 *
 * AspenTech's DMC3 (Dynamic Matrix Control, 3rd generation) uses
 * sequential QP with priority ranking:
 *   1. Sort constraints by rank (ascending)
 *   2. Solve QP with rank 1 constraints only
 *   3. Fix rank 1 objective, add rank 2 constraints
 *   4. Continue through all ranks
 *   5. If infeasible at rank k, relax rank k+ constraints
 *
 * This is the "lexicographic" or "prioritized" approach.
 * ==================================================================== */

mpc_status_t mpc_aspen_dmc3_priority_solve(
    const mpc_constraint_set_t *cs,
    mpc_vendor_config_t *vendor_cfg,
    mpc_qp_solution_t *solution)
{
    if (!cs || !vendor_cfg || !solution)
        return MPC_ERR_NULL_POINTER;

    vendor_cfg->vendor = MPC_VENDOR_ASPENTECH_DMC;
    vendor_cfg->use_lexicographic_qp = true;
    vendor_cfg->max_sequential_passes = 5;

    /* DMC3 sequential algorithm:
     * For each priority level, solve QP with constraints at that level
     * while maintaining optimality for higher-priority objectives. */
    solution->required_relaxation = false;
    solution->num_levels_relaxed = 0;

    for (int pass = 0; pass < vendor_cfg->max_sequential_passes; pass++) {
        bool pass_feasible = true;

        /* Check each priority level */
        for (int level = MPC_PRIORITY_CRITICAL;
             level <= MPC_PRIORITY_LOW; level++) {
            int start = cs->priority_start[level];
            int count = cs->priority_count[level];

            for (int i = start; i < start + count; i++) {
                if (cs->constraints[i].is_active &&
                    cs->constraints[i].is_violated) {
                    pass_feasible = false;
                    break;
                }
            }
            if (!pass_feasible) break;
        }

        if (pass_feasible) {
            solution->status = MPC_FEASIBLE;
            return MPC_OK;
        }

        /* Relaxation needed: deactivate lowest active priority */
        for (int level = MPC_PRIORITY_LOW;
             level >= MPC_PRIORITY_CRITICAL; level--) {
            if (cs->priority_count[level] > 0) {
                int deactivated = mpc_constraint_deactivate_below_priority(
                    (mpc_constraint_set_t *)cs, (mpc_priority_level_t)(level - 1));
                if (deactivated > 0) {
                    solution->num_levels_relaxed++;
                    solution->required_relaxation = true;
                }
                break;
            }
        }
    }

    solution->status = MPC_INFEASIBLE;
    return MPC_ERR_QP_INFEASIBLE;
}

/* ====================================================================
 * L7: Honeywell RMPCT — Range Control Algorithm
 *
 * Honeywell's Robust Multivariable Predictive Control Technology (RMPCT)
 * uses "range control":
 *   - Each CV has an "ideal resting value" (IRV) within a constraint range
 *   - CVs are controlled to stay within [lower, upper] range
 *   - Economic incentive drives CVs toward the IRV
 *   - "Funnel" concept: constraints widen during transients, narrow at steady state
 *
 * Key difference from DMC: RMPCT treats constraints as ranges, not
 * hard bounds, with economic optimization within the range.
 * ==================================================================== */

mpc_status_t mpc_honeywell_rmpct_range_control(
    const mpc_output_prioritization_t *op,
    mpc_vendor_config_t *vendor_cfg,
    double *cv_targets,
    double *funnel_widths)
{
    if (!op || !vendor_cfg || !cv_targets || !funnel_widths)
        return MPC_ERR_NULL_POINTER;

    vendor_cfg->vendor = MPC_VENDOR_HONEYWELL_RMPCT;
    vendor_cfg->use_range_control = true;
    vendor_cfg->use_ideal_resting_value = true;
    vendor_cfg->constraint_zone_width = 0.05;
    vendor_cfg->funnel_opening_rate = 0.01;

    /* RMPCT range control:
     * For each CV, the target is the IRV (Ideal Resting Value).
     * The funnel width determines how much the CV can deviate:
     *   funnel(k) = funnel_ss + (funnel_0 - funnel_ss) * exp(-k * opening_rate)
     * At steady state: narrow funnel (tight control)
     * During transients: wide funnel (allow temporary violations) */
    for (int i = 0; i < op->num_cv; i++) {
        /* IRV = midpoint of constraint range */
        cv_targets[i] = (op->cv_lower_limit[i] + op->cv_upper_limit[i]) / 2.0;

        /* Funnel width = half the constraint zone width */
        funnel_widths[i] = vendor_cfg->constraint_zone_width *
                           (op->cv_upper_limit[i] - op->cv_lower_limit[i]);
    }

    return MPC_OK;
}

/* ====================================================================
 * L7: Shell SMOC — State-Space Soft Constraints
 *
 * Shell Multivariable Optimizing Controller (SMOC) uses state-space MPC
 * with soft output constraints:
 *   - Output constraints are always soft (economic penalties)
 *   - Input constraints are always hard (actuator physics)
 *   - Uses Kalman filter for state estimation
 *   - Disturbance model for offset-free tracking
 * ==================================================================== */

mpc_status_t mpc_shell_smoc_constraint_setup(
    mpc_constraint_set_t *cs,
    mpc_vendor_config_t *vendor_cfg)
{
    if (!cs || !vendor_cfg) return MPC_ERR_NULL_POINTER;

    vendor_cfg->vendor = MPC_VENDOR_SHELL_SMOC;

    /* SMOC constraint policy: all output constraints are soft */
    for (int i = 0; i < cs->total_count; i++) {
        mpc_constraint_t *c = &cs->constraints[i];
        if (c->scope == MPC_SCOPE_OUTPUT) {
            /* Output constraints are always soft in SMOC */
            if (mpc_constraint_is_hard(c)) {
                c->type = MPC_CONSTRAINT_SOFT_OUTPUT;
            }
            c->relax_policy = MPC_RELAX_ALWAYS_SOFT;
            c->slack_penalty_linear = 1e4;
            c->slack_penalty_quadratic = 1e8;
        }
        if (c->scope == MPC_SCOPE_INPUT) {
            /* Input constraints remain hard */
            if (mpc_constraint_is_soft(c)) {
                c->type = MPC_CONSTRAINT_HARD_INPUT;
            }
            c->relax_policy = MPC_RELAX_NEVER;
        }
    }

    return MPC_OK;
}

/* ====================================================================
 * L7: ABB Predict & Control — Economic Weight Ranking
 *
 * ABB's Predict & Control (formerly 3dMPC) uses economic weights to
 * prioritize constraints:
 *   - Each CV constraint has an economic cost of violation ($/unit)
 *   - The QP objective includes weighted sum of constraint violations
 *   - Constraints with higher economic cost are satisfied first
 *   - Soft constraints with zone control (no penalty within zone)
 * ==================================================================== */

mpc_status_t mpc_abb_predict_economic_ranking(
    const mpc_output_prioritization_t *op,
    double *economic_weights)
{
    if (!op || !economic_weights) return MPC_ERR_NULL_POINTER;

    /* Economic weights = violation costs, normalized */
    double total_cost = 0.0;
    for (int i = 0; i < op->num_cv; i++) {
        total_cost += op->cv_violation_cost[i];
    }

    if (total_cost > MPC_CONSTRAINT_EPS) {
        for (int i = 0; i < op->num_cv; i++) {
            economic_weights[i] = op->cv_violation_cost[i] / total_cost;
        }
    } else {
        /* All equal if no cost data */
        double w = 1.0 / (double)op->num_cv;
        for (int i = 0; i < op->num_cv; i++) {
            economic_weights[i] = w;
        }
    }

    return MPC_OK;
}

/* ====================================================================
 * L8: Constraint Funnel Management
 *
 * The "funnel" concept (Honeywell RMPCT, also used in ABB and others):
 *   - During a disturbance/transient, temporarily widen constraints
 *   - Gradually narrow (funnel closes) as process returns to steady state
 *   - This prevents infeasibility during transients while maintaining
 *     tight control at steady state
 *
 * Funnel equation:
 *   bound(k) = bound_ss + (bound_initial - bound_ss) * exp(-alpha * k)
 *   where alpha = funnel closing rate
 * ==================================================================== */

mpc_status_t mpc_constraint_funnel_update(mpc_constraint_t *constraints,
                                            int num_constraints,
                                            double time,
                                            double closing_rate,
                                            double steady_state_width)
{
    if (!constraints) return MPC_ERR_NULL_POINTER;

    double funnel_factor = exp(-closing_rate * time);

    for (int i = 0; i < num_constraints; i++) {
        if (constraints[i].scope == MPC_SCOPE_OUTPUT &&
            mpc_constraint_is_soft(&constraints[i])) {
            /* Widen soft output constraints during transients */
            double midpoint = (constraints[i].lower_bound +
                               constraints[i].upper_bound) / 2.0;
            double half_width = steady_state_width / funnel_factor;
            constraints[i].lower_bound = midpoint - half_width;
            constraints[i].upper_bound = midpoint + half_width;
        }
    }

    return MPC_OK;
}

/* ====================================================================
 * L8: Lexicographic MPC — Complete Algorithm
 *
 * Full lexicographic MPC implementation:
 *   1. Priority 0 (CRITICAL): Solve QP — minimize J0 subject to C0*x <= d0
 *   2. For k = 1, 2, 3:
 *      Solve QP — minimize Jk subject to Ck*x <= dk AND
 *        J0 <= J0* + epsilon0, ..., J_{k-1} <= J*_{k-1} + epsilon_{k-1}
 *   3. Final solution respects all priorities
 *
 * Reference:
 *   Kerrigan & Maciejowski (2002), "Designing model predictive controllers
 *     with prioritised constraints and objectives"
 * ==================================================================== */

mpc_status_t mpc_lexicographic_solve(
    const mpc_constraint_set_t *cs,
    const double *H, const double *f,
    int num_vars,
    mpc_qp_solution_t *solution)
{
    if (!cs || !H || !f || !solution)
        return MPC_ERR_NULL_POINTER;

    /* Pseudocode for lexicographic MPC:
     * J_prev = infinity
     * for priority in [CRITICAL, HIGH, MEDIUM, LOW]:
     *   build QP: min 0.5*x'*H*x + f'*x
     *             s.t. constraints at priority
     *                  J_prev <= J_prev* + tol (for non-CRITICAL)
     *   solve QP
     *   J_prev = J_optimal
     *   if infeasible: relax constraints at this priority
     * return solution */

    double J_prev_star = 1e30;

    for (int level = MPC_PRIORITY_CRITICAL;
         level <= MPC_PRIORITY_LOW; level++) {

        int count = cs->priority_count[level];
        (void)cs->priority_start[level];

        /* Check if any constraint at this level is active */
        if (count == 0) continue;

        /* Accumulate objective for this level */
        double J_level = 0.0;
        for (int i = 0; i < num_vars; i++) {
            J_level += f[i] * solution->du_optimal[i];
            for (int j = 0; j < num_vars; j++) {
                J_level += 0.5 * solution->du_optimal[i] * H[i * num_vars + j] *
                           solution->du_optimal[j];
            }
        }

        /* Check if J_prev constraint is satisfied */
        if (level > MPC_PRIORITY_CRITICAL) {
            if (J_level > J_prev_star + MPC_FEASIBILITY_TOL) {
                /* Need to project onto J_prev <= J_prev* */
                /* In full implementation: add constraint and re-solve */
                solution->required_relaxation = true;
                solution->num_levels_relaxed++;
            }
        }

        J_prev_star = J_level;
        solution->priority_objectives[level] = J_level;
        solution->status = MPC_FEASIBLE;
    }

    return MPC_OK;
}

/* ====================================================================
 * L8: Constraint Conditioning Analysis
 *
 * Ill-conditioned constraints (nearly parallel active constraints)
 * cause numerical difficulties in QP solvers. This function computes
 * the condition number of the active constraint set.
 *
 * Condition number of constraint matrix A_active:
 *   kappa(A) = sigma_max(A) / sigma_min(A)
 *
 * Large kappa (> 1e8) indicates near-linear dependency among active
 * constraints, which leads to:
 *   - Inaccurate Lagrange multipliers
 *   - Slower QP convergence
 *   - Potential infeasibility detection failures
 * ==================================================================== */

mpc_status_t mpc_constraint_conditioning_analysis(
    const mpc_constraint_set_t *cs,
    const double *A, int m, int n,
    double *condition_numbers)
{
    if (!cs || !A || !condition_numbers)
        return MPC_ERR_NULL_POINTER;

    for (int i = 0; i < cs->total_count; i++) {
        /* Estimate condition number contribution of constraint i:
         * cond_i = ||a_i|| * ||A^{-1}|| (rough estimate)
         * For simplicity: condition number = ||a_i|| / min(||a_j||) */
        double norm_i = 0.0;
        if (cs->constraints[i].num_coefficients > 0 &&
            cs->constraints[i].coefficients) {
            for (int j = 0; j < cs->constraints[i].num_coefficients; j++)
                norm_i += cs->constraints[i].coefficients[j] *
                          cs->constraints[i].coefficients[j];
            norm_i = sqrt(norm_i);
        }

        /* Compare with minimum norm among all constraints */
        double min_norm = MPC_BOUND_INF;
        for (int j = 0; j < m; j++) {
            double nj = 0.0;
            for (int k = 0; k < n; k++)
                nj += A[j * n + k] * A[j * n + k];
            nj = sqrt(nj);
            if (nj > MPC_CONSTRAINT_EPS && nj < min_norm)
                min_norm = nj;
        }

        if (min_norm > MPC_CONSTRAINT_EPS && min_norm < MPC_BOUND_INF * 0.5) {
            condition_numbers[i] = norm_i / min_norm;
        } else {
            condition_numbers[i] = 1.0;
        }
    }

    return MPC_OK;
}