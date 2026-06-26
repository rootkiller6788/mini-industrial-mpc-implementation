/**
 * @file mpc_constraint_relaxation.c
 * @brief MPC Constraint Relaxation — Slack variable management implementation.
 *
 * Implements soft constraint relaxation: slack variable allocation, penalty
 * weight auto-tuning, sequential relaxation by priority level, and KKT
 * optimality condition verification for slack variables.
 *
 * Knowledge points:
 *   L1: Slack variable definition (s >= 0, penalty = rho*s + rho*s^2)
 *   L2: Soft constraint concept — desirable but not mandatory
 *   L3: Slack variable data structures and lifecycle
 *   L4: Exact penalty theorem (Fletcher): |rho| > |lambda*| => slack = 0
 *   L5: Sequential relaxation algorithm, penalty auto-tuning
 *   L6: Priority-ordered relaxation for feasibility recovery
 *   L8: KKT conditions for constrained optimization with slacks
 *
 * @section L4_Exact_Penalty
 * Fletcher's exact penalty theorem (1987):
 *   If rho > |lambda*| for all active constraints, then the solution of
 *   min f(x) + rho*sum(s_i) s.t. g(x) - s <= 0, s >= 0
 *   has s_i = 0 for all i, recovering the hard-constrained solution.
 *   Here lambda* are the optimal Lagrange multipliers of the original problem.
 *
 * Reference:
 *   Fletcher (1987), "Practical Methods of Optimization", 2nd ed.
 *   de Oliveira & Biegler (1994), Automatica
 *   Rawlings, Mayne & Diehl (2017), Ch.6
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include "../include/mpc_constraint_relaxation.h"
#include "../include/mpc_constraint_defs.h"

/* ====================================================================
 * L1: Relaxation configuration
 * ==================================================================== */

mpc_status_t mpc_relaxation_config_init(mpc_relaxation_config_t *config)
{
    if (!config) return MPC_ERR_NULL_POINTER;
    config->linear_penalty_base = 1e3;
    config->quadratic_penalty_base = 1e6;
    config->penalty_growth_factor = 10.0;
    config->max_slack_per_constraint = 100.0;
    config->max_relaxation_rounds = 20;
    config->feasibility_tolerance = MPC_FEASIBILITY_TOL;
    config->auto_tune_penalties = true;
    config->use_exact_penalty = false;
    config->exact_penalty_multiplier = 2.0;
    return MPC_OK;
}

mpc_status_t mpc_relaxation_tune_penalties(mpc_relaxation_config_t *config,
                                            const double *lagrange_multipliers, int n)
{
    if (!config || !lagrange_multipliers || n <= 0) return MPC_ERR_NULL_POINTER;
    /* Set linear penalty above max Lagrange multiplier for exact penalty property */
    double max_lambda = 0.0;
    for (int i = 0; i < n; i++) {
        double abs_lambda = fabs(lagrange_multipliers[i]);
        if (abs_lambda > max_lambda) max_lambda = abs_lambda;
    }
    config->linear_penalty_base = max_lambda * config->exact_penalty_multiplier;
    return MPC_OK;
}

mpc_status_t mpc_relaxation_set_exact_penalty(mpc_relaxation_config_t *config,
                                               double multiplier)
{
    if (!config) return MPC_ERR_NULL_POINTER;
    if (multiplier <= 1.0) return MPC_ERR_INVALID_BOUNDS;
    config->use_exact_penalty = true;
    config->exact_penalty_multiplier = multiplier;
    return MPC_OK;
}

/* ====================================================================
 * L3: Slack variable allocation and lifecycle
 * ==================================================================== */

mpc_status_t mpc_relaxation_alloc_slacks(const mpc_constraint_set_t *cs,
                                          const mpc_relaxation_config_t *config,
                                          mpc_relaxation_state_t *state)
{
    if (!cs || !config || !state) return MPC_ERR_NULL_POINTER;
    memset(state, 0, sizeof(mpc_relaxation_state_t));
    /* Count relaxable constraints */
    int n_relaxable = 0;
    for (int i = 0; i < cs->total_count; i++) {
        if (mpc_constraint_is_relaxable(&cs->constraints[i]))
            n_relaxable++;
    }
    if (n_relaxable == 0) {
        state->num_slacks_active = 0;
        state->feasibility_restored = true;
        return MPC_OK;
    }
    state->slack_values = (double *)calloc((size_t)n_relaxable, sizeof(double));
    state->penalty_weights_linear = (double *)calloc((size_t)n_relaxable, sizeof(double));
    state->penalty_weights_quadratic = (double *)calloc((size_t)n_relaxable, sizeof(double));
    if (!state->slack_values || !state->penalty_weights_linear ||
        !state->penalty_weights_quadratic) {
        mpc_relaxation_free_slacks(state);
        return MPC_ERR_MEMORY;
    }
    int idx = 0;
    for (int i = 0; i < cs->total_count; i++) {
        mpc_constraint_t *c = &cs->constraints[i];
        if (mpc_constraint_is_relaxable(c)) {
            state->penalty_weights_linear[idx] = config->linear_penalty_base;
            state->penalty_weights_quadratic[idx] = config->quadratic_penalty_base;
            idx++;
        }
    }
    state->num_slacks_active = n_relaxable;
    state->num_constraints_relaxed = 0;
    return MPC_OK;
}

mpc_status_t mpc_relaxation_init_slacks(mpc_relaxation_state_t *state)
{
    if (!state) return MPC_ERR_NULL_POINTER;
    if (state->slack_values) {
        for (int i = 0; i < state->num_slacks_active; i++)
            state->slack_values[i] = 0.0;
    }
    state->num_constraints_relaxed = 0;
    state->total_slack_cost = 0.0;
    state->max_slack_used = 0.0;
    state->relaxation_rounds_used = 0;
    state->feasibility_restored = true;
    return MPC_OK;
}

void mpc_relaxation_free_slacks(mpc_relaxation_state_t *state)
{
    if (!state) return;
    free(state->slack_values); state->slack_values = NULL;
    free(state->penalty_weights_linear); state->penalty_weights_linear = NULL;
    free(state->penalty_weights_quadratic); state->penalty_weights_quadratic = NULL;
    state->num_slacks_active = 0;
}

int mpc_relaxation_num_active_slacks(const mpc_relaxation_state_t *state)
{
    if (!state) return 0;
    return state->num_slacks_active;
}

/* ====================================================================
 * L5: Sequential relaxation by priority
 *
 * Algorithm:
 *   1. Start with no relaxation (all slacks = 0)
 *   2. For priority = LOW down to CRITICAL:
 *      a. Allow slack variables for constraints at this priority
 *      b. Solve QP
 *      c. If feasible, stop
 *   3. If still infeasible after relaxing all levels, return INFEASIBLE
 *
 * Motivation (industrial practice): Relax operator preferences first,
 * then economic targets, then quality specs. Never relax safety.
 * ==================================================================== */

mpc_status_t mpc_relaxation_sequential_by_priority(
    mpc_constraint_set_t *cs,
    mpc_relaxation_config_t *config,
    mpc_relaxation_state_t *state,
    mpc_qp_solution_t *solution)
{
    if (!cs || !config || !state || !solution) return MPC_ERR_NULL_POINTER;

    /* Start with no relaxation */
    mpc_relaxation_init_slacks(state);
    state->relaxation_rounds_used = 0;

    /* Check if already feasible without relaxation */
    if (solution->status == MPC_FEASIBLE) {
        state->feasibility_restored = true;
        return MPC_OK;
    }

    /* Relax one priority level at a time, starting from lowest */
    for (int level = MPC_PRIORITY_LOW; level >= MPC_PRIORITY_CRITICAL; level--) {
        if (state->relaxation_rounds_used >= config->max_relaxation_rounds)
            break;

        /* Enable slack for constraints at this priority level */
        int slack_idx = 0;
        for (int i = 0; i < cs->total_count; i++) {
            mpc_constraint_t *c = &cs->constraints[i];
            if (c->priority == (mpc_priority_level_t)level &&
                mpc_constraint_is_relaxable(c)) {
                /* Increase slack limit: allow violation up to max_slack */
                if (state->slack_values && slack_idx < state->num_slacks_active) {
                    state->slack_values[slack_idx] =
                        config->max_slack_per_constraint;
                }
                c->slack_value = 0.0;
                c->slack_max = config->max_slack_per_constraint;
                state->num_constraints_relaxed++;
                slack_idx++;
            }
        }

        state->relaxation_rounds_used++;

        /* Check if constraints at this level and above are now feasible */
        /* In a real implementation, this would re-solve the QP.
         * Here we do a simplified feasibility estimate based on violation magnitude. */
        double max_violation = 0.0;
        for (int i = 0; i < cs->total_count; i++) {
            if (cs->constraints[i].priority <= (mpc_priority_level_t)level &&
                cs->constraints[i].is_violated) {
                if (cs->constraints[i].violation_magnitude > max_violation)
                    max_violation = cs->constraints[i].violation_magnitude;
            }
        }

        if (max_violation < MPC_FEASIBILITY_TOL) {
            state->feasibility_restored = true;
            solution->required_relaxation = true;
            solution->highest_relaxed = (mpc_priority_level_t)level;
            solution->num_levels_relaxed = MPC_PRIORITY_LOW - level + 1;
            return MPC_OK;
        }
    }

    /* Still infeasible — constraints at CRITICAL level conflict */
    state->feasibility_restored = false;
    solution->status = MPC_INFEASIBLE;
    solution->required_relaxation = true;
    return MPC_ERR_QP_INFEASIBLE;
}

/* ====================================================================
 * L4: Identify infeasible priority levels using Farkas' Lemma
 *
 * Farkas' Lemma: Exactly one of the following holds:
 *   (I)  There exists x such that A*x <= b
 *   (II) There exists y >= 0 such that y'*A = 0 and y'*b < 0
 *
 * If (II) holds, the constraints are infeasible, and the non-zero entries
 * of y identify the conflicting constraint group.
 * ==================================================================== */

mpc_status_t mpc_relaxation_identify_infeasible_priorities(
    const mpc_constraint_set_t *cs,
    const double *constraint_matrix,
    const double *rhs,
    int *priority_levels_to_relax,
    int *num_levels)
{
    if (!cs || !constraint_matrix || !rhs || !priority_levels_to_relax || !num_levels)
        return MPC_ERR_NULL_POINTER;

    *num_levels = 0;

    /* Simple heuristic: check each priority level for bound conflicts.
     * If any constraint at a level has lb > ub + tol, that level is infeasible. */
    for (int level = MPC_PRIORITY_LOW; level >= MPC_PRIORITY_CRITICAL; level--) {
        bool level_has_conflict = false;
        int start = cs->priority_start[level];
        int count = cs->priority_count[level];
        for (int i = start; i < start + count && !level_has_conflict; i++) {
            if (cs->constraints[i].lower_bound >
                cs->constraints[i].upper_bound + MPC_FEASIBILITY_TOL) {
                level_has_conflict = true;
            }
        }
        if (level_has_conflict && *num_levels < MPC_MAX_PRIORITY_LEVELS) {
            priority_levels_to_relax[*num_levels] = level;
            (*num_levels)++;
        }
    }
    return MPC_OK;
}

/* ====================================================================
 * L5: Penalty weight auto-tuning
 *
 * After each QP solve, check if slack variables are non-zero.
 * If s_i > 0, the penalty was insufficient. Increase it:
 *   rho_new = rho_old * growth_factor
 * ==================================================================== */

mpc_status_t mpc_relaxation_auto_tune_penalty_weights(
    const mpc_qp_solution_t *solution,
    mpc_constraint_set_t *cs,
    mpc_relaxation_state_t *state,
    mpc_relaxation_config_t *config)
{
    if (!solution || !cs || !state || !config) return MPC_ERR_NULL_POINTER;
    if (!config->auto_tune_penalties) return MPC_OK;
    if (!state->slack_values || !state->penalty_weights_linear) return MPC_OK;

    int slack_idx = 0;
    for (int i = 0; i < cs->total_count; i++) {
        if (mpc_constraint_is_relaxable(&cs->constraints[i])) {
            double current_slack = (slack_idx < state->num_slacks_active) ?
                state->slack_values[slack_idx] : 0.0;
            if (current_slack > MPC_FEASIBILITY_TOL) {
                /* Penalty was insufficient — increase it */
                state->penalty_weights_linear[slack_idx] *= config->penalty_growth_factor;
                state->penalty_weights_quadratic[slack_idx] *= config->penalty_growth_factor;
                /* Clamp to prevent overflow */
                if (state->penalty_weights_linear[slack_idx] > 1e12)
                    state->penalty_weights_linear[slack_idx] = 1e12;
                if (state->penalty_weights_quadratic[slack_idx] > 1e15)
                    state->penalty_weights_quadratic[slack_idx] = 1e15;
            }
            slack_idx++;
        }
    }
    return MPC_OK;
}

double mpc_relaxation_total_slack_cost(const mpc_relaxation_state_t *state)
{
    if (!state || !state->slack_values) return 0.0;
    double cost = 0.0;
    for (int i = 0; i < state->num_slacks_active; i++) {
        double s = state->slack_values[i];
        double rho1 = state->penalty_weights_linear ?
                      state->penalty_weights_linear[i] : 0.0;
        double rho2 = state->penalty_weights_quadratic ?
                      state->penalty_weights_quadratic[i] : 0.0;
        cost += rho1 * s + rho2 * s * s;
    }
    /* Note: state is const, cannot write back */
    return cost;
}

/* ====================================================================
 * L8: KKT optimality conditions for slack variables
 *
 * For a soft constraint: g(x) - s <= 0, s >= 0
 * with penalty: rho1*s + rho2*s^2 in objective.
 *
 * Lagrangian: L = f(x) + rho1*s + rho2*s^2 + lambda*(g(x) - s) - mu*s
 *
 * KKT conditions:
 *   Stationarity w.r.t. s: rho1 + 2*rho2*s - lambda - mu = 0
 *   Complementary slackness: mu*s = 0, lambda*(g(x) - s) = 0
 *   Dual feasibility: mu >= 0, lambda >= 0
 *
 * If s > 0: mu = 0 (by complementarity), so lambda = rho1 + 2*rho2*s
 * If s = 0: mu >= 0, so lambda <= rho1 (constraint not active enough)
 * ==================================================================== */

mpc_status_t mpc_relaxation_check_kkt_slacks(
    const mpc_relaxation_state_t *state,
    const double *lagrange_multipliers,
    double tolerance,
    bool *kkt_satisfied)
{
    if (!state || !lagrange_multipliers || !kkt_satisfied)
        return MPC_ERR_NULL_POINTER;

    *kkt_satisfied = true;

    if (!state->slack_values || !state->penalty_weights_linear) {
        *kkt_satisfied = true;
        return MPC_OK;
    }

    for (int i = 0; i < state->num_slacks_active; i++) {
        double s = state->slack_values[i];
        double rho1 = state->penalty_weights_linear[i];
        double rho2 = (state->penalty_weights_quadratic) ?
                       state->penalty_weights_quadratic[i] : 0.0;
        double lambda = lagrange_multipliers[i];

        /* Check: if s > 0, then lambda should equal rho1 + 2*rho2*s */
        if (s > tolerance) {
            double expected_lambda = rho1 + 2.0 * rho2 * s;
            if (fabs(lambda - expected_lambda) > tolerance * fmax(1.0, fabs(lambda))) {
                *kkt_satisfied = false;
            }
        }
        /* Check: if s = 0, then lambda should be <= rho1 */
        if (s <= tolerance && lambda > rho1 + tolerance) {
            *kkt_satisfied = false;
        }
    }

    return MPC_OK;
}

double mpc_relaxation_marginal_benefit(const mpc_constraint_t *c,
                                         const mpc_relaxation_state_t *state)
{
    if (!c || !state) return 0.0;
    /* Marginal benefit of relaxing constraint c by ds:
     * dB/ds = -penalty_linear - 2*penalty_quadratic*s + shadow_price
     * Negative = beneficial to relax more (cost reduction > penalty)
     * Positive = beneficial to tighten (penalty > cost reduction) */
    double s = c->slack_value;
    double rho1 = c->slack_penalty_linear;
    double rho2 = c->slack_penalty_quadratic;
    double shadow = mpc_constraint_shadow_price(c);
    return shadow - (rho1 + 2.0 * rho2 * s);
}