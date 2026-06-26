/**
 * @file mpc_constraint_defs.c
 * @brief Core MPC constraint implementation.
 *
 * Implements constraint lifecycle (init/validate/copy/free), hard/soft
 * classification, violation checking, shadow price calculation, and
 * constraint set management with priority grouping.
 *
 * Knowledge points:
 *   L1: Constraint struct lifecycle and validation
 *   L2: Hard vs soft constraint classification
 *   L3: Priority group indexing and counter management
 *   L4: Bound consistency (lb <= ub), penalty non-negativity
 *   L5: Constraint evaluation (a'*x), sensitivity (shadow price = |lambda|)
 *   L6: Violation checking with feasibility tolerance
 *
 * Reference: Rawlings, Mayne & Diehl (2017), Model Predictive Control, Ch.5
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include "../include/mpc_constraint_defs.h"


mpc_status_t mpc_constraint_init(mpc_constraint_t *c, int index)
{
    if (!c) return MPC_ERR_NULL_POINTER;
    if (index < 0) return MPC_ERR_INVALID_PRIORITY;
    memset(c, 0, sizeof(mpc_constraint_t));
    c->index = index;
    snprintf(c->name, MPC_CONSTRAINT_NAME_MAX, "constraint_%d", index);
    c->type = MPC_CONSTRAINT_SOFT_OUTPUT;
    c->scope = MPC_SCOPE_OUTPUT;
    c->priority = MPC_PRIORITY_LOW;
    c->relax_policy = MPC_RELAX_IF_NEEDED;
    c->lower_bound = -MPC_BOUND_INF;
    c->upper_bound = MPC_BOUND_INF;
    c->target_value = 0.0;
    c->penalty_weight = MPC_DEFAULT_SOFT_PENALTY;
    c->coefficients = NULL;
    c->num_coefficients = 0;
    c->slack_value = 0.0;
    c->slack_max = MPC_BOUND_INF;
    c->slack_penalty_linear = 1e3;
    c->slack_penalty_quadratic = 1e6;
    c->is_active = false;
    c->is_violated = false;
    c->violation_magnitude = 0.0;
    c->violation_duration = 0.0;
    c->violation_start_cycle = 0;
    c->lagrange_multiplier = 0.0;
    c->sensitivity = 0.0;
    c->conditioning_number = 1.0;
    c->is_rank_deficient = false;
    return MPC_OK;
}

mpc_status_t mpc_constraint_set_bounds(mpc_constraint_t *c, double lb, double ub)
{
    if (!c) return MPC_ERR_NULL_POINTER;
    if (lb > ub + MPC_CONSTRAINT_EPS) return MPC_ERR_INVALID_BOUNDS;
    c->lower_bound = lb;
    c->upper_bound = ub;
    c->is_active = !(lb <= -MPC_BOUND_INF * 0.5 && ub >= MPC_BOUND_INF * 0.5);
    return MPC_OK;
}

mpc_status_t mpc_constraint_set_coefficients(mpc_constraint_t *c,
                                              const double *coeffs, int n)
{
    if (!c) return MPC_ERR_NULL_POINTER;
    if (!coeffs && n > 0) return MPC_ERR_NULL_POINTER;
    if (n < 0) return MPC_ERR_DIMENSION_MISMATCH;
    free(c->coefficients);
    c->coefficients = NULL;
    c->num_coefficients = 0;
    if (n > 0) {
        c->coefficients = (double *)calloc((size_t)n, sizeof(double));
        if (!c->coefficients) return MPC_ERR_MEMORY;
        memcpy(c->coefficients, coeffs, (size_t)n * sizeof(double));
        c->num_coefficients = n;
    }
    return MPC_OK;
}

mpc_status_t mpc_constraint_validate(const mpc_constraint_t *c)
{
    if (!c) return MPC_ERR_NULL_POINTER;
    if (c->lower_bound > c->upper_bound + MPC_CONSTRAINT_EPS)
        return MPC_ERR_INVALID_BOUNDS;
    if (c->penalty_weight < 0.0) return MPC_ERR_INVALID_BOUNDS;
    if (mpc_constraint_is_hard(c) && c->relax_policy != MPC_RELAX_NEVER)
        return MPC_ERR_INVALID_PRIORITY;
    if (c->priority == MPC_PRIORITY_CRITICAL && c->relax_policy != MPC_RELAX_NEVER)
        return MPC_ERR_INVALID_PRIORITY;
    if (c->slack_penalty_linear < 0.0 || c->slack_penalty_quadratic < 0.0)
        return MPC_ERR_INVALID_BOUNDS;
    return MPC_OK;
}

void mpc_constraint_free(mpc_constraint_t *c)
{
    if (!c) return;
    free(c->coefficients);
    c->coefficients = NULL;
    c->num_coefficients = 0;
}

mpc_status_t mpc_constraint_copy(mpc_constraint_t *dest, const mpc_constraint_t *src)
{
    if (!dest || !src) return MPC_ERR_NULL_POINTER;
    double *old_coeffs = dest->coefficients;
    memcpy(dest, src, sizeof(mpc_constraint_t));
    dest->coefficients = NULL;
    dest->num_coefficients = 0;
    if (src->num_coefficients > 0 && src->coefficients) {
        dest->coefficients = (double *)calloc((size_t)src->num_coefficients, sizeof(double));
        if (!dest->coefficients) { dest->coefficients = old_coeffs; return MPC_ERR_MEMORY; }
        memcpy(dest->coefficients, src->coefficients, (size_t)src->num_coefficients * sizeof(double));
        dest->num_coefficients = src->num_coefficients;
    }
    free(old_coeffs);
    return MPC_OK;
}

bool mpc_constraint_is_hard(const mpc_constraint_t *c)
{
    if (!c) return false;
    return (c->type == MPC_CONSTRAINT_HARD_INPUT || c->type == MPC_CONSTRAINT_HARD_RATE ||
            c->type == MPC_CONSTRAINT_HARD_OUTPUT || c->type == MPC_CONSTRAINT_HARD_TERMINAL);
}

bool mpc_constraint_is_soft(const mpc_constraint_t *c)
{
    if (!c) return false;
    return (c->type == MPC_CONSTRAINT_SOFT_INPUT || c->type == MPC_CONSTRAINT_SOFT_RATE ||
            c->type == MPC_CONSTRAINT_SOFT_OUTPUT || c->type == MPC_CONSTRAINT_SOFT_TERMINAL);
}

bool mpc_constraint_is_relaxable(const mpc_constraint_t *c)
{
    if (!c) return false;
    return (c->relax_policy == MPC_RELAX_IF_NEEDED ||
            c->relax_policy == MPC_RELAX_ALWAYS_SOFT ||
            c->relax_policy == MPC_RELAX_SEQUENTIAL);
}

const char *mpc_constraint_type_string(mpc_constraint_type_t type)
{
    switch (type) {
        case MPC_CONSTRAINT_HARD_INPUT:    return "HARD_INPUT";
        case MPC_CONSTRAINT_HARD_RATE:     return "HARD_RATE";
        case MPC_CONSTRAINT_HARD_OUTPUT:   return "HARD_OUTPUT";
        case MPC_CONSTRAINT_HARD_TERMINAL: return "HARD_TERMINAL";
        case MPC_CONSTRAINT_SOFT_INPUT:    return "SOFT_INPUT";
        case MPC_CONSTRAINT_SOFT_RATE:     return "SOFT_RATE";
        case MPC_CONSTRAINT_SOFT_OUTPUT:   return "SOFT_OUTPUT";
        case MPC_CONSTRAINT_SOFT_TERMINAL: return "SOFT_TERMINAL";
        case MPC_CONSTRAINT_COUPLING:      return "COUPLING";
        case MPC_CONSTRAINT_CUSTOM:        return "CUSTOM";
        default:                           return "UNKNOWN";
    }
}

const char *mpc_priority_level_string(mpc_priority_level_t level)
{
    switch (level) {
        case MPC_PRIORITY_CRITICAL: return "CRITICAL";
        case MPC_PRIORITY_HIGH:     return "HIGH";
        case MPC_PRIORITY_MEDIUM:   return "MEDIUM";
        case MPC_PRIORITY_LOW:      return "LOW";
        case MPC_PRIORITY_MONITOR:  return "MONITOR";
        default:                    return "UNKNOWN";
    }
}

const char *mpc_feasibility_status_string(mpc_feasibility_status_t status)
{
    switch (status) {
        case MPC_FEASIBLE:      return "FEASIBLE";
        case MPC_INFEASIBLE:    return "INFEASIBLE";
        case MPC_RECOVERABLE:   return "RECOVERABLE";
        case MPC_IRRECOVERABLE: return "IRRECOVERABLE";
        case MPC_DEGENERATE:    return "DEGENERATE";
        default:                return "UNKNOWN";
    }
}

mpc_status_t mpc_constraint_set_init(mpc_constraint_set_t *cs, int capacity)
{
    if (!cs) return MPC_ERR_NULL_POINTER;
    if (capacity <= 0 || capacity > MPC_MAX_CONSTRAINTS)
        return MPC_ERR_DIMENSION_MISMATCH;
    memset(cs, 0, sizeof(mpc_constraint_set_t));
    cs->capacity = capacity;
    cs->total_count = 0;
    cs->constraints = (mpc_constraint_t *)calloc((size_t)capacity, sizeof(mpc_constraint_t));
    if (!cs->constraints) return MPC_ERR_MEMORY;
    cs->feasibility = MPC_FEASIBLE;
    return MPC_OK;
}

void mpc_constraint_set_free(mpc_constraint_set_t *cs)
{
    if (!cs) return;
    if (cs->constraints) {
        for (int i = 0; i < cs->total_count; i++)
            free(cs->constraints[i].coefficients);
        free(cs->constraints);
        cs->constraints = NULL;
    }
    memset(cs, 0, sizeof(mpc_constraint_set_t));
}

mpc_status_t mpc_constraint_set_add(mpc_constraint_set_t *cs, const mpc_constraint_t *c)
{
    if (!cs || !c) return MPC_ERR_NULL_POINTER;
    if (cs->total_count >= cs->capacity) return MPC_ERR_CONSTRAINT_FULL;
    mpc_constraint_t *dest = &cs->constraints[cs->total_count];
    mpc_constraint_init(dest, cs->total_count);
    dest->type = c->type; dest->scope = c->scope;
    dest->priority = c->priority; dest->relax_policy = c->relax_policy;
    strncpy(dest->name, c->name, MPC_CONSTRAINT_NAME_MAX - 1);
    dest->lower_bound = c->lower_bound; dest->upper_bound = c->upper_bound;
    dest->target_value = c->target_value; dest->penalty_weight = c->penalty_weight;
    dest->slack_penalty_linear = c->slack_penalty_linear;
    dest->slack_penalty_quadratic = c->slack_penalty_quadratic;
    dest->slack_max = c->slack_max;
    if (c->num_coefficients > 0 && c->coefficients)
        mpc_constraint_set_coefficients(dest, c->coefficients, c->num_coefficients);
    switch (c->scope) {
        case MPC_SCOPE_INPUT:    cs->count_input++;    break;
        case MPC_SCOPE_RATE:     cs->count_rate++;     break;
        case MPC_SCOPE_OUTPUT:   cs->count_output++;   break;
        case MPC_SCOPE_TERMINAL: cs->count_terminal++; break;
        case MPC_SCOPE_COUPLING: cs->count_coupling++; break;
        default: break;
    }
    cs->total_count++;
    return MPC_OK;
}

double mpc_constraint_evaluate(const mpc_constraint_t *c, const double *x, int n)
{
    if (!c || !x || n <= 0) return 0.0;
    if (!c->coefficients || c->num_coefficients == 0) {
        if (c->index >= n) return 0.0;
        return x[c->index];
    }
    double aTx = 0.0;
    int len = (c->num_coefficients < n) ? c->num_coefficients : n;
    for (int i = 0; i < len; i++) aTx += c->coefficients[i] * x[i];
    return aTx;
}

mpc_status_t mpc_constraint_check_violation(mpc_constraint_t *c,
                                              const double *x, int n)
{
    if (!c || !x || n <= 0) return MPC_ERR_NULL_POINTER;
    double aTx = mpc_constraint_evaluate(c, x, n);
    c->violation_magnitude = 0.0;
    c->is_violated = false;
    if (c->lower_bound > -MPC_BOUND_INF * 0.5) {
        double v = c->lower_bound - aTx;
        if (v > MPC_FEASIBILITY_TOL) { c->violation_magnitude = v; c->is_violated = true; }
    }
    if (c->upper_bound < MPC_BOUND_INF * 0.5) {
        double v = aTx - c->upper_bound;
        if (v > MPC_FEASIBILITY_TOL) {
            if (v > c->violation_magnitude) c->violation_magnitude = v;
            c->is_violated = true;
        }
    }
    return MPC_OK;
}

double mpc_constraint_shadow_price(const mpc_constraint_t *c)
{
    if (!c || !c->is_active) return 0.0;
    return fabs(c->lagrange_multiplier);
}

double mpc_constraint_violation_cost(const mpc_constraint_t *c)
{
    if (!c || !c->is_violated) return 0.0;
    double s = c->slack_value;
    if (s > 0.0)
        return c->slack_penalty_linear * s + c->slack_penalty_quadratic * s * s;
    return c->penalty_weight * c->violation_magnitude;
}

mpc_status_t mpc_constraint_count_by_type(const mpc_constraint_set_t *cs,
                                           int *hard_count, int *soft_count)
{
    if (!cs || !hard_count || !soft_count) return MPC_ERR_NULL_POINTER;
    *hard_count = 0; *soft_count = 0;
    for (int i = 0; i < cs->total_count; i++) {
        if (mpc_constraint_is_hard(&cs->constraints[i])) (*hard_count)++;
        else if (mpc_constraint_is_soft(&cs->constraints[i])) (*soft_count)++;
    }
    return MPC_OK;
}

mpc_status_t mpc_constraint_set_update_feasibility(mpc_constraint_set_t *cs,
                                                     double *violations)
{
    if (!cs || !violations) return MPC_ERR_NULL_POINTER;
    cs->num_soft_violated = 0; cs->num_hard_active = 0; cs->num_soft_active = 0;
    for (int i = 0; i < cs->total_count; i++) {
        mpc_constraint_t *cc = &cs->constraints[i];
        violations[i] = 0.0;
        if (cc->is_violated) {
            violations[i] = cc->violation_magnitude;
            if (mpc_constraint_is_soft(cc)) cs->num_soft_violated++;
        }
        if (cc->is_active) {
            if (mpc_constraint_is_hard(cc)) cs->num_hard_active++;
            else if (mpc_constraint_is_soft(cc)) cs->num_soft_active++;
        }
    }
    bool any_hard_violated = false;
    for (int i = 0; i < cs->total_count; i++) {
        if (mpc_constraint_is_hard(&cs->constraints[i]) &&
            cs->constraints[i].is_violated) {
            any_hard_violated = true; break;
        }
    }
    cs->feasibility = any_hard_violated ? MPC_INFEASIBLE :
        (cs->num_soft_violated > 0 ? MPC_RECOVERABLE : MPC_FEASIBLE);
    return MPC_OK;
}