/**
 * @file mpc_constraint_priority.c
 * @brief MPC Constraint Priority Management implementation.
 *
 * Implements priority-based constraint sorting, indexing, activation,
 * conflict detection and resolution, and priority inheritance.
 *
 * Knowledge points:
 *   L1: Priority descriptors and level definitions
 *   L2: Lexicographic constraint ordering concept
 *   L3: Priority index building with O(n log n) sort
 *   L5: Quicksort-based priority sorting, conflict detection algorithms
 *   L6: Priority inheritance for cascaded constraints
 *
 * Reference: Rawlings, Mayne & Diehl (2017), Ch.5
 *            Qin & Badgwell (2003), Control Engineering Practice
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../include/mpc_constraint_priority.h"
#include "../include/mpc_constraint_defs.h"

static const mpc_priority_descriptor_t g_priority_table[] = {
    { MPC_PRIORITY_CRITICAL, "CRITICAL", "Safety and actuator physical limits", false, 0.0 },
    { MPC_PRIORITY_HIGH,     "HIGH",     "Process stability and key quality",  false, 0.05 },
    { MPC_PRIORITY_MEDIUM,   "MEDIUM",   "Economic optimization",              true,  0.10 },
    { MPC_PRIORITY_LOW,      "LOW",      "Operator preferences",               true,  0.25 },
    { MPC_PRIORITY_MONITOR,  "MONITOR",  "Information only, no enforcement",   true,  1.00 }
};
#define NUM_PRIORITY_LEVELS 5

mpc_status_t mpc_priority_init_descriptors(mpc_priority_descriptor_t *descriptors,
                                            int num_levels)
{
    if (!descriptors || num_levels <= 0) return MPC_ERR_NULL_POINTER;
    int n = (num_levels < NUM_PRIORITY_LEVELS) ? num_levels : NUM_PRIORITY_LEVELS;
    memcpy(descriptors, g_priority_table, (size_t)n * sizeof(mpc_priority_descriptor_t));
    return MPC_OK;
}

const char *mpc_priority_label(mpc_priority_level_t level)
{
    for (int i = 0; i < NUM_PRIORITY_LEVELS; i++)
        if (g_priority_table[i].level == level)
            return g_priority_table[i].label;
    return "UNKNOWN";
}

const char *mpc_priority_description(mpc_priority_level_t level)
{
    for (int i = 0; i < NUM_PRIORITY_LEVELS; i++)
        if (g_priority_table[i].level == level)
            return g_priority_table[i].description;
    return "Unknown priority level";
}

bool mpc_priority_is_relaxable(mpc_priority_level_t level)
{
    for (int i = 0; i < NUM_PRIORITY_LEVELS; i++)
        if (g_priority_table[i].level == level)
            return g_priority_table[i].is_relaxable;
    return (level >= MPC_PRIORITY_MEDIUM);
}

mpc_status_t mpc_constraint_set_priority(mpc_constraint_t *c,
                                          mpc_priority_level_t priority)
{
    if (!c) return MPC_ERR_NULL_POINTER;
    if (priority < MPC_PRIORITY_CRITICAL || priority > MPC_PRIORITY_MONITOR)
        return MPC_ERR_INVALID_PRIORITY;
    c->priority = priority;
    if (priority == MPC_PRIORITY_CRITICAL)
        c->relax_policy = MPC_RELAX_NEVER;
    return MPC_OK;
}

mpc_status_t mpc_constraint_validate_priorities(const mpc_constraint_set_t *cs)
{
    if (!cs) return MPC_ERR_NULL_POINTER;
    for (int i = 0; i < cs->total_count; i++) {
        const mpc_constraint_t *c = &cs->constraints[i];
        if (c->priority == MPC_PRIORITY_CRITICAL && c->relax_policy != MPC_RELAX_NEVER)
            return MPC_ERR_INVALID_PRIORITY;
        if (mpc_constraint_is_hard(c) && c->relax_policy != MPC_RELAX_NEVER)
            return MPC_ERR_INVALID_PRIORITY;
    }
    return MPC_OK;
}

int mpc_constraint_audit_relaxation_policy(const mpc_constraint_set_t *cs)
{
    if (!cs) return 0;
    int violations = 0;
    for (int i = 0; i < cs->total_count; i++) {
        const mpc_constraint_t *c = &cs->constraints[i];
        if (c->priority == MPC_PRIORITY_CRITICAL && c->relax_policy != MPC_RELAX_NEVER)
            violations++;
    }
    return violations;
}

/* Comparison function for qsort: sort by priority (ascending = CRITICAL first) */
static int cmp_priority(const void *a, const void *b)
{
    const mpc_constraint_t *ca = (const mpc_constraint_t *)a;
    const mpc_constraint_t *cb = (const mpc_constraint_t *)b;
    if (ca->priority < cb->priority) return -1;
    if (ca->priority > cb->priority) return 1;
    return 0;
}

mpc_status_t mpc_constraint_sort_by_priority(mpc_constraint_set_t *cs)
{
    if (!cs) return MPC_ERR_NULL_POINTER;
    if (cs->total_count <= 1) return MPC_OK;
    qsort(cs->constraints, (size_t)cs->total_count, sizeof(mpc_constraint_t),
          cmp_priority);
    /* Reindex after sort */
    for (int i = 0; i < cs->total_count; i++)
        cs->constraints[i].index = i;
    return mpc_constraint_build_priority_index(cs);
}

mpc_status_t mpc_constraint_build_priority_index(mpc_constraint_set_t *cs)
{
    if (!cs) return MPC_ERR_NULL_POINTER;
    memset(cs->priority_count, 0, sizeof(cs->priority_count));
    memset(cs->priority_start, 0, sizeof(cs->priority_start));
    for (int i = 0; i < cs->total_count; i++) {
        int level = (int)cs->constraints[i].priority;
        if (level >= 0 && level < MPC_MAX_PRIORITY_LEVELS)
            cs->priority_count[level]++;
    }
    int start = 0;
    for (int level = 0; level < MPC_MAX_PRIORITY_LEVELS; level++) {
        cs->priority_start[level] = start;
        start += cs->priority_count[level];
    }
    return MPC_OK;
}

int mpc_constraint_count_at_priority(const mpc_constraint_set_t *cs,
                                      mpc_priority_level_t level)
{
    if (!cs || level < 0 || level >= MPC_MAX_PRIORITY_LEVELS) return 0;
    return cs->priority_count[level];
}

int mpc_constraint_start_at_priority(const mpc_constraint_set_t *cs,
                                      mpc_priority_level_t level)
{
    if (!cs || level < 0 || level >= MPC_MAX_PRIORITY_LEVELS) return -1;
    return cs->priority_start[level];
}

mpc_status_t mpc_constraint_activate_upto_priority(mpc_constraint_set_t *cs,
                                                     mpc_priority_level_t max_level)
{
    if (!cs) return MPC_ERR_NULL_POINTER;
    for (int i = 0; i < cs->total_count; i++) {
        if (cs->constraints[i].priority <= max_level)
            cs->constraints[i].is_active = true;
        else
            cs->constraints[i].is_active = false;
    }
    return MPC_OK;
}

int mpc_constraint_deactivate_below_priority(mpc_constraint_set_t *cs,
                                              mpc_priority_level_t min_level)
{
    if (!cs) return 0;
    int count = 0;
    for (int i = 0; i < cs->total_count; i++) {
        if (cs->constraints[i].priority > min_level && cs->constraints[i].is_active) {
            cs->constraints[i].is_active = false;
            count++;
        }
    }
    return count;
}

mpc_status_t mpc_constraint_active_count_by_priority(const mpc_constraint_set_t *cs,
                                                       int *counts)
{
    if (!cs || !counts) return MPC_ERR_NULL_POINTER;
    memset(counts, 0, MPC_MAX_PRIORITY_LEVELS * sizeof(int));
    for (int i = 0; i < cs->total_count; i++) {
        if (cs->constraints[i].is_active) {
            int level = (int)cs->constraints[i].priority;
            if (level >= 0 && level < MPC_MAX_PRIORITY_LEVELS)
                counts[level]++;
        }
    }
    return MPC_OK;
}

/* L5: Conflict detection — find constraints that cannot be simultaneously satisfied.
 * Two constraints A1*x <= b1 and A2*x <= b2 conflict if the intersection of their
 * half-spaces is empty given other active constraints. */
mpc_status_t mpc_constraint_detect_conflicts(const mpc_constraint_set_t *cs,
                                              int *conflict_count,
                                              mpc_priority_conflict_t *conflicts,
                                              int max_conflicts)
{
    if (!cs || !conflict_count || !conflicts) return MPC_ERR_NULL_POINTER;
    *conflict_count = 0;
    for (int i = 0; i < cs->total_count && *conflict_count < max_conflicts; i++) {
        if (!cs->constraints[i].is_active) continue;
        for (int j = i + 1; j < cs->total_count && *conflict_count < max_conflicts; j++) {
            if (!cs->constraints[j].is_active) continue;
            /* Simple heuristic: if lower_bound of one exceeds upper_bound of other
             * and they share coefficients, they may conflict */
            bool share_coeffs = (cs->constraints[i].num_coefficients > 0 &&
                                 cs->constraints[j].num_coefficients > 0 &&
                                 cs->constraints[i].num_coefficients ==
                                 cs->constraints[j].num_coefficients);
            double conflict_measure = 0.0;
            if (share_coeffs) {
                /* Check if constraint bounds are mutually exclusive */
                if (cs->constraints[i].upper_bound < cs->constraints[j].lower_bound -
                    MPC_FEASIBILITY_TOL) {
                    conflict_measure = cs->constraints[j].lower_bound -
                                       cs->constraints[i].upper_bound;
                }
            }
            if (conflict_measure > 0.0) {
                conflicts[*conflict_count].conflicting_constraint_1 = i;
                conflicts[*conflict_count].conflicting_constraint_2 = j;
                conflicts[*conflict_count].conflict_measure = conflict_measure;
                conflicts[*conflict_count].resolved = false;
                conflicts[*conflict_count].relaxed_constraint = -1;
                conflicts[*conflict_count].compromise_slack = conflict_measure;
                (*conflict_count)++;
            }
        }
    }
    return MPC_OK;
}

mpc_status_t mpc_constraint_resolve_conflict(mpc_constraint_set_t *cs,
                                              const mpc_priority_conflict_t *conflict)
{
    if (!cs || !conflict) return MPC_ERR_NULL_POINTER;
    int i = conflict->conflicting_constraint_1;
    int j = conflict->conflicting_constraint_2;
    if (i < 0 || i >= cs->total_count || j < 0 || j >= cs->total_count)
        return MPC_ERR_INVALID_PRIORITY;
    mpc_constraint_t *c1 = &cs->constraints[i];
    mpc_constraint_t *c2 = &cs->constraints[j];
    /* Relax the lower-priority constraint (higher enum value) */
    if (c1->priority < c2->priority) {
        c2->slack_value = conflict->conflict_measure;
        c2->is_violated = true;
    } else if (c2->priority < c1->priority) {
        c1->slack_value = conflict->conflict_measure;
        c1->is_violated = true;
    } else {
        /* Same priority: relax the one with lower sensitivity (shadow price) */
        if (c1->sensitivity <= c2->sensitivity) {
            c1->slack_value = conflict->conflict_measure;
            c1->is_violated = true;
        } else {
            c2->slack_value = conflict->conflict_measure;
            c2->is_violated = true;
        }
    }
    return MPC_OK;
}

mpc_status_t mpc_constraint_inherit_priority(mpc_constraint_set_t *cs,
                                              int dependent_idx, int dependency_idx)
{
    if (!cs) return MPC_ERR_NULL_POINTER;
    if (dependent_idx < 0 || dependent_idx >= cs->total_count ||
        dependency_idx < 0 || dependency_idx >= cs->total_count)
        return MPC_ERR_INVALID_PRIORITY;
    mpc_constraint_t *dep = &cs->constraints[dependent_idx];
    mpc_constraint_t *src = &cs->constraints[dependency_idx];
    /* Inherit the higher priority (lower enum value) */
    if (src->priority < dep->priority)
        dep->priority = src->priority;
    return MPC_OK;
}

int mpc_constraint_detect_inheritance_violations(const mpc_constraint_set_t *cs,
                                                   int *violation_pairs, int max_pairs)
{
    if (!cs || !violation_pairs) return 0;
    int count = 0;
    for (int i = 0; i < cs->total_count && count < max_pairs; i++) {
        for (int j = i + 1; j < cs->total_count && count < max_pairs; j++) {
            /* If constraint j depends on i but has higher priority, it's a violation */
            if (cs->constraints[j].priority < cs->constraints[i].priority) {
                violation_pairs[2 * count] = i;
                violation_pairs[2 * count + 1] = j;
                count++;
            }
        }
    }
    return count;
}

mpc_status_t mpc_constraint_priority_summary(const mpc_constraint_set_t *cs,
                                               char *buffer, int buffer_size)
{
    if (!cs || !buffer || buffer_size <= 0) return MPC_ERR_NULL_POINTER;
    int pos = 0;
    pos += snprintf(buffer + pos, (size_t)(buffer_size - pos),
                    "Constraint Priority Summary [%d total]:\n", cs->total_count);
    for (int level = 0; level < MPC_MAX_PRIORITY_LEVELS; level++) {
        if (cs->priority_count[level] > 0 && pos < buffer_size) {
            pos += snprintf(buffer + pos, (size_t)(buffer_size - pos),
                            "  %s: %d constraints\n",
                            mpc_priority_label((mpc_priority_level_t)level),
                            cs->priority_count[level]);
        }
    }
    return MPC_OK;
}

mpc_priority_level_t mpc_constraint_highest_violated_priority(
    const mpc_constraint_set_t *cs)
{
    if (!cs) return MPC_PRIORITY_MONITOR;
    for (int level = 0; level < MPC_MAX_PRIORITY_LEVELS; level++) {
        int start = cs->priority_start[level];
        int count = cs->priority_count[level];
        for (int i = start; i < start + count; i++) {
            if (cs->constraints[i].is_violated)
                return (mpc_priority_level_t)level;
        }
    }
    return MPC_PRIORITY_MONITOR;
}

mpc_status_t mpc_constraint_violation_count_by_priority(
    const mpc_constraint_set_t *cs, int *violation_counts)
{
    if (!cs || !violation_counts) return MPC_ERR_NULL_POINTER;
    memset(violation_counts, 0, MPC_MAX_PRIORITY_LEVELS * sizeof(int));
    for (int i = 0; i < cs->total_count; i++) {
        if (cs->constraints[i].is_violated) {
            int level = (int)cs->constraints[i].priority;
            if (level >= 0 && level < MPC_MAX_PRIORITY_LEVELS)
                violation_counts[level]++;
        }
    }
    return MPC_OK;
}