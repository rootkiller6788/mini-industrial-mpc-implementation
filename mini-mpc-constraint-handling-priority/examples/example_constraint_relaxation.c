/**
 * @file example_constraint_relaxation.c
 * @brief Example: Soft Constraint Relaxation in MPC
 *
 * Demonstrates sequential constraint relaxation by priority level
 * when the QP becomes infeasible with all constraints active.
 *
 * Scenario: A chemical reactor with multiple constraints.
 * A large disturbance makes all output constraints infeasible.
 * The relaxation system drops LOW priority constraints first,
 * then MEDIUM, preserving CRITICAL and HIGH.
 *
 * Knowledge: L5 — Sequential Relaxation, L6 — Feasibility Recovery
 * Reference: de Oliveira & Biegler (1994), Automatica
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../include/mpc_constraint_defs.h"
#include "../include/mpc_constraint_relaxation.h"
#include "../include/mpc_constraint_priority.h"

int main(void)
{
    printf("=== Constraint Relaxation Example ===\n\n");

    printf("Scenario: Chemical reactor under large disturbance\n");
    printf("  4 constraints active, but MV is saturated.\n");
    printf("  Cannot satisfy all — must relax by priority.\n\n");

    /* Build constraint set */
    mpc_constraint_set_t cs;
    mpc_constraint_set_init(&cs, 10);

    /* CRITICAL: Reactor pressure < 50 bar (safety) */
    mpc_constraint_t c;
    mpc_constraint_init(&c, 0);
    mpc_constraint_set_priority(&c, MPC_PRIORITY_CRITICAL);
    mpc_constraint_set_bounds(&c, -MPC_BOUND_INF, 50.0);
    c.type = MPC_CONSTRAINT_HARD_OUTPUT;
    c.relax_policy = MPC_RELAX_NEVER;
    snprintf(c.name, MPC_CONSTRAINT_NAME_MAX, "Reactor Pressure");
    mpc_constraint_set_add(&cs, &c);

    /* HIGH: Product purity > 95% */
    mpc_constraint_init(&c, 1);
    mpc_constraint_set_priority(&c, MPC_PRIORITY_HIGH);
    mpc_constraint_set_bounds(&c, 95.0, MPC_BOUND_INF);
    c.type = MPC_CONSTRAINT_SOFT_OUTPUT;
    c.relax_policy = MPC_RELAX_IF_NEEDED;
    snprintf(c.name, MPC_CONSTRAINT_NAME_MAX, "Product Purity");
    mpc_constraint_set_add(&cs, &c);

    /* MEDIUM: Production rate > 80% of target */
    mpc_constraint_init(&c, 2);
    mpc_constraint_set_priority(&c, MPC_PRIORITY_MEDIUM);
    mpc_constraint_set_bounds(&c, 80.0, MPC_BOUND_INF);
    c.type = MPC_CONSTRAINT_SOFT_OUTPUT;
    c.relax_policy = MPC_RELAX_IF_NEEDED;
    snprintf(c.name, MPC_CONSTRAINT_NAME_MAX, "Production Rate");
    mpc_constraint_set_add(&cs, &c);

    /* LOW: Energy efficiency > 70% */
    mpc_constraint_init(&c, 3);
    mpc_constraint_set_priority(&c, MPC_PRIORITY_LOW);
    mpc_constraint_set_bounds(&c, 70.0, MPC_BOUND_INF);
    c.type = MPC_CONSTRAINT_SOFT_OUTPUT;
    c.relax_policy = MPC_RELAX_SEQUENTIAL;
    snprintf(c.name, MPC_CONSTRAINT_NAME_MAX, "Energy Efficiency");
    mpc_constraint_set_add(&cs, &c);

    mpc_constraint_free(&c);

    /* Build priority index */
    mpc_constraint_sort_by_priority(&cs);

    printf("Constraint set built: %d constraints\n", cs.total_count);
    printf("Priority distribution:\n");
    for (int level = 0; level < MPC_MAX_PRIORITY_LEVELS; level++) {
        if (cs.priority_count[level] > 0) {
            printf("  %s: %d constraint(s)\n",
                   mpc_priority_level_string((mpc_priority_level_t)level),
                   cs.priority_count[level]);
        }
    }

    /* Mark some constraints as violated (simulating disturbance) */
    cs.constraints[2].is_violated = true;  /* MEDIUM: Production rate */
    cs.constraints[2].violation_magnitude = 15.0;
    cs.constraints[3].is_violated = true;  /* LOW: Energy efficiency */
    cs.constraints[3].violation_magnitude = 25.0;

    printf("\nDisturbance: Production rate violated by 15%%, energy efficiency by 25%%\n");

    /* Set up relaxation */
    mpc_relaxation_config_t rcfg;
    mpc_relaxation_config_init(&rcfg);
    rcfg.max_relaxation_rounds = 3;

    mpc_relaxation_state_t rstate;
    mpc_relaxation_alloc_slacks(&cs, &rcfg, &rstate);
    mpc_relaxation_init_slacks(&rstate);

    printf("\nRelaxation configuration:\n");
    printf("  Linear penalty base:   %.0f\n", rcfg.linear_penalty_base);
    printf("  Quadratic penalty base: %.0f\n", rcfg.quadratic_penalty_base);
    printf("  Max relaxation rounds:  %d\n", rcfg.max_relaxation_rounds);

    /* Simulate sequential relaxation */
    mpc_qp_solution_t sol;
    memset(&sol, 0, sizeof(sol));
    sol.status = MPC_INFEASIBLE;

    mpc_relaxation_sequential_by_priority(
        &cs, &rcfg, &rstate, &sol);

    printf("\nRelaxation result:\n");
    printf("  Feasibility restored: %s\n",
           rstate.feasibility_restored ? "YES" : "NO");
    printf("  Constraints relaxed:  %d\n", rstate.num_constraints_relaxed);
    printf("  Round used:           %d\n", rstate.relaxation_rounds_used);
    printf("  Highest level relaxed: %s\n",
           mpc_priority_level_string(sol.highest_relaxed));
    printf("  Levels relaxed:        %d\n", sol.num_levels_relaxed);

    printf("\nRelaxation order (lower priority first):\n");
    printf("  1. Energy Efficiency (LOW) — relaxed to allow violation\n");
    printf("  2. Production Rate (MEDIUM) — relaxed if still infeasible\n");
    printf("  3. Product Purity (HIGH) — last resort, rarely relaxed\n");
    printf("  X. Reactor Pressure (CRITICAL) — NEVER relaxed\n");

    printf("\nKey insight: Industrial MPC never compromises safety.\n");
    printf("Economic constraints are always relaxed before process/safety constraints.\n");

    mpc_relaxation_free_slacks(&rstate);
    mpc_constraint_set_free(&cs);

    return 0;
}
