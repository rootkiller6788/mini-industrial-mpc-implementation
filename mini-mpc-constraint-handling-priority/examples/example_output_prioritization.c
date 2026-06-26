/**
 * @file example_output_prioritization.c
 * @brief Example: Multi-CV Output Constraint Prioritization
 *
 * Demonstrates priority-based output constraint management in
 * a multi-input multi-output (MIMO) MPC application.
 *
 * Scenario: FCC (Fluid Catalytic Cracking) unit with 4 CVs:
 *   CV1: Reactor temperature (CRITICAL — safety, 480-520 C)
 *   CV2: Regenerator temperature (HIGH — catalyst protection, 650-720 C)
 *   CV3: Gasoline yield (MEDIUM — economics, 45-55 vol%)
 *   CV4: LPG production (LOW — secondary product, 15-25 vol%)
 *
 * Knowledge: L6 — Output Prioritization, Multi-CV Constraint Ranking
 * Reference: Darby & Nikolaou (2012), Control Engineering Practice
 */

#include <stdio.h>
#include <stdlib.h>
#include "../include/mpc_constraint_defs.h"

int main(void)
{
    printf("=== Output Prioritization: FCC Unit Example ===\n\n");

    mpc_output_prioritization_t op;
    mpc_output_prioritization_init(&op, 4);

    /* Configure CVs with priorities */
    mpc_output_prioritization_set_cv(&op, 0, MPC_PRIORITY_CRITICAL,
                                      480.0, 520.0, 10000.0); /* Reactor temp */
    mpc_output_prioritization_set_cv(&op, 1, MPC_PRIORITY_HIGH,
                                      650.0, 720.0, 5000.0);  /* Regen temp */
    mpc_output_prioritization_set_cv(&op, 2, MPC_PRIORITY_MEDIUM,
                                      45.0, 55.0, 1000.0);    /* Gasoline yield */
    mpc_output_prioritization_set_cv(&op, 3, MPC_PRIORITY_LOW,
                                      15.0, 25.0, 200.0);     /* LPG production */

    printf("CV configuration:\n");
    const char *cv_names[] = {"Reactor Temp", "Regen Temp",
                               "Gasoline Yield", "LPG Production"};
    for (int i = 0; i < 4; i++) {
        printf("  CV%d [%s]: priority=%s, range=[%.0f, %.0f], cost=$%.0f/unit\n",
               i + 1, cv_names[i],
               mpc_priority_level_string((mpc_priority_level_t)op.cv_priority[i]),
               op.cv_lower_limit[i], op.cv_upper_limit[i],
               op.cv_violation_cost[i]);
    }

    /* Scenario: Disturbance causes violations */
    printf("\nScenario: Feed composition disturbance\n");
    double cv_values[] = {515.0, 735.0, 42.0, 28.0};
    printf("Current CV values: R=%.0f, G=%.0f, Y=%.1f, L=%.1f\n",
           cv_values[0], cv_values[1], cv_values[2], cv_values[3]);

    mpc_output_prioritization_evaluate(&op, cv_values);

    printf("\nViolation report:\n");
    for (int i = 0; i < 4; i++) {
        printf("  CV%d: ", i + 1);
        if (op.cv_constraint_active[i]) {
            if (cv_values[i] < op.cv_lower_limit[i])
                printf("VIOLATED (below lower limit by %.1f)\n",
                       op.cv_lower_limit[i] - cv_values[i]);
            else
                printf("VIOLATED (above upper limit by %.1f)\n",
                       cv_values[i] - op.cv_upper_limit[i]);
        } else {
            printf("OK\n");
        }
    }

    /* Rank violations */
    int ranked[4] = {-1, -1, -1, -1};
    mpc_output_prioritization_rank_violations(&op, ranked);

    printf("\nConstraint satisfaction order (by priority):\n");
    for (int i = 0; i < 4 && ranked[i] >= 0; i++) {
        printf("  %d. CV%d (%s) — Priority: %s\n",
               i + 1, ranked[i] + 1, cv_names[ranked[i]],
               mpc_priority_level_string((mpc_priority_level_t)op.cv_priority[ranked[i]]));
    }

    printf("\nInterpretation:\n");
    printf("  Regenerator temp (HIGH) violated first — protect catalyst\n");
    printf("  Gasoline yield (MEDIUM) and LPG (LOW) also violated\n");
    printf("  MPC should prioritize MV moves to satisfy HIGH constraint first\n");

    mpc_output_prioritization_free(&op);
    return 0;
}
