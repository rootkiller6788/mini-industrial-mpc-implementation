/**
 * @file example_input_saturation.c
 * @brief Example: MPC Input Saturation Handling
 *
 * Demonstrates detection and management of MV saturation in MPC.
 * When a manipulated variable hits its physical limit, the MPC
 * must redistribute control effort or accept performance degradation.
 *
 * Scenario: A distillation column with reflux (MV1) and reboil (MV2).
 * Reflux saturates at max, requiring reboil to compensate.
 *
 * Knowledge: L6 — Input Saturation, Desaturation Path Planning
 * Reference: Qin & Badgwell (2003), Section 5.1
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../include/mpc_constraint_defs.h"

int main(void)
{
    printf("=== MPC Input Saturation Example ===\n\n");

    printf("Scenario: Distillation column with MV constraints\n");
    printf("  MV1: Reflux flow [0, 100] kg/h  (saturated at 100)\n");
    printf("  MV2: Reboil duty  [0, 200] kW   (available)\n\n");

    /* Detect saturation */
    double mv_values[] = {100.0, 120.0};
    double mv_lower[] = {0.0, 0.0};
    double mv_upper[] = {100.0, 200.0};
    mpc_input_saturation_t sat[2];
    memset(sat, 0, sizeof(sat));

    mpc_detect_input_saturation(mv_values, mv_lower, mv_upper, 2, 1, sat);

    printf("Saturation detection results:\n");
    for (int i = 0; i < 2; i++) {
        printf("  MV%d: ", i + 1);
        if (sat[i].at_upper_bound) {
            printf("SATURATED at upper bound (%.1f)\n", sat[i].saturated_value);
            printf("        Lost control authority: %.0f%%\n",
                   sat[i].lost_control_authority * 100.0);
        } else if (sat[i].at_lower_bound) {
            printf("SATURATED at lower bound (%.1f)\n", sat[i].saturated_value);
        } else {
            printf("Available (%.1f)\n", mv_values[i]);
        }
    }

    /* Plan desaturation */
    printf("\nDesaturation path (5-step horizon):\n");
    double mv_steady[] = {50.0, 100.0};
    double desat_moves[10];
    mpc_compute_desaturation_path(sat, mv_steady, 2, desat_moves, 5);

    printf("  Step |  dMV1  |  dMV2\n");
    printf("  -----|--------|-------\n");
    for (int k = 0; k < 5; k++) {
        printf("    %d  | %+6.2f | %+6.2f\n",
               k + 1, desat_moves[0 * 5 + k], desat_moves[1 * 5 + k]);
    }

    printf("\nImplication: MV1 (reflux) is saturated, cannot increase further.\n");
    printf("The MPC should rely on MV2 (reboil) to control composition.\n");
    printf("Desaturation path moves MV1 toward its steady-state value of 50.\n");

    return 0;
}
