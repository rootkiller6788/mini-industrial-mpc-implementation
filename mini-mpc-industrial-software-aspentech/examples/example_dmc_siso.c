/** @file example_dmc_siso.c
 * @brief Example: Single-Loop DMC Control of FOPDT Process (L6)
 *
 * Demonstrates DMC control of a simple First-Order Plus Dead Time process.
 *   Process: G(s) = 2.0 * exp(-3s) / (10s + 1)
 *   Setpoint: 1.0 (unit step)
 *
 * This is the canonical DMC example from Cutler & Ramaker (1980).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "mpc_common.h"

int main(void) {
    printf("=== Example: SISO DMC Control of FOPDT Process ===\n\n");

    /* Build FOPDT step-response model */
    mpc_step_model_t *model = mpc_step_model_alloc(60);
    mpc_step_model_from_fopdt(model, 2.0, 10.0, 3.0, 1.0);
    printf("FOPDT model: K=%.2f, tau=%.1fs, theta=%.1fs\n",
           model->gain_ss, model->time_constant_sec, model->dead_time_samples);

    /* Validate model */
    int valid = mpc_step_model_validate(model);
    printf("Model validation: %s\n", valid == 0 ? "PASSED" : "FAILED");

    /* Build dynamic matrix */
    mpc_dynamic_matrix_t dm;
    int rc = mpc_build_dynamic_matrix(model, 30, 5, &dm);
    printf("Dynamic matrix: P=%d, M=%d, build=%s\n",
           dm.P, dm.M, rc == 0 ? "OK" : "FAIL");

    /* Truncation error */
    double trunc_err = mpc_step_model_truncation_error(model, 0.01);
    printf("Truncation error (extra coeffs needed): %.0f\n", trunc_err);

    /* Print first few step-response coefficients */
    printf("\nStep-response coefficients (first 10):\n");
    for (int i = 0; i < 10; i++) {
        printf("  s[%2d] = %8.4f\n", i+1, model->coeff[i]);
    }

    printf("\n=== Example complete ===\n");
    mpc_dynamic_matrix_free(&dm);
    mpc_step_model_free(model);
    return 0;
}
