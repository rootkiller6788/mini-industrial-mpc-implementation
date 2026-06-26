/** @file example_distillation.c
 * @brief Example: MIMO DMC for Binary Distillation Column (L6, L7)
 *
 * Canonical distillation column control problem (Wood & Berry 1973):
 *   2x2 system: Reflux (MV1) and Steam (MV2) control
 *               Top composition (CV1) and Bottom composition (CV2)
 *
 *   G(s) = [ 12.8*exp(-s)/(16.7s+1)   -18.9*exp(-3s)/(21.0s+1)  ]
 *          [ 6.6*exp(-7s)/(10.9s+1)   -19.4*exp(-3s)/(14.4s+1)  ]
 *
 * Ref: Wood & Berry (1973), "Terminal composition control of a
 *      binary distillation column", Chemical Engineering Science.
 *
 * This is a standard benchmark for multivariable control (used by
 * AspenTech DMC3 for refinery CDU demonstrations).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "mpc_common.h"

int main(void) {
    printf("=== Example: MIMO DMC for Wood-Berry Distillation Column ===\n\n");

    /* Create MIMO model: 2 MVs, 2 CVs, 0 DVs */
    mpc_mimo_model_t *mimo = mpc_mimo_model_alloc(2, 2, 0, 60);
    mimo->sample_time_sec = 1.0;

    /* Sub-model (1,1): Top composition / Reflux */
    mpc_step_model_t *g11 = mpc_step_model_alloc(60);
    mpc_step_model_from_fopdt(g11, 12.8, 16.7, 1.0, 1.0);
    mpc_mimo_set_submodel(mimo, 0, 0, g11);

    /* Sub-model (1,2): Top composition / Steam */
    mpc_step_model_t *g12 = mpc_step_model_alloc(60);
    mpc_step_model_from_fopdt(g12, -18.9, 21.0, 3.0, 1.0);
    mpc_mimo_set_submodel(mimo, 0, 1, g12);

    /* Sub-model (2,1): Bottom composition / Reflux */
    mpc_step_model_t *g21 = mpc_step_model_alloc(60);
    mpc_step_model_from_fopdt(g21, 6.6, 10.9, 7.0, 1.0);
    mpc_mimo_set_submodel(mimo, 1, 0, g21);

    /* Sub-model (2,2): Bottom composition / Steam */
    mpc_step_model_t *g22 = mpc_step_model_alloc(60);
    mpc_step_model_from_fopdt(g22, -19.4, 14.4, 3.0, 1.0);
    mpc_mimo_set_submodel(mimo, 1, 1, g22);

    printf("Wood-Berry Distillation Column Model:\n");
    printf("  MV1: Reflux flow rate\n");
    printf("  MV2: Steam flow rate\n");
    printf("  CV1: Top composition (mole fraction)\n");
    printf("  CV2: Bottom composition (mole fraction)\n\n");

    /* Extract steady-state gain matrix */
    double G_ss[4];
    mpc_mimo_extract_ss_gain(mimo, G_ss);
    printf("Steady-state gain matrix:\n");
    printf("  [ %7.2f  %7.2f ]\n", G_ss[0], G_ss[1]);
    printf("  [ %7.2f  %7.2f ]\n\n", G_ss[2], G_ss[3]);

    /* Compute condition number */
    double kappa = mpc_condition_number_svd(G_ss, 2, 2);
    printf("Condition number kappa(G): %.2f\n", kappa);
    if (kappa > 100.0) {
        printf("  WARNING: Ill-conditioned! Consider SVD truncation.\n");
    } else if (kappa > 10.0) {
        printf("  Moderate conditioning. Use move suppression.\n");
    } else {
        printf("  Well-conditioned. Standard DMC is fine.\n");
    }

    printf("\n=== Example complete ===\n");

    mpc_step_model_free(g11);
    mpc_step_model_free(g12);
    mpc_step_model_free(g21);
    mpc_step_model_free(g22);
    mpc_mimo_model_free(mimo);
    return 0;
}
