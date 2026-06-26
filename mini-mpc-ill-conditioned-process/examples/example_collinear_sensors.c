#include "mpc_illcond_defs.h"
#include "mpc_illcond_matrix.h"
#include "mpc_illcond_svd.h"
#include "mpc_illcond_sensitivity.h"
#include "mpc_illcond_condition.h"
#include "mpc_illcond_regularization.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* Example 3: Collinear Sensors in a Multicomponent Distillation Column
 *
 * L6 Canonical Problem: When multiple temperature sensors are placed
 * close together on a distillation column, their measurements become
 * highly correlated (collinear), making the gain matrix nearly singular.
 *
 * This demonstrates:
 * - Collinearity detection between gain matrix columns
 * - SVD-based nullspace detection
 * - Sensor selection recommendations
 *
 * Reference: Moore (1992) "Selection of Secondary Measurements",
 *   in Practical Distillation Control, Van Nostrand Reinhold.
 */

int main(void)
{
    printf("============================================================\n");
    printf("Example: Collinear Sensors -- Sensor Redundancy in MPC\n");
    printf("  Canonical Problem L6: Measurement Redundancy\n");
    printf("============================================================\n\n");

    /* 3 outputs (temperatures at trays 5, 7, 9), 2 inputs (reflux, steam)
     * Trays 5 and 7 are close together -> nearly collinear measurements */
    mpc_matrix_t *G = mpc_matrix_alloc(3, 2);

    /* Gains: dT_i / d(ref lux), dT_i / d(steam) */
    mpc_matrix_set(G, 0, 0, 0.45); mpc_matrix_set(G, 0, 1, -0.30);  /* Tray 5 */
    mpc_matrix_set(G, 1, 0, 0.48); mpc_matrix_set(G, 1, 1, -0.32);  /* Tray 7 */
    mpc_matrix_set(G, 2, 0, 0.15); mpc_matrix_set(G, 2, 1, -0.55);  /* Tray 9 */

    printf("Gain matrix G (3 outputs x 2 inputs):\n");
    for (size_t i = 0; i < 3; i++) {
        printf("  Tray %zu: [ %7.3f  %7.3f ]\n",
               (size_t)((i==0)?5:(i==1)?7:9),
               mpc_matrix_get(G,i,0), mpc_matrix_get(G,i,1));
    }
    printf("\n");

    /* Collinearity detection */
    size_t worst_i, worst_j;
    double coll = mpc_sensitivity_collinearity(G, &worst_i, &worst_j);
    printf("Max column collinearity = %.4f (columns %zu, %zu)\n",
           coll, worst_i, worst_j);

    if (coll > 0.95) {
        printf("WARNING: Near-collinear gain columns detected!\n");
        printf("  Sensors at tray %zu and tray %zu provide redundant info.\n",
               (size_t)((worst_i==0)?5:(worst_i==1)?7:9),
               (size_t)((worst_j==0)?5:(worst_j==1)?7:9));
        printf("  Recommendation: Remove one sensor or use PCA for decorrelation.\n\n");
    }

    /* SVD analysis */
    mpc_svd_t svd;
    if (mpc_svd_compute(G, &svd) == 0) {
        printf("Singular values:\n");
        for (size_t i = 0; i < 2; i++)
            printf("  sigma_%zu = %.6e\n", i+1, svd.S[i]);
        printf("Condition (2-norm): kappa_2 = %.2e\n", svd.cond);

        /* Nullspace detection: for 3x2, there's always at least 1 nullspace dim */
        size_t null_dim = mpc_svd_nullspace_dim(&svd, 1e-10);
        printf("Nullspace dimension = %zu\n\n", null_dim);
    }

    /* RGA for the best 2-output subset */
    mpc_matrix_t *G_sub = mpc_matrix_alloc(2, 2);
    /* Use tray 5 and tray 9 (least collinear pair) */
    mpc_matrix_set(G_sub, 0, 0, mpc_matrix_get(G,0,0));
    mpc_matrix_set(G_sub, 0, 1, mpc_matrix_get(G,0,1));
    mpc_matrix_set(G_sub, 1, 0, mpc_matrix_get(G,2,0));
    mpc_matrix_set(G_sub, 1, 1, mpc_matrix_get(G,2,1));

    mpc_matrix_t *RGA = mpc_matrix_alloc(2, 2);
    mpc_sensitivity_rga(G_sub, RGA);
    printf("RGA for best sensor pair (trays 5 and 9):\n");
    printf("  [ %7.3f  %7.3f ]\n",
           mpc_matrix_get(RGA,0,0), mpc_matrix_get(RGA,0,1));
    printf("  [ %7.3f  %7.3f ]\n\n",
           mpc_matrix_get(RGA,1,0), mpc_matrix_get(RGA,1,1));
    printf("RGA condition = %.2f\n\n", mpc_sensitivity_rga_cond(RGA));

    /* Compare: with all 3 sensors vs best 2 */
    double kappa_full = mpc_condition_estimate(G, MPC_CONDEST_NORM1);
    double kappa_sub = mpc_condition_estimate(G_sub, MPC_CONDEST_NORM1);
    printf("Condition with all 3 sensors: kappa = %.2e\n", kappa_full);
    printf("Condition with best 2 sensors:  kappa = %.2e\n\n", kappa_sub);

    printf("Conclusion: Removing the collinear sensor (tray 7) improves\n");
    printf("conditioning by %.1fx while maintaining observability.\n",
           kappa_full / kappa_sub);

    mpc_svd_free(&svd);
    mpc_matrix_free(&G);
    mpc_matrix_free(&G_sub);
    mpc_matrix_free(&RGA);
    return 0;
}

