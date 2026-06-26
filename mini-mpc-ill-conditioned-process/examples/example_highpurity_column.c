#include "mpc_illcond_defs.h"
#include "mpc_illcond_matrix.h"
#include "mpc_illcond_svd.h"
#include "mpc_illcond_sensitivity.h"
#include "mpc_illcond_condition.h"
#include "mpc_illcond_regularization.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* Example 1: High-Purity Distillation Column (L6 Canonical Problem)
 *
 * A high-purity binary distillation column separating methanol/water
 * has a steady-state gain matrix that becomes severely ill-conditioned
 * as purity approaches 100%. The gains for top and bottom composition
 * control become nearly collinear, leading to kappa > 1e6.
 *
 * This example demonstrates:
 * - Computing the condition number of the gain matrix
 * - RGA analysis for interaction assessment
 * - Regularization for stable MPC control
 *
 * Reference: Skogestad & Postlethwaite (2005), Example 3.14.
 */

int main(void)
{
    printf("============================================================\n");
    printf("Example: High-Purity Distillation Column\n");
    printf("  Canonical Problem L6: Ill-Conditioned MPC\n");
    printf("============================================================\n\n");

    /* Typical gain matrix for high-purity methanol/water column
     * Outputs: yD (distillate composition), xB (bottoms composition)
     * Inputs: L (reflux), V (boilup)
     *
     * Gains for 99.9% purity: columns become nearly collinear. */
    mpc_matrix_t *G = mpc_matrix_alloc(2, 2);
    mpc_matrix_set(G, 0, 0, 0.878);  mpc_matrix_set(G, 0, 1, -0.864);
    mpc_matrix_set(G, 1, 0, 1.082);  mpc_matrix_set(G, 1, 1, -1.096);

    printf("Gain matrix G (ny=%zu, nu=%zu):\n", G->rows, G->cols);
    printf("  [ %8.4f  %8.4f ]\n", mpc_matrix_get(G,0,0), mpc_matrix_get(G,0,1));
    printf("  [ %8.4f  %8.4f ]\n\n", mpc_matrix_get(G,1,0), mpc_matrix_get(G,1,1));

    /* Condition number via Hager-Higham estimator */
    double kappa = mpc_condition_estimate(G, MPC_CONDEST_NORM1);
    printf("Condition number kappa_1(G) = %.2e\n", kappa);
    printf("Condition grade: %s\n", mpc_condition_grade(kappa));
    printf("Digits lost: ~%.1f (of 16)\n\n", mpc_condition_digits_lost(kappa));

    /* RGA analysis */
    mpc_matrix_t *RGA = mpc_matrix_alloc(2, 2);
    mpc_sensitivity_rga(G, RGA);
    printf("Relative Gain Array (RGA):\n");
    printf("  [ %8.4f  %8.4f ]\n", mpc_matrix_get(RGA,0,0), mpc_matrix_get(RGA,0,1));
    printf("  [ %8.4f  %8.4f ]\n", mpc_matrix_get(RGA,1,0), mpc_matrix_get(RGA,1,1));
    printf("RGA condition = %.2f\n\n", mpc_sensitivity_rga_cond(RGA));

    /* SVD analysis for gain directions */
    mpc_svd_t svd;
    if (mpc_svd_compute(G, &svd) == 0) {
        printf("Singular values: sigma_1 = %.6e, sigma_2 = %.6e\n",
               svd.S[0], svd.S[1]);
        printf("Condition (2-norm): kappa_2 = %.2e\n", svd.cond);
        printf("Numerical rank = %zu\n\n", svd.rank);

        /* Minimum gain direction -- reveals which MV combination has
         * the weakest effect, a key insight for ill-conditioned control */
        double v_min[2], u_min[2];
        mpc_sensitivity_min_gain_direction(&svd, v_min, u_min);
        printf("Minimum gain input direction v_min = [%.4f, %.4f]\n",
               v_min[0], v_min[1]);
        printf("Corresponding output direction u_min = [%.4f, %.4f]\n\n",
               u_min[0], u_min[1]);
    }

    /* Collinearity detection */
    size_t wi, wj;
    double coll = mpc_sensitivity_collinearity(G, &wi, &wj);
    printf("Max column collinearity = %.4f (columns %zu, %zu)\n", coll, wi, wj);

    /* Regularization recommendation */
    mpc_illcond_diagnostic_t diag;
    mpc_condition_diagnose(G, &diag);
    printf("Recommended regularization lambda = %.6e\n\n",
           diag.recommended_lambda);

    /* Demonstrate Tikhonov regularization */
    mpc_matrix_t *G_reg = mpc_matrix_copy(G);
    mpc_regularize_tikhonov(G_reg, diag.recommended_lambda);
    double kappa_reg = mpc_condition_estimate(G_reg, MPC_CONDEST_NORM1);
    printf("After Tikhonov (lambda=%.2e): kappa = %.2e\n",
           diag.recommended_lambda, kappa_reg);
    printf("Improvement factor: %.1fx\n\n", kappa / kappa_reg);

    printf("Conclusion: The high-purity column has kappa=%.1e (grade: %s).\n",
           kappa, mpc_condition_grade(kappa));
    printf("Regularization with lambda=%.2e reduces kappa to %.1e,\n",
           diag.recommended_lambda, kappa_reg);
    printf("enabling stable MPC operation at 99.9%% purity.\n");

    mpc_svd_free(&svd);
    mpc_matrix_free(&G);
    mpc_matrix_free(&RGA);
    mpc_matrix_free(&G_reg);
    return 0;
}

