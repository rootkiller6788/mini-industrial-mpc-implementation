#include "mpc_illcond_defs.h"
#include "mpc_illcond_matrix.h"
#include "mpc_illcond_svd.h"
#include "mpc_illcond_sensitivity.h"
#include "mpc_illcond_condition.h"
#include <stdio.h>
#include <math.h>

/* Example 2: Stiff CSTR Reactor (L6 Canonical Problem)
 *
 * A continuous stirred-tank reactor with fast temperature dynamics
 * (tau ~ 10s) and slow concentration dynamics (tau ~ 1000s) has a
 * stiffness ratio of ~100, leading to ill-conditioned dynamic matrices
 * when the MPC sample time is chosen for the fast dynamics.
 *
 * This demonstrates stiffness detection and multi-rate considerations
 * for ill-conditioned MPC in chemical reactor control.
 *
 * Reference: Seborg, Edgar, Mellichamp (2016), Ch. 16.
 */

int main(void)
{
    printf("============================================================\n");
    printf("Example: Stiff CSTR Reactor with MPC\n");
    printf("  Canonical Problem L6: Stiffness-Induced Ill-Conditioning\n");
    printf("============================================================\n\n");

    /* Time constants for 2x2 reactor:
     *   Output 1 (temperature): tau11 = 10s, tau12 = 15s
     *   Output 2 (concentration): tau21 = 800s, tau22 = 1200s
     *   Input 1: cooling water, Input 2: feed flow */
    double tau[4] = {10.0, 15.0, 800.0, 1200.0};
    double stiffness = mpc_sensitivity_stiffness(tau, 2, 2);

    printf("Time constants (s):\n");
    printf("  T->CW: %.0f, T->F: %.0f\n", tau[0], tau[1]);
    printf("  C->CW: %.0f, C->F: %.0f\n", tau[2], tau[3]);
    printf("Stiffness ratio: tau_max/tau_min = %.1f\n\n", stiffness);

    if (stiffness > 1.0e5) {
        printf("WARNING: High stiffness ratio (%.1f > %.1e threshold).\n",
               stiffness, 1.0e5);
        printf("This reactor requires multi-rate MPC or stiff ODE solvers.\n\n");
    }

    /* Construct steady-state gain matrix for reactor */
    mpc_matrix_t *G = mpc_matrix_alloc(2, 2);
    /* K11 = dT/dCW, K12 = dT/dF, K21 = dC/dCW, K22 = dC/dF */
    mpc_matrix_set(G, 0, 0, -2.5); mpc_matrix_set(G, 0, 1, 0.3);
    mpc_matrix_set(G, 1, 0, -0.1); mpc_matrix_set(G, 1, 1, 0.8);

    printf("Gain matrix G:\n");
    printf("  [ %7.2f  %7.2f ]\n", mpc_matrix_get(G,0,0), mpc_matrix_get(G,0,1));
    printf("  [ %7.2f  %7.2f ]\n\n", mpc_matrix_get(G,1,0), mpc_matrix_get(G,1,1));

    double kappa = mpc_condition_estimate(G, MPC_CONDEST_NORM1);
    printf("Condition number kappa(G) = %.2e\n", kappa);

    /* Dynamic matrix condition growth with prediction horizon */
    printf("\nPrediction horizon impact on conditioning:\n");
    for (size_t P = 5; P <= 40; P += 5) {
        double growth = mpc_condition_horizon_growth(P, 5, 0.05);
        printf("  P=%2zu: condition growth factor = %.2f\n", P, growth);
    }

    /* Stiffness diagnostic */
    mpc_illcond_diagnostic_t diag;
    mpc_condition_diagnose(G, &diag);
    diag.stiffness_diagnostic = stiffness;
    diag.primary_cause = mpc_sensitivity_rootcause(&diag);

    printf("\nSensitivity: %d (0=LOW, 1=MODERATE, 2=HIGH, 3=EXTREME)\n",
           (int)diag.sensitivity);
    printf("Root cause: %d (2=HIGH_STIFFNESS_RATIO)\n",
           (int)diag.primary_cause);
    printf("Recommended lambda: %.2e\n\n", diag.recommended_lambda);

    printf("Conclusion: Stiff reactors need careful MPC sample time\n");
    printf("selection. Consider multi-rate MPC with fast inner loop\n");
    printf("(temperature, Ts=1s) and slow outer loop (concentration, Ts=100s).\n");

    mpc_matrix_free(&G);
    return 0;
}

