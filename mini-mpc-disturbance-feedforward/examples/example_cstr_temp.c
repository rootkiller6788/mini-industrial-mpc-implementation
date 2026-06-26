/**
 * @file example_cstr_temp.c
 * @brief CSTR MPC + Feedforward Demonstration
 *
 * Demonstrates MPC model construction, prediction, and
 * feedforward gain computation for a CSTR reactor.
 */

#include "mpc_controller.h"
#include <stdio.h>
#include <math.h>

int main(void) {
    mpc_ss_model_t plant;
    mpc_tuning_t tuning;
    mpc_aug_model_t aug;
    mpc_prediction_t pred;
    mpc_ff_config_t ff;
    double Kd[MPC_MAX_NU][MPC_MAX_ND];
    int i;

    printf("=== CSTR MPC + Feedforward Demo ===\n\n");

    /* 1. Create FOPDT plant: G(s) = 1.0*exp(-2s)/(10s+1), Ts=1.0 */
    mpc_model_fopdt(&plant, 1.0, 10.0, 2.0, 1.0);
    plant.nd = 1;
    plant.Bd[0][0] = 0.5;
    printf("Plant: FOPDT K=1.0, tau=10, theta=2, Ts=1.0\n");
    printf("  nx=%d, nu=%d, ny=%d, nd=%d\n", plant.nx, plant.nu, plant.ny, plant.nd);
    printf("  A[0,0]=%.6f, B[0,0]=%.6f\n", plant.A[0][0], plant.B[0][0]);

    /* 2. Tune MPC */
    mpc_tuning_init_default(&tuning);
    tuning.np = 10;
    tuning.nc = 3;
    tuning.ts = 1.0;
    tuning.q_weight[0] = 10.0;
    tuning.r_weight[0] = 0.1;
    printf("\nMPC Tuning: Np=%d, Nc=%d, Ts=%.1f\n", tuning.np, tuning.nc, tuning.ts);

    /* 3. Build augmented model (output disturbance for offset-free) */
    mpc_build_augmented_model(&plant, MPC_MODE_OUTPUT_DISTURBANCE, &aug);
    printf("\nAugmented Model: nx_aug=%d\n", aug.nx_aug);

    /* 4. Build prediction matrices */
    mpc_build_prediction_matrices(&aug, &tuning, &pred);
    printf("Prediction: Phi[0,0]=%.6f, Gamma[0,0]=%.6f\n",
           pred.Phi[0][0], pred.Gamma[0][0]);

    /* 5. Compute steady-state target for setpoint y=50 */
    double y_ref[MPC_MAX_NY] = {50.0};
    double x_ss[MPC_MAX_NX], u_ss[MPC_MAX_NU];
    mpc_compute_steady_state_target(&plant, y_ref, x_ss, u_ss);
    printf("\nSteady-state target for y=50: u_ss=%.3f\n", u_ss[0]);

    /* 6. Compute feedforward gain */
    memset(&ff, 0, sizeof(ff));
    ff.ff_gain_scale = 1.0;
    mpc_ff_compute_static_gain(&plant, Kd);
    printf("\nFeedforward gain Kd = %.6f\n", Kd[0][0]);

    /* 7. Demonstrate FF computation for a disturbance */
    double d_meas[MPC_MAX_ND] = {5.0};  /* +5 degC disturbance */
    double u_ff[MPC_MAX_NU];
    mpc_ff_compute_static(&ff, d_meas, u_ff);
    printf("For d=%.1f: u_ff = %.3f (feedforward action)\n", d_meas[0], u_ff[0]);

    /* 8. Check controllability and observability */
    int obs = mpc_check_observability(&aug);
    int ctr = mpc_check_controllability(&aug);
    printf("\nObservability: %s, Controllability: %s\n",
           obs ? "YES" : "NO", ctr ? "YES" : "NO");

    /* 9. Compute DARE solution for terminal cost */
    double Q_dare[MPC_MAX_NX][MPC_MAX_NX];
    double R_dare[MPC_MAX_NU][MPC_MAX_NU];
    double P_dare[MPC_MAX_NX][MPC_MAX_NX];
    for (i = 0; i < aug.nx_aug; i++) Q_dare[i][i] = 1.0;
    R_dare[0][0] = 0.1;
    int iters = mpc_solve_dare(aug.Aa, aug.Ba, Q_dare, R_dare,
                                aug.nx_aug, aug.nu, P_dare, 500, 1e-8);
    printf("DARE solved in %d iterations, P[0,0]=%.6f\n", iters, P_dare[0][0]);

    /* 10. Build step response model */
    mpc_step_model_t step;
    mpc_build_step_model(&plant, 10, &step);
    printf("\nStep response (first 5):");
    for (i = 0; i < 5 && i < step.n; i++)
        printf(" %.4f", step.S[i][0][0]);
    printf("\n");

    printf("\n=== Demo Complete ===\n");
    return 0;
}
