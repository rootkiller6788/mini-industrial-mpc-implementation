/**
 * gpc_vs_dmc_comparison.c — GPC vs DMC Comparison for Integrating Level
 *
 * Compares Generalized Predictive Control (CARIMA model) with Dynamic
 * Matrix Control (step response model) for the same integrating tank.
 * Demonstrates that both methods achieve offset-free tracking for
 * integrating processes, with different tuning characteristics.
 *
 * DMC uses step response + constant disturbance bias correction.
 * GPC uses CARIMA model + inherent integral action from Δ operator.
 *
 * This example maps to L5: Algorithm comparison and L6: Canonical problem.
 *
 * Reference: Clarke et al. (1987) vs Cutler & Ramaker (1979)
 *            Lundström et al. (1995) "Limitations of DMC for integrating processes"
 *
 * Oxford (Clarke/GPC) · MIT 2.171 · CMU 24-677
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "../include/mpc_level_types.h"
#include "../include/mpc_integrating_model.h"
#include "../include/mpc_dmc.h"
#include "../include/mpc_gpc.h"
#include "../include/mpc_qp_solver.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int main(void) {
    printf("\n========================================\n");
    printf("  GPC vs DMC COMPARISON\n");
    printf("  Integrating Tank Level Control\n");
    printf("========================================\n\n");

    /* Common process: tank with D=2m, A=π m² */
    double A_tank = M_PI * 2.0 * 2.0 / 4.0; /* π m² */
    double valve_Cv = 0.08;
    double level_op = 1.5;

    /* Process gain: K = -Cv*sqrt(h)/(100*A) [m/(s·%valve)] */
    double K = -valve_Cv * sqrt(level_op) / (100.0 * A_tank);
    printf("Tank: A=%.4f m², K=%.6f m/(s·%%valve)\n", A_tank, K);
    printf("Operating level: %.2f m\n\n", level_op);

    /* Common sampling and horizons */
    double Ts = 1.0;
    int N_p = 30;
    int N_c = 5;
    double lambda = 1.0;

    /* ─── DMC Setup ─────────────────────────────────────────────── */
    printf("--- DMC Configuration ---\n");

    integrating_process_t model_dmc;
    mpc_model_pure_integrator(&model_dmc, K, Ts, A_tank);

    step_response_t step;
    mpc_step_response_from_model(&step, &model_dmc, N_p);
    printf("Step response: N=%d, s1=%.6f, s%d=%.6f\n",
           step.n_coeffs, step.coeff[0], N_p-1, step.coeff[N_p-1]);

    mpc_tuning_t tuning_dmc;
    memset(&tuning_dmc, 0, sizeof(tuning_dmc));
    tuning_dmc.prediction_horizon = N_p;
    tuning_dmc.control_horizon = N_c;
    tuning_dmc.sampling_time = Ts;
    tuning_dmc.move_suppression = lambda;
    tuning_dmc.output_weight = 1.0;
    tuning_dmc.terminal_weight = 100.0;
    tuning_dmc.reference_trajectory_alpha = 0.7;

    mpc_level_config_t config;
    memset(&config, 0, sizeof(config));
    config.level_setpoint = 1.8;
    config.valve_min = 5.0;
    config.valve_max = 95.0;
    config.valve_rate_max = 100.0;
    config.level_lo_limit = 0.0;
    config.level_hi_limit = 3.0;

    dmc_dynamic_t dyn;
    dmc_init(&dyn, &step, &tuning_dmc);

    /* ─── GPC Setup ─────────────────────────────────────────────── */
    printf("\n--- GPC Configuration ---\n");

    gpc_config_t gpc;
    gpc_integrating_setup(&gpc, K, Ts, N_p, N_c, lambda);
    printf("CARIMA: A(z)=1 - z^{-1}, B(z)=%.6f\n", gpc.B_coeff[0]);
    printf("N1=%d, N2=%d, Nu=%d, lambda=%.2f\n",
           gpc.n1, gpc.n2, gpc.nu, gpc.lambda_gpc);

    /* ─── Comparison Simulation ────────────────────────────────── */
    printf("\n--- Simulating Both Controllers ---\n");

    /* Plant simulation: single tank dynamics */
    double h_dmc = 1.5;     /* DMC-controlled level */
    double h_gpc = 1.5;     /* GPC-controlled level */
    double u_dmc = 50.0;    /* DMC valve position */
    double u_gpc = 50.0;    /* GPC valve position */

    double du_past_dmc[30] = {0};
    double du_past_gpc[30] = {0};
    double y_past_gpc[10] = {1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5};

    double F_in = 0.18; /* nominal inlet flow [m³/s] */

    printf("Step     h_dmc    h_gpc   u_dmc   u_gpc   F_out_dmc F_out_gpc\n");
    printf("----  -------- --------  ------  ------  --------- ---------\n");

    for (int k = 0; k < 60; k++) {
        /* DMC control step */
        mpc_solution_t sol_dmc;
        memset(&sol_dmc, 0, sizeof(sol_dmc));
        double du_dmc = dmc_step(&sol_dmc, &dyn, &step, &tuning_dmc,
                                  &config, h_dmc, u_dmc, MPC_SOLVER_HILDRETH);

        u_dmc += du_dmc;
        if (u_dmc < 5.0) u_dmc = 5.0;
        if (u_dmc > 95.0) u_dmc = 95.0;

        /* GPC control step */
        mpc_solution_t sol_gpc;
        memset(&sol_gpc, 0, sizeof(sol_gpc));
        double du_gpc = gpc_step(&sol_gpc, &gpc, h_gpc, 1.8,
                                  u_gpc, du_past_gpc, y_past_gpc);

        u_gpc += du_gpc;
        if (u_gpc < 5.0) u_gpc = 5.0;
        if (u_gpc > 95.0) u_gpc = 95.0;

        /* Step disturbance at k=20 */
        if (k == 20) {
            F_in = 0.30;
            printf("  *** Inlet flow step +67%% at step %d ***\n", k);
        }
        if (k == 40) {
            F_in = 0.18;
            printf("  *** Inlet flow restored at step %d ***\n", k);
        }

        /* Outflow via valve */
        double F_out_dmc = (valve_Cv > 0.0)
            ? valve_Cv * sqrt(h_dmc) * (u_dmc / 100.0)
            : u_dmc / 100.0;
        double F_out_gpc = (valve_Cv > 0.0)
            ? valve_Cv * sqrt(h_gpc) * (u_gpc / 100.0)
            : u_gpc / 100.0;

        /* Euler update */
        h_dmc += Ts * (F_in - F_out_dmc) / A_tank;
        h_gpc += Ts * (F_in - F_out_gpc) / A_tank;
        if (h_dmc < 0.0) h_dmc = 0.0;
        if (h_gpc < 0.0) h_gpc = 0.0;

        if (k % 10 == 0) {
            printf("%4d   %7.3f   %7.3f   %5.1f   %5.1f   %7.4f   %7.4f\n",
                   k, h_dmc, h_gpc, u_dmc, u_gpc, F_out_dmc, F_out_gpc);
        }

        /* Shift past data for next iteration */
        memmove(&du_past_dmc[1], du_past_dmc, 29 * sizeof(double));
        memmove(&du_past_gpc[1], du_past_gpc, 29 * sizeof(double));
        du_past_dmc[0] = du_dmc;
        du_past_gpc[0] = du_gpc;
        memmove(&y_past_gpc[1], y_past_gpc, 9 * sizeof(double));
        y_past_gpc[0] = h_gpc;
    }

    printf("\nFinal state:\n");
    printf("  DMC: h=%.4f m, u=%.1f %%, F_out=%.4f m³/s\n",
           h_dmc, u_dmc,
           valve_Cv * sqrt(h_dmc) * (u_dmc / 100.0));
    printf("  GPC: h=%.4f m, u=%.1f %%, F_out=%.4f m³/s\n",
           h_gpc, u_gpc,
           valve_Cv * sqrt(h_gpc) * (u_gpc / 100.0));
    printf("  Setpoint = 1.80 m\n");

    printf("\n========================================\n");
    printf("  GPC vs DMC Comparison Complete\n");
    printf("========================================\n\n");

    return 0;
}
