/**
 * dmc_level_control_demo.c — DMC Level Control Demonstration
 *
 * Demonstrates Dynamic Matrix Control for a pure integrating tank level
 * process. Shows step response model building, DMC configuration, and
 * closed-loop simulation comparing tight vs. averaging level control.
 *
 * This example maps to L6: "Single tank level control" and demonstrates
 * the classic trade-off between tight level regulation and flow smoothing.
 *
 * MIT 2.171 · Purdue ME 575 · RWTH Industrial Control
 */

#include <stdio.h>
#include <math.h>
#include "../include/mpc_level_types.h"
#include "../include/mpc_integrating_model.h"
#include "../include/mpc_dmc.h"
#include "../include/mpc_qp_solver.h"
#include "../include/mpc_surge_tank.h"

int main(void) {
    printf("\n========================================\n");
    printf("  DMC LEVEL CONTROL DEMONSTRATION\n");
    printf("  Pure Integrating Tank Process\n");
    printf("========================================\n\n");

    /* Tank parameters: D=2m, H=3m, A=π m² */
    surge_tank_t tank;
    surge_tank_init(&tank, 2.0, 3.0, 1.5, 0.1);

    /* Process model: integrating with gain from physical parameters */
    double K = mpc_tank_area_to_gain(tank.cross_area, tank.valve_coeff, tank.level);
    printf("Process gain K = %.6f m/(s·%%valve)\n", K);
    printf("Tank area A = %.4f m², residence time = %.1f s\n\n",
           tank.cross_area, surge_tank_residence_time(&tank, 0.1));

    /* Build step response model for DMC */
    integrating_process_t model;
    mpc_model_pure_integrator(&model, K, 1.0, tank.cross_area);
    step_response_t step;
    mpc_step_response_from_model(&step, &model, 30);
    printf("Step response: N=%d coefficients, s1=%.4f, s30=%.4f\n\n",
           step.n_coeffs, step.coeff[0], step.coeff[29]);

    /* Configure MPC for surge (flow smoothing) mode */
    mpc_tuning_t tuning;
    mpc_level_config_t config;
    surge_mpc_config(&tuning, &config, &tank, LEVEL_MODE_SURGE, 0.2);

    printf("MPC Configuration (Surge Mode):\n");
    printf("  N_p = %d, N_c = %d, Ts = %.1f s\n",
           tuning.prediction_horizon, tuning.control_horizon,
           tuning.sampling_time);
    printf("  lambda = %.2f, Q = %.2f, alpha = %.2f\n",
           tuning.move_suppression, tuning.output_weight,
           tuning.reference_trajectory_alpha);
    printf("  Level SP = %.2f m, limits: [%.2f, %.2f]\n",
           config.level_setpoint, config.level_lo_limit, config.level_hi_limit);

    /* Initialize DMC */
    dmc_dynamic_t dyn;
    dmc_init(&dyn, &step, &tuning);

    /* Simulation: 100 steps of closed-loop control */
    int n_steps = 100;
    double t_total = n_steps * tuning.sampling_time;

    /* Inlet flow profile with step disturbance */
    double F_in_hist[100];
    for (int k = 0; k < n_steps; k++) {
        double t = k * tuning.sampling_time;
        if (t < 20.0)       F_in_hist[k] = 0.18;
        else if (t < 50.0)  F_in_hist[k] = 0.30; /* +67% disturbance */
        else if (t < 80.0)  F_in_hist[k] = 0.12; /* -60% disturbance */
        else                F_in_hist[k] = 0.18;
    }

    double F_out_hist[100], level_hist[100], valve_hist[100];
    mpc_kpi_t kpi;

    printf("\nRunning simulation for %.0f seconds (%d steps)...\n",
           t_total, n_steps);

    int ret = surge_mpc_simulation_run(&tank, &tuning, &config,
        &step, &dyn, F_in_hist, n_steps,
        F_out_hist, level_hist, valve_hist, &kpi);

    if (ret == 0) {
        printf("\n--- Results ---\n");
        printf("ISE  = %.4f m²\n", kpi.integral_squared_error);
        printf("IAE  = %.4f m·s\n", kpi.integral_absolute_error);
        printf("ITAE = %.4f m·s²\n", kpi.integral_time_absolute_error);
        printf("Total valve travel = %.1f %%\n", kpi.valve_travel_total);
        printf("MV variance = %.4f %%²\n", kpi.mv_variance);
        printf("Constraint violation = %.1f %%\n", kpi.constraint_violation_pct);
        printf("CpK = %.3f\n", kpi.process_capability_cpk);

        printf("\n--- Level Evolution (every 10 steps) ---\n");
        printf("Time[s]   Level[m]   Valve[%%]   F_in    F_out\n");
        for (int k = 0; k < n_steps; k += 10) {
            printf("%5.0f    %7.3f    %7.1f    %6.3f  %6.3f\n",
                   k * tuning.sampling_time,
                   level_hist[k], valve_hist[k],
                   F_in_hist[k], F_out_hist[k]);
        }
    }

    printf("\n========================================\n");
    printf("  DMC Level Control Demo Complete\n");
    printf("========================================\n\n");

    return 0;
}
