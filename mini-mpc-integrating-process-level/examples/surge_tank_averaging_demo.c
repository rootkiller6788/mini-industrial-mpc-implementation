/**
 * surge_tank_averaging_demo.c — Surge Tank Level Averaging with MPC
 *
 * Demonstrates the key trade-off in surge tank operation: flow smoothing
 * vs. level variation. Compares three MPC tuning modes (tight, averaging,
 * surge) on the same tank subjected to sinusoidal inlet flow disturbances.
 *
 * Industrial Context:
 *   In refineries and chemical plants, surge tanks between distillation
 *   columns buffer flow fluctuations. The outlet flow to the downstream
 *   column should be as steady as possible (good for column stability),
 *   while the tank level is allowed to vary within safe bounds.
 *
 * This example maps to:
 *   L6: Surge tank level control (canonical problem)
 *   L7: Refinery distillation surge tank (industrial application)
 *
 * Reference: Luyben (2007) "Chemical Reactor Design and Control" §8
 *            Shinskey (1996) "Process Control Systems" §10
 *            McDonald & McAvoy (1986) "DMC for distillation columns"
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "../include/mpc_level_types.h"
#include "../include/mpc_integrating_model.h"
#include "../include/mpc_dmc.h"
#include "../include/mpc_surge_tank.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static void simulate_mode(const char *label, level_control_mode_t mode,
                           const surge_tank_t *base_tank,
                           const double *F_in_hist, int n_steps,
                           double *F_out_var, double *level_var) {
    surge_tank_t tank = *base_tank;
    mpc_tuning_t tuning;
    mpc_level_config_t config;
    surge_mpc_config(&tuning, &config, &tank, mode,
                     F_in_hist[0]);

    integrating_process_t model;
    double K = mpc_tank_area_to_gain(tank.cross_area,
                                      tank.valve_coeff, tank.level);
    mpc_model_pure_integrator(&model, K, tuning.sampling_time,
                               tank.cross_area);

    step_response_t step;
    mpc_step_response_from_model(&step, &model, tuning.prediction_horizon);

    dmc_dynamic_t dyn;
    dmc_init(&dyn, &step, &tuning);

    double *F_out = malloc((size_t)n_steps * sizeof(double));
    double *level = malloc((size_t)n_steps * sizeof(double));
    double *valve = malloc((size_t)n_steps * sizeof(double));
    mpc_kpi_t kpi;

    surge_mpc_simulation_run(&tank, &tuning, &config,
        &step, &dyn, F_in_hist, n_steps,
        F_out, level, valve, &kpi);

    /* Compute variance statistics */
    double F_out_mean = 0.0, F_out_sum2 = 0.0;
    double level_mean = 0.0, level_sum2 = 0.0;
    for (int k = 0; k < n_steps; k++) {
        F_out_mean += F_out[k];
        level_mean += level[k];
    }
    F_out_mean /= n_steps;
    level_mean /= n_steps;
    for (int k = 0; k < n_steps; k++) {
        double dF = F_out[k] - F_out_mean;
        double dh = level[k] - level_mean;
        F_out_sum2 += dF * dF;
        level_sum2 += dh * dh;
    }
    *F_out_var = sqrt(F_out_sum2 / n_steps);   /* std dev of outflow */
    *level_var = sqrt(level_sum2 / n_steps);   /* std dev of level */

    printf("%-12s  %7.2f   %7.4f    %7.4f    %9.1f   %9.1f\n",
           label, kpi.valve_travel_total,
           *F_out_var, *level_var,
           kpi.constraint_violation_pct,
           kpi.integral_absolute_error);

    free(F_out);
    free(level);
    free(valve);
}

int main(void) {
    printf("\n========================================\n");
    printf("  SURGE TANK LEVEL AVERAGING WITH MPC\n");
    printf("  Flow Smoothing vs. Level Variation\n");
    printf("========================================\n\n");

    /* Tank: D=3m, H=4m, A≈7.07 m², 10 m³ nominal */
    surge_tank_t tank;
    surge_tank_init(&tank, 3.0, 4.0, 2.0, 0.15);

    printf("Tank Specifications:\n");
    printf("  Diameter = %.1f m\n", tank.diameter);
    printf("  Height = %.1f m\n", tank.max_level);
    printf("  Cross-sectional area = %.3f m²\n", tank.cross_area);
    printf("  Initial level = %.1f m (50%%)\n", tank.level);
    printf("  Valve Cv = %.3f\n\n", tank.valve_coeff);

    /* Generate sinusoidal inlet flow disturbance:
     * F_in(t) = 0.2 + 0.08*sin(2π*t/180)  [m³/s]
     * Period = 180 s (3 minutes), amplitude = ±40% */
    int n_steps = 200;
    double Ts_base = 1.0;
    double *F_in = malloc((size_t)n_steps * sizeof(double));

    for (int k = 0; k < n_steps; k++) {
        double t = k * Ts_base;
        F_in[k] = 0.20 + 0.08 * sin(2.0 * M_PI * t / 180.0);
    }

    printf("Inlet Flow Disturbance:\n");
    printf("  Mean = 0.20 m³/s, Amplitude = ±0.08 m³/s\n");
    printf("  Period = 180 s (3 minutes)\n\n");

    /* Estimate residence time */
    double F_out_nom = tank.valve_coeff * sqrt(tank.level) * 0.50;
    double tau_res = tank.cross_area * tank.level / F_out_nom;
    printf("Nominal residence time = %.0f s (%.1f min)\n\n",
           tau_res, tau_res / 60.0);

    /* Compute optimal phi */
    double omega = 2.0 * M_PI / 180.0;  /* rad/s */
    double phi_opt = surge_level_filter_factor(omega, tau_res);
    printf("Optimal filter factor φ = %.3f\n", phi_opt);
    printf("  φ=1.0: tight level, no flow smoothing\n");
    printf("  φ=0.0: pure averaging, max flow smoothing\n\n");

    double ampl;
    double A_f_max = surge_capacity_utilization(&tank, omega, phi_opt, &ampl);
    printf("Capacity analysis at φ=%.3f:\n", phi_opt);
    printf("  Max handleable amplitude = %.4f m³/s\n", A_f_max);
    printf("  Expected level amplitude = %.3f m\n\n", ampl);

    /* Minimum tank sizing */
    double V_min = surge_tank_sizing(0.20, 180.0, phi_opt, 0.3);
    printf("Minimum tank volume at φ=%.3f: %.2f m³\n", phi_opt, V_min);
    printf("Actual tank volume: %.2f m³\n\n",
           tank.cross_area * tank.max_level);

    /* Run three modes */
    printf("Comparison of MPC Level Control Modes:\n");
    printf("----------------------------------------");
    printf("----------------------------------------\n");
    printf("%-12s  %7s   %7s    %7s    %9s   %9s\n",
           "Mode", "Travel%%", "F_std", "h_std",
           "Viol%%", "IAE");
    printf("----------------------------------------");
    printf("----------------------------------------\n");

    double F_var, h_var;

    simulate_mode("TIGHT", LEVEL_MODE_TIGHT, &tank,
                  F_in, n_steps, &F_var, &h_var);
    simulate_mode("AVERAGING", LEVEL_MODE_AVERAGING, &tank,
                  F_in, n_steps, &F_var, &h_var);
    simulate_mode("SURGE", LEVEL_MODE_SURGE, &tank,
                  F_in, n_steps, &F_var, &h_var);

    printf("----------------------------------------");
    printf("----------------------------------------\n\n");

    printf("Interpretation:\n");
    printf("  - TIGHT mode: Low level variance, high flow variability.\n");
    printf("    Best for processes where level precision matters.\n");
    printf("  - AVERAGING mode: Balanced. Moderate level + flow variation.\n");
    printf("    Standard industrial default for surge tanks.\n");
    printf("  - SURGE mode: Low flow variability, high level variation.\n");
    printf("    Best for protecting downstream unit stability.\n\n");

    printf("========================================\n");
    printf("  Surge Tank Averaging Demo Complete\n");
    printf("========================================\n\n");

    free(F_in);
    return 0;
}
