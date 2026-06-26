/**
 * mpc_surge_visual.c — ASCII Visualization of Surge Tank MPC
 *
 * Simple terminal-based visualization of surge tank level under MPC.
 * Shows tank level as a vertical bar chart updated each sampling period.
 * Useful for operator training and control room familiarization.
 *
 * Knowledge: L7 - Industrial Applications (operator interface visualization)
 * Reference: ISA-101 HMI standard for process visualization
 */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "../include/mpc_level_types.h"
#include "../include/mpc_integrating_model.h"
#include "../include/mpc_dmc.h"
#include "../include/mpc_surge_tank.h"

#define TANK_HEIGHT_CHARS 20

static void draw_tank(double level, double max_level, double sp,
                       double lo_lim, double hi_lim, double valve) {
    int level_bar = (int)(level / max_level * TANK_HEIGHT_CHARS);
    int sp_bar    = (int)(sp / max_level * TANK_HEIGHT_CHARS);
    int hi_bar    = (int)(hi_lim / max_level * TANK_HEIGHT_CHARS);
    int lo_bar    = (int)(lo_lim / max_level * TANK_HEIGHT_CHARS);

    printf("  +----+  Level: %6.2f m  SP: %6.2f m  Valve: %5.1f%%\n",
           level, sp, valve);
    for (int i = TANK_HEIGHT_CHARS - 1; i >= 0; i--) {
        printf("  |");
        if (i == hi_bar && hi_bar > 0) printf("--");
        else if (i == lo_bar && lo_bar > 0) printf("--");
        else printf("  ");
        if (i < level_bar) printf("██");
        else printf("  ");
        printf("|");
        if (i == sp_bar) printf(" < SP");
        if (i == hi_bar) printf(" < HI");
        if (i == lo_bar) printf(" < LO");
        printf("\n");
    }
    printf("  +----+\n");
}

int main(void) {
    surge_tank_t tank;
    surge_tank_init(&tank, 2.0, 3.0, 1.5, 0.1);

    mpc_tuning_t tuning;
    mpc_level_config_t config;
    surge_mpc_config(&tuning, &config, &tank, LEVEL_MODE_AVERAGING, 0.2);

    double K = mpc_tank_area_to_gain(tank.cross_area,
                                      tank.valve_coeff, tank.level);
    integrating_process_t model;
    mpc_model_pure_integrator(&model, K, tuning.sampling_time,
                               tank.cross_area);
    step_response_t step;
    mpc_step_response_from_model(&step, &model, tuning.prediction_horizon);
    dmc_dynamic_t dyn;
    dmc_init(&dyn, &step, &tuning);

    printf("\nSURGE TANK MPC — LIVE VISUALIZATION\n");
    printf("Disturbance: Inlet flow steps at t=20s (+50%%) and t=60s (-50%%)\n\n");

    int n_steps = 100;
    double F_in = 0.18;
    double u_prev = 50.0;

    for (int k = 0; k < n_steps; k++) {
        double t = k * tuning.sampling_time;
        if (k == 20) F_in = 0.27;
        if (k == 60) F_in = 0.09;

        mpc_solution_t sol;
        memset(&sol, 0, sizeof(sol));
        double du = dmc_step(&sol, &dyn, &step, &tuning, &config,
                              tank.level, u_prev, MPC_SOLVER_HILDRETH);
        u_prev += du;
        if (u_prev < 5.0) u_prev = 5.0;
        if (u_prev > 95.0) u_prev = 95.0;

        double F_out = surge_tank_outflow(&tank, u_prev);
        surge_tank_simulate(&tank, F_in, F_out, tuning.sampling_time);

        if (k % 5 == 0) {
            printf("\033[2J\033[H"); /* clear screen */
            printf("Time: %.0f s  |  F_in: %.3f  F_out: %.3f\n\n",
                   t, F_in, F_out);
            draw_tank(tank.level, tank.max_level,
                      config.level_setpoint,
                      config.level_lo_limit, config.level_hi_limit,
                      u_prev);
        }
    }

    printf("\nFinal level: %.3f m (SP: %.2f m)\n",
           tank.level, config.level_setpoint);
    return 0;
}
