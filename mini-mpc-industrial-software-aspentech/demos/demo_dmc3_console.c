/** @file demo_dmc3_console.c
 * @brief Demo: AspenTech DMC3 Console Emulator for Operator Training
 *
 * Interactive console demo simulating an operator station for DMC3.
 * Displays current CV/MV values, constraint status, and allows
 * manual mode entry for operator override testing.
 *
 * This demonstrates the L7 industrial application of DMC3 in
 * a refinery control room context.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "mpc_common.h"
#include "aspen_interface.h"

static void print_header(void)
{
    printf("\n");
    printf("============================================================\n");
    printf("  AspenTech DMC3 Console — Operator Training Demo\n");
    printf("============================================================\n\n");
}

static void print_controller_state(mpc_aspen_config_t *cfg, int step, double *cv, double *mv)
{
    printf("--- Step %3d -------------------------------------------------\n", step);
    printf("  %-15s %10s %10s %10s %10s\n", "Tag", "Value", "Setpoint", "Lo Limit", "Hi Limit");
    printf("  %-15s %10s %10s %10s %10s\n", "---", "-----", "--------", "--------", "--------");

    for (int i = 0; i < cfg->n_cv; i++) {
        char status = ' ';
        if (cv[i] < cfg->cv_config[i].lo_limit) status = 'L';
        else if (cv[i] > cfg->cv_config[i].hi_limit) status = 'H';
        else if (fabs(cv[i] - cfg->cv_config[i].setpoint) < cfg->cv_config[i].deadband) status = '*';

        printf("%c %-14s %10.2f %10.2f %10.2f %10.2f\n",
            status, cfg->cv_config[i].name, cv[i], cfg->cv_config[i].setpoint,
            cfg->cv_config[i].lo_limit, cfg->cv_config[i].hi_limit);
    }

    printf("\n  %-15s %10s %10s %10s %10s\n", "MV", "Value", "Rate Lo", "Rate Hi", "Util%%");
    for (int i = 0; i < cfg->n_mv; i++) {
        double util = mpc_compute_mv_utilization(&cfg->mv_config[i], &mv[i]);
        printf("  %-15s %10.2f %10.2f %10.2f %9.1f%%\n",
            cfg->mv_config[i].name, mv[i],
            cfg->mv_config[i].rate_lo, cfg->mv_config[i].rate_hi, util);
    }
    printf("\n");
}

int main(void)
{
    print_header();

    /* Configure a simple 2x2 DMC3 controller */
    mpc_aspen_config_t *cfg = mpc_aspen_config_alloc();
    cfg->n_mv = 2; cfg->n_cv = 2; cfg->n_dv = 0;
    cfg->P = 20; cfg->M = 3; cfg->model_horizon = 40;
    cfg->bias_filter_gain = 0.4;

    /* Configure MVs */
    cfg->mv_config = (mpc_mv_config_t*)calloc(2, sizeof(mpc_mv_config_t));
    strcpy(cfg->mv_config[0].name, "FUEL_GAS");  strcpy(cfg->mv_config[0].eu, "%");
    cfg->mv_config[0].eu0 = 0; cfg->mv_config[0].eu100 = 100;
    cfg->mv_config[0].lo_limit = 10; cfg->mv_config[0].hi_limit = 90;
    cfg->mv_config[0].rate_lo = -5; cfg->mv_config[0].rate_hi = 5;
    cfg->mv_config[0].current_value = 50; cfg->mv_config[0].is_enabled = 1;

    strcpy(cfg->mv_config[1].name, "AIR_FLOW");  strcpy(cfg->mv_config[1].eu, "kg/h");
    cfg->mv_config[1].eu0 = 0; cfg->mv_config[1].eu100 = 1000;
    cfg->mv_config[1].lo_limit = 100; cfg->mv_config[1].hi_limit = 900;
    cfg->mv_config[1].rate_lo = -50; cfg->mv_config[1].rate_hi = 50;
    cfg->mv_config[1].current_value = 500; cfg->mv_config[1].is_enabled = 1;

    /* Configure CVs */
    cfg->cv_config = (mpc_cv_config_t*)calloc(2, sizeof(mpc_cv_config_t));
    strcpy(cfg->cv_config[0].name, "FURNACE_TEMP"); strcpy(cfg->cv_config[0].eu, "degC");
    cfg->cv_config[0].eu0 = 0; cfg->cv_config[0].eu100 = 1000;
    cfg->cv_config[0].setpoint = 750; cfg->cv_config[0].deadband = 5;
    cfg->cv_config[0].lo_limit = 700; cfg->cv_config[0].hi_limit = 800;
    cfg->cv_config[0].current_value = 720; cfg->cv_config[0].is_enabled = 1;
    cfg->cv_config[0].is_controlled = 1;

    strcpy(cfg->cv_config[1].name, "O2_EXCESS"); strcpy(cfg->cv_config[1].eu, "%");
    cfg->cv_config[1].eu0 = 0; cfg->cv_config[1].eu100 = 21;
    cfg->cv_config[1].setpoint = 3; cfg->cv_config[1].deadband = 0.5;
    cfg->cv_config[1].lo_limit = 1; cfg->cv_config[1].hi_limit = 6;
    cfg->cv_config[1].current_value = 5; cfg->cv_config[1].is_enabled = 1;
    cfg->cv_config[1].is_controlled = 1;

    /* Build model */
    cfg->model.n_mv = 2; cfg->model.n_cv = 2; cfg->model.n_dv = 0;
    cfg->model.model_horizon = 40;
    cfg->model.sub_models = (mpc_step_model_t**)calloc(2, sizeof(mpc_step_model_t*));
    for (int cv = 0; cv < 2; cv++) {
        cfg->model.sub_models[cv] = (mpc_step_model_t*)calloc(2, sizeof(mpc_step_model_t));
        for (int mv = 0; mv < 2; mv++) {
            cfg->model.sub_models[cv][mv].n_coeffs = 40;
            cfg->model.sub_models[cv][mv].coeff = (double*)calloc(40, sizeof(double));
            cfg->model.sub_models[cv][mv].sample_time_sec = 1.0;
        }
    }

    /* FOPDT: Temp = 200/(600s+1) * e^{-120s} * Fuel + 80/(400s+1) * Air */
    for (int i = 0; i < 40; i++) {
        double t = i + 1;
        double tau = 600.0, theta = 120.0, K = 200.0;
        cfg->model.sub_models[0][0].coeff[i] = (t > theta/1.0) ? K*(1.0-exp(-(t-theta/1.0)/tau)) : 0.0;
        tau = 400.0; K = 80.0;
        cfg->model.sub_models[0][1].coeff[i] = (t > 0) ? K*(1.0-exp(-t/tau)) : 0.0;
        tau = 300.0; theta = 60.0; K = -2.0;
        cfg->model.sub_models[1][0].coeff[i] = (t > theta/1.0) ? K*(1.0-exp(-(t-theta/1.0)/tau)) : 0.0;
        tau = 200.0; K = 1.5;
        cfg->model.sub_models[1][1].coeff[i] = (t > 0) ? K*(1.0-exp(-t/tau)) : 0.0;
    }

    cfg->weights.n_cv = 2; cfg->weights.n_mv = 2;
    cfg->weights.Q = (double*)calloc(2, sizeof(double));
    cfg->weights.R = (double*)calloc(2, sizeof(double));
    cfg->weights.S = (double*)calloc(2, sizeof(double));
    cfg->weights.Q[0] = 10.0; cfg->weights.Q[1] = 2.0;
    cfg->weights.R[0] = 1.0;  cfg->weights.R[1] = 0.5;

    /* Run simulation */
    double cv_hist[400], mv_hist[400];
    mpc_dmc3_closed_loop_simulation(cfg, 100, 2.0, cv_hist, mv_hist);

    /* Display every 10th step */
    for (int step = 0; step <= 100; step += 10) {
        double cv[2] = {cv_hist[step*2], cv_hist[step*2+1]};
        double mv[2] = {mv_hist[step*2], mv_hist[step*2+1]};
        print_controller_state(cfg, step, cv, mv);
    }

    printf("=== Demo Complete: 100-step DMC3 closed-loop simulation ===\n");
    mpc_aspen_config_free(cfg);
    return 0;
}
