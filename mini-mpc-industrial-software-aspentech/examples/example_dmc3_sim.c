/** @file example_dmc3_sim.c
 * @brief Example: AspenTech DMC3 Closed-Loop Simulation (L7)
 *
 * Simulates a complete AspenTech DMC3 control cycle for a 2x2 process.
 * Demonstrates:
 *   - DCS Read (measurement acquisition)
 *   - Bias Update (feedback correction)
 *   - LP Steady-State Target Calculation
 *   - QP Solve (optimal move computation)
 *   - DCS Write (MV implementation)
 *
 * This mirrors the actual DMC3 execution cycle in a refinery.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "mpc_common.h"
#include "aspen_interface.h"

int main(void) {
    printf("=== Example: AspenTech DMC3 Closed-Loop Simulation ===\n\n");

    /* Create DMC3 configuration */
    mpc_aspen_config_t *cfg = mpc_aspen_config_alloc();
    strcpy(cfg->controller_name, "CrudeUnit_DMC3");
    strcpy(cfg->unit_name, "CDU #1 Preflash");
    cfg->execution_interval_sec = 60.0;
    cfg->sample_time_sec = 1.0;
    cfg->n_mv = 2;
    cfg->n_cv = 2;
    cfg->n_dv = 1;
    cfg->P = 20;
    cfg->M = 3;
    cfg->model_horizon = 60;

    /* Allocate MIMO model */
    cfg->model.n_mv = 2;
    cfg->model.n_cv = 2;
    cfg->model.n_dv = 1;
    cfg->model.model_horizon = 60;
    cfg->model.sample_time_sec = 1.0;
    cfg->model.sub_models = (mpc_step_model_t**)calloc(2, sizeof(mpc_step_model_t*));
    for (int i = 0; i < 2; i++)
        cfg->model.sub_models[i] = (mpc_step_model_t*)calloc(2, sizeof(mpc_step_model_t));

    /* Setup models: furnace pass outlet temp (CV1) from fuel gas (MV1) */
    mpc_step_model_t m11; memset(&m11, 0, sizeof(m11));
    m11.n_coeffs = cfg->model_horizon;
    m11.coeff = (double*)calloc(cfg->model_horizon, sizeof(double));
    mpc_step_model_from_fopdt(&m11, 0.5, 15.0, 5.0, 1.0);
    memcpy(&cfg->model.sub_models[0][0], &m11, sizeof(mpc_step_model_t));

    mpc_step_model_t m12; memset(&m12, 0, sizeof(m12));
    m12.n_coeffs = cfg->model_horizon;
    m12.coeff = (double*)calloc(cfg->model_horizon, sizeof(double));
    mpc_step_model_from_fopdt(&m12, 0.1, 20.0, 8.0, 1.0);
    memcpy(&cfg->model.sub_models[0][1], &m12, sizeof(mpc_step_model_t));

    mpc_step_model_t m21; memset(&m21, 0, sizeof(m21));
    m21.n_coeffs = cfg->model_horizon;
    m21.coeff = (double*)calloc(cfg->model_horizon, sizeof(double));
    mpc_step_model_from_fopdt(&m21, 0.3, 12.0, 3.0, 1.0);
    memcpy(&cfg->model.sub_models[1][0], &m21, sizeof(mpc_step_model_t));

    mpc_step_model_t m22; memset(&m22, 0, sizeof(m22));
    m22.n_coeffs = cfg->model_horizon;
    m22.coeff = (double*)calloc(cfg->model_horizon, sizeof(double));
    mpc_step_model_from_fopdt(&m22, 0.8, 10.0, 2.0, 1.0);
    memcpy(&cfg->model.sub_models[1][1], &m22, sizeof(mpc_step_model_t));

    /* Configure MVs and CVs */
    cfg->mv_config = (mpc_mv_config_t*)calloc(2, sizeof(mpc_mv_config_t));
    cfg->cv_config = (mpc_cv_config_t*)calloc(2, sizeof(mpc_cv_config_t));
    cfg->dv_config = (mpc_dv_config_t*)calloc(1, sizeof(mpc_dv_config_t));

    cfg->mv_config[0].is_enabled = 1;
    cfg->mv_config[0].is_optimizing = 1;
    cfg->mv_config[0].eu0 = 0.0;
    cfg->mv_config[0].eu100 = 100.0;
    cfg->mv_config[0].lo_limit = 0.0;
    cfg->mv_config[0].hi_limit = 100.0;
    cfg->mv_config[0].rate_lo = -5.0;
    cfg->mv_config[0].rate_hi = 5.0;
    cfg->mv_config[0].current_value = 50.0;
    cfg->mv_config[0].lp_cost = 1.0;
    strcpy(cfg->mv_config[0].name, "FuelGas_Valve");
    strcpy(cfg->mv_config[0].eu, "%");

    cfg->mv_config[1] = cfg->mv_config[0];
    cfg->mv_config[1].current_value = 40.0;
    cfg->mv_config[1].lp_cost = 2.0;
    strcpy(cfg->mv_config[1].name, "Air_Valve");

    cfg->cv_config[0].is_enabled = 1;
    cfg->cv_config[0].is_controlled = 1;
    cfg->cv_config[0].eu0 = 300.0;
    cfg->cv_config[0].eu100 = 500.0;
    cfg->cv_config[0].lo_limit = 350.0;
    cfg->cv_config[0].hi_limit = 450.0;
    cfg->cv_config[0].setpoint = 400.0;
    cfg->cv_config[0].current_value = 400.0;
    cfg->cv_config[0].constraint_type = MPC_CONSTRAINT_BOTH;
    cfg->cv_config[0].constraint_lo = 350.0;
    cfg->cv_config[0].constraint_hi = 450.0;
    strcpy(cfg->cv_config[0].name, "Furnace_Outlet_Temp");
    strcpy(cfg->cv_config[0].eu, "degC");

    cfg->cv_config[1] = cfg->cv_config[0];
    cfg->cv_config[1].setpoint = 350.0;
    cfg->cv_config[1].current_value = 350.0;
    strcpy(cfg->cv_config[1].name, "O2_Analyzer");
    strcpy(cfg->cv_config[1].eu, "%");

    /* Setup weights */
    cfg->weights.n_cv = 2;
    cfg->weights.n_mv = 2;
    cfg->weights.Q = (double*)calloc(2, sizeof(double));
    cfg->weights.R = (double*)calloc(2, sizeof(double));
    cfg->weights.Q[0] = 10.0;
    cfg->weights.Q[1] = 1.0;
    cfg->weights.R[0] = 0.1;
    cfg->weights.R[1] = 0.5;

    /* Run closed-loop simulation */
    int n_steps = 50;
    double *cv_hist = (double*)calloc(n_steps * 2, sizeof(double));
    double *mv_hist = (double*)calloc(n_steps * 2, sizeof(double));

    printf("Running %d-step closed-loop simulation...\n", n_steps);
    int rc = mpc_dmc3_closed_loop_simulation(cfg, n_steps, 0.1, cv_hist, mv_hist);

    if (rc == 0) {
        printf("\nSimulation Results (final 5 steps):\n");
        printf("Step |  CV1 (Furnace T) |  CV2 (O2)    |  MV1 (Fuel) |  MV2 (Air)\n");
        printf("-----|------------------|--------------|-------------|------------\n");
        for (int i = n_steps - 5; i < n_steps; i++) {
            printf(" %3d | %14.2f | %12.2f | %11.2f | %10.2f\n",
                   i, cv_hist[i*2], cv_hist[i*2+1], mv_hist[i*2], mv_hist[i*2+1]);
        }
    } else {
        printf("Simulation FAILED with code %d\n", rc);
    }

    /* Cleanup */
    free(cv_hist); free(mv_hist);
    free(cfg->weights.Q); free(cfg->weights.R);
    free(cfg->mv_config); free(cfg->cv_config); free(cfg->dv_config);
    free(m11.coeff); free(m12.coeff); free(m21.coeff); free(m22.coeff);
    for (int i = 0; i < 2; i++) free(cfg->model.sub_models[i]);
    free(cfg->model.sub_models);
    mpc_aspen_config_free(cfg);

    printf("\n=== Example complete ===\n");
    return 0;
}
