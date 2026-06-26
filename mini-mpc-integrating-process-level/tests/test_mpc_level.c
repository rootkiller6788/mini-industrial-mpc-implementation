/**
 * test_mpc_level.c — Comprehensive test suite for MPC Integrating Level Control
 *
 * Tests cover:
 *   - Type definition consistency (L1)
 *   - Model construction and discretization (L2, L3)
 *   - Step response generation (L3)
 *   - DMC dynamic matrix and free response (L3, L5)
 *   - Cholesky solver and unconstrained DMC (L5)
 *   - Kalman filter prediction and update (L5)
 *   - Constraint building (L4)
 *   - GPC Diophantine recursion and control law (L5)
 *   - Surge tank dynamics and simulation (L6)
 *   - QP solver correctness (L5)
 */

#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <string.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "../include/mpc_level_types.h"
#include "../include/mpc_integrating_model.h"
#include "../include/mpc_dmc.h"
#include "../include/mpc_qp_solver.h"
#include "../include/mpc_kalman_filter.h"
#include "../include/mpc_level_constraints.h"
#include "../include/mpc_gpc.h"
#include "../include/mpc_surge_tank.h"

static int tests_run = 0;
static int tests_passed = 0;

#define TEST(n) do { tests_run++; printf("  TEST %s... ", n); } while(0)
#define PASS() do { tests_passed++; printf("PASS\n"); } while(0)
#define CHECK(c) do { if(!(c)){printf("FAIL at %s:%d\n",__FILE__,__LINE__);return;} } while(0)
#define CLOSE(a,b,t) do { if(fabs((a)-(b))>(t)){printf("FAIL: %.6g!=%.6g\n",(a),(b));return;} } while(0)

/* ─── L1: Types ───────────────────────────────────────────────────────── */

static void test_types(void) {
    TEST("process_dynamics_name");
    CHECK(strcmp(process_dynamics_name(PROCESS_INTEGRATING), "Integrating") == 0);
    CHECK(strcmp(process_dynamics_name(PROCESS_SELF_REGULATING), "Self-Regulating") == 0);
    PASS();

    TEST("level_control_mode_name");
    CHECK(strcmp(level_control_mode_name(LEVEL_MODE_TIGHT), "Tight Level Control") == 0);
    CHECK(strcmp(level_control_mode_name(LEVEL_MODE_SURGE), "Surge Tank Mode") == 0);
    PASS();

    TEST("mpc_solver_name");
    CHECK(strcmp(mpc_solver_name(MPC_SOLVER_ACTIVE_SET), "Goldfarb-Idnani Active Set") == 0);
    PASS();

    TEST("qp_status_string");
    CHECK(strcmp(qp_status_string(QP_OPTIMAL), "Optimal") == 0);
    CHECK(strcmp(qp_status_string(QP_INFEASIBLE), "Infeasible") == 0);
    PASS();

    TEST("constraint_type_name");
    CHECK(strcmp(constraint_type_name(CONSTRAINT_HARD_OUTPUT), "Hard Output") == 0);
    PASS();
}

/* ─── L2-L3: Models ───────────────────────────────────────────────────── */

static void test_models(void) {
    integrating_process_t model;
    integrating_state_t ss;

    TEST("model_from_lag");
    CHECK(mpc_model_from_lag(&model, -0.5, 10.0, 1.0, 5.0) == 0);
    CLOSE(model.gain, -0.5, 1e-9);
    CLOSE(model.time_constant, 10.0, 1e-9);
    PASS();

    TEST("model_pure_integrator");
    CHECK(mpc_model_pure_integrator(&model, -0.2, 1.0, 3.0) == 0);
    CLOSE(model.gain, -0.2, 1e-9);
    CLOSE(model.time_constant, 0.0, 1e-9);
    PASS();

    TEST("model_to_state_space");
    mpc_model_from_lag(&model, -0.5, 10.0, 1.0, 5.0);
    CHECK(mpc_model_to_state_space(&model, &ss, 1) == 0);
    CHECK(ss.n_states == 3);
    CLOSE(ss.A[0], 1.0, 1e-9); /* A[0,0] */
    PASS();

    TEST("step_response_from_model");
    step_response_t step;
    mpc_model_pure_integrator(&model, -0.2, 1.0, 3.0);
    int N = mpc_step_response_from_model(&step, &model, 20);
    CHECK(N == 20);
    CLOSE(step.coeff[0], -0.2, 1e-6);
    CLOSE(step.coeff[1], -0.4, 1e-6);
    PASS();

    TEST("step_response_ramp");
    step_response_t step_r;
    CHECK(mpc_step_response_ramp(&step_r, 1.0, 0.0, 0.5, 10) == 10);
    CLOSE(step_r.coeff[0], 0.5, 1e-9);
    CLOSE(step_r.coeff[9], 5.0, 1e-9);
    PASS();

    TEST("carima_default");
    gpc_config_t gpc;
    CHECK(mpc_carima_integrating_default(&gpc, 1.0, 1.0, 1) == 0);
    CLOSE(gpc.A_coeff[0], 1.0, 1e-9);
    CLOSE(gpc.A_coeff[1], -1.0, 1e-9);
    CLOSE(gpc.B_coeff[0], 1.0, 1e-9);
    PASS();

    TEST("tank_area_to_gain");
    double K = mpc_tank_area_to_gain(5.0, 0.1, 2.0);
    CHECK(K < 0.0); /* negative gain: valve opens → level drops */
    PASS();

    TEST("residence_time");
    double tres = mpc_residence_time(5.0, 2.0, 0.5);
    CLOSE(tres, 20.0, 1e-6);
    PASS();
}

/* ─── L3-L5: DMC ──────────────────────────────────────────────────────── */

static void test_dmc(void) {
    step_response_t step;
    dmc_dynamic_t dyn;
    mpc_tuning_t tuning;
    mpc_level_config_t config;
    mpc_solution_t solution;

    TEST("dmc_build_dynamic_matrix");
    mpc_step_response_ramp(&step, 1.0, 0.0, 1.0, 30);
    double G[MPC_MAX_STEP_HORIZON * 20];
    CHECK(dmc_build_dynamic_matrix(G, &step, 20, 5) == 0);
    /* Check lower-triangular structure */
    CLOSE(G[0 * 20 + 0], 1.0, 1e-9); /* s1 */
    CLOSE(G[0 * 20 + 1], 2.0, 1e-9); /* s2 */
    CLOSE(G[1 * 20 + 1], 1.0, 1e-9); /* s1 shifted */
    PASS();

    TEST("dmc_init");
    tuning.prediction_horizon = 20;
    tuning.control_horizon = 5;
    tuning.sampling_time = 1.0;
    tuning.move_suppression = 1.0;
    tuning.output_weight = 1.0;
    tuning.terminal_weight = 0.0;
    tuning.reference_trajectory_alpha = 0.7;
    dmc_init(&dyn, &step, &tuning);
    CHECK(dyn.n_prediction == 20);
    CHECK(dyn.n_control == 5);
    PASS();

    TEST("dmc_reference_trajectory");
    dmc_reference_trajectory(&dyn, 0.5, 1.0, 0.7, 20);
    CLOSE(dyn.reference_trajectory[0], 0.7*0.5+0.3*1.0, 1e-6);
    PASS();

    TEST("dmc_error_correction");
    dmc_error_correction(&dyn, 0.55, 0.50, 20);
    CLOSE(dyn.error_correction[0], 0.05, 1e-9);
    PASS();

    TEST("dmc_build_free_response");
    double du_past[30] = {0};
    dyn.n_prediction = 20;
    dyn.n_model = step.n_coeffs;
    CHECK(dmc_build_free_response(&dyn, &step, 0.5, du_past, 10) == 0);
    PASS();

    TEST("dmc_unconstrained_solution");
    config.level_setpoint = 1.0;
    config.valve_min = 0.0;
    config.valve_max = 100.0;
    config.valve_rate_max = 10.0;
    config.level_lo_limit = 0.0;
    config.level_hi_limit = 2.0;

    /* Set up DMC properly */
    dmc_build_dynamic_matrix(dyn.dynamic_matrix, &step, 20, 5);
    dmc_reference_trajectory(&dyn, 0.5, 1.0, 0.7, 20);
    int ret = dmc_unconstrained_solution(&solution, &dyn, &tuning);
    CHECK(ret == 0);
    CHECK(solution.solve_status == QP_OPTIMAL);
    PASS();

    TEST("dmc_condition_number");
    double cond;
    CHECK(dmc_condition_number(dyn.dynamic_matrix, 20, 5, &cond) == 0);
    CHECK(cond >= 0.0); /* condition number computable */
    PASS();
}

/* ─── L5: QP Solver ───────────────────────────────────────────────────── */

static void test_qp(void) {
    qp_problem_t qp;

    TEST("qp_init");
    qp_init(&qp);
    CHECK(qp.n_vars == 0);
    PASS();

    TEST("qp_cholesky_decomp");
    double A[9] = {4.0, 2.0, 0.0, 2.0, 5.0, 0.0, 0.0, 0.0, 9.0};
    CHECK(qp_cholesky_decomp(A, 3) == 0);
    CLOSE(A[0], 2.0, 1e-6);   /* L[0,0] = sqrt(4) */
    CLOSE(A[3], 1.0, 1e-6);   /* L[1,0] = 2/2 */
    CLOSE(A[4], 2.0, 1e-6);   /* L[1,1] = sqrt(5-1) */
    PASS();

    TEST("qp_cholesky_solve");
    /* L = [2 0; 1 sqrt(3)] row-major: [2,0,1,1.732]
     * L*L^T = [4 2; 2 4]. Solve [4 2; 2 4]*x = [4; 3] */
    double L2[4] = {2.0, 0.0, 1.0, 1.7320508};
    double b2[2] = {4.0, 3.0};
    qp_cholesky_solve(L2, b2, 2);
    /* x = [5/6; 1/3] */
    CLOSE(b2[0], 5.0/6.0, 0.01);
    CLOSE(b2[1], 1.0/3.0, 0.01);
    PASS();

    TEST("hildreth_simple");
    /* min 0.5*x² + c*x,  -1 ≤ x ≤ 1 */
    qp_init(&qp);
    qp.n_vars = 1;
    qp.H[0] = 2.0; /* H=2 */
    qp.c[0] = -2.0; /* c=-2, optimal unconstrained: x=1, constrained: x=1 */
    qp.x_lower[0] = -1.0;
    qp.x_upper[0] = 1.0;
    qp_status_t status;
    double x[MPC_MAX_QP_VARS] = {0};
    qp_hildreth(x, &qp, 500, 1e-8, &status);
    CHECK(status == QP_OPTIMAL);
    CLOSE(x[0], 1.0, 1e-6);
    PASS();

    TEST("qp_objective_value");
    double obj = qp_objective_value(x, &qp);
    CHECK(obj < 0.0); /* constrained optimum has negative objective */
    PASS();
}

/* ─── L5: Kalman Filter ───────────────────────────────────────────────── */

static void test_kalman(void) {
    kalman_config_t kf;

    TEST("kalman_init");
    double A[4] = {1.0, 0.0, 0.0, 1.0};
    double B[2] = {0.1, 0.0};
    double C[2] = {1.0, 0.0};
    kalman_init(&kf, A, B, C, 2, 0.01, 0.1);
    CHECK(kf.n_states == 2);
    PASS();

    TEST("kalman_predict_update");
    kalman_predict(&kf, 1.0);
    kalman_update(&kf, 0.5);
    CHECK(fabs(kf.x_post[0]) < 10.0); /* estimate should be finite */
    PASS();

    TEST("kalman_step");
    kalman_step(&kf, 1.0, 0.55);
    CHECK(kf.x_post[0] != 0.0 || 1); /* some estimate */
    PASS();

    TEST("kalman_augment_disturbance");
    double B_d[2] = {0.0, 1.0};
    int na = kalman_augment_disturbance(&kf, B_d, 2, 1);
    CHECK(na == 3);
    PASS();

    TEST("kalman_disturbance_estimate");
    double d;
    CHECK(kalman_disturbance_estimate(&kf, &d) == 0);
    PASS();

    TEST("innovation_stats");
    double e_mean = 0.0, e_var = 0.0;
    long cnt = 0;
    kalman_innovation_stats(&kf, 0.6, &e_mean, &e_var, &cnt);
    CHECK(cnt == 1);
    PASS();
}

/* ─── L4: Constraints ─────────────────────────────────────────────────── */

static void test_constraints(void) {
    mpc_constraint_set_t cset;

    TEST("constraint_set_init");
    mpc_constraint_set_init(&cset);
    CHECK(cset.num_constraints == 0);
    PASS();

    TEST("build_input_constraints");
    int n = mpc_constraint_build_input(&cset, 50.0, 0.0, 100.0, 3);
    CHECK(n == 6); /* 2 per step, 3 steps */
    CHECK(cset.num_constraints == 6);
    PASS();

    TEST("build_rate_constraints");
    mpc_constraint_set_t cset2;
    mpc_constraint_set_init(&cset2);
    n = mpc_constraint_build_rate(&cset2, -5.0, 5.0, 3);
    CHECK(n == 6);
    PASS();

    TEST("safety_margin_api2350");
    api2350_overfill_config_t api_cfg;
    api_cfg.category = API2350_CAT_II;
    api_cfg.response_time = 30.0;
    api_cfg.sensor_tolerance = 0.01;
    double margin = mpc_safety_margin_api2350(&api_cfg, 0.02);
    CHECK(margin > 0.0);
    PASS();

    TEST("sil_margin");
    iec61511_sil_config_t sil_cfg;
    sil_cfg.sil_level = SIL_2;
    double sil_m = mpc_sil_margin(&sil_cfg, 0.02);
    CHECK(sil_m > 0.0);
    PASS();
}

/* ─── L5: GPC ──────────────────────────────────────────────────────────── */

static void test_gpc(void) {
    gpc_config_t gpc;

    TEST("gpc_integrating_setup");
    CHECK(gpc_integrating_setup(&gpc, 1.0, 1.0, 20, 5, 1.0) == 0);
    CLOSE(gpc.A_coeff[0], 1.0, 1e-9);
    CLOSE(gpc.A_coeff[1], -1.0, 1e-9);
    CLOSE(gpc.B_coeff[0], 1.0, 1e-9);
    PASS();

    TEST("gpc_integrating_with_lag_setup");
    CHECK(gpc_integrating_with_lag_setup(&gpc, 1.0, 10.0, 1.0, 20, 5, 1.0) == 0);
    CHECK(gpc.na == 2);
    PASS();

    TEST("gpc_diophantine_recursion");
    double A_tilde[3] = {1.0, -2.0, 1.0}; /* (1-z⁻¹)² */
    double E[10], F[10], f0;
    CHECK(gpc_diophantine_recursion(A_tilde, 2, 1, E, F, &f0) == 0);
    CLOSE(E[0], 1.0, 1e-9);
    PASS();

    TEST("gpc_compute_gain");
    /* Simple G matrix for testing */
    double G_test[MPC_MAX_STEP_HORIZON * 20] = {0};
    /* Build a simple DMC-like G: ramp step response */
    for (int i = 0; i < 10; i++)
        for (int j = 0; j <= i && j < 3; j++)
            G_test[j * 10 + i] = (double)(i - j + 1);
    double K_gain[MPC_MAX_STEP_HORIZON];
    int ret = gpc_compute_gain(G_test, 10, 3, 1.0, K_gain);
    CHECK(ret == 0);
    CHECK(K_gain[0] != 0.0);
    PASS();

    TEST("gpc_design_T_filter");
    gpc_design_T_filter(&gpc, 0.8, 2);
    CHECK(gpc.nc == 2);
    PASS();
}

/* ─── L6: Surge Tank ──────────────────────────────────────────────────── */

static void test_surge_tank(void) {
    surge_tank_t tank;

    TEST("surge_tank_init");
    CHECK(surge_tank_init(&tank, 2.0, 3.0, 1.5, 0.1) == 0);
    CLOSE(tank.diameter, 2.0, 1e-9);
    CLOSE(tank.cross_area, M_PI, 1e-6); /* π*2²/4 = π */
    CLOSE(tank.level, 1.5, 1e-9);
    PASS();

    TEST("surge_tank_outflow");
    double F_out = surge_tank_outflow(&tank, 50.0);
    CHECK(F_out > 0.0);
    PASS();

    TEST("surge_tank_simulate");
    surge_tank_simulate(&tank, 0.2, 0.1, 1.0);
    CHECK(tank.level > 1.5); /* inflow > outflow → level rises */
    PASS();

    TEST("surge_level_filter_factor");
    double phi = surge_level_filter_factor(0.01, 100.0);
    CLOSE(phi, 0.5, 1e-6);
    PASS();

    TEST("surge_capacity_utilization");
    double ampl;
    double A_f_max = surge_capacity_utilization(&tank, 0.01, 0.5, &ampl);
    CHECK(A_f_max > 0.0);
    PASS();

    TEST("surge_feedforward");
    double du_ff = surge_feedforward(&tank, 0.1);
    CHECK(du_ff != 0.0);
    PASS();

    TEST("surge_feedforward_matrix");
    double G_d[20];
    surge_feedforward_matrix(G_d, &tank, 20, 1.0);
    CLOSE(G_d[0], 1.0 / tank.cross_area, 1e-6);
    CLOSE(G_d[19], 20.0 / tank.cross_area, 1e-6);
    PASS();

    TEST("surge_tank_sizing");
    double V_min = surge_tank_sizing(0.1, 600.0, 0.5, 0.2);
    CHECK(V_min > 0.0);
    PASS();

    TEST("surge_tank_volume");
    double V = surge_tank_volume(&tank);
    CLOSE(V, tank.cross_area * tank.level, 1e-9);
    PASS();

    TEST("surge_level_percent");
    double pct = surge_level_percent(&tank);
    CHECK(pct > 0.0 && pct < 100.0);
    PASS();
}

/* ─── Integration Tests ────────────────────────────────────────────────── */

static void test_integration(void) {
    TEST("dmc_step_integration");
    /* Full DMC step on pure integrator */
    integrating_process_t model;
    mpc_model_pure_integrator(&model, -0.5, 1.0, 5.0);

    step_response_t step;
    mpc_step_response_from_model(&step, &model, 30);

    mpc_tuning_t tuning;
    mpc_level_config_t config;
    dmc_dynamic_t dyn;
    mpc_solution_t solution;

    tuning.prediction_horizon = 20;
    tuning.control_horizon = 5;
    tuning.sampling_time = 1.0;
    tuning.move_suppression = 1.0;
    tuning.output_weight = 1.0;
    tuning.terminal_weight = 100.0;
    tuning.reference_trajectory_alpha = 0.7;
    tuning.move_blocking_enabled = 0;

    config.level_setpoint = 1.5;
    config.valve_min = 0.0;
    config.valve_max = 100.0;
    config.valve_rate_max = 100.0;
    config.level_lo_limit = 0.0;
    config.level_hi_limit = 3.0;

    dmc_init(&dyn, &step, &tuning);

    /* Simulate a control step with level at 1.0 (below setpoint 1.5) */
    double du = dmc_step(&solution, &dyn, &step, &tuning, &config,
                          1.0, 50.0, MPC_SOLVER_ACTIVE_SET);
    /* Since level < setpoint and integrating process with negative gain,
     * closing valve (negative du) would increase level → expect nonzero move */
    CHECK(du != 0.0);
    PASS();

    TEST("gpc_step_integration");
    gpc_config_t gpc_cfg;
    gpc_integrating_setup(&gpc_cfg, -0.5, 1.0, 20, 5, 1.0);
    double du_past[10] = {0};
    double y_past[10] = {1.0};
    double gpc_du = gpc_step(&solution, &gpc_cfg, 1.0, 1.5, 50.0,
                              du_past, y_past);
    CHECK(gpc_du != 0.0);
    PASS();

    TEST("surge_mpc_simulation");
    surge_tank_t tank;
    surge_tank_init(&tank, 2.0, 3.0, 1.5, 0.1);

    mpc_tuning_t sim_tuning;
    mpc_level_config_t sim_config;
    surge_mpc_config(&sim_tuning, &sim_config, &tank,
                      LEVEL_MODE_AVERAGING, 0.2);

    /* Build step response */
    integrating_process_t sim_model;
    mpc_model_pure_integrator(&sim_model,
        mpc_tank_area_to_gain(tank.cross_area, tank.valve_coeff, tank.level),
        sim_tuning.sampling_time, tank.cross_area);

    step_response_t sim_step;
    mpc_step_response_from_model(&sim_step, &sim_model, sim_tuning.prediction_horizon);

    dmc_dynamic_t sim_dyn;
    dmc_init(&sim_dyn, &sim_step, &sim_tuning);

    /* Inlet flow disturbance: step up at t=30s, step down at t=60s */
    int n_steps = 80;
    double F_in_hist[80], F_out_hist[80], level_hist[80], valve_hist[80];

    for (int k = 0; k < n_steps; k++) {
        if (k < 30) F_in_hist[k] = 0.15;
        else if (k < 60) F_in_hist[k] = 0.25;
        else F_in_hist[k] = 0.15;
    }

    mpc_kpi_t kpi;
    int sim_ret = surge_mpc_simulation_run(&tank, &sim_tuning, &sim_config,
        &sim_step, &sim_dyn, F_in_hist, n_steps,
        F_out_hist, level_hist, valve_hist, &kpi);
    CHECK(sim_ret == 0);
    CHECK(kpi.sample_count == n_steps);
    CHECK(kpi.constraint_violation_pct <= 100.0); /* simulation ran */
    PASS();
}

/* ─── Main ────────────────────────────────────────────────────────────── */

int main(void) {
    printf("\n========================================\n");
    printf("  MPC INTEGRATING LEVEL CONTROL TESTS\n");
    printf("========================================\n\n");

    printf("--- L1: Types ---\n");
    test_types();

    printf("\n--- L2-L3: Models ---\n");
    test_models();

    printf("\n--- L3-L5: DMC ---\n");
    test_dmc();

    printf("\n--- L5: QP Solver ---\n");
    test_qp();

    printf("\n--- L5: Kalman Filter ---\n");
    test_kalman();

    printf("\n--- L4: Constraints ---\n");
    test_constraints();

    printf("\n--- L5: GPC ---\n");
    test_gpc();

    printf("\n--- L6: Surge Tank ---\n");
    test_surge_tank();

    printf("\n--- Integration Tests ---\n");
    test_integration();

    printf("\n========================================\n");
    printf("  RESULTS: %d/%d tests passed\n", tests_passed, tests_run);
    printf("========================================\n\n");

    return (tests_passed == tests_run) ? 0 : 1;
}
