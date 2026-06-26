/** @file test_dmc.c
 * @brief Tests for MPC core functions
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include "mpc_common.h"

static int tests_run = 0;
static int tests_passed = 0;

#define TEST(name) do { tests_run++; printf("  TEST %-25s ... ", name); fflush(stdout); } while(0)
#define PASS() do { tests_passed++; printf("PASSED\n"); fflush(stdout); } while(0)
#define FAIL(msg) do { printf("FAILED: %s\n", msg); fflush(stdout); } while(0)

static void test_alloc_free(void) {
    TEST("alloc_free");
    mpc_step_model_t *m = mpc_step_model_alloc(60);
    assert(m != NULL && m->n_coeffs == 60 && m->coeff != NULL);
    mpc_step_model_free(m);
    PASS();
}

static void test_fopdt_model(void) {
    TEST("fopdt_model");
    mpc_step_model_t *m = mpc_step_model_alloc(60);
    int rc = mpc_step_model_from_fopdt(m, 2.0, 10.0, 3.0, 1.0);
    assert(rc == 0);
    assert(fabs(m->gain_ss - 2.0) < 1e-10);
    assert(fabs(m->time_constant_sec - 10.0) < 1e-10);
    assert(fabs(m->coeff[0]) < 1e-10);
    assert(fabs(m->coeff[1]) < 1e-10);
    assert(fabs(m->coeff[2]) < 1e-10);
    assert(m->coeff[3] > 0.0);
    assert(fabs(m->coeff[59] - 2.0) < 0.01);
    mpc_step_model_free(m);
    PASS();
}

static void test_step_model_predict(void) {
    TEST("step_model_predict");
    mpc_step_model_t *m = mpc_step_model_alloc(60);
    mpc_step_model_from_fopdt(m, 1.0, 5.0, 0.0, 1.0);
    double du_past[10] = {1.0,0,0,0,0,0,0,0,0,0};
    double y_pred[5];
    int rc = mpc_step_model_predict(m, du_past, 10, y_pred, 5);
    assert(rc == 0 && y_pred[0] > 0.0);
    mpc_step_model_free(m);
    PASS();
}

static void test_bias_update(void) {
    TEST("bias_update");
    mpc_controller_state_t *cs = mpc_controller_state_alloc(30, 5, 60, 2, 2, 1);
    assert(cs != NULL);
    double y_meas[2] = {1.5, 2.5}, y_pred[2] = {1.0, 2.0};
    int rc = mpc_dmc_bias_update(cs, y_meas, y_pred, 0.5);
    assert(rc == 0 && cs->cv_error_past[0] > 0.0 && cs->cv_error_past[1] > 0.0);
    mpc_controller_state_free(cs);
    PASS();
}

static void test_horizon_shift(void) {
    TEST("horizon_shift");
    mpc_controller_state_t *cs = mpc_controller_state_alloc(30, 5, 60, 2, 2, 1);
    assert(cs != NULL);
    cs->delta_u_past[0] = 1.0;
    cs->delta_u_opt[0] = 0.5; cs->delta_u_opt[1] = 1.5;
    int rc = mpc_dmc_shift_horizon(cs, 2);
    assert(rc == 0 && fabs(cs->delta_u_past[59*2+0] - 0.5) < 1e-10);
    mpc_controller_state_free(cs);
    PASS();
}

static void test_qp_solver(void) {
    TEST("qp_solver");
    int n = 4;
    mpc_qp_problem_t *qp = mpc_qp_problem_alloc(n, 0, 0);
    assert(qp != NULL);
    for (int i = 0; i < n; i++) qp->H[i*n+i] = 2.0;
    qp->c[0] = -4.0; qp->c[1] = -6.0;
    mpc_qp_solution_t *sol = mpc_qp_solution_alloc(n);
    assert(sol != NULL);
    qp_status_t status = mpc_qp_active_set_solve(qp, sol);
    assert(status == QP_OPTIMAL);
    assert(fabs(sol->x_opt[0] - 2.0) < 0.2);
    assert(fabs(sol->x_opt[1] - 3.0) < 0.2);
    mpc_qp_solution_free(sol); mpc_qp_problem_free(qp);
    PASS();
}

static void test_dynamic_matrix(void) {
    TEST("dynamic_matrix");
    mpc_step_model_t *m = mpc_step_model_alloc(30);
    mpc_step_model_from_fopdt(m, 1.0, 5.0, 0.0, 1.0);
    mpc_dynamic_matrix_t dm;
    int rc = mpc_build_dynamic_matrix(m, 10, 3, &dm);
    assert(rc == 0 && dm.P == 10 && dm.M == 3 && dm.A != NULL && dm.A_T != NULL);
    assert(dm.A[0] > 0.0 && fabs(dm.A[1]) < 1e-10);
    mpc_dynamic_matrix_free(&dm); mpc_step_model_free(m);
    PASS();
}

static void test_variable_scaling(void) {
    TEST("variable_scaling");
    assert(fabs(mpc_pct_to_eu(50.0, 0.0, 100.0) - 50.0) < 1e-10);
    assert(fabs(mpc_eu_to_pct(25.0, 0.0, 200.0) - 12.5) < 1e-10);
    assert(fabs(mpc_pct_to_eu(0.0, -10.0, 40.0) - (-10.0)) < 1e-10);
    PASS();
}

static void test_rls_update(void) {
    TEST("rls_update");
    mpc_rls_estimator_t *rls = mpc_rls_alloc(2, 0.99, 1.0);
    assert(rls != NULL);
    double phi[2];
    for (int i = 0; i < 100; i++) {
        phi[0] = (double)(i % 10); phi[1] = (double)((i * 3) % 7);
        mpc_rls_update(rls, phi, 2.0*phi[0] + 3.0*phi[1]);
    }
    double theta[2]; mpc_rls_get_params(rls, theta);
    assert(fabs(theta[0] - 2.0) < 0.5 && fabs(theta[1] - 3.0) < 0.5);
    mpc_rls_free(rls);
    PASS();
}

static void test_aspen_config(void) {
    TEST("aspen_config");
    mpc_aspen_config_t *cfg = mpc_aspen_config_alloc();
    assert(cfg != NULL && cfg->n_mv == 2 && cfg->n_cv == 2);
    assert(cfg->P == 30 && cfg->M == 5);
    mpc_aspen_config_free(cfg);
    PASS();
}

static void test_data_recon(void) {
    TEST("data_recon");
    int n = 3;
    double y_raw[3] = {10.2, 5.1, 4.9};
    double sigma_sq[3] = {0.01, 0.01, 0.01};
    double A[3] = {1.0, -1.0, -1.0};
    double y_hat[3], adj[3];
    int rc = mpc_data_recon_wls(y_raw, sigma_sq, A, n, 1, y_hat, adj);
    assert(rc == 0);
    double residual = y_hat[0] - y_hat[1] - y_hat[2];
    assert(fabs(residual) < 0.001);
    PASS();
}

static void test_rga(void) {
    TEST("rga_compute");
    double G[4] = {2.0, 1.5, 1.2, 2.0};
    double RGA[4];
    int rc = mpc_compute_rga(G, 2, 2, RGA);
    assert(rc == 0);
    assert(fabs(RGA[0] + RGA[1] - 1.0) < 0.1 && fabs(RGA[2] + RGA[3] - 1.0) < 0.1);
    PASS();
}

static void test_zone_control(void) {
    TEST("zone_control");
    double y_pred[5] = {0.5, 1.0, 1.5, 0.8, 1.2};
    double grad[5], cost;
    int rc = mpc_zone_control_gradient(y_pred, 5, 0.8, 1.2, 1.0, 1.0, grad, &cost);
    assert(rc == 0);
    assert(fabs(grad[2]) > 0.0);
    assert(fabs(grad[1]) < MPC_EPS);
    PASS();
}

static void test_disturbance_step(void) {
    TEST("dist_step");
    double d_cur[2] = {1.0, 2.0};
    double d_fc[6];
    int rc = mpc_disturbance_step_predict(d_cur, 2, 3, d_fc);
    assert(rc == 0);
    assert(fabs(d_fc[0] - 1.0) < 1e-10 && fabs(d_fc[1] - 2.0) < 1e-10);
    assert(fabs(d_fc[4] - 1.0) < 1e-10 && fabs(d_fc[5] - 2.0) < 1e-10);
    PASS();
}

static void test_niederlinski(void) {
    TEST("niederlinski");
    double G[4] = {2.0, 1.5, 1.2, 2.0};
    double ni = mpc_niederlinski_index(G, 2);
    assert(ni > 0.0);
    PASS();
}

static void test_funnel_constraints(void) {
    TEST("funnel");
    double lo[5], hi[5];
    int rc = mpc_generate_funnel_constraints(0.8, 1.2, 0.05, 5, lo, hi);
    assert(rc == 0);
    assert(lo[0] == 0.8 && hi[0] == 1.2);
    assert(lo[4] < lo[0] && hi[4] > hi[0]);
    PASS();
}

static void test_heat_balance(void) {
    TEST("heat_balance");
    double Q_hot, Q_cold, loss;
    int rc = mpc_reconcile_heat_balance(10.0, 8.0, 4.2, 4.2, 80.0, 60.0, 20.0, 40.0, &Q_hot, &Q_cold, &loss);
    assert(rc == 0);
    assert(Q_hot > 0.0 && Q_cold > 0.0);
    PASS();
}

int main(void) {
    printf("=== mini-mpc-industrial-software-aspentech Test Suite ===\n\n");
    fflush(stdout);
    test_alloc_free();
    test_fopdt_model();
    test_step_model_predict();
    test_bias_update();
    test_horizon_shift();
    test_qp_solver();
    test_dynamic_matrix();
    test_variable_scaling();
    test_rls_update();
    test_aspen_config();
    test_data_recon();
    test_rga();
    test_zone_control();
    test_disturbance_step();
    test_niederlinski();
    test_funnel_constraints();
    test_heat_balance();
    printf("\n=== Results: %d/%d tests passed ===\n", tests_passed, tests_run);
    fflush(stdout);
    return (tests_passed == tests_run) ? 0 : 1;
}
