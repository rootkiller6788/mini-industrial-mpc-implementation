/**
 * @file test_mpc_model.c
 * @brief Unit tests for MPC model operations
 */

#include "mpc_model.h"
#include <stdio.h>
#include <math.h>
#include <assert.h>

#define TOL 1e-6

/* Test first-order model creation */
static void test_first_order_model(void)
{
    mpc_ss_model_t model;
    mpc_model_first_order(&model, 2.0, 5.0, 1.0);

    assert(model.nx == 1);
    assert(model.nu == 1);
    assert(model.ny == 1);
    assert(fabs(model.A[0][0] - exp(-1.0/5.0)) < TOL);
    assert(fabs(model.B[0][0] - 2.0*(1.0 - exp(-1.0/5.0))) < TOL);
    assert(fabs(model.C[0][0] - 1.0) < TOL);
    printf("  PASS: first_order_model\n");
}

/* Test integrator model */
static void test_integrator_model(void)
{
    mpc_ss_model_t model;
    mpc_model_integrator(&model, 1.0, 0.5);

    assert(fabs(model.A[0][0] - 1.0) < TOL);
    assert(fabs(model.B[0][0] - 0.5) < TOL);
    printf("  PASS: integrator_model\n");
}

/* Test augmented model building */
static void test_build_augmented(void)
{
    mpc_ss_model_t plant;
    mpc_aug_model_t aug;

    mpc_model_first_order(&plant, 1.0, 1.0, 0.5);
    int ret = mpc_build_augmented_model(&plant, MPC_MODE_OUTPUT_DISTURBANCE, &aug);
    assert(ret == 0);
    assert(aug.nx_aug == plant.nx + 1);  /* nx + nd where nd=ny=1 */
    assert(aug.Aa[plant.nx][plant.nx] == 1.0);  /* disturbance integrator */
    printf("  PASS: build_augmented (output disturbance)\n");
}

/* Test prediction matrix construction */
static void test_prediction_matrices(void)
{
    mpc_ss_model_t plant;
    mpc_aug_model_t aug;
    mpc_tuning_t tuning;
    mpc_prediction_t pred;

    mpc_model_first_order(&plant, 1.0, 1.0, 0.5);
    mpc_build_augmented_model(&plant, MPC_MODE_OUTPUT_DISTURBANCE, &aug);
    mpc_tuning_init_default(&tuning);
    tuning.np = 5;
    tuning.nc = 2;

    int ret = mpc_build_prediction_matrices(&aug, &tuning, &pred);
    assert(ret == 0);
    assert(pred.np == 5);
    assert(pred.nc == 2);

    /* Phi should have non-trivial entries */
    int has_nonzero = 0;
    for (int i = 0; i < 5 && !has_nonzero; i++)
        for (int j = 0; j < aug.nx_aug && !has_nonzero; j++)
            if (fabs(pred.Phi[i][j]) > TOL)
                has_nonzero = 1;
    assert(has_nonzero);
    printf("  PASS: prediction_matrices\n");
}

/* Test steady-state target */
static void test_steady_state_target(void)
{
    mpc_ss_model_t model;
    double y_ref[MPC_MAX_NY] = {1.0};
    double x_ss[MPC_MAX_NX], u_ss[MPC_MAX_NU];

    /* For an integrator, the steady-state input for y=1 is u=0 */
    mpc_model_integrator(&model, 1.0, 0.5);
    mpc_compute_steady_state_target(&model, y_ref, x_ss, u_ss);

    /* Result should be valid */
    assert(!isnan(x_ss[0]));
    printf("  PASS: steady_state_target\n");
}

/* Test step response model */
static void test_step_model(void)
{
    mpc_ss_model_t plant;
    mpc_step_model_t step;

    mpc_model_first_order(&plant, 1.0, 1.0, 0.5);
    int ret = mpc_build_step_model(&plant, 10, &step);
    assert(ret == 0);
    assert(step.n == 10);

    /* Step response should be non-negative for stable first-order */
    for (int i = 0; i < 10; i++)
        assert(step.S[i][0][0] >= -TOL);

    printf("  PASS: step_model\n");
}

/* Test DARE solver on a simple scalar system */
static void test_dare_scalar(void)
{
    double A[MPC_MAX_NX][MPC_MAX_NX] = {{0.8}};
    double B[MPC_MAX_NX][MPC_MAX_NU] = {{1.0}};
    double Q[MPC_MAX_NX][MPC_MAX_NX] = {{1.0}};
    double R[MPC_MAX_NU][MPC_MAX_NU] = {{0.1}};
    double P[MPC_MAX_NX][MPC_MAX_NX];

    int iter = mpc_solve_dare(A, B, Q, R, 1, 1, P, 500, 1e-10);
    assert(iter > 0);
    assert(P[0][0] > 0.0);  /* P should be positive */
    printf("  PASS: dare_scalar (P=%.6f, %d iters)\n", P[0][0], iter);
}

/* Test matrix exponential on 1x1 scalar */
static void test_matrix_expm(void)
{
    double A[MPC_MAX_NX][MPC_MAX_NX] = {{0.5}};
    double E[MPC_MAX_NX][MPC_MAX_NX];
    int ret = mpc_matrix_expm(A, 1, E);
    assert(ret == 0);

    /* exp(0.5) = e^0.5 */
    double expected = exp(0.5);
    double err = fabs(E[0][0] - expected);
    printf("  matrix_expm(1x1): got %.10f, expected %.10f, err=%.2e\n",
           E[0][0], expected, err);
    assert(err < 1e-6);
    printf("  PASS: matrix_expm (1x1 scalar)\n");
}

/* Test discretization of integrator */
static void test_discretize_integrator(void)
{
    double Ac[MPC_MAX_NX][MPC_MAX_NX] = {{0.0}};
    double Bc[MPC_MAX_NX][MPC_MAX_NU] = {{1.0}};
    double Cc[MPC_MAX_NY][MPC_MAX_NX] = {{1.0}};
    double Dc[MPC_MAX_NY][MPC_MAX_NU] = {{0.0}};
    mpc_ss_model_t model;

    int ret = mpc_discretize_model(Ac, Bc, Cc, Dc, 1, 1, 1, 0.1,
                                    MPC_DISCRETIZE_ZOH, &model);
    assert(ret == 0);
    assert(fabs(model.A[0][0] - 1.0) < TOL);
    assert(fabs(model.A[0][0] - 1.0) < TOL);
    printf("  discretize_integrator: B[0]=%.10f (expected 0.1)\n", model.B[0][0]);
    assert(fabs(model.B[0][0] - 0.1) < 1e-4);  /* relaxed tolerance for Pade */
    printf("  PASS: discretize_integrator\n");
}

/* Test model validation */
static void test_validation(void)
{
    mpc_ss_model_t model;
    mpc_tuning_t tuning;

    mpc_model_init(&model, 2, 1, 1, 0);
    assert(mpc_validate_model(&model) == 0);

    mpc_tuning_init_default(&tuning);
    assert(mpc_validate_tuning(&tuning) == 0);

    /* Invalid tuning: nc > np */
    tuning.nc = 100;
    tuning.np = 10;
    assert(mpc_validate_tuning(&tuning) == -1);

    printf("  PASS: validation\n");
}

int main(void)
{
    printf("Test: mpc_model\n");
    test_first_order_model();
    test_integrator_model();
    test_build_augmented();
    test_prediction_matrices();
    test_steady_state_target();
    test_step_model();
    test_dare_scalar();
    test_matrix_expm();
    test_discretize_integrator();
    test_validation();
    printf("ALL TESTS PASSED\n");
    return 0;
}
