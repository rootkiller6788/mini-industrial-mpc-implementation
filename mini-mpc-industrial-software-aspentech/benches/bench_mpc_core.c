/** @file bench_mpc_core.c
 * @brief Performance Benchmarks for MPC Core Functions
 *
 * Measures execution time of core MPC operations:
 *   - FOPDT -> Step Response conversion
 *   - Convolution prediction (various horizon sizes)
 *   - QP solver (active-set vs interior-point)
 *   - Dynamic matrix construction
 *   - Complete DMC step
 *
 * Uses simple wall-clock timing via clock().
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "mpc_common.h"

static double elapsed_sec(clock_t start, clock_t end)
{
    return (double)(end - start) / CLOCKS_PER_SEC;
}

static void bench_fopdt_to_step(int n_coeffs, int n_repeat)
{
    mpc_step_model_t *m = mpc_step_model_alloc(n_coeffs);
    clock_t t0 = clock();
    for (int r = 0; r < n_repeat; r++) {
        mpc_step_model_from_fopdt(m, 2.0, 10.0, 3.0, 1.0);
    }
    clock_t t1 = clock();
    printf("  fopdt_to_step(N=%d, rep=%d): %.4f sec (%.2f us/call)\n",
        n_coeffs, n_repeat, elapsed_sec(t0, t1), elapsed_sec(t0, t1)*1e6/n_repeat);
    mpc_step_model_free(m);
}

static void bench_convolution_predict(int n_coeffs, int n_pred, int n_past, int n_repeat)
{
    mpc_step_model_t *m = mpc_step_model_alloc(n_coeffs);
    mpc_step_model_from_fopdt(m, 1.0, 5.0, 0.0, 1.0);
    double *du_past = (double*)calloc(n_past, sizeof(double));
    double *y_pred = (double*)calloc(n_pred, sizeof(double));
    for (int i = 0; i < n_past; i++) du_past[i] = (i == 0) ? 1.0 : 0.0;

    clock_t t0 = clock();
    for (int r = 0; r < n_repeat; r++) {
        mpc_step_model_predict(m, du_past, n_past, y_pred, n_pred);
    }
    clock_t t1 = clock();
    printf("  predict(N=%d, P=%d, past=%d, rep=%d): %.4f sec (%.2f us/call)\n",
        n_coeffs, n_pred, n_past, n_repeat, elapsed_sec(t0, t1), elapsed_sec(t0, t1)*1e6/n_repeat);

    free(du_past); free(y_pred);
    mpc_step_model_free(m);
}

static void bench_qp_solver(int n_vars, int n_repeat)
{
    mpc_qp_problem_t *qp = mpc_qp_problem_alloc(n_vars, 0, 0);
    mpc_qp_solution_t *sol = mpc_qp_solution_alloc(n_vars);

    for (int i = 0; i < n_vars; i++) qp->H[i*n_vars+i] = 2.0;
    for (int i = 0; i < n_vars; i++) qp->c[i] = -(double)(i % 5 + 1);

    clock_t t0 = clock();
    for (int r = 0; r < n_repeat; r++) {
        mpc_qp_active_set_solve(qp, sol);
    }
    clock_t t1 = clock();
    printf("  qp_active_set(n=%d, rep=%d): %.4f sec (%.2f us/call)  f_opt=%.4f\n",
        n_vars, n_repeat, elapsed_sec(t0, t1), elapsed_sec(t0, t1)*1e6/n_repeat, sol->f_opt);

    clock_t t2 = clock();
    for (int r = 0; r < n_repeat; r++) {
        mpc_qp_interior_point_solve(qp, sol);
    }
    clock_t t3 = clock();
    printf("  qp_interior_pt(n=%d, rep=%d): %.4f sec (%.2f us/call)  f_opt=%.4f\n",
        n_vars, n_repeat, elapsed_sec(t2, t3), elapsed_sec(t2, t3)*1e6/n_repeat, sol->f_opt);

    mpc_qp_solution_free(sol);
    mpc_qp_problem_free(qp);
}

static void bench_dynamic_matrix(int P, int M, int N, int n_repeat)
{
    mpc_step_model_t *m = mpc_step_model_alloc(N);
    mpc_step_model_from_fopdt(m, 1.0, 10.0, 2.0, 1.0);
    mpc_dynamic_matrix_t dm;

    clock_t t0 = clock();
    for (int r = 0; r < n_repeat; r++) {
        mpc_build_dynamic_matrix(m, P, M, &dm);
        if (r < n_repeat - 1) mpc_dynamic_matrix_free(&dm);
    }
    clock_t t1 = clock();
    printf("  build_dynamic_matrix(P=%d, M=%d, N=%d, rep=%d): %.4f sec (%.2f us/call)\n",
        P, M, N, n_repeat, elapsed_sec(t0, t1), elapsed_sec(t0, t1)*1e6/n_repeat);
    mpc_dynamic_matrix_free(&dm);
    mpc_step_model_free(m);
}

static void bench_rls_update(int n_params, int n_repeat)
{
    mpc_rls_estimator_t *rls = mpc_rls_alloc(n_params, 0.99, 1.0);
    double *phi = (double*)calloc(n_params, sizeof(double));
    for (int i = 0; i < n_params; i++) phi[i] = (double)(i % 3 + 1);

    double y_true = 0;
    for (int i = 0; i < n_params; i++) y_true += phi[i] * (i + 1.0);

    clock_t t0 = clock();
    for (int r = 0; r < n_repeat; r++) {
        mpc_rls_update(rls, phi, y_true);
    }
    clock_t t1 = clock();
    printf("  rls_update(n=%d, rep=%d): %.4f sec (%.2f us/call)\n",
        n_params, n_repeat, elapsed_sec(t0, t1), elapsed_sec(t0, t1)*1e6/n_repeat);

    free(phi);
    mpc_rls_free(rls);
}

static void bench_kalman_cycle(int nx, int ny, int nu, int n_repeat)
{
    mpc_kalman_state_t *kf = mpc_kalman_alloc(nx, ny, nu);
    double *u = (double*)calloc(nu, sizeof(double));
    double *y = (double*)calloc(ny, sizeof(double));
    for (int i = 0; i < ny; i++) y[i] = 1.0;

    clock_t t0 = clock();
    for (int r = 0; r < n_repeat; r++) {
        mpc_kalman_predict(kf, u);
        mpc_kalman_correct(kf, y);
    }
    clock_t t1 = clock();
    printf("  kalman_filter(nx=%d, ny=%d, nu=%d, rep=%d): %.4f sec (%.2f us/call)\n",
        nx, ny, nu, n_repeat, elapsed_sec(t0, t1), elapsed_sec(t0, t1)*1e6/n_repeat);

    free(u); free(y);
    mpc_kalman_free(kf);
}

int main(void)
{
    printf("=== MPC Core Function Benchmarks ===\n\n");
    printf("Configuration: O2 optimization, CLOCKS_PER_SEC=%ld\n\n", (long)CLOCKS_PER_SEC);

    bench_fopdt_to_step(60, 50000);
    bench_convolution_predict(60, 30, 10, 20000);
    bench_convolution_predict(120, 60, 20, 5000);
    bench_qp_solver(10, 5000);
    bench_qp_solver(50, 500);
    bench_dynamic_matrix(30, 5, 60, 50000);
    bench_dynamic_matrix(60, 10, 120, 10000);
    bench_rls_update(20, 50000);
    bench_rls_update(100, 10000);
    bench_kalman_cycle(10, 5, 3, 5000);
    bench_kalman_cycle(30, 10, 5, 1000);

    printf("\n=== Benchmarks Complete ===\n");
    return 0;
}
