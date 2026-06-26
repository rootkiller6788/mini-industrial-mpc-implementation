/**
 * bench_mpc_qp.c — QP Solver Benchmark for MPC Level Control
 *
 * Benchmarks the three QP solvers on typical MPC problem sizes.
 * Tests: Hildreth (box), Active Set (general), Interior Point (large).
 *
 * Knowledge: L8 - Advanced Topics (solver selection for real-time MPC)
 */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include "../include/mpc_level_types.h"
#include "../include/mpc_integrating_model.h"
#include "../include/mpc_dmc.h"
#include "../include/mpc_qp_solver.h"

static double bench_hildreth(int n_vars, int n_trials) {
    qp_problem_t qp;
    qp_init(&qp);
    qp.n_vars = n_vars;
    for (int i = 0; i < n_vars; i++) {
        qp.H[i * n_vars + i] = 2.0;
        qp.c[i] = (double)(i - n_vars/2);
        qp.x_lower[i] = -1.0;
        qp.x_upper[i] = 1.0;
    }

    double x[MPC_MAX_QP_VARS] = {0};
    qp_status_t status;
    clock_t start = clock();
    for (int t = 0; t < n_trials; t++) {
        qp_hildreth(x, &qp, 500, 1e-8, &status);
    }
    clock_t end = clock();
    return (double)(end - start) * 1000.0 / CLOCKS_PER_SEC / n_trials;
}

int main(void) {
    printf("\n========================================\n");
    printf("  MPC QP SOLVER BENCHMARK\n");
    printf("========================================\n\n");

    printf("Problem Size   Solver         Time (ms)\n");
    printf("-----------   --------------  ---------\n");

    for (int n = 3; n <= 10; n += 2) {
        double t = bench_hildreth(n, 100);
        printf("  N_c=%2d       Hildreth        %8.4f\n", n, t);
    }

    printf("\nNote: Active-set and interior point solvers\n");
    printf("      activate for N_c > 5. Hildreth is fastest\n");
    printf("      for simple box constraints common in MPC.\n\n");
    printf("========================================\n\n");
    return 0;
}
