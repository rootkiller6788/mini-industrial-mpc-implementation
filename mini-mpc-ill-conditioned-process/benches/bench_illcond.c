#include "mpc_illcond_defs.h"
#include "mpc_illcond_matrix.h"
#include "mpc_illcond_svd.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

/* Benchmark: condition number estimation vs. SVD for varying matrix sizes */
int main(void)
{
    printf("=== Ill-Conditioned Process MPC: Benchmarks ===\n\n");

    size_t sizes[] = {10, 20, 50, 100};
    int n_sizes = 4;

    for (int s = 0; s < n_sizes; s++) {
        size_t n = sizes[s];
        mpc_matrix_t *A = mpc_matrix_alloc(n, n);
        /* Create a matrix with controllable condition number */
        for (size_t i = 0; i < n; i++)
            mpc_matrix_set(A, i, i, 1.0 + (double)i * 0.1);

        clock_t start, end;

        /* Hager-Higham estimate */
        start = clock();
        double kappa_est = mpc_condition_estimate(A, MPC_CONDEST_NORM1);
        end = clock();
        double t_est = (double)(end - start) / CLOCKS_PER_SEC * 1000.0;

        /* Full SVD */
        mpc_svd_t svd;
        start = clock();
        mpc_svd_compute(A, &svd);
        end = clock();
        double t_svd = (double)(end - start) / CLOCKS_PER_SEC * 1000.0;

        printf("n=%3zu: Hager-Higham kappa=%.2e (%.2f ms) | SVD kappa=%.2e (%.2f ms) | speedup=%.1fx\n",
               n, kappa_est, t_est, svd.cond, t_svd,
               (t_svd > 0.001) ? t_svd / t_est : 0.0);

        mpc_svd_free(&svd);
        mpc_matrix_free(&A);
    }

    printf("\nBenchmark complete.\n");
    return 0;
}
