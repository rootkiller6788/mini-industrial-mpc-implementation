/** @file numerical_cond.c
 * @brief Numerical Conditioning: SVD, Condition Number (L4, L5)
 *
 * Theorem: kappa(G) = sigma_max(G) / sigma_min(G)
 *   kappa < 10: well-conditioned
 *   kappa > 100: ill-conditioned (use SVD truncation)
 *
 * Ref: Golub and Van Loan - Matrix Computations (2013) Ch.2, Ch.8
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "mpc_common.h"

/* Power iteration for largest singular value of A */
static double power_iteration(const double *A, int rows, int cols, int max_iter)
{
    if (!A || rows < 1 || cols < 1) return 0.0;
    double *v = (double*)calloc(cols, sizeof(double));
    double *Av = (double*)calloc(rows, sizeof(double));
    double *ATAv = (double*)calloc(cols, sizeof(double));
    if (!v || !Av || !ATAv) { free(v); free(Av); free(ATAv); return 0.0; }

    for (int i = 0; i < cols; i++) v[i] = 1.0 / sqrt((double)cols);
    double sigma = 0.0;

    for (int iter = 0; iter < max_iter; iter++) {
        memset(Av, 0, rows * sizeof(double));
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++) Av[i] += A[i*cols+j] * v[j];

        memset(ATAv, 0, cols * sizeof(double));
        for (int i = 0; i < cols; i++)
            for (int j = 0; j < rows; j++) ATAv[i] += A[j*cols+i] * Av[j];

        double norm = 0.0;
        for (int i = 0; i < cols; i++) norm += ATAv[i] * ATAv[i];
        norm = sqrt(norm);
        if (norm < MPC_EPS) break;
        sigma = sqrt(norm);
        for (int i = 0; i < cols; i++) v[i] = ATAv[i] / norm;
    }
    free(v); free(Av); free(ATAv);
    return sigma;
}

double mpc_condition_number_svd(const double *G, int n_rows, int n_cols)
{
    if (!G || n_rows < 1 || n_cols < 1) return MPC_INF;
    double sigma_max = power_iteration(G, n_rows, n_cols, 50);
    if (sigma_max < MPC_EPS) return MPC_INF;

    double *G_inv_approx = (double*)calloc(n_cols * n_rows, sizeof(double));
    if (!G_inv_approx) return MPC_INF;
    for (int i = 0; i < n_rows && i < n_cols; i++) {
        double val = G[i*n_cols+i];
        if (fabs(val) > MPC_EPS) G_inv_approx[i*n_rows+i] = 1.0 / val;
    }
    double sigma_min_inv = power_iteration(G_inv_approx, n_cols, n_rows, 50);
    free(G_inv_approx);

    if (sigma_min_inv < MPC_EPS) return MPC_INF;
    double sigma_min = 1.0 / sigma_min_inv;
    return sigma_max / sigma_min;
}

int mpc_detect_ill_conditioning(const mpc_mimo_model_t *mimo,
    double threshold, int *pairs, int max_pairs)
{
    if (!mimo || !pairs || max_pairs < 1) return -1;
    int n_cv = mimo->n_cv, n_mv = mimo->n_mv;
    int count = 0;
    double *G_block = (double*)calloc(n_cv * n_mv, sizeof(double));
    if (!G_block) return -2;
    mpc_mimo_extract_ss_gain(mimo, G_block);
    double kappa = mpc_condition_number_svd(G_block, n_cv, n_mv);

    if (kappa > threshold) {
        for (int cv = 0; cv < n_cv && count < max_pairs; cv++) {
            for (int mv = 0; mv < n_mv && count < max_pairs; mv++) {
                double g = G_block[cv * n_mv + mv];
                if (fabs(g) < MPC_EPS) {
                    pairs[count * 2] = mv;
                    pairs[count * 2 + 1] = cv;
                    count++;
                }
            }
        }
    }
    free(G_block);
    return count;
}

int mpc_svd_truncate(double *U, double *S, double *Vt,
    int rows, int cols, double s_floor)
{
    if (!S || s_floor < 0.0) return -1;
    int n_truncated = 0;
    int min_dim = (rows < cols) ? rows : cols;
    for (int i = 0; i < min_dim; i++) {
        if (fabs(S[i]) < s_floor) { S[i] = 0.0; n_truncated++; }
    }
    return n_truncated;
}

/* Tikhonov regularization: H_reg = H + lambda * I */
int mpc_regularize_hessian(double *H, int n, double lambda)
{
    if (!H || n < 1 || lambda < 0.0) return -1;
    for (int i = 0; i < n; i++) H[i * n + i] += lambda;
    return 0;
}
