/** @file mpc_decomposition.c
 * @brief Sub-Controller Decomposition for Large-Scale MPC (L5)
 *
 * In industrial MPC (AspenTech DMC3, Honeywell RMPCT), large controllers
 * are decomposed into smaller sub-controllers for computational efficiency.
 *
 * Block Decomposition (Schur complement method):
 *   Partition the full MIMO system into nearly-independent blocks.
 *   For block-diagonal decomposition:
 *     H = diag(H_1, H_2, ..., H_b)
 *   Each sub-problem is solved independently.
 *
 * Coordinate Decomposition:
 *   Sub-controllers share coupling variables through coordination terms.
 *   Iterate between sub-problems until convergence.
 *
 * This file implements:
 *   1. Block detection via relative gain array (RGA) analysis
 *   2. Block decomposition of dynamic matrix
 *   3. Coordinated sub-controller iteration
 *
 * Ref:
 *   Rawlings/Mayne/Diehl (2017) Ch.6 - Distributed MPC
 *   Bristol (1966) - Relative Gain Array
 *   AspenTech DMC3 Builder Manual - Sub-Controller Configuration
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "mpc_common.h"

/* ===== L5: Relative Gain Array (RGA) for Block Detection =====
 *
 * RGA (Bristol 1966): Lambda_ij = G_ij * (G^{-1})_ji
 *
 * Theorem (RGA pairing rule):
 *   Pairs with Lambda_ij close to 1.0 are preferred for decentralized control.
 *   Lambda_ij < 0 indicates an unstable pairing with integral action.
 *   |Lambda_ij| >> 1 indicates strong interaction requiring decoupling.
 *
 * Algorithm: Compute RGA from steady-state gain matrix.
 *   Blocks are formed by grouping CV-MV pairs with |Lambda_ij| > threshold.
 */

int mpc_compute_rga(const double *G_ss, int n_cv, int n_mv, double *RGA)
{
    if (!G_ss || !RGA || n_cv < 1 || n_mv < 1) return -1;
    if (n_cv != n_mv) return -2;  /* RGA requires square system */

    int n = n_cv;
    memset(RGA, 0, n * n * sizeof(double));

    /* Build augmented matrix for inversion [G | I] */
    double *aug = (double*)calloc(n * (2 * n), sizeof(double));
    if (!aug) return -3;

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) aug[i*(2*n)+j] = G_ss[i*n+j];
        aug[i*(2*n)+n+i] = 1.0;
    }

    /* Gauss-Jordan elimination for G^{-1} */
    for (int k = 0; k < n; k++) {
        int pivot = k;
        double max_val = fabs(aug[k*(2*n)+k]);
        for (int i = k+1; i < n; i++) {
            if (fabs(aug[i*(2*n)+k]) > max_val) {
                max_val = fabs(aug[i*(2*n)+k]); pivot = i;
            }
        }
        if (max_val < MPC_EPS) { free(aug); return -4; }

        if (pivot != k) {
            for (int j = 0; j < 2*n; j++) {
                double tmp = aug[k*(2*n)+j];
                aug[k*(2*n)+j] = aug[pivot*(2*n)+j];
                aug[pivot*(2*n)+j] = tmp;
            }
        }

        double piv_inv = 1.0 / aug[k*(2*n)+k];
        for (int j = 0; j < 2*n; j++) aug[k*(2*n)+j] *= piv_inv;

        for (int i = 0; i < n; i++) {
            if (i == k) continue;
            double factor = aug[i*(2*n)+k];
            for (int j = 0; j < 2*n; j++) aug[i*(2*n)+j] -= factor * aug[k*(2*n)+j];
        }
    }

    /* RGA: element-wise product G .* (G^{-1})^T */
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            RGA[i*n+j] = G_ss[i*n+j] * aug[j*(2*n)+n+i];
        }
    }

    free(aug);
    return 0;
}

/* ===== L5: Detect Block-Diagonal Structure =====
 *
 * Given an RGA matrix, detect block-diagonal structure by thresholding.
 * A block exists when |Lambda_ij| > block_threshold.
 *
 * Returns: number of blocks found, fills block_map with block indices.
 */

int mpc_detect_rga_blocks(const double *RGA, int n, double block_threshold,
    int *block_map, int *n_blocks)
{
    if (!RGA || !block_map || !n_blocks || n < 2) return -1;
    if (block_threshold <= 0.0) block_threshold = 0.5;

    for (int i = 0; i < n; i++) block_map[i] = -1;

    int block_id = 0;
    for (int i = 0; i < n; i++) {
        if (block_map[i] >= 0) continue;
        block_map[i] = block_id;

        for (int j = i + 1; j < n; j++) {
            if (block_map[j] >= 0) continue;
            if (fabs(RGA[i*n+j]) > block_threshold ||
                fabs(RGA[j*n+i]) > block_threshold) {
                block_map[j] = block_id;
            }
        }
        block_id++;
    }

    /* Assign remaining unassigned rows/cols */
    for (int i = 0; i < n; i++) {
        if (block_map[i] < 0) block_map[i] = block_id++;
    }

    *n_blocks = block_id;
    return 0;
}

/* ===== L5: Extract Sub-Controller Block =====
 *
 * From a full MIMO dynamic matrix, extract the sub-matrix
 * for a specific block of CVs and MVs.
 */

int mpc_extract_sub_controller(const double *A_full, int total_rows, int total_cols,
    const int *cv_block_indices, int n_cv_block,
    const int *mv_block_indices, int n_mv_block,
    double *A_sub)
{
    if (!A_full || !cv_block_indices || !mv_block_indices || !A_sub) return -1;
    if (n_cv_block < 1 || n_mv_block < 1) return -2;

    for (int i = 0; i < n_cv_block; i++) {
        int src_row = cv_block_indices[i];
        if (src_row < 0 || src_row >= total_rows) continue;
        for (int j = 0; j < n_mv_block; j++) {
            int src_col = mv_block_indices[j];
            if (src_col < 0 || src_col >= total_cols) continue;
            A_sub[i * n_mv_block + j] = A_full[src_row * total_cols + src_col];
        }
    }
    return 0;
}

/* ===== L5: Coordinated Sub-Controller Iteration =====
 *
 * For overlapping sub-controllers, use coordinate descent:
 *   While not converged:
 *     For each sub-controller i:
 *       Fix other sub-controller CV predictions at last iterate
 *       Solve QP for sub-controller i only
 *       Update coupling terms
 *
 * Simplified: Gauss-Seidel iteration over sub-controller moves.
 */

int mpc_coordinate_solve_iteration(const double *H_full, const double *c_full,
    int n_total, const int *block_map, int n_blocks,
    double *x_solution, int max_iter, double tol)
{
    if (!H_full || !c_full || !block_map || !x_solution || n_total < 1 || n_blocks < 1)
        return -1;

    memset(x_solution, 0, n_total * sizeof(double));

    for (int iter = 0; iter < max_iter; iter++) {
        double max_change = 0.0;
        double *x_old = (double*)calloc(n_total, sizeof(double));
        if (!x_old) return -2;
        memcpy(x_old, x_solution, n_total * sizeof(double));

        /* For each block, solve reduced system: H_ii * x_i = c_i - sum_{j!=i} H_ij*x_j */
        for (int blk = 0; blk < n_blocks; blk++) {
            /* Count variables in this block */
            int blk_size = 0;
            for (int v = 0; v < n_total; v++)
                if (block_map[v] == blk) blk_size++;
            if (blk_size == 0) continue;

            /* Compute RHS: c_i - sum_{j!=i} H_ij * x_j */
            for (int vi = 0; vi < n_total; vi++) {
                if (block_map[vi] != blk) continue;
                double rhs = c_full[vi];
                for (int vj = 0; vj < n_total; vj++) {
                    if (block_map[vj] == blk) continue;
                    rhs -= H_full[vi * n_total + vj] * x_solution[vj];
                }
                /* Gauss-Seidel update: x_i = -rhs / H_ii (diagonal approximation) */
                double h_ii = H_full[vi * n_total + vi];
                if (fabs(h_ii) > MPC_EPS) {
                    double x_new = -rhs / h_ii;
                    double change = fabs(x_new - x_old[vi]);
                    if (change > max_change) max_change = change;
                    x_solution[vi] = x_new;
                }
            }
        }

        free(x_old);
        if (max_change < tol) break;
    }
    return 0;
}

/* ===== L5: Niederlinski Index for Control Structure Selection =====
 *
 * NI = det(G) / prod(G_ii)
 *
 * Theorem (Niederlinski 1971):
 *   If NI < 0, the system is structurally unstable with
 *   decentralized integral control (at least one pairing
 *   makes the closed loop unstable).
 */

double mpc_niederlinski_index(const double *G_ss, int n)
{
    if (!G_ss || n < 1) return MPC_INF;
    if (n == 1) return 1.0;

    /* Compute determinant of G */
    double det = 0.0;
    if (n == 2) {
        det = G_ss[0]*G_ss[3] - G_ss[1]*G_ss[2];
    } else {
        /* Laplace expansion for small n */
        double *temp = (double*)calloc(n * n, sizeof(double));
        if (!temp) return MPC_INF;
        memcpy(temp, G_ss, n * n * sizeof(double));
        /* Gaussian elimination to upper triangular, det = product of diag */
        for (int k = 0; k < n - 1; k++) {
            if (fabs(temp[k*n+k]) < MPC_EPS) {
                /* Find non-zero below */
                int found = 0;
                for (int r = k+1; r < n; r++) {
                    if (fabs(temp[r*n+k]) > MPC_EPS) {
                        for (int c = 0; c < n; c++) {
                            double t = temp[k*n+c];
                            temp[k*n+c] = temp[r*n+c];
                            temp[r*n+c] = -t;
                        }
                        found = 1; break;
                    }
                }
                if (!found) { free(temp); return 0.0; }
            }
            for (int i = k+1; i < n; i++) {
                double factor = temp[i*n+k] / temp[k*n+k];
                for (int j = k; j < n; j++) temp[i*n+j] -= factor * temp[k*n+j];
            }
        }
        det = 1.0;
        for (int i = 0; i < n; i++) det *= temp[i*n+i];
        free(temp);
    }

    double diag_prod = 1.0;
    for (int i = 0; i < n; i++) diag_prod *= G_ss[i*n+i];

    if (fabs(diag_prod) < MPC_EPS) return MPC_INF;
    return det / diag_prod;
}
