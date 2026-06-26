/* ssid_svd.c -- SVD Computation and Order Selection for 4SID
 *
 * Reference: Golub & Van Loan (2013), Chapter 8
 *            Van Overschee & De Moor (1996), Section 2.4
 *            Ljung (1999), Section 16.4-16.5
 *
 * Implements the SVD-based core of subspace identification: computing
 * the singular value decomposition of the weighted projection matrix,
 * selecting the system order from the singular values, and extracting
 * system matrices {A,B,C,D} from the SVD factors.
 */

#include "ssid_svd.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <float.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ========================================================================
 * L4: SVD Computation via Bidiagonalization + QR
 *
 * Knowledge point: The SVD decomposes a matrix A (m x n) into:
 *   A = U * S * V^T
 *
 * U: m x k orthogonal (left singular vectors)
 * S: k x k diagonal (singular values, descending)
 * V: n x k orthogonal (right singular vectors)
 *
 * For subspace identification, SVD is applied to the weighted
 * projection O_i (i*n_y x j). The rank of O_i equals the system
 * order n_x, and:
 *   U1 = first n_x columns of U -> spans observability subspace
 *   S1 = first n_x singular values -> signal strength
 *   V1 = first n_x columns of V -> state sequence directions
 *
 * Implementation: Two-phase algorithm
 *   Phase 1: Bidiagonalize A via Golub-Kahan Householder reflections
 *   Phase 2: Implicit QR on bidiagonal matrix (Golub-Reinsch)
 *
 * For this reference implementation, we use a simplified approach:
 * compute the Gram matrix A * A^T, eigendecompose (which for symmetric
 * positive semidefinite matrices is equivalent to SVD of A), and
 * reconstruct U, S, V from the eigenvectors/eigenvalues.
 *
 * The eigenvalues lambda_i of A*A^T equal s_i^2 (squared singular values),
 * and the eigenvectors of A*A^T are the left singular vectors U.
 * ======================================================================== */

/* Helper: Jacobi eigenvalue algorithm for small symmetric matrices.
 * Robust for matrices up to ~20x20 (beyond that, use QR or divide-and-conquer).
 * Complexity: O(n^3) per sweep.
 *
 * Knowledge point: Jacobi's method (1846) is the most accurate eigenvalue
 * algorithm for symmetric positive definite matrices. It produces
 * eigenvectors that are numerically orthogonal to machine precision,
 * which is critical for extracting the observability basis. */
static void jacobi_eigen(double *A_sym, size_t n, size_t stride,
                         double *eigenvalues, double *eigenvectors)
{
    const size_t max_sweeps = 50;
    const double tol = 1e-12;

    /* Initialize eigenvectors to identity */
    for (size_t i = 0; i < n; i++) {
        for (size_t j = 0; j < n; j++) {
            eigenvectors[i * n + j] = (i == j) ? 1.0 : 0.0;
        }
    }

    for (size_t sweep = 0; sweep < max_sweeps; sweep++) {
        double max_off = 0.0;
        for (size_t i = 0; i < n; i++) {
            for (size_t j = i + 1; j < n; j++) {
                double a_ii = A_sym[i * stride + i];
                double a_jj = A_sym[j * stride + j];
                double a_ij = A_sym[j * stride + i];

                double off = fabs(a_ij);
                if (off > max_off) max_off = off;

                if (off < tol * (fabs(a_ii) + fabs(a_jj))) continue;

                /* Compute Jacobi rotation */
                double theta = 0.5 * (a_jj - a_ii) / a_ij;
                double t = 1.0 / (fabs(theta) + sqrt(1.0 + theta * theta));
                if (theta < 0) t = -t;
                double c = 1.0 / sqrt(1.0 + t * t);
                double s = c * t;

                /* Update A: A = J^T * A * J */
                /* Row updates */
                for (size_t k = 0; k < n; k++) {
                    double aik = (k > i) ? A_sym[k * stride + i] : A_sym[i * stride + k];
                    double ajk = (k > j) ? A_sym[k * stride + j] : A_sym[j * stride + k];
                    if (k > i) {
                        A_sym[k * stride + i] = c * aik - s * ajk;
                    } else {
                        A_sym[i * stride + k] = c * aik - s * ajk;
                    }
                    if (k > j) {
                        A_sym[k * stride + j] = s * aik + c * ajk;
                    } else {
                        A_sym[j * stride + k] = s * aik + c * ajk;
                    }
                }

                /* Column updates for diagonal blocks */
                for (size_t k = 0; k < n; k++) {
                    double aki = A_sym[i * stride + k];
                    double akj = A_sym[j * stride + k];
                    A_sym[i * stride + k] = c * aki - s * akj;
                    A_sym[j * stride + k] = s * aki + c * akj;
                }

                /* Rotate eigenvectors */
                for (size_t k = 0; k < n; k++) {
                    double v_ki = eigenvectors[k * n + i];
                    double v_kj = eigenvectors[k * n + j];
                    eigenvectors[k * n + i] = c * v_ki - s * v_kj;
                    eigenvectors[k * n + j] = s * v_ki + c * v_kj;
                }
            }
        }
        if (max_off < tol) break;
    }

    /* Extract eigenvalues from diagonal */
    for (size_t i = 0; i < n; i++) {
        eigenvalues[i] = A_sym[i * stride + i];
    }

    /* Sort eigenvalues and eigenvectors in descending order */
    for (size_t i = 0; i < n - 1; i++) {
        size_t max_idx = i;
        for (size_t j = i + 1; j < n; j++) {
            if (eigenvalues[j] > eigenvalues[max_idx]) max_idx = j;
        }
        if (max_idx != i) {
            double tmp = eigenvalues[i];
            eigenvalues[i] = eigenvalues[max_idx];
            eigenvalues[max_idx] = tmp;
            for (size_t k = 0; k < n; k++) {
                double tv = eigenvectors[k * n + i];
                eigenvectors[k * n + i] = eigenvectors[k * n + max_idx];
                eigenvectors[k * n + max_idx] = tv;
            }
        }
    }
}

/* Compute SVD via Gram matrix A*A^T eigendecomposition.
 *
 * Complexity: O(m * n * min(m,n) + min(m,n)^3). */
ssid_svd_t ssid_svd_compute(const ssid_matrix_t *A)
{
    ssid_svd_t svd;
    memset(&svd, 0, sizeof(svd));

    if (!A || !A->data) return svd;

    size_t m = A->rows;
    size_t n = A->cols;
    size_t k_min = (m < n) ? m : n;

    if (k_min == 0) return svd;

    svd.m = m;
    svd.n = n;
    svd.k = k_min;
    svd.s = (double *)calloc(k_min, sizeof(double));

    /* For tall matrices (m >> n), use A^T * A which is n x n */
    /* For wide matrices (n >> m), use A * A^T which is m x m */
    int use_ATA = (m >= n);
    size_t rows_gram = use_ATA ? n : m;

    double *gram = (double *)calloc(rows_gram * rows_gram, sizeof(double));
    double *eigvec = (double *)calloc(rows_gram * rows_gram, sizeof(double));
    double *eigval = (double *)calloc(rows_gram, sizeof(double));

    if (!gram || !eigvec || !eigval || !svd.s) {
        free(gram); free(eigvec); free(eigval);
        if (!svd.s) free(svd.s);
        svd.s = NULL; svd.k = 0;
        return svd;
    }

    if (use_ATA) {
        /* Compute A^T * A (n x n) */
        for (size_t a = 0; a < n; a++) {
            for (size_t b = 0; b < n; b++) {
                double sum = 0.0;
                for (size_t c = 0; c < m; c++) {
                    sum += A->data[a * A->stride + c] *
                           A->data[b * A->stride + c];
                }
                gram[b * n + a] = sum;
            }
        }
        jacobi_eigen(gram, n, n, eigval, eigvec);

        /* s_i = sqrt(lambda_i), U = A * V * S^{-1} */
        for (size_t i = 0; i < k_min; i++) {
            svd.s[i] = (eigval[i] > 1e-16) ? sqrt(eigval[i]) : 0.0;
        }

        /* Build U (m x k) */
        svd.U = ssid_matrix_alloc(m, k_min);
        if (svd.U.data) {
            for (size_t j = 0; j < k_min; j++) {
                if (svd.s[j] > 1e-16) {
                    for (size_t i = 0; i < m; i++) {
                        double sum = 0.0;
                        for (size_t k = 0; k < n; k++) {
                            sum += A->data[k * A->stride + i] * eigvec[k * n + j];
                        }
                        svd.U.data[j * svd.U.stride + i] = sum / svd.s[j];
                    }
                }
            }
        }

        /* Build V (n x k) from eigenvectors */
        svd.V = ssid_matrix_alloc(n, k_min);
        if (svd.V.data) {
            for (size_t j = 0; j < k_min; j++) {
                for (size_t i = 0; i < n; i++) {
                    svd.V.data[j * svd.V.stride + i] = eigvec[i * n + j];
                }
            }
        }
    } else {
        /* Compute A * A^T (m x m) */
        for (size_t a = 0; a < m; a++) {
            for (size_t b = 0; b < m; b++) {
                double sum = 0.0;
                for (size_t c = 0; c < n; c++) {
                    sum += A->data[c * A->stride + a] *
                           A->data[c * A->stride + b];
                }
                gram[b * m + a] = sum;
            }
        }
        jacobi_eigen(gram, m, m, eigval, eigvec);

        for (size_t i = 0; i < k_min; i++) {
            svd.s[i] = (eigval[i] > 1e-16) ? sqrt(eigval[i]) : 0.0;
        }

        /* U from eigenvectors */
        svd.U = ssid_matrix_alloc(m, k_min);
        if (svd.U.data) {
            for (size_t j = 0; j < k_min; j++) {
                for (size_t i = 0; i < m; i++) {
                    svd.U.data[j * svd.U.stride + i] = eigvec[i * m + j];
                }
            }
        }

        /* V = A^T * U * S^{-1} */
        svd.V = ssid_matrix_alloc(n, k_min);
        if (svd.V.data) {
            for (size_t j = 0; j < k_min; j++) {
                if (svd.s[j] > 1e-16) {
                    for (size_t i = 0; i < n; i++) {
                        double sum = 0.0;
                        for (size_t k = 0; k < m; k++) {
                            sum += A->data[i * A->stride + k] *
                                   svd.U.data[j * svd.U.stride + k];
                        }
                        svd.V.data[j * svd.V.stride + i] = sum / svd.s[j];
                    }
                }
            }
        }
    }

    free(gram);
    free(eigvec);
    free(eigval);

    return svd;
}

/* Truncate SVD to keep k largest singular values. */
ssid_svd_t ssid_svd_truncate(const ssid_svd_t *svd, size_t k)
{
    ssid_svd_t result;
    memset(&result, 0, sizeof(result));

    if (!svd || k == 0 || k > svd->k) k = svd->k;

    result.m = svd->m;
    result.n = svd->n;
    result.k = k;

    result.s = (double *)malloc(k * sizeof(double));
    if (!result.s) return result;

    memcpy(result.s, svd->s, k * sizeof(double));

    /* Truncate U */
    result.U = ssid_matrix_alloc(svd->m, k);
    if (result.U.data && svd->U.data) {
        for (size_t j = 0; j < k; j++) {
            memcpy(&result.U.data[j * result.U.stride],
                   &svd->U.data[j * svd->U.stride],
                   svd->m * sizeof(double));
        }
    }

    /* Truncate V */
    result.V = ssid_matrix_alloc(svd->n, k);
    if (result.V.data && svd->V.data) {
        for (size_t j = 0; j < k; j++) {
            memcpy(&result.V.data[j * result.V.stride],
                   &svd->V.data[j * svd->V.stride],
                   svd->n * sizeof(double));
        }
    }

    return result;
}

/* Reconstruct matrix from truncated SVD: A_k = U_k * S_k * V_k^T.
 *
 * Eckart-Young theorem: This is the optimal rank-k approximation. */
ssid_matrix_t ssid_svd_reconstruct(const ssid_svd_t *svd, size_t k)
{
    ssid_matrix_t A_k;
    memset(&A_k, 0, sizeof(A_k));

    if (!svd || k == 0 || k > svd->k) return A_k;
    if (!svd->U.data || !svd->V.data) return A_k;

    A_k = ssid_matrix_alloc(svd->m, svd->n);
    if (!A_k.data) return A_k;

    for (size_t j = 0; j < svd->n; j++) {
        for (size_t i = 0; i < svd->m; i++) {
            double sum = 0.0;
            for (size_t p = 0; p < k; p++) {
                sum += svd->U.data[p * svd->U.stride + i] *
                       svd->s[p] *
                       svd->V.data[p * svd->V.stride + j];
            }
            A_k.data[j * A_k.stride + i] = sum;
        }
    }
    return A_k;
}

/* ========================================================================
 * L5: Order Selection Methods
 * ======================================================================== */

/* SVD gap criterion: find largest relative gap s_{k-1} / s_k.
 *
 * Knowledge point: In the absence of noise, singular values drop to
 * zero at index n_x+1 (the true system order). With noise, there is
 * no sharp drop, but the ratio s_k / s_{k+1} peaks near the true order.
 *
 * The "scree plot" (Cattell, 1966) visualizes this: singular values
 * vs. index, with an "elbow" indicating the signal/noise boundary.
 *
 * This heuristic is widely used in practice because:
 *   - It requires no distributional assumptions
 *   - It works for short data records
 *   - It is visually intuitive for engineers
 * But it can fail when singular values decay gradually.
 */
size_t ssid_order_svd_gap(const double *s, size_t k, double rel_tol)
{
    if (!s || k < 2) return (k > 0) ? 1 : 0;
    if (rel_tol <= 0.0) rel_tol = 3.0;

    size_t best_n = 1;
    double best_gap = 0.0;

    /* Normalize by s[0] */
    double s0 = s[0];
    if (s0 < 1e-16) return 1;

    for (size_t i = 1; i < k; i++) {
        double gap = s[i - 1] / (s[i] + 1e-16 * s0);
        if (gap > best_gap && gap > rel_tol) {
            best_gap = gap;
            best_n = i;
        }
    }

    return best_n;
}

/* AIC criterion.
 *
 * AIC(n) = -2 * log(L_max) + 2 * p_n
 *
 * For Gaussian innovations with variance estimate sigma_n^2:
 * AIC(n) = N * n_y * log(2*pi) + N * sum(log(sigma_i^2)) + N * n_y
 *          + 2 * (n*(n_y+n_u) + n*n_y + n_y*n_u)
 *
 * Simplified: approximate sigma_n^2 from residual singular values. */
size_t ssid_order_aic(const double *s, size_t k, size_t N,
                      size_t n_y, size_t n_u,
                      double *aic_values)
{
    if (!s || k == 0 || N == 0) return 1;

    double total_var = 0.0;
    for (size_t i = 0; i < k; i++) {
        total_var += s[i] * s[i];
    }

    /* Number of free parameters per order n:
     * A: n*n, B: n*n_u, C: n_y*n, D: n_y*n_u */
    size_t best_n = 1;
    double best_aic = 1e308;

    for (size_t n = 1; n <= k; n++) {
        double signal_var = 0.0;
        for (size_t i = 0; i < n; i++) {
            signal_var += s[i] * s[i];
        }
        double noise_var = total_var - signal_var;
        if (noise_var < 1e-16) noise_var = 1e-16;

        /* Log-likelihood proxy */
        double log_L = -0.5 * (double)N * (double)n_y *
                        log(noise_var / (double)(k - n + 1));

        /* Parameter count */
        size_t p = n * n + n * n_u + n_y * n + n_y * n_u;

        double aic = -2.0 * log_L + 2.0 * (double)p;

        if (aic_values) aic_values[n - 1] = aic;

        if (aic < best_aic) {
            best_aic = aic;
            best_n = n;
        }
    }

    return best_n;
}

/* BIC criterion: stronger penalty than AIC, asymptotically consistent. */
size_t ssid_order_bic(const double *s, size_t k, size_t N,
                      size_t n_y, size_t n_u,
                      double *bic_values)
{
    if (!s || k == 0 || N == 0) return 1;

    double total_var = 0.0;
    for (size_t i = 0; i < k; i++) {
        total_var += s[i] * s[i];
    }

    size_t best_n = 1;
    double best_bic = 1e308;
    double log_N = log((double)N);

    for (size_t n = 1; n <= k; n++) {
        double signal_var = 0.0;
        for (size_t i = 0; i < n; i++) {
            signal_var += s[i] * s[i];
        }
        double noise_var = total_var - signal_var;
        if (noise_var < 1e-16) noise_var = 1e-16;

        double log_L = -0.5 * (double)N * (double)n_y *
                        log(noise_var / (double)(k - n + 1));

        size_t p = n * n + n * n_u + n_y * n + n_y * n_u;

        double bic = -2.0 * log_L + log_N * (double)p;

        if (bic_values) bic_values[n - 1] = bic;

        if (bic < best_bic) {
            best_bic = bic;
            best_n = n;
        }
    }

    return best_n;
}

/* MDL criterion: minimum description length (Rissanen). */
size_t ssid_order_mdl(const double *s, size_t k, size_t N,
                      size_t n_y, size_t n_u,
                      double *mdl_values)
{
    /* MDL is structurally similar to BIC for large N.
     * MDL(n) = -log(L) + 0.5 * p * log(N)
     * where p = total free parameters. */
    if (!s || k == 0 || N == 0) return 1;

    double total_var = 0.0;
    for (size_t i = 0; i < k; i++) total_var += s[i] * s[i];

    size_t best_n = 1;
    double best_mdl = 1e308;
    double log_N = log((double)N);

    for (size_t n = 1; n <= k; n++) {
        double signal_var = 0.0;
        for (size_t i = 0; i < n; i++) signal_var += s[i] * s[i];
        double noise_var = total_var - signal_var;
        if (noise_var < 1e-16) noise_var = 1e-16;

        double log_L = -0.5 * (double)N * (double)n_y *
                        log(noise_var / (double)(k - n + 1));
        size_t p = n * n + n * n_u + n_y * n + n_y * n_u;
        double mdl = -log_L + 0.5 * log_N * (double)p;

        if (mdl_values) mdl_values[n - 1] = mdl;
        if (mdl < best_mdl) { best_mdl = mdl; best_n = n; }
    }

    return best_n;
}

/* Unified order selection dispatcher. */
size_t ssid_order_select(const double *s, size_t k,
                         const ssid_config_t *cfg,
                         size_t N, size_t n_y, size_t n_u,
                         double *criteria_out)
{
    if (!s || !cfg || k == 0) return 1;

    size_t order = 1;
    size_t k_use = k;

    if (cfg->n_x_max > 0 && cfg->n_x_max < k_use) k_use = cfg->n_x_max;
    if (cfg->n_x_min > 0 && cfg->n_x_min > k_use) k_use = cfg->n_x_min;

    switch (cfg->order_crit) {
        case SSID_ORDER_SVD_GAP:
            order = ssid_order_svd_gap(s, k_use, cfg->svd_tol > 0 ? cfg->svd_tol * 1e5 : 3.0);
            break;
        case SSID_ORDER_AIC:
            order = ssid_order_aic(s, k_use, N, n_y, n_u, criteria_out);
            break;
        case SSID_ORDER_BIC:
            order = ssid_order_bic(s, k_use, N, n_y, n_u, criteria_out);
            break;
        case SSID_ORDER_MDL:
            order = ssid_order_mdl(s, k_use, N, n_y, n_u, criteria_out);
            break;
        case SSID_ORDER_CROSSVAL:
            /* Cross-validation is handled externally (needs train/test split) */
            order = ssid_order_svd_gap(s, k_use, 3.0);
            break;
        default:
            order = ssid_order_svd_gap(s, k_use, 3.0);
            break;
    }

    /* Clamp to configured range */
    if (cfg->n_x_min > 0 && order < cfg->n_x_min) order = cfg->n_x_min;
    if (cfg->n_x_max > 0 && order > cfg->n_x_max) order = cfg->n_x_max;

    return order;
}

/* Effective numerical rank. */
size_t ssid_svd_effective_rank(const double *s, size_t k,
                               size_t m, size_t n, double tol_factor)
{
    if (!s || k == 0) return 0;
    if (tol_factor <= 0.0) tol_factor = 1.0;

    double eps = DBL_EPSILON;
    double threshold = (double)((m > n) ? m : n) * eps * s[0] * tol_factor;

    size_t rank = 0;
    for (size_t i = 0; i < k; i++) {
        if (s[i] > threshold) rank++;
        else break;
    }
    return rank;
}

/* Extract observability matrix from SVD.
 *
 * Gamma_i = U_n * sqrt(S_n)
 * where U_n = first n_x columns of U, S_n = first n_x singular values.
 *
 * Gamma_i is the extended observability matrix:
 *   Gamma_i = [C; C*A; ...; C*A^{i-1}]  (size i*n_y x n_x) */
int ssid_svd_extract_observability(const ssid_svd_t *svd,
                                   size_t n_x,
                                   ssid_matrix_t *Gamma)
{
    if (!svd || !svd->U.data || !svd->s || n_x == 0 || n_x > svd->k) return -1;
    if (!Gamma || !Gamma->data) return -2;
    if (Gamma->rows < svd->m || Gamma->cols < n_x) return -3;

    for (size_t j = 0; j < n_x; j++) {
        double sqrt_s = sqrt(svd->s[j]);
        for (size_t i = 0; i < svd->m; i++) {
            Gamma->data[j * Gamma->stride + i] =
                svd->U.data[j * svd->U.stride + i] * sqrt_s;
        }
    }
    return 0;
}

/* Extract state sequence from SVD.
 *
 * X_hat_i = sqrt(S_n) * V_n^T
 * size n_x x j, where j is the number of columns (time instants). */
int ssid_svd_extract_state(const ssid_svd_t *svd,
                           size_t n_x,
                           ssid_matrix_t *X)
{
    if (!svd || !svd->V.data || !svd->s || n_x == 0 || n_x > svd->k) return -1;
    if (!X || !X->data) return -2;
    if (X->rows < n_x || X->cols < svd->n) return -3;

    for (size_t j = 0; j < svd->n; j++) {
        for (size_t i = 0; i < n_x; i++) {
            double sqrt_s = sqrt(svd->s[i]);
            X->data[j * X->stride + i] =
                sqrt_s * svd->V.data[i * svd->V.stride + j];
        }
    }
    return 0;
}

/* Estimate A and C via shift-invariance.
 *
 * Gamma_i_up * A = Gamma_i_down  (shift-invariance)
 * where Gamma_i_up = first (i-1)*n_y rows of Gamma_i
 *       Gamma_i_down = last (i-1)*n_y rows of Gamma_i
 *
 * C = first n_y rows of Gamma_i.
 *
 * Knowledge point: Shift-invariance is the property that makes
 * subspace identification possible. The observability matrix has
 * a nested block structure:
 *
 *   Gamma_i = [ C     ]    Gamma_i_up   = [ C       ]
 *              [ C*A   ]                   [ C*A     ]
 *              [ C*A^2 ]                   [ ...     ]
 *              [ ...   ]                   [ C*A^{i-2}]
 *              [ C*A^{i-1}]
 *
 * So Gamma_i_up * A = Gamma_i_down, which is a linear system.
 * Solve via least squares (or total least squares for EIV models). */
int ssid_svd_estimate_AC(const ssid_matrix_t *Gamma,
                         size_t n_x, size_t n_y,
                         ssid_matrix_t *A_est,
                         ssid_matrix_t *C_est)
{
    if (!Gamma || !A_est || !C_est || !Gamma->data) return -1;

    size_t total_rows = Gamma->rows;
    if (total_rows < 2 * n_y) return -2;

    size_t rows_up = total_rows - n_y;
    size_t rows_down = total_rows - n_y;

    /* Build Gamma_up and Gamma_down */
    /* Gamma_up: rows 0 .. total_rows-n_y-1 */
    /* Gamma_down: rows n_y .. total_rows-1 */
    ssid_matrix_t G_up = ssid_matrix_view(
        Gamma->data, rows_up, Gamma->cols, Gamma->stride);
    G_up.owner = 0;

    ssid_matrix_t G_down = ssid_matrix_view(
        Gamma->data + n_y, rows_down, Gamma->cols, Gamma->stride);
    G_down.owner = 0;

    /* Least squares: G_up * A = G_down => A = pinv(G_up) * G_down
     * Solve: G_up^T * G_up * A = G_up^T * G_down */
    ssid_matrix_t GupTGup = ssid_matrix_alloc(n_x, n_x);
    ssid_matrix_t GupTGdown = ssid_matrix_alloc(n_x, n_x);
    if (!GupTGup.data || !GupTGdown.data) {
        ssid_matrix_free(&GupTGup);
        ssid_matrix_free(&GupTGdown);
        return -3;
    }

    /* G_up^T * G_up */
    for (size_t a = 0; a < n_x; a++) {
        for (size_t b = 0; b < n_x; b++) {
            double sum = 0.0;
            for (size_t c = 0; c < rows_up; c++) {
                sum += G_up.data[a * G_up.stride + c] *
                       G_up.data[b * G_up.stride + c];
            }
            GupTGup.data[b * GupTGup.stride + a] = sum;
        }
    }

    /* G_up^T * G_down */
    for (size_t a = 0; a < n_x; a++) {
        for (size_t b = 0; b < n_x; b++) {
            double sum = 0.0;
            for (size_t c = 0; c < rows_up; c++) {
                sum += G_up.data[a * G_up.stride + c] *
                       G_down.data[b * G_down.stride + c];
            }
            GupTGdown.data[b * GupTGdown.stride + a] = sum;
        }
    }

    /* Regularize */
    for (size_t i = 0; i < n_x; i++) {
        GupTGup.data[i * GupTGup.stride + i] += 1e-12;
    }

    /* Solve for A (stored in GupTGdown) */
    int status = ssid_matrix_solve(&GupTGup, &GupTGdown);

    /* A_est = GupTGdown^T (solution was in column form) */
    for (size_t j = 0; j < n_x; j++) {
        for (size_t i = 0; i < n_x; i++) {
            A_est->data[j * A_est->stride + i] =
                GupTGdown.data[j * GupTGdown.stride + i];
        }
    }

    /* C_est = first n_y rows of Gamma */
    for (size_t j = 0; j < n_x; j++) {
        for (size_t i = 0; i < n_y; i++) {
            C_est->data[j * C_est->stride + i] =
                Gamma->data[j * Gamma->stride + i];
        }
    }

    ssid_matrix_free(&GupTGup);
    ssid_matrix_free(&GupTGdown);

    return status;
}

/* Estimate B and D from linear regression.
 *
 * After A, C, and X are known, B and D are obtained from:
 *   y_k - C*x_k = D*u_k + e_k
 *   x_{k+1} - A*x_k = B*u_k + K*e_k
 *
 * Solving for D first (output equation), then B (state equation).
 * This is a linear least squares problem. */
int ssid_svd_estimate_BD(const ssid_matrix_t *X,
                         const ssid_matrix_t *Y,
                         const ssid_matrix_t *U,
                         const ssid_matrix_t *A_est,
                         const ssid_matrix_t *C_est,
                         ssid_matrix_t *B_est,
                         ssid_matrix_t *D_est)
{
    if (!X || !Y || !U || !A_est || !C_est || !B_est || !D_est) return -1;

    size_t n_x = A_est->rows;
    size_t n_u = U->cols;
    size_t n_y = Y->cols;
    size_t N_data = X->cols - 1; /* Use columns 0..N-2 for regression */

    if (N_data < n_u) return -2;

    /* Step 1: Estimate D.
     * y_k = C*x_k + D*u_k => D*u_k = y_k - C*x_k
     * Form least squares: U^T * U * D^T = U^T * (Y - X^T*C^T)
     *
     * RHS = Y - C*X */
    ssid_matrix_t RHS = ssid_matrix_alloc(N_data, n_y);
    if (!RHS.data) return -3;

    for (size_t k = 0; k < N_data; k++) {
        for (size_t j = 0; j < n_y; j++) {
            double cx = 0.0;
            for (size_t i = 0; i < n_x; i++) {
                cx += C_est->data[i * C_est->stride + j] *
                      X->data[k * X->stride + i];
            }
            RHS.data[j * RHS.stride + k] =
                Y->data[j * Y->stride + k] - cx;
        }
    }

    /* UTU * D^T = U^T * RHS -> simple LS */
    ssid_matrix_t UTU = ssid_matrix_alloc(n_u, n_u);
    ssid_matrix_t UTYCX = ssid_matrix_alloc(n_y, n_u);
    if (!UTU.data || !UTYCX.data) {
        ssid_matrix_free(&RHS); ssid_matrix_free(&UTU);
        ssid_matrix_free(&UTYCX); return -4;
    }

    for (size_t a = 0; a < n_u; a++) {
        for (size_t b = 0; b < n_u; b++) {
            double sum = 0.0;
            for (size_t k = 0; k < N_data; k++) sum += U->data[a * U->stride + k] * U->data[b * U->stride + k];
            UTU.data[b * UTU.stride + a] = sum;
        }
    }

    for (size_t a = 0; a < n_u; a++) {
        for (size_t b = 0; b < n_y; b++) {
            double sum = 0.0;
            for (size_t k = 0; k < N_data; k++) sum += U->data[a * U->stride + k] * RHS.data[b * RHS.stride + k];
            UTYCX.data[b * UTYCX.stride + a] = sum;
        }
    }

    for (size_t i = 0; i < n_u; i++) UTU.data[i * UTU.stride + i] += 1e-10;

    ssid_matrix_t D_trans = UTYCX;
    int status = ssid_matrix_solve(&UTU, &D_trans);
    if (status == 0) {
        for (size_t j = 0; j < n_u; j++) {
            for (size_t i = 0; i < n_y; i++) {
                D_est->data[j * D_est->stride + i] =
                    D_trans.data[j * D_trans.stride + i];
            }
        }
    }

    /* Step 2: Estimate B.
     * x_{k+1} - A*x_k = B*u_k
     * Same LS structure as D. */
    ssid_matrix_t XRHS = ssid_matrix_alloc(N_data, n_x);
    if (!XRHS.data) {
        ssid_matrix_free(&RHS); ssid_matrix_free(&UTU);
        ssid_matrix_free(&UTYCX); return -5;
    }

    for (size_t k = 0; k < N_data; k++) {
        for (size_t i = 0; i < n_x; i++) {
            double ax = 0.0;
            for (size_t j = 0; j < n_x; j++) {
                ax += A_est->data[j * A_est->stride + i] *
                      X->data[k * X->stride + j];
            }
            XRHS.data[i * XRHS.stride + k] =
                X->data[(k + 1) * X->stride + i] - ax;
        }
    }

    ssid_matrix_t UTU2 = ssid_matrix_alloc(n_u, n_u);
    ssid_matrix_t UTX = ssid_matrix_alloc(n_x, n_u);
    if (!UTU2.data || !UTX.data) {
        status = -6;
    } else {
        for (size_t a = 0; a < n_u; a++) {
            for (size_t b = 0; b < n_u; b++) {
                double sum = 0.0;
                for (size_t k = 0; k < N_data; k++) sum += U->data[a * U->stride + k] * U->data[b * U->stride + k];
                UTU2.data[b * UTU2.stride + a] = sum;
            }
        }
        for (size_t a = 0; a < n_u; a++) {
            for (size_t b = 0; b < n_x; b++) {
                double sum = 0.0;
                for (size_t k = 0; k < N_data; k++) sum += U->data[a * U->stride + k] * XRHS.data[b * XRHS.stride + k];
                UTX.data[b * UTX.stride + a] = sum;
            }
        }
        for (size_t i = 0; i < n_u; i++) UTU2.data[i * UTU2.stride + i] += 1e-10;

        ssid_matrix_t B_trans = UTX;
        status = ssid_matrix_solve(&UTU2, &B_trans);
        if (status == 0) {
            for (size_t j = 0; j < n_u; j++) {
                for (size_t i = 0; i < n_x; i++) {
                    B_est->data[j * B_est->stride + i] =
                        B_trans.data[j * B_trans.stride + i];
                }
            }
        }
        ssid_matrix_free(&UTU2);
        ssid_matrix_free(&UTX);
    }

    ssid_matrix_free(&RHS);
    ssid_matrix_free(&UTU);
    ssid_matrix_free(&UTYCX);
    ssid_matrix_free(&XRHS);

    return status;
}

void ssid_svd_free(ssid_svd_t *svd)
{
    if (!svd) return;
    ssid_matrix_free(&svd->U);
    ssid_matrix_free(&svd->V);
    if (svd->s) { free(svd->s); svd->s = NULL; }
    svd->k = 0;
}

void ssid_svd_print_singular_values(const ssid_svd_t *svd)
{
    if (!svd || !svd->s) return;
    printf("Singular values (k=%lu):\n", (unsigned long)svd->k);
    for (size_t i = 0; i < svd->k && i < 50; i++) {
        double gap = (i > 0 && svd->s[i] > 1e-16) ?
                     svd->s[i-1] / svd->s[i] : 0.0;
        printf("  s[%2lu] = %12.6e  (gap: %8.2f)\n", (unsigned long)i, svd->s[i], gap);
    }
}

/* Condition number of weighting matrix (for stability analysis). */
double ssid_svd_weighting_condition(const ssid_matrix_t *W)
{
    if (!W || !W->data || W->rows == 0 || W->cols == 0) return 1.0;
    ssid_svd_t svd_w = ssid_svd_compute(W);
    if (svd_w.k == 0 || !svd_w.s) { ssid_svd_free(&svd_w); return 1e16; }
    double cond = (svd_w.s[svd_w.k-1] > 1e-16) ?
                  svd_w.s[0] / svd_w.s[svd_w.k-1] : 1e16;
    ssid_svd_free(&svd_w);
    return cond;
}
