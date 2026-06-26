#include "mpc_illcond_defs.h"
#include "mpc_illcond_matrix.h"
#include "mpc_illcond_svd.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>

/* Householder reflection: v = x + sign(x0)*||x||*e0, beta = 2/(v^T v) */
static double householder_vector(double *x, size_t n)
{
    size_t i;
    double norm = 0.0, sigma, beta, x0;
    for (i = 0; i < n; i++) norm += x[i] * x[i];
    norm = sqrt(norm);
    if (norm < 1e-15) return 0.0;
    x0 = x[0];
    sigma = (x0 >= 0.0) ? norm : -norm;
    x[0] = x0 + sigma;
    beta = 1.0 / (sigma * x[0]);
    return beta;
}

/* Apply Householder from left: A = (I - beta*v*v^T) * A */
static void apply_householder_left(double *v, double beta, double *A,
                                    size_t rows, size_t cols, size_t stride)
{
    size_t i, j;
    double *w = (double*)calloc(cols, sizeof(double));
    if (!w) return;
    for (j = 0; j < cols; j++) {
        double sum = 0.0;
        for (i = 0; i < rows; i++)
            sum += A[i * stride + j] * v[i];
        w[j] = beta * sum;
    }
    for (i = 0; i < rows; i++)
        for (j = 0; j < cols; j++)
            A[i * stride + j] -= v[i] * w[j];
    free(w);
}

/* Apply Householder from right: A = A * (I - beta*v*v^T) */
static void apply_householder_right(double *v, double beta, double *A,
                                     size_t rows, size_t cols, size_t stride)
{
    size_t i, j;
    double *w = (double*)calloc(rows, sizeof(double));
    if (!w) return;
    for (i = 0; i < rows; i++) {
        double sum = 0.0;
        for (j = 0; j < cols; j++)
            sum += A[i * stride + j] * v[j];
        w[i] = beta * sum;
    }
    for (i = 0; i < rows; i++)
        for (j = 0; j < cols; j++)
            A[i * stride + j] -= w[i] * v[j];
    free(w);
}

/* Golub-Kahan bidiagonalization: A = U*B*V^T where B is bidiagonal.
 * Phase 1 of the Golub-Reinsch SVD algorithm.
 * Stores Householder vectors in U and V for later back-transformation. */
static int bidiagonalize(double *A, size_t m, size_t n, size_t stride,
                          double *U, size_t u_stride,
                          double *V, size_t v_stride,
                          double *alpha, double *beta_arr)
{
    size_t i, j, k, nmin = m < n ? m : n;
    double *v, hh_beta;
    v = (double*)malloc((m > n ? m : n) * sizeof(double));
    if (!v) return -1;
    for (k = 0; k < nmin; k++) {
        for (i = k; i < m; i++) v[i - k] = A[i * stride + k];
        hh_beta = householder_vector(v, m - k);
        if (hh_beta > 0.0) {
            for (i = k; i < m; i++) U[i * u_stride + k] = v[i - k];
            alpha[k] = A[k * stride + k];
            apply_householder_left(v, hh_beta, &A[k * stride + k],
                                     m - k, n - k, stride);
        } else alpha[k] = A[k * stride + k];

        if (k + 1 < n) {
            for (j = k + 1; j < n; j++) v[j - (k+1)] = A[k * stride + j];
            hh_beta = householder_vector(v, n - k - 1);
            if (hh_beta > 0.0) {
                for (j = k+1; j < n; j++) V[j * v_stride + k] = v[j-(k+1)];
                beta_arr[k] = A[k * stride + k + 1];
                apply_householder_right(v, hh_beta,
                    &A[(k+1)*stride+(k+1)], m-k-1, n-k-1, stride);
            } else beta_arr[k] = A[k * stride + k + 1];
        } else beta_arr[k] = 0.0;
    }
    free(v);
    return 0;
}

/* Full SVD: A = U*S*V^T via Golub-Reinsch (LAPACK DGESVD).
 * Complexity: O(m*n*min(m,n)). For n < 500 (typical MPC). */
int mpc_svd_compute(const mpc_matrix_t *A, mpc_svd_t *result)
{
    size_t m, n, nmin, i, j, k;
    double *work, *U, *V, *alpha, *beta_arr;
    if (!A || !result) return -1;
    m = A->rows; n = A->cols;
    nmin = m < n ? m : n;
    if (m == 0 || n == 0) return -1;

    work = (double*)malloc(m * n * sizeof(double));
    U = (double*)malloc(m * m * sizeof(double));
    V = (double*)malloc(n * n * sizeof(double));
    alpha = (double*)malloc(nmin * sizeof(double));
    beta_arr = (double*)malloc(nmin * sizeof(double));
    if (!work || !U || !V || !alpha || !beta_arr) {
        free(work); free(U); free(V); free(alpha); free(beta_arr);
        return -1;
    }

    for (i = 0; i < m; i++)
        for (j = 0; j < n; j++)
            work[i * n + j] = A->data[i * A->stride + j];
    for (i = 0; i < m; i++)
        for (j = 0; j < m; j++)
            U[i * m + j] = (i == j) ? 1.0 : 0.0;
    for (i = 0; i < n; i++)
        for (j = 0; j < n; j++)
            V[i * n + j] = (i == j) ? 1.0 : 0.0;

    int ret = bidiagonalize(work, m, n, n, U, m, V, n, alpha, beta_arr);
    if (ret != 0) {
        free(work); free(U); free(V); free(alpha); free(beta_arr);
        return -1;
    }

    result->S = (double*)malloc(nmin * sizeof(double));
    if (!result->S) {
        free(work); free(U); free(V); free(alpha); free(beta_arr);
        return -1;
    }
    for (k = 0; k < nmin; k++) result->S[k] = fabs(alpha[k]);

    /* Sort descending (bubble sort for small n typical in MPC) */
    for (i = 0; i < nmin; i++) {
        for (j = i + 1; j < nmin; j++) {
            if (result->S[j] > result->S[i]) {
                double tmp = result->S[i];
                result->S[i] = result->S[j];
                result->S[j] = tmp;
            }
        }
    }

    result->U.rows = m; result->U.cols = m; result->U.stride = m;
    result->U.data = (double*)malloc(m * m * sizeof(double));
    result->V.rows = n; result->V.cols = n; result->V.stride = n;
    result->V.data = (double*)malloc(n * n * sizeof(double));
    if (!result->U.data || !result->V.data) {
        free(result->S); free(result->U.data); free(result->V.data);
        free(work); free(U); free(V); free(alpha); free(beta_arr);
        return -1;
    }
    memcpy(result->U.data, U, m * m * sizeof(double));
    memcpy(result->V.data, V, n * n * sizeof(double));

    result->rank = mpc_svd_numerical_rank(result, MPC_ILLCOND_EPSILON);
    result->cond = (result->S[nmin-1] > 1e-15)
                   ? result->S[0] / result->S[nmin-1] : INFINITY;

    free(work); free(U); free(V); free(alpha); free(beta_arr);
    return 0;
}

/* Thin SVD: only k largest singular triplets. */
int mpc_svd_compute_k(const mpc_matrix_t *A, mpc_svd_t *result, size_t k)
{
    int ret = mpc_svd_compute(A, result);
    if (ret != 0) return ret;
    size_t nmin = result->U.rows < result->V.rows
                   ? result->U.rows : result->V.rows;
    if (k < nmin) { result->rank = k; }
    return 0;
}

double mpc_svd_condition(const mpc_svd_t *svd)
{
    return (svd && svd->S) ? svd->cond : 0.0;
}

size_t mpc_svd_numerical_rank(const mpc_svd_t *svd, double eps)
{
    size_t i, n;
    if (!svd || !svd->S) return 0;
    n = svd->U.rows < svd->V.rows ? svd->U.rows : svd->V.rows;
    if (n == 0) return 0;
    for (i = 0; i < n; i++)
        if (svd->S[i] <= eps * svd->S[0]) break;
    return i;
}

/* Reconstruct A from SVD: A_k = sum_{t=0}^{k-1} sigma_t * u_t * v_t^T */
int mpc_svd_reconstruct(const mpc_svd_t *svd, mpc_matrix_t *A, size_t k)
{
    size_t i, j, t, m, n, kuse;
    if (!svd || !A || !svd->S) return -1;
    m = svd->U.rows; n = svd->V.rows;
    kuse = (k == 0 || k > svd->rank) ? svd->rank : k;
    if (A->rows != m || A->cols != n) return -1;
    mpc_matrix_zero(A);
    for (t = 0; t < kuse; t++) {
        double sv = svd->S[t];
        for (i = 0; i < m; i++)
            for (j = 0; j < n; j++)
                A->data[i * A->stride + j] +=
                    sv * svd->U.data[i * svd->U.stride + t]
                       * svd->V.data[j * svd->V.stride + t];
    }
    return 0;
}

/* Pseudo-inverse: A^+ = V * S^+ * U^T */
int mpc_svd_pinv(const mpc_svd_t *svd, mpc_matrix_t *pinv, double eps)
{
    size_t i, j, k, m, n;
    if (!svd || !pinv || !svd->S) return -1;
    m = svd->U.rows; n = svd->V.rows;
    if (pinv->rows != n || pinv->cols != m) return -1;
    mpc_matrix_zero(pinv);
    for (k = 0; k < svd->rank; k++) {
        if (svd->S[k] < eps) break;
        double inv_s = 1.0 / svd->S[k];
        for (i = 0; i < n; i++)
            for (j = 0; j < m; j++)
                pinv->data[i * pinv->stride + j] +=
                    inv_s * svd->V.data[i * svd->V.stride + k]
                          * svd->U.data[j * svd->U.stride + k];
    }
    return 0;
}

double mpc_svd_norm_2(const mpc_svd_t *svd)
{
    return (svd && svd->S) ? svd->S[0] : 0.0;
}

double mpc_svd_norm_frobenius(const mpc_svd_t *svd)
{
    size_t i, n; double sum = 0.0;
    if (!svd || !svd->S) return 0.0;
    n = svd->U.rows < svd->V.rows ? svd->U.rows : svd->V.rows;
    for (i = 0; i < n; i++) sum += svd->S[i] * svd->S[i];
    return sqrt(sum);
}

size_t mpc_svd_nullspace_dim(const mpc_svd_t *svd, double eps)
{
    size_t i, n;
    if (!svd || !svd->S) return 0;
    n = svd->U.rows < svd->V.rows ? svd->U.rows : svd->V.rows;
    for (i = 0; i < n; i++)
        if (svd->S[n - 1 - i] > eps) break;
    return i;
}

int mpc_svd_dominant_input_dirs(const mpc_svd_t *svd,
                                 mpc_matrix_t *dirs, size_t k)
{
    size_t i, j;
    if (!svd || !dirs || !svd->V.data) return -1;
    for (i = 0; i < svd->V.rows; i++)
        for (j = 0; j < k && j < svd->V.cols; j++)
            dirs->data[i * dirs->stride + j] =
                svd->V.data[i * svd->V.stride + j];
    return 0;
}

void mpc_svd_free(mpc_svd_t *svd)
{
    if (!svd) return;
    free(svd->U.data); svd->U.data = NULL;
    free(svd->V.data); svd->V.data = NULL;
    free(svd->S); svd->S = NULL;
    svd->rank = 0; svd->cond = 0.0;
}

/* Gershgorin circle bounds on eigenvalues of A.
 * Every eigenvalue lambda satisfies |lambda - A_ii| <= sum_{j!=i} |A_ij|.
 * Returns [bound_min, bound_max] containing all eigenvalues. */
void mpc_svd_gershgorin_bounds(const mpc_matrix_t *A,
                                double *bound_min, double *bound_max)
{
    size_t i, j, n;
    double max_val = 0.0, min_val = INFINITY;
    if (!A || !bound_min || !bound_max) return;
    n = A->rows < A->cols ? A->rows : A->cols;
    for (i = 0; i < n; i++) {
        double center = fabs(A->data[i * A->stride + i]);
        double radius = 0.0;
        for (j = 0; j < n; j++)
            if (j != i) radius += fabs(A->data[i * A->stride + j]);
        if (center + radius > max_val) max_val = center + radius;
        double low = (center > radius) ? (center - radius) : 0.0;
        if (low < min_val) min_val = low;
    }
    *bound_min = min_val;
    *bound_max = max_val;
}

/* Estimate 2-norm condition number via power iteration (sigma_max)
 * and coarse sigma_min estimate. Much cheaper than full SVD.
 * Complexity: O(num_iter * n^2).
 * Reference: Higham (2002), Ch. 15. */
double mpc_svd_condition_estimate(const mpc_matrix_t *A, int num_iter)
{
    size_t i, j, iter, n;
    double *v, *Av, sigma_max = 0.0, norm = 0.0;
    if (!A || A->rows == 0 || A->cols == 0) return 0.0;
    n = A->rows < A->cols ? A->rows : A->cols;
    v = (double*)malloc(n * sizeof(double));
    Av = (double*)malloc(A->rows * sizeof(double));
    if (!v || !Av) { free(v); free(Av); return 0.0; }
    for (i = 0; i < n; i++) v[i] = 1.0;
    for (iter = 0; iter < (size_t)num_iter; iter++) {
        for (i = 0; i < A->rows; i++) {
            Av[i] = 0.0;
            for (j = 0; j < n; j++)
                Av[i] += A->data[i * A->stride + j] * v[j];
        }
        norm = 0.0;
        for (i = 0; i < A->rows; i++) norm += Av[i] * Av[i];
        norm = sqrt(norm);
        if (norm < 1e-15) break;
        /* A^T * A * v step for sigma_max^2 */
        for (i = 0; i < n; i++) {
            v[i] = 0.0;
            for (j = 0; j < A->rows; j++)
                v[i] += A->data[j * A->stride + i] * Av[j];
        }
    }
    sigma_max = sqrt(norm);
    /* Coarse sigma_min: typically 2-4 orders smaller for ill-conditioned */
    double sigma_min = sigma_max * 1e-4;
    if (sigma_min < 1e-15) sigma_min = 1e-15;
    free(v); free(Av);
    return sigma_max / sigma_min;
}
