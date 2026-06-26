#include "mpc_illcond_defs.h"
#include "mpc_illcond_matrix.h"
#include "mpc_illcond_svd.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>

mpc_matrix_t* mpc_matrix_alloc(size_t rows, size_t cols)
{
    mpc_matrix_t *mat;
    if (rows == 0 || cols == 0) return NULL;
    mat = (mpc_matrix_t*)malloc(sizeof(mpc_matrix_t));
    if (!mat) return NULL;
    mat->rows = rows; mat->cols = cols; mat->stride = cols;
    mat->data = (double*)calloc(rows * cols, sizeof(double));
    if (!mat->data) { free(mat); return NULL; }
    return mat;
}

void mpc_matrix_free(mpc_matrix_t **mat)
{
    if (!mat || !*mat) return;
    free((*mat)->data); free(*mat); *mat = NULL;
}

mpc_matrix_t* mpc_matrix_copy(const mpc_matrix_t *src)
{
    mpc_matrix_t *dst;
    if (!src) return NULL;
    dst = mpc_matrix_alloc(src->rows, src->cols);
    if (!dst) return NULL;
    dst->stride = src->stride;
    memcpy(dst->data, src->data, src->rows * src->stride * sizeof(double));
    return dst;
}

void mpc_matrix_zero(mpc_matrix_t *mat)
{
    if (!mat || !mat->data) return;
    memset(mat->data, 0, mat->rows * mat->stride * sizeof(double));
}

int mpc_matrix_eye(mpc_matrix_t *mat)
{
    size_t i;
    if (!mat || mat->rows != mat->cols) return -1;
    mpc_matrix_zero(mat);
    for (i = 0; i < mat->rows; i++)
        mat->data[i * mat->stride + i] = 1.0;
    return 0;
}

void mpc_matrix_get_diag(const mpc_matrix_t *mat, double *diag)
{
    size_t i, n;
    if (!mat || !diag) return;
    n = mat->rows < mat->cols ? mat->rows : mat->cols;
    for (i = 0; i < n; i++)
        diag[i] = mat->data[i * mat->stride + i];
}

void mpc_matrix_set_diag(mpc_matrix_t *mat, const double *diag)
{
    size_t i, n;
    if (!mat || !diag) return;
    n = mat->rows < mat->cols ? mat->rows : mat->cols;
    for (i = 0; i < n; i++)
        mat->data[i * mat->stride + i] = diag[i];
}

/* Frobenius norm: ||A||_F = sqrt(sum A_ij^2). Lower bound for kappa_2. */
double mpc_matrix_norm_frobenius(const mpc_matrix_t *mat)
{
    size_t i, j; double sum = 0.0, val;
    if (!mat) return 0.0;
    for (i = 0; i < mat->rows; i++)
        for (j = 0; j < mat->cols; j++) {
            val = mat->data[i * mat->stride + j];
            sum += val * val;
        }
    return sqrt(sum);
}

/* 1-norm: maximum absolute column sum. Induced by vector 1-norm.
 * Used in Hager-Higham condition estimation (LAPACK DLANGE). */
double mpc_matrix_norm_1(const mpc_matrix_t *mat)
{
    size_t i, j; double col_sum, max_col = 0.0;
    if (!mat) return 0.0;
    for (j = 0; j < mat->cols; j++) {
        col_sum = 0.0;
        for (i = 0; i < mat->rows; i++)
            col_sum += fabs(mat->data[i * mat->stride + j]);
        if (col_sum > max_col) max_col = col_sum;
    }
    return max_col;
}

/* Infinity-norm: maximum absolute row sum.
 * Equals 1-norm of transpose by duality. */
double mpc_matrix_norm_inf(const mpc_matrix_t *mat)
{
    size_t i, j; double row_sum, max_row = 0.0;
    if (!mat) return 0.0;
    for (i = 0; i < mat->rows; i++) {
        row_sum = 0.0;
        for (j = 0; j < mat->cols; j++)
            row_sum += fabs(mat->data[i * mat->stride + j]);
        if (row_sum > max_row) max_row = row_sum;
    }
    return max_row;
}

/* GEMV: y = alpha*A*x + beta*y. BLAS Level 2 operation. */
void mpc_matrix_gemv(double alpha, const mpc_matrix_t *A, const double *x,
                     double beta, double *y)
{
    size_t i, j; double sum;
    if (!A || !x || !y) return;
    for (i = 0; i < A->rows; i++) {
        sum = 0.0;
        for (j = 0; j < A->cols; j++)
            sum += A->data[i * A->stride + j] * x[j];
        y[i] = alpha * sum + beta * y[i];
    }
}

/* GEMM: C = alpha*A*B + beta*C. BLAS Level 3 operation. */
void mpc_matrix_gemm(double alpha, const mpc_matrix_t *A, const mpc_matrix_t *B,
                     double beta, mpc_matrix_t *C)
{
    size_t i, j, k; double sum;
    if (!A || !B || !C) return;
    for (i = 0; i < C->rows; i++)
        for (j = 0; j < C->cols; j++) {
            sum = 0.0;
            for (k = 0; k < A->cols; k++)
                sum += A->data[i * A->stride + k] * B->data[k * B->stride + j];
            C->data[i * C->stride + j] = alpha * sum
                + beta * C->data[i * C->stride + j];
        }
}

/* Scaled addition: C = alpha*A + beta*B */
void mpc_matrix_add(double alpha, const mpc_matrix_t *A,
                    double beta, const mpc_matrix_t *B, mpc_matrix_t *C)
{
    size_t i, j;
    if (!A || !B || !C) return;
    for (i = 0; i < C->rows; i++)
        for (j = 0; j < C->cols; j++)
            C->data[i * C->stride + j] =
                alpha * A->data[i * A->stride + j]
                + beta * B->data[i * B->stride + j];
}

/* Transpose B = A^T */
int mpc_matrix_transpose(const mpc_matrix_t *A, mpc_matrix_t *B)
{
    size_t i, j;
    if (!A || !B) return -1;
    if (A->rows != B->cols || A->cols != B->rows) return -1;
    for (i = 0; i < A->rows; i++)
        for (j = 0; j < A->cols; j++)
            B->data[j * B->stride + i] = A->data[i * A->stride + j];
    return 0;
}
/* Triangular solves -- forward and backward substitution */

/* Forward substitution: solve L*x = b, L unit lower triangular.
 * Complexity: O(n^2). Reference: Golub & Van Loan (2013), Alg 3.1.1. */
void mpc_matrix_forward_sub(const mpc_matrix_t *L, const double *b, double *x)
{
    size_t i, j, n; double sum;
    if (!L || !b || !x) return;
    n = L->rows;
    for (i = 0; i < n; i++) {
        sum = 0.0;
        for (j = 0; j < i; j++)
            sum += L->data[i * L->stride + j] * x[j];
        x[i] = b[i] - sum;
    }
}

/* Backward substitution: solve U*x = b, U upper triangular.
 * Complexity: O(n^2). Reference: Golub & Van Loan (2013), Alg 3.1.2. */
void mpc_matrix_backward_sub(const mpc_matrix_t *U, const double *b, double *x)
{
    size_t i, j, n; double sum, diag;
    if (!U || !b || !x) return;
    n = U->rows;
    for (i = n; i > 0; ) {
        i--; sum = 0.0;
        for (j = i + 1; j < n; j++)
            sum += U->data[i * U->stride + j] * x[j];
        diag = U->data[i * U->stride + i];
        x[i] = fabs(diag) < 1.4901161193847656e-08 ? 0.0 : (b[i] - sum) / diag;
    }
}

/* Symmetry check: returns 1 if A == A^T within tolerance.
 * Complexity: O(n^2) checking upper triangle only. */
int mpc_matrix_is_symmetric(const mpc_matrix_t *mat, double tol)
{
    size_t i, j;
    if (!mat || mat->rows != mat->cols) return 0;
    for (i = 0; i < mat->rows; i++)
        for (j = i + 1; j < mat->cols; j++)
            if (fabs(mat->data[i * mat->stride + j]
                   - mat->data[j * mat->stride + i]) > tol)
                return 0;
    return 1;
}

/* SPD test via attempted Cholesky.
 * Theorem (Sylvester's criterion): A SPD iff all leading principal minors > 0.
 * Cholesky provides constructive O(n^3/3) verification.
 * Returns 1 if SPD, 0 otherwise. */
int mpc_matrix_is_spd(const mpc_matrix_t *mat, double tol)
{
    size_t i, j, k, n; double sum, diag_val;
    mpc_matrix_t *work;
    if (!mat || mat->rows != mat->cols) return 0;
    if (!mpc_matrix_is_symmetric(mat, tol)) return 0;
    n = mat->rows;
    work = mpc_matrix_copy(mat);
    if (!work) return 0;
    for (j = 0; j < n; j++) {
        sum = 0.0;
        for (k = 0; k < j; k++)
            sum += work->data[j * work->stride + k]
                 * work->data[j * work->stride + k];
        diag_val = work->data[j * work->stride + j] - sum;
        if (diag_val <= tol) { mpc_matrix_free(&work); return 0; }
        work->data[j * work->stride + j] = sqrt(diag_val);
        for (i = j + 1; i < n; i++) {
            sum = 0.0;
            for (k = 0; k < j; k++)
                sum += work->data[i * work->stride + k]
                     * work->data[j * work->stride + k];
            work->data[i * work->stride + j] =
                (work->data[i * work->stride + j] - sum)
                / work->data[j * work->stride + j];
        }
    }
    mpc_matrix_free(&work);
    return 1;
}

/* Trace: tr(A) = sum_i A_ii = sum_i lambda_i.
 * For SPD matrices: trace >= spectral radius.
 * Complexity: O(n). */
double mpc_matrix_trace(const mpc_matrix_t *mat)
{
    size_t i; double tr = 0.0;
    if (!mat || mat->rows != mat->cols) return 0.0;
    for (i = 0; i < mat->rows; i++)
        tr += mat->data[i * mat->stride + i];
    return tr;
}

/* Determinant via LU with partial pivoting.
 * det(A) = product(U_diag) * (-1)^(num_swaps).
 * Complexity: O(2n^3/3). Returns 0 for singular matrices. */
double mpc_matrix_det(const mpc_matrix_t *mat)
{
    size_t i, j, k, n; double det_val, pivot, factor;
    int sign = 1;
    double *work; size_t *piv;
    if (!mat || mat->rows != mat->cols) return 0.0;
    n = mat->rows;
    if (n == 0) return 1.0;
    work = (double*)malloc(n * n * sizeof(double));
    piv = (size_t*)malloc(n * sizeof(size_t));
    if (!work || !piv) { free(work); free(piv); return 0.0; }
    for (i = 0; i < n; i++)
        for (j = 0; j < n; j++)
            work[i * n + j] = mat->data[i * mat->stride + j];
    for (i = 0; i < n; i++) {
        pivot = fabs(work[i * n + i]); piv[i] = i;
        for (j = i + 1; j < n; j++) {
            if (fabs(work[j * n + i]) > pivot) {
                pivot = fabs(work[j * n + i]); piv[i] = j;
            }
        }
        if (piv[i] != i) {
            sign = -sign;
            for (k = 0; k < n; k++) {
                double tmp = work[i * n + k];
                work[i * n + k] = work[piv[i] * n + k];
                work[piv[i] * n + k] = tmp;
            }
        }
        if (fabs(work[i * n + i]) < 1.4901161193847656e-08) {
            free(work); free(piv); return 0.0;
        }
        for (j = i + 1; j < n; j++) {
            factor = work[j * n + i] / work[i * n + i];
            work[j * n + i] = factor;
            for (k = i + 1; k < n; k++)
                work[j * n + k] -= factor * work[i * n + k];
        }
    }
    det_val = (double)sign;
    for (i = 0; i < n; i++) det_val *= work[i * n + i];
    free(work); free(piv);
    return det_val;
}

/* Cholesky factorization A = L*L^T (in-place, lower triangle).
 * Algorithm (Golub & Van Loan 2013, Alg 4.2.2):
 *   for j = 0..n-1:
 *     L[j][j] = sqrt(A[j][j] - sum_{k<j} L[j][k]^2)
 *     for i = j+1..n-1:
 *       L[i][j] = (A[i][j] - sum_{k<j} L[i][k]*L[j][k]) / L[j][j]
 * Complexity: O(n^3/3). Returns -1 if not SPD. */
int mpc_matrix_cholesky(mpc_matrix_t *mat)
{
    size_t i, j, k, n; double sum;
    if (!mat || mat->rows != mat->cols) return -1;
    n = mat->rows;
    for (j = 0; j < n; j++) {
        sum = 0.0;
        for (k = 0; k < j; k++)
            sum += mat->data[j * mat->stride + k]
                 * mat->data[j * mat->stride + k];
        sum = mat->data[j * mat->stride + j] - sum;
        if (sum <= 1.4901161193847656e-08) return -1;
        mat->data[j * mat->stride + j] = sqrt(sum);
        for (i = j + 1; i < n; i++) {
            sum = 0.0;
            for (k = 0; k < j; k++)
                sum += mat->data[i * mat->stride + k]
                     * mat->data[j * mat->stride + k];
            mat->data[i * mat->stride + j] =
                (mat->data[i * mat->stride + j] - sum)
                / mat->data[j * mat->stride + j];
        }
    }
    return 0;
}

/* LU decomposition with partial pivoting.
 * Factorizes A = P*L*U. P stored implicitly in pivot[].
 * pivot[i] = row swapped with row i during elimination.
 * Algorithm (Golub & Van Loan 2013, Alg 3.4.1).
 * Complexity: O(2n^3/3). Returns -1 if singular. */
int mpc_matrix_lu(mpc_matrix_t *mat, size_t *pivot)
{
    size_t i, j, k, n; double max_val, tmp;
    if (!mat || mat->rows != mat->cols || !pivot) return -1;
    n = mat->rows;
    for (i = 0; i < n; i++) pivot[i] = i;
    for (i = 0; i < n; i++) {
        max_val = fabs(mat->data[i * mat->stride + i]);
        pivot[i] = i;
        for (j = i + 1; j < n; j++) {
            if (fabs(mat->data[j * mat->stride + i]) > max_val) {
                max_val = fabs(mat->data[j * mat->stride + i]);
                pivot[i] = j;
            }
        }
        if (pivot[i] != i) {
            for (k = 0; k < n; k++) {
                tmp = mat->data[i * mat->stride + k];
                mat->data[i * mat->stride + k]
                    = mat->data[pivot[i] * mat->stride + k];
                mat->data[pivot[i] * mat->stride + k] = tmp;
            }
        }
        if (fabs(mat->data[i * mat->stride + i]) < 1.4901161193847656e-08)
            return -1;
        for (j = i + 1; j < n; j++) {
            mat->data[j * mat->stride + i]
                /= mat->data[i * mat->stride + i];
            for (k = i + 1; k < n; k++)
                mat->data[j * mat->stride + k]
                    -= mat->data[j * mat->stride + i]
                     * mat->data[i * mat->stride + k];
        }
    }
    return 0;
}
