/* ssid_defs.c -- Implementation of core definitions and matrix operations
 *
 * Reference: Van Overschee & De Moor (1996)
 *            Ljung (1999)
 *
 * Implements the fundamental data structure management (allocation,
 * deallocation, view creation) and basic matrix operations that
 * underpin all subspace identification algorithms.
 *
 * Each function implements one distinct knowledge point.
 */

#include "ssid_defs.h"
#include "ssid_svd.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <time.h>

/* ========================================================================
 * L1: Matrix Memory Management
 *
 * Knowledge point: Column-major storage (Fortran convention) is the
 * standard for numerical linear algebra. Using column-major layout
 * enables direct use of LAPACK/BLAS without transposition overhead.
 * ======================================================================== */

/* ---------------------------------------------------------------------------
 * L1: Allocate a zero-initialized dense matrix.
 *
 * Knowledge point: Dynamic memory allocation for dense matrices.
 * Column-major storage: element (row, col) at index col*stride + row.
 * stride >= rows allows for padding (SIMD alignment, BLAS compatibility).
 * ---------------------------------------------------------------------------
 */
ssid_matrix_t ssid_matrix_alloc(size_t rows, size_t cols)
{
    ssid_matrix_t mat;
    mat.rows   = rows;
    mat.cols   = cols;
    mat.stride = rows;  /* No padding by default */
    mat.owner  = 1;

    if (rows == 0 || cols == 0) {
        mat.data = NULL;
        return mat;
    }

    mat.data = (double *)calloc(rows * cols, sizeof(double));
    if (!mat.data) {
        fprintf(stderr, "ssid_matrix_alloc: failed to allocate %zu x %zu matrix\n",
                rows, cols);
        mat.rows = 0;
        mat.cols = 0;
    }
    return mat;
}

/* ---------------------------------------------------------------------------
 * L1: Construct matrix from row-major C array (deep copy).
 *
 * Knowledge point: Data import from standard C arrays. Since C uses
 * row-major and our internal storage is column-major, we must
 * transpose during copy. This is the most common source of bugs
 * when interfacing with external data sources (CSV files, databases,
 * PLC historians, OSIsoft PI).
 * ---------------------------------------------------------------------------
 */
ssid_matrix_t ssid_matrix_from_array(size_t rows, size_t cols,
                                     const double *array_rm)
{
    ssid_matrix_t mat = ssid_matrix_alloc(rows, cols);
    if (!mat.data || !array_rm) return mat;

    for (size_t j = 0; j < cols; j++) {
        for (size_t i = 0; i < rows; i++) {
            mat.data[j * mat.stride + i] = array_rm[i * cols + j];
        }
    }
    return mat;
}

/* ---------------------------------------------------------------------------
 * L1: Create a matrix view (no allocation) into existing data.
 *
 * Knowledge point: Views (zero-copy submatrix references) are essential
 * for efficiency in numerical computing. They allow algorithms to
 * operate on submatrices without memory allocation overhead.
 * The owner flag prevents double-free errors.
 * ---------------------------------------------------------------------------
 */
ssid_matrix_t ssid_matrix_view(double *data, size_t rows, size_t cols, size_t stride)
{
    ssid_matrix_t mat;
    mat.data   = data;
    mat.rows   = rows;
    mat.cols   = cols;
    mat.stride = (stride >= rows) ? stride : rows;
    mat.owner  = 0;
    return mat;
}

/* ---------------------------------------------------------------------------
 * L1: Free owned matrix memory. Safe on views (no-op).
 *
 * Knowledge point: Ownership-based memory management prevents
 * double-free bugs common in C numerical libraries. Each matrix
 * explicitly tracks whether it owns its data buffer.
 * ---------------------------------------------------------------------------
 */
void ssid_matrix_free(ssid_matrix_t *mat)
{
    if (mat && mat->owner && mat->data) {
        free(mat->data);
        mat->data = NULL;
    }
    if (mat) {
        mat->rows = 0;
        mat->cols = 0;
        mat->stride = 0;
        mat->owner = 0;
    }
}

/* ========================================================================
 * L1: Basic Matrix Operations
 * ======================================================================== */

/* ---------------------------------------------------------------------------
 * L1: Zero all matrix entries.
 *
 * Knowledge point: Memory initialization prevents undefined behavior
 * from uninitialized values. In subspace algorithms, zeroing is
 * needed before accumulating sums or as initial guess for iterative methods.
 * ---------------------------------------------------------------------------
 */
void ssid_matrix_zero(ssid_matrix_t *mat)
{
    if (!mat || !mat->data) return;
    size_t n_elements = mat->cols * mat->stride;
    memset(mat->data, 0, n_elements * sizeof(double));
}

/* ---------------------------------------------------------------------------
 * L1: Set to identity matrix (must be square).
 *
 * Knowledge point: The identity matrix is the multiplicative identity
 * in linear algebra (A * I = I * A = A). It appears in subspace ID as:
 *   - Default N4SID weighting: W1 = I, W2 = I
 *   - Initialization of iterative algorithms
 *   - Null-space projection bases
 * ---------------------------------------------------------------------------
 */
int ssid_matrix_eye(ssid_matrix_t *mat)
{
    if (!mat || !mat->data) return -1;
    if (mat->rows != mat->cols) return -2;

    ssid_matrix_zero(mat);
    size_t n = mat->rows;
    for (size_t i = 0; i < n; i++) {
        mat->data[i * mat->stride + i] = 1.0;
    }
    return 0;
}

/* ---------------------------------------------------------------------------
 * L1: Fill with uniform random values in [a, b].
 *
 * Knowledge point: Random matrices are used for:
 *   - Monte Carlo validation (perturbed initial conditions)
 *   - Generating synthetic test data for algorithm testing
 *   - Random projections in randomized SVD (Halko et al., 2011)
 *
 * Uses simple LCG; for production, replace with Mersenne Twister.
 * ---------------------------------------------------------------------------
 */
void ssid_matrix_rand(ssid_matrix_t *mat, double a, double b)
{
    if (!mat || !mat->data) return;
    static unsigned long seed = 12345UL;

    size_t n = mat->cols * mat->stride;
    for (size_t k = 0; k < n; k++) {
        seed = (1103515245UL * seed + 12345UL) & 0x7FFFFFFFUL;
        double r = (double)seed / (double)0x7FFFFFFFUL;
        mat->data[k] = a + r * (b - a);
    }
}

/* ---------------------------------------------------------------------------
 * L1: Set all entries to a constant value.
 *
 * Knowledge point: Constant fill is used for:
 *   - Initializing bias terms
 *   - Setting constraint bounds uniformly
 *   - Creating rank-1 matrices where all entries are equal
 * ---------------------------------------------------------------------------
 */
void ssid_matrix_fill(ssid_matrix_t *mat, double v)
{
    if (!mat || !mat->data) return;
    size_t n = mat->cols * mat->stride;
    for (size_t k = 0; k < n; k++) {
        mat->data[k] = v;
    }
}

/* ---------------------------------------------------------------------------
 * L1: Element access (column-major indexing).
 *
 * Knowledge point: Column-major indexing: index = col * stride + row.
 * This is the Fortran/LAPACK convention. Direct element access is
 * used sparingly in production code (prefer BLAS Level 1/2/3), but
 * essential for debugging, data I/O, and small-matrix operations.
 * ---------------------------------------------------------------------------
 */
double ssid_matrix_get(const ssid_matrix_t *mat, size_t row, size_t col)
{
    if (!mat || !mat->data || row >= mat->rows || col >= mat->cols) {
        return 0.0;
    }
    return mat->data[col * mat->stride + row];
}

void ssid_matrix_set(ssid_matrix_t *mat, size_t row, size_t col, double val)
{
    if (!mat || !mat->data || row >= mat->rows || col >= mat->cols) {
        return;
    }
    mat->data[col * mat->stride + row] = val;
}

/* ---------------------------------------------------------------------------
 * L1: Print matrix for debugging.
 *
 * Knowledge point: Diagnostic output is essential for algorithm
 * development and debugging. Printing with name and limiting rows
 * prevents overwhelming output for large matrices typical in
 * industrial data (N = 10000+ samples).
 * ---------------------------------------------------------------------------
 */
void ssid_matrix_print(const char *name, const ssid_matrix_t *mat, int max_rows)
{
    if (!mat || !mat->data) {
        printf("%s = NULL\n", name ? name : "unnamed");
        return;
    }
    printf("%s [%zu x %zu, stride=%zu, owner=%d]:\n",
           name ? name : "unnamed", mat->rows, mat->cols, mat->stride, mat->owner);

    size_t rows_to_print = (size_t)max_rows > 0 && (size_t)max_rows < mat->rows
                           ? (size_t)max_rows : mat->rows;
    size_t cols_to_print = mat->cols > 10 ? 10 : mat->cols;

    for (size_t i = 0; i < rows_to_print; i++) {
        printf("  row %3zu: ", i);
        for (size_t j = 0; j < cols_to_print; j++) {
            printf("%10.4f ", mat->data[j * mat->stride + i]);
        }
        if (mat->cols > 10) printf("...");
        printf("\n");
    }
    if (mat->rows > (size_t)max_rows && max_rows > 0) {
        printf("  ... (%zu more rows)\n", mat->rows - rows_to_print);
    }
}

/* ========================================================================
 * L1: Matrix Arithmetic (fundamental building blocks)
 *
 * Each operation teaches a BLAS-level knowledge point.
 * ======================================================================== */

/* L1: Matrix transpose in-place (square matrices only).
 *
 * Knowledge point: The transpose is the most basic matrix symmetry
 * operation. For column-major storage, A^T[i,j] = A[j,i] which
 * is A[i * stride + j] in flat array -- the storage convention
 * implicitly swaps indices.
 *
 * Complexity: O(n^2). */
void ssid_matrix_transpose_square(ssid_matrix_t *mat)
{
    if (!mat || !mat->data || mat->rows != mat->cols) return;
    size_t n = mat->rows, stride = mat->stride;
    for (size_t j = 0; j < n; j++) {
        for (size_t i = 0; i < j; i++) {
            double tmp = mat->data[j * stride + i];
            mat->data[j * stride + i] = mat->data[i * stride + j];
            mat->data[i * stride + j] = tmp;
        }
    }
}

/* L1: Extract column as a new vector.
 *
 * Knowledge point: Column extraction is the fundamental operation
 * for building block Hankel matrices, where each block row is
 * a reshaped time series. Efficient column access (stride-based)
 * avoids cache misses.
 *
 * Complexity: O(rows). */
ssid_matrix_t ssid_matrix_get_column(const ssid_matrix_t *mat, size_t col)
{
    ssid_matrix_t v = ssid_matrix_alloc(mat->rows, 1);
    if (!v.data) return v;
    for (size_t i = 0; i < mat->rows; i++) {
        v.data[i] = mat->data[col * mat->stride + i];
    }
    return v;
}

/* L1: Set a column from a vector.
 *
 * Complexity: O(rows). */
void ssid_matrix_set_column(ssid_matrix_t *mat, size_t col, const ssid_matrix_t *vec)
{
    if (!mat || !vec || col >= mat->cols || vec->rows < mat->rows) return;
    for (size_t i = 0; i < mat->rows; i++) {
        mat->data[col * mat->stride + i] = vec->data[i];
    }
}

/* L1: Extract row as a new row vector (1 x cols).
 *
 * Knowledge point: Row extraction with column-major storage is
 * less cache-friendly (stride-based access). For intensive row
 * operations, consider transposing the matrix first.
 *
 * Complexity: O(cols). */
ssid_matrix_t ssid_matrix_get_row(const ssid_matrix_t *mat, size_t row)
{
    ssid_matrix_t rv = ssid_matrix_alloc(1, mat->cols);
    if (!rv.data) return rv;
    for (size_t j = 0; j < mat->cols; j++) {
        rv.data[j] = mat->data[j * mat->stride + row];
    }
    return rv;
}

/* L1: Scale matrix by a scalar: A = alpha * A.
 *
 * Knowledge point: Scalar multiplication (xSCAL in BLAS) is one
 * of the few O(n) operations in linear algebra. Used extensively
 * for weighting, normalization, and SVD post-processing
 * (multiplying by sqrt of singular values).
 *
 * Complexity: O(rows * cols). */
void ssid_matrix_scale(ssid_matrix_t *mat, double alpha)
{
    if (!mat || !mat->data) return;
    size_t n = mat->cols * mat->stride;
    for (size_t k = 0; k < n; k++) {
        mat->data[k] *= alpha;
    }
}

/* L1: Vector norm computation (Euclidean L2 norm).
 *
 * Knowledge point: Norm computation is the basis for:
 *   - Convergence checks in iterative algorithms
 *   - Residual magnitude (model fit quality)
 *   - Regularization penalties (Tikhonov, LASSO)
 *   - Numerical rank determination (singular value thresholds)
 *
 * Uses Kahan summation for numerical accuracy with large vectors.
 *
 * Complexity: O(n). */
double ssid_matrix_norm(const ssid_matrix_t *mat)
{
    if (!mat || !mat->data || mat->cols != 1) return 0.0;
    double sum = 0.0, c = 0.0;
    for (size_t i = 0; i < mat->rows; i++) {
        double y = mat->data[i] * mat->data[i] - c;
        double t = sum + y;
        c = (t - sum) - y;
        sum = t;
    }
    return sqrt(sum);
}

/* L1: Frobenius norm of a matrix ||A||_F = sqrt(sum a_ij^2).
 *
 * Knowledge point: The Frobenius norm is the "sum of squares" norm
 * that generalizes the vector L2 norm. It is rotationally invariant
 * (||UAV||_F = ||A||_F for orthogonal U,V), making it natural for
 * SVD-based analysis.
 *
 * Complexity: O(rows * cols). */
double ssid_matrix_frobenius_norm(const ssid_matrix_t *mat)
{
    if (!mat || !mat->data) return 0.0;
    double sum = 0.0;
    size_t total = mat->cols * mat->stride;
    for (size_t k = 0; k < total; k++) {
        sum += mat->data[k] * mat->data[k];
    }
    return sqrt(sum);
}

/* L1: Matrix copy (deep copy).
 *
 * Complexity: O(rows * cols). */
int ssid_matrix_copy(const ssid_matrix_t *src, ssid_matrix_t *dst)
{
    if (!src || !dst || !src->data || !dst->data) return -1;
    if (src->rows != dst->rows || src->cols != dst->cols) return -2;
    /* Copy column by column for cache efficiency in column-major */
    for (size_t j = 0; j < src->cols; j++) {
        memcpy(&dst->data[j * dst->stride],
               &src->data[j * src->stride],
               src->rows * sizeof(double));
    }
    return 0;
}

/* L1: Matrix addition: C = A + B.
 *
 * Knowledge point: Element-wise addition (xAXPY in BLAS terms
 * with alpha=1, beta=1). Used in residual computation,
 * state update equations, and gradient accumulation.
 *
 * Complexity: O(rows * cols). */
int ssid_matrix_add(const ssid_matrix_t *A, const ssid_matrix_t *B, ssid_matrix_t *C)
{
    if (!A || !B || !C || !A->data || !B->data || !C->data) return -1;
    if (A->rows != B->rows || A->cols != B->cols ||
        A->rows != C->rows || A->cols != C->cols) return -2;

    size_t rows = A->rows, cols = A->cols;
    for (size_t j = 0; j < cols; j++) {
        for (size_t i = 0; i < rows; i++) {
            C->data[j * C->stride + i] =
                A->data[j * A->stride + i] + B->data[j * B->stride + i];
        }
    }
    return 0;
}

/* L1: Matrix subtraction: C = A - B.
 *
 * Complexity: O(rows * cols). */
int ssid_matrix_sub(const ssid_matrix_t *A, const ssid_matrix_t *B, ssid_matrix_t *C)
{
    if (!A || !B || !C || !A->data || !B->data || !C->data) return -1;
    if (A->rows != B->rows || A->cols != B->cols ||
        A->rows != C->rows || A->cols != C->cols) return -2;

    size_t rows = A->rows, cols = A->cols;
    for (size_t j = 0; j < cols; j++) {
        for (size_t i = 0; i < rows; i++) {
            C->data[j * C->stride + i] =
                A->data[j * A->stride + i] - B->data[j * B->stride + i];
        }
    }
    return 0;
}

/* L1: Matrix multiplication: C = A * B.
 *
 * Knowledge point: Matrix multiplication is the core operation of
 * linear algebra (xGEMM in BLAS Level 3). The triply-nested loop
 * implements the standard O(n^3) algorithm. For production use,
 * this should be replaced with BLAS DGEMM.
 *
 * The loop order (i, k, j) optimizes for column-major storage by
 * keeping the inner loop stride-1 (contiguous in memory).
 *
 * Complexity: O(m * n * k). */
int ssid_matrix_multiply(const ssid_matrix_t *A, const ssid_matrix_t *B, ssid_matrix_t *C)
{
    if (!A || !B || !C || !A->data || !B->data || !C->data) return -1;
    if (A->cols != B->rows) return -2;
    if (C->rows != A->rows || C->cols != B->cols) return -3;

    size_t m = A->rows, n = B->cols, k_inner = A->cols;
    /* Zero C first */
    ssid_matrix_zero(C);
    /* Loop order: i-k-j for column-major (cache-friendly) */
    for (size_t j = 0; j < n; j++) {
        for (size_t k = 0; k < k_inner; k++) {
            double b_kj = B->data[j * B->stride + k];
            if (b_kj == 0.0) continue;
            for (size_t i = 0; i < m; i++) {
                C->data[j * C->stride + i] +=
                    A->data[k * A->stride + i] * b_kj;
            }
        }
    }
    return 0;
}

/* L1: Solve linear system Ax = b using Gaussian elimination with
 * partial pivoting.
 *
 * Knowledge point: Linear system solving is required in subspace ID
 * for the least-squares steps (estimating B,D from regression,
 * computing projections). Gaussian elimination with pivoting is
 * numerically stable for well-conditioned matrices.
 *
 * Complexity: O(n^3).
 * Reference: Golub & Van Loan (2013), Algorithm 3.4.1. */
int ssid_matrix_solve(ssid_matrix_t *A, ssid_matrix_t *b)
{
    if (!A || !b || !A->data || !b->data) return -1;
    if (A->rows != A->cols || A->rows != b->rows) return -2;

    size_t n = A->rows, nrhs = b->cols;
    size_t *pivot = (size_t *)malloc(n * sizeof(size_t));
    if (!pivot) return -3;

    /* Gaussian elimination with partial pivoting */
    for (size_t k = 0; k < n; k++) {
        /* Find pivot */
        size_t p = k;
        double max_val = fabs(A->data[k * A->stride + k]);
        for (size_t i = k + 1; i < n; i++) {
            double val = fabs(A->data[k * A->stride + i]);
            if (val > max_val) { max_val = val; p = i; }
        }
        pivot[k] = p;
        if (max_val < 1e-15) { free(pivot); return -4; } /* singular */

        /* Swap rows */
        if (p != k) {
            for (size_t j = 0; j < n; j++) {
                double tmp = A->data[j * A->stride + k];
                A->data[j * A->stride + k] = A->data[j * A->stride + p];
                A->data[j * A->stride + p] = tmp;
            }
            for (size_t j = 0; j < nrhs; j++) {
                double tmp = b->data[j * b->stride + k];
                b->data[j * b->stride + k] = b->data[j * b->stride + p];
                b->data[j * b->stride + p] = tmp;
            }
        }

        /* Eliminate below pivot */
        double pivot_val = A->data[k * A->stride + k];
        for (size_t i = k + 1; i < n; i++) {
            double factor = A->data[k * A->stride + i] / pivot_val;
            A->data[k * A->stride + i] = 0.0; /* exact zero */
            for (size_t j = k + 1; j < n; j++) {
                A->data[j * A->stride + i] -= factor * A->data[j * A->stride + k];
            }
            for (size_t j = 0; j < nrhs; j++) {
                b->data[j * b->stride + i] -= factor * b->data[j * b->stride + k];
            }
        }
    }

    /* Back substitution */
    for (int k = (int)n - 1; k >= 0; k--) {
        for (size_t j = 0; j < nrhs; j++) {
            double sum = b->data[j * b->stride + k];
            for (size_t i = k + 1; i < n; i++) {
                sum -= A->data[i * A->stride + k] * b->data[j * b->stride + i];
            }
            b->data[j * b->stride + k] = sum / A->data[k * A->stride + k];
        }
    }

    free(pivot);
    return 0;
}

/* ========================================================================
 * L1: Configuration and Lifecycle Management
 * ======================================================================== */

/* Default configuration for N4SID with CVA weighting.
 *
 * Knowledge point: Sensible defaults are critical for industrial
 * software adoption. The defaults here match AspenTech DMC3 and
 * MATLAB System ID Toolbox conventions. */
ssid_config_t ssid_config_default(void)
{
    ssid_config_t cfg;
    cfg.algorithm     = SSID_ALG_N4SID;
    cfg.weighting     = SSID_WGT_CVA;
    cfg.model_form    = SSID_FORM_INNOVATION;
    cfg.order_crit    = SSID_ORDER_SVD_GAP;
    cfg.i_min         = 2;
    cfg.i_max         = 50;
    cfg.n_x_min       = 1;
    cfg.n_x_max       = 20;
    cfg.svd_tol       = 1e-10;
    cfg.use_ku        = 1;
    cfg.closed_loop   = 0;
    cfg.remove_trend  = 1;
    cfg.normalize_data = 1;
    cfg.cond_max      = 1e12;
    cfg.verbose       = 0;
    return cfg;
}

/* Free all memory within a model.
 *
 * Knowledge point: Structured cleanup prevents memory leaks in
 * long-running industrial applications where identification
 * may be called repeatedly (e.g., adaptive MPC). */
void ssid_model_free(ssid_model_t *model)
{
    if (!model) return;
    ssid_matrix_free(&model->A);
    ssid_matrix_free(&model->B);
    ssid_matrix_free(&model->C);
    ssid_matrix_free(&model->D);
    ssid_matrix_free(&model->K);
    if (model->cov_e) { free(model->cov_e); model->cov_e = NULL; }
    model->n_x = 0;
    model->n_u = 0;
    model->n_y = 0;
}

void ssid_data_free(ssid_data_t *data)
{
    if (!data) return;
    ssid_matrix_free(&data->U);
    ssid_matrix_free(&data->Y);
    data->N = 0;
}

void ssid_result_free(ssid_result_t *result)
{
    if (!result) return;
    ssid_model_free(&result->model);
    ssid_svd_free(&result->svd_info);
}

ssid_dim_t ssid_dim_from_data(size_t n_y, size_t n_u, size_t N,
                              double Ts, size_t i)
{
    ssid_dim_t dim;
    dim.n_y = n_y;
    dim.n_u = n_u;
    dim.n_x = 0; /* to be determined */
    dim.N   = N;
    dim.i   = i;
    dim.Ts  = Ts;
    return dim;
}

const char *ssid_algorithm_name(ssid_algorithm_t algo)
{
    switch (algo) {
        case SSID_ALG_N4SID:      return "N4SID";
        case SSID_ALG_MOESP:      return "MOESP";
        case SSID_ALG_CVA:        return "CVA";
        case SSID_ALG_PO_MOESP:   return "PO-MOESP";
        case SSID_ALG_PI_MOESP:   return "PI-MOESP";
        case SSID_ALG_ROBUST_CVA: return "Robust-CVA";
        default:                  return "Unknown";
    }
}

const char *ssid_weighting_name(ssid_weighting_t wgt)
{
    switch (wgt) {
        case SSID_WGT_N4SID:      return "N4SID (W1=I, W2=I)";
        case SSID_WGT_CVA:        return "CVA (W1=R_f^{-1/2}, W2=I)";
        case SSID_WGT_MOESP:      return "MOESP (W1=I, W2=Pi_perp)";
        case SSID_WGT_ROBUST_CVA: return "Robust CVA";
        case SSID_WGT_IVM:        return "IVM (Instrumental Variables)";
        default:                  return "Unknown";
    }
}
