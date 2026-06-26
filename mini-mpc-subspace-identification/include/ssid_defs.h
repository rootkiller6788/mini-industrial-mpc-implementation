/* ssid_defs.h �� Core Definitions for Subspace Identification (4SID)
 *
 * Reference: Van Overschee & De Moor (1996), "Subspace Identification for Linear Systems"
 *            Ljung (1999), "System Identification: Theory for the User"
 *
 * Defines all fundamental data types, structures and enumerations for subspace-based
 * state-space system identification. Each struct definition maps to a distinct
 * knowledge point in L1-L3 of the SKILL.md nine-layer system.
 *
 * Theorem Source: Van Overschee & De Moor, Theorem 2 (Deterministic Identification)
 *   The state sequence X_i can be recovered (up to similarity transform) from
 *   the row space of the oblique projection of future outputs onto past I/O data.
 */

#ifndef SSID_DEFS_H
#define SSID_DEFS_H

#include <stddef.h>
#include <stdint.h>

/* ---------------------------------------------------------------------------
 * L1: Core Type Definitions
 * ---------------------------------------------------------------------------
 */

/* Subspace identification algorithm family (L2: Core Concepts)
 *
 * N4SID    �� Numerical algorithms for Subspace State Space ID
 * MOESP    �� Multivariable Output-Error State sPace
 * CVA      �� Canonical Variate Analysis
 */
typedef enum {
    SSID_ALG_N4SID      = 0,
    SSID_ALG_MOESP      = 1,
    SSID_ALG_CVA        = 2,
    SSID_ALG_PO_MOESP   = 3,
    SSID_ALG_PI_MOESP   = 4,
    SSID_ALG_ROBUST_CVA = 5
} ssid_algorithm_t;

/* Weighting schemes for subspace methods (L5: Algorithms)
 *
 * N4SID:     W1 = I, W2 = I
 * CVA:       W1 = (E[Y_f Y_f^T])^{-1/2}, W2 = I
 * MOESP:     W1 = I, W2 = Pi_{U_f^\perp}
 */
typedef enum {
    SSID_WGT_N4SID      = 0,
    SSID_WGT_CVA        = 1,
    SSID_WGT_MOESP      = 2,
    SSID_WGT_ROBUST_CVA = 3,
    SSID_WGT_IVM        = 4
} ssid_weighting_t;

/* Model form for identification (L1: Definitions)
 *
 * INNOVATION �� x_{k+1} = A x_k + B u_k + K e_k; y_k = C x_k + D u_k + e_k
 * OUTPUT     �� Standard deterministic form without noise model
 * ERR_IN_VAR �� Errors-in-variables: both input and output have measurement noise
 */
typedef enum {
    SSID_FORM_INNOVATION    = 0,
    SSID_FORM_OUTPUT_ERROR  = 1,
    SSID_FORM_ERR_IN_VAR    = 2
} ssid_model_form_t;

/* Order selection criterion (L5: Algorithms)
 *
 * AIC  �� Akaike Information Criterion: -2*log(L) + 2*k
 * BIC  �� Bayesian Information Criterion: -2*log(L) + k*log(N)
 * SVD_GAP �� Largest gap in singular value spectrum
 * MDL  �� Minimum Description Length (Rissanen)
 * CROSSVAL �� Hold-out cross-validation MSE
 */
typedef enum {
    SSID_ORDER_AIC         = 0,
    SSID_ORDER_BIC         = 1,
    SSID_ORDER_SVD_GAP     = 2,
    SSID_ORDER_MDL         = 3,
    SSID_ORDER_CROSSVAL    = 4
} ssid_order_criterion_t;

/* ---------------------------------------------------------------------------
 * L1: Core Data Structures
 * ---------------------------------------------------------------------------
 */

/* System dimensions descriptor (L1: Definitions)
 *
 * Captures the fundamental dimensions of an LTI system.
 * n_y = outputs, n_u = inputs, n_x = state order.
 * i = number of block rows in Hankel matrix.
 */
typedef struct {
    size_t n_y;
    size_t n_u;
    size_t n_x;
    size_t N;
    size_t i;
    double Ts;
} ssid_dim_t;

/* Identification configuration (L3: Engineering Structures)
 *
 * Bundles all user-specified parameters for a subspace identification run.
 * Reflects industrial practice from AspenTech DMC3, Honeywell Profit
 * Controller, and MathWorks System Identification Toolbox.
 */
typedef struct {
    ssid_algorithm_t algorithm;
    ssid_weighting_t weighting;
    ssid_model_form_t model_form;
    ssid_order_criterion_t order_crit;
    size_t i_min;
    size_t i_max;
    size_t n_x_min;
    size_t n_x_max;
    double svd_tol;
    int    use_ku;
    int    closed_loop;
    int    remove_trend;
    int    normalize_data;
    double cond_max;
    int    verbose;
} ssid_config_t;

/* ---------------------------------------------------------------------------
 * L1: Fundamental Matrix Types
 * ---------------------------------------------------------------------------
 */

/* Double-precision dense matrix (L1: Definitions, L3: Math Structures)
 *
 * Column-major storage (Fortran convention) for LAPACK/BLAS.
 * Fundamental data type for all subspace computations.
 */
typedef struct {
    double *data;
    size_t  rows;
    size_t  cols;
    size_t  stride;
    int     owner;
} ssid_matrix_t;

/* State-space model representation (L1: Definitions)
 *
 * Discrete-time: x_{k+1} = A x_k + B u_k + K e_k
 *                y_k     = C x_k + D u_k + e_k
 *
 * Continuous-time: dx/dt = A x + B u + K e,  y = C x + D u + e
 */
typedef struct {
    ssid_matrix_t A;
    ssid_matrix_t B;
    ssid_matrix_t C;
    ssid_matrix_t D;
    ssid_matrix_t K;
    double       *cov_e;
    size_t        n_x;
    size_t        n_u;
    size_t        n_y;
    double        Ts;
    int           is_ct;
} ssid_model_t;

/* ---------------------------------------------------------------------------
 * L2: Hankel Matrix Structures
 * ---------------------------------------------------------------------------
 */

/* Block Hankel matrix (L2: Core Concepts)
 *
 * The cornerstone of subspace identification. For sequence {z_k}:
 *
 *         | z_0    z_1    ...  z_{j-1}   |
 * Z_{0|i-1} = | z_1    z_2    ...  z_j       |
 *         |  ...                         |
 *         | z_{i-1} z_i   ...  z_{N-2}  |
 *
 * Key insight (Van Overschee & De Moor): the row space captures
 * the dynamic information needed to recover the state sequence.
 *
 * Block Hankel structure:
 *   Z_p = Z_{0|i-1}      (past, rows 0..i-1)
 *   Z_f = Z_{i|2i-1}     (future, rows i..2i-1)
 */
typedef struct {
    ssid_matrix_t past;
    ssid_matrix_t future;
    size_t        i;
    size_t        j;
    size_t        m;
} ssid_hankel_t;

/* ---------------------------------------------------------------------------
 * L3: SVD and LQ Decomposition Results
 * ---------------------------------------------------------------------------
 */

/* SVD result: A = U * S * V^T (economy-size) (L3: Math Structures)
 *
 * s[0] >= s[1] >= ... >= s[k-1] >= 0 (descending order).
 * The singular value spectrum reveals the system order.
 */
typedef struct {
    ssid_matrix_t U;
    ssid_matrix_t V;
    double       *s;
    size_t        m;
    size_t        n;
    size_t        k;
} ssid_svd_t;

/* LQ decomposition: A = L * Q, L lower triangular, Q orthogonal.
 * Stored compactly for square-root algorithms (L3: Math Structures) */
typedef struct {
    ssid_matrix_t A_mat;
    double       *tau;
    size_t        m;
    size_t        n;
} ssid_lq_t;

/* ---------------------------------------------------------------------------
 * L2: Data and Result packets
 * ---------------------------------------------------------------------------
 */

/* MIMO I/O data record (L2: Core Concepts)
 *
 * Theorem: If input is persistently exciting of order 2*i + n_x,
 * then state-space matrices are consistently identifiable up to
 * a similarity transform (Van Overschee & De Moor, Sec 3.4).
 */
typedef struct {
    ssid_matrix_t U;
    ssid_matrix_t Y;
    size_t        N;
    size_t        n_u;
    size_t        n_y;
    double        Ts;
    char          source[128];
} ssid_data_t;

/* Identification result bundle (L2: Core Concepts) */
typedef struct {
    ssid_model_t  model;
    ssid_svd_t    svd_info;
    size_t        n_x_selected;
    double        fit_metric;
    double        aic_score;
    double        bic_score;
    double        autocorr_max;
    double        crosscorr_max;
    double        stability_margin;
    int           is_stable;
    int           is_observable;
    int           is_controllable;
    size_t        n_iter;
    double        elapsed_ms;
    char          status_msg[256];
} ssid_result_t;

/* ---------------------------------------------------------------------------
 * L1: Construction / Destruction API
 * --------------------------------------------------------------------------- */

/* Matrix construction from raw sizes. O(rows*cols). */
ssid_matrix_t ssid_matrix_alloc(size_t rows, size_t cols);

/* Matrix construction from row-major C array (deep copy). O(rows*cols). */
ssid_matrix_t ssid_matrix_from_array(size_t rows, size_t cols, const double *array_rm);

/* Matrix view (no allocation) into existing data. O(1). */
ssid_matrix_t ssid_matrix_view(double *data, size_t rows, size_t cols, size_t stride);

/* Free owned matrix memory. O(1). */
void ssid_matrix_free(ssid_matrix_t *mat);

/* Zero all entries. O(rows*cols). */
void ssid_matrix_zero(ssid_matrix_t *mat);

/* Set to identity matrix (must be square). O(rows*cols). */
int ssid_matrix_eye(ssid_matrix_t *mat);

/* Fill with uniform random values in [a, b]. O(rows*cols). */
void ssid_matrix_rand(ssid_matrix_t *mat, double a, double b);

/* Set all entries to value v. O(rows*cols). */
void ssid_matrix_fill(ssid_matrix_t *mat, double v);

/* Get matrix element A(row, col). O(1). */
double ssid_matrix_get(const ssid_matrix_t *mat, size_t row, size_t col);

/* Set matrix element A(row, col) = val. O(1). */
void ssid_matrix_set(ssid_matrix_t *mat, size_t row, size_t col, double val);

/* Print matrix to stdout (for debugging). */
void ssid_matrix_print(const char *name, const ssid_matrix_t *mat, int max_rows);

/* --- Extended matrix operations (used internally by projection/SVD/N4SID) --- */

/* Gaussian elimination with partial pivoting: solve Ax = b in-place.
 * A becomes LU factors, b becomes solution x. O(n^3). */
int ssid_matrix_solve(ssid_matrix_t *A, ssid_matrix_t *b);

/* Matrix addition: C = A + B. O(rows*cols). */
int ssid_matrix_add(const ssid_matrix_t *A, const ssid_matrix_t *B, ssid_matrix_t *C);

/* Matrix subtraction: C = A - B. O(rows*cols). */
int ssid_matrix_sub(const ssid_matrix_t *A, const ssid_matrix_t *B, ssid_matrix_t *C);

/* Matrix multiplication: C = A * B. O(m*n*k). */
int ssid_matrix_multiply(const ssid_matrix_t *A, const ssid_matrix_t *B, ssid_matrix_t *C);

/* In-place transpose (square only). O(n^2). */
void ssid_matrix_transpose_square(ssid_matrix_t *mat);

/* Extract column as new vector. O(rows). */
ssid_matrix_t ssid_matrix_get_column(const ssid_matrix_t *mat, size_t col);

/* Set column from vector. O(rows). */
void ssid_matrix_set_column(ssid_matrix_t *mat, size_t col, const ssid_matrix_t *vec);

/* Extract row as 1 x cols vector. O(cols). */
ssid_matrix_t ssid_matrix_get_row(const ssid_matrix_t *mat, size_t row);

/* Scale: A = alpha * A. O(rows*cols). */
void ssid_matrix_scale(ssid_matrix_t *mat, double alpha);

/* Euclidean norm of a column vector. Uses Kahan summation. O(n). */
double ssid_matrix_norm(const ssid_matrix_t *mat);

/* Frobenius norm. O(rows*cols). */
double ssid_matrix_frobenius_norm(const ssid_matrix_t *mat);

/* Deep copy: dst = src. O(rows*cols). */
int ssid_matrix_copy(const ssid_matrix_t *src, ssid_matrix_t *dst);

/* ---------------------------------------------------------------------------
 * L1: Configuration + Model lifecycle
 * --------------------------------------------------------------------------- */

/* Default configuration: N4SID + CVA weighting, open-loop, innovation form */
ssid_config_t ssid_config_default(void);

/* Free all memory within an ssid_model_t. */
void ssid_model_free(ssid_model_t *model);

/* Free all memory within an ssid_data_t. */
void ssid_data_free(ssid_data_t *data);

/* Free an ssid_result_t (the model portion). */
void ssid_result_free(ssid_result_t *result);

/* Set dimension from data shape */
ssid_dim_t ssid_dim_from_data(size_t n_y, size_t n_u, size_t N, double Ts, size_t i);

/* Convert algorithm enum to human-readable name */
const char *ssid_algorithm_name(ssid_algorithm_t algo);

/* Convert weighting enum to human-readable name */
const char *ssid_weighting_name(ssid_weighting_t wgt);

#endif /* SSID_DEFS_H */
