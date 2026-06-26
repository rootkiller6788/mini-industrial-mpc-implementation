/* ssid_hankel.c -- Block Hankel Matrix Construction and Operations
 *
 * Reference: Van Overschee & De Moor (1996), Chapter 1-2
 *            Ljung (1999), Section 10.6
 *
 * Implements the construction and manipulation of block Hankel matrices,
 * the fundamental data structure of subspace identification.
 *
 * The block Hankel matrix maps a flat time series into a structured matrix
 * whose row space encodes the system dynamics. This transformation is the
 * key insight that enables the linear algebra approach to system ID.
 */

#include "ssid_hankel.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

/* ========================================================================
 * L2: Block Hankel Matrix Construction
 *
 * Knowledge point: A block Hankel matrix transforms a temporal sequence
 * into a spatial (row-space) representation. Each block row corresponds
 * to a time-shifted copy of the data, creating a structure where linear
 * combinations of rows capture dynamic modes.
 *
 * For a single-channel signal {z_0, z_1, ..., z_{N-1}} with i block rows:
 *
 *     H_{0|i-1} = [z_0   z_1   ... z_{N-i} ]
 *                   [z_1   z_2   ... z_{N-i+1}]
 *                   [  ...                      ]
 *                   [z_{i-1} z_i ... z_{N-1}   ]
 *
 * This is a Hankel matrix (constant along anti-diagonals), which is the
 * natural representation for linear time-invariant systems.
 * ======================================================================== */

ssid_hankel_t ssid_hankel_build(const ssid_matrix_t *Z, size_t i)
{
    ssid_hankel_t H;
    memset(&H, 0, sizeof(H));

    if (!Z || !Z->data || Z->rows == 0 || Z->cols == 0) return H;
    if (i == 0 || i > Z->rows) return H;

    size_t m = Z->cols;  /* Number of variables (e.g., inputs or outputs) */
    size_t j = Z->rows - i + 1;  /* Number of columns in Hankel */
    if (j == 0) return H;

    /* Allocate Hankel matrix: (i*m) rows x j columns */
    H.i = i;
    H.j = j;
    H.m = m;
    H.past = ssid_matrix_alloc(i * m, j);

    if (!H.past.data) return H;

    /* Fill block Hankel: row block k corresponds to time shift k */
    for (size_t k = 0; k < i; k++) {
        for (size_t r = 0; r < m; r++) {
            size_t hankel_row = k * m + r;
            for (size_t col = 0; col < j; col++) {
                /* Z[k+col, r] = Z[(k+col)*Z_stride + r]
                 * Read column-major from Z */
                H.past.data[col * H.past.stride + hankel_row] =
                    Z->data[r * Z->stride + (k + col)];
            }
        }
    }

    return H;
}

/* ========================================================================
 * L2: Split Hankel into past/future halves
 *
 * Knowledge point: The past/future split separates information into:
 *   - Past (rows 0..i-1): contains information correlated with the
 *     initial state x_k
 *   - Future (rows i..2i-1): contains information about how the state
 *     evolves and how it influences future outputs
 *
 * The oblique projection Y_f /_{U_f} [U_p; Y_p] recovers the product
 * of the extended observability matrix and the Kalman state sequence.
 * ======================================================================== */

ssid_hankel_t ssid_hankel_split(const ssid_hankel_t *H, size_t i)
{
    ssid_hankel_t result;
    memset(&result, 0, sizeof(result));

    if (!H || H->i < 2*i) return result;

    /* Upper i*m rows = past */
    result.past = ssid_matrix_view(H->past.data, i * H->m, H->j, H->past.stride);
    result.past.owner = 0;
    /* Lower i*m rows = future */
    result.future = ssid_matrix_view(
        H->past.data + i * H->m, /* offset by i*m rows */
        i * H->m, H->j, H->past.stride);
    result.future.owner = 0;

    result.i = i;
    result.j = H->j;
    result.m = H->m;
    return result;
}

/* ========================================================================
 * L2: Past data matrix W_p = [U_p; Y_p]
 *
 * Knowledge point: The vertical concatenation of past inputs and outputs
 * forms the instrumental variable matrix. This matrix is uncorrelated
 * with future noise (e_f) but correlated with the state X_i, making it
 * a valid instrument for consistent estimation.
 *
 * Theorem (Van Overschee & De Moor, ?2.3):
 *   Under open-loop conditions with white noise, W_p is uncorrelated
 *   with future noise e_f. This makes subspace estimates consistent.
 * ======================================================================== */

ssid_matrix_t ssid_hankel_past_data(const ssid_hankel_t *Up,
                                    const ssid_hankel_t *Yp)
{
    if (!Up || !Yp) {
        ssid_matrix_t empty = {0};
        return empty;
    }

    size_t rows_up = Up->past.rows;
    size_t rows_yp = Yp->past.rows;
    size_t cols = Up->j;

    ssid_matrix_t Wp = ssid_matrix_alloc(rows_up + rows_yp, cols);
    if (!Wp.data) return Wp;

    /* Copy Up rows */
    for (size_t j = 0; j < cols; j++) {
        for (size_t i = 0; i < rows_up; i++) {
            Wp.data[j * Wp.stride + i] =
                Up->past.data[j * Up->past.stride + i];
        }
    }

    /* Copy Yp rows below Up */
    for (size_t j = 0; j < cols; j++) {
        for (size_t i = 0; i < rows_yp; i++) {
            Wp.data[j * Wp.stride + rows_up + i] =
                Yp->past.data[j * Yp->past.stride + i];
        }
    }

    return Wp;
}

/* ========================================================================
 * L3: Optimal block row count i
 *
 * Knowledge point: The number of block rows i determines the maximum
 * identifiable system order: n_x <= min(i * n_y, j).
 *
 * Too small i:
 *   - Limits identifiable order (risk of under-modeling)
 *   - Reduces computational cost
 * Too large i:
 *   - Increases variance (fewer columns j = N-2i+1)
 *   - Numerical issues (rank deficiency from too-few columns)
 *
 * Ljung's rule: i >= 2 * n_x / n_y ensures the observability matrix
 * has at least as many rows as the state dimension.
 * ======================================================================== */

size_t ssid_hankel_optimal_i(size_t N, size_t n_y, size_t n_x_guess)
{
    /* Minimum i to observe order n_x_guess: ceil(2*n_x_guess / n_y) */
    size_t i_min_order = (2 * n_x_guess + n_y - 1) / n_y;
    if (i_min_order < 2) i_min_order = 2;

    /* Maximum i limited by data: j = N - 2*i + 1 must be >= 1 */
    size_t i_max_data = (N > 2) ? (N - 1) / 2 : 1;

    /* Balance: enough rows for observability, enough columns for statistics */
    size_t i_balanced = (i_min_order + i_max_data) / 2;

    /* Clamp to valid range */
    if (i_balanced > i_max_data) i_balanced = i_max_data;
    if (i_balanced < 2) i_balanced = 2;

    return i_balanced;
}

/* ========================================================================
 * L3: Persistent excitation rank
 *
 * Knowledge point: Persistent excitation (PE) is a necessary condition
 * for consistent identification. An input sequence is PE of order n if
 * the covariance matrix (or equivalently, the Hankel matrix) has full
 * row rank n.
 *
 * PE condition in subspace ID (Van Overschee & De Moor, ?3.4):
 *   rank(U_p) >= n_u * i + n_x   (for deterministic system)
 *   rank([U_p; Y_p]) = n_u*i + n_x (for stochastic system, w.p.1)
 *
 * We estimate the numerical rank using the ratio of consecutive
 * singular values (or equivalently, QR pivoting). This function
 * uses a simplified singular value threshold approach on the
 * Gram matrix U_p * U_p^T for efficiency.
 * ======================================================================== */

size_t ssid_hankel_persist_excitation_rank(const ssid_hankel_t *Up, double tol)
{
    if (!Up || Up->past.rows == 0) return 0;
    if (tol <= 0.0) tol = 1e-10;

    /* Compute Gram matrix G = U_p * U_p^T (rows x rows) */
    /* For large Hankel matrices, this is more efficient than full SVD */
    size_t r = Up->past.rows;
    size_t j = Up->j;
    double *G = (double *)calloc(r * r, sizeof(double));
    if (!G) return 0;

    /* G = U_p * U_p^T (row-major storage for G) */
    for (size_t c = 0; c < j; c++) {
        for (size_t a = 0; a < r; a++) {
            double va = Up->past.data[c * Up->past.stride + a];
            for (size_t b = 0; b < r; b++) {
                G[a * r + b] += va * Up->past.data[c * Up->past.stride + b];
            }
        }
    }

    /* Estimate trace for normalization */
    double trace = 0.0;
    for (size_t i = 0; i < r; i++) {
        trace += G[i * r + i];
    }

    if (trace < tol) { free(G); return 0; }

    /* Simple rank estimation: count diagonal elements above threshold
     * relative to the trace. This is a fast approximation. */
    double threshold = trace * tol;
    size_t rank = 0;
    for (size_t i = 0; i < r; i++) {
        /* Compute row i norm as approximate eigenvalue magnitude */
        double row_norm = 0.0;
        for (size_t jj = 0; jj < r; jj++) {
            row_norm += fabs(G[i * r + jj]);
        }
        if (row_norm > threshold / (double)r) {
            rank++;
        }
    }

    free(G);
    return rank;
}

/* ========================================================================
 * L3: Data equation matrix
 *
 * Knowledge point: The data equation is the fundamental algebraic
 * relationship in subspace identification:
 *
 *   Y_f = Gamma_i * X_i + H_i^d * U_f + H_i^s * E_f
 *
 * where:
 *   Gamma_i = [C; C*A; ...; C*A^{i-1}]           (observability)
 *   H_i^d   = Toeplitz matrix of Markov params     (deterministic)
 *   H_i^s   = Toeplitz matrix of stochastic Markov params
 *   X_i     = [x_i, x_{i+1}, ..., x_{i+j-1}]     (state sequence)
 *
 * By projecting Y_f along U_f onto W_p, we eliminate H_i^d*U_f
 * and H_i^s*E_f (orthogonal to W_p), leaving:
 *   O_i = Gamma_i * X_i_hat
 *
 * This function performs that elimination via oblique projection.
 * ======================================================================== */

int ssid_hankel_data_equation(const ssid_hankel_t *Yf,
                              const ssid_hankel_t *Uf,
                              const ssid_hankel_t *Yp,
                              const ssid_hankel_t *Up,
                              ssid_matrix_t *Gamma_X)
{
    if (!Yf || !Uf || !Yp || !Up || !Gamma_X) return -1;

    /* Form W_p = [U_p; Y_p] */
    ssid_matrix_t Wp = ssid_hankel_past_data(Up, Yp);
    if (!Wp.data) return -2;

    /* Build the combined matrix [W_p; U_f; Y_f] for LQ decomposition */
    size_t rows_wp = Wp.rows;
    size_t rows_uf = Uf->future.rows;
    size_t rows_yf = Yf->future.rows;
    size_t cols    = Wp.cols;

    /* Simplified approach: compute oblique projection directly.
     * O_i = Y_f /_{U_f} W_p
     *     = Y_f * [W_p; U_f]^{\dagger} * [I; 0] * W_p
     *
     * First, compute the joint matrix J = [W_p; U_f] */
    ssid_matrix_t J = ssid_matrix_alloc(rows_wp + rows_uf, cols);
    if (!J.data) { ssid_matrix_free(&Wp); return -3; }

    /* Fill J with W_p on top */
    for (size_t j = 0; j < cols; j++) {
        for (size_t i = 0; i < rows_wp; i++) {
            J.data[j * J.stride + i] = Wp.data[j * Wp.stride + i];
        }
    }
    /* U_f below W_p */
    for (size_t j = 0; j < cols; j++) {
        for (size_t i = 0; i < rows_uf; i++) {
            J.data[j * J.stride + rows_wp + i] =
                Uf->future.data[j * Uf->future.stride + i];
        }
    }

    /* Compute J^T * J for pseudoinverse (Gram matrix approach) */
    size_t k = J.rows;
    ssid_matrix_t JTJ = ssid_matrix_alloc(k, k);
    if (!JTJ.data) { ssid_matrix_free(&Wp); ssid_matrix_free(&J); return -4; }

    for (size_t a = 0; a < k; a++) {
        for (size_t b = 0; b < k; b++) {
            double sum = 0.0;
            for (size_t c = 0; c < cols; c++) {
                double ja = J.data[c * J.stride + a];
                double jb = J.data[c * J.stride + b];
                /* Guard against NaN */
                if (ja == ja && jb == jb) sum += ja * jb;
            }
            JTJ.data[b * JTJ.stride + a] = sum;
        }
    }

    /* Compute RHS = J^T * Y_f */
    ssid_matrix_t RHS = ssid_matrix_alloc(k, rows_yf);
    if (!RHS.data) {
        ssid_matrix_free(&Wp); ssid_matrix_free(&J);
        ssid_matrix_free(&JTJ); return -5;
    }

    for (size_t a = 0; a < k; a++) {
        for (size_t y = 0; y < rows_yf; y++) {
            double sum = 0.0;
            for (size_t c = 0; c < cols; c++) {
                double ja = J.data[c * J.stride + a];
                double yf_val = Yf->future.data[c * Yf->future.stride + y];
                if (ja == ja && yf_val == yf_val) sum += ja * yf_val;
            }
            RHS.data[y * RHS.stride + a] = sum;
        }
    }

    /* Solve JTJ * Theta = RHS using Gaussian elimination */
    int solve_ok = 0;
    /* Simple regularization: add small diagonal */
    double reg = 1e-10;
    for (size_t i = 0; i < k; i++) {
        JTJ.data[i * JTJ.stride + i] += reg;
    }

    /* In-place solve: RHS becomes Theta = pinv(J) * Y_f */
    if (ssid_matrix_solve(&JTJ, &RHS) == 0) {
        /* O_i = W_p * Theta_{1:rows_wp, :} */
        for (size_t y = 0; y < rows_yf; y++) {
            for (size_t c = 0; c < cols; c++) {
                double sum = 0.0;
                for (size_t a = 0; a < rows_wp; a++) {
                    double wp_val = Wp.data[c * Wp.stride + a];
                    double theta_val = RHS.data[y * RHS.stride + a];
                    if (wp_val == wp_val && theta_val == theta_val) {
                        sum += wp_val * theta_val;
                    }
                }
                if (c < Gamma_X->rows && y < Gamma_X->cols) {
                    Gamma_X->data[y * Gamma_X->stride + c] = sum;
                }
            }
        }
        solve_ok = 1;
    }

    ssid_matrix_free(&Wp);
    ssid_matrix_free(&J);
    ssid_matrix_free(&JTJ);
    ssid_matrix_free(&RHS);

    return solve_ok ? 0 : -6;
}

/* ========================================================================
 * L5: SVC-based i estimation
 *
 * Knowledge point: The number of block rows i can be selected
 * data-adaptively by monitoring the singular value spectrum of
 * Hankel matrices for increasing i. When adding more block rows
 * stops revealing new singular values (i.e., the rank stabilizes),
 * the optimal i has been reached.
 * ======================================================================== */

size_t ssid_hankel_estimate_i_svc(const ssid_matrix_t *Z,
                                  size_t n_y, size_t N,
                                  size_t i_min, size_t i_max,
                                  double tol)
{
    (void)n_y; /* reserved for future multi-output weighting */
    if (!Z || !Z->data || i_min < 2) return 2;
    if (i_max > (N-1)/2) i_max = (N-1)/2;

    size_t best_i = i_min;
    size_t prev_rank = 0;
    size_t stable_count = 0;

    for (size_t i = i_min; i <= i_max && i <= i_min + 20; i++) {
        ssid_hankel_t H = ssid_hankel_build(Z, i);
        if (!H.past.data) break;

        size_t rank = ssid_hankel_persist_excitation_rank(&H, tol);
        ssid_hankel_free(&H);

        if (rank == prev_rank) {
            stable_count++;
            if (stable_count >= 2) { best_i = i - 2; break; }
        } else {
            stable_count = 0;
        }
        prev_rank = rank;
        best_i = i;
    }

    return best_i;
}

/* ========================================================================
 * L4: Markov Parameters
 *
 * Knowledge point: The Markov parameters (impulse response coefficients)
 * h_k = C*A^{k-1}*B contain the complete I/O information of an LTI
 * system. They appear naturally as block entries of H_i^d (the
 * deterministic Toeplitz matrix in the data equation).
 *
 * Recovery from subspace estimates:
 *   From O_i = Gamma_i * X_i, we extract Gamma_i and X_i via SVD.
 *   Then B and D are obtained from the Markov parameters by
 *   regression, connecting subspace ID to classical impulse response
 *   modeling (the basis of DMC).
 * ======================================================================== */

int ssid_hankel_markov_parameters(const ssid_matrix_t *Gamma_X,
                                  const ssid_hankel_t *Uf,
                                  const ssid_hankel_t *Yf,
                                  ssid_matrix_t *H)
{
    if (!Gamma_X || !Uf || !Yf || !H) return -1;

    /* H = (Y_f - Gamma_X) * pinv(U_f)
     * This recovers the deterministic Toeplitz matrix
     * whose first block column gives the Markov parameters. */
    size_t cols = Uf->j;
    size_t rows_uf = Uf->future.rows;
    size_t rows_yf = Yf->future.rows;

    if (cols == 0 || rows_uf == 0) return -2;

    /* Compute U_f^T * U_f for pseudoinverse */
    ssid_matrix_t UTU = ssid_matrix_alloc(rows_uf, rows_uf);
    if (!UTU.data) return -3;

    for (size_t a = 0; a < rows_uf; a++) {
        for (size_t b = 0; b < rows_uf; b++) {
            double sum = 0.0;
            for (size_t c = 0; c < cols; c++) {
                sum += Uf->future.data[c * Uf->future.stride + a] *
                       Uf->future.data[c * Uf->future.stride + b];
            }
            UTU.data[b * UTU.stride + a] = sum;
        }
    }

    /* Regularize */
    for (size_t i = 0; i < rows_uf; i++) {
        UTU.data[i * UTU.stride + i] += 1e-8;
    }

    /* Compute residual Y_f - Gamma_X */
    ssid_matrix_t R = ssid_matrix_alloc(rows_yf, cols);
    if (!R.data) { ssid_matrix_free(&UTU); return -4; }

    for (size_t j = 0; j < cols; j++) {
        for (size_t i = 0; i < rows_yf; i++) {
            double y_val = Yf->future.data[j * Yf->future.stride + i];
            double gx_val = Gamma_X->data[j * Gamma_X->stride + i];
            R.data[j * R.stride + i] = y_val - gx_val;
        }
    }

    /* Compute R * U_f^T as RHS for LS */
    ssid_matrix_t RHS = ssid_matrix_alloc(rows_yf, rows_uf);
    if (!RHS.data) {
        ssid_matrix_free(&UTU); ssid_matrix_free(&R); return -5;
    }

    for (size_t a = 0; a < rows_yf; a++) {
        for (size_t b = 0; b < rows_uf; b++) {
            double sum = 0.0;
            for (size_t c = 0; c < cols; c++) {
                sum += R.data[c * R.stride + a] *
                       Uf->future.data[c * Uf->future.stride + b];
            }
            RHS.data[b * RHS.stride + a] = sum;
        }
    }

    /* Solve UTU * H^T = RHS^T */
    ssid_matrix_t Htemp = RHS; /* reuse RHS storage */
    int status = ssid_matrix_solve(&UTU, &Htemp);
    if (status == 0) {
        /* Htemp now contains H^T; transpose into H */
        for (size_t i = 0; i < rows_yf; i++) {
            for (size_t j = 0; j < rows_uf; j++) {
                H->data[i * H->stride + j] =
                    Htemp.data[j * Htemp.stride + i];
            }
        }
    }

    ssid_matrix_free(&UTU);
    ssid_matrix_free(&R);
    return status;
}

/* Free all resources in a Hankel structure */
void ssid_hankel_free(ssid_hankel_t *H)
{
    if (!H) return;
    ssid_matrix_free(&H->past);
    ssid_matrix_free(&H->future);
    H->i = 0;
    H->j = 0;
    H->m = 0;
}
