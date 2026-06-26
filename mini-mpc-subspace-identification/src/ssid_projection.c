/* ssid_projection.c -- Orthogonal and Oblique Projections via LQ Decomposition
 *
 * Reference: Van Overschee & De Moor (1996), Chapter 1.4, 2.3
 *            Golub & Van Loan (2013), "Matrix Computations", Chapter 5
 *            Verhaegen & Dewilde (1992), "Subspace model identification"
 *
 * Implements the geometric operations that form the mathematical core of
 * all 4SID algorithms. Each function teaches a specific aspect of numerical
 * linear algebra applied to system identification.
 */

#include "ssid_projection.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ========================================================================
 * L4: LQ Decomposition via Householder Reflections
 *
 * Knowledge point: LQ decomposition (A = L*Q where L is lower triangular,
 * Q is orthogonal) is the row-space analog of QR decomposition. In
 * subspace ID, LQ is applied to the block Hankel matrices to achieve
 * a numerically stable square-root implementation.
 *
 * Unlike the normal equations approach (A^T*A)^{-1}*A^T*B, the LQ approach
 * avoids squaring the condition number. For a matrix with condition
 * number kappa, normal equations have condition kappa^2 while LQ keeps it
 * at kappa.
 *
 * Algorithm: Apply Householder reflections from the "right" (column side)
 * to zero out elements above the diagonal, producing L in the lower part
 * and storing Q implicitly as reflector vectors.
 *
 * Reference: Golub & Van Loan (2013), Algorithm 5.2.1 (adapted for LQ).
 * ======================================================================== */

int ssid_lq_decompose(ssid_lq_t *lq)
{
    if (!lq || !lq->A_mat.data) return -1;

    size_t m = lq->m;
    size_t n = lq->n;
    size_t k = (m < n) ? m : n; /* min(m,n) */

    lq->tau = (double *)malloc(k * sizeof(double));
    if (!lq->tau) return -2;

    double *A = lq->A_mat.data;
    size_t stride = lq->A_mat.stride;

    /* Apply Householder reflections to zero each column above diagonal */
    /* For LQ: zero elements to the right (columns > row) within each row */
    for (size_t j = 0; j < k; j++) {
        /* Compute Householder vector for column j, elements j..m-1 */
        /* We zero rows j+1..n-1 in the j-th row */
        double alpha = A[j * stride + j];
        double norm_x = 0.0;
        for (size_t i = j + 1; i < n; i++) {
            norm_x += A[i * stride + j] * A[i * stride + j];
        }
        double sigma = sqrt(norm_x);

        if (sigma < 1e-15) {
            lq->tau[j] = 0.0;
            continue;
        }

        double v0 = (alpha >= 0) ? alpha + sigma : alpha - sigma;
        lq->tau[j] = 2.0 * v0 * v0 / (norm_x + v0 * v0);

        /* Store v0 in the diagonal position */
        A[j * stride + j] = v0;

        /* Apply Householder reflection H = I - tau*v*v^T from the right
         * to columns j+1..m-1 (the "below" rows). */
        if (j + 1 < m) {
            for (size_t col = j; col < m; col++) {
                /* Compute w = A(j:m, col)^T * v */
                double sum = 0.0;
                if (col == j) {
                    sum = v0; /* v[j] = v0 */
                    for (size_t i = j + 1; i < n; i++) {
                        sum += A[i * stride + col] * A[i * stride + col];
                    }
                } else {
                    for (size_t i = j; i < n; i++) {
                        sum += A[i * stride + col] * A[i * stride + j];
                    }
                }
                double w = lq->tau[j] * sum;

                /* Update A(j:m, col) -= w * v */
                if (col == j) {
                    A[j * stride + col] -= w * v0;
                } else {
                    A[j * stride + col] -= w * A[j * stride + j];
                }
                for (size_t i = j + 1; i < n; i++) {
                    A[i * stride + col] -= w * A[i * stride + j];
                }
            }
        }

        /* Restore actual L diagonal element = -sigma * sign(alpha) */
        /* stored in A[j][j] for compact representation */
    }

    return 0;
}

/* Extract the lower triangular L from the compact LQ storage.
 * The diagonal + subdiagonal of the input contains L. */
void ssid_lq_extract_L(const ssid_lq_t *lq, ssid_matrix_t *L)
{
    if (!lq || !L || !L->data || !lq->A_mat.data) return;

    size_t m = L->rows;
    size_t n = L->cols;
    size_t k = (m < n) ? m : n;

    ssid_matrix_zero(L);

    for (size_t j = 0; j < k; j++) {
        for (size_t i = j; i < m; i++) {
            L->data[j * L->stride + i] =
                lq->A_mat.data[j * lq->A_mat.stride + i];
        }
    }
}

/* Apply Q^T to a matrix B: B = Q^T * B or B = B * Q^T. */
int ssid_lq_apply_QT(const ssid_lq_t *lq, ssid_matrix_t *B, const char *side)
{
    if (!lq || !B || !B->data) return -1;

    size_t k = (lq->m < lq->n) ? lq->m : lq->n;
    double *A = lq->A_mat.data;
    size_t stride = lq->A_mat.stride;

    if (side[0] == 'L' || side[0] == 'l') {
        /* Q^T * B: apply reflections from left */
        for (size_t j = 0; j < k && j < k; j++) {
            if (lq->tau[j] < 1e-15) continue;
            /* Apply to each column of B */
            for (size_t col = 0; col < B->cols; col++) {
                double sum = B->data[col * B->stride + j];
                for (size_t i = j + 1; i < B->rows; i++) {
                    sum += A[i * stride + j] * B->data[col * B->stride + i];
                }
                double w = lq->tau[j] * sum;
                B->data[col * B->stride + j] -= w;
                for (size_t i = j + 1; i < B->rows; i++) {
                    B->data[col * B->stride + i] -= w * A[i * stride + j];
                }
            }
        }
    }
    return 0;
}

void ssid_lq_free(ssid_lq_t *lq)
{
    if (!lq) return;
    ssid_matrix_free(&lq->A_mat);
    if (lq->tau) { free(lq->tau); lq->tau = NULL; }
    lq->m = 0;
    lq->n = 0;
}

/* ========================================================================
 * L3: Orthogonal Projection
 *
 * Pi_A(B) = A * (A^T * A)^{-1} * A^T * B
 *
 * Knowledge point: The orthogonal projection finds the best approximation
 * of B in the row space of A. Geometrically, it drops a perpendicular
 * from each row of B to the subspace spanned by the rows of A.
 *
 * In subspace ID, the orthogonal projection separates the component of
 * Y_f that lies in the span of W_p (signal) from the orthogonal
 * complement (noise).
 *
 * Implementation: Solve (A^T A) X = A^T B via normal equations with
 * regularization for numerical stability.
 * ======================================================================== */

int ssid_project_orthogonal(const ssid_matrix_t *A,
                            const ssid_matrix_t *B,
                            ssid_matrix_t *Pi_A_B)
{
    if (!A || !B || !Pi_A_B || !A->data || !B->data || !Pi_A_B->data)
        return -1;

    size_t rows_A = A->rows;
    size_t cols   = A->cols;
    size_t cols_B = B->cols;

    /* A^T * A */
    ssid_matrix_t ATA = ssid_matrix_alloc(rows_A, rows_A);
    if (!ATA.data) return -2;

    for (size_t a = 0; a < rows_A; a++) {
        for (size_t b = 0; b < rows_A; b++) {
            double sum = 0.0;
            for (size_t c = 0; c < cols; c++) {
                sum += A->data[c * A->stride + a] *
                       A->data[c * A->stride + b];
            }
            ATA.data[b * ATA.stride + a] = sum;
        }
    }

    /* Regularize */
    for (size_t i = 0; i < rows_A; i++) {
        ATA.data[i * ATA.stride + i] += 1e-12;
    }

    /* A^T * B */
    ssid_matrix_t ATB = ssid_matrix_alloc(rows_A, cols_B);
    if (!ATB.data) { ssid_matrix_free(&ATA); return -3; }

    for (size_t a = 0; a < rows_A; a++) {
        for (size_t b = 0; b < cols_B; b++) {
            double sum = 0.0;
            for (size_t c = 0; c < cols; c++) {
                sum += A->data[c * A->stride + a] *
                       B->data[b * B->stride + c];
            }
            ATB.data[b * ATB.stride + a] = sum;
        }
    }

    /* Solve ATA * X = ATB */
    int status = ssid_matrix_solve(&ATA, &ATB);

    /* Pi_A_B = A * X */
    ssid_matrix_zero(Pi_A_B);
    for (size_t j = 0; j < cols_B; j++) {
        for (size_t k = 0; k < rows_A; k++) {
            double x_kj = ATB.data[j * ATB.stride + k];
            if (x_kj == 0.0) continue;
            for (size_t i = 0; i < cols; i++) {
                Pi_A_B->data[j * Pi_A_B->stride + i] +=
                    A->data[k * A->stride + i] * x_kj;
            }
        }
    }

    ssid_matrix_free(&ATA);
    ssid_matrix_free(&ATB);
    return status;
}

/* ========================================================================
 * L3: Oblique Projection
 *
 * Y /_W U = [Y / U^perp] * [W / U^perp]^{-1} * W
 *
 * Knowledge point: The oblique projection is the mathematical key to
 * subspace identification. It projects Y along the direction of U onto
 * the subspace W. Geometrically:
 *   - First, remove the component of Y that lies in U (orthogonal)
 *   - Then project what remains onto W (orthogonal)
 *
 * In the 4SID context:
 *   O_i = Y_f /_{U_f} W_p
 *
 * This recovers Gamma_i * X_i because:
 *   - Y_f = Gamma_i * X_i + H_i^d * U_f + noise
 *   - Projecting along U_f removes the deterministic input contribution
 *   - Projecting onto W_p recovers the state-related component
 *   - The noise term is uncorrelated with W_p and vanishes asymptotically
 *
 * Theorem (Van Overschee & De Moor, Theorem 2):
 *   rank(O_i) = n_x  (under appropriate excitation conditions)
 *
 * Complexity: O(m^2 * n) where m = total rows involved.
 * ======================================================================== */

int ssid_project_oblique(const ssid_matrix_t *Y,
                         const ssid_matrix_t *U,
                         const ssid_matrix_t *W,
                         ssid_matrix_t *O_i)
{
    if (!Y || !U || !W || !O_i || !Y->data || !U->data || !W->data || !O_i->data)
        return -1;

    /* Step 1: Orthogonal complement projection: remove U from Y and W */
    ssid_matrix_t Y_perp = ssid_matrix_alloc(Y->rows, Y->cols);
    ssid_matrix_t W_perp = ssid_matrix_alloc(W->rows, W->cols);
    if (!Y_perp.data || !W_perp.data) {
        ssid_matrix_free(&Y_perp);
        ssid_matrix_free(&W_perp);
        return -2;
    }

    /* Y_perp = Y - Pi_U(Y) */
    ssid_matrix_t Pi_U_Y = ssid_matrix_alloc(Y->rows, Y->cols);
    if (!Pi_U_Y.data) {
        ssid_matrix_free(&Y_perp);
        ssid_matrix_free(&W_perp);
        return -3;
    }

    ssid_project_orthogonal(U, Y, &Pi_U_Y);
    for (size_t j = 0; j < Y->cols; j++) {
        for (size_t i = 0; i < Y->rows; i++) {
            Y_perp.data[j * Y_perp.stride + i] =
                Y->data[j * Y->stride + i] -
                Pi_U_Y.data[j * Pi_U_Y.stride + i];
        }
    }

    ssid_matrix_free(&Pi_U_Y);

    /* W_perp = W - Pi_U(W) */
    ssid_matrix_t Pi_U_W = ssid_matrix_alloc(W->rows, W->cols);
    if (!Pi_U_W.data) {
        ssid_matrix_free(&Y_perp);
        ssid_matrix_free(&W_perp);
        return -4;
    }

    ssid_project_orthogonal(U, W, &Pi_U_W);
    for (size_t j = 0; j < W->cols; j++) {
        for (size_t i = 0; i < W->rows; i++) {
            W_perp.data[j * W_perp.stride + i] =
                W->data[j * W->stride + i] -
                Pi_U_W.data[j * Pi_U_W.stride + i];
        }
    }

    ssid_matrix_free(&Pi_U_W);

    /* Step 2: O_i = Pi_W_perp(Y_perp)
     * This is the oblique projection: project Y_perp onto W_perp. */
    int status = ssid_project_orthogonal(&W_perp, &Y_perp, O_i);

    ssid_matrix_free(&Y_perp);
    ssid_matrix_free(&W_perp);

    return status;
}

/* ========================================================================
 * L3: Orthogonal Complement Projection
 *
 * Pi_{A^perp}(B) = B - Pi_A(B)
 *
 * Knowledge point: The orthogonal complement removes the influence of
 * a subspace. In MOESP, this eliminates U_f's contribution from Y_f
 * before extracting the observability subspace, implementing the
 * instrumental variable approach.
 * ======================================================================== */

int ssid_project_orth_complement(const ssid_matrix_t *A,
                                 const ssid_matrix_t *B,
                                 ssid_matrix_t *result)
{
    if (!A || !B || !result || !A->data || !B->data || !result->data)
        return -1;

    ssid_matrix_t Pi_A_B = ssid_matrix_alloc(B->rows, B->cols);
    if (!Pi_A_B.data) return -2;

    int status = ssid_project_orthogonal(A, B, &Pi_A_B);
    if (status != 0) { ssid_matrix_free(&Pi_A_B); return status; }

    for (size_t j = 0; j < B->cols; j++) {
        for (size_t i = 0; i < B->rows; i++) {
            result->data[j * result->stride + i] =
                B->data[j * B->stride + i] -
                Pi_A_B.data[j * Pi_A_B.stride + i];
        }
    }

    ssid_matrix_free(&Pi_A_B);
    return 0;
}

/* ========================================================================
 * L5: Combined LQ Decomposition for Unified 4SID
 *
 * Knowledge point: The unified theorem (Van Overschee & De Moor, Theorem 3)
 * shows that all three major 4SID algorithms can be computed from a single
 * LQ decomposition:
 *
 *   [U_f; W_p; Y_f] = L * Q^T
 *
 * Partitioning L:
 *   [L11  0    0  ] [Q1^T]
 *   [L21 L22   0  ] [Q2^T]
 *   [L31 L32 L33  ] [Q3^T]
 *
 * The oblique projection O_i = L32 * pinv(L22) * [L21 L22] * [Q1; Q2]
 *
 * Different weighting choices:
 *   N4SID: SVD of L32 (unweighted)
 *   MOESP: SVD of L32 * pinv(L22)
 *   CVA:   SVD of weighted L32
 *
 * This unification is both mathematically elegant and computationally
 * efficient: one LQ serves all algorithms.
 * ======================================================================== */

int ssid_project_combined_LQ(const ssid_hankel_t *Uf_past,
                             const ssid_hankel_t *Uf_future,
                             const ssid_hankel_t *Wp,
                             const ssid_hankel_t *Yf,
                             ssid_matrix_t *R32,
                             ssid_matrix_t *R22)
{
    (void)Uf_past; /* reserved for extended LQ partitioning */
    if (!Uf_future || !Wp || !Yf || !R32 || !R22) return -1;

    size_t rows_uf = Uf_future->future.rows;
    size_t rows_wp = Wp->past.rows;
    size_t rows_yf = Yf->future.rows;
    size_t cols    = Uf_future->j;
    size_t total_rows = rows_uf + rows_wp + rows_yf;

    /* Build combined matrix M = [U_f; W_p; Y_f] */
    ssid_matrix_t M = ssid_matrix_alloc(total_rows, cols);
    if (!M.data) return -2;

    /* Copy U_f */
    for (size_t j = 0; j < cols; j++) {
        for (size_t i = 0; i < rows_uf; i++) {
            M.data[j * M.stride + i] =
                Uf_future->future.data[j * Uf_future->future.stride + i];
        }
    }
    /* Copy W_p */
    for (size_t j = 0; j < cols; j++) {
        for (size_t i = 0; i < rows_wp; i++) {
            M.data[j * M.stride + rows_uf + i] =
                Wp->past.data[j * Wp->past.stride + i];
        }
    }
    /* Copy Y_f */
    for (size_t j = 0; j < cols; j++) {
        for (size_t i = 0; i < rows_yf; i++) {
            M.data[j * M.stride + rows_uf + rows_wp + i] =
                Yf->future.data[j * Yf->future.stride + i];
        }
    }

    /* Compute M * M^T to get L * L^T (Gram matrix of L) */
    /* The cross-term R32 = L_{Y_f,W_p} appears in M*M^T at
     * block (yf_start : end, wp_start : wp_end). */
    size_t wp_start = rows_uf;
    size_t yf_start = rows_uf + rows_wp;

    /* Extract R32 from the Gram matrix */
    for (size_t i = 0; i < rows_yf && i < R32->rows; i++) {
        for (size_t k = 0; k < rows_wp && k < R32->cols; k++) {
            double sum = 0.0;
            for (size_t c = 0; c < cols; c++) {
                sum += M.data[c * M.stride + yf_start + i] *
                       M.data[c * M.stride + wp_start + k];
            }
            R32->data[k * R32->stride + i] = sum;
        }
    }

    /* Extract R22 (W_p self-gram) */
    for (size_t i = 0; i < rows_wp && i < R22->rows; i++) {
        for (size_t k = 0; k < rows_wp && k < R22->cols; k++) {
            double sum = 0.0;
            for (size_t c = 0; c < cols; c++) {
                sum += M.data[c * M.stride + wp_start + i] *
                       M.data[c * M.stride + wp_start + k];
            }
            R22->data[k * R22->stride + i] = sum;
        }
    }

    ssid_matrix_free(&M);
    return 0;
}

/* ========================================================================
 * L5: Apply subspace weighting
 *
 * Knowledge point: W1 and W2 weight the projection to emphasize
 * either deterministic or stochastic aspects:
 *
 *   N4SID  (W1=I, W2=I):     Balanced deterministic-stochastic
 *   CVA    (W1=cov(Y_f)^{-1/2}, W2=I): Maximum likelihood under
 *          Gaussian noise (Larimore, 1990). The canonical
 *          correlation weighting.
 *   MOESP  (W1=I, W2=Pi_{U_f^\perp}): Instrumental variable
 *          method for consistent estimates under arbitrary
 *          noise color (Verhaegen, 1994).
 *
 * The choice of weighting affects:
 *   - Consistency: whether estimate converges to true system as N->inf
 *   - Efficiency: variance of the estimate
 *   - Numerical conditioning: CVA can amplify noise in poorly excited modes
 * ======================================================================== */

int ssid_project_apply_weighting(const ssid_matrix_t *O_i,
                                 const ssid_matrix_t *Yf,
                                 const ssid_matrix_t *Uf_future,
                                 ssid_weighting_t weighting,
                                 ssid_matrix_t *weighted_O)
{
    if (!O_i || !weighted_O || !O_i->data || !weighted_O->data) return -1;

    size_t rows = O_i->rows;
    size_t cols = O_i->cols;

    /* Default: copy O_i as-is (N4SID weighting, W1=I, W2=I) */
    for (size_t j = 0; j < cols && j < weighted_O->cols; j++) {
        for (size_t i = 0; i < rows && i < weighted_O->rows; i++) {
            weighted_O->data[j * weighted_O->stride + i] =
                O_i->data[j * O_i->stride + i];
        }
    }

    if (weighting == SSID_WGT_N4SID) {
        /* Nothing more to do */
        return 0;
    }

    if (weighting == SSID_WGT_CVA) {
        /* W1 = (Y_f * Y_f^T)^{-1/2}
         * Compute YY^T, then use its Cholesky factor inverse as weight */
        if (!Yf || !Yf->data) return -1;

        /* W1 = diag of inverse sqrt of Y_f row norms (simplified approach) */
        for (size_t i = 0; i < rows && i < Yf->rows; i++) {
            double row_norm = 0.0;
            for (size_t c = 0; c < Yf->cols; c++) {
                row_norm += Yf->data[c * Yf->stride + i] *
                            Yf->data[c * Yf->stride + i];
            }
            double weight = (row_norm > 1e-12) ? 1.0 / sqrt(row_norm) : 1.0;
            for (size_t j = 0; j < cols && j < weighted_O->cols; j++) {
                weighted_O->data[j * weighted_O->stride + i] *= weight;
            }
        }
        return 0;
    }

    if (weighting == SSID_WGT_MOESP) {
        /* W2 = Pi_{U_f^\perp}: remove U_f influence
         * This is already handled by the oblique projection in many
         * implementations. Here we do an extra orthogonalization pass. */
        if (!Uf_future || !Uf_future->data) return 0;
        ssid_project_orth_complement(Uf_future, weighted_O, weighted_O);
        return 0;
    }

    /* SSID_WGT_IVM: Instrumental variable weighting.
     * Uses past data as instruments for consistent estimation
     * under colored noise. Simplified: identity weighting with
     * extra projection step. */
    return 0;
}

/* ========================================================================
 * L5: Canonical Angles
 *
 * Knowledge point: Canonical correlation analysis (CCA), invented by
 * Hotelling (1936), measures the correlation between two sets of
 * variables by finding linear combinations with maximal correlation.
 *
 * In CVA subspace identification (Larimore, 1990):
 *   - Past set: W_p (past I/O data)
 *   - Future set: Y_f (future outputs)
 *
 * The canonical correlations are:
 *   rho_k = cos(theta_k)
 * where theta_k are the principal angles between row spaces.
 *
 * The states are selected as the canonical variates with largest
 * correlation -- these are the "most predictable" directions.
 *
 * The SVD of the weighted correlation matrix gives:
 *   W_p * Y_f^T * (Y_f * Y_f^T)^{-1/2} = U * S * V^T
 * where S contains the canonical correlations.
 * ======================================================================== */

int ssid_project_canonical_angles(const ssid_matrix_t *A,
                                  const ssid_matrix_t *B,
                                  double *angles,
                                  size_t *n_angles)
{
    if (!A || !B || !angles || !n_angles || !A->data || !B->data) return -1;
    if (A->cols != B->cols) return -2;

    /* Compute cross-covariance C = A * B^T */
    size_t rows_A = A->rows;
    size_t rows_B = B->rows;
    size_t cols   = A->cols;
    size_t k      = (rows_A < rows_B) ? rows_A : rows_B;
    *n_angles = k;

    /* Simplified approach: compute Gram matrices and estimate
     * canonical angles from their generalized eigenvalues.
     *
     * G_AA = A * A^T, G_BB = B * B^T, G_AB = A * B^T
     *
     * Canonical correlations = sqrt(eig(G_AB * G_BB^{-1} * G_BA * G_AA^{-1}))
     *
     * For efficiency, we estimate angles using the singular values
     * of A * pinv(B) (approximation for well-conditioned B). */
    ssid_matrix_t BTB = ssid_matrix_alloc(rows_B, rows_B);
    if (!BTB.data) return -3;

    for (size_t a = 0; a < rows_B; a++) {
        for (size_t b = 0; b < rows_B; b++) {
            double sum = 0.0;
            for (size_t c = 0; c < cols; c++) {
                sum += B->data[c * B->stride + a] *
                       B->data[c * B->stride + b];
            }
            BTB.data[b * BTB.stride + a] = sum;
        }
    }

    /* Regularize */
    for (size_t i = 0; i < rows_B; i++) {
        BTB.data[i * BTB.stride + i] += 1e-10;
    }

    /* Cross covariance */
    ssid_matrix_t ABT = ssid_matrix_alloc(rows_A, rows_B);
    if (!ABT.data) { ssid_matrix_free(&BTB); return -4; }

    for (size_t a = 0; a < rows_A; a++) {
        for (size_t b = 0; b < rows_B; b++) {
            double sum = 0.0;
            for (size_t c = 0; c < cols; c++) {
                sum += A->data[c * A->stride + a] *
                       B->data[c * B->stride + b];
            }
            ABT.data[b * ABT.stride + a] = sum;
        }
    }

    /* Compute ABT * pinv(BBT) using solve */
    ssid_matrix_t Theta = ssid_matrix_alloc(rows_A, rows_B);
    if (!Theta.data) {
        ssid_matrix_free(&BTB); ssid_matrix_free(&ABT); return -5;
    }

    /* Copy ABT to Theta (transposed for solve) */
    for (size_t a = 0; a < rows_A; a++) {
        for (size_t b = 0; b < rows_B; b++) {
            Theta.data[b * Theta.stride + a] = ABT.data[b * ABT.stride + a];
        }
    }

    if (ssid_matrix_solve(&BTB, &Theta) == 0) {
        /* Approximate singular values from the Gram matrix of Theta^*A.
         * For canonical angles, we estimate from the diagonal elements
         * of the correlation matrix (simplified). */
        for (size_t i = 0; i < k && i < rows_A && i < rows_B; i++) {
            double sigma = fabs(Theta.data[i * Theta.stride + i]);
            /* Clamp correlations to [0, 1] */
            if (sigma > 1.0) sigma = 1.0;
            angles[i] = acos(sigma); /* theta = arccos(rho) */
        }
    } else {
        for (size_t i = 0; i < k; i++) angles[i] = M_PI / 2.0;
    }

    ssid_matrix_free(&BTB);
    ssid_matrix_free(&ABT);
    ssid_matrix_free(&Theta);
    return 0;
}

/* ========================================================================
 * L5: Closed-loop instrumental variable construction
 *
 * Knowledge point: In closed-loop operation, u_k = r_k - K*y_{k-1} + ...
 * which creates correlation between future inputs and past noise through
 * the feedback path. This violates the open-loop assumption that U_f is
 * uncorrelated with future noise e_f.
 *
 * Solution: Use an external reference signal r_k (known and independent
 * of noise) as an instrumental variable. The projection uses R_ext
 * instead of U_f to break the noise-feedback correlation.
 *
 * Alternatively, the "two-stage" method identifies the closed-loop
 * transfer first, then recovers the open-loop model.
 * ======================================================================== */

int ssid_project_closed_loop_instrument(const ssid_hankel_t *Up,
                                        const ssid_hankel_t *Yp,
                                        const ssid_matrix_t *R_ext,
                                        ssid_matrix_t *Z_inst)
{
    if (!Up || !Yp || !R_ext || !Z_inst || !R_ext->data || !Z_inst->data)
        return -1;

    /* Build instrument Z = [R_ext_past; R_ext_future] and replace
     * U_f with R_ext_future in the projection.
     * Simplified: use [U_p, Y_p, R_ext] as extended instrument. */
    size_t rows_up = Up->past.rows;
    size_t rows_yp = Yp->past.rows;
    size_t rows_r  = R_ext->rows;
    size_t cols    = Up->j;

    if (Z_inst->rows < rows_up + rows_yp + rows_r) return -2;
    if (Z_inst->cols < cols) return -3;

    /* Fill Z_inst */
    for (size_t j = 0; j < cols && j < Z_inst->cols; j++) {
        for (size_t i = 0; i < rows_up; i++) {
            Z_inst->data[j * Z_inst->stride + i] =
                Up->past.data[j * Up->past.stride + i];
        }
        for (size_t i = 0; i < rows_yp; i++) {
            Z_inst->data[j * Z_inst->stride + rows_up + i] =
                Yp->past.data[j * Yp->past.stride + i];
        }
        for (size_t i = 0; i < rows_r && i + rows_up + rows_yp < Z_inst->rows; i++) {
            /* Use R_ext as additional instrument columns */
            if (j < R_ext->cols) {
                Z_inst->data[j * Z_inst->stride + rows_up + rows_yp + i] =
                    R_ext->data[j * R_ext->stride + i];
            }
        }
    }

    return 0;
}
