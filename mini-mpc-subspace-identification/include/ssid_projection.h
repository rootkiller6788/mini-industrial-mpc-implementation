#ifndef SSID_PROJECTION_H
#define SSID_PROJECTION_H

#include "ssid_defs.h"
#include "ssid_hankel.h"

/* ============================================================================
 * ssid_projection.h -- Orthogonal and Oblique Projections for 4SID
 *
 * Reference: Van Overschee & De Moor (1996), Chapter 1.4, Chapter 2
 *            Golub & Van Loan (2013), "Matrix Computations", Chapter 5
 *
 * Subspace identification algorithms are built on two fundamental geometric
 * operations:
 *
 *   1. Orthogonal projection: Pi_{A} B = A * (A^T A)^{-1} A^T * B
 *      Projects the row space of B onto the row space of A.
 *
 *   2. Oblique projection: B /_A C
 *      Projects the row space of B along the row space of C onto A.
 *      B /_A C = [B / C^perp] [A / C^perp]^{-1} A
 *
 * These projections are implemented numerically as LQ decompositions
 * for stability (square-root algorithms).
 *
 * Theorem (Van Overschee & De Moor, Theorem 2):
 *   The oblique projection O_i = Y_f /_{U_f} [U_p; Y_p] has the property:
 *     O_i = Gamma_i * X_i_hat
 *   where Gamma_i is the extended observability matrix and X_i_hat
 *   is the Kalman filter state sequence.
 *
 * Each function maps to an independent knowledge point in geometric
 * subspace theory.
 * ============================================================================ */

/* L3: Orthogonal projection of B onto the row space of A.
 *
 * Pi_A(B) = A * pinv(A) * B
 *
 * Numerically stable implementation via LQ factorization:
 *   [A; B] = [L11 0; L21 L22] * Q^T
 *   Pi_A(B) = L21 * Q1^T  (first block row of Q^T)
 *
 * Knowledge point: The orthogonal projection is the mathematical
 *   foundation for separating signal from noise in subspace methods.
 *   Using LQ instead of normal equations avoids squaring the condition
 *   number.
 *
 * Complexity: O(m*n^2 + m^2*n) where A is m x n. */
int ssid_project_orthogonal(const ssid_matrix_t *A,
                            const ssid_matrix_t *B,
                            ssid_matrix_t *Pi_A_B);

/* L3: Oblique projection of Y along U onto W.
 *
 * Y /_W U = [Y / U^perp] * [W / U^perp]^{-1} * W
 *
 * This is the KEY operation in subspace identification:
 *   O_i = Y_f /_{U_f} W_p
 *
 * Knowledge point: The oblique projection removes the influence of
 *   future inputs (which are correlated with future outputs through
 *   the deterministic dynamics) while preserving the state information
 *   encoded in past data.
 *
 * Implementation: Uses two consecutive LQ decompositions for numerical
 * stability (Verhaegen square-root algorithm).
 *
 * Complexity: O(m^3) where m = total rows. */
int ssid_project_oblique(const ssid_matrix_t *Y,
                         const ssid_matrix_t *U,
                         const ssid_matrix_t *W,
                         ssid_matrix_t *O_i);

/* L3: Compute the orthogonal complement projection.
 *
 * Pi_{A^perp}(B) = B - Pi_A(B)
 *
 * Used in MOESP algorithm to eliminate the deterministic input
 * contribution from future outputs.
 *
 * Knowledge point: Deflation / null-space projection. The MOESP
 *   algorithm uses this to first remove U_f's influence, then
 *   extract the observability subspace from the residual.
 *
 * Complexity: O(m*n^2 + m^2*n). */
int ssid_project_orth_complement(const ssid_matrix_t *A,
                                 const ssid_matrix_t *B,
                                 ssid_matrix_t *result);

/* L4: Compute LQ decomposition: A = L * Q where L is lower triangular.
 *
 * Uses Householder reflections (Golub & Van Loan, Algorithm 5.2.1).
 * In-place operation: A is overwritten with L in lower triangle and
 * Householder vectors in the strict upper triangle.
 *
 * Knowledge Point: LQ is the "workhorse" decomposition of subspace ID.
 *   Unlike QR (applied to columns), LQ applied to rows corresponds to
 *   sequential orthogonalization of row vectors -- perfect for projecting
 *   Hankel matrix rows.
 *
 * Complexity: O(m*n^2) for m >= n, O(m^2*n) for m < n.
 * Reference: Golub & Van Loan (2013), Section 5.2. */
int ssid_lq_decompose(ssid_lq_t *lq);

/* L4: Extract L matrix from LQ decomposition. */
void ssid_lq_extract_L(const ssid_lq_t *lq, ssid_matrix_t *L);

/* L4: Apply Q^T from LQ decomposition to a matrix.
 * Useful for forming oblique projections without explicit Q. */
int ssid_lq_apply_QT(const ssid_lq_t *lq, ssid_matrix_t *B, const char *side);

/* L4: Free LQ decomposition resources. */
void ssid_lq_free(ssid_lq_t *lq);

/* L5: Compute the "R" factor in the combined LQ decomposition used
 * by the unified 4SID framework.
 *
 *   [U_f; W_p; Y_f] = [R11 0   0  ] [Q1^T]
 *                      [R21 R22 0  ] [Q2^T]
 *                      [R31 R32 R33] [Q3^T]
 *
 * The oblique projection O_i is then given by R32 * inv(R22) * [R21 R22] * [Q1; Q2].
 *
 * Knowledge point: The "R-factor" formulation of subspace ID allows
 *   computing all three major algorithms (N4SID, MOESP, CVA) from
 *   a single LQ decomposition by applying different weightings to
 *   the R32 block. This unifies the 4SID family (Van Overschee & De Moor,
 *   Theorem 3).
 *
 * Complexity: O(m^3) for the single LQ. */
int ssid_project_combined_LQ(const ssid_hankel_t *Uf_past,
                             const ssid_hankel_t *Uf_future,
                             const ssid_hankel_t *Wp,
                             const ssid_hankel_t *Yf,
                             ssid_matrix_t *R32,
                             ssid_matrix_t *R22);

/* L5: Apply subspace weighting to the projection result.
 *
 * weighted_O_i = W1 * O_i * W2
 *
 * Different weightings yield different 4SID variants:
 *   W1=I, W2=I        -> N4SID
 *   W1=I, W2=Pi_perp  -> MOESP
 *   W1=cov(Y_f)^{-1/2}, W2=I -> CVA
 *
 * Knowledge point: The weighting matrices W1 and W2 are the
 *   "unifying degree of freedom" that span the 4SID family.
 *   W1 controls stochastic consistency; W2 controls deterministic
 *   accuracy.
 *
 * Complexity: O(m^2 * n) for matrix multiplies. */
int ssid_project_apply_weighting(const ssid_matrix_t *O_i,
                                 const ssid_matrix_t *Yf,
                                 const ssid_matrix_t *Uf_future,
                                 ssid_weighting_t weighting,
                                 ssid_matrix_t *weighted_O);

/* L5: Compute canonical angles between the row spaces of two matrices.
 *
 * Canonical angles theta_k = arccos(sigma_k) where sigma_k are the
 * singular values of Pi_A(B). Used in CVA for measuring the "correlation"
 * between past and future -- the principal angles determine the state.
 *
 * Knowledge point: Canonical Correlation Analysis (Hotelling, 1936)
 *   applied to system identification. The CVA algorithm selects states
 *   as the canonical variates with largest correlation between past
 *   and future data.
 *
 * Complexity: O(m^3) for SVD. */
int ssid_project_canonical_angles(const ssid_matrix_t *A,
                                  const ssid_matrix_t *B,
                                  double *angles,
                                  size_t *n_angles);

/* L5: Compute the compressed "past" data matrix used for
 * closed-loop subspace identification.
 *
 * In closed-loop identification (where u_k = r_k - K*y_k),
 * the standard open-loop projection is biased because u_f
 * is correlated with past noise through the feedback.
 *
 * The solution uses an instrumental variable approach with
 * an external reference signal or known controller.
 *
 * Knowledge point: Closed-loop identifiability requires
 *   either an external persistently exciting reference signal,
 *   or a nonlinear/time-varying controller (e.g., GBN signal),
 *   or two-stage identification.
 *
 * Complexity: O(m*n*j). */
int ssid_project_closed_loop_instrument(const ssid_hankel_t *Up,
                                        const ssid_hankel_t *Yp,
                                        const ssid_matrix_t *R_ext,
                                        ssid_matrix_t *Z_inst);

#endif /* SSID_PROJECTION_H */
