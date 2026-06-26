#ifndef SSID_HANKEL_H
#define SSID_HANKEL_H

#include "ssid_defs.h"

/* ============================================================================
 * ssid_hankel.h -- Block Hankel Matrix Operations for Subspace Identification
 *
 * Reference: Van Overschee & De Moor (1996), Chapter 1-2
 *            Ljung (1999), Chapter 10
 *
 * Block Hankel matrices are the fundamental data structure of subspace
 * identification. The key insight (Theorem 1, Van Overschee & De Moor):
 *
 *   Given input-output data {u_k, y_k}, the block Hankel matrices U_p, Y_p
 *   (past) and U_f, Y_f (future) encode the complete dynamic information.
 *   The oblique projection of Y_f along U_f onto [U_p; Y_p] recovers the
 *   extended observability matrix times the state sequence.
 *
 * Each function implements an independent knowledge point about Hankel
 * matrix construction, manipulation, or analysis.
 * ============================================================================ */

/* L2: Construct a block Hankel matrix from a flat data sequence.
 *
 * Given a data matrix Z of size N x m (N time samples, m variables),
 * construct the block Hankel matrix with i block rows.
 *
 * Result: (i*m) x j matrix where j = N - i + 1.
 *
 * Knowledge point: Mapping from time-series data to the Hankel structure
 *   that encodes linear time-invariant dynamics. This is the normal
 *   form for subspace algorithms.
 *
 * Complexity: O(N*m*i) time, O(i*m*j) space. */
ssid_hankel_t ssid_hankel_build(const ssid_matrix_t *Z, size_t i);

/* L2: Split a full block Hankel into past/future halves.
 *
 * Given a block Hankel H_{0|2i-1} with 2i block rows, return a structure
 * with .past = rows 0..i-1 and .future = rows i..2i-1.
 *
 * Knowledge point: The past/future split that enables the fundamental
 *   subspace projection: future outputs projected onto past data
 *   recovers the observability matrix * state sequence.
 *
 * Complexity: O(1) -- returns views, no copy. */
ssid_hankel_t ssid_hankel_split(const ssid_hankel_t *H, size_t i);

/* L2: Construct combined past data matrix W_p = [U_p; Y_p].
 *
 * This matrix stacks past inputs on top of past outputs to form
 * the complete "past" regressor used in subspace projections.
 *
 * W_p dimension: i*(n_u + n_y) x j
 *
 * Knowledge point: Instrumental variable / regressor matrix in
 *   subspace identification. W_p acts as an instrument that is
 *   uncorrelated with future noise but correlated with the state.
 *
 * Complexity: O(i*(n_u+n_y)*j) for concatenation. */
ssid_matrix_t ssid_hankel_past_data(const ssid_hankel_t *Up,
                                    const ssid_hankel_t *Yp);

/* L3: Compute the optimal number of block rows i for Hankel matrices.
 *
 * Rule of thumb (Ljung, Sec 10.6): i should be at least log_2(N) * n_x_guess
 * to capture all system dynamics. More precisely:
 *   i >= 2 * n_x / n_y   (deterministic case)
 *   i >= n_x             (stochastic case, with CVA weighting)
 *
 * Knowledge point: The selection of i determines the maximum identifiable
 *   system order: n_x <= i * n_y. Too small i loses dynamics; too large
 *   increases variance.
 *
 * Complexity: O(1). */
size_t ssid_hankel_optimal_i(size_t N, size_t n_y, size_t n_x_guess);

/* L3: Verify that a block Hankel matrix satisfies the persistent
 * excitation condition.
 *
 * Returns the estimated order of persistent excitation = rank(U_p).
 * For identifiability of order n_x, we need rank(U_p) >= 2*i*n_u + n_x.
 *
 * Knowledge point: Practical check for the identifiability condition
 *   (persistence of excitation). Uses QR factorization with column
 *   pivoting to estimate numerical rank.
 *
 * Complexity: O(i*n_u * j^2) for rank estimation. */
size_t ssid_hankel_persist_excitation_rank(const ssid_hankel_t *Up, double tol);

/* L3: Compute the data equation matrix.
 *
 * The "data equation" is the fundamental matrix relation:
 *   Y_f = Gamma_i * X_i + H_i^d * U_f + H_i^s * M_f + N_f
 *
 * where Gamma_i is the extended observability matrix,
 * H_i^d is the deterministic Toeplitz matrix (Markov parameters),
 * H_i^s is the stochastic Toeplitz matrix.
 *
 * This function extracts the observability range space by
 * eliminating the U_f term via oblique projection.
 *
 * Knowledge point: The data equation is the starting point for
 *   all subspace identification algorithms. Understanding it
 *   separates 4SID from classical prediction error methods.
 *
 * Complexity: O(i^2 * n_y^2 * j) for projection computation. */
int ssid_hankel_data_equation(const ssid_hankel_t *Yf,
                              const ssid_hankel_t *Uf,
                              const ssid_hankel_t *Yp,
                              const ssid_hankel_t *Up,
                              ssid_matrix_t *Gamma_X);

/* L4: Extract Markov parameters from the identified Toeplitz structure.
 *
 * Once the state sequence is estimated, the deterministic Toeplitz
 * matrix H_i^d (containing impulse response parameters) can be
 * recovered via least squares: H_i^d = Y_f * pinv([U_f; X_i]) * ...
 *
 * The first block row of H_i^d gives D (feedthrough).
 * Subsequent rows give C*A^k*B for k = 0, 1, ..., i-2.
 *
 * Knowledge point: Recovery of impulse response / Markov parameters
 *   from subspace estimates. Connects 4SID to classical FIR modeling
 *   used in DMC (Dynamic Matrix Control).
 *
 * Complexity: O(i^2 * (n_u+n_x)^2 * j). */
int ssid_hankel_markov_parameters(const ssid_matrix_t *Gamma_X,
                                  const ssid_hankel_t *Uf,
                                  const ssid_hankel_t *Yf,
                                  ssid_matrix_t *H);

/* L5: Estimate the number of block rows needed using SVC (singular value
 * criterion). Tests increasing i and monitors the singular value gap.
 *
 * Knowledge point: Data-driven selection of i via rank-revealing
 *   spectral analysis. Avoids under-parameterization.
 *
 * Complexity: O(j^3) per i candidate. */
size_t ssid_hankel_estimate_i_svc(const ssid_matrix_t *Z,
                                  size_t n_y, size_t N,
                                  size_t i_min, size_t i_max,
                                  double tol);

/* Free a Hankel structure (frees past and future matrices). */
void ssid_hankel_free(ssid_hankel_t *H);

#endif /* SSID_HANKEL_H */
