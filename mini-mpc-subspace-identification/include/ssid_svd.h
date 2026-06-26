#ifndef SSID_SVD_H
#define SSID_SVD_H

#include "ssid_defs.h"

/* ============================================================================
 * ssid_svd.h -- SVD Computation and Order Selection for 4SID
 *
 * Reference: Golub & Van Loan (2013), "Matrix Computations", Chapter 8
 *            Van Overschee & De Moor (1996), Section 2.4
 *            Ljung (1999), Section 16.5
 *
 * The SVD is the central numerical engine of subspace identification.
 * Given the weighted projection matrix O_i (size i*n_y x j), compute:
 *
 *   O_i = U * S * V^T = [U1 U2] * [S1 0; 0 S2] * [V1 V2]^T
 *
 * Then:
 *   Gamma_i = U1 * S1^{1/2}          (extended observability matrix)
 *   X_i_hat = S1^{1/2} * V1^T       (Kalman state sequence)
 *   system order n_x = rank(S1)     (size of dominant singular values)
 *
 * Each function teaches an independent knowledge point about SVD-based
 * system identification.
 * ============================================================================ */

/* ---------------------------------------------------------------------------
 * L5: SVD Computation
 * ---------------------------------------------------------------------------
 */

/* L4: Compute the economy-size SVD of a matrix A.
 *
 * A = U * diag(s) * V^T
 *
 * Uses a two-stage algorithm:
 *   1. Bidiagonalize A via Householder reflections
 *   2. Compute SVD of bidiagonal matrix via implicit QR (Golub-Kahan)
 *
 * Knowledge point: The SVD reveals the "principal directions" in data.
 *   In subspace ID, the left singular vectors of the projection matrix
 *   span the observability subspace; the number of non-zero singular
 *   values equals the system order.
 *
 * Complexity: O(m*n*min(m,n)). */
ssid_svd_t ssid_svd_compute(const ssid_matrix_t *A);

/* L4: Truncate SVD to keep only the k largest singular values.
 *
 * Knowledge point: Truncation is the key step for model order reduction.
 *   Keeping too few modes loses dynamics; keeping too many includes noise.
 *   The singular value spectrum provides visual guidance ("scree plot").
 *
 * Complexity: O(m*k + n*k). */
ssid_svd_t ssid_svd_truncate(const ssid_svd_t *svd, size_t k);

/* L5: Reconstruct matrix from truncated SVD (best rank-k approximation).
 *
 * A_k = U_k * diag(s_0..s_{k-1}) * V_k^T
 *
 * By Eckart-Young theorem, A_k is the optimal rank-k approximation
 * in both spectral and Frobenius norms.
 *
 * Knowledge point: Low-rank approximation via SVD is the theoretical
 *   justification for subspace identification. The "true" system
 *   produces a rank-n_x projection matrix; noise adds small singular
 *   values. Truncation separates signal from noise.
 *
 * Complexity: O(m*n*k). */
ssid_matrix_t ssid_svd_reconstruct(const ssid_svd_t *svd, size_t k);

/* ---------------------------------------------------------------------------
 * L5: Order Selection Methods
 * ---------------------------------------------------------------------------
 */

/* L5: Select system order using the singular value gap criterion.
 *
 * Look for the largest relative gap: argmax_k (s_{k-1} / s_k).
 * This is the most common heuristic in practice (Ljung, Sec 16.5).
 *
 * Knowledge point: The singular value "scree plot" -- the point where
 *   singular values transition from "large" (signal) to "small" (noise)
 *   indicates the system order. An unambiguous gap implies a well-defined
 *   deterministic system.
 *
 * Complexity: O(k) where k is the number of singular values. */
size_t ssid_order_svd_gap(const double *s, size_t k, double rel_tol);

/* L5: Compute AIC (Akaike Information Criterion) for each candidate order.
 *
 * AIC(n) = N * log(det(Sigma_e(n))) + 2 * (n*(n_y+n_u) + n*n_y + n_y*n_u)
 *
 * where Sigma_e(n) is the innovation covariance estimate.
 * The AIC balances model fit against complexity; the minimizer is chosen.
 *
 * Knowledge point: AIC provides a statistically-grounded alternative to
 *   gap heuristics. It assumes Gaussian innovations and correct model
 *   structure (Ljung, Sec 16.4).
 *
 * Complexity: O(k * n_y^3) for covariance estimation per candidate. */
size_t ssid_order_aic(const double *s, size_t k, size_t N,
                      size_t n_y, size_t n_u,
                      double *aic_values);

/* L5: Compute BIC (Bayesian Information Criterion) for each candidate order.
 *
 * BIC(n) = N * log(det(Sigma_e(n))) + log(N) * n * (n_y + n_u + ...)
 *
 * BIC penalizes complexity more heavily than AIC (log(N) vs 2),
 * making it asymptotically consistent -- it will select the true order
 * as N goes to infinity.
 *
 * Knowledge point: BIC's stronger penalty makes it preferred for large
 *   datasets where AIC tends to overfit. The tradeoff depends on N.
 *
 * Complexity: Same as AIC, O(k * n_y^3). */
size_t ssid_order_bic(const double *s, size_t k, size_t N,
                      size_t n_y, size_t n_u,
                      double *bic_values);

/* L5: Compute MDL (Minimum Description Length) criterion.
 *
 * MDL(n) = -log(L) + 0.5 * p * log(N)
 * where p = total number of free parameters in an n-th order model.
 *
 * Rissanen's MDL principle: The best model compresses the data most.
 *   MDL is equivalent to BIC in the large-sample limit under certain
 *   regularity conditions.
 *
 * Knowledge point: MDL connects system identification to information
 *   theory and data compression -- a deeper theoretical foundation.
 *
 * Complexity: O(k * n_y^3). */
size_t ssid_order_mdl(const double *s, size_t k, size_t N,
                      size_t n_y, size_t n_u,
                      double *mdl_values);

/* L5: Unified order selection using the configured criterion.
 *
 * Dispatches to the specific method based on ssid_config_t.order_crit.
 * Returns the selected order and optionally fills the criteria array.
 *
 * Knowledge point: In industrial practice, multiple criteria are
 *   checked and engineering judgment applied (e.g., AspenTech DMC3
 *   shows both SVD plot and AIC, letting the engineer choose).
 *
 * Complexity: O(k * n_y^3) worst case. */
size_t ssid_order_select(const double *s, size_t k,
                         const ssid_config_t *cfg,
                         size_t N, size_t n_y, size_t n_u,
                         double *criteria_out);

/* L3: Compute the effective rank (number of singular values above threshold).
 *
 * rank_tol = max(m,n) * eps(max(s)) * tolerance_factor
 *
 * Knowledge point: Numerical rank determination distinguishes between
 *   "mathematical rank" (exact zero singular values) and "numerical rank"
 *   (singular values below machine precision times tolerance). Essential
 *   for finite-precision subspace ID.
 *
 * Complexity: O(k). */
size_t ssid_svd_effective_rank(const double *s, size_t k,
                               size_t m, size_t n, double tol_factor);

/* L4: Extract observability matrix from SVD: Gamma = U * sqrt(S).
 *
 * Knowledge point: The left singular vectors (scaled by sqrt of singular
 *   values) form the extended observability matrix. This is the key
 *   connection between SVD geometry and system theory.
 *
 * Complexity: O(m*n*k). */
int ssid_svd_extract_observability(const ssid_svd_t *svd,
                                   size_t n_x,
                                   ssid_matrix_t *Gamma);

/* L4: Extract state sequence from SVD: X = sqrt(S) * V^T.
 *
 * Knowledge point: The right singular vectors (scaled) give the state
 *   sequence. Combined with observability extraction, this completes
 *   the factorization O_i = Gamma_i * X_i.
 *
 * Complexity: O(k*n*min(k,n)). */
int ssid_svd_extract_state(const ssid_svd_t *svd,
                           size_t n_x,
                           ssid_matrix_t *X);

/* L4: Estimate {A, C} from Gamma_i (shift invariance property).
 *
 * Gamma_i(1:end-n_y, :) * A = Gamma_i(n_y+1:end, :)
 *
 * Solved via least squares (or total least squares for EIV).
 * C is simply the first n_y rows of Gamma_i.
 *
 * Knowledge point: The shift-invariance property of the observability
 *   matrix allows A and C to be recovered from the SVD factors without
 *   iterative optimization. This is the distinguishing feature of
 *   subspace methods over PEM (prediction error methods).
 *
 * Complexity: O((i-1)^2 * n_y^2 * n_x). */
int ssid_svd_estimate_AC(const ssid_matrix_t *Gamma,
                         size_t n_x, size_t n_y,
                         ssid_matrix_t *A_est,
                         ssid_matrix_t *C_est);

/* L4: Estimate {B, D} from the linear regression:
 *
 *   y_k - C*x_k = B*u_k + D*u_k  (steady-state)
 *
 * After A and C are estimated, B and D are obtained by linear
 * least squares on the residuals.
 *
 * Knowledge point: The "second step" of 4SID -- after the state
 *   sequence is known, B and D are estimated by standard regression.
 *   This two-step approach (first the state, then the I/O matrices)
 *   avoids the non-convexity of simultaneous estimation.
 *
 * Complexity: O(N * (n_y+n_u)^2). */
int ssid_svd_estimate_BD(const ssid_matrix_t *X,
                         const ssid_matrix_t *Y,
                         const ssid_matrix_t *U,
                         const ssid_matrix_t *A_est,
                         const ssid_matrix_t *C_est,
                         ssid_matrix_t *B_est,
                         ssid_matrix_t *D_est);

/* Free SVD result structure */
void ssid_svd_free(ssid_svd_t *svd);

/* Print singular values for inspection */
void ssid_svd_print_singular_values(const ssid_svd_t *svd);

/* L8: Compute the condition number of the weighting matrix for
 * numerical stability analysis. */
double ssid_svd_weighting_condition(const ssid_matrix_t *W);

#endif /* SSID_SVD_H */
