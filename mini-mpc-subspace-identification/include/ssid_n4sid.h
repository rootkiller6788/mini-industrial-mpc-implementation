#ifndef SSID_N4SID_H
#define SSID_N4SID_H

#include "ssid_defs.h"
#include "ssid_hankel.h"
#include "ssid_projection.h"
#include "ssid_svd.h"

/* ============================================================================
 * ssid_n4sid.h -- N4SID Algorithm (Van Overschee & De Moor, 1994)
 *
 * Reference: Van Overschee & De Moor (1994), "N4SID: Subspace algorithms
 *            for the identification of combined deterministic-stochastic systems"
 *            Automatica, 30(1), 75-93.
 *
 * N4SID (Numerical algorithms for Subspace State Space System IDentification)
 * is the canonical subspace identification algorithm. It handles combined
 * deterministic-stochastic systems in innovation form:
 *
 *   x_{k+1} = A x_k + B u_k + K e_k
 *   y_k     = C x_k + D u_k + e_k          (e_k ~ N(0, R))
 *
 * Key steps:
 *   1. Construct block Hankel matrices U_p, U_f, Y_p, Y_f
 *   2. Compute oblique projection O_i = Y_f /_{U_f} W_p
 *   3. Apply weighting: W1 * O_i * W2
 *   4. SVD of weighted projection -> order selection + observability
 *   5. Shift-invariance -> A, C
 *   6. Linear regression -> B, D
 *   7. Innovation analysis -> K, R
 *
 * Each function maps to an independent knowledge point in the N4SID
 * algorithm family.
 * ============================================================================ */

/* ---------------------------------------------------------------------------
 * L5: Main N4SID Algorithm
 * ---------------------------------------------------------------------------
 */

/* L5: Complete N4SID identification of an LTI state-space model.
 *
 * This is the main entry point. Given I/O data and configuration,
 * it executes the full N4SID pipeline and returns an ssid_result_t.
 *
 * Knowledge point: The full N4SID algorithm unifies earlier subspace
 *   methods (MOESP, CVA) into a single framework parameterized by
 *   weighting matrices W1 and W2. The choice of weighting determines
 *   whether the algorithm is more like MOESP (deterministic focus),
 *   CVA (stochastic focus), or classical N4SID (balanced).
 *
 * Algorithmic steps (all implemented in sub-functions):
 *   Step 1: Data preprocessing (trend removal, scaling)
 *   Step 2: Block Hankel construction (past/future I/O)
 *   Step 3: Oblique projection O_i = Y_f /_{U_f} [U_p; Y_p]
 *   Step 4: Weighted projection M = W1 * O_i * W2
 *   Step 5: SVD of M -> observability matrix + state sequence
 *   Step 6: Order selection (SVD gap / AIC / BIC / MDL)
 *   Step 7: Estimate {A, C} from observability shift-invariance
 *   Step 8: Estimate {B, D} from linear regression
 *   Step 9: Estimate {K} innovation gain from Kalman filter residuals
 *   Step 10: Model validation (fit, residuals, stability)
 *
 * Complexity: O(N^3) dominated by LQ/SVD steps.
 * Reference: Van Overschee & De Moor (1994), Algorithm 1, p. 82. */
ssid_result_t ssid_n4sid_identify(const ssid_data_t *data,
                                  const ssid_config_t *cfg);

/* ---------------------------------------------------------------------------
 * L5: N4SID Sub-steps (exposed for testing and educational purposes)
 * ---------------------------------------------------------------------------
 */

/* L5: Step 1 -- Data preprocessing.
 *
 * Removes mean (detrending) and optionally normalizes to unit
 * variance. Detrending is essential for subspace methods because
 * non-zero means cause constant offsets in the Hankel structure
 * that bias the projection.
 *
 * Knowledge point: Detrending and scaling are mandatory pre-steps
 *   for real-world industrial data where measurements drift with
 *   ambient temperature, raw material variations, etc.
 *
 * Complexity: O(N * (n_y+n_u)). */
int ssid_n4sid_preprocess(ssid_data_t *data, const ssid_config_t *cfg);

/* L5: Step 2 -- Build data equation matrices from preprocessed data.
 *
 * Constructs U_p, U_f, Y_p, Y_f block Hankel matrices with the
 * appropriate number of block rows i.
 *
 * Knowledge point: The choice of i affects the maximum identifiable
 *   order (n_x <= i*n_y). In practice, i is chosen as
 *   i = max(2*n_x_guess / n_y, 10) for robustness.
 *
 * Complexity: O(N*i*(n_y+n_u)). */
int ssid_n4sid_build_hankels(const ssid_data_t *data,
                             size_t i,
                             ssid_hankel_t *Up,
                             ssid_hankel_t *Uf,
                             ssid_hankel_t *Yp,
                             ssid_hankel_t *Yf);

/* L5: Step 3 -- Compute oblique projection using LQ decomposition.
 *
 * Implementation of the unified theorem (Van Overschee & De Moor, Theorem 3):
 *
 *   O_i = Gamma_i * X_i  (the oblique projection factorizes into
 *                          observability matrix * state sequence)
 *
 * Knowledge point: The geometric interpretation of subspace ID.
 *   The oblique projection eliminates the influence of future inputs
 *   while preserving the state information. The projection theorem
 *   guarantees that the row space of O_i equals the row space of
 *   the true state sequence (up to similarity transform).
 *
 * Complexity: O((2i*(n_y+n_u))^2 * j) for LQ. */
int ssid_n4sid_oblique_projection(const ssid_hankel_t *Up,
                                  const ssid_hankel_t *Uf,
                                  const ssid_hankel_t *Yp,
                                  const ssid_hankel_t *Yf,
                                  ssid_matrix_t *O_i);

/* L5: Step 4 -- Apply weighting and compute SVD for order determination.
 *
 * M = W1 * O_i * W2,  then SVD(M)
 *
 * Different weightings yield different variants:
 *   CVA weighting -> A canonical correlation viewpoint
 *   MOESP weighting -> Instrumental variable viewpoint
 *
 * Knowledge point: Weighting controls the asymptotic properties.
 *   CVA weighting makes the estimate asymptotically efficient
 *   (minimum variance), while MOESP weighting is simpler to
 *   compute and handles deterministic systems better.
 *
 * Complexity: O((i*n_y)^3) for SVD. */
int ssid_n4sid_weighted_svd(const ssid_matrix_t *O_i,
                            const ssid_weighting_t weighting,
                            const ssid_hankel_t *Uf,
                            const ssid_hankel_t *Yf,
                            ssid_svd_t *svd_out);

/* L5: Step 5 -- Order selection from singular values. */
size_t ssid_n4sid_select_order(const ssid_svd_t *svd,
                               const ssid_config_t *cfg,
                               size_t N);

/* L5: Step 6 -- Compute A and C matrices.
 *
 * Uses shift-invariance: Gamma_i(1:end-n_y, :) * A = Gamma_i(n_y+1:end, :)
 * Solved via least squares with column pivoting for numerical stability.
 *
 * Knowledge point: Shift-invariance is the property that makes subspace
 *   methods computationally efficient. The observability matrix has a
 *   block structure where each block row is C*A^k. This means the
 *   "shifted" observability matrix differs by exactly one factor of A.
 *
 * Complexity: O((i-1)*n_y * n_x^2). */
int ssid_n4sid_estimate_AC(const ssid_matrix_t *Gamma,
                           size_t n_x, size_t n_y,
                           ssid_matrix_t *A, ssid_matrix_t *C);

/* L5: Step 7 -- Compute B and D matrices via least squares.
 *
 * After the state sequence X is known, the output equation gives:
 *   y_k = C*x_k + D*u_k + e_k
 *
 * With x_k known, D can be estimated directly; B is then obtained
 * from the state equation by regressing x_{k+1} - A*x_k on u_k.
 *
 * Knowledge point: Two-step estimation (first subspace, then
 *   regression) avoids the non-convex simultaneous estimation
 *   problem. This is the key advantage of subspace methods over
 *   prediction error methods for MIMO systems.
 *
 * Complexity: O(N * n_x^2 + N * n_u^2). */
int ssid_n4sid_estimate_BD(const ssid_matrix_t *Gamma,
                           const ssid_svd_t *svd,
                           const ssid_data_t *data,
                           const ssid_matrix_t *A,
                           const ssid_matrix_t *C,
                           size_t n_x,
                           ssid_matrix_t *B, ssid_matrix_t *D);

/* L5: Step 8 -- Estimate Kalman gain K and innovation covariance.
 *
 * e_k = y_k - C*x_k_hat - D*u_k
 * K = A * P * C^T * (C*P*C^T + R)^{-1}  (steady-state Kalman gain)
 *
 * Knowledge point: The innovation form includes the Kalman gain K
 *   which models the stochastic part of the system. K is estimated
 *   from the residuals of the deterministic fit.
 *
 * Complexity: O(N * n_x * n_y + n_x^3). */
int ssid_n4sid_estimate_K(const ssid_matrix_t *X,
                          const ssid_matrix_t *A,
                          const ssid_matrix_t *B,
                          const ssid_matrix_t *C,
                          const ssid_matrix_t *D,
                          const ssid_data_t *data,
                          size_t n_x,
                          ssid_matrix_t *K, double *cov_e);

/* L5: Step 9 -- Compute model quality metrics.
 *
 * Computes NRMSE fit, residual whiteness test, stability check,
 * and controllability/observability tests.
 *
 * Knowledge point: Model validation is critical in industrial
 *   practice. A model that fits training data perfectly but
 *   fails the whiteness test will give poor MPC predictions.
 *
 * Complexity: O(N * n_x^2 + n_x^3) for eigenvalue computation. */
int ssid_n4sid_validate(const ssid_model_t *model,
                        const ssid_data_t *data,
                        ssid_result_t *result);

/* L5: Iterative refinement of N4SID estimates.
 *
 * After the initial estimate, the state sequence can be re-estimated
 * using the full Kalman filter and the model re-estimated (bootstrap).
 * Typically 2-3 iterations suffice for convergence.
 *
 * Knowledge point: Iterative refinement bridges subspace methods
 *   and PEM -- repeated Kalman smoothing + least squares converges
 *   to the same optimum as PEM under Gaussian noise, but with
 *   much better initialization.
 *
 * Complexity: O(n_iter * N * n_x^3) per iteration. */
int ssid_n4sid_refine(const ssid_data_t *data,
                      const ssid_config_t *cfg,
                      ssid_result_t *result,
                      size_t max_iterations);

/* ---------------------------------------------------------------------------
 * L8: Advanced N4SID variants
 * ---------------------------------------------------------------------------
 */

/* L8: Recursive N4SID for online/adaptive identification.
 *
 * Uses a sliding window of recent data to update the model
 * as new measurements arrive. Suitable for slowly time-varying
 * processes (e.g., catalyst deactivation, heat exchanger fouling).
 *
 * Knowledge point: Recursive subspace ID enables adaptive MPC
 *   where the model is continuously updated. Key challenge is
 *   maintaining computational efficiency for real-time operation.
 *
 * Complexity: O(w^3) per update where w is window size. */
int ssid_n4sid_recursive(const ssid_data_t *data_window,
                         const ssid_config_t *cfg,
                         const ssid_model_t *prev_model,
                         ssid_result_t *updated);

/* L8: Closed-loop N4SID with instrumental variables.
 *
 * Direct closed-loop identification using the controller as a
 * known filter. Implements the "projection" approach of
 * Van Overschee & De Moor (1997) for unbiased estimates under
 * output feedback.
 *
 * Knowledge point: Closed-loop ID is required when the process
 *   cannot be operated in open loop (unstable, or production
 *   constraints). Direct methods use the controller knowledge;
 *   indirect methods identify the closed-loop transfer function
 *   and then recover the open-loop model.
 *
 * Complexity: Similar to open-loop + extra projection step. */
int ssid_n4sid_closed_loop(const ssid_data_t *data,
                           const ssid_config_t *cfg,
                           const ssid_matrix_t *controller_params,
                           ssid_result_t *result);

/* ---------------------------------------------------------------------------
 * L7: Industrial vendor-specific N4SID wrappers
 * ---------------------------------------------------------------------------
 */

/* L7: N4SID configured for AspenTech DMC3-style identification.
 *
 * AspenTech DMC3 uses a proprietary subspace identification engine
 * based on N4SID with CVA weighting and automatic order selection
 * via singular value ratios. This wrapper emulates those settings.
 *
 * Knowledge point: Industrial MPC vendors (AspenTech, Honeywell,
 *   Shell, ABB) all use subspace identification internally but
 *   with different customization. Understanding the vendor-specific
 *   parameterization helps in technology selection and migration.
 *
 * Complexity: Same as standard N4SID. */
ssid_result_t ssid_n4sid_aspentech_dmc3(const ssid_data_t *data);

/* L7: N4SID configured for Honeywell Profit Controller identification.
 *
 * Honeywell Profit Controller uses a multi-step identification
 * approach: first an FIR model via correlation analysis, then
 * subspace refinement. This wrapper emulates the subspace stage.
 *
 * Knowledge point: The Honeywell approach combines classical
 *   correlation-based pre-identification with modern subspace
 *   methods for robustness against outliers and missing data.
 *
 * Complexity: Same as standard N4SID with extra outlier handling. */
ssid_result_t ssid_n4sid_honeywell_profit(const ssid_data_t *data);

#endif /* SSID_N4SID_H */
