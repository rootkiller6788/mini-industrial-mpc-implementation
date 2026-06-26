#ifndef MPC_ILLCOND_SENSITIVITY_H
#define MPC_ILLCOND_SENSITIVITY_H

#include "mpc_illcond_defs.h"

/* Compute Relative Gain Array (RGA) from steady-state gain matrix G.
 * RGA = G .* (G^{-1})^T  (element-wise product)
 * L4 Theorem (Bristol, 1966): RGA quantifies steady-state interaction.
 * RGA_ij near 1: good pairing. RGA_ij negative: unstable pairing.
 * RGA_ij >> 1: severe interaction, ill-conditioning indicator.
 *
 * Sum of each row = 1, sum of each column = 1 (for square G).
 * For non-square G, the pseudo-inverse is used.
 * Complexity: O(ny*nu*min(ny,nu)) plus O(min^3) for inverse. */
int mpc_sensitivity_rga(const mpc_matrix_t *G, mpc_matrix_t *RGA);

/* Compute the RGA condition number: ||RGA - I||_sum
 * A measure of overall interaction severity. Values > 1 indicate
 * significant interaction; values > 5 indicate severe ill-conditioning.
 * L4: Skogestad & Postlethwaite (2005, Section 3.3). */
double mpc_sensitivity_rga_cond(const mpc_matrix_t *RGA);

/* Compute condition number sensitivity: d(kappa)/d(A_ij)
 * L4 Theorem (Golub & Van Loan, 2013, Section 2.6):
 *   d(kappa_2(A))/d(A_ij) = (y_i * x_j) / ||x||_2^2
 * where x, y are the right and left singular vectors for sigma_min.
 *
 * This identifies which matrix entries most affect the condition number,
 * guiding sensor/actuator selection for improved conditioning.
 * Complexity: O(n^2) after SVD. */
int mpc_sensitivity_gradient(const mpc_svd_t *svd, mpc_matrix_t *grad);

/* Detect collinearity between pairs of gain matrix columns.
 * L4: For columns g_i, g_j of G, the collinearity is:
 *   coll(i,j) = |g_i^T g_j| / (||g_i|| * ||g_j||)
 *
 * Returns the maximum collinearity over all pairs and sets the
 * index pair (i, j) for the worst pair.
 * Complexity: O(nu^2 * ny). */
double mpc_sensitivity_collinearity(const mpc_matrix_t *G,
                                     size_t *worst_i, size_t *worst_j);

/* Estimate stiffness ratio from step response data.
 * L4: For first-order-plus-deadtime models, tau_i are the dominant
 * time constants. stiffness = max(tau_i) / min(tau_i).
 *
 * Uses the matrix of time constants from system identification.
 * High stiffness (>1e5) requires multi-rate MPC or stiff integrators.
 * Complexity: O(ny*nu). */
double mpc_sensitivity_stiffness(const double *tau, size_t ny, size_t nu);

/* Analyze input direction sensitivity via singular vectors.
 * L4: The right singular vector v_n (for sigma_min) gives the
 * input direction with smallest gain. The left singular vector u_n
 * gives the corresponding output direction.
 *
 * For ill-conditioned processes, the v_n direction indicates
 * which combination of MVs has negligible effect -- a candidate
 * for MV removal or constraint.
 *
 * Complexity: O(ny*nu) after SVD. Returns the minimum gain. */
double mpc_sensitivity_min_gain_direction(const mpc_svd_t *svd,
                                           double *v_min, double *u_min);

/* Comprehensive sensitivity analysis of an MPC model.
 * L6: Runs all diagnostics (condition number, RGA, collinearity,
 * stiffness, gain direction) and populates the diagnostic report.
 *
 * This is the main entry point for offline engineering analysis
 * before deploying MPC to an ill-conditioned process.
 *
 * Returns 0 on success, -1 on failure. */
int mpc_sensitivity_analyze(const mpc_illcond_model_t *model,
                             mpc_illcond_diagnostic_t *diag);

/* Determine the primary root cause of ill-conditioning.
 * L2/L6: Uses the diagnostic metrics to classify the dominant
 * cause, enabling targeted remediation (rescaling, MV removal,
 * regularization, etc.). */
mpc_illcond_rootcause_t mpc_sensitivity_rootcause(
    const mpc_illcond_diagnostic_t *diag);

/* Generate operator-readable sensitivity report string.
 * Returns length of generated string (not including null). */
int mpc_sensitivity_report(const mpc_illcond_diagnostic_t *diag,
                            char *buffer, size_t bufsize);

/* Numerical rank estimation via SVD thresholding.
 * L4: Numerical rank = count of sigma_i > epsilon * sigma_1.
 * This is more robust than structural rank for ill-conditioned matrices.
 * Reference: Golub & Van Loan (2013), Section 5.5. */
size_t mpc_sensitivity_numerical_rank(const double *S, size_t n, double eps);

/* Compute the nearest rank-deficient matrix distance.
 * L4 Theorem (Eckart-Young-Mirsky):
 *   min_{rank(B)=k} ||A - B||_2 = sigma_{k+1}
 *
 * This gives the "distance to singularity" -- how much perturbation
 * is needed for the matrix to lose rank.
 * Returns sigma_min (the 2-norm distance to rank deficiency). */
double mpc_sensitivity_rank_distance(const mpc_svd_t *svd);

#endif /* MPC_ILLCOND_SENSITIVITY_H */
