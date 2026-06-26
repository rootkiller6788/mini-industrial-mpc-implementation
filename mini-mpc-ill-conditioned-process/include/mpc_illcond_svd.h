#ifndef MPC_ILLCOND_SVD_H
#define MPC_ILLCOND_SVD_H

#include "mpc_illcond_defs.h"

/* Compute the SVD of a matrix A = U * S * V^T using Golub-Reinsch algorithm.
 * L5: Two-phase algorithm:
 *   Phase 1: Bidiagonalize A via Householder reflections (O(m*n*min(m,n)))
 *   Phase 2: Implicit QR iteration on bidiagonal matrix (O(min(m,n)^2))
 *
 * For large matrices, this is the computational bottleneck.
 * For MPC applications (n < 500), the dense Golub-Reinsch algorithm
 * is preferred over Lanczos-based methods.
 *
 * Reference: Golub & Reinsch (1970); Golub & Van Loan (2013), Alg 8.6.2.
 *
 * Returns 0 on success. Caller must free the result with mpc_svd_free(). */
int mpc_svd_compute(const mpc_matrix_t *A, mpc_svd_t *result);

/* Compute thin SVD: only compute first k singular values/vectors.
 * L5: Uses Lanczos bidiagonalization for efficiency when k << min(m,n).
 * This is the preferred method for large MPC problems where only
 * the dominant singular subspace is needed.
 *
 * Complexity: O(m*n*k) vs O(m*n*min(m,n)) for full SVD.
 * Reference: Baglama & Reichel (2005), "Augmented Implicitly Restarted
 * Lanczos Bidiagonalization Methods". */
int mpc_svd_compute_k(const mpc_matrix_t *A, mpc_svd_t *result, size_t k);

/* Compute condition number from SVD: kappa_2 = sigma_max / sigma_min.
 * L1: Returns the 2-norm condition number.
 * For well-conditioned problems, kappa ~ 1-1000.
 * For ill-conditioned problems, kappa > 1e6.
 * For near-singular, kappa approaches infinity (returns INFINITY). */
double mpc_svd_condition(const mpc_svd_t *svd);

/* Compute numerical rank from singular values.
 * L4: Count sigma_i > eps * sigma_1. Standard eps = sqrt(machine_epsilon).
 *
 * Theorem (Weyl, 1912): |sigma_i(A+E) - sigma_i(A)| <= ||E||_2.
 * This bounds the sensitivity of singular values to perturbations,
 * justifying the threshold-based rank definition. */
size_t mpc_svd_numerical_rank(const mpc_svd_t *svd, double eps);

/* Reconstruct matrix from SVD: A = U * S * V^T.
 * Optionally truncate to first k singular components.
 * Complexity: O(m*n*k). */
int mpc_svd_reconstruct(const mpc_svd_t *svd, mpc_matrix_t *A, size_t k);

/* Compute the pseudo-inverse via SVD: A^+ = V * S^+ * U^T.
 * L5: For singular value sigma_i, S^+_ii = 1/sigma_i if sigma_i > eps, else 0.
 * This is the minimum-norm least-squares solution for rank-deficient systems.
 *
 * For ill-conditioned MPC, the pseudo-inverse via truncated SVD
 * provides a stable solution that ignores near-nullspace directions.
 * Complexity: O(m*n*min(m,n)). */
int mpc_svd_pinv(const mpc_svd_t *svd, mpc_matrix_t *pinv, double eps);

/* Compute the 2-norm of a matrix from its singular values.
 * ||A||_2 = sigma_max. O(1) from pre-computed SVD. */
double mpc_svd_norm_2(const mpc_svd_t *svd);

/* Compute the Frobenius norm from singular values.
 * ||A||_F = sqrt(sum sigma_i^2). O(k) from pre-computed SVD. */
double mpc_svd_norm_frobenius(const mpc_svd_t *svd);

/* Compute the effective nullspace dimension.
 * L4: Count singular values below epsilon. The corresponding
 * right singular vectors span the numerical nullspace.
 * For collinear actuators, the nullspace reveals which input
 * combinations produce zero output response. */
size_t mpc_svd_nullspace_dim(const mpc_svd_t *svd, double eps);

/* Extract dominant input directions (right singular vectors for largest sigma).
 * L6: The first few right singular vectors reveal the most effective
 * input combinations -- useful for control structure design and
 * identifying which MVs have the strongest influence on CVs.
 * Complexity: O(nu * k). */
int mpc_svd_dominant_input_dirs(const mpc_svd_t *svd, mpc_matrix_t *dirs, size_t k);

/* Free all resources allocated for an SVD result. */
void mpc_svd_free(mpc_svd_t *svd);

/* Compute Gershgorin circle bounds on eigenvalues.
 * L4 Theorem (Gershgorin, 1931): Every eigenvalue lambda of A lies in
 * at least one Gershgorin disc:
 *   |lambda - A_ii| <= sum_{j!=i} |A_ij|   (row discs)
 *
 * Returns the interval [min_center-radius, max_center+radius] containing
 * all eigenvalues. Used for quick conditioning estimates without full SVD.
 * Complexity: O(n^2). */
void mpc_svd_gershgorin_bounds(const mpc_matrix_t *A,
                                double *bound_min, double *bound_max);

/* Estimate 2-norm condition number via power iteration.
 * L5: Uses power iteration to estimate sigma_max and inverse power
 * iteration (with shift) to estimate sigma_min, then kappa = sigma_max/sigma_min.
 * Much cheaper than full SVD: O(k * n^2) for k iterations.
 *
 * Reference: Higham (2002), "Accuracy and Stability of Numerical Algorithms",
 * Chapter 15 (Condition Number Estimation). */
double mpc_svd_condition_estimate(const mpc_matrix_t *A, int num_iter);

#endif /* MPC_ILLCOND_SVD_H */
