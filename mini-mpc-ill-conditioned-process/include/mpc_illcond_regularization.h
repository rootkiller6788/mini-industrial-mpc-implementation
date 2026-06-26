#ifndef MPC_ILLCOND_REGULARIZATION_H
#define MPC_ILLCOND_REGULARIZATION_H

#include "mpc_illcond_defs.h"

/**
 * Apply Tikhonov regularization to a matrix: A_reg = A + lambda * I
 *
 * L5 Algorithm: Standard L2 Tikhonov regularization (Phillips, 1962;
 * Tikhonov, 1963). Adds lambda to the diagonal, shifting all eigenvalues
 * by +lambda. This guarantees A_reg is SPD for lambda > 0 if A is PSD.
 *
 * Theorem: If A is PSD with eigenvalues sigma_i >= 0, then A + lambda*I
 * has eigenvalues sigma_i + lambda > 0, and condition number:
 *   kappa(A_reg) = (sigma_max + lambda) / (sigma_min + lambda)
 * which is always less than kappa(A) for lambda > 0.
 *
 * Complexity: O(n) -- diagonal update only.
 */
void mpc_regularize_tikhonov(mpc_matrix_t *A, double lambda);

/**
 * Apply Tikhonov regularization to the MPC Hessian with move suppression.
 *
 * L5 Algorithm: In MPC, the Hessian is H = Theta^T * Q * Theta + R.
 * For ill-conditioned H, we add lambda_delta_u * (M^T * M) where M
 * is the move-difference matrix, penalizing large control moves.
 *
 * This corresponds to the cost term: lambda_delta_u * ||Delta_U||^2
 * which directly suppresses aggressive control action.
 *
 * Complexity: O(n) for diagonal move-suppression structure.
 * Reference: Maciejowski (2002), "Predictive Control with Constraints", Ch. 3.
 */
void mpc_regularize_move_suppression(mpc_matrix_t *H, size_t nu, size_t M,
                                      double lambda_delta_u);

/**
 * Truncated SVD regularization: reconstruct matrix using only k largest singular values.
 *
 * L5 Algorithm: Given SVD A = U*S*V^T, truncated SVD regularization
 * reconstructs A_k = U_k * S_k * V_k^T where k is the number of singular
 * values above the threshold: sigma_i / sigma_1 > threshold.
 *
 * This is the optimal rank-k approximation in the 2-norm (Eckart-Young-Mirsky theorem):
 *   ||A - A_k||_2 = sigma_{k+1}
 *
 * For ill-conditioned MPC, dropping small singular values removes near-nullspace
 * directions that amplify noise without contributing to control authority.
 *
 * Complexity: O(m*n) for reconstruction from existing SVD.
 * Reference: Hansen (1998), "Rank-Deficient and Discrete Ill-Posed Problems".
 */
int mpc_regularize_truncated_svd(const mpc_svd_t *svd, mpc_matrix_t *A_reg,
                                  double threshold);

/**
 * Levenberg-Marquardt adaptive regularization.
 *
 * L5 Algorithm: Adaptive lambda search combining Gauss-Newton (lambda small)
 * and gradient descent (lambda large). Starting from lambda_0, if the step
 * reduces the cost, decrease lambda (closer to Newton). If the step increases
 * the cost, increase lambda (more gradient descent, more robust).
 *
 * The update rule:
 *   lambda_new = lambda * decay         if cost decreased
 *   lambda_new = lambda / decay         if cost increased
 *
 * This is the trust-region interpretation of LM (More, 1978).
 *
 * Complexity: O(iterations * n^3) for repeated Cholesky.
 * Reference: Marquardt (1963), Levenberg (1944).
 */
double mpc_regularize_levenberg_marquardt(
    const mpc_matrix_t *H, const double *f,
    mpc_matrix_t *H_reg, double lambda_init,
    int max_iter, double decay);

/**
 * Elastic net regularization: (1-alpha)*||Ax-b||^2 + alpha*||Ax-b||_1 + lambda*||x||^2
 *
 * L5 Algorithm: Combined L1+L2 penalty. The L1 term promotes sparsity
 * (zeroing small elements), while L2 ensures strict convexity and stability.
 * For MPC, this encourages sparse control moves (only move when necessary).
 *
 * alpha = 0: pure ridge (L2)
 * alpha = 1: lasso + ridge (naive elastic net)
 *
 * Complexity: Solved via coordinate descent with soft-thresholding.
 * Reference: Zou & Hastie (2005), "Regularization and Variable Selection
 * via the Elastic Net".
 */
void mpc_regularize_elastic_net(mpc_matrix_t *A_reg, double lambda,
                                 double alpha, int max_iter, double tol);

/**
 * Compute recommended lambda using the discrepancy principle.
 *
 * L4 Theorem (Morozov, 1966): For the linear system Ax = b with noise
 * level delta (||b_true - b_measured|| <= delta), the optimal lambda
 * satisfies ||A*x_lambda - b|| = delta.
 *
 * For MPC, we estimate noise from prediction error and set:
 *   lambda = sigma_n^2 / (sigma_1^2 + sigma_n^2)
 * where sigma_1, sigma_n are the largest and smallest singular values.
 *
 * This gives a balanced regularization that scales with problem conditioning.
 */
double mpc_regularize_recommend_lambda(const mpc_illcond_diagnostic_t *diag);

/**
 * Apply regularization to the MPC QP Hessian and optionally constraints.
 *
 * Master function that dispatches to the appropriate regularization method
 * based on the configuration. Also updates the QP diagnostic report.
 *
 * Returns 0 on success, -1 if regularization method fails.
 */
int mpc_regularize_qp(mpc_illcond_qp_t *qp,
                       const mpc_regularization_t *config);

#endif /* MPC_ILLCOND_REGULARIZATION_H */
