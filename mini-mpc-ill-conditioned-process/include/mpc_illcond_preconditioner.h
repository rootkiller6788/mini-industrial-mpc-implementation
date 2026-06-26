#ifndef MPC_ILLCOND_PRECONDITIONER_H
#define MPC_ILLCOND_PRECONDITIONER_H

#include "mpc_illcond_defs.h"

/* Jacobi preconditioner: M = diag(A), M^{-1} = 1/diag(A)
 * L5: Simplest preconditioner. Effective when A is diagonally dominant.
 * Complexity: O(n) to build, O(n) per application.
 * Reference: Saad (2003), Section 10.1. */
int mpc_precond_jacobi_build(const mpc_matrix_t *A, mpc_preconditioner_t *prec);
void mpc_precond_jacobi_apply(const mpc_preconditioner_t *prec,
                               const double *r, double *z);

/* SSOR preconditioner: M = (D+wL)D^{-1}(D+wU) / (w(2-w))
 * L5: Symmetric Successive Over-Relaxation. Better than Jacobi for
 * matrices with significant off-diagonal structure.
 * Optimal omega is typically between 1.0 and 1.5.
 * Complexity: O(nnz) to build, O(nnz) per application (forward+back solve).
 * Reference: Axelsson (1994), "Iterative Solution Methods". */
int mpc_precond_ssor_build(const mpc_matrix_t *A, double omega,
                            mpc_preconditioner_t *prec);
void mpc_precond_ssor_apply(const mpc_preconditioner_t *prec,
                             const double *r, double *z);

/* ILU(0) preconditioner: incomplete LU with zero fill-in.
 * L5: The most widely used general-purpose preconditioner.
 * Preserves the sparsity pattern of A in L and U factors.
 * For SPD matrices, IC(0) (incomplete Cholesky) is preferred.
 *
 * ILU(0) factorization:
 *   for i = 1..n:
 *     for k = 1..i-1 where A(i,k) != 0:
 *       A(i,k) /= A(k,k)
 *       for j = k+1..n where A(i,j) != 0:
 *         A(i,j) -= A(i,k) * A(k,j)
 *
 * Complexity: O(nnz * avg_fill) where avg_fill << n for sparse matrices.
 * Reference: Saad (2003), Section 10.3. */
int mpc_precond_ilu0_build(const mpc_matrix_t *A, mpc_preconditioner_t *prec);
void mpc_precond_ilu0_apply(const mpc_preconditioner_t *prec,
                              const double *r, double *z);

/* Polynomial preconditioner: M^{-1} = p(A) where p is a polynomial
 * approximating x^{-1} on the eigenvalue interval [lambda_min, lambda_max].
 *
 * L5: Uses Chebyshev polynomials shifted to [a,b] = [lambda_min, lambda_max].
 * The Chebyshev polynomial of degree d minimizes the maximum error on [a,b]:
 *   T_d(x) = cos(d * arccos((2x - a - b)/(b - a)))
 *
 * For SPD matrices, Gershgorin circles provide the eigenvalue bounds.
 * Complexity: O(d * nnz) where d = poly_degree.
 * Reference: Saad (2003), Section 12.3. */
int mpc_precond_polynomial_build(const mpc_matrix_t *A, int degree,
                                  mpc_preconditioner_t *prec);
void mpc_precond_polynomial_apply(const mpc_preconditioner_t *prec,
                                   const double *r, double *z);

/* Block Jacobi preconditioner for structured MIMO systems.
 * L5: Partition variables by input channel (nu blocks), build diagonal
 * blocks of H, and apply the inverse of each block independently.
 * This exploits the natural block structure of MPC Hessians.
 *
 * For MPC with control horizon M and nu inputs:
 *   H has nu*M x nu*M structure with M x M blocks per input pair.
 * Block Jacobi inverts only the nu diagonal blocks (each M x M).
 *
 * Complexity: O(nu * M^3) to build, O(nu * M^2) per application.
 * Reference: Rawlings, Mayne, Diehl (2017), Ch. 8. */
int mpc_precond_block_jacobi_build(const mpc_matrix_t *H, size_t nu, size_t M,
                                    mpc_preconditioner_t *prec);
void mpc_precond_block_jacobi_apply(const mpc_preconditioner_t *prec,
                                     const double *r, double *z);

/* Additive Schwarz preconditioner with overlap.
 * L5/L8: Domain decomposition preconditioner. Partitions the index set
 * into overlapping subdomains, solves each independently, and sums.
 * For MPC, subdomains correspond to groups of time steps.
 *
 * The additive Schwarz preconditioner:
 *   M^{-1} = sum_{i=1}^{p} R_i^T (R_i A R_i^T)^{-1} R_i
 * where R_i is the restriction operator to subdomain i.
 *
 * Complexity: O(p * (n/p)^3) to build.
 * Reference: Smith, Bjorstad, Gropp (1996), "Domain Decomposition". */
int mpc_precond_schwarz_build(const mpc_matrix_t *A, int num_domains,
                               int overlap, mpc_preconditioner_t *prec);
void mpc_precond_schwarz_apply(const mpc_preconditioner_t *prec,
                                const double *r, double *z);

/* Dispatch: build preconditioner based on type configuration. */
int mpc_precond_build(const mpc_matrix_t *A, mpc_preconditioner_t *prec);

/* Dispatch: apply preconditioner z = M^{-1} * r. */
void mpc_precond_apply(const mpc_preconditioner_t *prec,
                        const double *r, double *z);

/* Free preconditioner resources. */
void mpc_precond_free(mpc_preconditioner_t *prec);

/* Estimate eigenvalue bounds of SPD matrix via power iteration.
 * Returns (lambda_min, lambda_max) estimates used for polynomial
 * and Chebyshev preconditioner construction.
 * Complexity: O(k * n^2) for k iterations of power method. */
void mpc_precond_eig_bounds(const mpc_matrix_t *A, int k,
                             double *lambda_min, double *lambda_max);

/* Preconditioned Conjugate Gradient (PCG) solver.
 * L5: Solves Ax = b using CG accelerated by a preconditioner M.
 * For SPD systems, PCG converges in O(sqrt(kappa(M^{-1}A))) iterations
 * vs O(sqrt(kappa(A))) for unpreconditioned CG.
 *
 * Algorithm (Saad 2003, Alg 9.1):
 *   r0 = b - A*x0, z0 = M^{-1}*r0, p0 = z0
 *   for k = 0,1,...:
 *     alpha = (r_k^T z_k) / (p_k^T A p_k)
 *     x_{k+1} = x_k + alpha * p_k
 *     r_{k+1} = r_k - alpha * A * p_k
 *     z_{k+1} = M^{-1} * r_{k+1}
 *     beta = (r_{k+1}^T z_{k+1}) / (r_k^T z_k)
 *     p_{k+1} = z_{k+1} + beta * p_k
 *
 * Returns number of iterations, or -1 if exceeded max_iter. */
int mpc_pcg_solve(const mpc_matrix_t *A, const double *b, double *x,
                  const mpc_preconditioner_t *prec,
                  int max_iter, double tol);

#endif /* MPC_ILLCOND_PRECONDITIONER_H */
