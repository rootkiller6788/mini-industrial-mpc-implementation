/**
 * mpc_qp_solver.h — Quadratic Programming Solvers for MPC
 *
 * Domain: Numerical Optimization for Model Predictive Control
 *
 * MPC reduces to solving a convex QP at each sampling instant.
 * For integrating process level control, the QP has special structure:
 *   1. H = G^T*G + λ*I is SPD (for λ > 0)
 *   2. Hessian depends on G which changes if move blocking is used
 *   3. Constraints may be infeasible (need soft constraint relaxation)
 *
 * This module provides multiple QP solvers with different speed/robustness
 * tradeoffs, suitable for different industrial deployment contexts.
 *
 * Reference: Nocedal & Wright (2006) "Numerical Optimization" §16
 *            Goldfarb & Idnani (1983) "A numerically stable dual method for
 *              solving strictly convex quadratic programs"
 *            Hildreth (1957) "A quadratic programming procedure"
 *
 * Knowledge Coverage:
 *   L5 - Algorithms: Active-set QP, Hildreth QP, interior point
 *   L3 - Eng. Structures: QP formulation for MPC
 *
 * MIT 6.302 · CMU 24-677 · Stanford ENGR205 · Purdue ME 575
 */

#ifndef MPC_QP_SOLVER_H
#define MPC_QP_SOLVER_H

#include "mpc_level_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ─── QP Solver API ───────────────────────────────────────────────────── */

/**
 * qp_solve — Dispatcher: solve QP using selected method
 *
 * Routes to the appropriate solver based on type.
 * For MPC level control, Hildreth is used for simple bound constraints,
 * Active-set for general inequality constraints, and Interior-point for
 * large problems (N_c > 10).
 *
 * @param solution   Output solution (delta_u_plan, status, iterations)
 * @param qp         QP problem specification
 * @param solver     Solver type to use
 * Returns QP status code.
 */
qp_status_t qp_solve(mpc_solution_t *solution, const qp_problem_t *qp,
                      mpc_solver_type_t solver);

/* ─── Active Set Method (Goldfarb-Idnani Dual) ────────────────────────── */

/**
 * qp_active_set_goldfarb_idnani — Dual active set QP solver
 *
 * Solves:
 *   min  0.5 * x^T * H * x + c^T * x
 *   s.t. A * x ≤ b
 *
 * Algorithm (Goldfarb & Idnani, 1983):
 *   1. Start with unconstrained minimum x* = -H⁻¹*c
 *   2. If all constraints satisfied → done (optimal)
 *   3. Add most violated constraint to working set W
 *   4. Solve equality-constrained QP with W
 *   5. If a constraint in W becomes non-binding, drop it
 *   6. Repeat from step 2
 *
 * Key advantage for MPC: warm-start from previous solution.
 * Previous Δu* is feasible for the new shifted problem after
 * Δu(0) is applied (with slight modification for new last move).
 *
 * Complexity: O((n+m) * m²) per iteration, where m = active constraints.
 * Finite termination guaranteed for strictly convex QP (H ≻ 0).
 *
 * Numerical stability innovations (from Goldfarb-Idnani paper):
 *   - Cholesky factorization R^T*R of H in working set subspace
 *   - Givens rotations for rank-1 updates to R (adding constraint)
 *   - Stable constraint deletion via partial refactorization
 *
 * Reference: Goldfarb & Idnani (1983) Mathematical Programming 27:1-33
 *            Schmid & Biegler (1994) "QP methods for MPC"
 *
 * @param x       Input/output: initial guess / solution (n_vars)
 * @param qp      QP problem
 * @param max_iter Maximum iterations (typ. 5*n_vars)
 * @param status  Output: solver status
 * Returns objective value at solution.
 */
double qp_active_set_goldfarb_idnani(double *x, const qp_problem_t *qp,
                                      int max_iter, qp_status_t *status);

/* ─── Hildreth's Method (Simple Bounds Only) ──────────────────────────── */

/**
 * qp_hildreth — Hildreth's QP for box constraints
 *
 * Special case: only simple bounds: l ≤ x ≤ u
 *
 * Hildreth's algorithm (1957) solves the dual problem:
 *   max_{λ≥0, μ≥0}  -0.5*(c + λ - μ)^T*H⁻¹*(c + λ - μ)
 *                    - λ^T*l + μ^T*u + 0.5*c^T*H⁻¹*c
 *
 * by coordinate descent on dual variables λ_i, μ_i.
 *
 * Algorithm:
 *   For each variable i:
 *     x_i = -(c_i + Σ_{j≠i} H_{ij}*x_j) / H_{ii}   (Gauss-Seidel step)
 *     x_i = clamp(x_i, l_i, u_i)                      (projection)
 *
 * This is essentially projected Gauss-Seidel on the KKT system.
 * Guaranteed convergence for SPD H (Cryer 1971).
 *
 * Advantage: Extremely simple, no matrix factorization needed.
 * Disadvantage: Slow convergence for ill-conditioned H (many iterations).
 *
 * For MPC with move suppression λ: H diagonal dominance improves with
 * larger λ, making Hildreth more efficient.
 *
 * Reference: Hildreth (1957) Naval Research Logistics Quarterly
 *            Luenberger (1973) "Linear and Nonlinear Programming"
 *
 * @param x         Input/output: initial guess / solution (n_vars)
 * @param qp        QP problem (only x_lower, x_upper used from constraints)
 * @param max_iter  Maximum iterations
 * @param tol       Convergence tolerance on Δx norm
 * @param status    Output solver status
 * Returns objective value.
 */
double qp_hildreth(double *x, const qp_problem_t *qp, int max_iter,
                    double tol, qp_status_t *status);

/* ─── Interior Point Method ───────────────────────────────────────────── */

/**
 * qp_interior_point — Mehrotra predictor-corrector interior point for QP
 *
 * For large-scale MPC (N_c > 10), interior point methods are superior.
 *
 * Standard form:
 *   min  0.5*x^T*H*x + c^T*x
 *   s.t. A*x ≤ b
 *        x ≥ 0
 *
 * KKT conditions (with slacks s ≥ 0):
 *   H*x + c + A^T*λ - μ = 0
 *   A*x + s - b = 0
 *   x_i * μ_i = σ*τ           [complementarity, central path]
 *
 * Mehrotra (1992) predictor-corrector:
 *   1. Predictor step: solve for affine direction (σ=0)
 *   2. Compute centering parameter σ from predictor step length
 *   3. Corrector step: solve with nonlinear correction + centering
 *
 * The Mehrotra algorithm achieves quadratic convergence near the central
 * path and is the basis of virtually all modern interior point codes (CPLEX,
 * Gurobi, MOSEK, IPOPT for the QP subproblem in SQP).
 *
 * Reference: Mehrotra (1992) SIAM J. Optimization 2(4):575-601
 *            Wright (1997) "Primal-Dual Interior Point Methods"
 *
 * @param x       Input/output: initial point / solution (n_vars)
 * @param qp      QP problem
 * @param max_iter Max iterations
 * @param status  Output solver status
 * Returns objective value.
 */
double qp_interior_point(double *x, const qp_problem_t *qp,
                          int max_iter, qp_status_t *status);

/* ─── QP Construction Helpers ─────────────────────────────────────────── */

/**
 * qp_init — Initialize QP problem structure
 *
 * Sets all matrices/vectors to zero, dimensions to 0.
 *
 * @param qp  QP problem to initialize
 */
void qp_init(qp_problem_t *qp);

/**
 * qp_check_feasibility — Quick feasibility check
 *
 * Checks if there exists any x satisfying all constraints.
 * Returns 1 if feasible, 0 if infeasible.
 * Uses LP-based phase I (simplified simplex for small problems).
 *
 * For MPC level control, common infeasibility causes:
 *   - Level already outside bounds (can't recover within N_p steps)
 *   - Valve rate limit too restrictive for required move
 *   - Conflicting output and input constraints
 *
 * @param qp    QP problem
 * @param tol   Feasibility tolerance
 * Returns 1 if feasible, 0 otherwise.
 */
int qp_check_feasibility(const qp_problem_t *qp, double tol);

/**
 * qp_objective_value — Compute J = 0.5*x^T*H*x + c^T*x
 *
 * Used to verify solver outputs and monitor closed-loop cost.
 *
 * @param x      Solution vector (n_vars)
 * @param qp     QP problem (H, c)
 * @returns Objective value.
 */
double qp_objective_value(const double *x, const qp_problem_t *qp);

/**
 * qp_soften_output_constraints — Convert hard output constraints to soft
 *
 * Replaces y_min ≤ G*Δu + f ≤ y_max with:
 *   y_min - ε ≤ G*Δu + f ≤ y_max + ε
 * and adds penalty term ρ*ε² to objective.
 *
 * This guarantees feasibility at the cost of allowing minor violations.
 * Essential for integrating processes where ramping disturbances can
 * temporarily push level beyond bounds.
 *
 * Algorithm (Scokaert & Rawlings 1999):
 *   1. Add slack variable ε_i for each output constraint
 *   2. Augment H with ρ*I block (n_slack × n_slack)
 *   3. Augment A_ineq with +I or -I for slacks
 *   4. Weight ρ balances constraint violation vs. tracking
 *
 * @param qp   QP problem (modified in place)
 * @param rho  Penalty weight ρ > 0 (larger = less violation)
 * @returns Number of softened constraints.
 */
int qp_soften_output_constraints(qp_problem_t *qp, double rho);

/* ─── Numerical Utilities ─────────────────────────────────────────────── */

/**
 * qp_cholesky_decomp — In-place Cholesky decomposition A = L*L^T
 *
 * For SPD matrix A (column-major). Lower triangular L stored in A.
 * Returns 0 on success, -1 if A is not SPD (negative pivot).
 *
 * Algorithm: standard LDL^T without pivoting.
 *   For column j: A[j][j] = sqrt(Ajj - Σ_{k<j} A[j][k]²)
 *   For row i > j: A[i][j] = (A[i][j] - Σ_{k<j} A[i][k]*A[j][k]) / A[j][j]
 *
 * Reference: Golub & Van Loan (2013) "Matrix Computations" §4.2
 *
 * @param A   n×n SPD matrix (col-major), overwritten with L
 * @param n   Dimension
 * Returns 0 on success.
 */
int qp_cholesky_decomp(double *A, int n);

/**
 * qp_cholesky_solve — Solve L*L^T * x = b using precomputed L
 *
 * Forward substitution: L*y = b
 * Back substitution:  L^T*x = y
 *
 * @param L   Lower-triangular Cholesky factor (n×n col-major)
 * @param b   RHS, overwritten with solution
 * @param n   Dimension
 */
void qp_cholesky_solve(const double *L, double *b, int n);

/**
 * qp_forward_substitution — Solve L*x = b (L lower-triangular)
 *
 * x_i = (b_i - Σ_{j<i} L_{ij}*x_j) / L_{ii}
 */
void qp_forward_substitution(const double *L, double *x,
                              const double *b, int n);

/**
 * qp_back_substitution — Solve U*x = b (U upper-triangular)
 *
 * x_i = (b_i - Σ_{j>i} U_{ij}*x_j) / U_{ii}
 */
void qp_back_substitution(const double *U, double *x,
                           const double *b, int n);

#ifdef __cplusplus
}
#endif

#endif /* MPC_QP_SOLVER_H */
