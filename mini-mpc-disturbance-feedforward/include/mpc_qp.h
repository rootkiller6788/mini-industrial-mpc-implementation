/**
 * @file mpc_qp.h
 * @brief Quadratic Programming Solver for MPC
 *
 * Implements active-set and interior-point QP solvers specialized
 * for the structure of MPC optimization problems.
 *
 * The MPC optimization at each time step reduces to:
 *   min  0.5 * DeltaU^T * H * DeltaU + f^T * DeltaU
 *   s.t. A_ineq * DeltaU <= b_ineq
 *
 * where H = Gamma^T * Q_bar * Gamma + R_bar (Hessian)
 *       f = Gamma^T * Q_bar * (Phi * x[k] - Y_ref)  (gradient)
 *
 * Reference: Nocedal and Wright, "Numerical Optimization" (2006)
 * Reference: Bartlett, Wachter, Biegler, "Active Set vs. Interior Point
 *            for MPC", Automatica (2002)
 *
 * @knowledge L1: QP standard form definition
 * @knowledge L2: MPC cost function structure
 * @knowledge L4: KKT optimality conditions
 * @knowledge L5: Active-set QP algorithm
 * @knowledge L5: Interior-point QP algorithm
 */

#ifndef MPC_QP_H
#define MPC_QP_H

#include "mpc_common.h"
#include "mpc_model.h"

/* Solver type enumeration */
typedef enum {
    MPC_QP_ACTIVE_SET     = 0,
    MPC_QP_INTERIOR_POINT = 1,
    MPC_QP_GRADIENT_PROJ  = 2
} mpc_qp_solver_type_t;

/* Working set for active-set method */
typedef struct {
    int     active_set[MPC_MAX_NC * (MPC_MAX_NX + MPC_MAX_NU)];
    int     n_active;
    int     max_active;
} mpc_working_set_t;

/* Interior-point solver parameters */
typedef struct {
    double  barrier_param;     /* Barrier parameter mu */
    double  barrier_decay;     /* Decay factor for mu (typically 0.1-0.5) */
    double  backtrack_alpha;   /* Backtracking line search parameter */
    double  armijo_c1;         /* Armijo condition constant */
    int     max_backtrack;     /* Maximum backtracking steps */
} mpc_ip_params_t;

/* ==========================================================================
 * Core QP Solver Functions
 * ========================================================================== */

/* Initialize QP problem with zeros */
void mpc_qp_init(mpc_qp_t *qp);

/* Set QP solver tolerances */
void mpc_qp_set_tolerances(mpc_qp_t *qp, double tol_opt, double tol_feas, int max_iter);

/* Compute Hessian H = Gamma^T * Q_bar * Gamma + R_bar for MPC */
int mpc_qp_build_hessian(const mpc_prediction_t *pred,
                          const mpc_tuning_t *tuning,
                          mpc_qp_t *qp);

/* Compute gradient f = Gamma^T * Q_bar * (Phi * xa - Y_ref) */
int mpc_qp_build_gradient(const mpc_prediction_t *pred,
                           const mpc_aug_model_t *model,
                           const mpc_tuning_t *tuning,
                           const double xa[MPC_MAX_NX + MPC_MAX_ND],
                           const double y_ref[MPC_MAX_NY],
                           const double d_pred[MPC_MAX_NP * MPC_MAX_ND],
                           mpc_qp_t *qp);

/* Build inequality constraint matrices from MPC constraints */
int mpc_qp_build_constraints(const mpc_prediction_t *pred,
                              const mpc_constraints_t *cons,
                              const double xa[MPC_MAX_NX + MPC_MAX_ND],
                              const double u_prev[MPC_MAX_NU],
                              const mpc_tuning_t *tuning,
                              mpc_qp_t *qp);

/* Solve QP using active-set method (Goldfarb-Idnani dual active set) */
int mpc_qp_solve_active_set(mpc_qp_t *qp, mpc_working_set_t *ws);

/* Solve QP using primal-dual interior-point method (Mehrotra predictor-corrector) */
int mpc_qp_solve_interior_point(mpc_qp_t *qp, const mpc_ip_params_t *params);

/* Solve QP using projected gradient method (for simple bound constraints) */
int mpc_qp_solve_projected_gradient(mpc_qp_t *qp);

/* Validate QP solution against KKT conditions */
int mpc_qp_validate_solution(const mpc_qp_t *qp);

/* ==========================================================================
 * Matrix-Vector Operations for QP Construction
 * ========================================================================== */

/* Compute u = A * v (matrix-vector multiply) */
void mpc_matvec_mul(const double A[], const double v[], double u[],
                     int rows, int cols, int max_cols);

/* Compute C = A^T * B (matrix-matrix multiply with transpose) */
void mpc_matmat_transpose_mul(const double A[], const double B[],
                               double C[], int rows_A, int cols_A, int cols_B,
                               int lda, int ldb, int ldc);

/* Cholesky factorization: A = L * L^T (in-place, lower triangular) */
int mpc_cholesky_decomp(double A[], int n, int ld);

/* Forward/backward substitution for L * L^T * x = b */
void mpc_cholesky_solve(const double L[], const double b[], double x[],
                         int n, int ld);

/* LDL^T factorization for symmetric indefinite matrices */
int mpc_ldlt_decomp(double A[], double D[], int n, int ld);

/* LDL^T solve */
void mpc_ldlt_solve(const double L[], const double D[], const double b[],
                     double x[], int n, int ld);

/* Compute infinity-norm of residual for convergence check */
double mpc_residual_infnorm(const double A[], const double x[],
                             const double b[], int n, int ld);

/* ==========================================================================
 * Hot-start and Condensing
 * ========================================================================== */

/* Condense the QP: eliminate equality constraints to reduce problem size */
int mpc_qp_condense(const mpc_qp_t *qp, mpc_qp_t *condensed_qp);

/* Hot-start: reuse previous solution to accelerate convergence */
void mpc_qp_hotstart(mpc_qp_t *qp, const double z_prev[], int nz);

/* Compute nullspace basis for equality constraints */
int mpc_qp_nullspace_basis(const double A_eq[], int n_eq, int n_var,
                            double Z[], int max_cols);

/* Range-space iteration for equality-constrained QP */
int mpc_qp_range_space_solve(const double H[], const double A_eq[],
                              const double f[], const double b_eq[],
                              double z[], double nu[],
                              int n_var, int n_eq);

#endif /* MPC_QP_H */
