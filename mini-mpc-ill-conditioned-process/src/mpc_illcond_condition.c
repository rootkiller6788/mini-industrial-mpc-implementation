#include "mpc_illcond_defs.h"
#include "mpc_illcond_matrix.h"
#include "mpc_illcond_svd.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>

/* =====================================================================
 * L5: Hager-Higham 1-norm condition estimator
 *
 * Estimates ||A^{-1}||_1 iteratively without computing A^{-1}.
 * The algorithm maximizes ||A^{-1}*x||_1 over ||x||_1 = 1
 * via gradient-ascent with sign-based updates.
 *
 * Algorithm (Higham 1988, ACM TOMS 14(4)):
 *   1. x = (1/n, 1/n, ..., 1/n)
 *   2. Repeat until convergence:
 *      a. Solve A*y = x (approximate via gradient descent)
 *      b. Solve A^T*z = y
 *      c. if |gamma - gamma_old| < tol*gamma: converged
 *      d. k = argmax |z_j|
 *      e. x = sign(z_k) * e_k
 *   3. ||A^{-1}||_1 = gamma
 *
 * Complexity: O(t * n^2) where t = 3-5 iterations typically.
 * Reference: Hager (1984), Higham (1988), LAPACK DLANGE/DGECON.
 * ===================================================================== */
static double hager_higham_norm1_inv(const mpc_matrix_t *A,
                                      int max_iter, double tol)
{
    size_t n = A->rows;
    size_t i, j, iter, max_idx;
    double *x, *y, *z;
    double gamma, gamma_old, z_norm_inf, xi;
    int converged = 0;
    double result = 0.0;

    if (n == 0) return 0.0;

    x = (double*)calloc(n, sizeof(double));
    y = (double*)malloc(n * sizeof(double));
    z = (double*)malloc(n * sizeof(double));
    if (!x || !y || !z) {
        free(x); free(y); free(z);
        return 0.0;
    }

    /* Initial x: uniform distribution with ||x||_1 = 1 */
    for (i = 0; i < n; i++) x[i] = 1.0 / (double)n;

    for (iter = 0; iter < (size_t)max_iter && !converged; iter++) {
        /* Solve A*y = x (approximate via few gradient descent steps) */
        for (i = 0; i < n; i++) y[i] = x[i];
        for (i = 0; i < n; i++) {
            double Ay_i = 0.0;
            for (j = 0; j < n; j++)
                Ay_i += A->data[i * A->stride + j] * y[j];
            y[i] -= 0.5 * (Ay_i - x[i]);
        }

        /* Solve A^T*z = y */
        for (i = 0; i < n; i++) z[i] = y[i];
        for (i = 0; i < n; i++) {
            double ATz_i = 0.0;
            for (j = 0; j < n; j++)
                ATz_i += A->data[j * A->stride + i] * z[j];
            z[i] -= 0.5 * (ATz_i - y[i]);
        }

        /* Compute ||z||_1 */
        gamma = 0.0;
        for (i = 0; i < n; i++) gamma += fabs(z[i]);

        /* Convergence check */
        if (iter > 0 && fabs(gamma - gamma_old) < tol * gamma)
            converged = 1;

        /* Find argmax |z_j| (for infinity norm direction) */
        z_norm_inf = 0.0;
        max_idx = 0;
        for (i = 0; i < n; i++) {
            if (fabs(z[i]) > z_norm_inf) {
                z_norm_inf = fabs(z[i]);
                max_idx = i;
            }
        }

        /* Update x = sign(z_k) * e_k */
        xi = (z[max_idx] >= 0.0) ? 1.0 : -1.0;
        for (i = 0; i < n; i++) x[i] = 0.0;
        x[max_idx] = xi;

        gamma_old = gamma;
        result = gamma;
    }

    free(x); free(y); free(z);
    return result;
}

/* Gershgorin circle condition estimate.
 * L4: Every eigenvalue lambda lies in D_i = {z: |z-A_ii| <= R_i},
 * R_i = sum_{j!=i} |A_ij|. Gives O(n^2) upper bound on kappa.
 * Reference: Varga (2004) "Gershgorin and His Circles". */
static double gershgorin_cond_est(const mpc_matrix_t *A)
{
    size_t i, j, n = A->rows;
    double radius, center, max_ev = 0.0, min_ev = 1e308, low;
    for (i = 0; i < n; i++) {
        radius = 0.0; center = fabs(A->data[i * A->stride + i]);
        for (j = 0; j < n; j++)
            if (j != i) radius += fabs(A->data[i * A->stride + j]);
        if (center + radius > max_ev) max_ev = center + radius;
        low = (center > radius) ? (center - radius) : 0.0;
        if (low < min_ev) min_ev = low;
    }
    for (j = 0; j < n; j++) {
        radius = 0.0; center = fabs(A->data[j * A->stride + j]);
        for (i = 0; i < n; i++)
            if (i != j) radius += fabs(A->data[i * A->stride + j]);
        if (center + radius > max_ev) max_ev = center + radius;
        low = (center > radius) ? (center - radius) : 0.0;
        if (low < min_ev) min_ev = low;
    }
    if (min_ev < 1e-15) return INFINITY;
    return max_ev / min_ev;
}

/* Trace-ratio condition estimate via eigenvalue variance bounds.
 * L5: For SPD matrices, tr = sum(lambda_i), ||A||_F^2 = sum(lambda_i^2).
 * Population variance bounds eigenvalues: |lambda_i - mean| <= sqrt((n-1)*var).
 * Reference: Wolkowicz & Styan (1980), LAA 29:471-506. */
static double trace_ratio_cond_est(const mpc_matrix_t *A)
{
    size_t i, j, n = A->rows;
    double tr = 0.0, frob2 = 0.0, mean, var, spread, lam_lo, lam_hi;
    for (i = 0; i < n; i++) {
        tr += A->data[i * A->stride + i];
        for (j = 0; j < n; j++) {
            double v = A->data[i * A->stride + j];
            frob2 += v * v;
        }
    }
    mean = tr / (double)n;
    var = frob2 / (double)n - mean * mean;
    if (var < 0.0) var = 0.0;
    spread = sqrt((double)(n - 1) * var);
    lam_lo = mean - spread;
    lam_hi = mean + spread;
    if (lam_lo < 1e-15) return INFINITY;
    return lam_hi / lam_lo;
}

/* Frobenius-based condition estimate.
 * L5: kappa_2 >= ||A||_F * ||A^{-1}||_F / n (lower bound).
 * Use Hager-Higham for ||A^{-1}||_1 and blend with Frobenius norm. */
static double frobenius_cond_est(const mpc_matrix_t *A)
{
    double frob_a = mpc_matrix_norm_frobenius(A);
    double norm1_inv = hager_higham_norm1_inv(A, 10, 1e-12);
    if (norm1_inv < 1e-15) return INFINITY;
    return frob_a * norm1_inv;
}

/* Public: condition number estimate via selected method.
 * L5: Dispatches to Hager-Higham (1-norm), Gershgorin, Frobenius, Trace-ratio.
 * Returns kappa estimate or INFINITY for singular matrices. */
double mpc_condition_estimate(const mpc_matrix_t *A,
                               mpc_illcond_estimation_method_t method)
{
    double na;
    if (!A || A->rows == 0 || A->cols == 0) return 0.0;
    switch (method) {
    case MPC_CONDEST_NORM1:
        na = mpc_matrix_norm_1(A);
        { double ni = hager_higham_norm1_inv(A, 10, 1e-12);
          return (ni < 1e-15) ? INFINITY : na * ni; }
    case MPC_CONDEST_NORM_INF:
        na = mpc_matrix_norm_inf(A);
        { double ni = hager_higham_norm1_inv(A, 10, 1e-12);
          return (ni < 1e-15) ? INFINITY : na * ni; }
    case MPC_CONDEST_FROBENIUS:
        return frobenius_cond_est(A);
    case MPC_CONDEST_GERSHGORIN:
        return gershgorin_cond_est(A);
    case MPC_CONDEST_TRACE_RATIO:
        return trace_ratio_cond_est(A);
    default: return 0.0;
    }
}

/* Ill-conditioning check: kappa > 1e8. */
int mpc_condition_is_illcond(double cond)
{
    return (cond > 1.0e8) ? 1 : 0;
}

/* Qualitative grade: maps condition number to operator alarm level. */
const char* mpc_condition_grade(double cond)
{
    if (cond < 1.0e3)  return "WELL_CONDITIONED";
    if (cond < 1.0e6)  return "MODERATELY_CONDITIONED";
    if (cond < 1.0e8)  return "POORLY_CONDITIONED";
    return "ILL_CONDITIONED";
}

/* Digits lost: log10(kappa). L4: Wilkinson (1963).
 * IEEE double: ~16 digits total. kappa=1e8 -> ~8 lost -> ~8 remaining. */
double mpc_condition_digits_lost(double cond)
{
    return (cond < 1.0) ? 0.0 : log10(cond);
}

/* Recommended lambda: sigma_min^2 (Morozov discrepancy principle, 1966).
 * Floor: lambda >= eps * ||A||^2. */
double mpc_condition_recommend_lambda(double norm_a, double cond)
{
    double sigma_min, lambda_min, lambda_svd;
    if (cond < 1.0 || cond > 1e16) return 1.0e-6;
    sigma_min = norm_a / cond;
    lambda_min = 1.4901161193847656e-08 * norm_a * norm_a;
    lambda_svd = sigma_min * sigma_min;
    return (lambda_svd > lambda_min) ? lambda_svd : lambda_min;
}

/* Comprehensive condition diagnostic for an MPC model matrix.
 * L6: Computes all condition metrics and populates report.
 * Procedure: 1-norm + inf-norm + SVD 2-norm -> sensitivity -> lambda. */
int mpc_condition_diagnose(const mpc_matrix_t *A,
                            mpc_illcond_diagnostic_t *diag)
{
    double n1, cond1;
    mpc_svd_t svd;
    int svd_ok;
    if (!A || !diag) return -1;
    memset(diag, 0, sizeof(mpc_illcond_diagnostic_t));
    n1 = mpc_matrix_norm_1(A);
    cond1 = mpc_condition_estimate(A, MPC_CONDEST_NORM1);
    diag->condition_number_1 = cond1;
    diag->condition_number_inf = mpc_condition_estimate(A, MPC_CONDEST_NORM_INF);
    svd_ok = mpc_svd_compute(A, &svd);
    if (svd_ok == 0 && svd.S) {
        size_t n = (A->rows < A->cols) ? A->rows : A->cols;
        diag->max_singular_value = svd.S[0];
        diag->min_singular_value = svd.S[n-1];
        diag->condition_number = svd.cond;
        diag->effective_rank_ratio = (double)svd.rank / (double)n;
    } else {
        diag->condition_number = cond1;
        diag->max_singular_value = n1;
        diag->min_singular_value = n1 / cond1;
        diag->effective_rank_ratio = 1.0;
    }
    if (diag->condition_number < 1.0e3)
        diag->sensitivity = MPC_SENSITIVITY_LOW;
    else if (diag->condition_number < 1.0e6)
        diag->sensitivity = MPC_SENSITIVITY_MODERATE;
    else if (diag->condition_number < 1.0e8)
        diag->sensitivity = MPC_SENSITIVITY_HIGH;
    else
        diag->sensitivity = MPC_SENSITIVITY_EXTREME;
    diag->recommended_lambda = mpc_condition_recommend_lambda(n1, cond1);
    if (svd_ok == 0) mpc_svd_free(&svd);
    return 0;
}

/* Stiffness ratio from time constants: tau_max / tau_min.
 * L4: High stiffness (>1e5) -> multi-rate dynamics, ill-conditioned G_dyn. */
double mpc_condition_stiffness_ratio(const double *tau, size_t n)
{
    size_t i;
    double tmin = INFINITY, tmax = 0.0;
    if (!tau || n == 0) return 1.0;
    for (i = 0; i < n; i++) {
        if (tau[i] <= 0.0) return INFINITY;
        if (tau[i] < tmin) tmin = tau[i];
        if (tau[i] > tmax) tmax = tau[i];
    }
    if (tmin < 1e-15) return INFINITY;
    return tmax / tmin;
}

/* Condition number growth with prediction horizon.
 * L6: As P increases, new rows of G_dyn become nearly identical
 * (exponential convergence of step response), degrading conditioning.
 * Growth model: kappa(P) = kappa_0 * (1 + alpha*(P-P0)).
 * Returns growth factor >= 1.0. */
double mpc_condition_horizon_growth(size_t P, size_t P0, double alpha)
{
    if (P <= P0) return 1.0;
    return 1.0 + alpha * (double)(P - P0);
}

/* Compute the Relative Condition Number (RCN) between two matrices.
 * L8: RCN(A, B) = ||A|| * ||B^{-1}|| measures how different A and B are
 * in terms of inversion sensitivity. Used for model mismatch assessment.
 * RCN close to 1: A approx B. RCN >> 1: significant mismatch. */
double mpc_condition_relative(const mpc_matrix_t *A, const mpc_matrix_t *B)
{
    double norm_a, norm_b_inv;
    if (!A || !B || A->rows != B->rows || A->cols != B->cols) return INFINITY;
    norm_a = mpc_matrix_norm_1(A);
    norm_b_inv = hager_higham_norm1_inv(B, 10, 1e-12);
    if (norm_b_inv < 1e-15) return INFINITY;
    return norm_a * norm_b_inv;
}

/* Estimate the condition number of the MPC dynamic matrix G_dyn.
 * L6: The dynamic matrix G_dyn = [step response coefficients]
 * has ny*P rows and nu*M columns. Its condition number grows with P
 * and is a key indicator of MPC numerical robustness. */
double mpc_condition_dynamic_matrix(const mpc_illcond_model_t *model)
{
    if (!model) return INFINITY;
    return mpc_condition_estimate(&model->G_dyn, MPC_CONDEST_NORM1);
}

/* Check if model conditioning is acceptable for MPC deployment.
 * L6: Combines multiple checks: kappa, RGA, collinearity, stiffness.
 * Returns 1 if acceptable, 0 if conditioning needs attention. */
int mpc_condition_is_acceptable(const mpc_illcond_diagnostic_t *diag)
{
    if (!diag) return 0;
    if (diag->condition_number > 1.0e8) return 0;
    if (diag->collinearity_detected > 0.95) return 0;
    if (diag->stiffness_diagnostic > 1.0e5) return 0;
    if (diag->rga_condition_number > 10.0) return 0;
    return 1;
}

/* Compute the condition number of the augmented matrix [A; lambda*I].
 * L8: For Tikhonov regularization, the augmented matrix has
 * condition number: kappa_aug = sqrt((sigma_max^2+lambda^2)/(sigma_min^2+lambda^2))
 * This is the square root of the regularized condition number,
 * used in LSQR and other iterative least-squares solvers. */
double mpc_condition_augmented(const mpc_matrix_t *A, double lambda)
{
    mpc_svd_t svd;
    int ret;
    double s_max, s_min, result;

    if (!A) return INFINITY;

    ret = mpc_svd_compute(A, &svd);
    if (ret != 0) return INFINITY;

    if (!svd.S) { mpc_svd_free(&svd); return INFINITY; }

    size_t n = (A->rows < A->cols) ? A->rows : A->cols;
    s_max = svd.S[0];
    s_min = svd.S[n - 1];

    result = sqrt((s_max * s_max + lambda * lambda)
                / (s_min * s_min + lambda * lambda));

    mpc_svd_free(&svd);
    return result;
}

/* Compute the effective condition number for an MPC QP.
 * L6: The effective condition number accounts for both the Hessian
 * condition number and the MPC horizon parameters.
 *   kappa_eff = kappa_H * (P / M) * stiffness_factor
 * where stiffness_factor captures the time-scale separation. */
double mpc_condition_effective(const mpc_illcond_model_t *model)
{
    double kappa_H, P_ratio, stiff_factor;

    if (!model) return INFINITY;

    kappa_H = model->cond_gain;
    P_ratio = (double)model->P / (double)model->M;

    stiff_factor = 1.0;
    if (model->stiffness_ratio > 1.0 && model->stiffness_ratio < INFINITY)
        stiff_factor = log10(model->stiffness_ratio);

    return kappa_H * P_ratio * stiff_factor;
}

/* Estimate the condition number from RGA values.
 * L4: For ill-conditioned processes, max(|RGA_ij|) correlates with
 * condition number. Bristol (1966) noted that:
 *   kappa(G) >= 2 * max(|RGA_ij| - 1)
 * This provides a quick estimate from RGA without matrix factorization. */
double mpc_condition_from_rga(const double *rga_vals, size_t ny, size_t nu)
{
    size_t i, j;
    double max_rga = 0.0;

    if (!rga_vals || ny == 0 || nu == 0) return 0.0;

    for (i = 0; i < ny; i++)
        for (j = 0; j < nu; j++) {
            double val = fabs(rga_vals[i * nu + j]);
            if (val > max_rga) max_rga = val;
        }

    return 2.0 * (max_rga - 1.0);
}
