#include "mpc_illcond_defs.h"
#include "mpc_illcond_matrix.h"
#include "mpc_illcond_regularization.h"
#include "mpc_illcond_preconditioner.h"
#include "mpc_illcond_sensitivity.h"
#include "mpc_illcond_svd.h"
#include "mpc_illcond_condition.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* Forward declaration */
void mpc_qp_free(mpc_illcond_qp_t *qp);

/* Build MPC Hessian: H = Theta^T*Q*Theta + R.
 * L3: Standard DMC/GPC QP formulation.
 * Reference: Rawlings, Mayne, Diehl (2017), Eq. 8.17. */
int mpc_qp_build_hessian(const mpc_illcond_model_t *model,
                          const double *Q_diag, const double *R_diag,
                          mpc_illcond_qp_t *qp)
{
    size_t i, j, k, nyP, nuM;
    if (!model || !Q_diag || !R_diag || !qp) return -1;
    nyP = model->ny * model->P;
    nuM = model->nu * model->M;
    qp->n_vars = nuM;
    qp->n_eq = 0;
    qp->n_ineq = 0;
    mpc_matrix_zero(&qp->H);
    for (i = 0; i < nuM; i++)
        qp->H.data[i * qp->H.stride + i] = R_diag[i % model->nu];
    for (i = 0; i < nuM; i++)
        for (j = 0; j < nuM; j++) {
            double sum = 0.0;
            for (k = 0; k < nyP; k++)
                sum += model->G_dyn.data[k * model->G_dyn.stride + i]
                     * Q_diag[k % model->ny]
                     * model->G_dyn.data[k * model->G_dyn.stride + j];
            qp->H.data[i * qp->H.stride + j] += sum;
        }
    mpc_condition_diagnose(&qp->H, &qp->diag);
    qp->cond_H = qp->diag.condition_number;
    qp->is_illcond = mpc_condition_is_illcond(qp->cond_H);
    return 0;
}

/* Free response error: e0 = y_sp - y_free.
 * y_free = contribution of past control moves to future outputs. */
int mpc_qp_free_response_error(const mpc_illcond_model_t *model,
                                const double *y_sp, const double *y_meas,
                                const double *past_delta_u, size_t past_len,
                                double *e0)
{
    size_t i, j, nyP;
    if (!model || !y_sp || !y_meas || !e0) return -1;
    nyP = model->ny * model->P;
    for (i = 0; i < nyP; i++) {
        double free_resp = y_meas[i % model->ny];
        for (j = 0; j < past_len && j < model->nu * model->M; j++)
            free_resp += model->G_dyn.data[i * model->G_dyn.stride + j]
                       * past_delta_u[j];
        e0[i] = y_sp[i % model->ny] - free_resp;
    }
    return 0;
}

/* Solve regularized MPC QP via Cholesky (direct) or PCG (iterative).
 * L5: For n < 200, use Cholesky. For larger or ill-conditioned, use PCG. */
int mpc_qp_solve_regularized(mpc_illcond_qp_t *qp,
                              const double *e0, double *delta_u,
                              const mpc_regularization_t *reg)
{
    size_t i, n;
    double *rhs;
    mpc_matrix_t *H_reg;
    if (!qp || !e0 || !delta_u || !reg) return -1;
    n = qp->n_vars;
    if (n == 0) return -1;
    H_reg = mpc_matrix_copy(&qp->H);
    if (!H_reg) return -1;
    rhs = (double*)malloc(n * sizeof(double));
    if (!rhs) { mpc_matrix_free(&H_reg); return -1; }
    if (reg->type == MPC_REGULARIZE_TIKHONOV)
        mpc_regularize_tikhonov(H_reg, reg->lambda);
    else if (reg->type == MPC_REGULARIZE_TIKHONOV_DX)
        mpc_regularize_tikhonov(H_reg, reg->lambda_delta_u);
    else
        mpc_regularize_tikhonov(H_reg, 1e-8);
    for (i = 0; i < n; i++) rhs[i] = e0[i % qp->H.rows];
    if (mpc_matrix_cholesky(H_reg) == 0) {
        mpc_matrix_forward_sub(H_reg, rhs, delta_u);
        for (i = 0; i < n; i++) rhs[i] = delta_u[i];
        mpc_matrix_backward_sub(H_reg, rhs, delta_u);
    } else {
        mpc_preconditioner_t prec;
        prec.type = MPC_PRECOND_JACOBI;
        mpc_precond_jacobi_build(H_reg, &prec);
        mpc_pcg_solve(H_reg, rhs, delta_u, &prec, 1000, 1e-8);
        mpc_precond_free(&prec);
    }
    mpc_condition_diagnose(H_reg, &qp->diag);
    qp->cond_H = qp->diag.condition_number;
    qp->is_illcond = mpc_condition_is_illcond(qp->cond_H);
    mpc_matrix_free(&H_reg);
    free(rhs);
    return 0;
}

/* Check if QP Hessian is well-conditioned for unregularized solve. */
int mpc_qp_check_conditioning(const mpc_illcond_qp_t *qp)
{
    if (!qp) return 0;
    return (qp->cond_H <= 1.0e8 && !qp->is_illcond) ? 1 : 0;
}

/* Worst-case control error bound from ill-conditioning.
 * ||dx||/||x|| <= kappa(H) * eps_mach (Wilkinson, 1963).
 * Returns estimated max relative error. */
double mpc_qp_control_error_bound(const mpc_illcond_qp_t *qp)
{
    if (!qp) return 0.0;
    return qp->cond_H * 2.2204460492503131e-16;
}

/* Initialize QP structure with given dimensions. Allocates all matrices. */
int mpc_qp_init(mpc_illcond_qp_t *qp, size_t n_vars,
                 size_t n_eq, size_t n_ineq)
{
    if (!qp || n_vars == 0) return -1;
    memset(qp, 0, sizeof(mpc_illcond_qp_t));
    qp->n_vars = n_vars; qp->n_eq = n_eq; qp->n_ineq = n_ineq;
    qp->H.rows = n_vars; qp->H.cols = n_vars; qp->H.stride = n_vars;
    qp->H.data = (double*)calloc(n_vars * n_vars, sizeof(double));
    qp->f = (double*)calloc(n_vars, sizeof(double));
    if (n_eq > 0) {
        qp->A_eq.rows = n_eq; qp->A_eq.cols = n_vars;
        qp->A_eq.stride = n_vars;
        qp->A_eq.data = (double*)calloc(n_eq * n_vars, sizeof(double));
        qp->b_eq = (double*)calloc(n_eq, sizeof(double));
    }
    if (n_ineq > 0) {
        qp->A_ineq.rows = n_ineq; qp->A_ineq.cols = n_vars;
        qp->A_ineq.stride = n_vars;
        qp->A_ineq.data = (double*)calloc(n_ineq * n_vars, sizeof(double));
        qp->b_ineq = (double*)calloc(n_ineq, sizeof(double));
    }
    if (!qp->H.data || !qp->f ||
        (n_eq > 0 && (!qp->A_eq.data || !qp->b_eq)) ||
        (n_ineq > 0 && (!qp->A_ineq.data || !qp->b_ineq))) {
        mpc_qp_free(qp); return -1;
    }
    return 0;
}

/* Free all QP resources. */
void mpc_qp_free(mpc_illcond_qp_t *qp)
{
    if (!qp) return;
    free(qp->H.data); qp->H.data = NULL;
    free(qp->f); qp->f = NULL;
    free(qp->A_eq.data); qp->A_eq.data = NULL;
    free(qp->b_eq); qp->b_eq = NULL;
    free(qp->A_ineq.data); qp->A_ineq.data = NULL;
    free(qp->b_ineq); qp->b_ineq = NULL;
}

/* Auto-detect ill-conditioning and recommend regularization.
 * L6: Integrated diagnostic for MPC operators.
 * Threshold-based strategy mapping condition to regularization strength. */
int mpc_qp_auto_recommend(const mpc_illcond_qp_t *qp,
                           mpc_regularization_t *recommended)
{
    if (!qp || !recommended) return -1;
    memset(recommended, 0, sizeof(mpc_regularization_t));
    if (qp->cond_H < 1.0e3) {
        recommended->type = MPC_REGULARIZE_TIKHONOV;
        recommended->lambda = 1e-10;
    } else if (qp->cond_H < 1.0e6) {
        recommended->type = MPC_REGULARIZE_TIKHONOV;
        recommended->lambda = 1e-6;
    } else if (qp->cond_H < 1.0e8) {
        recommended->type = MPC_REGULARIZE_TIKHONOV;
        recommended->lambda = 1e-4;
    } else {
        recommended->type = MPC_REGULARIZE_LEVENBERG;
        recommended->lambda = 1e-2;
        recommended->max_lm_iter = 20;
        recommended->lm_decay = 0.5;
    }
    return 0;
}

/* Build the dynamic matrix G_dyn from step response coefficients.
 * L3: G_dyn(i,j) = step_response coefficient for output i at step j.
 * For FOPDT model: a_k = K*(1 - exp(-k*Ts/tau)) for k >= 1.
 * This constructs the matrix used in DMC prediction.
 * Returns 0 on success, -1 on failure. */
int mpc_qp_build_dynamic_matrix(size_t ny, size_t nu, size_t P, size_t M,
                                 const double *K, const double *tau,
                                 double Ts, mpc_matrix_t *G_dyn)
{
    size_t i, j, k;
    if (!K || !tau || !G_dyn || ny == 0 || nu == 0 || P == 0 || M == 0)
        return -1;
    if (G_dyn->rows != ny * P || G_dyn->cols != nu * M) return -1;
    mpc_matrix_zero(G_dyn);

    for (i = 0; i < ny; i++) {
        for (j = 0; j < nu; j++) {
            double gain = K[i * nu + j];
            double tc = tau[i * nu + j];
            if (tc <= 0.0) tc = 1.0;
            for (k = 0; k < M; k++) {
                double a_k = gain * (1.0 - exp(-((double)(k+1) * Ts) / tc));
                size_t row = i * P + k;
                size_t col = j * M + k;
                G_dyn->data[row * G_dyn->stride + col] = a_k;
            }
            /* Fill below diagonal for later prediction steps */
            for (k = 0; k < M; k++) {
                double a_k = gain * (1.0 - exp(-((double)(k+1) * Ts) / tc));
                for (size_t p = k + 1; p < P; p++) {
                    size_t row = i * P + p;
                    size_t col = j * M + k;
                    double a_p = gain * (1.0 - exp(-((double)(p+1) * Ts) / tc));
                    G_dyn->data[row * G_dyn->stride + col] = a_p - a_k;
                }
            }
        }
    }
    return 0;
}

/* Build the MPC QP from model and tuning parameters.
 * L6: Complete QP construction for DMC.
 *   min 0.5 * dU^T * H * dU + c^T * dU
 *   s.t. constraints (optional)
 * Returns 0 on success. */
int mpc_qp_build(const mpc_illcond_model_t *model,
                  const double *Q_diag, const double *R_diag,
                  const double *y_sp, const double *y_meas,
                  mpc_illcond_qp_t *qp, double *e0)
{
    int ret;
    if (!model || !Q_diag || !R_diag || !y_sp || !y_meas || !qp || !e0)
        return -1;

    /* Build Hessian H = Theta^T*Q*Theta + R */
    ret = mpc_qp_build_hessian(model, Q_diag, R_diag, qp);
    if (ret != 0) return ret;

    /* Compute free response error e0 */
    ret = mpc_qp_free_response_error(model, y_sp, y_meas,
                                      NULL, 0, e0);
    if (ret != 0) return ret;

    /* Set linear cost term: c = -Theta^T * Q * e0 */
    for (size_t i = 0; i < qp->n_vars; i++) {
        qp->f[i] = 0.0;
        for (size_t k = 0; k < model->ny * model->P; k++) {
            double theta_ki = model->G_dyn.data[k * model->G_dyn.stride + i];
            qp->f[i] -= theta_ki * Q_diag[k % model->ny] * e0[k];
        }
    }

    return 0;
}

/* Compute the predicted output trajectory given control moves.
 * L6: y_pred = y_free + Theta * delta_u
 * Used for closed-loop simulation and constraint checking. */
int mpc_qp_predict_output(const mpc_illcond_model_t *model,
                           const double *y_meas,
                           const double *delta_u,
                           double *y_pred)
{
    size_t i, j, nyP, nuM;
    if (!model || !y_meas || !delta_u || !y_pred) return -1;
    nyP = model->ny * model->P;
    nuM = model->nu * model->M;

    for (i = 0; i < nyP; i++) {
        y_pred[i] = y_meas[i % model->ny];
        for (j = 0; j < nuM; j++) {
            y_pred[i] += model->G_dyn.data[i * model->G_dyn.stride + j]
                       * delta_u[j];
        }
    }
    return 0;
}

/* Estimate the numerical rank loss from prediction horizon extension.
 * L6: Adding prediction horizon steps adds nearly-collinear rows to G_dyn.
 * Returns the estimated effective rank for given P. */
size_t mpc_qp_estimate_rank_loss(const mpc_illcond_model_t *model, size_t P_new)
{
    size_t nu, P_orig;
    double tau_avg;

    if (!model) return 0;
    nu = model->nu;
    P_orig = model->P;

    if (P_new <= P_orig) return nu * model->M;

    /* Estimate: each output channel contributes min(P * nu, nu * M)
     * independent directions. Extended P adds near-collinear rows. */
    tau_avg = 1.0;
    if (model->stiffness_ratio > 0.0 && model->stiffness_ratio < INFINITY)
        tau_avg = model->stiffness_ratio;

    /* Rank growth saturates after ~5*tau_avg/Ts steps */
    size_t effective_P = (P_new < (size_t)(5.0 * tau_avg)) ? P_new
                        : (size_t)(5.0 * tau_avg);
    (void)P_orig;

    return nu * effective_P;
}
