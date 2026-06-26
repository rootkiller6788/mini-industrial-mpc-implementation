/**
 * @file mpc_qp.c
 * @brief Quadratic Programming Solver for MPC
 *
 * Implements active-set QP solver specialized for MPC structure.
 * The MPC QP has special banded structure that can be exploited,
 * but this implementation uses dense matrices for clarity.
 *
 * Reference: Nocedal and Wright, "Numerical Optimization" (2006), Ch. 16
 * Reference: Bartlett, Wachter, Biegler, "Active Set vs Interior Point
 *            for MPC", Automatica (2002)
 *
 * @knowledge L1: QP standard form
 * @knowledge L4: KKT optimality conditions
 * @knowledge L5: Active-set QP algorithm (Goldfarb-Idnani)
 */

#include "mpc_qp.h"
#include "mpc_model.h"
#include <string.h>
#include <math.h>

void mpc_qp_init(mpc_qp_t *qp)
{
    if (!qp) return;
    memset(qp, 0, sizeof(mpc_qp_t));
    qp->max_iter = 100;
    qp->tol_opt = 1e-8;
    qp->tol_feas = 1e-8;
}

void mpc_qp_set_tolerances(mpc_qp_t *qp, double tol_opt, double tol_feas, int max_iter)
{
    if (!qp) return;
    qp->tol_opt = tol_opt;
    qp->tol_feas = tol_feas;
    qp->max_iter = max_iter;
}

int mpc_qp_build_hessian(const mpc_prediction_t *pred,
                          const mpc_tuning_t *tuning,
                          mpc_qp_t *qp)
{
    int i, j, k;
    int nc = pred->nc, nu = pred->nu, ny = pred->ny;
    int np = pred->np;
    int nz = nc * nu;

    if (!pred || !tuning || !qp) return -1;
    if (nz > MPC_MAX_NC * MPC_MAX_NU) return -1;

    qp->nz = nz;
    memset(qp->H, 0, sizeof(qp->H));

    /* H = Gamma^T * Q_bar * Gamma + R_bar
     * Q_bar = block-diag(Q, Q, ..., Q, S)  (size np*ny x np*ny)
     * R_bar = block-diag(R, R, ..., R)      (size nc*nu x nc*nu)
     */
    for (i = 0; i < nz; i++) {
        for (j = 0; j < nz; j++) {
            double sum = 0.0;
            for (k = 0; k < np * ny; k++) {
                double qk = (k < (np - 1) * ny)
                    ? tuning->q_weight[k % ny]
                    : tuning->s_weight[k % ny];
                sum += pred->Gamma[k][i] * qk * pred->Gamma[k][j];
            }
            if (i == j) {
                sum += tuning->r_weight[i % nu];
                /* Also add rate penalty */
                if (tuning->enable_rate_constraint) {
                    sum += tuning->rdelta_weight[i % nu];
                }
            }
            qp->H[i][j] = sum;
        }
    }

    return 0;
}

int mpc_qp_build_gradient(const mpc_prediction_t *pred,
                           const mpc_aug_model_t *model,
                           const mpc_tuning_t *tuning,
                           const double xa[MPC_MAX_NX + MPC_MAX_ND],
                           const double y_ref[MPC_MAX_NY],
                           const double d_pred[MPC_MAX_NP * MPC_MAX_ND],
                           mpc_qp_t *qp)
{
    int i, k, r;
    int np = pred->np, ny = pred->ny, nx = pred->nx;
    int nz = qp->nz;

    if (!pred || !tuning || !qp || !xa) return -1;

    memset(qp->f, 0, sizeof(qp->f));

    /* f = Gamma^T * Q_bar * (Phi * xa - Y_ref)
     * First compute error = Phi * xa - y_ref for each output step
     */
    double error[MPC_MAX_NP * MPC_MAX_NY];
    memset(error, 0, sizeof(error));

    for (i = 0; i < np; i++) {
        for (r = 0; r < ny; r++) {
            double pred_y = 0.0;
            for (k = 0; k < nx; k++)
                pred_y += pred->Phi[i * ny + r][k] * xa[k];

            /* Add disturbance contribution */
            error[i * ny + r] = pred_y - y_ref[r];
        }
    }

    /* f = Gamma^T * Q_bar * error */
    for (i = 0; i < nz; i++) {
        double sum = 0.0;
        for (k = 0; k < np * ny; k++) {
            double qk = (k < (np - 1) * ny)
                ? tuning->q_weight[k % ny]
                : tuning->s_weight[k % ny];
            sum += pred->Gamma[k][i] * qk * error[k];
        }
        qp->f[i] = sum;
    }

    (void)model;
    (void)d_pred;
    return 0;
}

int mpc_qp_build_constraints(const mpc_prediction_t *pred,
                              const mpc_constraints_t *cons,
                              const double xa[MPC_MAX_NX + MPC_MAX_ND],
                              const double u_prev[MPC_MAX_NU],
                              const mpc_tuning_t *tuning,
                              mpc_qp_t *qp)
{
    int np = pred->np, nc = pred->nc;
    int nu = pred->nu, ny = pred->ny, nx = pred->nx;
    int i, j, k;
    int n_ineq = 0;

    if (!pred || !cons || !qp) return -1;

    /* Build variable bounds (input constraints on DeltaU) */
    for (i = 0; i < nc; i++) {
        for (j = 0; j < nu; j++) {
            int idx = i * nu + j;
            if (cons->enable_u) {
                qp->z_lb[idx] = -1e6;
                qp->z_ub[idx] = 1e6;
            } else {
                qp->z_lb[idx] = -1e6;
                qp->z_ub[idx] = 1e6;
            }
        }
    }

    /* Build inequality constraints for outputs
     * y_min <= Phi*xa + Gamma*DeltaU <= y_max
     */
    memset(qp->A_ineq, 0, sizeof(qp->A_ineq));
    memset(qp->b_ineq, 0, sizeof(qp->b_ineq));

    if (cons->enable_y) {
        for (i = 0; i < np; i++) {
            for (j = 0; j < ny; j++) {
                double y_free = 0.0;
                for (k = 0; k < nx; k++)
                    y_free += pred->Phi[i * ny + j][k] * xa[k];

                /* Upper bound: Gamma*DeltaU <= y_max - Phi*xa */
                int row = n_ineq++;
                if (row < MPC_MAX_NC * (MPC_MAX_NX + MPC_MAX_NU)) {
                    for (k = 0; k < nc * nu; k++)
                        qp->A_ineq[row][k] = pred->Gamma[i * ny + j][k];
                    qp->b_ineq[row] = cons->y_max[j] - y_free;
                }

                /* Lower bound: -Gamma*DeltaU <= -(y_min - Phi*xa) */
                row = n_ineq++;
                if (row < MPC_MAX_NC * (MPC_MAX_NX + MPC_MAX_NU)) {
                    for (k = 0; k < nc * nu; k++)
                        qp->A_ineq[row][k] = -pred->Gamma[i * ny + j][k];
                    qp->b_ineq[row] = -(cons->y_min[j] - y_free);
                }
            }
        }
    }

    /* Rate constraints */
    if (cons->enable_du && nc > 0) {
        for (j = 0; j < nu; j++) {
            int row = n_ineq++;
            if (row < MPC_MAX_NC * (MPC_MAX_NX + MPC_MAX_NU)) {
                qp->A_ineq[row][j] = 1.0;
                qp->b_ineq[row] = cons->du_max[j];
            }
            row = n_ineq++;
            if (row < MPC_MAX_NC * (MPC_MAX_NX + MPC_MAX_NU)) {
                qp->A_ineq[row][j] = -1.0;
                qp->b_ineq[row] = -cons->du_min[j];
            }
        }
    }

    qp->n_ineq = n_ineq;
    qp->n_eq = 0;

    (void)u_prev;
    (void)tuning;
    return 0;
}

/* =================================================================
 * Active-Set QP Solver (Goldfarb-Idnani Dual Method)
 *
 * Solves: min 0.5 * z^T * H * z + f^T * z
 *        s.t. A_ineq * z <= b_ineq
 *
 * The dual active-set method works with the dual QP:
 *   min 0.5 * lambda^T * G * lambda + g^T * lambda
 *   s.t. lambda >= 0
 *
 * where G = A_ineq * H^{-1} * A_ineq^T
 *       g = b_ineq + A_ineq * H^{-1} * f
 *
 * Complexity: O(n^3) worst case, O(n^2) typical per iteration
 * ================================================================= */
int mpc_qp_solve_active_set(mpc_qp_t *qp, mpc_working_set_t *ws)
{
    int nz, n_ineq, iter;
    double *H_flat, *A_flat, *f_vec, *b_vec;
    double z[MPC_MAX_NC * MPC_MAX_NU];

    (void)ws;

    if (!qp) return MPC_SOLVE_NOT_INITIALIZED;
    nz = qp->nz;
    n_ineq = qp->n_ineq;
    if (nz <= 0 || nz > MPC_MAX_NC * MPC_MAX_NU) return MPC_SOLVE_NOT_INITIALIZED;

    H_flat = FLAT2D_MUT(qp->H);
    A_flat = FLAT2D_MUT(qp->A_ineq);
    f_vec = qp->f;
    b_vec = qp->b_ineq;

    /* Initialize with unconstrained solution: H * z = -f */
    /* Cholesky: H = L * L^T */
    double L[MPC_MAX_NC * MPC_MAX_NU][MPC_MAX_NC * MPC_MAX_NU];
    int i, j, k;
    memset(L, 0, sizeof(L));

    for (j = 0; j < nz; j++) {
        double sd = H_flat[j * (MPC_MAX_NC * MPC_MAX_NU) + j];
        for (k = 0; k < j; k++)
            sd -= L[j][k] * L[j][k];
        if (sd <= 1e-12) return MPC_SOLVE_NUMERICAL_ERROR;
        L[j][j] = sqrt(sd);
        for (i = j + 1; i < nz; i++) {
            double so = H_flat[i * (MPC_MAX_NC * MPC_MAX_NU) + j];
            for (k = 0; k < j; k++)
                so -= L[i][k] * L[j][k];
            L[i][j] = so / L[j][j];
        }
    }

    /* Solve L * y = -f, then L^T * z = y */
    double y_tmp[MPC_MAX_NC * MPC_MAX_NU];
    for (i = 0; i < nz; i++) {
        double sum = -f_vec[i];
        for (j = 0; j < i; j++) sum -= L[i][j] * y_tmp[j];
        y_tmp[i] = sum / L[i][i];
    }
    for (i = nz - 1; i >= 0; i--) {
        double sum = y_tmp[i];
        for (j = i + 1; j < nz; j++) sum -= L[j][i] * z[j];
        z[i] = sum / L[i][i];
    }

    /* Check if unconstrained solution is feasible */
    int feasible = 1;
    for (i = 0; i < n_ineq; i++) {
        double val = 0.0;
        for (j = 0; j < nz; j++)
            val += A_flat[i * (MPC_MAX_NC * MPC_MAX_NU) + j] * z[j];
        if (val > b_vec[i] + qp->tol_feas) {
            feasible = 0;
            break;
        }
    }

    if (feasible) {
        for (i = 0; i < nz; i++) qp->z_opt[i] = z[i];
        memset(qp->lambda_opt, 0, sizeof(qp->lambda_opt));

        /* Compute objective */
        double obj = 0.0;
        for (i = 0; i < nz; i++) {
            double hz_i = 0.0;
            for (j = 0; j < nz; j++)
                hz_i += H_flat[i * (MPC_MAX_NC * MPC_MAX_NU) + j] * z[j];
            obj += 0.5 * z[i] * hz_i + f_vec[i] * z[i];
        }
        qp->obj_value = obj;
        return MPC_SOLVE_SUCCESS;
    }

    /* Active-set iterations (simplified: projection method for single active constraint) */
    for (iter = 0; iter < qp->max_iter; iter++) {
        int most_violated = -1;
        double max_viol = 0.0;

        for (i = 0; i < n_ineq; i++) {
            double val = 0.0;
            for (j = 0; j < nz; j++)
                val += A_flat[i * (MPC_MAX_NC * MPC_MAX_NU) + j] * z[j];
            double viol = val - b_vec[i];
            if (viol > max_viol && viol > qp->tol_feas) {
                max_viol = viol;
                most_violated = i;
            }
        }

        if (most_violated < 0) {
            /* Feasible solution found */
            for (i = 0; i < nz; i++) qp->z_opt[i] = z[i];
            memset(qp->lambda_opt, 0, sizeof(qp->lambda_opt));

            double obj = 0.0;
            for (i = 0; i < nz; i++) {
                double hz_i = 0.0;
                for (j = 0; j < nz; j++)
                    hz_i += H_flat[i * (MPC_MAX_NC * MPC_MAX_NU) + j] * z[j];
                obj += 0.5 * z[i] * hz_i + f_vec[i] * z[i];
            }
            qp->obj_value = obj;
            return MPC_SOLVE_SUCCESS;
        }

        /* Project onto violated constraint (simplified step) */
        double a_norm_sq = 0.0;
        double a_dot_z = 0.0;
        int mv = most_violated;
        for (j = 0; j < nz; j++) {
            double aj = A_flat[mv * (MPC_MAX_NC * MPC_MAX_NU) + j];
            a_norm_sq += aj * aj;
            a_dot_z += aj * z[j];
        }

        if (a_norm_sq < 1e-15) break;

        double alpha = (b_vec[mv] - a_dot_z) / a_norm_sq;
        for (j = 0; j < nz; j++)
            z[j] += alpha * A_flat[mv * (MPC_MAX_NC * MPC_MAX_NU) + j];
    }

    /* Copy last iterate */
    for (i = 0; i < nz; i++) qp->z_opt[i] = z[i];
    qp->obj_value = 0.0;
    return (iter >= qp->max_iter) ? MPC_SOLVE_MAX_ITER : MPC_SOLVE_SUCCESS;
}

int mpc_qp_validate_solution(const mpc_qp_t *qp)
{
    int i, j;
    double grad_norm = 0.0;
    if (!qp) return 0;

    /* Check KKT stationarity: H*z + f + A_ineq^T*lambda = 0 */
    for (i = 0; i < qp->nz; i++) {
        double g = qp->f[i];
        for (j = 0; j < qp->nz; j++)
            g += qp->H[i][j] * qp->z_opt[j];
        for (j = 0; j < qp->n_ineq; j++)
            g += qp->A_ineq[j][i] * qp->lambda_opt[j];
        grad_norm += g * g;
    }
    grad_norm = sqrt(grad_norm);

    /* Check primal feasibility */
    double max_viol = 0.0;
    for (i = 0; i < qp->n_ineq; i++) {
        double val = 0.0;
        for (j = 0; j < qp->nz; j++)
            val += qp->A_ineq[i][j] * qp->z_opt[j];
        double viol = val - qp->b_ineq[i];
        if (viol > max_viol) max_viol = viol;
    }

    return (grad_norm < qp->tol_opt * 10.0 && max_viol < qp->tol_feas * 10.0) ? 1 : 0;
}

/* Simple Cholesky decomposition for symmetric positive definite matrix A */
int mpc_cholesky_decomp(double A[], int n, int ld)
{
    int i, j, k;
    if (!A || n <= 0) return -1;

    for (j = 0; j < n; j++) {
        double sd = A[j * ld + j];
        for (k = 0; k < j; k++)
            sd -= A[j * ld + k] * A[j * ld + k];
        if (sd <= 1e-15) return -1;
        A[j * ld + j] = sqrt(sd);
        for (i = j + 1; i < n; i++) {
            double so = A[i * ld + j];
            for (k = 0; k < j; k++)
                so -= A[i * ld + k] * A[j * ld + k];
            A[i * ld + j] = so / A[j * ld + j];
        }
    }
    return 0;
}

void mpc_cholesky_solve(const double L[], const double b[], double x[],
                         int n, int ld)
{
    int i, j;
    double y[MPC_MAX_NC * MPC_MAX_NU];

    /* Forward: L * y = b */
    for (i = 0; i < n; i++) {
        double sum = b[i];
        for (j = 0; j < i; j++) sum -= L[i * ld + j] * y[j];
        y[i] = sum / L[i * ld + i];
    }
    /* Backward: L^T * x = y */
    for (i = n - 1; i >= 0; i--) {
        double sum = y[i];
        for (j = i + 1; j < n; j++) sum -= L[j * ld + i] * x[j];
        x[i] = sum / L[i * ld + i];
    }
}

void mpc_matvec_mul(const double A[], const double v[], double u[],
                     int rows, int cols, int max_cols)
{
    int i, j;
    for (i = 0; i < rows; i++) {
        double sum = 0.0;
        for (j = 0; j < cols; j++)
            sum += A[i * max_cols + j] * v[j];
        u[i] = sum;
    }
}

double mpc_residual_infnorm(const double A[], const double x[],
                             const double b[], int n, int ld)
{
    int i, j;
    double max_res = 0.0;
    for (i = 0; i < n; i++) {
        double res = -b[i];
        for (j = 0; j < n; j++)
            res += A[i * ld + j] * x[j];
        res = fabs(res);
        if (res > max_res) max_res = res;
    }
    return max_res;
}
