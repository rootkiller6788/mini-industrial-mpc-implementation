/** @file mpc_adaptation.c
 * @brief Adaptive MPC: Recursive Least Squares & Kalman Filter (L8)
 *
 * L8 Advanced Topics:
 *   - Recursive Least Squares (RLS) for online model adaptation
 *   - Kalman Filter for state estimation in state-space MPC
 *
 * RLS Theorem (Ljung 1999, Thm 11.1):
 *   For persistently exciting regressor phi(k),
 *   theta_hat(k) -> theta_true as k -> infinity
 *   with covariance P(k) ~ (1/k) * R_phi^{-1}
 *
 * Kalman Filter Theorem (Kalman 1960):
 *   For linear Gaussian system, KF is the optimal
 *   minimum-variance state estimator (MMSE).
 *   x_hat(k|k) minimizes E[||x(k)-x_hat(k|k)||^2]
 *
 * Ref: Ljung "System Identification" (1999) Ch.11
 *      Simon "Optimal State Estimation" (2006) Ch.5-7
 *      Astrom & Wittenmark "Adaptive Control" (2008) Ch.3
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "mpc_common.h"

/* ===== L8: Recursive Least Squares ===== */

mpc_rls_estimator_t* mpc_rls_alloc(int n_params, double lambda, double delta)
{
    if (n_params < 1 || n_params > 200) return NULL;
    if (lambda <= 0.0 || lambda > 1.0) return NULL;
    mpc_rls_estimator_t *rls = (mpc_rls_estimator_t*)calloc(1, sizeof(mpc_rls_estimator_t));
    if (!rls) return NULL;
    rls->n_params = n_params;
    rls->lambda = lambda;
    rls->delta = delta;
    rls->theta = (double*)calloc(n_params, sizeof(double));
    rls->P = (double*)calloc(n_params * n_params, sizeof(double));
    if (!rls->theta || !rls->P) { mpc_rls_free(rls); return NULL; }
    for (int i = 0; i < n_params; i++) rls->P[i * n_params + i] = 1.0 / delta;
    rls->n_updates = 0;
    return rls;
}

void mpc_rls_free(mpc_rls_estimator_t *rls)
{ if (rls) { free(rls->theta); free(rls->P); free(rls); } }

int mpc_rls_update(mpc_rls_estimator_t *rls, const double *phi, double y_meas)
{
    if (!rls || !phi) return -1;
    int n = rls->n_params;
    double lambda = rls->lambda;

    double y_pred = 0.0;
    for (int i = 0; i < n; i++) y_pred += rls->theta[i] * phi[i];
    double error = y_meas - y_pred;

    double *P_phi = (double*)calloc(n, sizeof(double));
    if (!P_phi) return -2;

    for (int i = 0; i < n; i++) {
        double sum = 0.0;
        for (int j = 0; j < n; j++) sum += rls->P[i * n + j] * phi[j];
        P_phi[i] = sum;
    }

    double denom = lambda;
    for (int i = 0; i < n; i++) denom += phi[i] * P_phi[i];

    if (fabs(denom) < MPC_EPS) { free(P_phi); return -3; }

    double *K = (double*)calloc(n, sizeof(double));
    if (!K) { free(P_phi); return -4; }
    for (int i = 0; i < n; i++) K[i] = P_phi[i] / denom;

    for (int i = 0; i < n; i++) rls->theta[i] += K[i] * error;

    double *P_new = (double*)calloc(n * n, sizeof(double));
    if (!P_new) { free(P_phi); free(K); return -5; }

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            double sum = 0.0;
            for (int k = 0; k < n; k++) sum += K[i] * phi[k] * rls->P[k * n + j];
            P_new[i * n + j] = (rls->P[i * n + j] - sum) / lambda;
        }
    }
    memcpy(rls->P, P_new, n * n * sizeof(double));
    rls->n_updates++;

    free(P_phi); free(K); free(P_new);
    return 0;
}

int mpc_rls_get_params(const mpc_rls_estimator_t *rls, double *theta)
{
    if (!rls || !theta) return -1;
    memcpy(theta, rls->theta, rls->n_params * sizeof(double));
    return 0;
}

/* ===== L8: Kalman Filter ===== */

mpc_kalman_state_t* mpc_kalman_alloc(int nx, int ny, int nu)
{
    if (nx < 1 || ny < 1 || nu < 0 || nx > 50) return NULL;
    mpc_kalman_state_t *kf = (mpc_kalman_state_t*)calloc(1, sizeof(mpc_kalman_state_t));
    if (!kf) return NULL;
    kf->nx = nx; kf->ny = ny; kf->nu = nu;
    kf->A    = (double*)calloc(nx * nx, sizeof(double));
    kf->B    = (double*)calloc(nx * nu, sizeof(double));
    kf->C    = (double*)calloc(ny * nx, sizeof(double));
    kf->x_hat = (double*)calloc(nx, sizeof(double));
    kf->P_kf = (double*)calloc(nx * nx, sizeof(double));
    kf->Q_kf = (double*)calloc(nx * nx, sizeof(double));
    kf->R_kf = (double*)calloc(ny * ny, sizeof(double));
    if (!kf->A || !kf->C || !kf->x_hat || !kf->P_kf) { mpc_kalman_free(kf); return NULL; }
    for (int i = 0; i < nx; i++) { kf->A[i*nx+i] = 1.0; kf->P_kf[i*nx+i] = 1.0; kf->Q_kf[i*nx+i] = 0.01; }
    for (int i = 0; i < ny; i++) kf->C[i*nx+i] = 1.0;
    for (int i = 0; i < ny; i++) kf->R_kf[i*ny+i] = 0.1;
    kf->is_initialized = 0;
    return kf;
}

void mpc_kalman_free(mpc_kalman_state_t *kf)
{
    if (kf) {
        free(kf->A); free(kf->B); free(kf->C);
        free(kf->x_hat); free(kf->P_kf);
        free(kf->Q_kf); free(kf->R_kf);
        free(kf);
    }
}

/* Kalman Predict: x(k|k-1) = A * x(k-1|k-1) + B * u(k-1) */
int mpc_kalman_predict(mpc_kalman_state_t *kf, const double *u)
{
    if (!kf || !kf->A || !kf->x_hat) return -1;
    int nx = kf->nx;

    double *x_pred = (double*)calloc(nx, sizeof(double));
    if (!x_pred) return -2;

    for (int i = 0; i < nx; i++) {
        x_pred[i] = 0.0;
        for (int j = 0; j < nx; j++) x_pred[i] += kf->A[i*nx+j] * kf->x_hat[j];
    }
    if (u && kf->B) {
        for (int i = 0; i < nx; i++) {
            for (int j = 0; j < kf->nu; j++) x_pred[i] += kf->B[i*kf->nu+j] * u[j];
        }
    }

    double *APA = (double*)calloc(nx * nx, sizeof(double));
    if (!APA) { free(x_pred); return -3; }
    for (int i = 0; i < nx; i++) {
        for (int j = 0; j < nx; j++) {
            double sum = 0.0;
            for (int k = 0; k < nx; k++) sum += kf->A[i*nx+k] * kf->P_kf[k*nx+j];
            APA[i*nx+j] = sum;
        }
    }
    double *P_pred = (double*)calloc(nx * nx, sizeof(double));
    if (!P_pred) { free(x_pred); free(APA); return -4; }
    for (int i = 0; i < nx; i++) {
        for (int j = 0; j < nx; j++) {
            double sum = 0.0;
            for (int k = 0; k < nx; k++) sum += APA[i*nx+k] * kf->A[j*nx+k];
            P_pred[i*nx+j] = sum + kf->Q_kf[i*nx+j];
        }
    }

    memcpy(kf->x_hat, x_pred, nx * sizeof(double));
    memcpy(kf->P_kf, P_pred, nx * nx * sizeof(double));
    free(x_pred); free(APA); free(P_pred);
    return 0;
}

/* Kalman Correct: x(k|k) = x(k|k-1) + K * (y - C*x(k|k-1)) */
int mpc_kalman_correct(mpc_kalman_state_t *kf, const double *y)
{
    if (!kf || !y || !kf->C) return -1;
    int nx = kf->nx, ny = kf->ny;

    double *y_pred = (double*)calloc(ny, sizeof(double));
    if (!y_pred) return -2;
    for (int i = 0; i < ny; i++) {
        y_pred[i] = 0.0;
        for (int j = 0; j < nx; j++) y_pred[i] += kf->C[i*nx+j] * kf->x_hat[j];
    }

    double *S = (double*)calloc(ny * ny, sizeof(double));
    if (!S) { free(y_pred); return -3; }
    for (int i = 0; i < ny; i++) {
        for (int j = 0; j < ny; j++) {
            double sum = 0.0;
            for (int k = 0; k < nx; k++) {
                double tmp = 0.0;
                for (int l = 0; l < nx; l++) tmp += kf->C[i*nx+l] * kf->P_kf[l*nx+k];
                sum += tmp * kf->C[j*nx+k];
            }
            S[i*ny+j] = sum + kf->R_kf[i*ny+j];
        }
    }

    double *S_inv = (double*)calloc(ny * ny, sizeof(double));
    if (!S_inv) { free(y_pred); free(S); return -4; }
    for (int i = 0; i < ny; i++) {
        if (fabs(S[i*ny+i]) > MPC_EPS) S_inv[i*ny+i] = 1.0 / S[i*ny+i];
    }

    double *K = (double*)calloc(nx * ny, sizeof(double));
    if (!K) { free(y_pred); free(S); free(S_inv); return -5; }
    for (int i = 0; i < nx; i++) {
        for (int j = 0; j < ny; j++) {
            double sum = 0.0;
            for (int k = 0; k < nx; k++) sum += kf->P_kf[i*nx+k] * kf->C[j*nx+k];
            double k_sum = 0.0;
            for (int k = 0; k < ny; k++) k_sum += sum * S_inv[k*ny+j];
            K[i*ny+j] = k_sum;
        }
    }

    double *innovation = (double*)calloc(ny, sizeof(double));
    if (!innovation) { free(y_pred); free(S); free(S_inv); free(K); return -6; }
    for (int i = 0; i < ny; i++) innovation[i] = y[i] - y_pred[i];

    for (int i = 0; i < nx; i++) {
        double corr = 0.0;
        for (int j = 0; j < ny; j++) corr += K[i*ny+j] * innovation[j];
        kf->x_hat[i] += corr;
    }

    kf->is_initialized = 1;
    free(y_pred); free(S); free(S_inv); free(K); free(innovation);
    return 0;
}

/* ===== L8: State-Space to Step-Response Model ===== */

int mpc_ss_to_step_model(const double *A, const double *B,
    const double *C, int nx, int nu, int ny,
    mpc_step_model_t *model_out, int n_steps)
{
    if (!A || !B || !C || !model_out || nx < 1 || nu < 1 || ny < 1 || n_steps < 1) return -1;
    if (n_steps > model_out->n_coeffs) return -2;

    double *x = (double*)calloc(nx, sizeof(double));
    double *x_next = (double*)calloc(nx, sizeof(double));
    if (!x || !x_next) { free(x); free(x_next); return -3; }

    for (int step = 0; step < n_steps; step++) {
        for (int i = 0; i < nx; i++) {
            x[i] += B[i];
        }
        for (int i = 0; i < nx; i++) {
            x_next[i] = 0.0;
            for (int j = 0; j < nx; j++) x_next[i] += A[i*nx+j] * x[j];
        }
        memcpy(x, x_next, nx * sizeof(double));

        double y = 0.0;
        for (int i = 0; i < nx; i++) y += C[i] * x[i];
        model_out->coeff[step] = y;
    }

    model_out->gain_ss = model_out->coeff[n_steps - 1];
    free(x); free(x_next);
    return 0;
}

/* State-space prediction */
int mpc_ss_predict(const double *A, const double *B, const double *C,
    int nx, int nu, int ny, const double *x0,
    const double *u_seq, double *y_seq, int N)
{
    if (!A || !B || !C || !x0 || !u_seq || !y_seq) return -1;
    double *x = (double*)calloc(nx, sizeof(double));
    double *x_next = (double*)calloc(nx, sizeof(double));
    if (!x || !x_next) { free(x); free(x_next); return -2; }
    memcpy(x, x0, nx * sizeof(double));

    for (int k = 0; k < N; k++) {
        for (int i = 0; i < nx; i++) {
            x_next[i] = 0.0;
            for (int j = 0; j < nx; j++) x_next[i] += A[i*nx+j] * x[j];
            for (int j = 0; j < nu; j++) x_next[i] += B[i*nu+j] * u_seq[k*nu+j];
        }
        for (int i = 0; i < ny; i++) {
            y_seq[k*ny+i] = 0.0;
            for (int j = 0; j < nx; j++) y_seq[k*ny+i] += C[i*nx+j] * x_next[j];
        }
        memcpy(x, x_next, nx * sizeof(double));
    }
    free(x); free(x_next);
    return 0;
}
