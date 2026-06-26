/**
 * @file mpc_observer.c
 * @brief State and Disturbance Estimation for MPC
 *
 * Reference: Simon, "Optimal State Estimation" (2006)
 * Reference: Muske and Badgwell, "Disturbance Modeling for
 *            Offset-Free Linear MPC", J. Process Control (2002)
 *
 * @knowledge L2: Observer concept (Luenberger, Kalman)
 * @knowledge L4: Separation principle for LQG
 * @knowledge L5: Kalman filter algorithm
 */

#include "mpc_observer.h"
#include <string.h>
#include <math.h>

void mpc_kalman_init(mpc_kalman_params_t *kf, int nx_aug, int ny)
{
    int i, j;
    if (!kf) return;
    memset(kf, 0, sizeof(mpc_kalman_params_t));
    for (i = 0; i < nx_aug; i++) {
        for (j = 0; j < nx_aug; j++) {
            kf->Q[i][j] = (i == j) ? 0.01 : 0.0;
            kf->P[i][j] = (i == j) ? 1.0 : 0.0;
            kf->P0[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }
    for (i = 0; i < ny; i++)
        for (j = 0; j < ny; j++)
            kf->R[i][j] = (i == j) ? 0.1 : 0.0;
}

int mpc_kalman_predict(const mpc_aug_model_t *model,
                        const mpc_kalman_params_t *kf,
                        const double u[MPC_MAX_NU],
                        mpc_observer_state_t *obs)
{
    int i, j, k;
    int n = model->nx_aug;
    double Ax[MPC_MAX_NX + MPC_MAX_ND];
    double Bu[MPC_MAX_NX + MPC_MAX_ND];
    double AP[MPC_MAX_NX + MPC_MAX_ND][MPC_MAX_NX + MPC_MAX_ND];
    double APA[MPC_MAX_NX + MPC_MAX_ND][MPC_MAX_NX + MPC_MAX_ND];
    mpc_kalman_params_t *kf_mut;

    if (!model || !kf || !obs) return -1;
    if (n > MPC_MAX_NX + MPC_MAX_ND) return -1;

    /* x[k|k-1] = A * x[k-1|k-1] + B * u[k-1] */
    for (i = 0; i < n; i++) {
        double s1 = 0.0, s2 = 0.0;
        for (j = 0; j < n; j++)
            s1 += model->Aa[i][j] * obs->x_aug_hat[j];
        for (j = 0; j < model->nu; j++)
            s2 += model->Ba[i][j] * u[j];
        Ax[i] = s1;
        Bu[i] = s2;
    }
    for (i = 0; i < n; i++)
        obs->x_aug_hat[i] = Ax[i] + Bu[i];

    /* P[k|k-1] = A * P * A^T + Q */
    for (i = 0; i < n; i++)
        for (j = 0; j < n; j++) {
            double s = 0.0;
            for (k = 0; k < n; k++)
                s += model->Aa[i][k] * kf->P[k][j];
            AP[i][j] = s;
        }
    for (i = 0; i < n; i++)
        for (j = 0; j < n; j++) {
            double s = 0.0;
            for (k = 0; k < n; k++)
                s += AP[i][k] * model->Aa[j][k];
            APA[i][j] = s + kf->Q[i][j];
        }

    kf_mut = (mpc_kalman_params_t *)kf;
    for (i = 0; i < n; i++)
        for (j = 0; j < n; j++)
            kf_mut->P[i][j] = APA[i][j];

    return 0;
}

int mpc_kalman_update(const mpc_aug_model_t *model,
                       mpc_kalman_params_t *kf,
                       const double y_meas[MPC_MAX_NY],
                       mpc_observer_state_t *obs)
{
    int i, j, k;
    int n = model->nx_aug, ny = model->ny;
    double y_pred[MPC_MAX_NY];
    double CP[MPC_MAX_NY][MPC_MAX_NX + MPC_MAX_ND];
    double CPC_R[MPC_MAX_NY][MPC_MAX_NY];
    double S_inv[MPC_MAX_NY][MPC_MAX_NY];
    double K_gain[MPC_MAX_NX + MPC_MAX_ND][MPC_MAX_NY];
    double KC[MPC_MAX_NX + MPC_MAX_ND][MPC_MAX_NX + MPC_MAX_ND];
    double P_new[MPC_MAX_NX + MPC_MAX_ND][MPC_MAX_NX + MPC_MAX_ND];

    if (!model || !kf || !obs || !y_meas) return -1;

    /* Innovation: y_tilde = y_meas - C * x_hat */
    for (i = 0; i < ny; i++) {
        double s = 0.0;
        for (j = 0; j < n; j++)
            s += model->Ca[i][j] * obs->x_aug_hat[j];
        y_pred[i] = s;
        obs->innovation[i] = y_meas[i] - y_pred[i];
        obs->y_hat[i] = y_pred[i];
    }

    /* S = C * P * C^T + R */
    for (i = 0; i < ny; i++)
        for (j = 0; j < n; j++) {
            double s = 0.0;
            for (k = 0; k < n; k++)
                s += model->Ca[i][k] * kf->P[k][j];
            CP[i][j] = s;
        }
    for (i = 0; i < ny; i++)
        for (j = 0; j < ny; j++) {
            double s = 0.0;
            for (k = 0; k < n; k++)
                s += CP[i][k] * model->Ca[j][k];
            CPC_R[i][j] = s + kf->R[i][j];
        }

    /* Invert S (innovation covariance) */
    memset(S_inv, 0, sizeof(S_inv));
    if (ny == 1) {
        if (fabs(CPC_R[0][0]) < 1e-12) return -1;
        S_inv[0][0] = 1.0 / CPC_R[0][0];
    } else if (ny == 2) {
        double det = CPC_R[0][0]*CPC_R[1][1] - CPC_R[0][1]*CPC_R[1][0];
        if (fabs(det) < 1e-12) return -1;
        S_inv[0][0] = CPC_R[1][1] / det;
        S_inv[0][1] = -CPC_R[0][1] / det;
        S_inv[1][0] = -CPC_R[1][0] / det;
        S_inv[1][1] = CPC_R[0][0] / det;
    } else {
        /* Gauss-Jordan */
        double aug[MPC_MAX_NY][2 * MPC_MAX_NY];
        for (i = 0; i < ny; i++) {
            for (j = 0; j < ny; j++) aug[i][j] = CPC_R[i][j];
            for (j = 0; j < ny; j++) aug[i][ny + j] = (i == j) ? 1.0 : 0.0;
        }
        for (k = 0; k < ny; k++) {
            double piv = aug[k][k];
            if (fabs(piv) < 1e-12) return -1;
            for (j = 0; j < 2 * ny; j++) aug[k][j] /= piv;
            for (i = 0; i < ny; i++) {
                if (i != k) {
                    double fac = aug[i][k];
                    for (j = 0; j < 2 * ny; j++) aug[i][j] -= fac * aug[k][j];
                }
            }
        }
        for (i = 0; i < ny; i++)
            for (j = 0; j < ny; j++)
                S_inv[i][j] = aug[i][ny + j];
    }

    /* K = P * C^T * S_inv */
    for (i = 0; i < n; i++)
        for (j = 0; j < ny; j++) {
            double s = 0.0;
            for (k = 0; k < ny; k++)
                s += CP[k][i] * S_inv[k][j];
            K_gain[i][j] = s;
            obs->kf_gain[i][j] = s;
        }

    /* Update state: x = x + K * innovation */
    for (i = 0; i < n; i++) {
        double s = 0.0;
        for (j = 0; j < ny; j++)
            s += K_gain[i][j] * obs->innovation[j];
        obs->x_aug_hat[i] += s;
    }

    /* P = (I - K*C) * P */
    for (i = 0; i < n; i++)
        for (j = 0; j < n; j++) {
            double s = 0.0;
            for (k = 0; k < ny; k++)
                s += K_gain[i][k] * model->Ca[k][j];
            KC[i][j] = s;
        }
    for (i = 0; i < n; i++)
        for (j = 0; j < n; j++) {
            double s = kf->P[i][j];
            for (k = 0; k < n; k++)
                s -= KC[i][k] * kf->P[k][j];
            P_new[i][j] = s;
        }
    for (i = 0; i < n; i++)
        for (j = 0; j < n; j++)
            kf->P[i][j] = P_new[i][j];

    return 0;
}
