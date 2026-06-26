/**
 * mpc_kalman_filter.c — Kalman Filter for MPC State Estimation
 *
 * Implements Kalman filtering for integrating process states and
 * disturbance estimation. Essential for offset-free MPC.
 *
 * The Kalman filter provides optimal state estimation under Gaussian
 * noise assumptions. For MPC, we augment the state with an integrating
 * output disturbance to achieve offset-free tracking.
 *
 * Knowledge Coverage:
 *   L5 - Algorithms: Kalman filter, DARE solution, disturbance estimation
 *   L4 - Theorems: detectability, Joseph form covariance, Riccati convergence
 *   L3 - Eng. Structures: Augmented observer for MPC
 *
 * Reference: Muske & Badgwell (2002), Pannocchia & Rawlings (2003)
 *            Simon (2006) "Optimal State Estimation"
 */

#include <math.h>
#include <string.h>
#include <stdio.h>
#include "../include/mpc_kalman_filter.h"

/* ─── Lifecycle ───────────────────────────────────────────────────────── */

void kalman_init(kalman_config_t *kf, const double *A, const double *B,
                  const double *C, int n, double q_scale, double r_meas) {
    if (!kf || !A || !B || !C || n < 1 || n > KALMAN_MAX_STATES) return;

    memset(kf, 0, sizeof(*kf));
    kf->n_states = n;
    kf->n_outputs = 1;

    /* Copy A, B, C */
    memcpy(kf->A, A, (size_t)(n * n) * sizeof(double));
    memcpy(kf->B, B, (size_t)n * sizeof(double));
    memcpy(kf->C, C, (size_t)n * sizeof(double));

    /* Initialize covariance P_post = Q_w (large uncertainty) */
    for (int i = 0; i < n; i++) {
        kf->Q_w[i * n + i] = q_scale;
        kf->P_post[i * n + i] = q_scale;
    }
    kf->R_v = r_meas;

    /* Initialize state estimate to zero */
    memset(kf->x_post, 0, sizeof(kf->x_post));
}

/* ─── Predict-Update Cycle ────────────────────────────────────────────── */

void kalman_predict(kalman_config_t *kf, double u) {
    if (!kf || kf->n_states < 1) return;

    int n = kf->n_states;

    /* x_prior = A * x_post + B * u */
    for (int i = 0; i < n; i++) {
        double sum = 0.0;
        for (int j = 0; j < n; j++) {
            sum += kf->A[i * n + j] * kf->x_post[j];
        }
        kf->x_prior[i] = sum + kf->B[i] * u;
    }

    /* P_prior = A * P_post * A^T + Q_w */
    double temp[KALMAN_MAX_STATES * KALMAN_MAX_STATES] = {0};

    /* temp = A * P_post */
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            for (int k = 0; k < n; k++)
                temp[i * n + j] += kf->A[i * n + k] * kf->P_post[k * n + j];

    /* P_prior = temp * A^T */
    memset(kf->P_prior, 0, sizeof(kf->P_prior));
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            for (int k = 0; k < n; k++)
                kf->P_prior[i * n + j] += temp[i * n + k] * kf->A[j * n + k];

    /* Add Q_w */
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            kf->P_prior[i * n + j] += kf->Q_w[i * n + j];
}

void kalman_update(kalman_config_t *kf, double y_meas) {
    if (!kf || kf->n_states < 1) return;

    int n = kf->n_states;

    /* Innovation covariance: S = C * P_prior * C^T + R_v */
    double S = 0.0;
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            S += kf->C[i] * kf->P_prior[i * n + j] * kf->C[j];
    S += kf->R_v;

    if (S <= 0.0) {
        /* Degenerate: measurement exactly matches prediction */
        memcpy(kf->x_post, kf->x_prior, (size_t)n * sizeof(double));
        memcpy(kf->P_post, kf->P_prior, (size_t)(n * n) * sizeof(double));
        memset(kf->K_gain, 0, sizeof(kf->K_gain));
        return;
    }

    /* Kalman gain: K = P_prior * C^T / S */
    for (int i = 0; i < n; i++) {
        kf->K_gain[i] = 0.0;
        for (int j = 0; j < n; j++) {
            kf->K_gain[i] += kf->P_prior[i * n + j] * kf->C[j];
        }
        kf->K_gain[i] /= S;
    }

    /* Innovation: e = y - C*x_prior */
    double y_pred = 0.0;
    for (int j = 0; j < n; j++) {
        y_pred += kf->C[j] * kf->x_prior[j];
    }
    double e = y_meas - y_pred;

    /* State update: x_post = x_prior + K * e */
    for (int i = 0; i < n; i++) {
        kf->x_post[i] = kf->x_prior[i] + kf->K_gain[i] * e;
    }

    /* Covariance update (Joseph form):
     * P_post = (I - K*C) * P_prior * (I - K*C)^T + K * R_v * K^T
     *
     * Joseph form guarantees symmetry and PSD even under roundoff.
     */
    double I_KCl[KALMAN_MAX_STATES * KALMAN_MAX_STATES];

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            I_KCl[i * n + j] = (i == j) ? 1.0 - kf->K_gain[i] * kf->C[j]
                                        : -kf->K_gain[i] * kf->C[j];
        }
    }

    /* temp = (I-KC) * P_prior */
    double temp[KALMAN_MAX_STATES * KALMAN_MAX_STATES] = {0};
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            for (int k = 0; k < n; k++)
                temp[i * n + j] += I_KCl[i * n + k] * kf->P_prior[k * n + j];

    /* P_post = temp * (I-KC)^T */
    memset(kf->P_post, 0, sizeof(kf->P_post));
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            for (int k = 0; k < n; k++)
                kf->P_post[i * n + j] += temp[i * n + k] * I_KCl[j * n + k];

    /* Add K * R_v * K^T */
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            kf->P_post[i * n + j] += kf->K_gain[i] * kf->R_v * kf->K_gain[j];
}

void kalman_step(kalman_config_t *kf, double u, double y_meas) {
    kalman_predict(kf, u);
    kalman_update(kf, y_meas);
}

/* ─── Disturbance Augmentation ────────────────────────────────────────── */

int kalman_augment_disturbance(kalman_config_t *kf, const double *B_d,
                                int n_states, int n_dist) {
    if (!kf || !B_d || n_states < 1 || n_dist < 1) return -1;
    if (n_states + n_dist > KALMAN_MAX_STATES) return -1;

    int n_aug = n_states + n_dist;

    /* Save original matrices */
    double A_orig[KALMAN_MAX_STATES * KALMAN_MAX_STATES];
    double B_orig[KALMAN_MAX_STATES];
    double C_orig[KALMAN_MAX_STATES];
    memcpy(A_orig, kf->A, (size_t)(n_states * n_states) * sizeof(double));
    memcpy(B_orig, kf->B, (size_t)n_states * sizeof(double));
    memcpy(C_orig, kf->C, (size_t)n_states * sizeof(double));

    /* Build augmented A = [A  B_d; 0  I] */
    memset(kf->A, 0, sizeof(kf->A));
    for (int i = 0; i < n_states; i++) {
        for (int j = 0; j < n_states; j++) {
            kf->A[i * n_aug + j] = A_orig[i * n_states + j];
        }
        kf->A[i * n_aug + n_states] = B_d[i]; /* B_d in top-right */
    }
    for (int i = 0; i < n_dist; i++) {
        kf->A[(n_states + i) * n_aug + (n_states + i)] = 1.0;
    }

    /* Augmented B = [B; 0] */
    memset(kf->B, 0, sizeof(kf->B));
    memcpy(kf->B, B_orig, (size_t)n_states * sizeof(double));

    /* Augmented C = [C  I_dist] */
    memset(kf->C, 0, sizeof(kf->C));
    memcpy(kf->C, C_orig, (size_t)n_states * sizeof(double));
    for (int i = 0; i < n_dist; i++) {
        kf->C[n_states + i] = 1.0;
    }

    /* Update dimensions */
    kf->n_states = n_aug;

    /* Reset filter to match new dimension */
    for (int i = 0; i < n_aug; i++) {
        kf->Q_w[i * n_aug + i] = kf->Q_w[0]; /* reuse scale */
        kf->P_post[i * n_aug + i] = kf->Q_w[i * n_aug + i];
    }
    memset(kf->x_post, 0, sizeof(kf->x_post));

    return n_aug;
}

int kalman_disturbance_estimate(const kalman_config_t *kf, double *d) {
    if (!kf || !d) return -1;
    if (kf->n_states < 1) return -1;

    /* Last state is the disturbance */
    *d = kf->x_post[kf->n_states - 1];
    return 0;
}

/* ─── Steady-State Gain ───────────────────────────────────────────────── */

int kalman_steady_state_gain(kalman_config_t *kf, double tol, int max_iter) {
    if (!kf || kf->n_states < 1) return -1;

    int n = kf->n_states;
    int iter;
    double P[KALMAN_MAX_STATES * KALMAN_MAX_STATES];
    double P_next[KALMAN_MAX_STATES * KALMAN_MAX_STATES];

    memcpy(P, kf->P_post, (size_t)(n * n) * sizeof(double));

    for (iter = 0; iter < max_iter; iter++) {
        /* P_next = A*P*A^T + Q - A*P*C^T*(C*P*C^T+R)^{-1}*C*P*A^T */

        /* S = C*P*C^T + R */
        double S = 0.0;
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                S += kf->C[i] * P[i * n + j] * kf->C[j];
        S += kf->R_v;

        if (S <= 0.0) break;

        /* K_temp = P*C^T / S */
        double K_temp[KALMAN_MAX_STATES];
        for (int i = 0; i < n; i++) {
            K_temp[i] = 0.0;
            for (int j = 0; j < n; j++)
                K_temp[i] += P[i * n + j] * kf->C[j];
            K_temp[i] /= S;
        }

        /* temp1 = A*P */
        double temp1[KALMAN_MAX_STATES * KALMAN_MAX_STATES] = {0};
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                for (int k = 0; k < n; k++)
                    temp1[i * n + j] += kf->A[i * n + k] * P[k * n + j];

        /* temp2 = temp1 * A^T */
        double temp2[KALMAN_MAX_STATES * KALMAN_MAX_STATES] = {0};
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                for (int k = 0; k < n; k++)
                    temp2[i * n + j] += temp1[i * n + k] * kf->A[j * n + k];

        /* Correction term: temp1*C^T * S^{-1} * C * temp1^T */
        double corr[KALMAN_MAX_STATES * KALMAN_MAX_STATES] = {0};
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                corr[i * n + j] = K_temp[i] * S * K_temp[j];

        /* P_next = temp2 - corr + Q_w */
        double max_diff = 0.0;
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                P_next[i * n + j] = temp2[i * n + j] - corr[i * n + j]
                                    + kf->Q_w[i * n + j];
                double diff = fabs(P_next[i * n + j] - P[i * n + j]);
                if (diff > max_diff) max_diff = diff;
            }
        }

        memcpy(P, P_next, (size_t)(n * n) * sizeof(double));

        if (max_diff < tol) break;
    }

    /* Compute steady-state gain from P_ss */
    double S_ss = 0.0;
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            S_ss += kf->C[i] * P[i * n + j] * kf->C[j];
    S_ss += kf->R_v;

    if (S_ss > 0.0) {
        for (int i = 0; i < n; i++) {
            kf->K_gain[i] = 0.0;
            for (int j = 0; j < n; j++)
                kf->K_gain[i] += P[i * n + j] * kf->C[j];
            kf->K_gain[i] /= S_ss;
        }
    }

    memcpy(kf->P_post, P, (size_t)(n * n) * sizeof(double));

    return (iter < max_iter) ? iter : -1;
}

/* ─── Innovation Monitoring ───────────────────────────────────────────── */

void kalman_innovation_stats(const kalman_config_t *kf, double y_meas,
                              double *e_mean, double *e_var, long *count) {
    if (!kf || !e_mean || !e_var || !count) return;

    /* Current innovation */
    double y_pred = 0.0;
    for (int j = 0; j < kf->n_states; j++) {
        y_pred += kf->C[j] * kf->x_prior[j];
    }
    double e = y_meas - y_pred;

    /* Welford online mean/variance update */
    long n = *count + 1;
    double delta = e - *e_mean;
    *e_mean += delta / (double)n;
    double delta2 = e - *e_mean;
    *e_var = ((double)(n - 1) * (*e_var) + delta * delta2) / (double)n;
    *count = n;
}

double kalman_whiteness_test(const double *innovations, int n, int L) {
    if (!innovations || n < L + 2 || L < 1) return 0.0;

    /* Compute mean */
    double mean = 0.0;
    for (int i = 0; i < n; i++) mean += innovations[i];
    mean /= (double)n;

    /* Compute autocorrelation up to lag L */
    double Q = 0.0;
    for (int k = 1; k <= L; k++) {
        double num = 0.0, den = 0.0;
        for (int i = 0; i < n - k; i++) {
            num += (innovations[i] - mean) * (innovations[i + k] - mean);
        }
        for (int i = 0; i < n; i++) {
            den += (innovations[i] - mean) * (innovations[i] - mean);
        }
        double r_k = (den > 0.0) ? num / den : 0.0;
        Q += (r_k * r_k) / (double)(n - k);
    }
    Q *= (double)n * (double)(n + 2);

    return Q;
}

/* ─── Accessors ───────────────────────────────────────────────────────── */

void kalman_get_state(const kalman_config_t *kf, double *x) {
    if (!kf || !x) return;
    memcpy(x, kf->x_post, (size_t)(kf->n_states) * sizeof(double));
}

void kalman_get_covariance(const kalman_config_t *kf, double *P) {
    if (!kf || !P) return;
    int n = kf->n_states;
    memcpy(P, kf->P_post, (size_t)(n * n) * sizeof(double));
}
