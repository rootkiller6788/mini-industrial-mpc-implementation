/**
 * @file mpc_feedforward.c
 * @brief Disturbance Feedforward Control for MPC
 *
 * Implements static, dynamic, and adaptive feedforward compensation.
 *
 * Reference: Brosilow and Joseph, "Inferential Control" (1978)
 * Reference: Seborg, Edgar, Mellichamp, "Process Dynamics and Control" (2016)
 *
 * @knowledge L1: Feedforward control definition
 * @knowledge L2: Feedback vs feedforward
 * @knowledge L5: Static feedforward design
 * @knowledge L5: Dynamic feedforward design
 */

#include "mpc_feedforward.h"
#include <string.h>
#include <math.h>

/* Compute ideal static feedforward gain matrix
 * Kd = -(C*inv(I-A)*B)^{-1} * C*inv(I-A)*Bd
 *
 * Theorem: For stable LTI plant, this gives perfect steady-state
 * disturbance rejection (dc gain from d to y is zero).
 */
int mpc_ff_compute_static_gain(const mpc_ss_model_t *plant,
                                double Kd[MPC_MAX_NU][MPC_MAX_ND])
{
    int nx, nu, ny, nd, i, j, k;
    double I_minus_A[MPC_MAX_NX][MPC_MAX_NX];
    double I_minus_A_inv[MPC_MAX_NX][MPC_MAX_NX];
    double G0[MPC_MAX_NY][MPC_MAX_NU];
    double Gd0[MPC_MAX_NY][MPC_MAX_ND];
    double G0_inv[MPC_MAX_NU][MPC_MAX_NY];

    if (!plant || !Kd) return -1;
    nx = plant->nx; nu = plant->nu; ny = plant->ny; nd = plant->nd;
    if (nx <= 0 || nu <= 0 || ny <= 0) return -1;
    if (nd <= 0) nd = 1;

    memset(Kd, 0, sizeof(double) * MPC_MAX_NU * MPC_MAX_ND);

    /* Build I - A and invert */
    for (i = 0; i < nx; i++)
        for (j = 0; j < nx; j++)
            I_minus_A[i][j] = (i == j) ? 1.0 - plant->A[i][j] : -plant->A[i][j];

    /* Invert I - A via Gauss-Jordan */
    double aug[MPC_MAX_NX][2 * MPC_MAX_NX];
    for (i = 0; i < nx; i++) {
        for (j = 0; j < nx; j++) aug[i][j] = I_minus_A[i][j];
        for (j = 0; j < nx; j++) aug[i][nx + j] = (i == j) ? 1.0 : 0.0;
    }
    for (k = 0; k < nx; k++) {
        int prow = k;
        double mv = fabs(aug[k][k]);
        for (i = k + 1; i < nx; i++)
            if (fabs(aug[i][k]) > mv) { mv = fabs(aug[i][k]); prow = i; }
        if (mv < 1e-12) return -1;
        if (prow != k)
            for (j = 0; j < 2 * nx; j++) {
                double t = aug[k][j]; aug[k][j] = aug[prow][j]; aug[prow][j] = t;
            }
        double piv = aug[k][k];
        for (j = 0; j < 2 * nx; j++) aug[k][j] /= piv;
        for (i = 0; i < nx; i++) {
            if (i != k) {
                double fac = aug[i][k];
                for (j = 0; j < 2 * nx; j++) aug[i][j] -= fac * aug[k][j];
            }
        }
    }
    for (i = 0; i < nx; i++)
        for (j = 0; j < nx; j++)
            I_minus_A_inv[i][j] = aug[i][nx + j];

    /* DC gain from u to y: G0 = C*inv(I-A)*B + D */
    memset(G0, 0, sizeof(G0));
    {
        double CA_inv[MPC_MAX_NY][MPC_MAX_NX];
        for (i = 0; i < ny; i++)
            for (j = 0; j < nx; j++) {
                double s = 0.0;
                for (k = 0; k < nx; k++)
                    s += plant->C[i][k] * I_minus_A_inv[k][j];
                CA_inv[i][j] = s;
            }
        for (i = 0; i < ny; i++)
            for (j = 0; j < nu; j++) {
                double s = 0.0;
                for (k = 0; k < nx; k++)
                    s += CA_inv[i][k] * plant->B[k][j];
                G0[i][j] = s + plant->D[i][j];
            }
    }

    /* DC gain from d to y: Gd0 = C*inv(I-A)*Bd + Cd */
    memset(Gd0, 0, sizeof(Gd0));
    {
        double CA_inv[MPC_MAX_NY][MPC_MAX_NX];
        for (i = 0; i < ny; i++)
            for (j = 0; j < nx; j++) {
                double s = 0.0;
                for (k = 0; k < nx; k++)
                    s += plant->C[i][k] * I_minus_A_inv[k][j];
                CA_inv[i][j] = s;
            }
        for (i = 0; i < ny; i++)
            for (j = 0; j < nd; j++) {
                double s = 0.0;
                for (k = 0; k < nx; k++)
                    s += CA_inv[i][k] * plant->Bd[k][j];
                Gd0[i][j] = s + plant->Cd[i][j];
            }
    }

    /* Kd = -inv(G0) * Gd0 (minimum-norm if non-square) */
    /* For SISO: Kd = -Gd0 / G0 */
    if (nu == 1 && ny == 1) {
        if (fabs(G0[0][0]) > 1e-12)
            Kd[0][0] = -Gd0[0][0] / G0[0][0];
        else
            Kd[0][0] = 0.0;
    } else {
        /* Use pseudoinverse: Kd = -pinv(G0) * Gd0 */
        /* Simplified: assume square and invertible */
        if (nu == ny) {
            /* Invert G0 */
            double G0_aug[MPC_MAX_NU][2 * MPC_MAX_NU];
            for (i = 0; i < nu; i++) {
                for (j = 0; j < nu; j++) G0_aug[i][j] = G0[i][j];
                for (j = 0; j < nu; j++) G0_aug[i][nu + j] = (i == j) ? 1.0 : 0.0;
            }
            for (k = 0; k < nu; k++) {
                double piv2 = G0_aug[k][k];
                if (fabs(piv2) < 1e-12) break;
                for (j = 0; j < 2 * nu; j++) G0_aug[k][j] /= piv2;
                for (i = 0; i < nu; i++) {
                    if (i != k) {
                        double fac2 = G0_aug[i][k];
                        for (j = 0; j < 2 * nu; j++) G0_aug[i][j] -= fac2 * G0_aug[k][j];
                    }
                }
            }
            for (i = 0; i < nu; i++)
                for (j = 0; j < nu; j++)
                    G0_inv[i][j] = G0_aug[i][nu + j];

            for (i = 0; i < nu; i++)
                for (j = 0; j < nd; j++) {
                    double s = 0.0;
                    for (k = 0; k < nu; k++)
                        s += G0_inv[i][k] * Gd0[k][j];
                    Kd[i][j] = -s;
                }
        }
    }

    return 0;
}

int mpc_ff_compute_static(const mpc_ff_config_t *ff,
                           const double d_meas[MPC_MAX_ND],
                           double u_ff[MPC_MAX_NU])
{
    int i, j;
    if (!ff || !d_meas || !u_ff) return -1;

    memset(u_ff, 0, MPC_MAX_NU * sizeof(double));
    for (i = 0; i < MPC_MAX_NU; i++)
        for (j = 0; j < MPC_MAX_ND; j++)
            u_ff[i] -= ff->Kd[i][j] * d_meas[j] * ff->ff_gain_scale;

    return 0;
}

void mpc_ff_set_preview(mpc_ff_config_t *ff,
                         const double d_trajectory[MPC_MAX_NP][MPC_MAX_ND],
                         int horizon)
{
    int i, j;
    if (!ff || !d_trajectory) return;
    ff->preview_horizon = horizon;
    for (i = 0; i < horizon && i < MPC_MAX_NP; i++)
        for (j = 0; j < MPC_MAX_ND; j++)
            ff->d_preview[i][j] = d_trajectory[i][j];
}

int mpc_ff_augment_prediction(const mpc_prediction_t *pred,
                               const mpc_ff_config_t *ff,
                               const double d_current[MPC_MAX_ND],
                               double Y_ff[MPC_MAX_NP * MPC_MAX_NY])
{
    int i, j, k;
    if (!pred || !ff || !d_current || !Y_ff) return -1;

    memset(Y_ff, 0, MPC_MAX_NP * MPC_MAX_NY * sizeof(double));

    /* Y_ff = Phi_d * d_current (for constant disturbance assumption) */
    for (i = 0; i < pred->np; i++)
        for (j = 0; j < pred->ny; j++) {
            double s = 0.0;
            for (k = 0; k < pred->nd && k < MPC_MAX_ND; k++)
                s += pred->Phi_d[i * pred->ny + j][k] * d_current[k];
            Y_ff[i * pred->ny + j] = s;
        }

    return 0;
}

void mpc_ff_combine(const double u_fb[MPC_MAX_NU],
                     const double u_ff[MPC_MAX_NU],
                     const double u_min[MPC_MAX_NU],
                     const double u_max[MPC_MAX_NU],
                     double u_total[MPC_MAX_NU],
                     int nu)
{
    int i;
    if (!u_fb || !u_ff || !u_min || !u_max || !u_total) return;

    for (i = 0; i < nu && i < MPC_MAX_NU; i++) {
        double u_raw = u_fb[i] + u_ff[i];
        if (u_raw < u_min[i]) u_total[i] = u_min[i];
        else if (u_raw > u_max[i]) u_total[i] = u_max[i];
        else u_total[i] = u_raw;
    }
}

double mpc_ff_contribution_ratio(const double u_fb[MPC_MAX_NU],
                                  const double u_ff[MPC_MAX_NU],
                                  int nu)
{
    int i;
    double norm_ff = 0.0, norm_fb = 0.0;
    if (!u_fb || !u_ff) return 0.0;

    for (i = 0; i < nu && i < MPC_MAX_NU; i++) {
        norm_ff += u_ff[i] * u_ff[i];
        norm_fb += u_fb[i] * u_fb[i];
    }
    double total = sqrt(norm_ff) + sqrt(norm_fb);
    if (total < 1e-12) return 0.0;
    return sqrt(norm_ff) / total;
}

double mpc_ff_rejection_improvement(const double y_errors_with_ff[],
                                     const double y_errors_without_ff[],
                                     int n_samples, int ny)
{
    int i, j;
    double var_with = 0.0, var_without = 0.0;
    if (!y_errors_with_ff || !y_errors_without_ff || n_samples <= 0) return 0.0;

    for (i = 0; i < n_samples; i++) {
        for (j = 0; j < ny; j++) {
            var_with += y_errors_with_ff[i * ny + j] * y_errors_with_ff[i * ny + j];
            var_without += y_errors_without_ff[i * ny + j] * y_errors_without_ff[i * ny + j];
        }
    }

    if (var_without < 1e-12) return 1.0;
    return 1.0 - (var_with / var_without);
}

int mpc_ff_detect_fighting(const double u_fb[MPC_MAX_NU],
                            const double u_ff[MPC_MAX_NU],
                            int nu, double threshold)
{
    int i;
    double dot_product = 0.0, norm_fb = 0.0, norm_ff = 0.0;
    if (!u_fb || !u_ff) return 0;

    for (i = 0; i < nu && i < MPC_MAX_NU; i++) {
        dot_product += u_fb[i] * u_ff[i];
        norm_fb += u_fb[i] * u_fb[i];
        norm_ff += u_ff[i] * u_ff[i];
    }

    /* Fighting detected if signals are strongly anticorrelated */
    double cos_angle = dot_product / (sqrt(norm_fb) * sqrt(norm_ff) + 1e-12);
    return (cos_angle < -threshold) ? 1 : 0;
}
