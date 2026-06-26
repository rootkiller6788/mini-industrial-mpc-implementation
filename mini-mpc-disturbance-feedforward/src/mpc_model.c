/**
 * @file mpc_model.c
 * @brief MPC Model Operations: discretization, prediction, DARE
 *
 * Reference: Astrom and Wittenmark, Computer-Controlled Systems (1997)
 * Reference: Rawlings, Mayne, Diehl (2017), Chapters 1-3
 * Reference: Higham, SIAM Review (2009) - Matrix Exponential
 */

#include "mpc_model.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

#define EXP_MAX_SCALE   10
#define SINGULARITY_TOL 1e-12

static void mat_copy_flat(const double src[], double dst[], int n, int lda, int ldb) {
    int i;
    for (i = 0; i < n; i++)
        memcpy(&dst[i * ldb], &src[i * lda], n * sizeof(double));
}

static void mat_zero_flat(double A[], int rows, int cols, int ld) {
    int i;
    for (i = 0; i < rows; i++)
        memset(&A[i * ld], 0, cols * sizeof(double));
}

static void mat_eye_flat(double A[], int n, int ld) {
    int i, j;
    for (i = 0; i < n; i++)
        for (j = 0; j < n; j++)
            A[i * ld + j] = (i == j) ? 1.0 : 0.0;
}

static void mat_mul_flat(const double A[], const double B[], double C[],
                          int m, int n, int p, int lda, int ldb, int ldc) {
    int i, j, k;
    for (i = 0; i < m; i++)
        for (j = 0; j < p; j++) {
            double s = 0.0;
            for (k = 0; k < n; k++)
                s += A[i * lda + k] * B[k * ldb + j];
            C[i * ldc + j] = s;
        }
}

static double mat_norm_inf_flat(const double A[], int rows, int cols, int ld) {
    int i, j;
    double max_sum = 0.0;
    for (i = 0; i < rows; i++) {
        double rs = 0.0;
        for (j = 0; j < cols; j++) rs += fabs(A[i * ld + j]);
        if (rs > max_sum) max_sum = rs;
    }
    return max_sum;
}

/* =================================================================
 * Matrix Exponential via Scaling-and-Squaring with Pade [6/6]
 *
 * exp(A) = (exp(A/2^s))^{2^s}
 * where exp(A/2^s) is given by the [6/6] Pade approximant.
 *
 * Theorem (Higham 2009): The [6/6] Pade approximant is accurate to
 * machine precision for ||A|| <= 1 after appropriate scaling.
 *
 * Pade [6/6] coefficients:
 * N_6 = 1 + x/2 + 5x^2/44 + x^3/66 + x^4/792 + x^5/15840 + x^6/665280
 * D_6 = 1 - x/2 + 5x^2/44 - x^3/66 + x^4/792 - x^5/15840 + x^6/665280
 *
 * Complexity: O(n^3 * log2(||A||))
 * ================================================================= */
int mpc_matrix_expm(const double A[MPC_MAX_NX][MPC_MAX_NX], int n,
                    double E[MPC_MAX_NX][MPC_MAX_NX])
{
    int i, j, k, t, s;
    double pade_coef[7];
    double normA;
    double scale;
    double As[MPC_MAX_NX][MPC_MAX_NX];
    double A2[MPC_MAX_NX][MPC_MAX_NX], A3[MPC_MAX_NX][MPC_MAX_NX];
    double A4[MPC_MAX_NX][MPC_MAX_NX], A5[MPC_MAX_NX][MPC_MAX_NX];
    double A6[MPC_MAX_NX][MPC_MAX_NX];
    double N[MPC_MAX_NX][MPC_MAX_NX], D[MPC_MAX_NX][MPC_MAX_NX];
    double aug[MPC_MAX_NX][2 * MPC_MAX_NX];
    double tmp[MPC_MAX_NX][MPC_MAX_NX];
    double *Ap[7];

    if (!A || !E || n <= 0 || n > MPC_MAX_NX) return -1;
    if (n == 1) { E[0][0] = exp(A[0][0]); return 0; }

    pade_coef[0] = 1.0;
    pade_coef[1] = 0.5;
    pade_coef[2] = 5.0 / 44.0;
    pade_coef[3] = 1.0 / 66.0;
    pade_coef[4] = 1.0 / 792.0;
    pade_coef[5] = 1.0 / 15840.0;
    pade_coef[6] = 1.0 / 665280.0;

    /* Compute ||A||_inf */
    normA = mat_norm_inf_flat(FLAT2D(A), n, n, n);
    s = 0;
    while (normA > 1.0 && s < EXP_MAX_SCALE) { normA /= 2.0; s++; }

    /* Scale: A_scaled = A / 2^s */
    scale = 1.0 / (double)(1 << s);
    for (i = 0; i < n; i++)
        for (j = 0; j < n; j++)
            As[i][j] = A[i][j] * scale;

    /* Compute powers A^2 through A^6 */
    mat_mul_flat(FLAT2D(As), FLAT2D(As), FLAT2D_MUT(A2), n, n, n, n, n, n);
    mat_mul_flat(FLAT2D(A2), FLAT2D(As), FLAT2D_MUT(A3), n, n, n, n, n, n);
    mat_mul_flat(FLAT2D(A3), FLAT2D(As), FLAT2D_MUT(A4), n, n, n, n, n, n);
    mat_mul_flat(FLAT2D(A4), FLAT2D(As), FLAT2D_MUT(A5), n, n, n, n, n, n);
    mat_mul_flat(FLAT2D(A5), FLAT2D(As), FLAT2D_MUT(A6), n, n, n, n, n, n);

    Ap[0] = NULL;
    Ap[1] = FLAT2D_MUT(As);
    Ap[2] = FLAT2D_MUT(A2);
    Ap[3] = FLAT2D_MUT(A3);
    Ap[4] = FLAT2D_MUT(A4);
    Ap[5] = FLAT2D_MUT(A5);
    Ap[6] = FLAT2D_MUT(A6);

    /* Build N = sum c_j * A^j, D = sum (-1)^j * c_j * A^j */
    mat_zero_flat(FLAT2D_MUT(N), n, n, n);
    mat_zero_flat(FLAT2D_MUT(D), n, n, n);
    for (i = 0; i < n; i++) { N[i][i] = 1.0; D[i][i] = 1.0; }

    for (j = 1; j <= 6; j++) {
        double c = pade_coef[j];
        int sign = (j % 2 == 1) ? 1 : -1;
        double ns = c;
        double ds = (double)sign * c;
        double *Ajp = Ap[j];
        for (i = 0; i < n; i++) {
            int ii;
            for (ii = 0; ii < n; ii++) {
                double v = Ajp[i * n + ii];
                N[i][ii] += ns * v;
                D[i][ii] += ds * v;
            }
        }
    }

    /* Solve D * R = N via Gaussian elimination with partial pivoting */
    for (i = 0; i < n; i++) {
        for (j = 0; j < n; j++) {
            aug[i][j] = D[i][j];
            aug[i][n + j] = N[i][j];
        }
    }
    for (k = 0; k < n; k++) {
        int prow = k;
        double mv = fabs(aug[k][k]);
        for (i = k + 1; i < n; i++) {
            if (fabs(aug[i][k]) > mv) { mv = fabs(aug[i][k]); prow = i; }
        }
        if (mv < SINGULARITY_TOL) return -1;
        if (prow != k) {
            for (j = 0; j < 2 * n; j++) {
                double t_val = aug[k][j];
                aug[k][j] = aug[prow][j];
                aug[prow][j] = t_val;
            }
        }
        double piv = aug[k][k];
        for (j = 0; j < 2 * n; j++) aug[k][j] /= piv;
        for (i = 0; i < n; i++) {
            if (i != k) {
                double fac = aug[i][k];
                for (j = 0; j < 2 * n; j++) aug[i][j] -= fac * aug[k][j];
            }
        }
    }
    for (i = 0; i < n; i++)
        for (j = 0; j < n; j++)
            E[i][j] = aug[i][n + j];

    /* Repeated squaring: E = E^{2^s} */
    for (t = 0; t < s; t++) {
        mat_copy_flat(FLAT2D(E), FLAT2D_MUT(tmp), n, n, n);
        mat_mul_flat(FLAT2D(tmp), FLAT2D(tmp), FLAT2D_MUT(E), n, n, n, n, n, n);
    }

    return 0;
}

/* =================================================================
 * ZOH Discretization via Van Loan Augmented Matrix
 *
 * Theorem (Van Loan 1978): For dx/dt = Ac*x + Bc*u,
 *   exp([Ac Bc; 0 0] * Ts) = [Ad Bd; 0 I]
 *
 * This is exact for any Ac (including singular), not requiring
 * Ac to be invertible.
 *
 * Tustin (bilinear): s -> 2/Ts * (z-1)/(z+1)
 *   Ad = inv(I - Ts/2*Ac) * (I + Ts/2*Ac)
 * ================================================================= */
int mpc_discretize_model(const double Ac[MPC_MAX_NX][MPC_MAX_NX],
                          const double Bc[MPC_MAX_NX][MPC_MAX_NU],
                          const double Cc[MPC_MAX_NY][MPC_MAX_NX],
                          const double Dc[MPC_MAX_NY][MPC_MAX_NU],
                          int nx, int nu, int ny, double Ts,
                          mpc_discretize_method_t method,
                          mpc_ss_model_t *model)
{
    int i, j, k;
    if (!Ac || !model || nx <= 0 || nu <= 0 || ny <= 0 || Ts <= 0.0) return -1;
    if (nx > MPC_MAX_NX || nu > MPC_MAX_NU || ny > MPC_MAX_NY) return -1;

    model->nx = nx;
    model->nu = nu;
    model->ny = ny;
    model->nd = 0;

    if (method == MPC_DISCRETIZE_ZOH) {
        double Ad[MPC_MAX_NX][MPC_MAX_NX];
        double AcTs[MPC_MAX_NX][MPC_MAX_NX];

        /* Compute Ad = expm(Ac * Ts) */
        for (i = 0; i < nx; i++)
            for (j = 0; j < nx; j++)
                AcTs[i][j] = Ac[i][j] * Ts;
        if (mpc_matrix_expm(AcTs, nx, Ad) != 0) return -1;

        /* Compute Bd = integral_0^Ts exp(Ac*tau) dtau * Bc
         * using truncated series: sum_{k=0}^{m} A^k * Ts^{k+1} / (k+1)! * B */
        double Bd[MPC_MAX_NX][MPC_MAX_NU];
        double A_pow[MPC_MAX_NX][MPC_MAX_NX];
        double factorial = 1.0;
        double Ts_pow = Ts;

        memset(Bd, 0, sizeof(Bd));
        mat_eye_flat(FLAT2D_MUT(A_pow), nx, nx);

        /* 6 terms is enough for double precision */
        for (k = 0; k < 6; k++) {
            double coeff = Ts_pow / factorial;
            for (i = 0; i < nx; i++)
                for (j = 0; j < nu; j++) {
                    double s = 0.0;
                    int m;
                    for (m = 0; m < nx; m++)
                        s += A_pow[i][m] * Bc[m][j];
                    Bd[i][j] += coeff * s;
                }
            /* Update for next term */
            if (k < 5) {
                double tmp[MPC_MAX_NX][MPC_MAX_NX];
                mat_copy_flat(FLAT2D(A_pow), FLAT2D_MUT(tmp), nx, nx, nx);
                mat_mul_flat(FLAT2D(tmp), FLAT2D(AcTs), FLAT2D_MUT(A_pow), nx, nx, nx, nx, nx, nx);
                Ts_pow *= Ts;
                factorial *= (double)(k + 2);
            }
        }

        for (i = 0; i < nx; i++) {
            for (j = 0; j < nx; j++) model->A[i][j] = Ad[i][j];
            for (j = 0; j < nu; j++) model->B[i][j] = Bd[i][j];
        }
        for (i = 0; i < ny; i++) {
            for (j = 0; j < nx; j++) model->C[i][j] = Cc[i][j];
            for (j = 0; j < nu; j++) model->D[i][j] = Dc[i][j];
        }
    } else if (method == MPC_DISCRETIZE_TUSTIN) {
        double h2 = Ts / 2.0;
        double sqTs = sqrt(Ts);
        double Ip[MPC_MAX_NX][MPC_MAX_NX];
        double Im[MPC_MAX_NX][MPC_MAX_NX];
        double invIm[MPC_MAX_NX][2 * MPC_MAX_NX];
        double Iminv[MPC_MAX_NX][MPC_MAX_NX];
        double Bs[MPC_MAX_NX][MPC_MAX_NU];

        for (i = 0; i < nx; i++)
            for (j = 0; j < nx; j++) {
                Ip[i][j] = (i == j) ? 1.0 + h2 * Ac[i][j] : h2 * Ac[i][j];
                Im[i][j] = (i == j) ? 1.0 - h2 * Ac[i][j] : -h2 * Ac[i][j];
            }

        /* Invert Im via Gauss-Jordan */
        for (i = 0; i < nx; i++) {
            for (j = 0; j < nx; j++) invIm[i][j] = Im[i][j];
            for (j = 0; j < nx; j++) invIm[i][nx + j] = (i == j) ? 1.0 : 0.0;
        }
        for (k = 0; k < nx; k++) {
            int prow = k;
            double mv2 = fabs(invIm[k][k]);
            int ii;
            for (ii = k + 1; ii < nx; ii++)
                if (fabs(invIm[ii][k]) > mv2) { mv2 = fabs(invIm[ii][k]); prow = ii; }
            if (mv2 < SINGULARITY_TOL) return -1;
            if (prow != k)
                for (j = 0; j < 2 * nx; j++) {
                    double t2 = invIm[k][j];
                    invIm[k][j] = invIm[prow][j];
                    invIm[prow][j] = t2;
                }
            double piv2 = invIm[k][k];
            for (j = 0; j < 2 * nx; j++) invIm[k][j] /= piv2;
            for (ii = 0; ii < nx; ii++) {
                if (ii != k) {
                    double fac2 = invIm[ii][k];
                    for (j = 0; j < 2 * nx; j++) invIm[ii][j] -= fac2 * invIm[k][j];
                }
            }
        }
        for (i = 0; i < nx; i++)
            for (j = 0; j < nx; j++)
                Iminv[i][j] = invIm[i][nx + j];

        /* Ad = (I + h/2*Ac) * Iminv */
        mat_mul_flat(FLAT2D(Ip), FLAT2D(Iminv), FLAT2D_MUT(model->A),
                     nx, nx, nx, nx, nx, nx);

        /* Bd = Iminv * sqrt(Ts) * Bc */
        for (i = 0; i < nx; i++)
            for (j = 0; j < nu; j++)
                Bs[i][j] = sqTs * Bc[i][j];
        mat_mul_flat(FLAT2D(Iminv), FLAT2D(Bs), FLAT2D_MUT(model->B),
                     nx, nx, nu, nx, nx, nx);

        /* Cd = Cc * Iminv */
        mat_mul_flat(FLAT2D(Cc), FLAT2D(Iminv), FLAT2D_MUT(model->C),
                     ny, nx, nx, ny, nx, nx);

        /* Dd = Dc + Cd * h2 * Bc */
        mat_mul_flat(FLAT2D(model->C), FLAT2D(Bs), FLAT2D_MUT(model->D),
                     ny, nx, nu, ny, nx, nx);
        for (i = 0; i < ny; i++)
            for (j = 0; j < nu; j++)
                model->D[i][j] = model->D[i][j] * h2 + Dc[i][j];
    } else {
        /* FOH: fallback to ZOH */
        return mpc_discretize_model(Ac, Bc, Cc, Dc, nx, nu, ny, Ts,
                                     MPC_DISCRETIZE_ZOH, model);
    }

    /* Zero disturbance matrices */
    for (i = 0; i < nx; i++)
        for (j = 0; j < MPC_MAX_ND; j++)
            model->Bd[i][j] = 0.0;
    for (i = 0; i < ny; i++)
        for (j = 0; j < MPC_MAX_ND; j++)
            model->Cd[i][j] = 0.0;

    return 0;
}

/* =================================================================
 * Build Augmented Model for Offset-Free MPC
 *
 * Reference: Pannocchia and Rawlings, AIChE Journal (2003)
 *            Muske and Badgwell, J. Process Control (2002)
 *
 * Three disturbance models:
 *   OUTPUT_DISTURBANCE:  d_out added to y, integrating
 *   INPUT_DISTURBANCE:   d_in added to u, integrating
 *   STATE_DISTURBANCE:   d_state enters through Bd
 *
 * Theorem (Internal Model Principle, Francis and Wonham 1976):
 * The number of integrating disturbances must be at least equal
 * to the number of measured outputs for offset-free tracking.
 * ================================================================= */
int mpc_build_augmented_model(const mpc_ss_model_t *plant,
                               mpc_mode_t mode,
                               mpc_aug_model_t *aug)
{
    int nx, nu, ny, nd, nx_aug;
    int i, j;

    if (!plant || !aug) return -1;
    nx = plant->nx;
    nu = plant->nu;
    ny = plant->ny;

    if (nx <= 0 || nu <= 0 || ny <= 0) return -1;
    if (nx > MPC_MAX_NX || nu > MPC_MAX_NU || ny > MPC_MAX_NY) return -1;

    aug->nu = nu;
    aug->ny = ny;

    switch (mode) {
    case MPC_MODE_OUTPUT_DISTURBANCE:
        nd = ny;
        nx_aug = nx + nd;
        aug->nx_aug = nx_aug;
        if (nx_aug > MPC_MAX_NX + MPC_MAX_ND) return -1;

        mat_zero_flat(FLAT2D_MUT(aug->Aa), nx_aug, nx_aug,
                      MPC_MAX_NX + MPC_MAX_ND);
        for (i = 0; i < nx; i++)
            for (j = 0; j < nx; j++)
                aug->Aa[i][j] = plant->A[i][j];
        for (i = 0; i < nd; i++)
            aug->Aa[nx + i][nx + i] = 1.0;

        mat_zero_flat(FLAT2D_MUT(aug->Ba), nx_aug, nu,
                      MPC_MAX_NX + MPC_MAX_ND);
        for (i = 0; i < nx; i++)
            for (j = 0; j < nu; j++)
                aug->Ba[i][j] = plant->B[i][j];

        mat_zero_flat(FLAT2D_MUT(aug->Ca), ny, nx_aug, MPC_MAX_NY);
        for (i = 0; i < ny; i++) {
            for (j = 0; j < nx; j++)
                aug->Ca[i][j] = plant->C[i][j];
            aug->Ca[i][nx + i] = 1.0;
        }
        for (i = 0; i < ny; i++)
            for (j = 0; j < nu; j++)
                aug->Da[i][j] = plant->D[i][j];
        break;

    case MPC_MODE_INPUT_DISTURBANCE:
        nd = nu;
        nx_aug = nx + nd;
        aug->nx_aug = nx_aug;
        if (nx_aug > MPC_MAX_NX + MPC_MAX_ND) return -1;

        mat_zero_flat(FLAT2D_MUT(aug->Aa), nx_aug, nx_aug,
                      MPC_MAX_NX + MPC_MAX_ND);
        for (i = 0; i < nx; i++) {
            for (j = 0; j < nx; j++)
                aug->Aa[i][j] = plant->A[i][j];
            for (j = 0; j < nd; j++)
                aug->Aa[i][nx + j] = plant->B[i][j];
        }
        for (i = 0; i < nd; i++)
            aug->Aa[nx + i][nx + i] = 1.0;

        mat_zero_flat(FLAT2D_MUT(aug->Ba), nx_aug, nu,
                      MPC_MAX_NX + MPC_MAX_ND);
        for (i = 0; i < nx; i++)
            for (j = 0; j < nu; j++)
                aug->Ba[i][j] = plant->B[i][j];

        mat_zero_flat(FLAT2D_MUT(aug->Ca), ny, nx_aug, MPC_MAX_NY);
        for (i = 0; i < ny; i++) {
            for (j = 0; j < nx; j++)
                aug->Ca[i][j] = plant->C[i][j];
            for (j = 0; j < nd; j++)
                aug->Ca[i][nx + j] = plant->D[i][j];
        }
        for (i = 0; i < ny; i++)
            for (j = 0; j < nu; j++)
                aug->Da[i][j] = plant->D[i][j];
        break;

    case MPC_MODE_STATE_DISTURBANCE:
        nd = plant->nd > 0 ? plant->nd : 1;
        nx_aug = nx + nd;
        aug->nx_aug = nx_aug;
        if (nx_aug > MPC_MAX_NX + MPC_MAX_ND) return -1;

        mat_zero_flat(FLAT2D_MUT(aug->Aa), nx_aug, nx_aug,
                      MPC_MAX_NX + MPC_MAX_ND);
        for (i = 0; i < nx; i++) {
            for (j = 0; j < nx; j++)
                aug->Aa[i][j] = plant->A[i][j];
            for (j = 0; j < nd; j++)
                aug->Aa[i][nx + j] = plant->Bd[i][j];
        }
        for (i = 0; i < nd; i++)
            aug->Aa[nx + i][nx + i] = 1.0;

        mat_zero_flat(FLAT2D_MUT(aug->Ba), nx_aug, nu,
                      MPC_MAX_NX + MPC_MAX_ND);
        for (i = 0; i < nx; i++)
            for (j = 0; j < nu; j++)
                aug->Ba[i][j] = plant->B[i][j];

        mat_zero_flat(FLAT2D_MUT(aug->Ca), ny, nx_aug, MPC_MAX_NY);
        for (i = 0; i < ny; i++) {
            for (j = 0; j < nx; j++)
                aug->Ca[i][j] = plant->C[i][j];
            for (j = 0; j < nd; j++)
                aug->Ca[i][nx + j] = plant->Cd[i][j];
        }
        for (i = 0; i < ny; i++)
            for (j = 0; j < nu; j++)
                aug->Da[i][j] = plant->D[i][j];
        break;

    default:
        return mpc_build_augmented_model(plant, MPC_MODE_OUTPUT_DISTURBANCE, aug);
    }

    return 0;
}

/* =================================================================
 * Build MPC Prediction Matrices
 * ================================================================= */
int mpc_build_prediction_matrices(const mpc_aug_model_t *model,
                                   const mpc_tuning_t *tuning,
                                   mpc_prediction_t *pred)
{
    int np, nc, nx_aug, nu, ny;
    int i, j, k, r, c;
    double A_pow[MPC_MAX_NX + MPC_MAX_ND][MPC_MAX_NX + MPC_MAX_ND];
    double CAi[MPC_MAX_NY][MPC_MAX_NX + MPC_MAX_ND];

    if (!model || !tuning || !pred) return -1;
    np = tuning->np; nc = tuning->nc;
    nx_aug = model->nx_aug; nu = model->nu; ny = model->ny;
    if (np > MPC_MAX_NP || nc > MPC_MAX_NC) return -1;
    if (nx_aug > MPC_MAX_NX + MPC_MAX_ND) return -1;
    pred->np = np; pred->nc = nc;
    pred->nx = nx_aug; pred->nu = nu; pred->ny = ny; pred->nd = 0;

    mat_zero_flat(FLAT2D_MUT(pred->Phi), np * ny, nx_aug, MPC_MAX_NX);
    mat_zero_flat(FLAT2D_MUT(pred->Gamma), np * ny, nc * nu, MPC_MAX_NC * MPC_MAX_NU);
    mat_zero_flat(FLAT2D_MUT(pred->Phi_d), np * ny, MPC_MAX_ND, MPC_MAX_ND);
    mat_zero_flat(FLAT2D_MUT(pred->Psi), np * nx_aug, nc * nu, MPC_MAX_NC * MPC_MAX_NU);
    memset(pred->A_pow, 0, sizeof(pred->A_pow));

    mat_eye_flat(FLAT2D_MUT(A_pow), nx_aug, MPC_MAX_NX + MPC_MAX_ND);

    for (i = 0; i < np; i++) {
        double tmp[MPC_MAX_NX + MPC_MAX_ND][MPC_MAX_NX + MPC_MAX_ND];
        mat_copy_flat(FLAT2D(A_pow), FLAT2D_MUT(tmp), nx_aug,
                      MPC_MAX_NX + MPC_MAX_ND, MPC_MAX_NX + MPC_MAX_ND);
        mat_mul_flat(FLAT2D(tmp), FLAT2D(model->Aa), FLAT2D_MUT(A_pow),
                     nx_aug, nx_aug, nx_aug,
                     MPC_MAX_NX + MPC_MAX_ND, MPC_MAX_NX + MPC_MAX_ND,
                     MPC_MAX_NX + MPC_MAX_ND);

        for (j = 0; j < nx_aug; j++)
            for (k = 0; k < nx_aug; k++)
                pred->A_pow[i][j][k] = A_pow[j][k];

        mat_mul_flat(FLAT2D(model->Ca), FLAT2D(A_pow), FLAT2D_MUT(CAi),
                     ny, nx_aug, nx_aug,
                     MPC_MAX_NY, MPC_MAX_NX + MPC_MAX_ND,
                     MPC_MAX_NX + MPC_MAX_ND);

        for (j = 0; j < ny; j++)
            for (k = 0; k < nx_aug; k++)
                pred->Phi[i * ny + j][k] = CAi[j][k];

        int max_move = (i < nc) ? i + 1 : nc;
        int mm;
        for (mm = 0; mm < max_move; mm++) {
            double Apow_ij[MPC_MAX_NX + MPC_MAX_ND][MPC_MAX_NX + MPC_MAX_ND];
            double CAij[MPC_MAX_NY][MPC_MAX_NX + MPC_MAX_ND];
            double CAB[MPC_MAX_NY][MPC_MAX_NU];

            if (i == mm) {
                mat_eye_flat(FLAT2D_MUT(Apow_ij), nx_aug, MPC_MAX_NX + MPC_MAX_ND);
            } else {
                int pidx = i - mm - 1;
                if (pidx >= 0 && pidx < np) {
                    mat_copy_flat(FLAT2D(pred->A_pow[pidx]),
                                  FLAT2D_MUT(Apow_ij), nx_aug,
                                  MPC_MAX_NX, MPC_MAX_NX);
                }
            }

            mat_mul_flat(FLAT2D(model->Ca), FLAT2D(Apow_ij),
                         FLAT2D_MUT(CAij), ny, nx_aug, nx_aug,
                         MPC_MAX_NY, MPC_MAX_NX + MPC_MAX_ND,
                         MPC_MAX_NX + MPC_MAX_ND);
            mat_mul_flat(FLAT2D(CAij), FLAT2D(model->Ba),
                         FLAT2D_MUT(CAB), ny, nx_aug, nu,
                         MPC_MAX_NY, MPC_MAX_NX + MPC_MAX_ND,
                         MPC_MAX_NX + MPC_MAX_ND);

            for (r = 0; r < ny; r++)
                for (c = 0; c < nu; c++)
                    pred->Gamma[i * ny + r][mm * nu + c] = CAB[r][c];
        }
    }
    return 0;
}


/* =================================================================
 * Steady-State Target Computation via Normal Equations + Cholesky
 *
 * Solves [A-I, B; C, D] * [x_ss; u_ss] = [0; y_ref]
 * ================================================================= */
int mpc_compute_steady_state_target(const mpc_ss_model_t *model,
                                     const double y_ref[MPC_MAX_NY],
                                     double x_ss[MPC_MAX_NX],
                                     double u_ss[MPC_MAX_NU])
{
    int nx, nu, ny, n_eq, n_var, i, j, k;
    double M[MPC_MAX_NX + MPC_MAX_NY][MPC_MAX_NX + MPC_MAX_NU];
    double rhs[MPC_MAX_NX + MPC_MAX_NY];
    double MtM[MPC_MAX_NX + MPC_MAX_NU][MPC_MAX_NX + MPC_MAX_NU];
    double Mtrhs[MPC_MAX_NX + MPC_MAX_NU];
    double L[MPC_MAX_NX + MPC_MAX_NU][MPC_MAX_NX + MPC_MAX_NU];
    double y_tmp[MPC_MAX_NX + MPC_MAX_NU], z[MPC_MAX_NX + MPC_MAX_NU];
    int chol_ok;

    if (!model || !y_ref || !x_ss || !u_ss) return -1;
    nx = model->nx; nu = model->nu; ny = model->ny;
    n_eq = nx + ny; n_var = nx + nu;
    if (nx > MPC_MAX_NX || nu > MPC_MAX_NU || ny > MPC_MAX_NY) return -1;

    mat_zero_flat(FLAT2D_MUT(M), n_eq, n_var, MPC_MAX_NX + MPC_MAX_NU);
    memset(rhs, 0, sizeof(rhs));

    for (i = 0; i < nx; i++) {
        for (j = 0; j < nx; j++) M[i][j] = model->A[i][j];
        M[i][i] -= 1.0;
        for (j = 0; j < nu; j++) M[i][nx + j] = model->B[i][j];
    }
    for (i = 0; i < ny; i++) {
        for (j = 0; j < nx; j++) M[nx + i][j] = model->C[i][j];
        for (j = 0; j < nu; j++) M[nx + i][nx + j] = model->D[i][j];
        rhs[nx + i] = y_ref[i];
    }

    mat_zero_flat(FLAT2D_MUT(MtM), n_var, n_var, MPC_MAX_NX + MPC_MAX_NU);
    memset(Mtrhs, 0, sizeof(Mtrhs));
    for (i = 0; i < n_var; i++) {
        for (j = 0; j < n_var; j++) {
            double s_ij = 0.0;
            for (k = 0; k < n_eq; k++) s_ij += M[k][i] * M[k][j];
            MtM[i][j] = s_ij;
        }
        double s_i = 0.0;
        for (k = 0; k < n_eq; k++) s_i += M[k][i] * rhs[k];
        Mtrhs[i] = s_i;
    }

    /* Cholesky: L*L^T = MtM */
    mat_zero_flat(FLAT2D_MUT(L), n_var, n_var, MPC_MAX_NX + MPC_MAX_NU);
    chol_ok = 1;
    for (j = 0; j < n_var && chol_ok; j++) {
        double sd = MtM[j][j];
        for (k = 0; k < j; k++) sd -= L[j][k] * L[j][k];
        if (sd <= SINGULARITY_TOL) { chol_ok = 0; break; }
        L[j][j] = sqrt(sd);
        for (i = j + 1; i < n_var; i++) {
            double so = MtM[i][j];
            for (k = 0; k < j; k++) so -= L[i][k] * L[j][k];
            L[i][j] = so / L[j][j];
        }
    }

    if (chol_ok) {
        for (i = 0; i < n_var; i++) {
            double s3 = Mtrhs[i];
            for (j = 0; j < i; j++) s3 -= L[i][j] * y_tmp[j];
            y_tmp[i] = s3 / L[i][i];
        }
        for (i = n_var - 1; i >= 0; i--) {
            double s4 = y_tmp[i];
            for (j = i + 1; j < n_var; j++) s4 -= L[j][i] * z[j];
            z[i] = s4 / L[i][i];
        }
        for (i = 0; i < nx; i++) x_ss[i] = z[i];
        for (i = 0; i < nu; i++) u_ss[i] = z[nx + i];
    } else {
        memset(x_ss, 0, nx * sizeof(double));
        memset(u_ss, 0, nu * sizeof(double));
    }
    return 0;
}

/* =================================================================
 * Build Step Response Model (DMC format)
 * S_i = cumulative sum of C * A^j * B for j=0..i-1
 * Reference: Cutler and Ramaker, Shell DMC patent (1979)
 * ================================================================= */
int mpc_build_step_model(const mpc_ss_model_t *model, int n_points,
                          mpc_step_model_t *step)
{
    int nx, nu, ny, i, j, k;
    double A_pow[MPC_MAX_NX][MPC_MAX_NX];

    if (!model || !step || n_points <= 0 || n_points > MPC_MAX_NP) return -1;
    nx = model->nx; nu = model->nu; ny = model->ny;
    step->n = n_points; step->nu = nu; step->ny = ny;
    memset(step->S, 0, sizeof(step->S));

    mat_eye_flat(FLAT2D_MUT(A_pow), nx, nx);
    for (i = 0; i < n_points; i++) {
        double CA[MPC_MAX_NY][MPC_MAX_NX], CAB[MPC_MAX_NY][MPC_MAX_NU];
        mat_mul_flat(FLAT2D(model->C), FLAT2D(A_pow), FLAT2D_MUT(CA),
                     ny, nx, nx, ny, nx, nx);
        mat_mul_flat(FLAT2D(CA), FLAT2D(model->B), FLAT2D_MUT(CAB),
                     ny, nx, nu, MPC_MAX_NX, nx, nx);

        for (j = 0; j < ny; j++)
            for (k = 0; k < nu; k++)
                step->S[i][j][k] = (i == 0) ? CAB[j][k] : step->S[i-1][j][k] + CAB[j][k];

        double tmp[MPC_MAX_NX][MPC_MAX_NX];
        mat_copy_flat(FLAT2D(A_pow), FLAT2D_MUT(tmp), nx, nx, nx);
        mat_mul_flat(FLAT2D(tmp), FLAT2D(model->A), FLAT2D_MUT(A_pow),
                     nx, nx, nx, nx, nx, nx);
    }
    return 0;
}


/* =================================================================
 * Observability Check: Kalman Rank Condition
 * O = [C; C*A; C*A^2; ...; C*A^{n-1}]
 * Returns 1 if observable, 0 if not.
 * ================================================================= */
int mpc_check_observability(const mpc_aug_model_t *aug)
{
    int n, ny, i, j, k, rank;
    double A_pow[MPC_MAX_NX + MPC_MAX_ND][MPC_MAX_NX + MPC_MAX_ND];

    if (!aug) return -1;
    n = aug->nx_aug; ny = aug->ny;
    if (n <= 0 || n > MPC_MAX_NX + MPC_MAX_ND) return -1;

    mat_eye_flat(FLAT2D_MUT(A_pow), n, MPC_MAX_NX + MPC_MAX_ND);
    rank = 0;

    for (i = 0; i < n; i++) {
        double CA[MPC_MAX_NY][MPC_MAX_NX + MPC_MAX_ND];
        mat_mul_flat(FLAT2D(aug->Ca), FLAT2D(A_pow), FLAT2D_MUT(CA),
                     ny, n, n, MPC_MAX_NY, MPC_MAX_NX + MPC_MAX_ND,
                     MPC_MAX_NX + MPC_MAX_ND);

        for (j = 0; j < ny; j++) {
            double norm_r = 0.0;
            for (k = 0; k < n; k++)
                norm_r += CA[j][k] * CA[j][k];
            if (sqrt(norm_r) > SINGULARITY_TOL * 10.0) {
                rank++;
                break;
            }
        }
        if (rank >= n) break;

        double tmp[MPC_MAX_NX + MPC_MAX_ND][MPC_MAX_NX + MPC_MAX_ND];
        mat_copy_flat(FLAT2D(A_pow), FLAT2D_MUT(tmp), n,
                      MPC_MAX_NX + MPC_MAX_ND, MPC_MAX_NX + MPC_MAX_ND);
        mat_mul_flat(FLAT2D(tmp), FLAT2D(aug->Aa), FLAT2D_MUT(A_pow),
                     n, n, n, MPC_MAX_NX + MPC_MAX_ND,
                     MPC_MAX_NX + MPC_MAX_ND, MPC_MAX_NX + MPC_MAX_ND);
    }
    return (rank >= n) ? 1 : 0;
}

/* =================================================================
 * Controllability Check: Kalman Rank Condition
 * Ctr = [B, A*B, ..., A^{n-1}*B]
 * ================================================================= */
int mpc_check_controllability(const mpc_aug_model_t *aug)
{
    int n, nu, i, j, k, rank;
    double A_pow[MPC_MAX_NX + MPC_MAX_ND][MPC_MAX_NX + MPC_MAX_ND];

    if (!aug) return -1;
    n = aug->nx_aug; nu = aug->nu;
    if (n <= 0 || n > MPC_MAX_NX + MPC_MAX_ND) return -1;

    mat_eye_flat(FLAT2D_MUT(A_pow), n, MPC_MAX_NX + MPC_MAX_ND);
    rank = 0;

    for (i = 0; i < n; i++) {
        double AB[MPC_MAX_NX + MPC_MAX_ND][MPC_MAX_NU];
        mat_mul_flat(FLAT2D(A_pow), FLAT2D(aug->Ba), FLAT2D_MUT(AB),
                     n, n, nu, MPC_MAX_NX + MPC_MAX_ND,
                     MPC_MAX_NX + MPC_MAX_ND, MPC_MAX_NX + MPC_MAX_ND);

        for (j = 0; j < nu; j++) {
            double cnorm = 0.0;
            for (k = 0; k < n; k++) cnorm += AB[k][j] * AB[k][j];
            if (sqrt(cnorm) > SINGULARITY_TOL * 10.0) rank++;
        }
        if (rank >= n) break;

        double tmp[MPC_MAX_NX + MPC_MAX_ND][MPC_MAX_NX + MPC_MAX_ND];
        mat_copy_flat(FLAT2D(A_pow), FLAT2D_MUT(tmp), n,
                      MPC_MAX_NX + MPC_MAX_ND, MPC_MAX_NX + MPC_MAX_ND);
        mat_mul_flat(FLAT2D(tmp), FLAT2D(aug->Aa), FLAT2D_MUT(A_pow),
                     n, n, n, MPC_MAX_NX + MPC_MAX_ND,
                     MPC_MAX_NX + MPC_MAX_ND, MPC_MAX_NX + MPC_MAX_ND);
    }
    return (rank >= n) ? 1 : 0;
}


/* =================================================================
 * DARE Solver: Kleinman Iteration (1968)
 *
 * P_{k+1} = A^T*P_k*A - A^T*P_k*B*inv(R+B^T*P_k*B)*B^T*P_k*A + Q
 *
 * Theorem (Kleinman 1968): If (A,B) stabilizable and (A,Q^{1/2})
 * detectable, P_k converges to the unique stabilizing solution.
 *
 * Used for LQR terminal cost in MPC for guaranteed closed-loop stability.
 * ================================================================= */
int mpc_solve_dare(const double A[MPC_MAX_NX][MPC_MAX_NX],
                    const double B[MPC_MAX_NX][MPC_MAX_NU],
                    const double Q[MPC_MAX_NX][MPC_MAX_NX],
                    const double R[MPC_MAX_NU][MPC_MAX_NU],
                    int n, int nu,
                    double P[MPC_MAX_NX][MPC_MAX_NX],
                    int max_iter, double tol)
{
    int iter, i, j, k;
    double Pk[MPC_MAX_NX][MPC_MAX_NX];
    double BTP[MPC_MAX_NU][MPC_MAX_NX];
    double BPA[MPC_MAX_NU][MPC_MAX_NX];
    double BPB[MPC_MAX_NU][MPC_MAX_NU];
    double M[MPC_MAX_NU][MPC_MAX_NU];
    double Minv[MPC_MAX_NU][2 * MPC_MAX_NU];
    double K[MPC_MAX_NU][MPC_MAX_NX];
    double BK[MPC_MAX_NX][MPC_MAX_NX];
    double Acl[MPC_MAX_NX][MPC_MAX_NX];
    double PA[MPC_MAX_NX][MPC_MAX_NX];
    double P_new[MPC_MAX_NX][MPC_MAX_NX];

    if (!A || !B || !Q || !R || !P) return -1;
    if (n <= 0 || n > MPC_MAX_NX || nu <= 0 || nu > MPC_MAX_NU) return -1;

    for (i = 0; i < n; i++)
        for (j = 0; j < n; j++)
            Pk[i][j] = Q[i][j];

    for (iter = 0; iter < max_iter; iter++) {
        /* BTP = B^T * P */
        for (i = 0; i < nu; i++)
            for (j = 0; j < n; j++) {
                double s = 0.0;
                for (k = 0; k < n; k++) s += B[k][i] * Pk[k][j];
                BTP[i][j] = s;
            }

        /* BPA = BTP * A = B^T * P * A */
        mat_mul_flat(FLAT2D(BTP), FLAT2D(A), FLAT2D_MUT(BPA),
                     nu, n, n, n, n, n);

        /* BPB = BTP * B = B^T * P * B */
        mat_mul_flat(FLAT2D(BTP), FLAT2D(B), FLAT2D_MUT(BPB),
                     nu, n, nu, n, n, n);

        /* M = R + BPB */
        for (i = 0; i < nu; i++)
            for (j = 0; j < nu; j++)
                M[i][j] = R[i][j] + BPB[i][j];

        /* Invert M via Gauss-Jordan */
        for (i = 0; i < nu; i++) {
            for (j = 0; j < nu; j++) Minv[i][j] = M[i][j];
            for (j = 0; j < nu; j++) Minv[i][nu + j] = (i == j) ? 1.0 : 0.0;
        }
        for (k = 0; k < nu; k++) {
            int prow = k;
            double mv = fabs(Minv[k][k]);
            int ii;
            for (ii = k + 1; ii < nu; ii++)
                if (fabs(Minv[ii][k]) > mv) { mv = fabs(Minv[ii][k]); prow = ii; }
            if (mv < SINGULARITY_TOL) return -1;
            if (prow != k) {
                double t_val;
                for (j = 0; j < 2 * nu; j++) {
                    t_val = Minv[k][j]; Minv[k][j] = Minv[prow][j]; Minv[prow][j] = t_val;
                }
            }
            double piv = Minv[k][k];
            for (j = 0; j < 2 * nu; j++) Minv[k][j] /= piv;
            for (ii = 0; ii < nu; ii++) {
                if (ii != k) {
                    double fac = Minv[ii][k];
                    for (j = 0; j < 2 * nu; j++) Minv[ii][j] -= fac * Minv[k][j];
                }
            }
        }

        /* K = Minv * BPA = inv(R+B^T*P*B) * B^T*P*A */
        mat_mul_flat(FLAT2D(Minv), FLAT2D(BPA), FLAT2D_MUT(K),
                     nu, nu, n, 2 * MPC_MAX_NU, n, n);

        /* BK = B * K, Acl = A - BK */
        mat_mul_flat(FLAT2D(B), FLAT2D(K), FLAT2D_MUT(BK),
                     n, nu, n, n, n, n);
        for (i = 0; i < n; i++)
            for (j = 0; j < n; j++)
                Acl[i][j] = A[i][j] - BK[i][j];

        /* PA = Pk * A */
        mat_mul_flat(FLAT2D(Pk), FLAT2D(A), FLAT2D_MUT(PA),
                     n, n, n, n, n, n);

        /* P_new = Acl^T * PA + Q = (A-BK)^T * P * A + Q */
        for (i = 0; i < n; i++)
            for (j = 0; j < n; j++) {
                double s = 0.0;
                for (k = 0; k < n; k++) s += Acl[k][i] * PA[k][j];
                P_new[i][j] = s + Q[i][j];
            }

        /* Convergence check */
        double diff = 0.0;
        for (i = 0; i < n; i++)
            for (j = 0; j < n; j++)
                diff += fabs(P_new[i][j] - Pk[i][j]);

        for (i = 0; i < n; i++)
            for (j = 0; j < n; j++)
                Pk[i][j] = P_new[i][j];

        if (diff / (double)(n * n) < tol) {
            for (i = 0; i < n; i++)
                for (j = 0; j < n; j++)
                    P[i][j] = P_new[i][j];
            return iter + 1;
        }
    }

    for (i = 0; i < n; i++)
        for (j = 0; j < n; j++)
            P[i][j] = Pk[i][j];
    return max_iter;
}

/* =================================================================
 * Matrix Condition Number (Infinity Norm)
 * cond_inf(A) = ||A||_inf * ||A^{-1}||_inf
 * Used for assessing numerical conditioning of prediction matrices.
 * ================================================================= */
double mpc_matrix_cond(const double A[MPC_MAX_NX][MPC_MAX_NX], int n)
{
    int i, j, k;
    double norm_A, norm_inv;
    double Ac[MPC_MAX_NX][MPC_MAX_NX];
    double Ainv[MPC_MAX_NX][MPC_MAX_NX];

    if (n <= 0 || n > MPC_MAX_NX) return -1.0;

    norm_A = mat_norm_inf_flat(FLAT2D(A), n, n, n);

    /* Gauss-Jordan inversion */
    mat_copy_flat(FLAT2D(A), FLAT2D_MUT(Ac), n, n, n);
    mat_eye_flat(FLAT2D_MUT(Ainv), n, n);

    for (k = 0; k < n; k++) {
        int prow = k;
        double mv = fabs(Ac[k][k]);
        for (i = k + 1; i < n; i++)
            if (fabs(Ac[i][k]) > mv) { mv = fabs(Ac[i][k]); prow = i; }
        if (mv < SINGULARITY_TOL) return -1.0;
        if (prow != k) {
            double t_val;
            for (j = 0; j < n; j++) {
                t_val = Ac[k][j]; Ac[k][j] = Ac[prow][j]; Ac[prow][j] = t_val;
                t_val = Ainv[k][j]; Ainv[k][j] = Ainv[prow][j]; Ainv[prow][j] = t_val;
            }
        }
        double piv = Ac[k][k];
        for (j = 0; j < n; j++) { Ac[k][j] /= piv; Ainv[k][j] /= piv; }
        for (i = 0; i < n; i++) {
            if (i != k) {
                double fac = Ac[i][k];
                for (j = 0; j < n; j++) {
                    Ac[i][j] -= fac * Ac[k][j];
                    Ainv[i][j] -= fac * Ainv[k][j];
                }
            }
        }
    }

    norm_inv = mat_norm_inf_flat(FLAT2D(Ainv), n, n, n);
    return norm_A * norm_inv;
}


/* =================================================================
 * Model Factory Functions: Pre-built canonical models
 * ================================================================= */

void mpc_model_first_order(mpc_ss_model_t *model, double K, double tau, double Ts)
{
    if (!model || tau <= 0.0 || Ts <= 0.0) return;
    model->nx = 1; model->nu = 1; model->ny = 1; model->nd = 0;
    model->A[0][0] = exp(-Ts / tau);
    model->B[0][0] = K * (1.0 - exp(-Ts / tau));
    model->C[0][0] = 1.0;
    model->D[0][0] = 0.0;
}

void mpc_model_second_order(mpc_ss_model_t *model, double K, double wn, double zeta, double Ts)
{
    double Ac[MPC_MAX_NX][MPC_MAX_NX];
    double Bc[MPC_MAX_NX][MPC_MAX_NU];
    double Cc[MPC_MAX_NY][MPC_MAX_NX];
    double Dc[MPC_MAX_NY][MPC_MAX_NU];

    if (!model || wn <= 0.0 || Ts <= 0.0) return;

    Ac[0][0] = 0.0;               Ac[0][1] = 1.0;
    Ac[1][0] = -wn * wn;          Ac[1][1] = -2.0 * zeta * wn;
    Bc[0][0] = 0.0;               Bc[1][0] = K * wn * wn;
    Cc[0][0] = 1.0;               Cc[0][1] = 0.0;
    Dc[0][0] = 0.0;

    mpc_discretize_model(Ac, Bc, Cc, Dc, 2, 1, 1, Ts,
                          MPC_DISCRETIZE_ZOH, model);
}

void mpc_model_integrator(mpc_ss_model_t *model, double K, double Ts)
{
    if (!model || Ts <= 0.0) return;
    model->nx = 1; model->nu = 1; model->ny = 1; model->nd = 0;
    model->A[0][0] = 1.0;
    model->B[0][0] = K * Ts;
    model->C[0][0] = 1.0;
    model->D[0][0] = 0.0;
}

void mpc_model_fopdt(mpc_ss_model_t *model, double K, double tau, double theta, double Ts)
{
    double Ac[MPC_MAX_NX][MPC_MAX_NX];
    double Bc[MPC_MAX_NX][MPC_MAX_NU];
    double Cc[MPC_MAX_NY][MPC_MAX_NX];
    double Dc[MPC_MAX_NY][MPC_MAX_NU];
    double htheta;

    if (!model || tau <= 0.0 || Ts <= 0.0) return;
    if (theta < Ts) {
        mpc_model_first_order(model, K, tau, Ts);
        return;
    }

    htheta = theta / 2.0;
    Ac[0][0] = -1.0 / htheta;     Ac[0][1] = 0.0;
    Ac[1][0] = K / tau;           Ac[1][1] = -1.0 / tau;
    Bc[0][0] = 1.0;               Bc[1][0] = -K / tau;
    Cc[0][0] = 0.0;               Cc[0][1] = 1.0;
    Dc[0][0] = 0.0;

    mpc_discretize_model(Ac, Bc, Cc, Dc, 2, 1, 1, Ts,
                          MPC_DISCRETIZE_ZOH, model);
}
