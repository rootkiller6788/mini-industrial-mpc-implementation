/** @file state_space_mpc.c
 * @brief State-Space MPC Formulation (L2, L5, L8)
 *
 * Implements state-space MPC using the standard formulation:
 *   x(k+1) = A*x(k) + B*u(k)
 *   y(k)   = C*x(k)
 *
 * Prediction matrices (Maciejowski 2002):
 *   X = Phi*x(k) + Gamma*U
 *   Y = C_bar*X
 *
 * where:
 *   Phi = [A; A^2; ...; A^P]  (n*P x n)
 *   Gamma = Toeplitz-like matrix of Markov parameters
 *
 * Ref: Maciejowski (2002) Ch.2, Rawlings/Mayne/Diehl (2017) Ch.2
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "mpc_common.h"

/* Compute A^k (matrix power) for state-space prediction */
static void matrix_power(const double *A, int n, int power, double *A_pow)
{
    if (power == 1) {
        memcpy(A_pow, A, n * n * sizeof(double));
        return;
    }

    double *temp = (double*)calloc(n * n, sizeof(double));
    if (!temp) return;
    memcpy(A_pow, A, n * n * sizeof(double));

    for (int p = 2; p <= power; p++) {
        memset(temp, 0, n * n * sizeof(double));
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                double sum = 0.0;
                for (int k = 0; k < n; k++) sum += A_pow[i*n+k] * A[k*n+j];
                temp[i*n+j] = sum;
            }
        }
        memcpy(A_pow, temp, n * n * sizeof(double));
    }
    free(temp);
}

/* Build Phi matrix: Phi = [A; A^2; ...; A^P]^T (block column) */
static int build_phi_matrix(const double *A, int n, int P, double *Phi)
{
    if (!A || !Phi || n < 1 || P < 1) return -1;
    double *A_pow = (double*)calloc(n * n, sizeof(double));
    if (!A_pow) return -2;

    for (int p = 0; p < P; p++) {
        matrix_power(A, n, p + 1, A_pow);
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                Phi[p*n*n + i*n + j] = A_pow[i*n+j];
    }
    free(A_pow);
    return 0;
}

/* Build Gamma matrix: Gamma(i,j) = C*A^(i-j)*B  for i >= j, else 0 */
static int build_gamma_matrix(const double *A, const double *B,
    const double *C, int nx, int nu, int ny, int P, int M, double *Gamma)
{
    if (!A || !B || !C || !Gamma || nx < 1 || nu < 1 || ny < 1 || P < 1 || M < 1) return -1;
    int n = nx;
    memset(Gamma, 0, P * ny * M * nu * sizeof(double));
    double *A_pow = (double*)calloc(n * n, sizeof(double));
    double *CAB = (double*)calloc(ny * nu, sizeof(double));
    double *temp = (double*)calloc(n * nu, sizeof(double));
    if (!A_pow || !CAB || !temp) { free(A_pow); free(CAB); free(temp); return -2; }

    int *powers = (int*)malloc(P * sizeof(int));
    if (!powers) { free(A_pow); free(CAB); free(temp); return -3; }
    for (int i = 0; i < P; i++) powers[i] = i + 1;

    for (int i = 0; i < P; i++) {
        for (int j = 0; j < M && j <= i; j++) {
            int diff = i - j;
            matrix_power(A, n, diff + 1, A_pow);
            memset(temp, 0, n * nu * sizeof(double));
            for (int r = 0; r < n; r++)
                for (int c = 0; c < nu; c++)
                    for (int k = 0; k < n; k++)
                        temp[r*nu+c] += A_pow[r*n+k] * B[k*nu+c];
            memset(CAB, 0, ny * nu * sizeof(double));
            for (int r = 0; r < ny; r++)
                for (int c = 0; c < nu; c++)
                    for (int k = 0; k < n; k++)
                        CAB[r*nu+c] += C[r*n+k] * temp[k*nu+c];
            int row_base = i * ny;
            int col_base = j * nu;
            for (int r = 0; r < ny; r++)
                for (int c = 0; c < nu; c++)
                    Gamma[(row_base+r)*(M*nu) + (col_base+c)] = CAB[r*nu+c];
        }
    }
    free(powers); free(A_pow); free(CAB); free(temp);
    return 0;
}

/* State-space prediction: Y = C_bar*(Phi*x0 + Gamma*U) */
int mpc_ss_mpc_predict(const double *A, const double *B, const double *C,
    int nx, int nu, int ny, int P, int M,
    const double *x0, const double *U_seq, double *Y_pred)
{
    if (!A || !B || !C || !x0 || !U_seq || !Y_pred) return -1;
    if (nx < 1 || nu < 1 || ny < 1 || P < 1 || M < 1) return -1;

    double *Phi = (double*)calloc(P * nx * nx, sizeof(double));
    double *Gamma = (double*)calloc(P * ny * M * nu, sizeof(double));
    double *AX0 = (double*)calloc(P * nx, sizeof(double));
    if (!Phi || !Gamma || !AX0) {
        free(Phi); free(Gamma); free(AX0); return -2;
    }

    build_phi_matrix(A, nx, P, Phi);
    build_gamma_matrix(A, B, C, nx, nu, ny, P, M, Gamma);

    for (int p = 0; p < P; p++) {
        for (int i = 0; i < nx; i++) {
            double sum = 0.0;
            for (int j = 0; j < nx; j++)
                sum += Phi[p*nx*nx + i*nx + j] * x0[j];
            AX0[p*nx + i] = sum;
        }
    }

    for (int p = 0; p < P; p++) {
        for (int r = 0; r < ny; r++) {
            double y = 0.0;
            for (int k = 0; k < nx; k++) y += C[r*nx+k] * AX0[p*nx+k];

            for (int m = 0; m < M; m++) {
                for (int c = 0; c < nu; c++) {
                    int gamma_row = p * ny + r;
                    int gamma_col = m * nu + c;
                    y += Gamma[gamma_row * (M*nu) + gamma_col] * U_seq[m*nu+c];
                }
            }
            Y_pred[p*ny + r] = y;
        }
    }

    free(Phi); free(Gamma); free(AX0);
    return 0;
}

/* Build QP Hessian for state-space MPC: H = Gamma^T*C_bar^T*Q*C_bar*Gamma + R */
int mpc_ss_build_hessian(const double *A, const double *B, const double *C,
    int nx, int nu, int ny, int P, int M,
    const double *Q, const double *R, double *H)
{
    if (!A || !B || !C || !Q || !R || !H) return -1;
    int n_u = M * nu;
    memset(H, 0, n_u * n_u * sizeof(double));

    double *Gamma = (double*)calloc(P * ny * M * nu, sizeof(double));
    if (!Gamma) return -2;
    build_gamma_matrix(A, B, C, nx, nu, ny, P, M, Gamma);

    for (int i = 0; i < M; i++) {
        for (int ci = 0; ci < nu; ci++) {
            int row_i = i * nu + ci;
            for (int j = 0; j < M; j++) {
                for (int cj = 0; cj < nu; cj++) {
                    int col_j = j * nu + cj;
                    double sum = 0.0;
                    for (int p = 0; p < P; p++) {
                        for (int cv = 0; cv < ny; cv++) {
                            double g_i = Gamma[(p*ny+cv)*(M*nu) + (i*nu+ci)];
                            double g_j = Gamma[(p*ny+cv)*(M*nu) + (j*nu+cj)];
                            sum += Q[cv] * g_i * g_j;
                        }
                    }
                    H[row_i * n_u + col_j] += sum;
                }
            }
            H[row_i * n_u + row_i] += R[ci];
        }
    }
    free(Gamma);
    return 0;
}

/* Compute controllability Gramian: W_c = sum_{k=0}^{N-1} A^k*B*B^T*(A^T)^k */
int mpc_compute_controllability_gramian(const double *A, const double *B,
    int nx, int nu, int N, double *Wc)
{
    if (!A || !B || !Wc || nx < 1 || nu < 1 || N < 1) return -1;
    memset(Wc, 0, nx * nx * sizeof(double));
    double *A_pow = (double*)calloc(nx * nx, sizeof(double));
    double *temp = (double*)calloc(nx * nu, sizeof(double));
    if (!A_pow || !temp) { free(A_pow); free(temp); return -2; }

    for (int k = 0; k < N; k++) {
        matrix_power(A, nx, k, A_pow);
        memset(temp, 0, nx * nu * sizeof(double));
        for (int i = 0; i < nx; i++)
            for (int j = 0; j < nu; j++)
                for (int l = 0; l < nx; l++)
                    temp[i*nu+j] += A_pow[i*nx+l] * B[l*nu+j];

        for (int i = 0; i < nx; i++)
            for (int j = 0; j < nx; j++)
                for (int l = 0; l < nu; l++)
                    Wc[i*nx+j] += temp[i*nu+l] * temp[j*nu+l];
    }
    free(A_pow); free(temp);
    return 0;
}

/* Check observability via Observability Gramian */
int mpc_compute_observability_gramian(const double *A, const double *C,
    int nx, int ny, int N, double *Wo)
{
    if (!A || !C || !Wo || nx < 1 || ny < 1 || N < 1) return -1;
    memset(Wo, 0, nx * nx * sizeof(double));
    double *A_pow = (double*)calloc(nx * nx, sizeof(double));
    double *CA = (double*)calloc(ny * nx, sizeof(double));
    if (!A_pow || !CA) { free(A_pow); free(CA); return -2; }

    for (int k = 0; k < N; k++) {
        matrix_power(A, nx, k, A_pow);
        memset(CA, 0, ny * nx * sizeof(double));
        for (int i = 0; i < ny; i++)
            for (int j = 0; j < nx; j++)
                for (int l = 0; l < nx; l++)
                    CA[i*nx+j] += C[i*nx+l] * A_pow[l*nx+j];

        for (int i = 0; i < nx; i++)
            for (int j = 0; j < nx; j++)
                for (int l = 0; l < ny; l++)
                    Wo[i*nx+j] += CA[l*nx+i] * CA[l*nx+j];
    }
    free(A_pow); free(CA);
    return 0;
}
