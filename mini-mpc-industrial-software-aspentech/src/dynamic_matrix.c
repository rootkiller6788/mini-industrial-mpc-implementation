/** @file dynamic_matrix.c
 * @brief Dynamic Matrix Construction and Operations (L1, L3)
 * Builds the P x M Toeplitz matrix A from step-response coefficients.
 * A[i][j] = s_{i-j+1} if i >= j, else 0
 * Ref: Cutler & Ramaker (1980), Maciejowski (2002)
 */
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "mpc_common.h"

int mpc_build_dynamic_matrix(const mpc_step_model_t *model,
    int P, int M, mpc_dynamic_matrix_t *dm)
{
    if (!model || !dm || P < 1 || M < 1 || M > P) return -1;
    if (!model->coeff || model->n_coeffs < 1) return -2;
    int N = model->n_coeffs;
    dm->P = P; dm->M = M;
    dm->A = (double*)calloc(P * M, sizeof(double));
    dm->A_T = (double*)calloc(M * P, sizeof(double));
    if (!dm->A || !dm->A_T) { mpc_dynamic_matrix_free(dm); return -3; }
    for (int i = 0; i < P; i++) {
        for (int j = 0; j < M; j++) {
            if (i >= j) {
                int k = i - j;
                dm->A[i*M+j] = (k < N) ? model->coeff[k] : model->coeff[N-1];
            } else {
                dm->A[i*M+j] = 0.0;
            }
        }
    }
    for (int i = 0; i < M; i++)
        for (int j = 0; j < P; j++)
            dm->A_T[i*P+j] = dm->A[j*M+i];
    return 0;
}

void mpc_dynamic_matrix_free(mpc_dynamic_matrix_t *dm)
{ if (dm) { free(dm->A); free(dm->A_T); dm->A = NULL; dm->A_T = NULL; } }

int mpc_dynamic_matrix_apply(const mpc_dynamic_matrix_t *dm,
    const double *du, double *y_forced)
{
    if (!dm || !du || !y_forced) return -1;
    for (int i = 0; i < dm->P; i++) {
        y_forced[i] = 0.0;
        for (int j = 0; j < dm->M; j++)
            y_forced[i] += dm->A[i*dm->M+j] * du[j];
    }
    return 0;
}

int mpc_build_mimo_dynamic_matrix(const mpc_mimo_model_t *mimo,
    int P, int M, double *A_global, int ldA)
{
    if (!mimo || !A_global || P < 1 || M < 1 || ldA < M * mimo->n_mv) return -1;
    int n_cv = mimo->n_cv, n_mv = mimo->n_mv;
    int total_rows = P * n_cv, total_cols = M * n_mv;
    memset(A_global, 0, total_rows * ldA * sizeof(double));

    for (int cv = 0; cv < n_cv; cv++) {
        for (int mv = 0; mv < n_mv; mv++) {
            mpc_step_model_t *sm = &mimo->sub_models[cv][mv];
            if (!sm->coeff || sm->n_coeffs < 1) continue;
            int N = sm->n_coeffs;
            for (int i = 0; i < P; i++) {
                for (int j = 0; j < M; j++) {
                    if (i >= j) {
                        int k = i - j;
                        double val = (k < N) ? sm->coeff[k] : sm->coeff[N-1];
                        A_global[(cv*P+i)*ldA + (mv*M+j)] = val;
                    }
                }
            }
        }
    }
    return 0;
}
