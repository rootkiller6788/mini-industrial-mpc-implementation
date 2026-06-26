#ifndef MPC_ILLCOND_MATRIX_H
#define MPC_ILLCOND_MATRIX_H

#include "mpc_illcond_defs.h"

mpc_matrix_t* mpc_matrix_alloc(size_t rows, size_t cols);
void mpc_matrix_free(mpc_matrix_t **mat);
mpc_matrix_t* mpc_matrix_copy(const mpc_matrix_t *src);
void mpc_matrix_zero(mpc_matrix_t *mat);
int mpc_matrix_eye(mpc_matrix_t *mat);
void mpc_matrix_get_diag(const mpc_matrix_t *mat, double *diag);
void mpc_matrix_set_diag(mpc_matrix_t *mat, const double *diag);

static inline double mpc_matrix_get(const mpc_matrix_t *mat, size_t i, size_t j) {
    return mat->data[i * mat->stride + j];
}
static inline void mpc_matrix_set(mpc_matrix_t *mat, size_t i, size_t j, double val) {
    mat->data[i * mat->stride + j] = val;
}

double mpc_matrix_norm_frobenius(const mpc_matrix_t *mat);
double mpc_matrix_norm_1(const mpc_matrix_t *mat);
double mpc_matrix_norm_inf(const mpc_matrix_t *mat);

void mpc_matrix_gemv(double alpha, const mpc_matrix_t *A, const double *x,
                     double beta, double *y);
void mpc_matrix_gemm(double alpha, const mpc_matrix_t *A, const mpc_matrix_t *B,
                     double beta, mpc_matrix_t *C);
void mpc_matrix_add(double alpha, const mpc_matrix_t *A,
                    double beta, const mpc_matrix_t *B, mpc_matrix_t *C);
int mpc_matrix_transpose(const mpc_matrix_t *A, mpc_matrix_t *B);

void mpc_matrix_forward_sub(const mpc_matrix_t *L, const double *b, double *x);
void mpc_matrix_backward_sub(const mpc_matrix_t *U, const double *b, double *x);

int mpc_matrix_is_symmetric(const mpc_matrix_t *mat, double tol);
int mpc_matrix_is_spd(const mpc_matrix_t *mat, double tol);
double mpc_matrix_trace(const mpc_matrix_t *mat);
double mpc_matrix_det(const mpc_matrix_t *mat);
int mpc_matrix_cholesky(mpc_matrix_t *mat);
int mpc_matrix_lu(mpc_matrix_t *mat, size_t *pivot);

#endif /* MPC_ILLCOND_MATRIX_H */
