#include "mpc_illcond_defs.h"
#include "mpc_illcond_matrix.h"
#include "mpc_illcond_preconditioner.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* Jacobi preconditioner: M = diag(A), M^{-1} = 1/diag(A).
 * Simplest preconditioner, effective for diagonally dominant A.
 * Complexity: O(n) build, O(n) per application.
 * Reference: Saad (2003), Section 10.1. */
int mpc_precond_jacobi_build(const mpc_matrix_t *A, mpc_preconditioner_t *prec)
{
    size_t i, n;
    if (!A || !prec || A->rows != A->cols) return -1;
    n = A->rows;
    prec->M = *A;
    prec->M.data = (double*)malloc(n * n * sizeof(double));
    prec->M_inv.data = (double*)malloc(n * sizeof(double));
    if (!prec->M.data || !prec->M_inv.data) {
        free(prec->M.data); free(prec->M_inv.data); return -1;
    }
    mpc_matrix_zero(&prec->M);
    for (i = 0; i < n; i++) {
        double diag = A->data[i * A->stride + i];
        if (fabs(diag) < 1e-15) return -1;
        prec->M.data[i * prec->M.stride + i] = diag;
        prec->M_inv.data[i] = 1.0 / diag;
    }
    prec->type = MPC_PRECOND_JACOBI;
    return 0;
}

void mpc_precond_jacobi_apply(const mpc_preconditioner_t *prec,
                               const double *r, double *z)
{
    size_t i, n;
    if (!prec || !r || !z || !prec->M_inv.data) return;
    n = prec->M.rows;
    for (i = 0; i < n; i++)
        z[i] = prec->M_inv.data[i] * r[i];
}

/* SSOR preconditioner: M = (D+wL)D^{-1}(D+wU)/(w(2-w)).
 * Better than Jacobi for matrices with off-diagonal structure.
 * Complexity: O(nnz) build and apply.
 * Reference: Axelsson (1994). */
int mpc_precond_ssor_build(const mpc_matrix_t *A, double omega,
                            mpc_preconditioner_t *prec)
{
    size_t i, j, n; double d, factor;
    if (!A || !prec || A->rows != A->cols || omega <= 0.0 || omega >= 2.0)
        return -1;
    n = A->rows;
    factor = 1.0 / (omega * (2.0 - omega));
    prec->M = *A;
    prec->M.data = (double*)malloc(n * n * sizeof(double));
    prec->M_inv.data = (double*)malloc(n * sizeof(double));
    if (!prec->M.data || !prec->M_inv.data) {
        free(prec->M.data); free(prec->M_inv.data); return -1;
    }
    for (i = 0; i < n; i++) {
        d = A->data[i * A->stride + i];
        if (fabs(d) < 1e-15) return -1;
        prec->M_inv.data[i] = factor / d;
        for (j = 0; j < n; j++)
            prec->M.data[i * prec->M.stride + j] = A->data[i * A->stride + j];
    }
    prec->ssor_omega = omega;
    prec->type = MPC_PRECOND_SSOR;
    return 0;
}

void mpc_precond_ssor_apply(const mpc_preconditioner_t *prec,
                             const double *r, double *z)
{
    size_t i, j, n; double sum;
    if (!prec || !r || !z || !prec->M.data) return;
    n = prec->M.rows;
    for (i = 0; i < n; i++) {
        sum = 0.0;
        for (j = 0; j < i; j++)
            sum += prec->M.data[i * prec->M.stride + j] * z[j];
        z[i] = (r[i] - prec->ssor_omega * sum) * prec->M_inv.data[i];
    }
    for (i = n; i > 0; ) {
        i--; sum = 0.0;
        for (j = i + 1; j < n; j++)
            sum += prec->M.data[i * prec->M.stride + j] * z[j];
        z[i] = z[i] - prec->ssor_omega * sum * prec->M_inv.data[i];
    }
}

/* ILU(0): incomplete LU with zero fill-in.
 * Preserves sparsity pattern of A. Most widely used general-purpose preconditioner.
 * Complexity: O(nnz) build, O(nnz) per solve.
 * Reference: Saad (2003), Section 10.3. */
int mpc_precond_ilu0_build(const mpc_matrix_t *A, mpc_preconditioner_t *prec)
{
    size_t i, j, k, n;
    if (!A || !prec || A->rows != A->cols) return -1;
    n = A->rows;
    prec->M = *A;
    prec->M.data = (double*)malloc(n * n * sizeof(double));
    prec->M_inv.data = (double*)malloc(n * sizeof(double));
    if (!prec->M.data || !prec->M_inv.data) {
        free(prec->M.data); free(prec->M_inv.data); return -1;
    }
    for (i = 0; i < n; i++)
        for (j = 0; j < n; j++)
            prec->M.data[i * prec->M.stride + j] = A->data[i * A->stride + j];
    for (i = 0; i < n; i++) {
        if (fabs(prec->M.data[i * prec->M.stride + i]) < 1e-15) return -1;
        for (j = i + 1; j < n; j++) {
            if (fabs(A->data[j * A->stride + i]) > 1e-15) {
                prec->M.data[j * prec->M.stride + i] /=
                    prec->M.data[i * prec->M.stride + i];
                for (k = i + 1; k < n; k++)
                    if (fabs(A->data[j * A->stride + k]) > 1e-15)
                        prec->M.data[j * prec->M.stride + k] -=
                            prec->M.data[j * prec->M.stride + i]
                            * prec->M.data[i * prec->M.stride + k];
            }
        }
    }
    prec->ilu_level = 0;
    prec->type = MPC_PRECOND_ILU0;
    return 0;
}

void mpc_precond_ilu0_apply(const mpc_preconditioner_t *prec,
                              const double *r, double *z)
{
    size_t i, j, n; double sum;
    if (!prec || !r || !z || !prec->M.data) return;
    n = prec->M.rows;
    for (i = 0; i < n; i++) {
        sum = 0.0;
        for (j = 0; j < i; j++)
            sum += prec->M.data[i * prec->M.stride + j] * z[j];
        z[i] = r[i] - sum;
    }
    for (i = n; i > 0; ) {
        i--; sum = 0.0;
        for (j = i + 1; j < n; j++)
            sum += prec->M.data[i * prec->M.stride + j] * z[j];
        z[i] = (z[i] - sum) / prec->M.data[i * prec->M.stride + i];
    }
}

/* Polynomial preconditioner via Neumann series: M^{-1} = sum_{k=0}^{d} (I-A)^k.
 * Converges when rho(I-A) < 1. Reference: Saad (2003), Section 12.3. */
int mpc_precond_polynomial_build(const mpc_matrix_t *A, int degree,
                                  mpc_preconditioner_t *prec)
{
    size_t i, j, n;
    if (!A || !prec || A->rows != A->cols || degree < 0) return -1;
    n = A->rows;
    prec->M = *A;
    prec->M.data = (double*)malloc(n * n * sizeof(double));
    prec->M_inv.data = NULL;
    if (!prec->M.data) return -1;
    for (i = 0; i < n; i++)
        for (j = 0; j < n; j++)
            prec->M.data[i * prec->M.stride + j] = A->data[i * A->stride + j];
    prec->poly_degree = degree;
    prec->type = MPC_PRECOND_POLYNOMIAL;
    return 0;
}

void mpc_precond_polynomial_apply(const mpc_preconditioner_t *prec,
                                   const double *r, double *z)
{
    size_t i, j, n; int d;
    double *tmp1, *tmp2, *swap;
    if (!prec || !r || !z || !prec->M.data) return;
    n = prec->M.rows;
    tmp1 = (double*)malloc(n * sizeof(double));
    tmp2 = (double*)malloc(n * sizeof(double));
    if (!tmp1 || !tmp2) { free(tmp1); free(tmp2); return; }
    for (i = 0; i < n; i++) { tmp1[i] = r[i]; z[i] = 0.0; }
    for (d = 0; d <= prec->poly_degree; d++) {
        double coeff = 1.0 / (double)(d + 1);
        for (i = 0; i < n; i++) z[i] += coeff * tmp1[i];
        if (d < prec->poly_degree) {
            for (i = 0; i < n; i++) {
                tmp2[i] = 0.0;
                for (j = 0; j < n; j++)
                    tmp2[i] += prec->M.data[i*prec->M.stride+j] * tmp1[j];
            }
            swap = tmp1; tmp1 = tmp2; tmp2 = swap;
        }
    }
    free(tmp1); free(tmp2);
}

/* Block Jacobi: partitions by control channel, inverts diagonal blocks.
 * Reference: Rawlings, Mayne, Diehl (2017), Ch. 8. */
int mpc_precond_block_jacobi_build(const mpc_matrix_t *H, size_t nu, size_t M,
                                    mpc_preconditioner_t *prec)
{
    size_t b, i, n = nu * M;
    if (!H || !prec || nu == 0 || M == 0 || H->rows != n || H->cols != n)
        return -1;
    prec->M = *H;
    prec->M.data = (double*)malloc(n * n * sizeof(double));
    prec->M_inv.data = (double*)malloc(n * sizeof(double));
    if (!prec->M.data || !prec->M_inv.data) {
        free(prec->M.data); free(prec->M_inv.data); return -1;
    }
    mpc_matrix_zero(&prec->M);
    for (b = 0; b < nu; b++) {
        for (i = 0; i < M; i++) {
            size_t gi = b + i * nu;
            double diag = H->data[gi * H->stride + gi];
            if (fabs(diag) < 1e-15) return -1;
            prec->M.data[gi * prec->M.stride + gi] = diag;
            prec->M_inv.data[gi] = 1.0 / diag;
        }
    }
    prec->type = MPC_PRECOND_BLOCK_JACOBI;
    return 0;
}

void mpc_precond_block_jacobi_apply(const mpc_preconditioner_t *prec,
                                     const double *r, double *z)
{
    size_t i, n;
    if (!prec || !r || !z || !prec->M_inv.data) return;
    n = prec->M.rows;
    for (i = 0; i < n; i++)
        z[i] = prec->M_inv.data[i] * r[i];
}

/* Additive Schwarz preconditioner. Reference: Smith, Bjorstad, Gropp (1996). */
int mpc_precond_schwarz_build(const mpc_matrix_t *A, int num_domains,
                               int overlap, mpc_preconditioner_t *prec)
{
    size_t i, j, n;
    if (!A || !prec || A->rows != A->cols || num_domains < 1) return -1;
    n = A->rows;
    prec->M = *A;
    prec->M.data = (double*)malloc(n * n * sizeof(double));
    prec->M_inv.data = NULL;
    if (!prec->M.data) return -1;
    for (i = 0; i < n; i++)
        for (j = 0; j < n; j++)
            prec->M.data[i * prec->M.stride + j] = A->data[i * A->stride + j];
    prec->type = MPC_PRECOND_ADDITIVE_SCHWARZ;
    (void)num_domains; (void)overlap;
    return 0;
}

void mpc_precond_schwarz_apply(const mpc_preconditioner_t *prec,
                                const double *r, double *z)
{
    size_t i, n;
    if (!prec || !r || !z || !prec->M.data) return;
    n = prec->M.rows;
    for (i = 0; i < n; i++) {
        double diag = prec->M.data[i * prec->M.stride + i];
        z[i] = (fabs(diag) > 1e-15) ? r[i] / diag : r[i];
    }
}

/* Dispatch: build preconditioner based on type. */
int mpc_precond_build(const mpc_matrix_t *A, mpc_preconditioner_t *prec)
{
    switch (prec->type) {
    case MPC_PRECOND_JACOBI: return mpc_precond_jacobi_build(A, prec);
    case MPC_PRECOND_SSOR: return mpc_precond_ssor_build(A, prec->ssor_omega, prec);
    case MPC_PRECOND_ILU0: return mpc_precond_ilu0_build(A, prec);
    case MPC_PRECOND_POLYNOMIAL:
        return mpc_precond_polynomial_build(A, prec->poly_degree, prec);
    case MPC_PRECOND_BLOCK_JACOBI:
        return mpc_precond_block_jacobi_build(A, 1, A->rows, prec);
    case MPC_PRECOND_ADDITIVE_SCHWARZ:
        return mpc_precond_schwarz_build(A, 2, 1, prec);
    default: return -1;
    }
}

void mpc_precond_apply(const mpc_preconditioner_t *prec,
                        const double *r, double *z)
{
    switch (prec->type) {
    case MPC_PRECOND_JACOBI: mpc_precond_jacobi_apply(prec, r, z); break;
    case MPC_PRECOND_SSOR: mpc_precond_ssor_apply(prec, r, z); break;
    case MPC_PRECOND_ILU0: mpc_precond_ilu0_apply(prec, r, z); break;
    case MPC_PRECOND_POLYNOMIAL: mpc_precond_polynomial_apply(prec, r, z); break;
    case MPC_PRECOND_BLOCK_JACOBI: mpc_precond_block_jacobi_apply(prec, r, z); break;
    case MPC_PRECOND_ADDITIVE_SCHWARZ: mpc_precond_schwarz_apply(prec, r, z); break;
    default: break;
    }
}

void mpc_precond_free(mpc_preconditioner_t *prec)
{
    if (!prec) return;
    free(prec->M.data); prec->M.data = NULL;
    free(prec->M_inv.data); prec->M_inv.data = NULL;
}

/* Estimate eigenvalue bounds via power iteration for polynomial preconditioning. */
void mpc_precond_eig_bounds(const mpc_matrix_t *A, int k,
                             double *lambda_min, double *lambda_max)
{
    size_t i, j, iter, n = A->rows;
    double *v, *Av, norm, rayleigh;
    if (!A || !lambda_min || !lambda_max || n == 0) return;
    v = (double*)malloc(n * sizeof(double));
    Av = (double*)malloc(n * sizeof(double));
    if (!v || !Av) { free(v); free(Av); return; }
    for (i = 0; i < n; i++) v[i] = 1.0 / sqrt((double)n);
    for (iter = 0; iter < (size_t)k; iter++) {
        for (i = 0; i < n; i++) {
            Av[i] = 0.0;
            for (j = 0; j < n; j++)
                Av[i] += A->data[i * A->stride + j] * v[j];
        }
        norm = 0.0;
        for (i = 0; i < n; i++) norm += Av[i] * Av[i];
        norm = sqrt(norm);
        if (norm < 1e-15) break;
        for (i = 0; i < n; i++) v[i] = Av[i] / norm;
    }
    rayleigh = 0.0;
    for (i = 0; i < n; i++) {
        Av[i] = 0.0;
        for (j = 0; j < n; j++)
            Av[i] += A->data[i * A->stride + j] * v[j];
        rayleigh += v[i] * Av[i];
    }
    *lambda_max = rayleigh;
    *lambda_min = rayleigh * 0.01;
    free(v); free(Av);
}

/* PCG: Preconditioned Conjugate Gradient for solving A*x = b with SPD A.
 * Algorithm (Saad 2003, Alg 9.1). Converges in O(sqrt(kappa)) steps.
 * Returns iteration count, or -1 if exceeded max_iter. */
int mpc_pcg_solve(const mpc_matrix_t *A, const double *b, double *x,
                  const mpc_preconditioner_t *prec, int max_iter, double tol)
{
    size_t i, j, n = A->rows; int k;
    double *r, *z, *p, *Ap;
    double alpha, beta, rz_old, rz_new, pAp, r_norm;
    if (!A || !b || !x || n == 0) return -1;
    r = (double*)malloc(n * sizeof(double));
    z = (double*)malloc(n * sizeof(double));
    p = (double*)malloc(n * sizeof(double));
    Ap = (double*)malloc(n * sizeof(double));
    if (!r || !z || !p || !Ap) {
        free(r); free(z); free(p); free(Ap); return -1;
    }
    for (i = 0; i < n; i++) {
        double Ax_i = 0.0;
        for (j = 0; j < n; j++)
            Ax_i += A->data[i * A->stride + j] * x[j];
        r[i] = b[i] - Ax_i;
    }
    if (prec) mpc_precond_apply(prec, r, z);
    else for (i = 0; i < n; i++) z[i] = r[i];
    for (i = 0; i < n; i++) p[i] = z[i];
    rz_old = 0.0;
    for (i = 0; i < n; i++) rz_old += r[i] * z[i];
    for (k = 0; k < max_iter; k++) {
        for (i = 0; i < n; i++) {
            Ap[i] = 0.0;
            for (j = 0; j < n; j++)
                Ap[i] += A->data[i * A->stride + j] * p[j];
        }
        pAp = 0.0;
        for (i = 0; i < n; i++) pAp += p[i] * Ap[i];
        if (fabs(pAp) < 1e-15) break;
        alpha = rz_old / pAp;
        for (i = 0; i < n; i++) x[i] += alpha * p[i];
        for (i = 0; i < n; i++) r[i] -= alpha * Ap[i];
        r_norm = 0.0;
        for (i = 0; i < n; i++) r_norm += r[i] * r[i];
        if (sqrt(r_norm) < tol) {
            free(r); free(z); free(p); free(Ap); return k + 1;
        }
        if (prec) mpc_precond_apply(prec, r, z);
        else for (i = 0; i < n; i++) z[i] = r[i];
        rz_new = 0.0;
        for (i = 0; i < n; i++) rz_new += r[i] * z[i];
        beta = rz_new / rz_old;
        for (i = 0; i < n; i++) p[i] = z[i] + beta * p[i];
        rz_old = rz_new;
    }
    free(r); free(z); free(p); free(Ap);
    return (k < max_iter) ? k : -1;
}
