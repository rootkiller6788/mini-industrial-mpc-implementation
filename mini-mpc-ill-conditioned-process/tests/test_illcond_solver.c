#include "mpc_illcond_defs.h"
#include "mpc_illcond_matrix.h"
#include "mpc_illcond_regularization.h"
#include "mpc_illcond_preconditioner.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>

static int passed = 0, failed = 0;
#define T(name) printf("  TEST: %s ... ", name)
#define P() do { printf("PASS\n"); passed++; } while(0)
#define F(m) do { printf("FAIL: %s\n", m); failed++; return; } while(0)
#define C(c) do { if(!(c)) { F(#c); } } while(0)
#define CE(a,b,t) do { if(fabs((a)-(b))>(t)) { printf("FAIL: %g!=%g\n",(double)(a),(double)(b)); failed++; return; } } while(0)

static void test_tikhonov(void)
{
    T("Tikhonov regularization");
    mpc_matrix_t *A = mpc_matrix_alloc(3, 3);
    C(A != NULL);
    mpc_matrix_eye(A);
    mpc_regularize_tikhonov(A, 1.0);
    CE(mpc_matrix_get(A, 0, 0), 2.0, 1e-15);
    CE(mpc_matrix_get(A, 0, 1), 0.0, 1e-15);
    mpc_matrix_free(&A);
    P();
}

static void test_jacobi_precond(void)
{
    T("Jacobi preconditioner");
    mpc_matrix_t *A = mpc_matrix_alloc(3, 3);
    C(A != NULL);
    mpc_matrix_eye(A);
    mpc_matrix_set(A, 0, 0, 4.0);
    mpc_matrix_set(A, 1, 1, 9.0);
    mpc_matrix_set(A, 2, 2, 16.0);

    mpc_preconditioner_t prec;
    C(mpc_precond_jacobi_build(A, &prec) == 0);

    double r[3] = {8.0, 27.0, 64.0};
    double z[3];
    mpc_precond_jacobi_apply(&prec, r, z);
    CE(z[0], 2.0, 1e-14);
    CE(z[1], 3.0, 1e-14);
    CE(z[2], 4.0, 1e-14);

    mpc_precond_free(&prec);
    mpc_matrix_free(&A);
    P();
}

static void test_pcg_solver(void)
{
    T("PCG solver");
    /* Solve diag(4,9)*x = [8,27] -> x = [2,3] */
    mpc_matrix_t *A = mpc_matrix_alloc(2, 2);
    C(A != NULL);
    mpc_matrix_set(A, 0, 0, 4.0);
    mpc_matrix_set(A, 1, 1, 9.0);

    double b[2] = {8.0, 27.0};
    double x[2] = {0.0, 0.0};

    mpc_preconditioner_t prec;
    prec.type = MPC_PRECOND_JACOBI;
    mpc_precond_jacobi_build(A, &prec);

    int iters = mpc_pcg_solve(A, b, x, &prec, 100, 1e-10);
    C(iters > 0);
    CE(x[0], 2.0, 1e-8);
    CE(x[1], 3.0, 1e-8);

    mpc_precond_free(&prec);
    mpc_matrix_free(&A);
    P();
}

static void test_elastic_net(void)
{
    T("Elastic net regularization");
    mpc_matrix_t *A = mpc_matrix_alloc(4, 4);
    C(A != NULL);
    mpc_matrix_eye(A);
    mpc_matrix_set(A, 0, 0, 0.5);
    mpc_matrix_set(A, 1, 1, 2.0);
    mpc_matrix_set(A, 2, 2, -0.3);
    mpc_matrix_set(A, 3, 3, 5.0);

    mpc_regularize_elastic_net(A, 0.1, 0.5, 50, 1e-8);

    /* After regularization, all diagonal values should be positive */
    C(mpc_matrix_get(A, 0, 0) > 0.0);
    C(mpc_matrix_get(A, 2, 2) > 0.0);

    mpc_matrix_free(&A);
    P();
}

int main(void)
{
    printf("=== Ill-Conditioned Process MPC: Solver Tests ===\n\n");
    test_tikhonov();
    test_jacobi_precond();
    test_pcg_solver();
    test_elastic_net();
    printf("\n=== %d passed, %d failed ===\n", passed, failed);
    return failed > 0 ? 1 : 0;
}
