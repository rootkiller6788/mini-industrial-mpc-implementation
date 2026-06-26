#include "mpc_illcond_defs.h"
#include "mpc_illcond_matrix.h"
#include "mpc_illcond_svd.h"
#include "mpc_illcond_condition.h"
#include "mpc_illcond_sensitivity.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>

static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) do { printf("  TEST: %s ... ", name); } while(0)
#define PASS() do { printf("PASS\n"); tests_passed++; } while(0)
#define FAIL(msg) do { printf("FAIL: %s\n", msg); tests_failed++; } while(0)
#define CHECK(cond) do { if (!(cond)) { FAIL(#cond); return; } } while(0)
#define CHECK_CLOSE(a, b, tol) do { \
    if (fabs((a)-(b)) > (tol)) { \
        printf("FAIL: %g != %g (tol=%g)\n", (double)(a), (double)(b), (tol)); \
        tests_failed++; return; \
    } \
} while(0)

/* Test matrix allocation and basic operations */
static void test_matrix_lifecycle(void)
{
    TEST("matrix alloc/free");
    mpc_matrix_t *m = mpc_matrix_alloc(3, 3);
    CHECK(m != NULL);
    CHECK(m->rows == 3);
    CHECK(m->cols == 3);
    mpc_matrix_free(&m);
    CHECK(m == NULL);
    PASS();
}

static void test_matrix_eye(void)
{
    TEST("matrix identity");
    mpc_matrix_t *m = mpc_matrix_alloc(3, 3);
    CHECK(m != NULL);
    CHECK(mpc_matrix_eye(m) == 0);
    CHECK_CLOSE(mpc_matrix_get(m, 0, 0), 1.0, 1e-15);
    CHECK_CLOSE(mpc_matrix_get(m, 0, 1), 0.0, 1e-15);
    CHECK_CLOSE(mpc_matrix_get(m, 2, 2), 1.0, 1e-15);
    mpc_matrix_free(&m);
    PASS();
}

static void test_matrix_norms(void)
{
    TEST("matrix norms");
    mpc_matrix_t *m = mpc_matrix_alloc(2, 2);
    CHECK(m != NULL);
    mpc_matrix_eye(m);
    mpc_matrix_set(m, 0, 1, 3.0);

    double fnorm = mpc_matrix_norm_frobenius(m);
    CHECK_CLOSE(fnorm, sqrt(1.0 + 9.0 + 1.0), 1e-14);

    double n1 = mpc_matrix_norm_1(m);
    CHECK_CLOSE(n1, 4.0, 1e-14);

    double ninf = mpc_matrix_norm_inf(m);
    CHECK_CLOSE(ninf, 4.0, 1e-14);

    mpc_matrix_free(&m);
    PASS();
}

static void test_matrix_ops(void)
{
    TEST("matrix multiply");
    mpc_matrix_t *A = mpc_matrix_alloc(2, 2);
    mpc_matrix_t *B = mpc_matrix_alloc(2, 2);
    mpc_matrix_t *C = mpc_matrix_alloc(2, 2);
    CHECK(A && B && C);

    mpc_matrix_set(A, 0, 0, 1.0); mpc_matrix_set(A, 0, 1, 2.0);
    mpc_matrix_set(A, 1, 0, 3.0); mpc_matrix_set(A, 1, 1, 4.0);
    mpc_matrix_set(B, 0, 0, 5.0); mpc_matrix_set(B, 0, 1, 6.0);
    mpc_matrix_set(B, 1, 0, 7.0); mpc_matrix_set(B, 1, 1, 8.0);

    mpc_matrix_gemm(1.0, A, B, 0.0, C);
    CHECK_CLOSE(mpc_matrix_get(C, 0, 0), 19.0, 1e-14);
    CHECK_CLOSE(mpc_matrix_get(C, 0, 1), 22.0, 1e-14);
    CHECK_CLOSE(mpc_matrix_get(C, 1, 0), 43.0, 1e-14);
    CHECK_CLOSE(mpc_matrix_get(C, 1, 1), 50.0, 1e-14);

    mpc_matrix_free(&A); mpc_matrix_free(&B); mpc_matrix_free(&C);
    PASS();
}

static void test_matrix_cholesky(void)
{
    TEST("Cholesky factorization");
    mpc_matrix_t *A = mpc_matrix_alloc(2, 2);
    CHECK(A != NULL);
    /* SPD matrix: [4, 1; 1, 3] */
    mpc_matrix_set(A, 0, 0, 4.0); mpc_matrix_set(A, 0, 1, 1.0);
    mpc_matrix_set(A, 1, 0, 1.0); mpc_matrix_set(A, 1, 1, 3.0);

    CHECK(mpc_matrix_is_spd(A, 1e-15) == 1);
    CHECK(mpc_matrix_cholesky(A) == 0);
    CHECK_CLOSE(mpc_matrix_get(A, 0, 0), 2.0, 1e-14);  /* L[0,0] */
    CHECK_CLOSE(mpc_matrix_get(A, 1, 0), 0.5, 1e-14);  /* L[1,0] */

    mpc_matrix_free(&A);
    PASS();
}

static void test_condition_estimate(void)
{
    TEST("condition estimate");
    /* Well-conditioned identity matrix */
    mpc_matrix_t *A = mpc_matrix_alloc(3, 3);
    CHECK(A != NULL);
    mpc_matrix_eye(A);

    double kappa = mpc_condition_estimate(A, MPC_CONDEST_NORM1);
    CHECK(kappa > 0.0 && kappa < 100.0);

    /* Ill-conditioned matrix */
    mpc_matrix_set(A, 0, 0, 1.0); mpc_matrix_set(A, 0, 1, 0.0);
    mpc_matrix_set(A, 1, 0, 0.0); mpc_matrix_set(A, 1, 1, 1e-8);

    double kappa2 = mpc_condition_estimate(A, MPC_CONDEST_NORM1);
    CHECK(kappa2 > kappa);

    mpc_matrix_free(&A);
    PASS();
}

static void test_svd_compute(void)
{
    TEST("SVD compute");
    mpc_matrix_t *A = mpc_matrix_alloc(2, 2);
    CHECK(A != NULL);
    mpc_matrix_set(A, 0, 0, 3.0); mpc_matrix_set(A, 0, 1, 0.0);
    mpc_matrix_set(A, 1, 0, 0.0); mpc_matrix_set(A, 1, 1, 1.0);

    mpc_svd_t svd;
    CHECK(mpc_svd_compute(A, &svd) == 0);
    CHECK(svd.S != NULL);
    CHECK(svd.rank > 0);
    CHECK(svd.cond > 0.0);

    mpc_svd_free(&svd);
    mpc_matrix_free(&A);
    PASS();
}

int main(void)
{
    printf("=== Ill-Conditioned Process MPC: Core Tests ===\n\n");

    test_matrix_lifecycle();
    test_matrix_eye();
    test_matrix_norms();
    test_matrix_ops();
    test_matrix_cholesky();
    test_condition_estimate();
    test_svd_compute();

    printf("\n=== Results: %d passed, %d failed ===\n",
           tests_passed, tests_failed);
    return tests_failed > 0 ? 1 : 0;
}
