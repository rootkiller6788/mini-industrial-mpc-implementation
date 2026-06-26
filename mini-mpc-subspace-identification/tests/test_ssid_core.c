#include <stdio.h>
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "ssid_defs.h"
#include "ssid_hankel.h"
#include "ssid_svd.h"
#include "ssid_validation.h"

int main(void) {
    printf("Simple integrated test\n");
    fflush(stdout);

    printf("1. Matrix alloc...\n"); fflush(stdout);
    ssid_matrix_t m = ssid_matrix_alloc(2, 3);
    assert(m.data);

    printf("2. Identity...\n"); fflush(stdout);
    ssid_matrix_t eye = ssid_matrix_alloc(2, 2);
    ssid_matrix_eye(&eye);

    printf("3. Config...\n"); fflush(stdout);
    ssid_config_t cfg = ssid_config_default();
    assert(cfg.algorithm == SSID_ALG_N4SID);

    printf("4. Hankel build...\n"); fflush(stdout);
    ssid_matrix_t Z = ssid_matrix_alloc(10, 1);
    for (size_t i = 0; i < 10; i++) ssid_matrix_set(&Z, i, 0, (double)i);
    ssid_hankel_t H = ssid_hankel_build(&Z, 3);
    assert(H.past.data);
    printf("   Hankel: i=%lu, j=%lu, m=%lu\n", (unsigned long)H.i, (unsigned long)H.j, (unsigned long)H.m);

    printf("5. SVD...\n"); fflush(stdout);
    double arr[4] = {3.0, 0.0, 4.0, 0.0};
    ssid_matrix_t A = ssid_matrix_from_array(2, 2, arr);
    ssid_svd_t svd = ssid_svd_compute(&A);
    assert(svd.s);
    printf("   s[0]=%.4f\n", svd.s[0]);

    printf("6. Order selection...\n"); fflush(stdout);
    double s[] = {10.0, 5.0, 3.0, 0.01, 0.001};
    size_t n = ssid_order_svd_gap(s, 5, 3.0);
    printf("   order=%lu\n", (unsigned long)n);

    printf("7. NRMSE...\n"); fflush(stdout);
    double y[] = {1,2,3,4,5}, yh[] = {1.1,1.9,3.0,4.1,4.9};
    ssid_matrix_t Ym = ssid_matrix_from_array(1, 5, y);
    ssid_matrix_t Yh = ssid_matrix_from_array(1, 5, yh);
    double fit = ssid_validation_nrmse(&Ym, &Yh);
    printf("   fit=%.2f%%\n", fit);

    printf("8. Stability...\n"); fflush(stdout);
    double a[] = {0.5, 0.1, 0.0, 0.6};
    ssid_matrix_t As = ssid_matrix_from_array(2, 2, a);
    double sr; int st;
    ssid_validation_check_stability(&As, 0, &sr, &st);
    printf("   stable=%d, sr=%.4f\n", st, sr);

    printf("9. Cleanup...\n"); fflush(stdout);
    ssid_matrix_free(&As); ssid_matrix_free(&Ym); ssid_matrix_free(&Yh);
    ssid_svd_free(&svd); ssid_matrix_free(&A);
    ssid_hankel_free(&H); ssid_matrix_free(&Z);
    ssid_matrix_free(&eye); ssid_matrix_free(&m);

    printf("ALL PASSED\n");
    return 0;
}
