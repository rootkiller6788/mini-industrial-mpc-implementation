/* example_order_selection.c -- Compare order selection criteria */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "ssid_defs.h"
#include "ssid_svd.h"
#include "ssid_n4sid.h"
#include "ssid_validation.h"

int main(void) {
    printf("=== Subspace ID: Order Selection Comparison ===\n\n");
    printf("True order: n_x = 4 (two coupled oscillators)\n\n");

    size_t N = 400;
    double *u_arr = (double *)malloc(N * sizeof(double));
    double *y_arr = (double *)malloc(N * sizeof(double));

    for (size_t k = 0; k < N; k++)
        u_arr[k] = ((k / 7) % 2 == 0) ? 1.0 : -1.0;

    double x[4] = {0, 0, 0, 0};
    for (size_t k = 0; k < N; k++) {
        double u = u_arr[k];
        double nz = 0.02 * ((double)rand() / RAND_MAX - 0.5);
        y_arr[k] = x[0] + 0.5*x[2] + nz;
        double nx[4];
        nx[0] = 0.9*x[0] + 0.1*x[1] + 0.5*u;
        nx[1] = -0.1*x[0] + 0.9*x[1] + 0.3*u;
        nx[2] = 0.7*x[2] + 0.2*x[3] + 0.2*u;
        nx[3] = -0.2*x[2] + 0.7*x[3] + 0.1*u;
        for (int i = 0; i < 4; i++) x[i] = nx[i] + 0.01*nz;
    }

    ssid_data_t data;
    memset(&data, 0, sizeof(data));
    data.N = N; data.n_u = 1; data.n_y = 1; data.Ts = 1.0;
    data.U = ssid_matrix_alloc(N, 1);
    data.Y = ssid_matrix_alloc(N, 1);
    for (size_t k = 0; k < N; k++) {
        data.U.data[0 * data.U.stride + k] = u_arr[k];
        data.Y.data[0 * data.Y.stride + k] = y_arr[k];
    }
    free(u_arr); free(y_arr);

    ssid_order_criterion_t crits[] = {SSID_ORDER_SVD_GAP, SSID_ORDER_AIC, SSID_ORDER_BIC, SSID_ORDER_MDL};
    const char *names[] = {"SVD-gap", "AIC", "BIC", "MDL"};

    printf("%-12s %-8s %-12s\n", "Criterion", "Order", "Fit %%");
    printf("------------------------------------\n");

    for (int c = 0; c < 4; c++) {
        ssid_config_t cfg = ssid_config_default();
        cfg.order_crit = crits[c];
        cfg.n_x_min = 1; cfg.n_x_max = 10;
        cfg.remove_trend = 1;
        ssid_result_t r = ssid_n4sid_identify(&data, &cfg);
        printf("%-12s %-8lu %-12.2f\n", names[c], (unsigned long)r.n_x_selected, r.fit_metric);
        ssid_result_free(&r);
    }

    printf("\nTrue order = 4. SVD-gap: visual heuristic. AIC: weak penalty.\n");
    printf("BIC: consistent. MDL: information-theoretic (Rissanen).\n");

    ssid_matrix_free(&data.U); ssid_matrix_free(&data.Y);
    printf("\n=== Complete ===\n");
    return 0;
}
