/* example_2nd_order.c -- Identify 2nd-order system and validate
 *
 * L6: Second-order system identification from I/O data.
 * Demonstrates N4SID pipeline, NRMSE fit, and MPC readiness.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "ssid_defs.h"
#include "ssid_n4sid.h"
#include "ssid_validation.h"

int main(void) {
    printf("=== Subspace ID: 2nd-Order System Example ===\n\n");
    printf("True: x_{k+1}=[0.8 0.1;-0.1 0.7]x_k+[0.1;0.2]u_k, y_k=[1 0]x_k\n\n");

    size_t N = 500;
    double *u_arr = (double *)malloc(N * sizeof(double));
    double *y_arr = (double *)malloc(N * sizeof(double));

    for (size_t k = 0; k < N; k++)
        u_arr[k] = ((k / 5) % 2 == 0) ? 1.0 : -1.0;

    double x1 = 0.0, x2 = 0.0;
    for (size_t k = 0; k < N; k++) {
        double u = u_arr[k];
        double noise = 0.02 * ((double)rand() / RAND_MAX - 0.5);
        y_arr[k] = x1 + noise;
        double nx1 = 0.8*x1 + 0.1*x2 + 0.1*u;
        double nx2 = -0.1*x1 + 0.7*x2 + 0.2*u;
        x1 = nx1; x2 = nx2;
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

    ssid_config_t cfg = ssid_config_default();
    cfg.n_x_min = 1; cfg.n_x_max = 6;
    cfg.remove_trend = 1; cfg.normalize_data = 1;

    printf("Running N4SID...\n");
    ssid_result_t result = ssid_n4sid_identify(&data, &cfg);

    printf("\n--- Results ---\n");
    printf("  Status: %s\n", result.status_msg);
    printf("  Order: n_x = %lu\n", (unsigned long)result.n_x_selected);
    printf("  NRMSE fit: %.2f %%\n", result.fit_metric);
    printf("  Stable: %s\n", result.is_stable ? "YES" : "NO");

    if (result.model.n_x == 2 && result.model.A.data) {
        printf("\n  Identified A:\n");
        for (size_t i = 0; i < 2; i++) {
            printf("    ");
            for (size_t j = 0; j < 2; j++)
                printf("%8.4f ", ssid_matrix_get(&result.model.A, i, j));
            printf("\n");
        }
    }

    char report[2048];
    ssid_validation_industrial_report(&result, &data, NULL, &cfg, report, sizeof(report));
    printf("\n%s\n", report);

    ssid_result_free(&result);
    ssid_matrix_free(&data.U); ssid_matrix_free(&data.Y);
    printf("=== Complete ===\n");
    return 0;
}
