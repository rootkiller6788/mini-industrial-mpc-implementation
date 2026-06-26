/* example_cstr_reactor.c -- CSTR reactor temperature identification */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "ssid_defs.h"
#include "ssid_n4sid.h"
#include "ssid_validation.h"

int main(void) {
    printf("=== Subspace ID: CSTR Reactor ===\n\n");
    printf("MIMO: 2 MVs (T_jacket, q_feed), 2 CVs (T_reactor, C_product)\n\n");

    size_t N = 300;
    double Ts = 0.5;
    double C = 0.5, T = 350.0;

    double *u1 = (double *)malloc(N * sizeof(double));
    double *u2 = (double *)malloc(N * sizeof(double));
    double *y1 = (double *)malloc(N * sizeof(double));
    double *y2 = (double *)malloc(N * sizeof(double));

    double V = 100.0, Cf = 1.0, Tf = 350.0, rho = 1000.0, Cp = 0.239;
    double dH = -5.0e4, UA = 5.0e4, k0 = 7.2e10, Ea = 72750.0, Rg = 8.314;

    for (size_t k = 0; k < N; k++) {
        double T_j = 335.0 + 5.0 * sin(0.1 * k);
        double q   = 100.0 + 10.0 * ((k / 8) % 3 == 0 ? 1.0 : -1.0);
        double rate = k0 * exp(-Ea / (Rg * T));
        double dC = (q/V)*(Cf - C) - rate*C;
        double dT_val = (q/V)*(Tf - T) + (-dH)/(rho*Cp)*rate*C + (UA)/(rho*Cp*V)*(T_j - T);
        C += dC * Ts; T += dT_val * Ts;
        u1[k] = T_j; u2[k] = q;
        y1[k] = T + 0.1 * ((double)rand()/RAND_MAX - 0.5);
        y2[k] = C + 0.002 * ((double)rand()/RAND_MAX - 0.5);
    }

    ssid_data_t data;
    memset(&data, 0, sizeof(data));
    data.N = N; data.n_u = 2; data.n_y = 2; data.Ts = Ts;
    data.U = ssid_matrix_alloc(N, 2);
    data.Y = ssid_matrix_alloc(N, 2);
    for (size_t k = 0; k < N; k++) {
        data.U.data[0 * data.U.stride + k] = u1[k];
        data.U.data[1 * data.U.stride + k] = u2[k];
        data.Y.data[0 * data.Y.stride + k] = y1[k];
        data.Y.data[1 * data.Y.stride + k] = y2[k];
    }
    free(u1); free(u2); free(y1); free(y2);

    ssid_config_t cfg = ssid_config_default();
    cfg.n_x_min = 1; cfg.n_x_max = 10; cfg.i_min = 2; cfg.i_max = 30;
    cfg.remove_trend = 1; cfg.normalize_data = 1;

    printf("Running N4SID on CSTR data...\n");
    ssid_result_t result = ssid_n4sid_identify(&data, &cfg);

    printf("\n--- Results ---\n");
    printf("  Order: n_x = %lu\n", (unsigned long)result.n_x_selected);
    printf("  NRMSE: %.2f %%\n", result.fit_metric);
    printf("  Stable: %s\n", result.is_stable ? "YES" : "NO");

    char report[2048];
    ssid_validation_industrial_report(&result, &data, NULL, &cfg, report, sizeof(report));
    printf("\n%s\n", report);

    ssid_result_free(&result);
    ssid_matrix_free(&data.U); ssid_matrix_free(&data.Y);
    printf("=== Complete ===\n");
    return 0;
}
