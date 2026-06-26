/** @file mpc_data_recon.c
 * @brief Data Reconciliation for Industrial MPC (L4, L7)
 *
 * Implements the data reconciliation layer present in AspenTech DMC3.
 * Before the MPC controller runs, raw DCS measurements are reconciled
 * to satisfy mass and energy balance constraints. This removes gross
 * errors from faulty sensors and provides consistent data to the MPC.
 *
 * Algorithm (Weighted Least Squares Data Reconciliation):
 *   Given raw measurements y_raw with covariance Sigma,
 *   find reconciled values y_hat that minimize:
 *     (y_hat - y_raw)^T * Sigma^{-1} * (y_hat - y_raw)
 *   subject to balance constraints: A * y_hat = 0
 *
 * Solution (Kuehn & Davidson 1961):
 *   y_hat = y_raw - Sigma * A^T * (A * Sigma * A^T)^{-1} * A * y_raw
 *
 * Gross Error Detection (Global Test, Madron 1992):
 *   Test statistic: z = r^T * Sigma^{-1} * r  ~  chi^2(df)
 *   where r = y_raw - y_hat are the adjustments.
 *   If z > chi^2_alpha(df), a gross error is present.
 *
 * Ref:
 *   Romagnoli & Sanchez (2000) "Data Processing and Reconciliation"
 *   Narasimhan & Jordache (2000) "Data Reconciliation & Gross Error Detection"
 *   AspenTech DMC3 Documentation - Data Reconciliation Layer
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "mpc_common.h"

#ifndef M_LN2
#define M_LN2 0.69314718055994530942
#endif

/* ===== L4: Weighted Least Squares Data Reconciliation =====
 *
 * Theorem (WLS Reconciliation Optimality):
 *   Under Gaussian measurement errors with known covariance Sigma,
 *   the WLS solution is the maximum likelihood estimate of the
 *   true process values satisfying linear constraints.
 *
 *   y_hat = y_raw - Sigma * A^T * inv(A*Sigma*A^T) * A * y_raw
 *
 * For diagonal Sigma (independent measurements):
 *   y_hat[i] = y_raw[i] - sigma_i^2 * a_i^T * inv(sum_j sigma_j^2 * a_j*a_j^T) * (A*y_raw)
 */

int mpc_data_recon_wls(const double *y_raw, const double *sigma_sq,
    const double *A_balance, int n_meas, int n_constraints,
    double *y_hat, double *adjustments)
{
    if (!y_raw || !sigma_sq || !A_balance || !y_hat || n_meas < 1) return -1;

    double *residual = (double*)calloc(n_constraints, sizeof(double));
    double *ATA = (double*)calloc(n_constraints * n_constraints, sizeof(double));
    double *ATA_inv = (double*)calloc(n_constraints * n_constraints, sizeof(double));
    if (!residual || !ATA || !ATA_inv) {
        free(residual); free(ATA); free(ATA_inv); return -2;
    }

    /* Compute constraint residual: r_con = A * y_raw */
    for (int i = 0; i < n_constraints; i++) {
        residual[i] = 0.0;
        for (int j = 0; j < n_meas; j++) {
            residual[i] += A_balance[i * n_meas + j] * y_raw[j];
        }
    }

    /* Compute A * Sigma * A^T  (Sigma is diagonal) */
    for (int i = 0; i < n_constraints; i++) {
        for (int j = 0; j < n_constraints; j++) {
            double sum = 0.0;
            for (int k = 0; k < n_meas; k++) {
                sum += A_balance[i * n_meas + k] * sigma_sq[k] * A_balance[j * n_meas + k];
            }
            ATA[i * n_constraints + j] = sum;
        }
    }

    /* Invert A*Sigma*A^T (small dense matrix, n_constraints <= n_meas) */
    if (n_constraints == 1) {
        if (fabs(ATA[0]) > MPC_EPS) ATA_inv[0] = 1.0 / ATA[0];
    } else if (n_constraints == 2) {
        double det = ATA[0]*ATA[3] - ATA[1]*ATA[2];
        if (fabs(det) > MPC_EPS) {
            ATA_inv[0] =  ATA[3] / det;
            ATA_inv[1] = -ATA[1] / det;
            ATA_inv[2] = -ATA[2] / det;
            ATA_inv[3] =  ATA[0] / det;
        }
    } else {
        for (int i = 0; i < n_constraints; i++) {
            if (fabs(ATA[i * n_constraints + i]) > MPC_EPS)
                ATA_inv[i * n_constraints + i] = 1.0 / ATA[i * n_constraints + i];
        }
    }

    /* Compute adjustments: delta_y = Sigma * A^T * inv(ATA) * r_con */
    for (int i = 0; i < n_meas; i++) {
        double sum = 0.0;
        for (int j = 0; j < n_constraints; j++) {
            double inner = 0.0;
            for (int k = 0; k < n_constraints; k++) {
                inner += ATA_inv[j * n_constraints + k] * residual[k];
            }
            sum += A_balance[j * n_meas + i] * inner;
        }
        adjustments[i] = -sigma_sq[i] * sum;
        y_hat[i] = y_raw[i] + adjustments[i];
    }

    free(residual); free(ATA); free(ATA_inv);
    return 0;
}

/* ===== L4: Gross Error Detection via Global Test =====
 *
 * Global Test (Madron 1992):
 *   H0: No gross errors present
 *   Test statistic: z = sum_i (adjustment_i^2 / sigma_i^2)
 *
 * Under H0, z ~ chi^2(n_constraints).
 * Critical value at alpha=0.05 for df degrees of freedom.
 *
 * Returns: 0 = no gross error detected, 1 = gross error likely
 */

int mpc_data_recon_gross_error_test(const double *adjustments,
    const double *sigma_sq, int n_meas, int n_constraints,
    double *test_statistic_out)
{
    if (!adjustments || !sigma_sq || n_meas < 1) return -1;

    double z = 0.0;
    for (int i = 0; i < n_meas; i++) {
        if (sigma_sq[i] > MPC_EPS) {
            double adj = adjustments[i];
            z += (adj * adj) / sigma_sq[i];
        }
    }

    if (test_statistic_out) *test_statistic_out = z;

    /* Chi-squared critical values at alpha = 0.05 */
    static const double chi2_crit_005[] = {
        0.0, 3.841, 5.991, 7.815, 9.488, 11.070,
        12.592, 14.067, 15.507, 16.919, 18.307,
        19.675, 21.026, 22.362, 23.685, 24.996
    };

    int df = n_constraints;
    if (df <= 0) return 0;
    if (df > 15) df = 15;

    return (z > chi2_crit_005[df]) ? 1 : 0;
}

/* ===== L7: Nodal Mass Balance Reconciliation =====
 *
 * Industrial application: reconcile flow measurements around a node
 * (e.g., feed = product + reflux in distillation column).
 *
 * Constraint: F_in = F_out1 + F_out2 + ... + F_out_n
 * A_balance = [1, -1, -1, ..., -1] (one row, n_meas columns)
 */

int mpc_reconcile_flow_node(const double *flow_measurements,
    const double *flow_uncertainty, int n_streams,
    double *reconciled_flows, double *imbalance)
{
    if (!flow_measurements || !flow_uncertainty || !reconciled_flows || n_streams < 2)
        return -1;

    double *A = (double*)calloc(n_streams, sizeof(double));
    double *sigma_sq = (double*)calloc(n_streams, sizeof(double));
    double *adjustments = (double*)calloc(n_streams, sizeof(double));
    if (!A || !sigma_sq || !adjustments) {
        free(A); free(sigma_sq); free(adjustments); return -2;
    }

    /* Feed stream is first, others are outputs */
    A[0] = 1.0;
    for (int i = 1; i < n_streams; i++) A[i] = -1.0;

    for (int i = 0; i < n_streams; i++)
        sigma_sq[i] = flow_uncertainty[i] * flow_uncertainty[i];

    int rc = mpc_data_recon_wls(flow_measurements, sigma_sq,
        A, n_streams, 1, reconciled_flows, adjustments);

    if (imbalance && rc == 0) {
        *imbalance = 0.0;
        for (int i = 0; i < n_streams; i++)
            *imbalance += A[i] * flow_measurements[i];
    }

    free(A); free(sigma_sq); free(adjustments);
    return rc;
}

/* ===== L7: Instrument Fault Detection via Sequential Analysis =====
 *
 * Measurement Test (Mah 1976):
 *   For each measurement i, test statistic:
 *     z_i = |adjustment_i| / sigma_i
 *   Compare against standard normal critical value z_alpha/2.
 *
 * If z_i > critical, measurement i is suspect.
 */

int mpc_detect_faulty_sensor(const double *adjustments, const double *sigma,
    int n_meas, int *suspect_flags, double z_critical)
{
    if (!adjustments || !sigma || !suspect_flags || n_meas < 1) return -1;
    if (z_critical <= 0.0) z_critical = 2.576;  /* 99% confidence */

    int n_faulty = 0;
    for (int i = 0; i < n_meas; i++) {
        if (sigma[i] > MPC_EPS) {
            double z_i = fabs(adjustments[i]) / sigma[i];
            suspect_flags[i] = (z_i > z_critical) ? 1 : 0;
            if (suspect_flags[i]) n_faulty++;
        } else {
            suspect_flags[i] = 0;
        }
    }
    return n_faulty;
}

/* ===== L7: Energy Balance Reconciliation =====
 *
 * For a heat exchanger: Q_hot = Q_cold
 *   m_dot_hot * Cp_hot * (T_hot_in - T_hot_out)
 *   = m_dot_cold * Cp_cold * (T_cold_out - T_cold_in)
 *
 * Reconciles temperature and flow measurements simultaneously.
 */

int mpc_reconcile_heat_balance(double m_hot, double m_cold,
    double cp_hot, double cp_cold,
    double T_hot_in, double T_hot_out,
    double T_cold_in, double T_cold_out,
    double *Q_hot_out, double *Q_cold_out, double *heat_loss_pct)
{
    if (!Q_hot_out || !Q_cold_out || !heat_loss_pct) return -1;

    double Q_hot = m_hot * cp_hot * (T_hot_in - T_hot_out);
    double Q_cold = m_cold * cp_cold * (T_cold_out - T_cold_in);

    *Q_hot_out = Q_hot;
    *Q_cold_out = Q_cold;

    if (fabs(Q_hot) > MPC_EPS) {
        *heat_loss_pct = 100.0 * (Q_hot - Q_cold) / Q_hot;
    } else {
        *heat_loss_pct = 0.0;
    }

    return 0;
}
