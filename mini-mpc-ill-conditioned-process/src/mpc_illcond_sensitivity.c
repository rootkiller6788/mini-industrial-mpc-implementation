#include "mpc_illcond_defs.h"
#include "mpc_illcond_matrix.h"
#include "mpc_illcond_svd.h"
#include "mpc_illcond_sensitivity.h"
#include "mpc_illcond_condition.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdio.h>

/* RGA: Relative Gain Array = G .* (G^{-1})^T.
 * Theorem (Bristol, 1966): RGA_ij quantifies steady-state interaction.
 * RGA_ij = 1 -> perfect pairing. RGA_ij < 0 -> unstable.
 * RGA_ij >> 1 -> severe interaction, ill-conditioning indicator.
 * Complexity: O(ny*nu*min(ny,nu)) via SVD pseudo-inverse.
 * Reference: Skogestad & Postlethwaite (2005), Section 3.3. */
int mpc_sensitivity_rga(const mpc_matrix_t *G, mpc_matrix_t *RGA)
{
    size_t i, j, k, ny, nu;
    mpc_svd_t svd;
    int ret;

    if (!G || !RGA) return -1;
    ny = G->rows; nu = G->cols;
    if (RGA->rows != ny || RGA->cols != nu) return -1;

    ret = mpc_svd_compute(G, &svd);
    if (ret != 0) return -1;

    for (i = 0; i < ny; i++) {
        for (j = 0; j < nu; j++) {
            double g_ij = G->data[i * G->stride + j];
            double pinv_ji = 0.0;
            for (k = 0; k < svd.rank; k++) {
                pinv_ji += svd.V.data[j * svd.V.stride + k]
                         * (1.0 / svd.S[k])
                         * svd.U.data[i * svd.U.stride + k];
            }
            RGA->data[i * RGA->stride + j] = g_ij * pinv_ji;
        }
    }

    mpc_svd_free(&svd);
    return 0;
}

/* RGA condition number: ||RGA - I||_sum. Values > 1: interaction, > 5: severe. */
double mpc_sensitivity_rga_cond(const mpc_matrix_t *RGA)
{
    size_t i, j, n;
    double sum = 0.0;

    if (!RGA) return 0.0;
    n = RGA->rows < RGA->cols ? RGA->rows : RGA->cols;

    for (i = 0; i < n; i++) {
        for (j = 0; j < n; j++) {
            double expected = (i == j) ? 1.0 : 0.0;
            sum += fabs(RGA->data[i * RGA->stride + j] - expected);
        }
    }

    return sum;
}

/* Condition number sensitivity gradient.
 * L4 Theorem (Golub & Van Loan, 2013, Sec 2.6):
 *   d(kappa_2(A))/d(A_ij) = (u_i * v_j) / ||v||^2
 * where u_i, v_j are singular vectors for sigma_min. */
int mpc_sensitivity_gradient(const mpc_svd_t *svd, mpc_matrix_t *grad)
{
    size_t i, j, m, n, last_idx;
    double v_norm_sq = 0.0;

    if (!svd || !grad || !svd->S) return -1;
    m = svd->U.rows;
    n = svd->V.rows;
    if (grad->rows != m || grad->cols != n) return -1;

    last_idx = (m < n ? m : n) - 1;

    for (i = 0; i < n; i++) {
        double v = svd->V.data[i * svd->V.stride + last_idx];
        v_norm_sq += v * v;
    }

    if (v_norm_sq < 1e-15) return -1;

    for (i = 0; i < m; i++) {
        double u_i = svd->U.data[i * svd->U.stride + last_idx];
        for (j = 0; j < n; j++) {
            double v_j = svd->V.data[j * svd->V.stride + last_idx];
            grad->data[i * grad->stride + j] = (u_i * v_j) / v_norm_sq;
        }
    }

    return 0;
}

/* Collinearity: max |g_i^T g_j| / (||g_i|| * ||g_j||) over gain columns. */
double mpc_sensitivity_collinearity(const mpc_matrix_t *G,
                                     size_t *worst_i, size_t *worst_j)
{
    size_t i, j, k, ny, nu;
    double max_coll = 0.0;

    if (!G) return 0.0;
    ny = G->rows; nu = G->cols;

    for (i = 0; i < nu; i++) {
        double norm_i = 0.0;
        for (k = 0; k < ny; k++) {
            double v = G->data[k * G->stride + i];
            norm_i += v * v;
        }
        norm_i = sqrt(norm_i);
        if (norm_i < 1e-15) continue;

        for (j = i + 1; j < nu; j++) {
            double norm_j = 0.0, dot = 0.0;
            for (k = 0; k < ny; k++) {
                dot += G->data[k * G->stride + i]
                     * G->data[k * G->stride + j];
                double vj = G->data[k * G->stride + j];
                norm_j += vj * vj;
            }
            norm_j = sqrt(norm_j);
            if (norm_j < 1e-15) continue;

            double coll = fabs(dot) / (norm_i * norm_j);
            if (coll > max_coll) {
                max_coll = coll;
                if (worst_i) *worst_i = i;
                if (worst_j) *worst_j = j;
            }
        }
    }

    return max_coll;
}

/* Stiffness ratio: tau_max / tau_min over all i/o channel time constants. */
double mpc_sensitivity_stiffness(const double *tau, size_t ny, size_t nu)
{
    size_t i, n = ny * nu;
    double tmin = INFINITY, tmax = 0.0;
    if (!tau || n == 0) return 1.0;
    for (i = 0; i < n; i++) {
        if (tau[i] <= 0.0) return INFINITY;
        if (tau[i] < tmin) tmin = tau[i];
        if (tau[i] > tmax) tmax = tau[i];
    }
    if (tmin < 1e-15) return INFINITY;
    return tmax / tmin;
}

/* Minimum gain direction via SVD: returns sigma_min.
 * v_min = right singular vector (input direction), u_min = left (output). */
double mpc_sensitivity_min_gain_direction(const mpc_svd_t *svd,
                                           double *v_min, double *u_min)
{
    size_t i, n;
    if (!svd || !svd->S) return 0.0;
    n = svd->U.rows < svd->V.rows ? svd->U.rows : svd->V.rows;
    if (n == 0) return 0.0;
    double sv_min = svd->S[n - 1];
    if (v_min)
        for (i = 0; i < svd->V.rows; i++)
            v_min[i] = svd->V.data[i * svd->V.stride + (n - 1)];
    if (u_min)
        for (i = 0; i < svd->U.rows; i++)
            u_min[i] = svd->U.data[i * svd->U.stride + (n - 1)];
    return sv_min;
}

/* Comprehensive sensitivity analysis of an MPC model.
 * Computes all diagnostics and populates the report. */
int mpc_sensitivity_analyze(const mpc_illcond_model_t *model,
                             mpc_illcond_diagnostic_t *diag)
{
    mpc_matrix_t *RGA;
    double coll;
    size_t worst_i, worst_j;

    if (!model || !diag) return -1;
    memset(diag, 0, sizeof(mpc_illcond_diagnostic_t));

    mpc_condition_diagnose(&model->G, diag);

    RGA = mpc_matrix_alloc(model->ny, model->nu);
    if (RGA) {
        if (mpc_sensitivity_rga(&model->G, RGA) == 0)
            diag->rga_condition_number = mpc_sensitivity_rga_cond(RGA);
        mpc_matrix_free(&RGA);
    }

    coll = mpc_sensitivity_collinearity(&model->G, &worst_i, &worst_j);
    diag->collinearity_detected = coll;
    diag->stiffness_diagnostic = model->stiffness_ratio;
    diag->primary_cause = mpc_sensitivity_rootcause(diag);

    return 0;
}

/* Root cause classification based on diagnostic values. */
mpc_illcond_rootcause_t mpc_sensitivity_rootcause(
    const mpc_illcond_diagnostic_t *diag)
{
    if (!diag) return MPC_ROOTCAUSE_NEARLY_RANK_DEFICIENT;
    if (diag->collinearity_detected > 0.95)
        return MPC_ROOTCAUSE_NEAR_COLLINEAR_GAINS;
    if (diag->stiffness_diagnostic > 1.0e5)
        return MPC_ROOTCAUSE_HIGH_STIFFNESS_RATIO;
    if (diag->rga_condition_number > 10.0)
        return MPC_ROOTCAUSE_HIGH_RGA;
    if (diag->effective_rank_ratio < 0.5)
        return MPC_ROOTCAUSE_NEARLY_RANK_DEFICIENT;
    if (diag->min_singular_value < 1e-8 * diag->max_singular_value)
        return MPC_ROOTCAUSE_ZERO_GAIN_DIRECTION;
    if (diag->condition_number > 1.0e8)
        return MPC_ROOTCAUSE_POOR_SCALING;
    return MPC_ROOTCAUSE_NEARLY_RANK_DEFICIENT;
}

/* Generate operator-readable sensitivity report string. */
int mpc_sensitivity_report(const mpc_illcond_diagnostic_t *diag,
                            char *buffer, size_t bufsize)
{
    int n;
    const char *sens_str, *cause_str;

    if (!diag || !buffer || bufsize == 0) return 0;

    switch (diag->sensitivity) {
    case MPC_SENSITIVITY_LOW: sens_str = "LOW"; break;
    case MPC_SENSITIVITY_MODERATE: sens_str = "MODERATE"; break;
    case MPC_SENSITIVITY_HIGH: sens_str = "HIGH"; break;
    default: sens_str = "EXTREME"; break;
    }

    switch (diag->primary_cause) {
    case MPC_ROOTCAUSE_NEAR_COLLINEAR_GAINS:
        cause_str = "Near-collinear gains"; break;
    case MPC_ROOTCAUSE_HIGH_STIFFNESS_RATIO:
        cause_str = "High stiffness ratio"; break;
    case MPC_ROOTCAUSE_NEARLY_RANK_DEFICIENT:
        cause_str = "Nearly rank-deficient"; break;
    case MPC_ROOTCAUSE_POOR_SCALING:
        cause_str = "Poor scaling"; break;
    case MPC_ROOTCAUSE_HIGH_RGA:
        cause_str = "High RGA"; break;
    case MPC_ROOTCAUSE_MEASUREMENT_REDUNDANCY:
        cause_str = "Measurement redundancy"; break;
    case MPC_ROOTCAUSE_ZERO_GAIN_DIRECTION:
        cause_str = "Zero-gain direction"; break;
    default: cause_str = "Unknown"; break;
    }

    n = snprintf(buffer, bufsize,
        "Cond: %.2e (1:%.2e inf:%.2e)\n"
        "SV: [%.2e, %.2e] rank=%.2f\n"
        "RGA: %.2f Coll: %.4f Stiff: %.2e\n"
        "Sens: %s Cause: %s\n"
        "Lambda: %.2e\n",
        diag->condition_number, diag->condition_number_1,
        diag->condition_number_inf,
        diag->max_singular_value, diag->min_singular_value,
        diag->effective_rank_ratio,
        diag->rga_condition_number, diag->collinearity_detected,
        diag->stiffness_diagnostic,
        sens_str, cause_str, diag->recommended_lambda);

    return n;
}

/* Numerical rank via SVD threshold: count S[i] > eps * S[0]. */
size_t mpc_sensitivity_numerical_rank(const double *S, size_t n, double eps)
{
    size_t i, rank = 0;
    if (!S || n == 0) return 0;
    for (i = 0; i < n; i++) {
        if (S[i] > eps * S[0]) rank++;
        else break;
    }
    return rank;
}

/* Distance to rank deficiency: min_{rank=k} ||A-B||_2 = sigma_{k+1}. */
double mpc_sensitivity_rank_distance(const mpc_svd_t *svd)
{
    if (!svd || !svd->S) return 0.0;
    return svd->S[svd->rank];
}

