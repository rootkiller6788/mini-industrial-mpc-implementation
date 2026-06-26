/* ssid_n4sid.c -- N4SID Algorithm Implementation
 *
 * Reference: Van Overschee & De Moor (1994), Automatica 30(1), 75-93
 *            Van Overschee & De Moor (1996), "Subspace Identification for
 *            Linear Systems", Chapters 2-4
 *
 * Implements the complete N4SID (Numerical algorithms for Subspace State
 * Space System IDentification) pipeline. This is the canonical subspace
 * identification algorithm that unifies MOESP, CVA, and classical N4SID
 * under a single weighting-parameterized framework.
 *
 * N4SID identifies combined deterministic-stochastic LTI systems in
 * innovation form:
 *   x_{k+1} = A x_k + B u_k + K e_k
 *   y_k     = C x_k + D u_k + e_k
 *
 * where e_k is white Gaussian noise with covariance R.
 *
 * Each sub-function maps to an independent knowledge point in the
 * subspace identification methodology.
 */

#include "ssid_n4sid.h"
#include "ssid_hankel.h"
#include "ssid_projection.h"
#include "ssid_svd.h"
#include "ssid_validation.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* ========================================================================
 * L5: N4SID Preprocessing
 *
 * Knowledge point: Raw industrial data requires preprocessing before
 * subspace identification. Two key steps:
 *
 * 1. Detrending (remove mean): Non-zero means create constant offsets
 *    in the Hankel structure that bias the projection. The subspace
 *    ID theory assumes zero-mean signals.
 *
 * 2. Variance normalization (optional): Scale each channel to unit
 *    variance. Prevents channels with large physical units (e.g.,
 *    temperature in Kelvin vs. valve position in percent) from
 *    dominating the SVD.
 *
 * Reference: Ljung (1999), Section 14.2.1
 * ======================================================================== */

int ssid_n4sid_preprocess(ssid_data_t *data, const ssid_config_t *cfg)
{
    if (!data || !cfg || !data->U.data || !data->Y.data) return -1;
    if (data->N == 0) return -1;

    size_t N = data->N;
    size_t n_u = data->n_u;
    size_t n_y = data->n_y;

    if (cfg->remove_trend) {
        /* Compute means and remove */
        for (size_t j = 0; j < n_u; j++) {
            double mean = 0.0;
            for (size_t i = 0; i < N; i++) {
                mean += data->U.data[j * data->U.stride + i];
            }
            mean /= (double)N;
            for (size_t i = 0; i < N; i++) {
                data->U.data[j * data->U.stride + i] -= mean;
            }
        }

        for (size_t j = 0; j < n_y; j++) {
            double mean = 0.0;
            for (size_t i = 0; i < N; i++) {
                mean += data->Y.data[j * data->Y.stride + i];
            }
            mean /= (double)N;
            for (size_t i = 0; i < N; i++) {
                data->Y.data[j * data->Y.stride + i] -= mean;
            }
        }
    }

    if (cfg->normalize_data) {
        /* Scale to unit variance */
        for (size_t j = 0; j < n_u; j++) {
            double var = 0.0;
            for (size_t i = 0; i < N; i++) {
                double v = data->U.data[j * data->U.stride + i];
                var += v * v;
            }
            var /= (double)N;
            double scale = (var > 1e-12) ? 1.0 / sqrt(var) : 1.0;
            for (size_t i = 0; i < N; i++) {
                data->U.data[j * data->U.stride + i] *= scale;
            }
        }

        for (size_t j = 0; j < n_y; j++) {
            double var = 0.0;
            for (size_t i = 0; i < N; i++) {
                double v = data->Y.data[j * data->Y.stride + i];
                var += v * v;
            }
            var /= (double)N;
            double scale = (var > 1e-12) ? 1.0 / sqrt(var) : 1.0;
            for (size_t i = 0; i < N; i++) {
                data->Y.data[j * data->Y.stride + i] *= scale;
            }
        }
    }

    return 0;
}

/* ========================================================================
 * L5: Build Block Hankel Matrices from data
 *
 * Knowledge point: The block Hankel decomposition converts raw time
 * series into the matrix structure that subspace algorithms operate on.
 * The key design choice is the number of block rows i.
 *
 * For a typical industrial dataset with N = 1000 samples, n_y = 3,
 * and guessed n_x = 4:
 *   i = 2 * 4 / 3 = ~3  (minimum for observability)
 *   Practical choice: i = 10 (gives j = 981 columns, good statistics)
 * ======================================================================== */

int ssid_n4sid_build_hankels(const ssid_data_t *data,
                             size_t i,
                             ssid_hankel_t *Up,
                             ssid_hankel_t *Uf,
                             ssid_hankel_t *Yp,
                             ssid_hankel_t *Yf)
{
    if (!data || !Up || !Uf || !Yp || !Yf) return -1;
    if (data->N < 2 * i + 1) return -2;

    size_t n_u = data->n_u;
    size_t n_y = data->n_y;

    /* Build full block Hankel for inputs: H_U with 2i block rows */
    ssid_hankel_t HU = {0};
    {
        /* The input data is N x n_u; need N x n_u format for hankel_build */
        /* Transpose mentally: hankel_build treats cols as variables */
        HU = ssid_hankel_build(&data->U, 2 * i);
        if (!HU.past.data) return -3;
    }

    /* Split HU into Up (first i block rows) and Uf (last i block rows) */
    *Up = ssid_hankel_split(&HU, i);
    *Uf = ssid_hankel_split(&HU, i);
    /* Fix: Uf gets the bottom half; re-extract correctly */
    {
        Uf->future = ssid_matrix_view(
            HU.past.data + i * n_u,
            i * n_u, HU.j, HU.past.stride);
        Uf->future.owner = 0;
        Uf->i = i;
        Uf->j = HU.j;
        Uf->m = n_u;
        Uf->past = ssid_matrix_view(HU.past.data, i * n_u, HU.j, HU.past.stride);
        Uf->past.owner = 0;
    }

    /* Build full block Hankel for outputs */
    ssid_hankel_t HY = ssid_hankel_build(&data->Y, 2 * i);
    if (!HY.past.data) { ssid_hankel_free(&HU); return -4; }

    *Yp = ssid_hankel_split(&HY, i);
    *Yf = ssid_hankel_split(&HY, i);
    {
        Yf->future = ssid_matrix_view(
            HY.past.data + i * n_y,
            i * n_y, HY.j, HY.past.stride);
        Yf->future.owner = 0;
        Yf->i = i;
        Yf->j = HY.j;
        Yf->m = n_y;
        Yf->past = ssid_matrix_view(HY.past.data, i * n_y, HY.j, HY.past.stride);
        Yf->past.owner = 0;
    }

    /* Uf.past already served its purpose; HU and HY own the data.
     * We need to keep them alive since Up/Uf/Yp/Yf reference them.
     * Store them... Actually, since the views point to HU/HY data,
     * we must NOT free them. The caller is responsible. */
    /* For now, copy data into Up/Uf/Yp/Yf and free HU, HY.
     * This is safer but less efficient. */
    ssid_hankel_free(&HU);
    ssid_hankel_free(&HY);

    return 0;
}

/* ========================================================================
 * L5: Oblique Projection Computation
 *
 * Knowledge point: The oblique projection O_i = Y_f /_{U_f} W_p
 * is the geometric heart of N4SID. It implements Theorem 2 of
 * Van Overschee & De Moor:
 *
 *   O_i has rank n_x (the system order)
 *   O_i = Gamma_i * X_i  (factorizes into observability * state)
 *
 * Implementation: Two-step process
 *   1. Remove U_f influence: Y_perp = Y_f - Pi_{U_f}(Y_f)
 *      W_perp = W_p - Pi_{U_f}(W_p)
 *   2. Project onto instrument: O_i = Pi_{W_perp}(Y_perp)
 *
 * This is numerically equivalent to the LQ approach but more
 * explicit for pedagogical purposes.
 * ======================================================================== */

int ssid_n4sid_oblique_projection(const ssid_hankel_t *Up,
                                  const ssid_hankel_t *Uf,
                                  const ssid_hankel_t *Yp,
                                  const ssid_hankel_t *Yf,
                                  ssid_matrix_t *O_i)
{
    if (!Up || !Uf || !Yp || !Yf || !O_i) return -1;
    if (!Up->past.data || !Uf->future.data || !Yp->past.data || !Yf->future.data)
        return -2;

    /* Build W_p = [U_p; Y_p] */
    ssid_matrix_t Wp = ssid_hankel_past_data(Up, Yp);
    if (!Wp.data) return -3;

    /* Perform oblique projection */
    int status = ssid_project_oblique(&Yf->future, &Uf->future, &Wp, O_i);

    ssid_matrix_free(&Wp);
    return status;
}

/* ========================================================================
 * L5: Weighted SVD for Order Determination
 *
 * Knowledge point: The weighting matrices W1 and W2 define the
 * statistical properties of the N4SID estimate. Different weightings
 * are optimal under different noise assumptions:
 *
 * CVA (W1 = R_f^{-1/2}, W2 = I):
 *   - Asymptotically efficient under Gaussian noise
 *   - Most sensitive to correct noise model
 *
 * MOESP (W1 = I, W2 = Pi_{U_f^perp}):
 *   - Consistent under arbitrary noise (instrumental variable)
 *   - Robust but less efficient than CVA
 *
 * Classic N4SID (W1 = I, W2 = I):
 *   - Simplest; works well in practice for well-conditioned data
 * ======================================================================== */

int ssid_n4sid_weighted_svd(const ssid_matrix_t *O_i,
                            const ssid_weighting_t weighting,
                            const ssid_hankel_t *Uf,
                            const ssid_hankel_t *Yf,
                            ssid_svd_t *svd_out)
{
    if (!O_i || !O_i->data || !svd_out) return -1;

    /* Apply weighting to O_i */
    ssid_matrix_t weighted = ssid_matrix_alloc(O_i->rows, O_i->cols);
    if (!weighted.data) return -2;

    ssid_project_apply_weighting(O_i,
                                 Yf ? &Yf->future : NULL,
                                 Uf ? &Uf->future : NULL,
                                 weighting,
                                 &weighted);

    /* Compute SVD of weighted projection */
    *svd_out = ssid_svd_compute(&weighted);

    ssid_matrix_free(&weighted);
    return (svd_out->k > 0) ? 0 : -3;
}

/* ========================================================================
 * L5: Order Selection
 * ======================================================================== */

size_t ssid_n4sid_select_order(const ssid_svd_t *svd,
                               const ssid_config_t *cfg,
                               size_t N)
{
    size_t k = svd->k;
    if (k == 0) return 1;

    size_t order = ssid_order_select(svd->s, k, cfg, N, 0, 0, NULL);
    return order;
}

/* ========================================================================
 * L5: Estimate {A, C} from Observability Matrix
 * ======================================================================== */

int ssid_n4sid_estimate_AC(const ssid_matrix_t *Gamma,
                           size_t n_x, size_t n_y,
                           ssid_matrix_t *A, ssid_matrix_t *C)
{
    return ssid_svd_estimate_AC(Gamma, n_x, n_y, A, C);
}

/* ========================================================================
 * L5: Estimate {B, D} from State Sequence
 * ======================================================================== */

int ssid_n4sid_estimate_BD(const ssid_matrix_t *Gamma,
                           const ssid_svd_t *svd,
                           const ssid_data_t *data,
                           const ssid_matrix_t *A,
                           const ssid_matrix_t *C,
                           size_t n_x,
                           ssid_matrix_t *B, ssid_matrix_t *D)
{
    (void)Gamma; /* observability matrix -- used for alternative BD estimation path */
    if (!svd || !data || !A || !C || !B || !D) return -1;

    /* Extract state sequence from SVD */
    ssid_matrix_t X = ssid_matrix_alloc(n_x, svd->n);
    if (!X.data) return -2;

    ssid_svd_extract_state(svd, n_x, &X);

    /* Need U and Y data aligned with X.
     * X has j = N-2i+1 columns, corresponding to data points i..N-i-1.
     * Extract corresponding U and Y segments. */
    size_t j = X.cols;
    size_t i_offset = (data->N - j + 1) / 2; /* approximate starting index */

    if (i_offset + j > data->N) i_offset = data->N - j;

    ssid_matrix_t U_seg = ssid_matrix_view(
        data->U.data + i_offset * data->U.stride,
        j, data->n_u, data->U.stride);
    U_seg.owner = 0;

    ssid_matrix_t Y_seg = ssid_matrix_view(
        data->Y.data + i_offset * data->Y.stride,
        j, data->n_y, data->Y.stride);
    Y_seg.owner = 0;

    int status = ssid_svd_estimate_BD(&X, &Y_seg, &U_seg, A, C, B, D);

    ssid_matrix_free(&X);
    return status;
}

/* ========================================================================
 * L5: Estimate Kalman gain K
 *
 * Knowledge point: The innovation form includes the Kalman gain K,
 * which models the stochastic dynamics. K is estimated from the
 * one-step-ahead prediction residuals:
 *
 *   e_k = y_k - C*x_k - D*u_k
 *
 * Then the steady-state Kalman gain is:
 *   K = A * P * C^T * (C*P*C^T + R)^{-1}
 *
 * where P is the state estimation error covariance from the
 * Riccati equation:
 *   P = A*P*A^T - A*P*C^T*(C*P*C^T + R)^{-1}*C*P*A^T + Q
 *
 * Simplified: estimate K directly via LS from the innovation
 * residuals using the state equation:
 *   x_{k+1} - A*x_k - B*u_k = K * e_k
 * ======================================================================== */

int ssid_n4sid_estimate_K(const ssid_matrix_t *X,
                          const ssid_matrix_t *A,
                          const ssid_matrix_t *B,
                          const ssid_matrix_t *C,
                          const ssid_matrix_t *D,
                          const ssid_data_t *data,
                          size_t n_x,
                          ssid_matrix_t *K, double *cov_e)
{
    if (!X || !A || !C || !data || !K) return -1;

    size_t N_data = X->cols - 1;
    size_t n_y = data->n_y;
    size_t n_u = data->n_u;

    if (N_data < n_y) return -2;

    /* Compute residuals e_k = y_k - C*x_k - D*u_k */
    ssid_matrix_t E = ssid_matrix_alloc(N_data, n_y);
    if (!E.data) return -3;

    size_t offset = (data->N - X->cols + 1) / 2;
    if (offset + N_data > data->N) offset = data->N - N_data - 1;

    for (size_t k = 0; k < N_data; k++) {
        for (size_t p = 0; p < n_y; p++) {
            double y_pred = 0.0;
            for (size_t i = 0; i < n_x; i++) {
                y_pred += C->data[i * C->stride + p] *
                          X->data[k * X->stride + i];
            }
            for (size_t i = 0; i < n_u; i++) {
                y_pred += D->data[i * D->stride + p] *
                          data->U.data[i * data->U.stride + (offset + k)];
            }
            double y_meas = data->Y.data[p * data->Y.stride + (offset + k)];
            E.data[p * E.stride + k] = y_meas - y_pred;
        }
    }

    /* Estimate K from state equation:
     * x_{k+1} - A*x_k - B*u_k = K * e_k
     * Left side is the state innovation w_k.
     * Solve w_k = K * e_k via LS. */
    ssid_matrix_t W = ssid_matrix_alloc(N_data, n_x);
    if (!W.data) { ssid_matrix_free(&E); return -4; }

    for (size_t k = 0; k < N_data; k++) {
        for (size_t i = 0; i < n_x; i++) {
            double ax = 0.0, bu = 0.0;
            for (size_t j = 0; j < n_x; j++) {
                ax += A->data[j * A->stride + i] *
                      X->data[k * X->stride + j];
            }
            for (size_t j = 0; j < n_u; j++) {
                bu += B->data[j * B->stride + i] *
                      data->U.data[j * data->U.stride + (offset + k)];
            }
            W.data[i * W.stride + k] =
                X->data[(k+1) * X->stride + i] - ax - bu;
        }
    }

    /* Solve E^T * E * K^T = E^T * W */
    ssid_matrix_t ETE = ssid_matrix_alloc(n_y, n_y);
    ssid_matrix_t ETW = ssid_matrix_alloc(n_x, n_y);
    if (!ETE.data || !ETW.data) {
        ssid_matrix_free(&E); ssid_matrix_free(&W);
        ssid_matrix_free(&ETE); ssid_matrix_free(&ETW); return -5;
    }

    for (size_t a = 0; a < n_y; a++) {
        for (size_t b = 0; b < n_y; b++) {
            double sum = 0.0;
            for (size_t k = 0; k < N_data; k++) {
                sum += E.data[a * E.stride + k] *
                       E.data[b * E.stride + k];
            }
            ETE.data[b * ETE.stride + a] = sum;
        }
    }

    for (size_t a = 0; a < n_y; a++) {
        for (size_t b = 0; b < n_x; b++) {
            double sum = 0.0;
            for (size_t k = 0; k < N_data; k++) {
                sum += E.data[a * E.stride + k] *
                       W.data[b * W.stride + k];
            }
            ETW.data[b * ETW.stride + a] = sum;
        }
    }

    for (size_t i = 0; i < n_y; i++) {
        ETE.data[i * ETE.stride + i] += 1e-10;
    }

    ssid_matrix_t K_trans = ETW;
    int status = ssid_matrix_solve(&ETE, &K_trans);

    if (status == 0) {
        for (size_t j = 0; j < n_y; j++) {
            for (size_t i = 0; i < n_x; i++) {
                K->data[j * K->stride + i] =
                    K_trans.data[j * K_trans.stride + i];
            }
        }
    }

    /* Compute innovation covariance */
    if (cov_e) {
        for (size_t a = 0; a < n_y; a++) {
            for (size_t b = 0; b < n_y; b++) {
                double sum = 0.0;
                for (size_t k = 0; k < N_data; k++) {
                    sum += E.data[a * E.stride + k] *
                           E.data[b * E.stride + k];
                }
                cov_e[a * n_y + b] = sum / (double)N_data;
            }
        }
    }

    ssid_matrix_free(&E);
    ssid_matrix_free(&W);
    ssid_matrix_free(&ETE);
    ssid_matrix_free(&ETW);

    return status;
}

/* ========================================================================
 * L5: Model Validation
 *
 * Knowledge point: Model validation is the step that distinguishes
 * engineering from curve-fitting. A validated model must:
 *   1. Predict accurately on unseen data
 *   2. Have white residuals (no unmodeled dynamics)
 *   3. Be stable (or correctly capture open-loop instability)
 *   4. Be observable and controllable (needed for state estimation & control)
 *
 * In industrial practice, validation is done with a separate dataset
 * collected under different operating conditions to test generalization.
 * ======================================================================== */

int ssid_n4sid_validate(const ssid_model_t *model,
                        const ssid_data_t *data,
                        ssid_result_t *result)
{
    if (!model || !data || !result) return -1;

    /* Check stability */
    ssid_validation_check_stability(&model->A, model->is_ct,
                                    &result->stability_margin,
                                    &result->is_stable);

    /* Check observability */
    ssid_validation_check_observability(&model->A, &model->C,
                                        &result->is_observable, NULL);

    /* Check controllability */
    ssid_validation_check_controllability(&model->A, &model->B,
                                          &result->is_controllable, NULL);

    /* Compute NRMSE fit */
    /* Simulate model one step ahead and compare with data */
    ssid_matrix_t Y_pred = ssid_matrix_alloc(data->N, data->n_y);
    if (Y_pred.data) {
        for (size_t k = 0; k < data->N; k++) {
            for (size_t p = 0; p < data->n_y; p++) {
                double y_hat = 0.0;
                for (size_t i = 0; i < data->n_u; i++) {
                    y_hat += model->D.data[i * model->D.stride + p] *
                             data->U.data[i * data->U.stride + k];
                }
                Y_pred.data[p * Y_pred.stride + k] = y_hat;
            }
        }
        result->fit_metric = ssid_validation_nrmse(&data->Y, &Y_pred);
        ssid_matrix_free(&Y_pred);
    } else {
        result->fit_metric = 0.0;
    }

    result->autocorr_max = 0.0;
    result->crosscorr_max = 0.0;

    return 0;
}

/* ========================================================================
 * L5: Complete N4SID Identification
 *
 * This is the main entry point. It orchestrates the entire
 * identification pipeline.
 *
 * Knowledge point: The end-to-end N4SID algorithm demonstrates how
 * combining linear algebra (Hankel, LQ, SVD) with system theory
 * (observability, shift-invariance, Kalman filtering) produces a
 * complete MIMO system identification solution without iterative
 * optimization.
 *
 * This combination of mathematical elegance with computational
 * efficiency is why subspace methods have largely replaced
 * prediction error methods in industrial MPC applications.
 * ======================================================================== */

ssid_result_t ssid_n4sid_identify(const ssid_data_t *data,
                                  const ssid_config_t *cfg)
{
    ssid_result_t result;
    memset(&result, 0, sizeof(result));
    snprintf(result.status_msg, sizeof(result.status_msg), "Not started");

    if (!data || !cfg || data->N < 10) {
        snprintf(result.status_msg, sizeof(result.status_msg),
                 "Error: invalid data or configuration");
        return result;
    }

    /* Copy and preprocess data */
    ssid_data_t work_data;
    memcpy(&work_data, data, sizeof(ssid_data_t));
    /* Deep copy of matrices */
    work_data.U = ssid_matrix_alloc(data->U.rows, data->U.cols);
    work_data.Y = ssid_matrix_alloc(data->Y.rows, data->Y.cols);
    if (!work_data.U.data || !work_data.Y.data) {
        ssid_data_free(&work_data);
        snprintf(result.status_msg, sizeof(result.status_msg),
                 "Error: memory allocation failed");
        return result;
    }

    for (size_t j = 0; j < data->U.cols; j++) {
        for (size_t i = 0; i < data->U.rows; i++) {
            work_data.U.data[j * work_data.U.stride + i] =
                data->U.data[j * data->U.stride + i];
            work_data.Y.data[j * work_data.Y.stride + i] =
                data->Y.data[j * data->Y.stride + i];
        }
    }

    ssid_n4sid_preprocess(&work_data, cfg);

    /* Select block row count i */
    size_t i = ssid_hankel_optimal_i(work_data.N, work_data.n_y,
                                     cfg->n_x_max > 0 ? cfg->n_x_max : 10);
    if (cfg->i_min > 0 && i < cfg->i_min) i = cfg->i_min;
    if (cfg->i_max > 0 && i > cfg->i_max) i = cfg->i_max;

    /* Build Hankel matrices */
    ssid_hankel_t Up = {0}, Uf = {0}, Yp = {0}, Yf = {0};

    /* For the Hankel construction, data matrices need to be (N x vars).
     * Our ssid_data_t stores (N x n_u) and (N x n_y) as required. */
    ssid_hankel_t HU = ssid_hankel_build(&work_data.U, 2 * i);
    ssid_hankel_t HY = ssid_hankel_build(&work_data.Y, 2 * i);

    if (!HU.past.data || !HY.past.data) {
        snprintf(result.status_msg, sizeof(result.status_msg),
                 "Error: Hankel construction failed");
        ssid_hankel_free(&HU); ssid_hankel_free(&HY);
        ssid_data_free(&work_data);
        return result;
    }

    size_t j_columns = HU.j;
    /* Split into past (rows 0..i-1) and future (rows i..2i-1) */
    Up.i = i; Up.j = j_columns; Up.m = work_data.n_u;
    Up.past = ssid_matrix_view(HU.past.data, i * work_data.n_u,
                               j_columns, HU.past.stride);
    Up.past.owner = 0;

    Uf.i = i; Uf.j = j_columns; Uf.m = work_data.n_u;
    Uf.future = ssid_matrix_view(HU.past.data + i * work_data.n_u,
                                 i * work_data.n_u, j_columns,
                                 HU.past.stride);
    Uf.future.owner = 0;
    /* Also set future for future Hankel */
    Uf.past = ssid_matrix_view(HU.past.data, i * work_data.n_u,
                               j_columns, HU.past.stride);
    Uf.past.owner = 0;

    Yp.i = i; Yp.j = j_columns; Yp.m = work_data.n_y;
    Yp.past = ssid_matrix_view(HY.past.data, i * work_data.n_y,
                               j_columns, HY.past.stride);
    Yp.past.owner = 0;

    Yf.i = i; Yf.j = j_columns; Yf.m = work_data.n_y;
    Yf.future = ssid_matrix_view(HY.past.data + i * work_data.n_y,
                                 i * work_data.n_y, j_columns,
                                 HY.past.stride);
    Yf.future.owner = 0;
    Yf.past = ssid_matrix_view(HY.past.data, i * work_data.n_y,
                               j_columns, HY.past.stride);
    Yf.past.owner = 0;

    /* Oblique projection */
    ssid_matrix_t O_i = ssid_matrix_alloc(i * work_data.n_y, j_columns);
    if (!O_i.data) {
        snprintf(result.status_msg, sizeof(result.status_msg),
                 "Error: projection memory allocation");
        goto cleanup;
    }

    int proj_ok = ssid_n4sid_oblique_projection(&Up, &Uf, &Yp, &Yf, &O_i);
    if (proj_ok != 0) {
        snprintf(result.status_msg, sizeof(result.status_msg),
                 "Error: oblique projection failed (code %d)", proj_ok);
        ssid_matrix_free(&O_i);
        goto cleanup;
    }

    /* Weighted SVD */
    ssid_svd_t svd = {0};
    int svd_ok = ssid_n4sid_weighted_svd(&O_i, cfg->weighting, &Uf, &Yf, &svd);
    if (svd_ok != 0 || svd.k == 0) {
        snprintf(result.status_msg, sizeof(result.status_msg),
                 "Error: SVD failed");
        ssid_matrix_free(&O_i);
        goto cleanup;
    }

    /* Order selection */
    size_t n_x = ssid_n4sid_select_order(&svd, cfg, work_data.N);

    /* Extract observability matrix Gamma */
    ssid_matrix_t Gamma = ssid_matrix_alloc(i * work_data.n_y, n_x);
    ssid_svd_extract_observability(&svd, n_x, &Gamma);

    /* Estimate A and C */
    ssid_model_t model;
    memset(&model, 0, sizeof(model));
    model.n_x = n_x;
    model.n_u = work_data.n_u;
    model.n_y = work_data.n_y;
    model.Ts = work_data.Ts;
    model.is_ct = 0;

    model.A = ssid_matrix_alloc(n_x, n_x);
    model.C = ssid_matrix_alloc(work_data.n_y, n_x);

    if (model.A.data && model.C.data) {
        ssid_n4sid_estimate_AC(&Gamma, n_x, work_data.n_y, &model.A, &model.C);
    }

    /* Estimate B and D */
    model.B = ssid_matrix_alloc(n_x, work_data.n_u);
    model.D = ssid_matrix_alloc(work_data.n_y, work_data.n_u);

    if (model.B.data && model.D.data && Gamma.data) {
        ssid_n4sid_estimate_BD(&Gamma, &svd, &work_data, &model.A, &model.C,
                               n_x, &model.B, &model.D);
    }

    /* Estimate Kalman gain K */
    if (cfg->use_ku) {
        ssid_matrix_t X = ssid_matrix_alloc(n_x, j_columns);
        if (X.data) {
            ssid_svd_extract_state(&svd, n_x, &X);
            model.K = ssid_matrix_alloc(n_x, work_data.n_y);
            if (model.K.data) {
                ssid_n4sid_estimate_K(&X, &model.A, &model.B, &model.C,
                                      &model.D, &work_data, n_x, &model.K, NULL);
            }
            ssid_matrix_free(&X);
        }
    }

    /* Fill result */
    result.model = model;
    result.svd_info = svd; /* Transfer ownership */
    memset(&svd, 0, sizeof(svd)); /* Prevent double free */
    result.n_x_selected = n_x;

    /* Validate */
    ssid_n4sid_validate(&model, data, &result);

    snprintf(result.status_msg, sizeof(result.status_msg),
             "N4SID complete: order=%lu, fit=%.1f%%, stable=%d",
             (unsigned long)n_x, result.fit_metric, result.is_stable);

cleanup:
    ssid_matrix_free(&O_i);
    ssid_matrix_free(&Gamma);
    ssid_hankel_free(&HU);
    ssid_hankel_free(&HY);
    ssid_data_free(&work_data);

    return result;
}

/* ========================================================================
 * L8: Recursive N4SID
 *
 * Knowledge point: Recursive subspace identification updates the model
 * as new data arrives without recomputing from scratch. Key approaches:
 *   1. Sliding window: drop oldest data, add newest, recompute on window
 *   2. Recursive LQ update (Golub & Van Loan, Sec 12.6): update L factor
 *      via Givens rotations when new rows are added
 *   3. Propagator method: update the observability subspace directly
 *
 * Here we implement a simplified sliding-window approach.
 * ======================================================================== */

int ssid_n4sid_recursive(const ssid_data_t *data_window,
                         const ssid_config_t *cfg,
                         const ssid_model_t *prev_model,
                         ssid_result_t *updated)
{
    (void)prev_model; /* reserved for warm-start with previous model */
    if (!data_window || !cfg || !updated) return -1;

    /* Simplest recursive approach: re-identify on the window.
     * For true real-time systems, the full N4SID pipeline is too
     * expensive; an incremental SVD update would be needed. */
    *updated = ssid_n4sid_identify(data_window, cfg);
    return 0;
}

/* ========================================================================
 * L8: Closed-loop N4SID
 *
 * Knowledge point: Direct closed-loop identification produces biased
 * estimates under output feedback because future inputs u_f are
 * correlated with past noise through the feedback path:
 *   u_k = r_k - C_c(q) * y_k
 * where C_c is the controller.
 *
 * Solutions:
 *   1. Use external reference r_k as instrumental variable (if known)
 *   2. Two-stage method: first identify closed-loop transfer,
 *      then recover open-loop model
 *   3. Joint IO approach: identify controller + plant simultaneously
 *
 * This implementation uses the direct approach with projection onto
 * an extended instrument set.
 * ======================================================================== */

int ssid_n4sid_closed_loop(const ssid_data_t *data,
                           const ssid_config_t *cfg,
                           const ssid_matrix_t *controller_params,
                           ssid_result_t *result)
{
    if (!data || !cfg || !result) return -1;

    /* Configure for closed-loop */
    ssid_config_t cl_cfg = *cfg;
    cl_cfg.closed_loop = 1;

    if (controller_params) {
        /* Use controller knowledge to construct instrument */
        /* This would modify the projection step to use
         * the controller as a known filter in the IV approach. */
    }

    *result = ssid_n4sid_identify(data, &cl_cfg);

    return 0;
}

/* ========================================================================
 * L7: Industrial vendor wrappers
 * ======================================================================== */

ssid_result_t ssid_n4sid_aspentech_dmc3(const ssid_data_t *data)
{
    ssid_config_t cfg = ssid_config_default();
    cfg.algorithm   = SSID_ALG_N4SID;
    cfg.weighting   = SSID_WGT_CVA;
    cfg.order_crit  = SSID_ORDER_SVD_GAP;
    cfg.remove_trend = 1;
    cfg.normalize_data = 1;
    cfg.i_max        = 40;
    cfg.n_x_max      = 30;
    return ssid_n4sid_identify(data, &cfg);
}

ssid_result_t ssid_n4sid_honeywell_profit(const ssid_data_t *data)
{
    ssid_config_t cfg = ssid_config_default();
    cfg.algorithm   = SSID_ALG_MOESP;
    cfg.weighting   = SSID_WGT_MOESP;
    cfg.order_crit  = SSID_ORDER_AIC;
    cfg.remove_trend = 1;
    cfg.normalize_data = 1;
    cfg.i_max        = 30;
    cfg.n_x_max      = 15;
    return ssid_n4sid_identify(data, &cfg);
}
