/** @file mimo_ops.c
 * @brief MIMO Model Operations (L2)
 * Setting sub-models, DV models, open-loop prediction, SS gain extraction.
 */
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "mpc_common.h"

int mpc_mimo_set_submodel(mpc_mimo_model_t *mimo,
    int cv_idx, int mv_idx, const mpc_step_model_t *sub)
{
    if (!mimo || !sub || cv_idx < 0 || cv_idx >= mimo->n_cv || mv_idx < 0 || mv_idx >= mimo->n_mv) return -1;
    mpc_step_model_t *dst = &mimo->sub_models[cv_idx][mv_idx];
    if (!dst->coeff) {
        dst->coeff = (double*)calloc(sub->n_coeffs, sizeof(double));
        if (!dst->coeff) return -2;
        dst->n_coeffs = sub->n_coeffs;
    }
    memcpy(dst->coeff, sub->coeff, sub->n_coeffs * sizeof(double));
    dst->gain_ss = sub->gain_ss;
    dst->time_constant_sec = sub->time_constant_sec;
    dst->dead_time_samples = sub->dead_time_samples;
    dst->sample_time_sec = sub->sample_time_sec;
    dst->mv_index = mv_idx;
    dst->cv_index = cv_idx;
    if (sub->n_coeffs > mimo->model_horizon) mimo->model_horizon = sub->n_coeffs;
    return 0;
}

int mpc_mimo_set_dvmodel(mpc_mimo_model_t *mimo,
    int cv_idx, int dv_idx, const mpc_step_model_t *sub)
{
    if (!mimo || !sub || cv_idx < 0 || cv_idx >= mimo->n_cv || dv_idx < 0 || dv_idx >= mimo->n_dv) return -1;
    mpc_step_model_t *dst = &mimo->dv_models[cv_idx][dv_idx];
    if (!dst->coeff) {
        dst->coeff = (double*)calloc(sub->n_coeffs, sizeof(double));
        if (!dst->coeff) return -2;
        dst->n_coeffs = sub->n_coeffs;
    }
    memcpy(dst->coeff, sub->coeff, sub->n_coeffs * sizeof(double));
    dst->gain_ss = sub->gain_ss;
    dst->time_constant_sec = sub->time_constant_sec;
    dst->dead_time_samples = sub->dead_time_samples;
    dst->sample_time_sec = sub->sample_time_sec;
    dst->mv_index = dv_idx;
    dst->cv_index = cv_idx;
    return 0;
}

int mpc_mimo_predict_openloop(const mpc_mimo_model_t *mimo,
    const double *delta_u_past, int n_past, double *y_openloop, int n_pred)
{
    if (!mimo || !delta_u_past || !y_openloop || n_past < 1 || n_pred < 1) return -1;
    int n_cv = mimo->n_cv, n_mv = mimo->n_mv;
    memset(y_openloop, 0, n_pred * n_cv * sizeof(double));
    double *y_cv = (double*)calloc(n_pred, sizeof(double));
    if (!y_cv) return -2;
    for (int cv = 0; cv < n_cv; cv++) {
        for (int mv = 0; mv < n_mv; mv++) {
            mpc_step_model_t *sm = &mimo->sub_models[cv][mv];
            if (!sm->coeff || sm->n_coeffs < 1) continue;
            mpc_step_model_predict(sm, delta_u_past, n_past, y_cv, n_pred);
            for (int p = 0; p < n_pred; p++) y_openloop[cv*n_pred+p] += y_cv[p];
        }
    }
    free(y_cv);
    return 0;
}

int mpc_mimo_extract_ss_gain(const mpc_mimo_model_t *mimo, double *G_ss)
{
    if (!mimo || !G_ss) return -1;
    for (int cv = 0; cv < mimo->n_cv; cv++)
        for (int mv = 0; mv < mimo->n_mv; mv++)
            G_ss[cv * mimo->n_mv + mv] = mimo->sub_models[cv][mv].gain_ss;
    return 0;
}

int mpc_mimo_validate(const mpc_mimo_model_t *mimo)
{
    if (!mimo) return -1;
    if (mimo->n_cv < 1 || mimo->n_mv < 0 || mimo->model_horizon < 2) return -2;
    if (!mimo->sub_models) return -3;
    for (int cv = 0; cv < mimo->n_cv; cv++) {
        for (int mv = 0; mv < mimo->n_mv; mv++) {
            int v = mpc_step_model_validate(&mimo->sub_models[cv][mv]);
            if (v != 0 && v != -6) return -4;
        }
    }
    return 0;
}
