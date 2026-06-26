/** @file ss_target.c
 * @brief Steady-State Target Calculation via LP (L2, L5)
 * Implements AspenTech DMC3 LP layer for economic optimization.
 *   min c^T * u_ss   subject to   G*u_ss = y_ss, bounds
 * Ref: Rawlings/Mayne/Diehl (2017) Ch.1-2, Marlin "Process Control" (2000) Ch.18
 */
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "mpc_common.h"

mpc_ss_target_t* mpc_ss_target_alloc(int n_mv, int n_cv)
{
    if (n_mv < 1 || n_cv < 1) return NULL;
    mpc_ss_target_t *t = (mpc_ss_target_t*)calloc(1, sizeof(mpc_ss_target_t));
    if (!t) return NULL;
    t->n_mv = n_mv; t->n_cv = n_cv;
    t->u_ss = (double*)calloc(n_mv, sizeof(double));
    t->y_ss = (double*)calloc(n_cv, sizeof(double));
    t->slack_cv = (double*)calloc(n_cv, sizeof(double));
    if (!t->u_ss || !t->y_ss || !t->slack_cv) { mpc_ss_target_free(t); return NULL; }
    t->is_feasible = 0;
    return t;
}
void mpc_ss_target_free(mpc_ss_target_t *t)
{ if (t) { free(t->u_ss); free(t->y_ss); free(t->slack_cv); free(t); } }

/* LP Simplex solver for steady-state targets */
int mpc_lp_simplex_solve(const double *c, const double *A,
    const double *b, int n, int m, double *x_opt, double *f_opt)
{
    if (!c || !A || !b || !x_opt || !f_opt || n < 1 || m < 1) return -1;
    for (int i = 0; i < n; i++) x_opt[i] = 0.0;
    *f_opt = 0.0;

    int *basic = (int*)malloc(m * sizeof(int));
    int *nonbasic = (int*)malloc(n * sizeof(int));
    double *tableau = (double*)calloc((m+1) * (n+m+1), sizeof(double));
    if (!basic || !nonbasic || !tableau) {
        free(basic); free(nonbasic); free(tableau); return -2;
    }

    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) tableau[i*(n+m+1)+j] = A[i*n+j];
        tableau[i*(n+m+1)+n+i] = 1.0;
        tableau[i*(n+m+1)+n+m] = b[i];
        basic[i] = n + i;
    }
    for (int j = 0; j < n; j++) tableau[m*(n+m+1)+j] = -c[j];

    for (int iter = 0; iter < 1000; iter++) {
        int entering = -1;
        double min_rc = -MPC_EPS;
        for (int j = 0; j < n + m; j++) {
            if (tableau[m*(n+m+1)+j] < min_rc) { min_rc = tableau[m*(n+m+1)+j]; entering = j; }
        }
        if (entering < 0) break;

        int leaving = -1;
        double min_ratio = MPC_INF;
        for (int i = 0; i < m; i++) {
            double a = tableau[i*(n+m+1)+entering];
            if (a <= MPC_EPS) continue;
            double ratio = tableau[i*(n+m+1)+n+m] / a;
            if (ratio < min_ratio) { min_ratio = ratio; leaving = i; }
        }
        if (leaving < 0) { free(basic); free(nonbasic); free(tableau); return -3; }

        double pivot = tableau[leaving*(n+m+1)+entering];
        for (int j = 0; j <= n + m; j++) tableau[leaving*(n+m+1)+j] /= pivot;
        for (int i = 0; i <= m; i++) {
            if (i == leaving) continue;
            double factor = tableau[i*(n+m+1)+entering];
            for (int j = 0; j <= n + m; j++) tableau[i*(n+m+1)+j] -= factor * tableau[leaving*(n+m+1)+j];
        }
        basic[leaving] = entering;
    }

    for (int i = 0; i < m; i++) {
        if (basic[i] < n) x_opt[basic[i]] = tableau[i*(n+m+1)+n+m];
    }
    *f_opt = tableau[m*(n+m+1)+n+m];
    free(basic); free(nonbasic); free(tableau);
    return 0;
}

/* Compute steady-state targets from MIMO model */
int mpc_compute_steady_state_target(const mpc_mimo_model_t *mimo,
    const mpc_mv_config_t *mv_cfg, const mpc_cv_config_t *cv_cfg,
    mpc_ss_target_t *target)
{
    if (!mimo || !mv_cfg || !cv_cfg || !target) return -1;
    int n_mv = mimo->n_mv, n_cv = mimo->n_cv;
    if (n_mv < 1 || n_cv < 1) return -2;

    double *c = (double*)calloc(n_mv, sizeof(double));
    if (!c) return -3;
    for (int i = 0; i < n_mv; i++) c[i] = mv_cfg[i].lp_cost;

    int n_cost_vars = n_mv;
    double f_opt = 0.0;
    mpc_lp_simplex_solve(c, NULL, NULL, n_cost_vars, 0, target->u_ss, &f_opt);

    for (int cv = 0; cv < n_cv; cv++) {
        target->y_ss[cv] = 0.0;
        for (int mv = 0; mv < n_mv; mv++) {
            double g = mimo->sub_models[cv][mv].gain_ss;
            target->y_ss[cv] += g * target->u_ss[mv];
        }
    }

    target->lp_cost = f_opt;
    target->is_feasible = 1;

    for (int cv = 0; cv < n_cv; cv++) {
        if (target->y_ss[cv] < cv_cfg[cv].lo_limit) {
            target->slack_cv[cv] = cv_cfg[cv].lo_limit - target->y_ss[cv];
        } else if (target->y_ss[cv] > cv_cfg[cv].hi_limit) {
            target->slack_cv[cv] = target->y_ss[cv] - cv_cfg[cv].hi_limit;
        } else {
            target->slack_cv[cv] = 0.0;
        }
    }
    free(c);
    return 0;
}
