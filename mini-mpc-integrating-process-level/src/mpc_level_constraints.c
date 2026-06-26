/**
 * mpc_level_constraints.c — Level Constraint Handling Implementation
 *
 * Implements constraint formulation, feasibility checking, and safety
 * margin calculations per API 2350 and IEC 61511 standards.
 *
 * Knowledge Coverage:
 *   L4 - Standards: API 2350 overfill prevention, IEC 61511 SIS, ISA-18.2
 *   L2 - Core Concepts: hard/soft constraints, feasibility
 *   L3 - Eng. Structures: constraint formulation for QP
 *
 * Reference: API 2350 5th Ed (2020), IEC 61511:2016, ISA-18.2:2016
 */

#include <math.h>
#include <string.h>
#include <stdio.h>
#include "../include/mpc_level_constraints.h"

/* ─── Constraint Set Management ───────────────────────────────────────── */

void mpc_constraint_set_init(mpc_constraint_set_t *cset) {
    if (!cset) return;
    memset(cset, 0, sizeof(*cset));
}

/* ─── Input (MV) Constraints ──────────────────────────────────────────── */

int mpc_constraint_build_input(mpc_constraint_set_t *cset, double u_prev,
                                double u_min, double u_max, int N_c) {
    if (!cset || N_c < 1) return 0;

    int added = 0;
    for (int j = 0; j < N_c; j++) {
        if (cset->num_constraints >= MPC_MAX_CONSTRAINTS) break;

        /* Σ_{l=0}^{j} Δu(k+l) ≤ u_max - u_prev */
        mpc_constraint_spec_t *cs = &cset->specs[cset->num_constraints];
        memset(cs->coefficients, 0, sizeof(cs->coefficients));
        for (int l = 0; l <= j; l++) cs->coefficients[l] = 1.0;
        cs->bound = u_max - u_prev;
        cs->type = CONSTRAINT_HARD_INPUT;
        cs->soft = 0;
        cset->num_constraints++;
        cset->num_hard++;
        added++;

        if (cset->num_constraints >= MPC_MAX_CONSTRAINTS) break;

        /* -Σ_{l=0}^{j} Δu(k+l) ≤ -(u_min - u_prev) */
        cs = &cset->specs[cset->num_constraints];
        memset(cs->coefficients, 0, sizeof(cs->coefficients));
        for (int l = 0; l <= j; l++) cs->coefficients[l] = -1.0;
        cs->bound = -(u_min - u_prev);
        cs->type = CONSTRAINT_HARD_INPUT;
        cs->soft = 0;
        cset->num_constraints++;
        cset->num_hard++;
        added++;
    }
    return added;
}

/* ─── Rate Constraints ────────────────────────────────────────────────── */

int mpc_constraint_build_rate(mpc_constraint_set_t *cset, double du_min,
                               double du_max, int N_c) {
    if (!cset || N_c < 1) return 0;

    int added = 0;
    for (int j = 0; j < N_c; j++) {
        if (cset->num_constraints >= MPC_MAX_CONSTRAINTS) break;

        /* Δu_j ≤ du_max */
        mpc_constraint_spec_t *cs = &cset->specs[cset->num_constraints];
        memset(cs->coefficients, 0, sizeof(cs->coefficients));
        cs->coefficients[j] = 1.0;
        cs->bound = du_max;
        cs->type = CONSTRAINT_RATE_OF_CHANGE;
        cs->soft = 0;
        cset->num_constraints++;
        cset->num_hard++;
        added++;

        if (cset->num_constraints >= MPC_MAX_CONSTRAINTS) break;

        /* -Δu_j ≤ -du_min → Δu_j ≥ du_min */
        cs = &cset->specs[cset->num_constraints];
        memset(cs->coefficients, 0, sizeof(cs->coefficients));
        cs->coefficients[j] = -1.0;
        cs->bound = -du_min;
        cs->type = CONSTRAINT_RATE_OF_CHANGE;
        cs->soft = 0;
        cset->num_constraints++;
        cset->num_hard++;
        added++;
    }
    return added;
}

/* ─── Output (Level) Constraints ──────────────────────────────────────── */

int mpc_constraint_build_output(mpc_constraint_set_t *cset,
                                 const double *G, const double *f,
                                 double y_min, double y_max,
                                 int N_p, int N_c, int start_i) {
    if (!cset || !G || !f || N_p < 1 || N_c < 1) return 0;

    int added = 0;
    for (int i = start_i; i < N_p; i++) {
        if (cset->num_constraints + 2 > MPC_MAX_CONSTRAINTS) break;

        /* G_i^T * Δu ≤ y_max - f_i */
        mpc_constraint_spec_t *cs = &cset->specs[cset->num_constraints];
        memset(cs->coefficients, 0, sizeof(cs->coefficients));
        for (int j = 0; j < N_c; j++) {
            cs->coefficients[j] = G[j * N_p + i];
        }
        cs->bound = y_max - f[i];
        cs->type = CONSTRAINT_HARD_OUTPUT;
        cs->soft = 0;
        cset->num_constraints++;
        cset->num_hard++;
        added++;

        if (cset->num_constraints >= MPC_MAX_CONSTRAINTS) break;

        /* -G_i^T * Δu ≤ -(y_min - f_i) */
        cs = &cset->specs[cset->num_constraints];
        memset(cs->coefficients, 0, sizeof(cs->coefficients));
        for (int j = 0; j < N_c; j++) {
            cs->coefficients[j] = -G[j * N_p + i];
        }
        cs->bound = -(y_min - f[i]);
        cs->type = CONSTRAINT_HARD_OUTPUT;
        cs->soft = 0;
        cset->num_constraints++;
        cset->num_hard++;
        added++;
    }
    return added;
}

/* ─── Terminal Constraint ──────────────────────────────────────────────── */

int mpc_constraint_build_terminal(mpc_constraint_set_t *cset,
                                   const double *G_last, double f_last,
                                   double setpoint, int N_c) {
    if (!cset || !G_last || N_c < 1) return 0;
    if (cset->num_constraints + 2 > MPC_MAX_CONSTRAINTS) return 0;

    /* ŷ(k+N_p) = r → G_last^T * Δu = setpoint - f_last
     * Implemented as two inequalities with tight tolerance:
     *   G_last^T * Δu ≤ setpoint - f_last + ε
     *  -G_last^T * Δu ≤ -(setpoint - f_last) + ε
     */
    double eps = 1e-6;
    int added = 0;

    mpc_constraint_spec_t *cs1 = &cset->specs[cset->num_constraints];
    memset(cs1->coefficients, 0, sizeof(cs1->coefficients));
    for (int j = 0; j < N_c; j++) cs1->coefficients[j] = G_last[j];
    cs1->bound = setpoint - f_last + eps;
    cs1->type = CONSTRAINT_TERMINAL;
    cs1->soft = 0;
    cset->num_constraints++;
    added++;

    if (cset->num_constraints < MPC_MAX_CONSTRAINTS) {
        mpc_constraint_spec_t *cs2 = &cset->specs[cset->num_constraints];
        memset(cs2->coefficients, 0, sizeof(cs2->coefficients));
        for (int j = 0; j < N_c; j++) cs2->coefficients[j] = -G_last[j];
        cs2->bound = -(setpoint - f_last) + eps;
        cs2->type = CONSTRAINT_TERMINAL;
        cs2->soft = 0;
        cset->num_constraints++;
        added++;
    }

    return added;
}

/* ─── Feasibility ──────────────────────────────────────────────────────── */

int mpc_feasibility_check(const mpc_constraint_set_t *cset,
                           const double *G, const double *f,
                           int N_p, int N_c) {
    if (!cset || !G || !f || N_p < 1 || N_c < 1) return 0;

    /* Simple check: test if there are contradicting input constraints */
    for (int j = 0; j < N_c; j++) {
        double lb = -1e10, ub = 1e10;
        for (int k = 0; k < cset->num_constraints; k++) {
            double a = cset->specs[k].coefficients[j];
            double b = cset->specs[k].bound;
            if (fabs(a) > 1e-10) {
                if (a > 0) {
                    double bound = b / a;
                    if (bound < ub) ub = bound;
                } else {
                    double bound = b / a; /* a < 0, division flips */
                    if (bound > lb) lb = bound;
                }
            }
        }
        if (lb > ub + 1e-6) return 0; /* infeasible */
    }

    return 1;
}

int mpc_feasibility_recover(mpc_constraint_set_t *cset,
                             const mpc_level_config_t *config) {
    if (!cset || !config) return 0;

    int recovered = 0;

    /* Strategy 1: Convert hard output constraints to soft */
    for (int k = 0; k < cset->num_constraints; k++) {
        if (cset->specs[k].type == CONSTRAINT_HARD_OUTPUT) {
            cset->specs[k].soft = 1;
            cset->specs[k].soft_weight = 100.0;
            cset->specs[k].bound += 0.05; /* 5% slack margin */
            cset->num_hard--;
            cset->num_soft++;
            recovered++;
        }
    }

    /* Strategy 2: Widen output bounds using hi-hi limit as emergency margin */
    if (config->level_hihi_limit > config->level_hi_limit) {
        double emergency_margin = config->level_hihi_limit - config->level_hi_limit;
        for (int k = 0; k < cset->num_constraints; k++) {
            if (cset->specs[k].type == CONSTRAINT_HARD_OUTPUT &&
                cset->specs[k].coefficients[0] > 0) {
                /* This is likely a high constraint, relax */
                cset->specs[k].bound += emergency_margin;
                cset->specs[k].soft = 1;
                recovered++;
            }
        }
    }

    return (recovered > 0) ? 1 : 0;
}

/* ─── Safety Margins ──────────────────────────────────────────────────── */

double mpc_safety_margin_api2350(const api2350_overfill_config_t *config,
                                  double max_fill_rate) {
    if (!config) return 1.0;

    double margin = max_fill_rate * config->response_time;
    margin += 2.0 * config->sensor_tolerance;

    /* Category-dependent additional margins */
    switch (config->category) {
        case API2350_CAT_III:
            margin *= 1.5; /* 50% extra for highest risk */
            break;
        case API2350_CAT_II:
            margin *= 1.2;
            break;
        default:
            break;
    }

    return margin;
}

double mpc_sil_margin(const iec61511_sil_config_t *sil, double max_fill_rate) {
    if (!sil || max_fill_rate <= 0.0) return 0.5;

    /* Base margin on response time implied by SIL */
    double base_margin = max_fill_rate * 30.0; /* 30 second base */

    switch (sil->sil_level) {
        case SIL_4: base_margin *= 2.0; break;
        case SIL_3: base_margin *= 1.5; break;
        case SIL_2: base_margin *= 1.2; break;
        case SIL_1: base_margin *= 1.0; break;
        default:    base_margin = 0.1;  break;
    }

    return base_margin;
}

/* ─── Active Constraint Counting ──────────────────────────────────────── */

int mpc_constraint_count_active(const mpc_constraint_set_t *cset,
                                 const double *x_sol, double tol) {
    if (!cset || !x_sol) return 0;

    int active = 0;
    for (int k = 0; k < cset->num_constraints; k++) {
        double ax = 0.0;
        for (int j = 0; j < MPC_MAX_QP_VARS; j++) {
            ax += cset->specs[k].coefficients[j] * x_sol[j];
        }
        if (fabs(ax - cset->specs[k].bound) < tol) active++;
    }
    return active;
}
