/**
 * @file mpc_constraints.c
 * @brief MPC Constraint Management
 *
 * Implements constraint handling for MPC: box constraints,
 * rate constraints, soft constraints, and terminal sets.
 *
 * Reference: Maciejowski, "Predictive Control with Constraints" (2002)
 * Reference: Mayne et al., "Constrained MPC: Stability and Optimality", Automatica (2000)
 *
 * @knowledge L2: Constraint types in MPC
 * @knowledge L4: Constraint qualification and feasibility
 * @knowledge L5: Soft constraint handling
 */

#include "mpc_common.h"
#include "mpc_model.h"
#include <string.h>

void mpc_constraints_set_input(mpc_constraints_t *cons,
                                const double u_min[MPC_MAX_NU],
                                const double u_max[MPC_MAX_NU],
                                int nu)
{
    int i;
    if (!cons) return;
    for (i = 0; i < nu && i < MPC_MAX_NU; i++) {
        cons->u_min[i] = u_min[i];
        cons->u_max[i] = u_max[i];
    }
    cons->enable_u = 1;
}

void mpc_constraints_set_rate(mpc_constraints_t *cons,
                               const double du_min[MPC_MAX_NU],
                               const double du_max[MPC_MAX_NU],
                               int nu)
{
    int i;
    if (!cons) return;
    for (i = 0; i < nu && i < MPC_MAX_NU; i++) {
        cons->du_min[i] = du_min[i];
        cons->du_max[i] = du_max[i];
    }
    cons->enable_du = 1;
}

void mpc_constraints_set_output(mpc_constraints_t *cons,
                                 const double y_min[MPC_MAX_NY],
                                 const double y_max[MPC_MAX_NY],
                                 int ny, int use_soft)
{
    int i;
    if (!cons) return;
    for (i = 0; i < ny && i < MPC_MAX_NY; i++) {
        cons->y_min[i] = y_min[i];
        cons->y_max[i] = y_max[i];
    }
    cons->enable_y = 1;
    cons->use_soft_y = use_soft;
}

void mpc_constraints_set_state(mpc_constraints_t *cons,
                                const double x_min[MPC_MAX_NX],
                                const double x_max[MPC_MAX_NX],
                                int nx, int use_soft)
{
    int i;
    if (!cons) return;
    for (i = 0; i < nx && i < MPC_MAX_NX; i++) {
        cons->x_min[i] = x_min[i];
        cons->x_max[i] = x_max[i];
    }
    cons->enable_x = 1;
    cons->use_soft_x = use_soft;
}

int mpc_constraints_check_input(const mpc_constraints_t *cons,
                                 const double u[MPC_MAX_NU], int nu)
{
    int i;
    if (!cons || !u) return 0;
    if (!cons->enable_u) return 1;
    for (i = 0; i < nu && i < MPC_MAX_NU; i++) {
        if (u[i] < cons->u_min[i] || u[i] > cons->u_max[i])
            return 0;
    }
    return 1;
}

int mpc_constraints_check_rate(const mpc_constraints_t *cons,
                                const double u[MPC_MAX_NU],
                                const double u_prev[MPC_MAX_NU],
                                int nu)
{
    int i;
    double du;
    if (!cons || !u || !u_prev) return 0;
    if (!cons->enable_du) return 1;
    for (i = 0; i < nu && i < MPC_MAX_NU; i++) {
        du = u[i] - u_prev[i];
        if (du < cons->du_min[i] || du > cons->du_max[i])
            return 0;
    }
    return 1;
}

void mpc_constraints_project_input(const mpc_constraints_t *cons,
                                    double u[MPC_MAX_NU], int nu)
{
    int i;
    if (!cons || !u) return;
    if (!cons->enable_u) return;
    for (i = 0; i < nu && i < MPC_MAX_NU; i++) {
        if (u[i] < cons->u_min[i]) u[i] = cons->u_min[i];
        if (u[i] > cons->u_max[i]) u[i] = cons->u_max[i];
    }
}

double mpc_constraints_violation_input(const mpc_constraints_t *cons,
                                        const double u[MPC_MAX_NU], int nu)
{
    int i;
    double viol = 0.0, v;
    if (!cons || !u) return 0.0;
    if (!cons->enable_u) return 0.0;
    for (i = 0; i < nu && i < MPC_MAX_NU; i++) {
        v = cons->u_min[i] - u[i];
        if (v > viol) viol = v;
        v = u[i] - cons->u_max[i];
        if (v > viol) viol = v;
    }
    return viol;
}

int mpc_constraints_terminal_set(const mpc_aug_model_t *aug,
                                  const mpc_tuning_t *tuning,
                                  double terminal_bound[MPC_MAX_NX + MPC_MAX_ND])
{
    int n, i, j;
    double Q_dare[MPC_MAX_NX][MPC_MAX_NX];
    double R_dare[MPC_MAX_NU][MPC_MAX_NU];
    double P_dare[MPC_MAX_NX][MPC_MAX_NX];
    int max_iter = 500;
    double tol = 1e-8;

    if (!aug || !tuning || !terminal_bound) return -1;
    n = aug->nx_aug;
    if (n > MPC_MAX_NX) return -1;

    for (i = 0; i < n && i < MPC_MAX_NX; i++)
        for (j = 0; j < n && j < MPC_MAX_NX; j++)
            Q_dare[i][j] = (i == j) ? 1.0 : 0.0;

    for (i = 0; i < aug->nu && i < MPC_MAX_NU; i++)
        for (j = 0; j < aug->nu && j < MPC_MAX_NU; j++)
            R_dare[i][j] = (i == j) ? tuning->r_weight[i] : 0.0;

    if (aug->nx_aug > MPC_MAX_NX) {
        for (i = 0; i < n && i < MPC_MAX_NX; i++)
            terminal_bound[i] = 1.0;
        return 0;
    }

    int ret = mpc_solve_dare(aug->Aa, aug->Ba, Q_dare, R_dare, n, aug->nu,
                              P_dare, max_iter, tol);
    if (ret < 0) {
        for (i = 0; i < n; i++) terminal_bound[i] = 1.0;
        return -1;
    }

    double alpha = 10.0;
    for (i = 0; i < n; i++) {
        if (P_dare[i][i] > 1e-10)
            terminal_bound[i] = sqrt(alpha / P_dare[i][i]);
        else
            terminal_bound[i] = 1.0;
    }
    return 0;
}

int mpc_constraints_in_terminal_set(const mpc_constraints_t *cons,
                                     const double x[MPC_MAX_NX + MPC_MAX_ND],
                                     int nx)
{
    int i;
    if (!cons || !x) return 0;
    if (!cons->enable_x) return 1;
    for (i = 0; i < nx && i < MPC_MAX_NX; i++) {
        if (x[i] < cons->x_min[i] || x[i] > cons->x_max[i])
            return 0;
    }
    return 1;
}

double mpc_constraints_max_violation(const mpc_constraints_t *cons,
                                      const mpc_solution_t *sol,
                                      int nu, int ny, int nx)
{
    int i, j;
    double max_viol = 0.0;
    (void)nx;

    if (!cons || !sol) return 0.0;
    if (cons->enable_u) {
        for (i = 0; i < 10; i++) {
            double viol = mpc_constraints_violation_input(cons, sol->u_seq[i], nu);
            if (viol > max_viol) max_viol = viol;
        }
    }
    if (cons->enable_y) {
        for (i = 0; i < 20; i++) {
            for (j = 0; j < ny && j < MPC_MAX_NY; j++) {
                double v;
                v = cons->y_min[j] - sol->y_pred[i][j];
                if (v > max_viol) max_viol = v;
                v = sol->y_pred[i][j] - cons->y_max[j];
                if (v > max_viol) max_viol = v;
            }
        }
    }
    return max_viol;
}
