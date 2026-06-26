/**
 * @file mpc_common.c
 * @brief MPC Common: validation, initialization, defaults
 *
 * @knowledge L1: Parameter validation and default initialization
 */

#include "mpc_common.h"

int mpc_validate_tuning(const mpc_tuning_t *tuning)
{
    int i;
    if (!tuning) return -1;
    if (tuning->np <= 0 || tuning->np > MPC_MAX_NP) return -1;
    if (tuning->nc <= 0 || tuning->nc > MPC_MAX_NC) return -1;
    if (tuning->nc > tuning->np) return -1;
    if (tuning->ts <= 0.0) return -1;

    for (i = 0; i < MPC_MAX_NY; i++)
        if (tuning->q_weight[i] < 0.0) return -1;
    for (i = 0; i < MPC_MAX_NU; i++)
        if (tuning->r_weight[i] < 0.0) return -1;
    for (i = 0; i < MPC_MAX_NY; i++)
        if (tuning->s_weight[i] < 0.0) return -1;
    if (tuning->rho_soft_constraint < 0.0) return -1;

    return 0;
}

int mpc_validate_model(const mpc_ss_model_t *model)
{
    if (!model) return -1;
    if (model->nx <= 0 || model->nx > MPC_MAX_NX) return -1;
    if (model->nu <= 0 || model->nu > MPC_MAX_NU) return -1;
    if (model->ny <= 0 || model->ny > MPC_MAX_NY) return -1;
    if (model->nd < 0 || model->nd > MPC_MAX_ND) return -1;
    return 0;
}

void mpc_tuning_init_default(mpc_tuning_t *tuning)
{
    int i;
    if (!tuning) return;

    tuning->np = 20;
    tuning->nc = 5;
    tuning->ts = 0.1;
    tuning->rho_soft_constraint = 1000.0;
    tuning->enable_terminal_constraint = 0;
    tuning->enable_rate_constraint = 0;

    for (i = 0; i < MPC_MAX_NY; i++)
        tuning->q_weight[i] = 1.0;
    for (i = 0; i < MPC_MAX_NU; i++)
        tuning->r_weight[i] = 0.1;
    for (i = 0; i < MPC_MAX_NY; i++)
        tuning->s_weight[i] = 10.0;
    for (i = 0; i < MPC_MAX_NU; i++)
        tuning->rdelta_weight[i] = 0.0;
}

void mpc_model_init(mpc_ss_model_t *model, int nx, int nu, int ny, int nd)
{
    int i, j;
    if (!model) return;

    model->nx = nx;
    model->nu = nu;
    model->ny = ny;
    model->nd = nd;

    for (i = 0; i < MPC_MAX_NX; i++)
        for (j = 0; j < MPC_MAX_NX; j++)
            model->A[i][j] = (i == j) ? 1.0 : 0.0;
    for (i = 0; i < MPC_MAX_NX; i++)
        for (j = 0; j < MPC_MAX_NU; j++)
            model->B[i][j] = 0.0;
    for (i = 0; i < MPC_MAX_NY; i++)
        for (j = 0; j < MPC_MAX_NX; j++)
            model->C[i][j] = 0.0;
    for (i = 0; i < MPC_MAX_NY; i++)
        for (j = 0; j < MPC_MAX_NU; j++)
            model->D[i][j] = 0.0;
    for (i = 0; i < MPC_MAX_NX; i++)
        for (j = 0; j < MPC_MAX_ND; j++)
            model->Bd[i][j] = 0.0;
    for (i = 0; i < MPC_MAX_NY; i++)
        for (j = 0; j < MPC_MAX_ND; j++)
            model->Cd[i][j] = 0.0;
}

void mpc_constraints_init_default(mpc_constraints_t *cons, int nu, int ny, int nx)
{
    int i;
    if (!cons) return;

    for (i = 0; i < MPC_MAX_NU; i++) {
        cons->u_min[i] = -1e6;
        cons->u_max[i] = 1e6;
        cons->du_min[i] = -1e6;
        cons->du_max[i] = 1e6;
    }
    for (i = 0; i < MPC_MAX_NY; i++) {
        cons->y_min[i] = -1e6;
        cons->y_max[i] = 1e6;
    }
    for (i = 0; i < MPC_MAX_NX; i++) {
        cons->x_min[i] = -1e6;
        cons->x_max[i] = 1e6;
    }
    cons->enable_u = 0;
    cons->enable_du = 0;
    cons->enable_y = 0;
    cons->enable_x = 0;
    cons->use_soft_y = 0;
    cons->use_soft_x = 0;

    (void)nu; (void)ny; (void)nx;
}
