/**
 * @file mpc_controller.c
 * @brief Main MPC Controller with Disturbance Feedforward
 *
 * Integrates all MPC components into a complete receding-horizon
 * controller with disturbance rejection capability.
 *
 * Reference: Rawlings, Mayne, Diehl (2017), Chapters 1-4
 * Reference: Qin and Badgwell, "A Survey of Industrial MPC Technology",
 *            Control Engineering Practice (2003)
 *
 * Course Mappings:
 * - MIT 6.302: Full-state feedback control
 * - Stanford ENGR205: MPC implementation
 *
 * @knowledge L1: MPC controller definition
 * @knowledge L2: Receding horizon implementation
 * @knowledge L6: CSTR temperature control
 * @knowledge L7: Industrial MPC (DMCplus, RMPCT)
 */

#include "mpc_controller.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

int mpc_controller_init(mpc_controller_t *ctl,
                         const mpc_ss_model_t *plant,
                         const mpc_tuning_t *tuning,
                         mpc_mode_t mode)
{
    int ret;

    if (!ctl || !plant || !tuning) return -1;
    if (mpc_validate_model(plant) != 0) return -1;
    if (mpc_validate_tuning(tuning) != 0) return -1;

    memset(ctl, 0, sizeof(mpc_controller_t));

    /* Copy plant model */
    memcpy(&ctl->plant, plant, sizeof(mpc_ss_model_t));
    memcpy(&ctl->tuning, tuning, sizeof(mpc_tuning_t));
    ctl->mode = mode;
    ctl->flags = MPC_FLAG_WARM_START | MPC_FLAG_USE_OBSERVER
               | MPC_FLAG_OFFSET_FREE | MPC_FLAG_USE_FF;

    /* Build augmented model */
    ret = mpc_build_augmented_model(plant, mode, &ctl->aug_model);
    if (ret != 0) return -1;

    /* Build prediction matrices */
    ret = mpc_build_prediction_matrices(&ctl->aug_model, tuning, &ctl->prediction);
    if (ret != 0) return -1;

    /* Initialize default constraints */
    mpc_constraints_init_default(&ctl->constraints, plant->nu, plant->ny, plant->nx);

    /* Initialize observer */
    ctl->observer_type = MPC_OBSERVER_KALMAN;
    mpc_kalman_init(&ctl->kalman, ctl->aug_model.nx_aug, plant->ny);

    /* Initialize QP solver */
    ctl->qp_solver_type = MPC_QP_ACTIVE_SET;
    mpc_qp_init(&ctl->qp);
    mpc_qp_build_hessian(&ctl->prediction, tuning, &ctl->qp);

    /* Initialize feedforward */
    memset(&ctl->ff_config, 0, sizeof(mpc_ff_config_t));
    ctl->ff_config.ff_gain_scale = 1.0;

    /* Initialize state */
    memset(&ctl->state, 0, sizeof(mpc_state_t));
    memset(&ctl->observer_state, 0, sizeof(mpc_observer_state_t));

    ctl->initialized = 1;
    return 0;
}

int mpc_controller_set_constraints(mpc_controller_t *ctl,
                                    const mpc_constraints_t *cons)
{
    if (!ctl || !cons) return -1;
    memcpy(&ctl->constraints, cons, sizeof(mpc_constraints_t));
    return 0;
}

void mpc_controller_set_reference(mpc_controller_t *ctl,
                                   const double y_ref[MPC_MAX_NY])
{
    int i;
    if (!ctl || !y_ref) return;
    for (i = 0; i < MPC_MAX_NY; i++)
        ctl->state.y_ref[i] = y_ref[i];
}

void mpc_controller_set_reference_trajectory(mpc_controller_t *ctl,
                                              const double Y_ref[MPC_MAX_NP][MPC_MAX_NY])
{
    int i, j;
    if (!ctl || !Y_ref) return;
    for (i = 0; i < MPC_MAX_NP; i++)
        for (j = 0; j < MPC_MAX_NY; j++)
            ctl->state.y_ref[j] = Y_ref[i][j];
}

int mpc_controller_step(mpc_controller_t *ctl,
                         const double y_meas[MPC_MAX_NY],
                         const double d_meas[MPC_MAX_ND],
                         double u_out[MPC_MAX_NU])
{
    int i, j, nu, nx_aug, ret;
    double u_fb[MPC_MAX_NU];
    double u_ff[MPC_MAX_NU];

    if (!ctl || !ctl->initialized || !y_meas || !u_out) return -1;
    nu = ctl->plant.nu;
    nx_aug = ctl->aug_model.nx_aug;

    /* Step 1: Update state estimate using Kalman filter */
    if (ctl->flags & MPC_FLAG_USE_OBSERVER) {
        mpc_kalman_predict(&ctl->aug_model, &ctl->kalman,
                           ctl->state.u_prev, &ctl->observer_state);
        mpc_kalman_update(&ctl->aug_model, &ctl->kalman, y_meas,
                          &ctl->observer_state);

        /* Copy estimated state to controller state */
        for (i = 0; i < nx_aug && i < MPC_MAX_NX + MPC_MAX_ND; i++)
            ctl->state.x_aug[i] = ctl->observer_state.x_aug_hat[i];
        for (i = 0; i < ctl->plant.nx && i < MPC_MAX_NX; i++)
            ctl->state.x[i] = ctl->observer_state.x_aug_hat[i];
    }

    /* Step 2: Compute feedforward */
    memset(u_ff, 0, sizeof(u_ff));
    if (ctl->flags & MPC_FLAG_USE_FF) {
        mpc_ff_compute_static(&ctl->ff_config, d_meas, u_ff);
    }

    /* Step 3: Update stored disturbances */
    for (i = 0; i < MPC_MAX_ND && i < ctl->plant.nd; i++) {
        ctl->state.d_meas[i] = d_meas[i];
    }

    /* Step 4: Build QP gradient for this time step */
    double d_pred[MPC_MAX_NP * MPC_MAX_ND];
    memset(d_pred, 0, sizeof(d_pred));
    mpc_qp_build_gradient(&ctl->prediction, &ctl->aug_model, &ctl->tuning,
                          ctl->state.x_aug, ctl->state.y_ref, d_pred, &ctl->qp);

    /* Step 5: Build constraint matrices */
    mpc_qp_build_constraints(&ctl->prediction, &ctl->constraints,
                              ctl->state.x_aug, ctl->state.u_prev,
                              &ctl->tuning, &ctl->qp);

    /* Step 6: Solve QP */
    ret = mpc_qp_solve_active_set(&ctl->qp, &ctl->working_set);
    ctl->solution.status = (mpc_solve_status_t)ret;
    ctl->solution.iterations = 1;

    if (ret == MPC_SOLVE_SUCCESS || ret == MPC_SOLVE_MAX_ITER) {
        /* Extract solution: DeltaU optimal sequence */
        for (i = 0; i < ctl->tuning.nc && i < MPC_MAX_NC; i++)
            for (j = 0; j < nu && j < MPC_MAX_NU; j++)
                ctl->solution.u_seq[i][j] = ctl->qp.z_opt[i * nu + j];

        /* Compute feedback input: u_fb = u_prev + DeltaU[0] */
        for (i = 0; i < nu && i < MPC_MAX_NU; i++) {
            u_fb[i] = ctl->state.u_prev[i] + ctl->solution.u_seq[0][i];
            ctl->solution.u_apply[i] = u_fb[i];
        }

        /* Predict trajectory */
        for (i = 0; i < ctl->tuning.np && i < MPC_MAX_NP; i++) {
            for (j = 0; j < ctl->plant.ny && j < MPC_MAX_NY; j++) {
                double pred_y = 0.0;
                int k;
                for (k = 0; k < nx_aug; k++)
                    pred_y += ctl->prediction.Phi[i * ctl->plant.ny + j][k]
                              * ctl->state.x_aug[k];
                int mm;
                for (mm = 0; mm < (i < ctl->tuning.nc ? i + 1 : ctl->tuning.nc); mm++)
                    for (k = 0; k < nu; k++)
                        pred_y += ctl->prediction.Gamma[i * ctl->plant.ny + j][mm * nu + k]
                                  * ctl->solution.u_seq[mm][k];
                ctl->solution.y_pred[i][j] = pred_y;
            }
        }

        ctl->solution.cost = ctl->qp.obj_value;
    } else {
        /* QP failed: hold last input */
        for (i = 0; i < nu && i < MPC_MAX_NU; i++)
            u_fb[i] = ctl->state.u_prev[i];
    }

    /* Step 7: Combine FF and FB with anti-windup */
    mpc_ff_combine(u_fb, u_ff, ctl->constraints.u_min,
                   ctl->constraints.u_max, u_out, nu);

    /* Step 8: Update controller state for next iteration */
    for (i = 0; i < nu && i < MPC_MAX_NU; i++) {
        ctl->state.u_prev[i] = u_out[i];
        ctl->state.u_ff[i] = u_ff[i];
    }

    /* Compute performance metrics */
    mpc_controller_compute_metrics(ctl, y_meas);

    ctl->state.k++;

    return 0;
}

void mpc_controller_reset(mpc_controller_t *ctl)
{
    if (!ctl) return;
    memset(&ctl->state, 0, sizeof(mpc_state_t));
    memset(&ctl->observer_state, 0, sizeof(mpc_observer_state_t));
    memset(&ctl->solution, 0, sizeof(mpc_solution_t));
    memset(&ctl->stats, 0, sizeof(mpc_runtime_stats_t));
    mpc_kalman_init(&ctl->kalman, ctl->aug_model.nx_aug, ctl->plant.ny);
}

void mpc_controller_compute_metrics(mpc_controller_t *ctl,
                                     const double y_meas[MPC_MAX_NY])
{
    int i;
    double error;
    if (!ctl || !y_meas) return;

    ctl->stats.total_steps++;

    for (i = 0; i < ctl->plant.ny && i < MPC_MAX_NY; i++) {
        error = y_meas[i] - ctl->state.y_ref[i];
        ctl->stats.ise += error * error;
        ctl->stats.iae += fabs(error);
    }

    /* Total variation of input */
    for (i = 0; i < ctl->plant.nu && i < MPC_MAX_NU; i++) {
        ctl->stats.tvu += fabs(ctl->state.u_prev[i] - ctl->solution.u_apply[i]);
    }

    if (ctl->solution.status == MPC_SOLVE_INFEASIBLE)
        ctl->stats.infeasible_count++;
}

const char* mpc_controller_status_string(const mpc_controller_t *ctl)
{
    if (!ctl) return "NULL";
    if (!ctl->initialized) return "NOT_INITIALIZED";
    switch (ctl->solution.status) {
        case MPC_SOLVE_SUCCESS: return "OPTIMAL";
        case MPC_SOLVE_MAX_ITER: return "MAX_ITER";
        case MPC_SOLVE_INFEASIBLE: return "INFEASIBLE";
        case MPC_SOLVE_UNBOUNDED: return "UNBOUNDED";
        case MPC_SOLVE_NUMERICAL_ERROR: return "NUMERICAL_ERROR";
        default: return "UNKNOWN";
    }
}

void mpc_controller_print_report(const mpc_controller_t *ctl)
{
    if (!ctl) return;
    printf("=== MPC Controller Report ===\n");
    printf("Status: %s\n", mpc_controller_status_string(ctl));
    printf("Mode: %d, Steps: %d\n", ctl->mode, ctl->stats.total_steps);
    printf("ISE: %.6f, IAE: %.6f, TVu: %.6f\n",
           ctl->stats.ise, ctl->stats.iae, ctl->stats.tvu);
    printf("Infeasible count: %d\n", ctl->stats.infeasible_count);
    printf("Last cost: %.6f\n", ctl->solution.cost);
    printf("=============================\n");
}
