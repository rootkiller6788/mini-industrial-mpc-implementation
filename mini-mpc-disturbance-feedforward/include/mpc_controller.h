/**
 * @file mpc_controller.h
 * @brief Main MPC Controller with Disturbance Feedforward
 *
 * Integrates all MPC components (model, QP, observer, feedforward)
 * into a complete receding-horizon controller with disturbance
 * rejection capability.
 *
 * The controller executes the following loop at each sampling instant:
 *   1. Measure outputs y[k], disturbances d_meas[k]
 *   2. Update state/disturbance estimate via observer
 *   3. Compute feedforward control u_ff[k]
 *   4. Formulate and solve QP for feedback control u_fb[k]
 *   5. Apply u[k] = u_ff[k] + u_fb[k] to plant
 *
 * Reference: Rawlings, Mayne, Diehl (2017), Chapters 1-4
 * Reference: Qin and Badgwell, "A Survey of Industrial MPC Technology"
 *            Control Engineering Practice (2003)
 *
 * Course Mappings:
 * - MIT 6.302: Full-state feedback control
 * - Stanford ENGR205: MPC implementation
 * - ETH: Constrained optimal control in practice
 * - RWTH Aachen: Industrial MPC application
 *
 * @knowledge L1: MPC controller definition
 * @knowledge L2: Receding horizon implementation
 * @knowledge L2: Feedback + feedforward combination
 * @knowledge L6: CSTR temperature control problem
 * @knowledge L7: Industrial MPC (Honeywell RMPCT, Aspen DMCplus)
 */

#ifndef MPC_CONTROLLER_H
#define MPC_CONTROLLER_H

#include "mpc_common.h"
#include "mpc_model.h"
#include "mpc_qp.h"
#include "mpc_observer.h"
#include "mpc_feedforward.h"

/* MPC controller runtime flags */
typedef enum {
    MPC_FLAG_WARM_START    = 0x01,  /* Use previous solution as initial guess */
    MPC_FLAG_USE_FF        = 0x02,  /* Enable feedforward path */
    MPC_FLAG_USE_OBSERVER  = 0x04,  /* Enable state estimation */
    MPC_FLAG_SOFT_CONST    = 0x08,  /* Use soft constraints */
    MPC_FLAG_INTEGRAL_ACTION = 0x10, /* Enable integral action via disturbance model */
    MPC_FLAG_ANTI_WINDUP   = 0x20,  /* Enable anti-windup for saturated inputs */
    MPC_FLAG_OFFSET_FREE   = 0x40   /* Enable offset-free tracking */
} mpc_flags_t;

/* MPC controller runtime statistics */
typedef struct {
    double  avg_solve_time_ms;       /* Average QP solve time */
    double  max_solve_time_ms;       /* Maximum QP solve time */
    double  avg_cost;                /* Average optimal cost */
    double  constraint_violation;    /* Cumulative constraint violation */
    int     total_steps;             /* Total control steps executed */
    int     qp_iterations_total;     /* Total QP iterations */
    int     infeasible_count;        /* Number of infeasible QPs */
    double  ise;                     /* Integral squared error */
    double  iae;                     /* Integral absolute error */
    double  tvu;                     /* Total variation of input */
} mpc_runtime_stats_t;

/* MPC controller handle - complete controller state */
typedef struct {
    /* Configuration */
    mpc_ss_model_t      plant;          /* Plant model */
    mpc_aug_model_t     aug_model;      /* Augmented model */
    mpc_tuning_t        tuning;         /* Tuning parameters */
    mpc_constraints_t   constraints;    /* Constraint definition */
    mpc_prediction_t    prediction;     /* Prediction matrices */
    mpc_mode_t          mode;           /* Controller mode */
    uint32_t            flags;          /* Runtime flags */

    /* Estimation */
    mpc_observer_type_t observer_type;
    mpc_observer_state_t observer_state;
    mpc_kalman_params_t  kalman;
    mpc_luenberger_gain_t luenberger;
    mpc_dob_params_t     dob;

    /* Feedforward */
    mpc_ff_config_t      ff_config;
    mpc_ff_industrial_t  ff_industrial;

    /* Optimization */
    mpc_qp_solver_type_t qp_solver_type;
    mpc_qp_t             qp;
    mpc_working_set_t    working_set;
    mpc_ip_params_t      ip_params;

    /* Controller state */
    mpc_state_t          state;
    mpc_solution_t       solution;

    /* Statistics */
    mpc_runtime_stats_t  stats;

    /* Initialized flag */
    int                  initialized;
} mpc_controller_t;

/* ==========================================================================
 * Controller Lifecycle
 * ========================================================================== */

/* Initialize MPC controller with given plant model */
int mpc_controller_init(mpc_controller_t *ctl,
                         const mpc_ss_model_t *plant,
                         const mpc_tuning_t *tuning,
                         mpc_mode_t mode);

/* Set controller constraints */
int mpc_controller_set_constraints(mpc_controller_t *ctl,
                                    const mpc_constraints_t *cons);

/* Set reference trajectory (constant setpoint for now) */
void mpc_controller_set_reference(mpc_controller_t *ctl,
                                   const double y_ref[MPC_MAX_NY]);

/* Set reference trajectory over the prediction horizon */
void mpc_controller_set_reference_trajectory(mpc_controller_t *ctl,
                                              const double Y_ref[MPC_MAX_NP][MPC_MAX_NY]);

/* ==========================================================================
 * Controller Execution
 * ========================================================================== */

/* Execute one MPC control step
 * y_meas: current measurement [ny]
 * d_meas: current measured disturbance [nd]
 * u_out:  computed control action [nu] (output)
 * Returns: 0 on success, <0 on error
 */
int mpc_controller_step(mpc_controller_t *ctl,
                         const double y_meas[MPC_MAX_NY],
                         const double d_meas[MPC_MAX_ND],
                         double u_out[MPC_MAX_NU]);

/* Reset controller state for restart */
void mpc_controller_reset(mpc_controller_t *ctl);

/* ==========================================================================
 * Performance Analysis
 * ========================================================================== */

/* Compute controller performance metrics (ISE, IAE, TVu) */
void mpc_controller_compute_metrics(mpc_controller_t *ctl,
                                     const double y_meas[MPC_MAX_NY]);

/* Get controller status string */
const char* mpc_controller_status_string(const mpc_controller_t *ctl);

/* Print controller runtime report */
void mpc_controller_print_report(const mpc_controller_t *ctl);

/* ==========================================================================
 * L7: Industrial MPC Interfaces
 * ========================================================================== */

/* Simulate one closed-loop step with the plant model
 * This is used for what-if analysis before applying control to real plant
 */
int mpc_controller_simulate_step(mpc_controller_t *ctl,
                                  const double u[MPC_MAX_NU],
                                  const double d[MPC_MAX_ND],
                                  double y_next[MPC_MAX_NY],
                                  double x_next[MPC_MAX_NX]);

/* Export controller configuration for DCS integration (Honeywell Experion format) */
int mpc_controller_export_dcs_config(const mpc_controller_t *ctl,
                                      char *buffer, int buf_size);

/* Import controller configuration from DCS (Rockwell PlantPAx format) */
int mpc_controller_import_dcs_config(mpc_controller_t *ctl,
                                      const char *buffer, int buf_size);

/* Serialize controller state for checkpointing */
int mpc_controller_serialize(const mpc_controller_t *ctl,
                              double *buffer, int buf_size);

/* Deserialize controller state from checkpoint */
int mpc_controller_deserialize(mpc_controller_t *ctl,
                                const double *buffer, int buf_size);

/* ==========================================================================
 * L8: Advanced Features
 * ========================================================================== */

/* Compute stability margin estimate using Lyapunov analysis */
double mpc_controller_stability_margin(const mpc_controller_t *ctl);

/* Detect model-plant mismatch using residual analysis */
double mpc_controller_mismatch_detect(const mpc_controller_t *ctl,
                                       const double y_meas[MPC_MAX_NY]);

/* Adaptive tuning: adjust Q/R weights based on performance */
int mpc_controller_adaptive_tune(mpc_controller_t *ctl,
                                  double target_overshoot,
                                  double target_settling_time);

/* Compute time-varying Kalman gain for changing noise conditions */
int mpc_controller_time_varying_kalman(mpc_controller_t *ctl,
                                        const double Q_new[MPC_MAX_NX + MPC_MAX_ND]
                                                          [MPC_MAX_NX + MPC_MAX_ND],
                                        const double R_new[MPC_MAX_NY][MPC_MAX_NY]);

/* Predictive maintenance indicator based on control effort trend */
double mpc_controller_maintenance_indicator(const mpc_controller_t *ctl);

/* Monte Carlo robustness analysis for parameter uncertainty */
int mpc_controller_monte_carlo_robustness(mpc_controller_t *ctl,
                                           int n_trials,
                                           double param_stddev,
                                           double *mean_cost,
                                           double *std_cost,
                                           int *n_infeasible);

#endif /* MPC_CONTROLLER_H */
