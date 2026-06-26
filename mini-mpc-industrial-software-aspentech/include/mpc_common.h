/** @file mpc_common.h
 * @brief MPC Core Definitions
 * L1: MV/CV/DV types, step-response model, dynamic matrix, QP, EU scaling
 * L2: Receding horizon, bias feedback, steady-state target
 * L3: AspenTech DMC3 layered architecture
 * Ref: Rawlings/Mayne/Diehl (2017), Cutler/Ramaker (1980)
 */
#ifndef MPC_COMMON_H
#define MPC_COMMON_H

#include <stddef.h>
#include <stdint.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/* System dimension constants */
#define MPC_MAX_MV              80
#define MPC_MAX_CV              160
#define MPC_MAX_DV              40
#define MPC_MAX_MODEL_HORIZON   120
#define MPC_MAX_PRED_HORIZON    60
#define MPC_MAX_CTRL_HORIZON    10
#define MPC_MAX_NAME_LEN        32
#define MPC_MAX_EU_LEN          16
#define MPC_EPS                 1e-12
#define MPC_INF                 1e30

/* Variable types */
typedef enum { MPC_VAR_TYPE_MV=0, MPC_VAR_TYPE_CV=1, MPC_VAR_TYPE_DV=2, MPC_VAR_TYPE_CCV=3 } mpc_var_type_t;

/* Operational modes */
typedef enum { MPC_MODE_OFF=0, MPC_MODE_IDLE=1, MPC_MODE_RUNNING=2, MPC_MODE_HOLD=3, MPC_MODE_BACKOFF=4, MPC_MODE_CALIB=5 } mpc_mode_t;

/* Constraint types */
typedef enum { MPC_CONSTRAINT_NONE=0, MPC_CONSTRAINT_LOWER=1, MPC_CONSTRAINT_UPPER=2, MPC_CONSTRAINT_BOTH=3, MPC_CONSTRAINT_EQUAL=4, MPC_CONSTRAINT_RATE=5 } mpc_constraint_type_t;

/* QP solver status */
typedef enum { QP_OPTIMAL=0, QP_INFEASIBLE=1, QP_UNBOUNDED=2, QP_MAX_ITER=3, QP_NUMERICAL=4 } qp_status_t;

/* LP optimization mode */
typedef enum { LP_MIN_COST=0, LP_MAX_PROFIT=1, LP_MIN_MV_MOVEMENT=2, LP_FEASIBILITY=3 } mpc_lp_mode_t;

/* Sub-controller decomposition */
typedef enum { MPC_DECOMP_NONE=0, MPC_DECOMP_BLOCK=1, MPC_DECOMP_COORDINATE=2 } mpc_decomp_t;

/* Step response model (FIR) for one (MV,CV) pair */
typedef struct {
    int n_coeffs; double sample_time_sec; double dead_time_samples;
    double gain_ss; double time_constant_sec; double *coeff;
    int mv_index; int cv_index;
} mpc_step_model_t;

/* MIMO step-response model collection */
typedef struct {
    int n_mv, n_cv, n_dv;
    int model_horizon;
    double sample_time_sec;
    mpc_step_model_t **sub_models;
    mpc_step_model_t **dv_models;
} mpc_mimo_model_t;

/* Dynamic matrix A (P x M Toeplitz) */
typedef struct { int P, M; double *A, *A_T; } mpc_dynamic_matrix_t;

/* QP weights */
typedef struct { int n_cv; double *Q; int n_mv; double *R, *S; } mpc_weights_t;

/* QP problem */
typedef struct {
    int n_vars, n_eq, n_ineq;
    double *H, *c;
    double *A_ineq, *b_ineq_low, *b_ineq_high;
    double *A_eq, *b_eq;
} mpc_qp_problem_t;

/* QP solution */
typedef struct {
    int n_vars; double *x_opt; double f_opt;
    int n_active, *active_set, iterations;
    qp_status_t status;
} mpc_qp_solution_t;

/* MV Configuration (AspenTech DMC3 style) */
typedef struct {
    int index; char name[MPC_MAX_NAME_LEN], eu[MPC_MAX_EU_LEN];
    double eu0, eu100, lo_limit, hi_limit, rate_lo, rate_hi;
    double current_value, setpoint, lp_cost;
    int is_enabled, is_optimizing, has_feedback;
} mpc_mv_config_t;

/* CV Configuration */
typedef struct {
    int index; char name[MPC_MAX_NAME_LEN], eu[MPC_MAX_EU_LEN];
    double eu0, eu100, lo_limit, hi_limit;
    double setpoint, deadband, current_value, lp_cost;
    int is_enabled, is_controlled, is_inferred;
    mpc_constraint_type_t constraint_type;
    double constraint_lo, constraint_hi;
} mpc_cv_config_t;

/* DV Configuration */
typedef struct {
    int index; char name[MPC_MAX_NAME_LEN], eu[MPC_MAX_EU_LEN];
    double current_value; int is_enabled;
} mpc_dv_config_t;

/* Receding horizon controller state */
typedef struct {
    int P, M, model_horizon, n_mv, n_cv, n_dv;
    double *delta_u_past, *cv_error_past;
    double *y_openloop, *y_corrected, *y_ref;
    double *delta_u_opt; double f_opt;
    double bias_filter_gain; mpc_mode_t mode;
    double *past_unforced;
} mpc_controller_state_t;

/* AspenTech DMC3 controller configuration */
typedef struct {
    char controller_name[64], unit_name[64];
    double execution_interval_sec, sample_time_sec;
    int n_mv, n_cv, n_dv, P, M, model_horizon;
    mpc_mimo_model_t model;
    mpc_mv_config_t *mv_config;
    mpc_cv_config_t *cv_config;
    mpc_dv_config_t *dv_config;
    mpc_weights_t weights;
    mpc_lp_mode_t lp_mode;
    double lp_max_cv_relax;
    int qp_max_iterations; double qp_tolerance;
    double bias_filter_gain; int bias_update_enabled;
    double singular_value_floor;
    int sub_controller_count;
    mpc_decomp_t decomp_strategy;
    int use_orthogonal_move;
    double ill_cond_threshold;
    int use_state_space;
} mpc_aspen_config_t;

/* Steady-state target from LP */
typedef struct {
    int n_mv, n_cv;
    double *u_ss, *y_ss;
    double lp_cost;
    int is_feasible;
    double *slack_cv;
} mpc_ss_target_t;

/* Recursive Least Squares estimator */
typedef struct {
    int n_params;
    double *theta, *P;
    double lambda, delta;
    int n_updates;
} mpc_rls_estimator_t;

/* Kalman filter state */
typedef struct {
    int nx, ny, nu;
    double *A, *B, *C;
    double *x_hat, *P_kf;
    double *Q_kf, *R_kf;
    int is_initialized;
} mpc_kalman_state_t;

/* === Function Declarations === */

mpc_step_model_t*    mpc_step_model_alloc(int n_coeffs);
void                 mpc_step_model_free(mpc_step_model_t *m);
mpc_mimo_model_t*    mpc_mimo_model_alloc(int n_mv, int n_cv, int n_dv, int horizon);
void                 mpc_mimo_model_free(mpc_mimo_model_t *m);
mpc_controller_state_t* mpc_controller_state_alloc(int P, int M, int model_horizon, int n_mv, int n_cv, int n_dv);
void                 mpc_controller_state_free(mpc_controller_state_t *cs);
mpc_qp_problem_t*    mpc_qp_problem_alloc(int n_vars, int n_eq, int n_ineq);
void                 mpc_qp_problem_free(mpc_qp_problem_t *qp);
mpc_qp_solution_t*   mpc_qp_solution_alloc(int n_vars);
void                 mpc_qp_solution_free(mpc_qp_solution_t *sol);
mpc_aspen_config_t*  mpc_aspen_config_alloc(void);
void                 mpc_aspen_config_free(mpc_aspen_config_t *cfg);
mpc_ss_target_t*     mpc_ss_target_alloc(int n_mv, int n_cv);
void                 mpc_ss_target_free(mpc_ss_target_t *tgt);
mpc_rls_estimator_t* mpc_rls_alloc(int n_params, double lambda, double delta);
void                 mpc_rls_free(mpc_rls_estimator_t *rls);
mpc_kalman_state_t*  mpc_kalman_alloc(int nx, int ny, int nu);
void                 mpc_kalman_free(mpc_kalman_state_t *kf);
int                  mpc_step_model_from_fopdt(mpc_step_model_t *m, double gain, double tau, double dead_time, double sample_time);
int                  mpc_step_model_from_fir(mpc_step_model_t *m, const double *fir_coeffs, int n, double sample_time);
int                  mpc_step_model_predict(const mpc_step_model_t *m, const double *delta_u_past, int n_past, double *y_pred, int n_pred);
double               mpc_step_model_truncation_error(const mpc_step_model_t *m, double error_tolerance);
int                  mpc_step_model_validate(const mpc_step_model_t *m);
int                  mpc_step_model_to_ss(const mpc_step_model_t *m, double *a, double *b, double *c);
int                  mpc_build_dynamic_matrix(const mpc_step_model_t *model, int P, int M, mpc_dynamic_matrix_t *dm);
void                 mpc_dynamic_matrix_free(mpc_dynamic_matrix_t *dm);
int                  mpc_dynamic_matrix_apply(const mpc_dynamic_matrix_t *dm, const double *du, double *y_forced);
int                  mpc_build_mimo_dynamic_matrix(const mpc_mimo_model_t *mimo, int P, int M, double *A_global, int ldA);
int                  mpc_mimo_set_submodel(mpc_mimo_model_t *mimo, int cv_idx, int mv_idx, const mpc_step_model_t *sub);
int                  mpc_mimo_set_dvmodel(mpc_mimo_model_t *mimo, int cv_idx, int dv_idx, const mpc_step_model_t *sub);
int                  mpc_mimo_predict_openloop(const mpc_mimo_model_t *mimo, const double *delta_u_past, int n_past, double *y_openloop, int n_pred);
int                  mpc_mimo_extract_ss_gain(const mpc_mimo_model_t *mimo, double *G_ss);
int                  mpc_mimo_validate(const mpc_mimo_model_t *mimo);
double               mpc_pct_to_eu(double pct, double eu0, double eu100);
double               mpc_eu_to_pct(double eu, double eu0, double eu100);
double               mpc_scale_mv(const mpc_mv_config_t *mv, double pct_val);
double               mpc_unscale_mv(const mpc_mv_config_t *mv, double eu_val);
int                  mpc_compute_steady_state_target(const mpc_mimo_model_t *mimo, const mpc_mv_config_t *mv_cfg, const mpc_cv_config_t *cv_cfg, mpc_ss_target_t *target);
int                  mpc_lp_simplex_solve(const double *c, const double *A, const double *b, int n, int m, double *x_opt, double *f_opt);
qp_status_t          mpc_qp_active_set_solve(mpc_qp_problem_t *prob, mpc_qp_solution_t *sol);
qp_status_t          mpc_qp_interior_point_solve(mpc_qp_problem_t *prob, mpc_qp_solution_t *sol);
int                  mpc_qp_check_optimality(const mpc_qp_problem_t *prob, const mpc_qp_solution_t *sol, double tol);
int                  mpc_dmc_step(mpc_aspen_config_t *cfg, mpc_controller_state_t *cs, const double *cv_measured, const double *dv_measured, double *mv_output, mpc_qp_solution_t *qp_sol);
int                  mpc_dmc_predict(const mpc_aspen_config_t *cfg, const mpc_controller_state_t *cs, double *y_pred);
int                  mpc_dmc_bias_update(mpc_controller_state_t *cs, const double *y_measured, const double *y_pred_1step, double bias_gain);
int                  mpc_dmc_shift_horizon(mpc_controller_state_t *cs, int n_mv);
int                  mpc_implement_first_move(const mpc_controller_state_t *cs, const mpc_mv_config_t *mv_cfg, double *mv_output);
double               mpc_condition_number_svd(const double *G, int n_rows, int n_cols);
int                  mpc_detect_ill_conditioning(const mpc_mimo_model_t *mimo, double threshold, int *pairs, int max_pairs);
int                  mpc_svd_truncate(double *U, double *S, double *Vt, int rows, int cols, double s_floor);
int                  mpc_regularize_hessian(double *H, int n, double lambda);
int                  mpc_orthogonal_move_compute(const mpc_dynamic_matrix_t *dm, const double *y_err, double *du_orth, double alpha);
int                  mpc_ss_to_step_model(const double *A, const double *B, const double *C, int nx, int nu, int ny, mpc_step_model_t *model_out, int n_steps);
int                  mpc_ss_predict(const double *A, const double *B, const double *C, int nx, int nu, int ny, const double *x0, const double *u_seq, double *y_seq, int N);
int                  mpc_rls_update(mpc_rls_estimator_t *rls, const double *phi, double y_meas);
int                  mpc_rls_get_params(const mpc_rls_estimator_t *rls, double *theta);
int                  mpc_kalman_predict(mpc_kalman_state_t *kf, const double *u);
int                  mpc_kalman_correct(mpc_kalman_state_t *kf, const double *y);
double               mpc_compute_cv_violation(const mpc_controller_state_t *cs, const mpc_cv_config_t *cv_cfg);
double               mpc_compute_mv_utilization(const mpc_mv_config_t *mv_cfg, const double *mv_output);
double               mpc_compute_performance_index(const mpc_controller_state_t *cs, const double *y_measured, int n_cv);
int                  mpc_model_quality_monitor(const mpc_mimo_model_t *mimo, const double *measured, const double *predicted, int n_samples, double *rmse);

/* Data Reconciliation (mpc_data_recon.c) */
int    mpc_data_recon_wls(const double *y_raw, const double *sigma_sq, const double *A_balance, int n_meas, int n_constraints, double *y_hat, double *adjustments);
int    mpc_data_recon_gross_error_test(const double *adjustments, const double *sigma_sq, int n_meas, int n_constraints, double *test_statistic_out);
int    mpc_reconcile_flow_node(const double *flow_measurements, const double *flow_uncertainty, int n_streams, double *reconciled_flows, double *imbalance);
int    mpc_detect_faulty_sensor(const double *adjustments, const double *sigma, int n_meas, int *suspect_flags, double z_critical);
int    mpc_reconcile_heat_balance(double m_hot, double m_cold, double cp_hot, double cp_cold, double T_hot_in, double T_hot_out, double T_cold_in, double T_cold_out, double *Q_hot_out, double *Q_cold_out, double *heat_loss_pct);

/* Sub-Controller Decomposition (mpc_decomposition.c) */
int    mpc_compute_rga(const double *G_ss, int n_cv, int n_mv, double *RGA);
int    mpc_detect_rga_blocks(const double *RGA, int n, double block_threshold, int *block_map, int *n_blocks);
int    mpc_extract_sub_controller(const double *A_full, int total_rows, int total_cols, const int *cv_block_indices, int n_cv_block, const int *mv_block_indices, int n_mv_block, double *A_sub);
int    mpc_coordinate_solve_iteration(const double *H_full, const double *c_full, int n_total, const int *block_map, int n_blocks, double *x_solution, int max_iter, double tol);
double mpc_niederlinski_index(const double *G_ss, int n);

/* Zone Control (mpc_zone_control.c) */
int mpc_zone_control_gradient(const double *y_pred, int n_pred, double y_lo, double y_hi, double q_lo, double q_hi, double *gradient_out, double *cost_out);
int mpc_generate_funnel_constraints(double y_lo_ss, double y_hi_ss, double funnel_slope, int n_pred, double *y_lo_profile, double *y_hi_profile);
int mpc_cv_tracking_error(const double *y_pred, int n_pred, double y_setpoint, double y_zone_lo, double y_zone_hi, double y_irv, int zone_enabled, int irv_enabled, double *error_profile);
int mpc_assign_cv_priority(double *slack_penalties, int n_cv, const int *cv_priorities, double base_penalty);
int mpc_soften_constraints(double *lb, double *ub, int n, const double *slack_penalties, double *slack_vars_out, double *cost_penalty_out, int use_l1);
int mpc_count_zone_crossings(const double *cv_history, int n_cv, int n_steps, const double *zone_lo, const double *zone_hi, int *crossing_counts);

/* Disturbance Modeling (mpc_disturbance_model.c) */
int    mpc_disturbance_step_predict(const double *disturbance_current, int n_d, int n_steps, double *disturbance_forecast);
int    mpc_disturbance_ramp_predict(const double *disturbance_current, const double *ramp_rate, int n_d, int n_steps, double *disturbance_forecast);
int    mpc_disturbance_exp_predict(const double *disturbance_current, int n_d, int n_steps, double alpha, double *disturbance_forecast);
int    mpc_disturbance_periodic_predict(double amplitude, double frequency_hz, double phase_rad, double sample_time_sec, int n_steps, double *disturbance_forecast);
int    mpc_disturbance_predict_by_type(int dist_type, const double *dist_current, const double *dist_param, int n_d, int n_steps, double *dist_forecast);
int    mpc_dv_feedforward_predict(const mpc_mimo_model_t *mimo, const double *dv_current, const double *dv_future, int n_pred, double *y_dv_contrib);
double mpc_disturbance_rejection_ratio(const double *y_closed_loop, const double *y_open_loop, int n_samples, int n_cv);

#ifdef __cplusplus
}
#endif

#endif /* MPC_COMMON_H */
