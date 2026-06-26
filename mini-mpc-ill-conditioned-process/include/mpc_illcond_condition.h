#ifndef MPC_ILLCOND_CONDITION_H
#define MPC_ILLCOND_CONDITION_H

#include "mpc_illcond_defs.h"

/* Condition number estimation */
double mpc_condition_estimate(const mpc_matrix_t *A, mpc_illcond_estimation_method_t method);
int mpc_condition_is_illcond(double cond);
const char* mpc_condition_grade(double cond);
double mpc_condition_digits_lost(double cond);
double mpc_condition_recommend_lambda(double norm_a, double cond);

/* Comprehensive diagnostics */
int mpc_condition_diagnose(const mpc_matrix_t *A, mpc_illcond_diagnostic_t *diag);

/* Stiffness and horizon */
double mpc_condition_stiffness_ratio(const double *tau, size_t n);
double mpc_condition_horizon_growth(size_t P, size_t P0, double alpha);

/* Advanced condition metrics */
double mpc_condition_augmented(const mpc_matrix_t *A, double lambda);
double mpc_condition_effective(const mpc_illcond_model_t *model);
double mpc_condition_from_rga(const double *rga_vals, size_t ny, size_t nu);
double mpc_condition_relative(const mpc_matrix_t *A, const mpc_matrix_t *B);
double mpc_condition_dynamic_matrix(const mpc_illcond_model_t *model);
int mpc_condition_is_acceptable(const mpc_illcond_diagnostic_t *diag);

#endif
