#ifndef MPC_ILLCOND_DEFS_H
#define MPC_ILLCOND_DEFS_H

#include <stddef.h>
#include <stdint.h>
#include <math.h>

#define MPC_ILLCOND_EPSILON 1.4901161193847656e-08
#define MPC_ILLCOND_COND_THRESHOLD 1.0e8
#define MPC_ILLCOND_STIFFNESS_THRESHOLD 1.0e5
#define MPC_ILLCOND_COLLINEARITY_THRESHOLD 0.95
#define MPC_ILLCOND_RGA_HIGH_THRESHOLD 5.0
#define MPC_ILLCOND_RGA_SEVERE_THRESHOLD 10.0
#define MPC_ILLCOND_MIN_RANK_RATIO 0.5
#define MPC_CONDEST_MAX_ITER 10
#define MPC_CONDEST_TOL 1.0e-12

typedef enum {
    MPC_CONDEST_NORM1,
    MPC_CONDEST_NORM_INF,
    MPC_CONDEST_FROBENIUS,
    MPC_CONDEST_GERSHGORIN,
    MPC_CONDEST_TRACE_RATIO
} mpc_illcond_estimation_method_t;

typedef enum {
    MPC_REGULARIZE_TIKHONOV,
    MPC_REGULARIZE_TIKHONOV_DX,
    MPC_REGULARIZE_TRUNC_SVD,
    MPC_REGULARIZE_LEVENBERG,
    MPC_REGULARIZE_ELASTIC_NET
} mpc_illcond_regularization_type_t;

typedef enum {
    MPC_PRECOND_JACOBI,
    MPC_PRECOND_SSOR,
    MPC_PRECOND_ILU0,
    MPC_PRECOND_POLYNOMIAL,
    MPC_PRECOND_BLOCK_JACOBI,
    MPC_PRECOND_ADDITIVE_SCHWARZ
} mpc_illcond_preconditioner_type_t;

typedef enum {
    MPC_ROOTCAUSE_NEAR_COLLINEAR_GAINS,
    MPC_ROOTCAUSE_HIGH_STIFFNESS_RATIO,
    MPC_ROOTCAUSE_NEARLY_RANK_DEFICIENT,
    MPC_ROOTCAUSE_POOR_SCALING,
    MPC_ROOTCAUSE_HIGH_RGA,
    MPC_ROOTCAUSE_MEASUREMENT_REDUNDANCY,
    MPC_ROOTCAUSE_ZERO_GAIN_DIRECTION
} mpc_illcond_rootcause_t;

typedef enum {
    MPC_SENSITIVITY_LOW,
    MPC_SENSITIVITY_MODERATE,
    MPC_SENSITIVITY_HIGH,
    MPC_SENSITIVITY_EXTREME
} mpc_sensitivity_level_t;

typedef struct {
    double *data;
    size_t  rows;
    size_t  cols;
    size_t  stride;
} mpc_matrix_t;

typedef struct {
    mpc_matrix_t U;
    mpc_matrix_t V;
    double     *S;
    size_t      rank;
    double      cond;
} mpc_svd_t;

typedef struct {
    mpc_matrix_t  G;
    mpc_matrix_t  G_dyn;
    size_t        ny;
    size_t        nu;
    size_t        P;
    size_t        M;
    double        cond_gain;
    double        cond_dynamic;
    double        stiffness_ratio;
    double        collinearity_index;
    double       *rga_max;
    mpc_illcond_rootcause_t rootcause;
} mpc_illcond_model_t;

typedef struct {
    mpc_illcond_regularization_type_t type;
    double lambda;
    double lambda_delta_u;
    double svd_threshold;
    double elastic_alpha;
    int    max_lm_iter;
    double lm_decay;
} mpc_regularization_t;

typedef struct {
    mpc_illcond_preconditioner_type_t type;
    mpc_matrix_t M;
    mpc_matrix_t M_inv;
    int          ilu_level;
    int          poly_degree;
    double       ssor_omega;
} mpc_preconditioner_t;

typedef struct {
    double condition_number;
    double condition_number_1;
    double condition_number_inf;
    double min_singular_value;
    double max_singular_value;
    double effective_rank_ratio;
    double rga_condition_number;
    double collinearity_detected;
    double stiffness_diagnostic;
    mpc_sensitivity_level_t sensitivity;
    mpc_illcond_rootcause_t primary_cause;
    double recommended_lambda;
} mpc_illcond_diagnostic_t;

typedef struct {
    mpc_matrix_t H;
    double      *f;
    mpc_matrix_t A_eq;
    double      *b_eq;
    mpc_matrix_t A_ineq;
    double      *b_ineq;
    size_t       n_vars;
    size_t       n_eq;
    size_t       n_ineq;
    double       cond_H;
    double       cond_A;
    int          is_illcond;
    mpc_illcond_diagnostic_t diag;
} mpc_illcond_qp_t;

#endif /* MPC_ILLCOND_DEFS_H */
