/**
 * @file mpc_constraint_defs.h
 * @brief Core type definitions and enumerations for MPC constraint handling.
 *
 * This header defines the foundational data types for constraint management in
 * Model Predictive Control, including constraint categories, priority levels,
 * feasibility status, and relaxation policies.
 *
 * @section L1_Definitions
 * - ConstraintType: HARD (physical/safety), SOFT (economic/quality)
 * - ConstraintPriority: CRITICAL, HIGH, MEDIUM, LOW, MONITOR
 * - MPCFeasibilityStatus: FEASIBLE, INFEASIBLE, RECOVERABLE, IRRECOVERABLE
 * - ConstraintScope: INPUT, RATE, OUTPUT, TERMINAL, CUSTOM
 *
 * @section L2_Core_Concepts
 * - Hard constraints must be strictly satisfied (actuator limits, safety bounds)
 * - Soft constraints may be violated with penalty (economic targets, quality ranges)
 * - Constraint prioritization: high-priority constraints satisfied first
 * - Feasibility recovery: relax low-priority soft constraints when QP is infeasible
 *
 * @section L3_Engineering_Structures
 * - Enum-to-string mapping for PLC/HMI display
 * - Struct layout optimized for real-time QP solver access
 * - Priority indices enable O(1) per-level constraint access
 *
 * Reference:
 *   Rawlings, Mayne & Diehl (2017), "Model Predictive Control", 2nd ed.
 *   Qin & Badgwell (2003), "A survey of industrial model predictive control technology"
 *   Maciejowski (2002), "Predictive Control with Constraints"
 *   Camacho & Bordons (2007), "Model Predictive Control", 2nd ed.
 */

#ifndef MPC_CONSTRAINT_DEFS_H
#define MPC_CONSTRAINT_DEFS_H

#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <stdbool.h>
#include <float.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MPC_MAX_CONSTRAINTS      2048
#define MPC_MAX_PRIORITY_LEVELS  8
#define MPC_CONSTRAINT_NAME_MAX  64
#define MPC_FEASIBILITY_TOL      1e-8
#define MPC_DEFAULT_SOFT_PENALTY 1e6
#define MPC_CONSTRAINT_EPS       1e-12
#define MPC_BOUND_INF            1e30

typedef enum {
    MPC_CONSTRAINT_HARD_INPUT      = 0,
    MPC_CONSTRAINT_HARD_RATE       = 1,
    MPC_CONSTRAINT_HARD_OUTPUT     = 2,
    MPC_CONSTRAINT_HARD_TERMINAL   = 3,
    MPC_CONSTRAINT_SOFT_INPUT      = 4,
    MPC_CONSTRAINT_SOFT_RATE       = 5,
    MPC_CONSTRAINT_SOFT_OUTPUT     = 6,
    MPC_CONSTRAINT_SOFT_TERMINAL   = 7,
    MPC_CONSTRAINT_COUPLING        = 8,
    MPC_CONSTRAINT_CUSTOM          = 9
} mpc_constraint_type_t;

typedef enum {
    MPC_PRIORITY_CRITICAL = 0,
    MPC_PRIORITY_HIGH     = 1,
    MPC_PRIORITY_MEDIUM   = 2,
    MPC_PRIORITY_LOW      = 3,
    MPC_PRIORITY_MONITOR  = 4
} mpc_priority_level_t;

typedef enum {
    MPC_FEASIBLE        = 0,
    MPC_INFEASIBLE      = 1,
    MPC_RECOVERABLE     = 2,
    MPC_IRRECOVERABLE   = 3,
    MPC_DEGENERATE      = 4
} mpc_feasibility_status_t;

typedef enum {
    MPC_SCOPE_INPUT    = 0,
    MPC_SCOPE_RATE     = 1,
    MPC_SCOPE_OUTPUT   = 2,
    MPC_SCOPE_TERMINAL = 3,
    MPC_SCOPE_COUPLING = 4,
    MPC_SCOPE_CUSTOM   = 5
} mpc_constraint_scope_t;

typedef enum {
    MPC_RELAX_NEVER       = 0,
    MPC_RELAX_IF_NEEDED   = 1,
    MPC_RELAX_ALWAYS_SOFT = 2,
    MPC_RELAX_SEQUENTIAL  = 3
} mpc_relaxation_policy_t;

typedef enum {
    MPC_OK                    = 0,
    MPC_ERR_NULL_POINTER      = -1,
    MPC_ERR_INVALID_PRIORITY  = -2,
    MPC_ERR_CONSTRAINT_FULL   = -3,
    MPC_ERR_QP_INFEASIBLE     = -4,
    MPC_ERR_QP_MAX_ITERATIONS = -5,
    MPC_ERR_INVALID_BOUNDS    = -6,
    MPC_ERR_DIMENSION_MISMATCH= -7,
    MPC_ERR_NOT_INITIALIZED   = -8,
    MPC_ERR_MEMORY            = -9,
    MPC_ERR_RELAXATION_LIMIT  = -10
} mpc_status_t;

typedef struct {
    int      index;
    char     name[MPC_CONSTRAINT_NAME_MAX];
    mpc_constraint_type_t  type;
    mpc_constraint_scope_t scope;
    mpc_priority_level_t   priority;
    mpc_relaxation_policy_t relax_policy;
    double   lower_bound;
    double   upper_bound;
    double   target_value;
    double   penalty_weight;
    double  *coefficients;
    int      num_coefficients;
    double   slack_value;
    double   slack_max;
    double   slack_penalty_linear;
    double   slack_penalty_quadratic;
    bool     is_active;
    bool     is_violated;
    double   violation_magnitude;
    double   violation_duration;
    uint64_t violation_start_cycle;
    double   lagrange_multiplier;
    double   sensitivity;
    double   conditioning_number;
    bool     is_rank_deficient;
} mpc_constraint_t;

typedef struct {
    mpc_constraint_t *constraints;
    int      capacity;
    int      total_count;
    int      priority_count[MPC_MAX_PRIORITY_LEVELS];
    int      priority_start[MPC_MAX_PRIORITY_LEVELS];
    int      count_input;
    int      count_rate;
    int      count_output;
    int      count_terminal;
    int      count_coupling;
    mpc_feasibility_status_t feasibility;
    int      num_hard_active;
    int      num_soft_active;
    int      num_soft_violated;
    int      num_relaxed;
    int      num_decision_vars;
    int      num_slack_vars;
} mpc_constraint_set_t;

typedef struct {
    double  *du_optimal;
    double  *slack_values;
    double   objective_value;
    double  *priority_objectives;
    int      qp_iterations;
    mpc_feasibility_status_t status;
    double   solve_time_ms;
    int      num_constraints_active;
    int      num_constraints_violated;
    double   max_violation;
    double   sum_violations;
    double   max_slack;
    mpc_priority_level_t highest_relaxed;
    int      num_levels_relaxed;
    bool     required_relaxation;
    double   infeasibility_measure;
} mpc_qp_solution_t;

typedef struct {
    int     *active_indices;
    int      active_count;
    int      active_capacity;
    double  *lagrange_multipliers;
    double  *active_gradients;
    double  *working_matrix;
    double  *working_rhs;
    bool     is_full_rank;
    double   condition_number;
} mpc_active_set_t;

typedef struct {
    int      mv_index;
    double   saturated_value;
    bool     at_upper_bound;
    bool     at_lower_bound;
    int      saturation_duration;
    double   lost_control_authority;
    double   nearest_feasible_cv[16];
} mpc_input_saturation_t;

typedef struct {
    int      num_cv;
    int     *cv_priority;
    double  *cv_lower_limit;
    double  *cv_upper_limit;
    double  *cv_current_value;
    bool    *cv_constraint_active;
    double  *cv_violation_cost;
    int      cv_most_critical;
} mpc_output_prioritization_t;

typedef struct {
    int      prediction_horizon;
    int      control_horizon;
    double **constraint_propagation;
    int      num_propagated_constraints;
    int     *propagation_path;
    bool     has_deadtime;
    double   deadtime_steps;
} mpc_constraint_propagation_t;

typedef enum {
    MPC_VENDOR_GENERIC           = 0,
    MPC_VENDOR_ASPENTECH_DMC     = 1,
    MPC_VENDOR_HONEYWELL_RMPCT   = 2,
    MPC_VENDOR_SHELL_SMOC        = 3,
    MPC_VENDOR_ABB_PREDICT       = 4,
    MPC_VENDOR_ROCKWELL_PAVILION = 5,
    MPC_VENDOR_SIEMENS_SIMATIC   = 6
} mpc_vendor_type_t;

typedef struct {
    mpc_vendor_type_t vendor;
    bool   use_range_control;
    bool   use_lexicographic_qp;
    bool   use_ideal_resting_value;
    double funnel_opening_rate;
    double constraint_zone_width;
    int    max_sequential_passes;
} mpc_vendor_config_t;


/* =========================================================================
 * L2: Constraint classification and evaluation functions
 * ========================================================================= */

/** Check if a constraint is hard (physical/safety, must be satisfied) */
bool mpc_constraint_is_hard(const mpc_constraint_t *c);

/** Check if a constraint is soft (economic/quality, may be violated with penalty) */
bool mpc_constraint_is_soft(const mpc_constraint_t *c);

/** Check if a constraint is relaxable (not MPC_RELAX_NEVER) */
bool mpc_constraint_is_relaxable(const mpc_constraint_t *c);

/** Get human-readable string for constraint type */
const char *mpc_constraint_type_string(mpc_constraint_type_t type);

/** Get human-readable string for priority level */
const char *mpc_priority_level_string(mpc_priority_level_t level);

/** Get human-readable string for feasibility status */
const char *mpc_feasibility_status_string(mpc_feasibility_status_t status);

/** Initialize a constraint with default values */
mpc_status_t mpc_constraint_init(mpc_constraint_t *c, int index);

/** Set constraint bounds (lb <= ub required) */
mpc_status_t mpc_constraint_set_bounds(mpc_constraint_t *c, double lb, double ub);

/** Set constraint coefficient vector */
mpc_status_t mpc_constraint_set_coefficients(mpc_constraint_t *c, const double *coeffs, int n);

/** Validate constraint consistency (bounds, priorities, penalties) */
mpc_status_t mpc_constraint_validate(const mpc_constraint_t *c);

/** Free constraint internal memory */
void mpc_constraint_free(mpc_constraint_t *c);

/** Deep copy a constraint */
mpc_status_t mpc_constraint_copy(mpc_constraint_t *dest, const mpc_constraint_t *src);

/** Evaluate constraint: compute a'*x for the constraint at point x */
double mpc_constraint_evaluate(const mpc_constraint_t *c, const double *x, int n);

/** Check if constraint is violated at point x, update violation fields */
mpc_status_t mpc_constraint_check_violation(mpc_constraint_t *c, const double *x, int n);

/** Calculate shadow price (Lagrange multiplier) of an active constraint */
double mpc_constraint_shadow_price(const mpc_constraint_t *c);

/** Calculate economic cost of constraint violation */
double mpc_constraint_violation_cost(const mpc_constraint_t *c);

/** Count hard and soft constraints in a constraint set */
mpc_status_t mpc_constraint_count_by_type(const mpc_constraint_set_t *cs, int *hard_count, int *soft_count);

/** Initialize a constraint set with given capacity */
mpc_status_t mpc_constraint_set_init(mpc_constraint_set_t *cs, int capacity);

/** Free a constraint set and all internal memory */
void mpc_constraint_set_free(mpc_constraint_set_t *cs);

/** Add a constraint to a constraint set */
mpc_status_t mpc_constraint_set_add(mpc_constraint_set_t *cs, const mpc_constraint_t *c);

/** Update feasibility status based on current violations */
mpc_status_t mpc_constraint_set_update_feasibility(mpc_constraint_set_t *cs, double *violations);

/* =========================================================================
 * L6: Industrial constraint management function declarations
 * ========================================================================= */

/** Detect MV saturation in an input vector */
mpc_status_t mpc_detect_input_saturation(const double *mv_values, const double *mv_lower,
                                          const double *mv_upper, int num_mv,
                                          int saturation_threshold,
                                          mpc_input_saturation_t *saturation_info);

/** Compute desaturation path for saturated MVs */
mpc_status_t mpc_compute_desaturation_path(const mpc_input_saturation_t *sat,
                                            const double *mv_steady_state, int num_mv,
                                            double *desaturation_moves, int horizon);

/** Initialize output prioritization structure */
mpc_status_t mpc_output_prioritization_init(mpc_output_prioritization_t *op, int num_cv);

/** Free output prioritization memory */
void mpc_output_prioritization_free(mpc_output_prioritization_t *op);

/** Set a single CV in the output prioritization structure */
mpc_status_t mpc_output_prioritization_set_cv(mpc_output_prioritization_t *op, int cv_idx,
                                                mpc_priority_level_t priority,
                                                double lower, double upper,
                                                double violation_cost);

/** Evaluate current CV values against constraints */
mpc_status_t mpc_output_prioritization_evaluate(mpc_output_prioritization_t *op,
                                                  const double *cv_values);

/** Rank violated CVs by priority */
mpc_status_t mpc_output_prioritization_rank_violations(const mpc_output_prioritization_t *op,
                                                         int *ranked_indices);

/* =========================================================================
 * L7: Industrial vendor function declarations
 * ========================================================================= */

/** AspenTech DMC3 sequential QP constraint handling */
mpc_status_t mpc_aspen_dmc3_priority_solve(const mpc_constraint_set_t *cs,
                                             mpc_vendor_config_t *vendor_cfg,
                                             mpc_qp_solution_t *solution);

/** Honeywell RMPCT range control setup */
mpc_status_t mpc_honeywell_rmpct_range_control(const mpc_output_prioritization_t *op,
                                                 mpc_vendor_config_t *vendor_cfg,
                                                 double *cv_targets, double *funnel_widths);

/** Shell SMOC soft-output constraint setup */
mpc_status_t mpc_shell_smoc_constraint_setup(mpc_constraint_set_t *cs,
                                               mpc_vendor_config_t *vendor_cfg);

/** ABB Predict & Control economic weight ranking */
mpc_status_t mpc_abb_predict_economic_ranking(const mpc_output_prioritization_t *op,
                                                double *economic_weights);

/** Constraint funnel management (transient widening) */
mpc_status_t mpc_constraint_funnel_update(mpc_constraint_t *constraints, int num_constraints,
                                            double time, double closing_rate,
                                            double steady_state_width);

/** Lexicographic MPC solver */
mpc_status_t mpc_lexicographic_solve(const mpc_constraint_set_t *cs,
                                      const double *H, const double *f,
                                      int num_vars, mpc_qp_solution_t *solution);

/** Constraint conditioning analysis */
mpc_status_t mpc_constraint_conditioning_analysis(const mpc_constraint_set_t *cs,
                                                    const double *A, int m, int n,
                                                    double *condition_numbers);


#ifdef __cplusplus
}
#endif

#endif
