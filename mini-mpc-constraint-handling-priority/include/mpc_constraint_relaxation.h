#ifndef MPC_CONSTRAINT_RELAXATION_H
#define MPC_CONSTRAINT_RELAXATION_H
#include "mpc_constraint_defs.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double   linear_penalty_base;
    double   quadratic_penalty_base;
    double   penalty_growth_factor;
    double   max_slack_per_constraint;
    int      max_relaxation_rounds;
    double   feasibility_tolerance;
    bool     auto_tune_penalties;
    bool     use_exact_penalty;
    double   exact_penalty_multiplier;
} mpc_relaxation_config_t;

typedef struct {
    int      num_slacks_active;
    int      num_constraints_relaxed;
    double  *slack_values;
    double  *penalty_weights_linear;
    double  *penalty_weights_quadratic;
    double   total_slack_cost;
    double   max_slack_used;
    int      relaxation_rounds_used;
    bool     feasibility_restored;
} mpc_relaxation_state_t;

mpc_status_t mpc_relaxation_config_init(mpc_relaxation_config_t *c);
mpc_status_t mpc_relaxation_tune_penalties(mpc_relaxation_config_t *c, const double *lm, int n);
mpc_status_t mpc_relaxation_set_exact_penalty(mpc_relaxation_config_t *c, double m);
mpc_status_t mpc_relaxation_alloc_slacks(const mpc_constraint_set_t *cs, const mpc_relaxation_config_t *cfg, mpc_relaxation_state_t *s);
mpc_status_t mpc_relaxation_init_slacks(mpc_relaxation_state_t *s);
void mpc_relaxation_free_slacks(mpc_relaxation_state_t *s);
int mpc_relaxation_num_active_slacks(const mpc_relaxation_state_t *s);
mpc_status_t mpc_relaxation_sequential_by_priority(mpc_constraint_set_t *cs, mpc_relaxation_config_t *cfg, mpc_relaxation_state_t *s, mpc_qp_solution_t *sol);
mpc_status_t mpc_relaxation_identify_infeasible_priorities(const mpc_constraint_set_t *cs, const double *cm, const double *rhs, int *lvls, int *nlvls);
mpc_status_t mpc_relaxation_auto_tune_penalty_weights(const mpc_qp_solution_t *sol, mpc_constraint_set_t *cs, mpc_relaxation_state_t *s, mpc_relaxation_config_t *cfg);
double mpc_relaxation_total_slack_cost(const mpc_relaxation_state_t *s);
mpc_status_t mpc_relaxation_check_kkt_slacks(const mpc_relaxation_state_t *s, const double *lm, double tol, bool *ok);
double mpc_relaxation_marginal_benefit(const mpc_constraint_t *c, const mpc_relaxation_state_t *s);

#ifdef __cplusplus
}
#endif
#endif