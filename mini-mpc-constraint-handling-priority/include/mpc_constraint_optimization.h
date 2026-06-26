#ifndef MPC_CONSTRAINT_OPTIMIZATION_H
#define MPC_CONSTRAINT_OPTIMIZATION_H
#include "mpc_constraint_defs.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int      num_variables;
    int      num_constraints;
    int      num_equalities;
    double  *H; double *f; double *A; double *lb; double *ub;
    double  *A_eq; double *b_eq;
    int      max_iterations;
    double   optimality_tolerance;
    double   feasibility_tolerance;
    bool     use_warm_start;
    double  *x0;
} mpc_qp_problem_t;

typedef struct {
    int      total_iterations;
    int      active_set_changes;
    int      line_search_steps;
    int      null_space_steps;
    int      range_space_steps;
    double   objective_initial;
    double   objective_final;
    double   kkt_residual;
    double   solve_time_ms;
    bool     converged;
} mpc_qp_stats_t;

mpc_status_t mpc_qp_alloc(mpc_qp_problem_t *qp, int nv, int nc, int neq);
void mpc_qp_free(mpc_qp_problem_t *qp);
mpc_status_t mpc_qp_set_hessian(mpc_qp_problem_t *qp, const double *H, int n);
mpc_status_t mpc_qp_set_objective(mpc_qp_problem_t *qp, const double *f, int n);
mpc_status_t mpc_qp_set_constraints(mpc_qp_problem_t *qp, const double *A, int m, int n, const double *lb, const double *ub);
mpc_status_t mpc_qp_set_equalities(mpc_qp_problem_t *qp, const double *A_eq, int meq, int n, const double *b_eq);
mpc_status_t mpc_qp_append_priority_constraints(mpc_qp_problem_t *qp, const mpc_constraint_set_t *cs, mpc_priority_level_t level);
mpc_status_t mpc_qp_append_constraints_upto_priority(mpc_qp_problem_t *qp, const mpc_constraint_set_t *cs, mpc_priority_level_t max_l);
mpc_status_t mpc_qp_solve_active_set(const mpc_qp_problem_t *qp, double *x, double *lm, mpc_active_set_t *as, mpc_qp_stats_t *stats);
mpc_status_t mpc_qp_warm_start(mpc_active_set_t *as, const double *x_prev, const double *lm_prev);
mpc_status_t mpc_qp_solve_prioritized_sequential(const mpc_qp_problem_t *qp, const mpc_constraint_set_t *cs, mpc_qp_solution_t *sol, mpc_qp_stats_t *stats);
mpc_status_t mpc_qp_verify_kkt(const mpc_qp_problem_t *qp, const double *x, const double *lm, double *kkt_res);
mpc_status_t mpc_qp_constraint_sensitivity(const mpc_qp_problem_t *qp, const mpc_active_set_t *as, double *sens);
mpc_status_t mpc_qp_lagrange_multipliers(const mpc_qp_problem_t *qp, const double *x, double *mult);
mpc_status_t mpc_qp_compute_critical_region(const mpc_qp_problem_t *qp, const mpc_active_set_t *as, double *rH, double *rK);
mpc_status_t mpc_qp_find_critical_region(const mpc_qp_problem_t *qp, const double *state, int nr, const double *rH, const double *rK, int *ri);

#ifdef __cplusplus
}
#endif
#endif