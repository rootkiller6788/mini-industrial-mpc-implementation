#ifndef MPC_CONSTRAINT_FEASIBILITY_H
#define MPC_CONSTRAINT_FEASIBILITY_H
#include "mpc_constraint_defs.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int      num_conflicting_constraints;
    int     *conflicting_indices;
    double  *farkas_vector;
    double   infeasibility_measure;
    bool     is_minimal;
    int      iterations_to_find;
} mpc_infeasibility_diagnosis_t;

typedef struct {
    int      num_constraints_to_relax;
    int     *relaxation_order;
    double  *suggested_slacks;
    double   estimated_relaxation_cost;
    bool     full_recovery_possible;
    int      num_irrecoverable_constraints;
    int     *irrecoverable_indices;
} mpc_feasibility_recovery_t;

mpc_status_t mpc_feasibility_check(const mpc_constraint_set_t *cs, const double *A, int m, int n, const double *lb, const double *ub, mpc_feasibility_status_t *status, double *inf_meas);
mpc_status_t mpc_feasibility_quick_check(const mpc_constraint_set_t *cs, const mpc_constraint_propagation_t *prop, mpc_feasibility_status_t *status);
mpc_status_t mpc_feasibility_find_iis(const mpc_constraint_set_t *cs, const double *A, int m, int n, const double *lb, const double *ub, mpc_infeasibility_diagnosis_t *diag);
mpc_status_t mpc_feasibility_farkas_certificate(const double *A, int m, int n, const double *lb, const double *ub, double *fv, double *cv);
mpc_status_t mpc_feasibility_classify(const mpc_constraint_set_t *cs, const mpc_infeasibility_diagnosis_t *diag, mpc_feasibility_status_t *classif);
mpc_status_t mpc_feasibility_recovery_plan(const mpc_constraint_set_t *cs, const mpc_infeasibility_diagnosis_t *diag, mpc_feasibility_recovery_t *rec);
mpc_status_t mpc_feasibility_execute_recovery(mpc_constraint_set_t *cs, const mpc_feasibility_recovery_t *rec);
mpc_status_t mpc_feasibility_restore_constraints(mpc_constraint_set_t *cs, const mpc_feasibility_recovery_t *rec, bool *fully_restored);
double mpc_feasibility_hoffman_bound(const double *A, int m, int n, const double *lb, const double *ub, const double *x);
mpc_status_t mpc_feasibility_check_near(const double *A, int m, int n, const double *lb, const double *ub, const double *x, double tol, bool *near_feas);
mpc_status_t mpc_feasibility_minimum_relaxation(const mpc_constraint_set_t *cs, const double *A, int m, int n, const double *lb, const double *ub, double *adj);

#ifdef __cplusplus
}
#endif
#endif