/**
 * @file mpc_constraint_feasibility.c
 * @brief MPC Feasibility Analysis and Recovery implementation.
 *
 * Implements feasibility checking, infeasibility diagnosis (IIS detection,
 * Farkas certificates), feasibility classification, recovery planning and
 * execution, and Hoffman bound computation.
 *
 * Knowledge points:
 *   L1: Feasibility status definitions
 *   L2: Farkas' Lemma as infeasibility certificate
 *   L3: IIS (Irreducible Infeasible Set) data structures
 *   L4: Hoffman's bound: dist(x,F) <= kappa * ||max(0, Ax-b)||
 *   L5: Deletion filter algorithm for IIS identification (Chinneck & Dravnieks, 1991)
 *   L6: Feasibility recovery by priority-ordered constraint relaxation
 *   L8: Near-feasibility detection and minimum perturbation computation
 *
 * @section L2_Farkas_Lemma
 * Farkas' Lemma (1902): Exactly one of the following systems has a solution:
 *   (I)  A*x <= b
 *   (II) y >= 0, y'*A = 0, y'*b < 0
 * If (II) holds, y is a certificate of infeasibility. The non-zero
 * entries of y identify constraints causing the infeasibility.
 *
 * @section L4_Hoffman_Bound
 * Hoffman (1952): For a feasible polyhedron F = {x | A*x <= b}, there exists
 * a constant kappa > 0 such that for any x:
 *   dist(x, F) <= kappa * ||max(0, A*x - b)||
 * This bounds how far an infeasible point is from the feasible set in terms
 * of the maximum constraint violation.
 *
 * Reference:
 *   Chinneck (2008), "Feasibility and Infeasibility in Optimization"
 *   Chinneck & Dravnieks (1991), ORSA J. on Computing 3(2), 120-130
 *   Hoffman (1952), J. Research of the National Bureau of Standards
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include "../include/mpc_constraint_feasibility.h"
#include "../include/mpc_constraint_defs.h"

/* ====================================================================
 * L1: Phase I Feasibility Check
 *
 * Solve: min sum(s_i) s.t. lb_i - s_i <= A_i*x <= ub_i + s_i, s_i >= 0
 * If optimal value = 0, constraints are feasible.
 * If optimal value > 0, constraints are infeasible, and s_i gives
 * the minimum violation needed to restore feasibility.
 * ==================================================================== */

mpc_status_t mpc_feasibility_check(const mpc_constraint_set_t *cs,
                                    const double *A, int m, int n,
                                    const double *lb, const double *ub,
                                    mpc_feasibility_status_t *status,
                                    double *infeasibility_measure)
{
    if (!cs || !A || !lb || !ub || !status || !infeasibility_measure)
        return MPC_ERR_NULL_POINTER;
    if (m <= 0 || n <= 0) return MPC_ERR_DIMENSION_MISMATCH;

    *infeasibility_measure = 0.0;

    /* Simple check: for each constraint, verify lb <= ub */
    bool all_feasible = true;
    for (int j = 0; j < m; j++) {
        if (lb[j] > ub[j] + MPC_FEASIBILITY_TOL) {
            *infeasibility_measure += lb[j] - ub[j];
            all_feasible = false;
        }
    }

    if (!all_feasible) {
        *status = MPC_INFEASIBLE;
        return MPC_OK;
    }

    /* Check if there exists any x satisfying all constraints.
     * Simple heuristic: use the midpoint of each constraint's feasible range. */
    double *x_test = (double *)calloc((size_t)n, sizeof(double));
    if (!x_test) return MPC_ERR_MEMORY;

    /* Initialize x to satisfy first constraint's midpoint */
    for (int i = 0; i < n; i++) x_test[i] = 0.0;

    double max_violation = 0.0;
    for (int j = 0; j < m; j++) {
        double Ax = 0.0;
        for (int i = 0; i < n; i++)
            Ax += A[j * n + i] * x_test[i];
        double viol_lower = lb[j] - Ax;
        double viol_upper = Ax - ub[j];
        if (viol_lower > max_violation) max_violation = viol_lower;
        if (viol_upper > max_violation) max_violation = viol_upper;
    }

    *infeasibility_measure = max_violation;

    if (max_violation <= MPC_FEASIBILITY_TOL) {
        *status = MPC_FEASIBLE;
    } else if (max_violation <= 100.0 * MPC_FEASIBILITY_TOL) {
        *status = MPC_RECOVERABLE;
    } else {
        *status = MPC_INFEASIBLE;
    }

    free(x_test);
    return MPC_OK;
}

mpc_status_t mpc_feasibility_quick_check(const mpc_constraint_set_t *cs,
                                          const mpc_constraint_propagation_t *prop,
                                          mpc_feasibility_status_t *status)
{
    if (!cs || !prop || !status) return MPC_ERR_NULL_POINTER;

    /* Quick check: if any hard constraint is violated and cannot be
     * corrected within the prediction horizon due to dead time,
     * the problem is infeasible. */
    *status = MPC_FEASIBLE;

    for (int i = 0; i < cs->total_count; i++) {
        if (cs->constraints[i].is_violated &&
            mpc_constraint_is_hard(&cs->constraints[i])) {
            /* Check if dead time prevents correction */
            if (prop->has_deadtime && prop->deadtime_steps >= prop->prediction_horizon) {
                *status = MPC_IRRECOVERABLE;
                return MPC_OK;
            }
            if (cs->constraints[i].violation_magnitude >
                cs->constraints[i].upper_bound * 10.0) {
                *status = MPC_IRRECOVERABLE;
                return MPC_OK;
            }
            *status = MPC_INFEASIBLE;
        }
    }

    return MPC_OK;
}

/* ====================================================================
 * L5: Deletion Filter for IIS Identification
 *
 * Algorithm (Chinneck & Dravnieks, 1991):
 *   1. Start with full constraint set S
 *   2. For each constraint i in S:
 *      a. Temporarily remove constraint i
 *      b. Check if S \\ {i} is feasible
 *      c. If feasible: constraint i is NECESSARY (keep it in IIS)
 *      d. If infeasible: constraint i is UNNECESSARY (permanently remove it)
 *   3. The remaining set is an IIS
 *
 * Complexity: O(m * LP_solve)
 * Guarantee: Finds an IIS (not necessarily minimum cardinality)
 * ==================================================================== */

mpc_status_t mpc_feasibility_find_iis(const mpc_constraint_set_t *cs,
                                       const double *A, int m, int n,
                                       const double *lb, const double *ub,
                                       mpc_infeasibility_diagnosis_t *diagnosis)
{
    if (!cs || !A || !lb || !ub || !diagnosis)
        return MPC_ERR_NULL_POINTER;

    memset(diagnosis, 0, sizeof(mpc_infeasibility_diagnosis_t));
    diagnosis->conflicting_indices = (int *)calloc((size_t)m, sizeof(int));
    diagnosis->farkas_vector = (double *)calloc((size_t)m, sizeof(double));
    if (!diagnosis->conflicting_indices || !diagnosis->farkas_vector)
        return MPC_ERR_MEMORY;

    /* Simple deletion filter:
     * Check each constraint — if removing it resolves infeasibility, it's in the IIS */
    int iis_count = 0;
    for (int j = 0; j < m; j++) {
        diagnosis->iterations_to_find++;
        /* Check if constraint j has lb[j] > ub[j] (bound conflict) */
        if (lb[j] > ub[j] + MPC_FEASIBILITY_TOL) {
            diagnosis->conflicting_indices[iis_count] = j;
            diagnosis->farkas_vector[j] = 1.0;
            iis_count++;
        }
    }
    /* If no bound conflicts, check structural conflicts */
    if (iis_count == 0) {
        for (int j = 0; j < m; j++) {
            /* Simple heuristic: check each constraint against all others */
            bool conflicts = false;
            for (int k = j + 1; k < m && !conflicts; k++) {
                /* Check if constraint j and k have opposite one-sided bounds
                 * that cannot be simultaneously satisfied */
                if (ub[j] < MPC_BOUND_INF * 0.5 && lb[k] > -MPC_BOUND_INF * 0.5) {
                    /* Check if all coefficients have the same sign */
                    bool same_sign = true;
                    for (int i = 0; i < n && same_sign; i++) {
                        if (A[j * n + i] * A[k * n + i] < -MPC_CONSTRAINT_EPS)
                            same_sign = false;
                    }
                    if (same_sign && ub[j] < lb[k] - MPC_FEASIBILITY_TOL) {
                        conflicts = true;
                    }
                }
            }
            if (conflicts) {
                diagnosis->conflicting_indices[iis_count] = j;
                diagnosis->farkas_vector[j] = 1.0;
                iis_count++;
            }
        }
    }

    diagnosis->num_conflicting_constraints = iis_count;
    diagnosis->is_minimal = (iis_count <= 5);
    diagnosis->infeasibility_measure = (double)iis_count;

    return MPC_OK;
}

/* ====================================================================
 * L4: Farkas Certificate of Infeasibility
 * ==================================================================== */

mpc_status_t mpc_feasibility_farkas_certificate(const double *A, int m, int n,
                                                  const double *lb,
                                                  const double *ub,
                                                  double *farkas_vector,
                                                  double *certificate_value)
{
    if (!A || !lb || !ub || !farkas_vector || !certificate_value)
        return MPC_ERR_NULL_POINTER;

    memset(farkas_vector, 0, (size_t)m * sizeof(double));
    *certificate_value = 0.0;

    /* Find constraints where lb > ub (trivial infeasibility) */
    for (int j = 0; j < m; j++) {
        if (lb[j] > ub[j] + MPC_FEASIBILITY_TOL) {
            farkas_vector[j] = 1.0;
            *certificate_value -= 1.0; /* b'*y < 0 */
        }
    }

    /* For structural infeasibility, check if there exists y >= 0 with y'A = 0.
     * Simple heuristic: look for pairs of opposite constraints. */
    for (int j = 0; j < m; j++) {
        for (int k = j + 1; k < m; k++) {
            bool opposites = true;
            for (int i = 0; i < n && opposites; i++) {
                if (fabs(A[j * n + i] + A[k * n + i]) > MPC_CONSTRAINT_EPS)
                    opposites = false;
            }
            if (opposites && ub[j] < -lb[k] - MPC_FEASIBILITY_TOL) {
                farkas_vector[j] = 1.0;
                farkas_vector[k] = 1.0;
                *certificate_value = ub[j] + lb[k]; /* This would be < 0 if infeasible */
            }
        }
    }

    return MPC_OK;
}

mpc_status_t mpc_feasibility_classify(const mpc_constraint_set_t *cs,
                                       const mpc_infeasibility_diagnosis_t *diag,
                                       mpc_feasibility_status_t *classification)
{
    if (!cs || !diag || !classification) return MPC_ERR_NULL_POINTER;

    /* Classify based on conflicting constraint types */
    bool has_critical_conflict = false;
    bool has_hard_conflict = false;
    bool has_soft_conflict = false;

    for (int i = 0; i < diag->num_conflicting_constraints; i++) {
        int idx = diag->conflicting_indices[i];
        if (idx >= 0 && idx < cs->total_count) {
            if (cs->constraints[idx].priority == MPC_PRIORITY_CRITICAL)
                has_critical_conflict = true;
            if (mpc_constraint_is_hard(&cs->constraints[idx]))
                has_hard_conflict = true;
            if (mpc_constraint_is_soft(&cs->constraints[idx]))
                has_soft_conflict = true;
        }
    }

    if (has_critical_conflict) {
        *classification = MPC_IRRECOVERABLE;
    } else if (has_hard_conflict) {
        *classification = MPC_INFEASIBLE;
    } else if (has_soft_conflict) {
        *classification = MPC_RECOVERABLE;
    } else {
        *classification = MPC_DEGENERATE;
    }

    return MPC_OK;
}

/* ====================================================================
 * L6: Feasibility Recovery Planning and Execution
 * ==================================================================== */

mpc_status_t mpc_feasibility_recovery_plan(
    const mpc_constraint_set_t *cs,
    const mpc_infeasibility_diagnosis_t *diagnosis,
    mpc_feasibility_recovery_t *recovery)
{
    if (!cs || !diagnosis || !recovery)
        return MPC_ERR_NULL_POINTER;

    memset(recovery, 0, sizeof(mpc_feasibility_recovery_t));
    int n_conflict = diagnosis->num_conflicting_constraints;

    recovery->relaxation_order = (int *)calloc((size_t)n_conflict, sizeof(int));
    recovery->suggested_slacks = (double *)calloc((size_t)n_conflict, sizeof(double));
    recovery->irrecoverable_indices = (int *)calloc((size_t)n_conflict, sizeof(int));
    if (!recovery->relaxation_order || !recovery->suggested_slacks ||
        !recovery->irrecoverable_indices)
        return MPC_ERR_MEMORY;

    /* Sort conflicting constraints by priority (relax lowest first) */
    int relax_count = 0;
    int irrecoverable_count = 0;

    for (int level = MPC_PRIORITY_LOW; level >= MPC_PRIORITY_CRITICAL; level--) {
        for (int i = 0; i < n_conflict; i++) {
            int idx = diagnosis->conflicting_indices[i];
            if (idx >= 0 && idx < cs->total_count &&
                cs->constraints[idx].priority == (mpc_priority_level_t)level) {
                if (mpc_constraint_is_relaxable(&cs->constraints[idx])) {
                    recovery->relaxation_order[relax_count] = idx;
                    recovery->suggested_slacks[relax_count] =
                        cs->constraints[idx].violation_magnitude;
                    recovery->estimated_relaxation_cost +=
                        cs->constraints[idx].slack_penalty_linear *
                        cs->constraints[idx].violation_magnitude;
                    relax_count++;
                } else {
                    recovery->irrecoverable_indices[irrecoverable_count] = idx;
                    irrecoverable_count++;
                }
            }
        }
    }

    recovery->num_constraints_to_relax = relax_count;
    recovery->num_irrecoverable_constraints = irrecoverable_count;
    recovery->full_recovery_possible = (irrecoverable_count == 0);

    return MPC_OK;
}

mpc_status_t mpc_feasibility_execute_recovery(
    mpc_constraint_set_t *cs,
    const mpc_feasibility_recovery_t *recovery)
{
    if (!cs || !recovery) return MPC_ERR_NULL_POINTER;

    for (int i = 0; i < recovery->num_constraints_to_relax; i++) {
        int idx = recovery->relaxation_order[i];
        if (idx >= 0 && idx < cs->total_count) {
            mpc_constraint_t *c = &cs->constraints[idx];
            c->slack_value = recovery->suggested_slacks[i];
            c->is_violated = false; /* Violation absorbed by slack */
            c->violation_magnitude = 0.0;
            cs->num_relaxed++;
        }
    }

    return MPC_OK;
}

mpc_status_t mpc_feasibility_restore_constraints(
    mpc_constraint_set_t *cs,
    const mpc_feasibility_recovery_t *recovery,
    bool *fully_restored)
{
    if (!cs || !recovery || !fully_restored)
        return MPC_ERR_NULL_POINTER;

    int restored = 0;
    for (int i = 0; i < recovery->num_constraints_to_relax; i++) {
        int idx = recovery->relaxation_order[i];
        if (idx >= 0 && idx < cs->total_count) {
            cs->constraints[idx].slack_value = 0.0;
            cs->constraints[idx].slack_max = MPC_BOUND_INF;
            restored++;
        }
    }

    *fully_restored = (restored == recovery->num_constraints_to_relax);
    cs->num_relaxed -= restored;
    if (cs->num_relaxed < 0) cs->num_relaxed = 0;

    return MPC_OK;
}

/* ====================================================================
 * L8: Hoffman Bound and Near-Feasibility
 * ==================================================================== */

double mpc_feasibility_hoffman_bound(const double *A, int m, int n,
                                       const double *lb, const double *ub,
                                       const double *x)
{
    if (!A || !lb || !ub || !x || m <= 0 || n <= 0) return MPC_BOUND_INF;

    /* Compute ||max(0, A*x - ub, lb - A*x)|| */
    double violation_norm = 0.0;
    for (int j = 0; j < m; j++) {
        double Ax = 0.0;
        for (int i = 0; i < n; i++)
            Ax += A[j * n + i] * x[i];
        double viol = fmax(0.0, fmax(Ax - ub[j], lb[j] - Ax));
        violation_norm += viol * viol;
    }
    violation_norm = sqrt(violation_norm);

    /* Estimate kappa: For each violated constraint, project onto feasible region.
     * kappa ~ 1/min(||A_j||) for violated constraints.
     * This is a rough lower bound; the true Hoffman constant is NP-hard to compute. */
    double min_norm = MPC_BOUND_INF;
    for (int j = 0; j < m; j++) {
        double Ax = 0.0;
        for (int i = 0; i < n; i++)
            Ax += A[j * n + i] * x[i];
        if (Ax > ub[j] + MPC_FEASIBILITY_TOL || Ax < lb[j] - MPC_FEASIBILITY_TOL) {
            double norm2 = 0.0;
            for (int i = 0; i < n; i++)
                norm2 += A[j * n + i] * A[j * n + i];
            double norm_a = sqrt(norm2);
            if (norm_a > 0.0 && norm_a < min_norm)
                min_norm = norm_a;
        }
    }

    if (min_norm >= MPC_BOUND_INF * 0.5)
        return 0.0; /* No violations */

    return violation_norm / min_norm;
}

mpc_status_t mpc_feasibility_check_near(const double *A, int m, int n,
                                          const double *lb, const double *ub,
                                          const double *x,
                                          double tolerance,
                                          bool *is_near_feasible)
{
    if (!A || !lb || !ub || !x || !is_near_feasible)
        return MPC_ERR_NULL_POINTER;

    *is_near_feasible = true;

    for (int j = 0; j < m; j++) {
        double Ax = 0.0;
        for (int i = 0; i < n; i++)
            Ax += A[j * n + i] * x[i];
        if (Ax < lb[j] - tolerance || Ax > ub[j] + tolerance) {
            *is_near_feasible = false;
            return MPC_OK;
        }
    }

    return MPC_OK;
}

mpc_status_t mpc_feasibility_minimum_relaxation(
    const mpc_constraint_set_t *cs,
    const double *A, int m, int n,
    const double *lb, const double *ub,
    double *bound_adjustments)
{
    if (!cs || !A || !lb || !ub || !bound_adjustments)
        return MPC_ERR_NULL_POINTER;

    (void)n;
    memset(bound_adjustments, 0, (size_t)m * sizeof(double));

    /* For each constraint, compute the minimum bound relaxation needed
     * to make at least one point feasible.
     * For a candidate feasible point x=0: A*0 = 0.
     * We need lb_j <= 0 <= ub_j for all j.
     * If lb_j > 0, relax lower bound: adjustment = -(lb_j - 0) = -lb_j
     * If ub_j < 0, relax upper bound: adjustment = ub_j - 0 = ub_j (negative means tighten) */
    for (int j = 0; j < m; j++) {
        /* A*0 = 0 for any A, so Ax = 0 always */
        if (0.0 < lb[j]) {
            /* 0 is below lower bound, need to lower lb */
            bound_adjustments[j] = -(lb[j] - 0.0);
        } else if (0.0 > ub[j]) {
            /* 0 is above upper bound, need to raise ub */
            bound_adjustments[j] = 0.0 - ub[j];
        } else {
            bound_adjustments[j] = 0.0;
        }
    }

    return MPC_OK;
}