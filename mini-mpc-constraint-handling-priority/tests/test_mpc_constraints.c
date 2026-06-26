/**
 * @file test_mpc_constraints.c
 * @brief Test suite for MPC constraint handling module.
 *
 * Tests all core APIs with assert-based verification.
 * Knowledge coverage: L1-L6 validation via runtime assertions.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include "../include/mpc_constraint_defs.h"
#include "../include/mpc_constraint_priority.h"
#include "../include/mpc_constraint_relaxation.h"
#include "../include/mpc_constraint_optimization.h"
#include "../include/mpc_constraint_feasibility.h"

static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) do { printf("  TEST %-50s ", name); } while(0)
#define PASS() do { printf("PASS\n"); tests_passed++; } while(0)
#define FAIL(msg) do { printf("FAIL: %s\n", msg); tests_failed++; } while(0)
#define CHECK(cond, msg) do { if (cond) PASS(); else FAIL(msg); } while(0)

/* ====================================================================
 * L1: Constraint lifecycle tests
 * ==================================================================== */

static void test_constraint_init(void)
{
    TEST("constraint_init (valid)");
    mpc_constraint_t c;
    mpc_status_t st = mpc_constraint_init(&c, 0);
    CHECK(st == MPC_OK && c.index == 0, "init failed");

    TEST("constraint_init (null)");
    st = mpc_constraint_init(NULL, 0);
    CHECK(st == MPC_ERR_NULL_POINTER, "null not rejected");

    TEST("constraint_init (negative index)");
    st = mpc_constraint_init(&c, -1);
    CHECK(st == MPC_ERR_INVALID_PRIORITY, "negative index not rejected");
}

static void test_constraint_bounds(void)
{
    TEST("constraint_set_bounds (valid)");
    mpc_constraint_t c;
    mpc_constraint_init(&c, 0);
    mpc_status_t st = mpc_constraint_set_bounds(&c, 0.0, 100.0);
    CHECK(st == MPC_OK && c.lower_bound == 0.0 && c.upper_bound == 100.0,
          "bounds not set");

    TEST("constraint_set_bounds (invalid lb>ub)");
    st = mpc_constraint_set_bounds(&c, 100.0, 0.0);
    CHECK(st == MPC_ERR_INVALID_BOUNDS, "lb>ub not rejected");

    TEST("constraint_set_bounds (null)");
    st = mpc_constraint_set_bounds(NULL, 0.0, 100.0);
    CHECK(st == MPC_ERR_NULL_POINTER, "null not rejected");
}

static void test_constraint_validate(void)
{
    TEST("constraint_validate (valid soft)");
    mpc_constraint_t c;
    mpc_constraint_init(&c, 0);
    mpc_status_t st = mpc_constraint_validate(&c);
    CHECK(st == MPC_OK, "valid soft constraint rejected");

    TEST("constraint_validate (hard+relax violation)");
    mpc_constraint_init(&c, 0);
    c.type = MPC_CONSTRAINT_HARD_INPUT;
    c.relax_policy = MPC_RELAX_IF_NEEDED;
    st = mpc_constraint_validate(&c);
    CHECK(st == MPC_ERR_INVALID_PRIORITY, "hard+relax not rejected");

    TEST("constraint_validate (critical+relax violation)");
    mpc_constraint_init(&c, 0);
    c.priority = MPC_PRIORITY_CRITICAL;
    c.relax_policy = MPC_RELAX_IF_NEEDED;
    st = mpc_constraint_validate(&c);
    CHECK(st == MPC_ERR_INVALID_PRIORITY, "critical+relax not rejected");

    TEST("constraint_validate (negative penalty)");
    mpc_constraint_init(&c, 0);
    c.penalty_weight = -1.0;
    st = mpc_constraint_validate(&c);
    CHECK(st == MPC_ERR_INVALID_BOUNDS, "negative penalty not rejected");
}

/* ====================================================================
 * L2: Hard/Soft classification tests
 * ==================================================================== */

static void test_constraint_classification(void)
{
    TEST("constraint_is_hard (hard input)");
    mpc_constraint_t c;
    mpc_constraint_init(&c, 0);
    c.type = MPC_CONSTRAINT_HARD_INPUT;
    CHECK(mpc_constraint_is_hard(&c), "hard input not recognized");

    TEST("constraint_is_hard (soft output)");
    c.type = MPC_CONSTRAINT_SOFT_OUTPUT;
    CHECK(!mpc_constraint_is_hard(&c), "soft misclassified as hard");

    TEST("constraint_is_soft (soft output)");
    CHECK(mpc_constraint_is_soft(&c), "soft not recognized");

    TEST("constraint_is_soft (hard input)");
    c.type = MPC_CONSTRAINT_HARD_INPUT;
    CHECK(!mpc_constraint_is_soft(&c), "hard misclassified as soft");

    TEST("constraint_is_hard (null)");
    CHECK(!mpc_constraint_is_hard(NULL), "null should return false");
}

/* ====================================================================
 * L3: Constraint set tests
 * ==================================================================== */

static void test_constraint_set(void)
{
    TEST("constraint_set_init (valid)");
    mpc_constraint_set_t cs;
    mpc_status_t st = mpc_constraint_set_init(&cs, 10);
    CHECK(st == MPC_OK && cs.capacity == 10 && cs.total_count == 0,
          "set init failed");

    TEST("constraint_set_init (null)");
    st = mpc_constraint_set_init(NULL, 10);
    CHECK(st == MPC_ERR_NULL_POINTER, "null not rejected");

    TEST("constraint_set_add (single)");
    mpc_constraint_t c;
    mpc_constraint_init(&c, 0);
    c.type = MPC_CONSTRAINT_HARD_INPUT;
    c.scope = MPC_SCOPE_INPUT;
    st = mpc_constraint_set_add(&cs, &c);
    CHECK(st == MPC_OK && cs.total_count == 1 && cs.count_input == 1,
          "add failed");

    TEST("constraint_set_free");
    mpc_constraint_set_free(&cs);
    CHECK(cs.constraints == NULL && cs.capacity == 0, "free incomplete");

    mpc_constraint_free(&c);
}

/* ====================================================================
 * L2: Violation checking tests
 * ==================================================================== */

static void test_violation_check(void)
{
    TEST("violation check (no violation)");
    mpc_constraint_t c;
    mpc_constraint_init(&c, 0);
    mpc_constraint_set_bounds(&c, 0.0, 100.0);
    double x[] = {50.0};
    mpc_constraint_check_violation(&c, x, 1);
    CHECK(!c.is_violated && c.violation_magnitude == 0.0,
          "false positive violation");

    TEST("violation check (upper bound violated)");
    mpc_constraint_init(&c, 0);
    mpc_constraint_set_bounds(&c, 0.0, 100.0);
    double x2[] = {150.0};
    mpc_constraint_check_violation(&c, x2, 1);
    CHECK(c.is_violated && c.violation_magnitude > 0.0,
          "upper bound violation not detected");

    TEST("violation check (lower bound violated)");
    mpc_constraint_init(&c, 0);
    mpc_constraint_set_bounds(&c, 10.0, 100.0);
    double x3[] = {5.0};
    mpc_constraint_check_violation(&c, x3, 1);
    CHECK(c.is_violated && c.violation_magnitude > 0.0,
          "lower bound violation not detected");

    TEST("violation check (null)");
    mpc_status_t st = mpc_constraint_check_violation(NULL, x, 1);
    CHECK(st == MPC_ERR_NULL_POINTER, "null not rejected");

    mpc_constraint_free(&c);
}

/* ====================================================================
 * L5: Priority sorting and indexing tests
 * ==================================================================== */

static void test_priority_sorting(void)
{
    TEST("priority sort (mixed priorities)");
    mpc_constraint_set_t cs;
    mpc_constraint_set_init(&cs, 6);
    mpc_priority_level_t levels[] = {
        MPC_PRIORITY_LOW, MPC_PRIORITY_CRITICAL, MPC_PRIORITY_HIGH,
        MPC_PRIORITY_LOW, MPC_PRIORITY_MEDIUM, MPC_PRIORITY_HIGH
    };
    for (int i = 0; i < 6; i++) {
        mpc_constraint_t c;
        mpc_constraint_init(&c, i);
        mpc_constraint_set_priority(&c, levels[i]);
        mpc_constraint_set_add(&cs, &c);
        mpc_constraint_free(&c);
    }
    mpc_constraint_sort_by_priority(&cs);
    CHECK(cs.constraints[0].priority == MPC_PRIORITY_CRITICAL,
          "CRITICAL not first after sort");
    CHECK(cs.priority_count[MPC_PRIORITY_CRITICAL] == 1,
          "critical count wrong");
    CHECK(cs.priority_count[MPC_PRIORITY_HIGH] == 2,
          "high count wrong");
    CHECK(cs.priority_count[MPC_PRIORITY_LOW] == 2,
          "low count wrong");
    mpc_constraint_set_free(&cs);
    PASS();
}

/* ====================================================================
 * L5: Relaxation tests
 * ==================================================================== */

static void test_relaxation(void)
{
    TEST("relaxation config init");
    mpc_relaxation_config_t cfg;
    mpc_status_t st = mpc_relaxation_config_init(&cfg);
    CHECK(st == MPC_OK && cfg.linear_penalty_base > 0.0, "config init failed");

    TEST("relaxation set exact penalty");
    st = mpc_relaxation_set_exact_penalty(&cfg, 2.0);
    CHECK(st == MPC_OK && cfg.use_exact_penalty, "exact penalty not set");

    TEST("relaxation null safety");
    st = mpc_relaxation_config_init(NULL);
    CHECK(st == MPC_ERR_NULL_POINTER, "null config not rejected");
}

/* ====================================================================
 * L4: Feasibility tests
 * ==================================================================== */

static void test_feasibility(void)
{
    TEST("feasibility check (feasible)");
    mpc_constraint_set_t cs;
    mpc_constraint_set_init(&cs, 4);
    double A[] = {1.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, -1.0};
    double lb[] = {0.0, 0.0, -10.0, -10.0};
    double ub[] = {10.0, 10.0, 0.0, 0.0};
    mpc_feasibility_status_t status;
    double inf_meas;
    mpc_status_t st = mpc_feasibility_check(&cs, A, 4, 2, lb, ub, &status, &inf_meas);
    CHECK(st == MPC_OK, "feasibility check failed");
    mpc_constraint_set_free(&cs);

    TEST("Hoffman bound (no violation)");
    double x[] = {5.0, 5.0};
    double hb = mpc_feasibility_hoffman_bound(A, 4, 2, lb, ub, x);
    CHECK(hb == 0.0, "hoffman bound should be 0 for feasible point");
}

/* ====================================================================
 * L6: Input saturation tests
 * ==================================================================== */

static void test_input_saturation(void)
{
    TEST("detect input saturation (at upper bound)");
    double mv[] = {100.0, 50.0};
    double mv_lb[] = {0.0, 0.0};
    double mv_ub[] = {100.0, 100.0};
    mpc_input_saturation_t sat[2];
    memset(sat, 0, sizeof(sat));
    mpc_detect_input_saturation(mv, mv_lb, mv_ub, 2, 1, sat);
    CHECK(sat[0].at_upper_bound && !sat[1].at_upper_bound,
          "saturation detection wrong");

    TEST("detect input saturation (not saturated)");
    double mv2[] = {50.0, 50.0};
    memset(sat, 0, sizeof(sat));
    mpc_detect_input_saturation(mv2, mv_lb, mv_ub, 2, 1, sat);
    CHECK(!sat[0].at_upper_bound && !sat[0].at_lower_bound,
          "false positive saturation");
}

/* ====================================================================
 * L7: Industrial vendor tests
 * ==================================================================== */

static void test_industrial_vendor(void)
{
    TEST("AspenTech DMC3 priority solve (empty set)");
    mpc_constraint_set_t cs;
    mpc_constraint_set_init(&cs, 10);
    mpc_vendor_config_t vcfg;
    memset(&vcfg, 0, sizeof(vcfg));
    mpc_qp_solution_t sol;
    memset(&sol, 0, sizeof(sol));
    mpc_status_t st = mpc_aspen_dmc3_priority_solve(&cs, &vcfg, &sol);
    CHECK(st == MPC_OK, "DMC3 solve should succeed on empty set");
    CHECK(vcfg.vendor == MPC_VENDOR_ASPENTECH_DMC, "vendor not set");
    mpc_constraint_set_free(&cs);

    TEST("Shell SMOC constraint setup");
    mpc_constraint_set_init(&cs, 10);
    mpc_constraint_t c;
    mpc_constraint_init(&c, 0);
    c.scope = MPC_SCOPE_OUTPUT;
    c.type = MPC_CONSTRAINT_HARD_OUTPUT;
    mpc_constraint_set_add(&cs, &c);
    mpc_constraint_init(&c, 1);
    c.scope = MPC_SCOPE_INPUT;
    c.type = MPC_CONSTRAINT_SOFT_INPUT;
    mpc_constraint_set_add(&cs, &c);
    memset(&vcfg, 0, sizeof(vcfg));
    st = mpc_shell_smoc_constraint_setup(&cs, &vcfg);
    CHECK(st == MPC_OK, "SMOC setup failed");
    CHECK(vcfg.vendor == MPC_VENDOR_SHELL_SMOC, "vendor not set");
    mpc_constraint_set_free(&cs);
    mpc_constraint_free(&c);
}

/* ====================================================================
 * L6: Output prioritization tests
 * ==================================================================== */

static void test_output_prioritization(void)
{
    TEST("output prioritization init");
    mpc_output_prioritization_t op;
    mpc_status_t st = mpc_output_prioritization_init(&op, 4);
    CHECK(st == MPC_OK && op.num_cv == 4, "init failed");

    TEST("output prioritization set CV");
    st = mpc_output_prioritization_set_cv(&op, 0, MPC_PRIORITY_CRITICAL,
                                           100.0, 200.0, 1000.0);
    CHECK(st == MPC_OK && op.cv_priority[0] == MPC_PRIORITY_CRITICAL,
          "set CV failed");

    TEST("output prioritization evaluate");
    double cv_vals[] = {150.0, 50.0, 300.0, 75.0};
    st = mpc_output_prioritization_evaluate(&op, cv_vals);
    CHECK(st == MPC_OK, "evaluate failed");

    mpc_output_prioritization_free(&op);
}

/* ====================================================================
 * L1: String conversion tests
 * ==================================================================== */

static void test_string_conversions(void)
{
    TEST("constraint type string");
    const char *s = mpc_constraint_type_string(MPC_CONSTRAINT_HARD_INPUT);
    CHECK(s != NULL && strcmp(s, "HARD_INPUT") == 0, "type string wrong");

    TEST("priority level string");
    s = mpc_priority_level_string(MPC_PRIORITY_CRITICAL);
    CHECK(s != NULL && strcmp(s, "CRITICAL") == 0, "priority string wrong");

    TEST("feasibility status string");
    s = mpc_feasibility_status_string(MPC_FEASIBLE);
    CHECK(s != NULL && strcmp(s, "FEASIBLE") == 0, "feasibility string wrong");
}

/* ====================================================================
 * L2: Constraint copy test
 * ==================================================================== */

static void test_constraint_copy(void)
{
    TEST("constraint copy (with coefficients)");
    mpc_constraint_t src, dest;
    mpc_constraint_init(&src, 0);
    mpc_constraint_init(&dest, 1);
    double coeffs[] = {1.0, 2.0, 3.0};
    mpc_constraint_set_coefficients(&src, coeffs, 3);
    mpc_constraint_set_bounds(&src, 0.0, 10.0);
    mpc_status_t st = mpc_constraint_copy(&dest, &src);
    CHECK(st == MPC_OK && dest.num_coefficients == 3 &&
          dest.lower_bound == 0.0 && dest.upper_bound == 10.0,
          "copy failed");

    TEST("constraint copy (null)");
    st = mpc_constraint_copy(NULL, &src);
    CHECK(st == MPC_ERR_NULL_POINTER, "null dest not rejected");

    mpc_constraint_free(&src);
    mpc_constraint_free(&dest);
}

/* ====================================================================
 * L5: Constraint evaluation with coefficients test
 * ==================================================================== */

static void test_constraint_evaluate(void)
{
    TEST("constraint evaluate (identity)");
    mpc_constraint_t c;
    mpc_constraint_init(&c, 0);
    double x[] = {42.0, 10.0};
    double val = mpc_constraint_evaluate(&c, x, 2);
    CHECK(fabs(val - 42.0) < 1e-10, "identity evaluate wrong");

    TEST("constraint evaluate (with coefficients)");
    double coeffs[] = {0.5, 0.5};
    mpc_constraint_set_coefficients(&c, coeffs, 2);
    val = mpc_constraint_evaluate(&c, x, 2);
    CHECK(fabs(val - 26.0) < 1e-10, "coefficient evaluate wrong");

    mpc_constraint_free(&c);
}

/* ====================================================================
 * Main
 * ==================================================================== */

int main(void)
{
    printf("=== MPC Constraint Handling Test Suite ===\n\n");

    printf("L1: Constraint Lifecycle\n");
    test_constraint_init();
    test_constraint_bounds();
    test_constraint_validate();
    printf("L1: %d tests\n\n", tests_passed + tests_failed - (tests_passed + tests_failed));

    printf("L2: Classification & Violation\n");
    test_constraint_classification();
    test_violation_check();
    test_constraint_copy();
    printf("\n");

    printf("L3: Constraint Set\n");
    test_constraint_set();
    printf("\n");

    printf("L5: Algorithms\n");
    test_priority_sorting();
    test_relaxation();
    test_constraint_evaluate();
    printf("\n");

    printf("L4: Feasibility\n");
    test_feasibility();
    printf("\n");

    printf("L6: Industrial Scenarios\n");
    test_input_saturation();
    test_output_prioritization();
    printf("\n");

    printf("L7: Vendor Implementations\n");
    test_industrial_vendor();
    printf("\n");

    printf("Misc\n");
    test_string_conversions();
    printf("\n");

    printf("========================================\n");
    printf("RESULTS: %d passed, %d failed\n", tests_passed, tests_failed);

    return tests_failed > 0 ? 1 : 0;
}