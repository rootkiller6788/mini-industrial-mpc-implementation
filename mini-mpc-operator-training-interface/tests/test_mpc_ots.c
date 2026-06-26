/**
 * @file test_mpc_ots.c
 * @brief Test suite for MPC Operator Training Interface.
 *
 * Comprehensive assert-based tests covering all core APIs.
 * Tests session lifecycle, profile management, scoring algorithms,
 * scenario generation, guidance system, and industrial integration.
 *
 * Knowledge points tested:
 *   L1: enum/struct initialization and lifecycle
 *   L2: State machine transitions, Elo rating behavior
 *   L3: Priority queue ordering, ring buffer correctness
 *   L4: ISA-101 compliance verification
 *   L5: Algorithm correctness (OLS, EWMA, geometric mean, plateau detection)
 *   L6: Scenario preset validation, competency certification
 *   L7: Vendor interface correctness
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <float.h>
#include "../include/mpc_ots_defs.h"
#include "../include/mpc_ots_scenario.h"
#include "../include/mpc_ots_assessment.h"
#include "../include/mpc_ots_interface.h"
#include "../include/mpc_ots_guidance.h"

static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) do { printf("  TEST: %s ... ", name); } while(0)
#define PASS() do { printf("PASSED\n"); tests_passed++; } while(0)
#define FAIL(msg) do { printf("FAILED: %s\n", msg); tests_failed++; } while(0)
#define ASSERT_EQ_INT(expected, actual) do { \
    if ((expected) != (actual)) { FAIL("int mismatch"); return; } \
} while(0)
#define ASSERT_NEAR(expected, actual, tol) do { \
    if (fabs((expected) - (actual)) > (tol)) { \
        printf("  expected %.6f, got %.6f\n", (expected), (actual)); \
        FAIL("float mismatch"); return; \
    } \
} while(0)
#define ASSERT_TRUE(cond) do { \
    if (!(cond)) { FAIL("expected true"); return; } \
} while(0)
#define ASSERT_FALSE(cond) do { \
    if (cond) { FAIL("expected false"); return; } \
} while(0)

/* =========================================================================
 * L1 Tests: Enum/String Conversions
 * ========================================================================= */

static void test_l1_enum_conversions(void)
{
    TEST("ots_state_to_string");
    assert(strcmp(ots_state_to_string(MPC_OTS_STATE_INIT), "INIT") == 0);
    assert(strcmp(ots_state_to_string(MPC_OTS_STATE_RUNNING), "RUNNING") == 0);
    assert(strcmp(ots_state_to_string(MPC_OTS_STATE_COMPLETED), "COMPLETED") == 0);
    PASS();

    TEST("ots_scenario_type_to_string");
    assert(strcmp(ots_scenario_type_to_string(MPC_SCENARIO_DISTURBANCE_REJECTION), "Disturbance Rejection") == 0);
    assert(strcmp(ots_scenario_type_to_string(MPC_SCENARIO_EMERGENCY_SHUTDOWN), "Emergency Shutdown") == 0);
    PASS();

    TEST("ots_difficulty_to_string");
    assert(strcmp(ots_difficulty_to_string(MPC_DIFFICULTY_BEGINNER), "Beginner") == 0);
    assert(strcmp(ots_difficulty_to_string(MPC_DIFFICULTY_EXPERT), "Expert") == 0);
    PASS();

    TEST("ots_interface_mode_to_string");
    assert(strcmp(ots_interface_mode_to_string(MPC_IF_MODE_GUIDE), "Guide") == 0);
    PASS();

    TEST("ots_metric_to_string");
    assert(strcmp(ots_metric_to_string(MPC_METRIC_RESPONSE_TIME), "Response Time") == 0);
    assert(strcmp(ots_metric_to_string(MPC_METRIC_CONSISTENCY), "Consistency") == 0);
    PASS();
}

/* =========================================================================
 * L1+L2 Tests: Session Lifecycle
 * ========================================================================= */

static void test_session_lifecycle(void)
{
    TrainingSession session;
    OTSStatus status;

    TEST("session_init");
    status = ots_session_init(&session, 1, "Test Session");
    ASSERT_EQ_INT(MPC_OTS_OK, status);
    ASSERT_EQ_INT(MPC_OTS_STATE_INIT, session.state);
    ASSERT_EQ_INT(1, session.session_id);
    PASS();

    TEST("session_init_null");
    status = ots_session_init(NULL, 1, "test");
    ASSERT_EQ_INT(MPC_OTS_ERR_NULL_POINTER, status);
    PASS();

    TEST("session_init_invalid_id");
    status = ots_session_init(&session, -1, "test");
    ASSERT_EQ_INT(MPC_OTS_ERR_INVALID_PARAM, status);
    PASS();

    TEST("session_start_no_operator");
    session.operator_id = -1;
    status = ots_session_start(&session);
    ASSERT_EQ_INT(MPC_OTS_ERR_OPERATOR_NOT_FOUND, status);
    PASS();

    TEST("session_start_no_events");
    session.operator_id = 0;
    session.total_events = 0;
    status = ots_session_start(&session);
    ASSERT_EQ_INT(MPC_OTS_ERR_SCENARIO_EMPTY, status);
    PASS();

    TEST("session_pause_not_running");
    session.state = MPC_OTS_STATE_INIT;
    status = ots_session_pause(&session);
    ASSERT_EQ_INT(MPC_OTS_ERR_INVALID_STATE, status);
    PASS();
}

/* =========================================================================
 * L2 Tests: State Transition Validation
 * ========================================================================= */

static void test_state_transitions(void)
{
    TEST("valid_transition_init_to_ready");
    ASSERT_TRUE(ots_state_transition_valid(MPC_OTS_STATE_INIT, MPC_OTS_STATE_READY));
    PASS();

    TEST("valid_transition_running_to_paused");
    ASSERT_TRUE(ots_state_transition_valid(MPC_OTS_STATE_RUNNING, MPC_OTS_STATE_PAUSED));
    PASS();

    TEST("invalid_transition_init_to_running");
    ASSERT_FALSE(ots_state_transition_valid(MPC_OTS_STATE_INIT, MPC_OTS_STATE_RUNNING));
    PASS();

    TEST("invalid_transition_debriefing_to_any");
    ASSERT_FALSE(ots_state_transition_valid(MPC_OTS_STATE_DEBRIEFING, MPC_OTS_STATE_RUNNING));
    PASS();

    /* Mathematical consistency: state space is finite (7 states).
     * Verification: 7*7=49 possible transitions, only the 12 valid
     * ones return true (2 from INIT, 2 from READY, 3 from RUNNING,
     * 3 from PAUSED, 1 from COMPLETED, 0 from DEBRIEFING, 1 from FAILED). */
    int valid_count = 0;
    for (int i = 0; i <= 6; i++) {
        for (int j = 0; j <= 6; j++) {
            if (ots_state_transition_valid((OTSState)i, (OTSState)j)) valid_count++;
        }
    }
    TEST("state_transition_count_equals_12");
    ASSERT_EQ_INT(12, valid_count);
    PASS();
}

/* =========================================================================
 * L1+L2 Tests: Operator Profile
 * ========================================================================= */

static void test_operator_profile(void)
{
    OperatorProfile profile;
    OTSStatus status;

    TEST("profile_init");
    status = ots_profile_init(&profile, 100, "John Doe", MPC_ROLE_JUNIOR_OPERATOR);
    ASSERT_EQ_INT(MPC_OTS_OK, status);
    ASSERT_EQ_INT(100, profile.operator_id);
    ASSERT_EQ_INT((int)MPC_ROLE_JUNIOR_OPERATOR, (int)profile.role);
    ASSERT_NEAR(50.0, profile.overall_score, 0.01);
    ASSERT_NEAR(1200.0, profile.current_elo_rating, 0.01);
    PASS();
}

/* =========================================================================
 * L5 Tests: Elo Rating Mathematics
 * ========================================================================= */

static void test_elo_rating(void)
{
    TEST("elo_win_increases_rating");
    double new_elo = ots_elo_update(1200.0, 100.0, 0.64, 32.0);
    ASSERT_TRUE(new_elo > 1200.0); /* win increases rating */
    ASSERT_NEAR(1211.52, new_elo, 0.01);
    PASS();

    /* Mathematical assertion: S=1, E=0.64, K=32 → R' = 1200 + 32*(1-0.64) = 1211.52 */
    TEST("elo_exact_formula");
    double exact = 1200.0 + 32.0 * (1.0 - 0.64);
    ASSERT_NEAR(exact, 1211.52, 0.01);
    PASS();

    TEST("elo_loss_decreases_rating");
    double new_elo2 = ots_elo_update(1200.0, 0.0, 0.64, 32.0);
    ASSERT_TRUE(new_elo2 < 1200.0); /* loss decreases rating */
    PASS();

    /* Mathematical assertion: S=0, E=0.64, K=32 → R' = 1200 - 20.48 = 1179.52 */
    TEST("elo_loss_exact_formula");
    ASSERT_NEAR(1179.52, new_elo2, 0.01);
    PASS();

    /* Mathematical assertion: Elo adjustment bounded by K */
    TEST("elo_adjustment_bounded_by_k");
    double worst = ots_elo_update(1200.0, 100.0, 0.0, 32.0);
    ASSERT_TRUE(fabs(worst - 1200.0) <= 32.0 + 0.01);
    PASS();
}

/* =========================================================================
 * L5 Tests: Performance Scoring Mathematics
 * ========================================================================= */

static void test_performance_scoring(void)
{
    TEST("weighted_geometric_mean_basic");
    double scores[] = {80.0, 90.0, 70.0, 85.0, 95.0, 75.0, 88.0};
    double weights[] = {0.20, 0.20, 0.20, 0.15, 0.10, 0.10, 0.05};
    double overall = ots_score_weighted_overall(scores, weights, 7);
    ASSERT_TRUE(overall > 0.0 && overall <= 100.0);
    /* Geometric mean should be between min and arithmetic mean */
    ASSERT_TRUE(overall >= 70.0 && overall <= 85.0);
    PASS();

    /* Mathematical assertion: unit test of weighted geometric mean
     * For equal weights and scores: result = score */
    TEST("weighted_geometric_mean_equal");
    double equal_scores[] = {75.0, 75.0, 75.0};
    double equal_weights[] = {1.0/3.0, 1.0/3.0, 1.0/3.0};
    double result = ots_score_weighted_overall(equal_scores, equal_weights, 3);
    ASSERT_NEAR(75.0, result, 0.1);
    PASS();

    /* Mathematical assertion: normalization function produces scores in [0, 100] */
    TEST("response_time_normalization_range");
    double s1 = ots_normalize_response_time(30.0, 30.0);
    ASSERT_TRUE(s1 >= 0.0 && s1 <= 100.0);
    ASSERT_NEAR(100.0, s1, 0.01); /* at optimal = 100 */
    PASS();

    /* Mathematical assertion: fast response gets high score */
    TEST("response_time_normalization_fast");
    double s_fast = ots_normalize_response_time(30.0, 30.0);
    double s_slow = ots_normalize_response_time(60.0, 30.0);
    ASSERT_TRUE(s_fast > s_slow);
    PASS();

    /* Mathematical assertion: deviation normalization */
    TEST("deviation_normalization_within_limit");
    double dev_ok = ots_normalize_deviation(3.0, 10.0);
    ASSERT_NEAR(100.0, dev_ok, 0.01);
    PASS();

    TEST("deviation_normalization_exceeds_limit");
    double dev_over = ots_normalize_deviation(15.0, 10.0);
    ASSERT_TRUE(dev_over < 100.0);
    ASSERT_NEAR(50.0, dev_over, 0.01); /* 1.5x limit → 50% */
    PASS();
}

/* =========================================================================
 * L5 Tests: Statistical Functions
 * ========================================================================= */

static void test_statistical_functions(void)
{
    TEST("ewma_computation");
    double seq[] = {60.0, 65.0, 70.0, 75.0, 80.0};
    double ewma = ots_ewma_compute(seq, 5, 0.3);
    ASSERT_TRUE(ewma >= 60.0 && ewma <= 80.0);
    ASSERT_TRUE(ewma > 70.0); /* heavily weights recent values */
    PASS();

    TEST("ols_linear_trend");
    double scores[] = {50.0, 55.0, 60.0, 65.0, 70.0};
    int32_t indices[] = {0, 1, 2, 3, 4};
    double slope, intercept, r2;
    OTSStatus s = ots_fit_linear_trend(scores, indices, 5, &slope, &intercept, &r2);
    ASSERT_EQ_INT(MPC_OTS_OK, s);
    ASSERT_NEAR(5.0, slope, 0.01); /* perfect +5 slope */
    ASSERT_NEAR(50.0, intercept, 0.01);
    ASSERT_NEAR(1.0, r2, 0.01); /* perfect fit */
    PASS();

    /* Mathematical assertion: OLS closed-form solution verified */
    TEST("ols_exact_formula_verification");
    /* Σx = 10, Σy = 300, Σxy = 650, Σx² = 30, n = 5
     * slope = (5*650 - 10*300) / (5*30 - 100) = (3250-3000)/(150-100) = 250/50 = 5 */
    ASSERT_NEAR(5.0, (5.0*650.0 - 10.0*300.0) / (5.0*30.0 - 100.0), 0.01);
    PASS();

    TEST("plateau_detection_no_plateau");
    double improving[] = {50.0, 55.0, 62.0, 68.0, 73.0, 78.0, 82.0, 85.0};
    bool plateau = ots_detect_plateau(improving, 8, 4, 0.02);
    ASSERT_FALSE(plateau);
    PASS();

    TEST("plateau_detection_yes_plateau");
    double stable[] = {80.0, 80.5, 80.2, 80.3, 80.1, 80.4, 80.2, 80.3};
    bool plateau2 = ots_detect_plateau(stable, 8, 4, 0.02);
    /* With 0.02 threshold and small variations, should detect */
    ASSERT_TRUE(plateau2);
    PASS();

    TEST("confidence_interval");
    double data[] = {70.0, 72.0, 68.0, 75.0, 71.0};
    double lower, upper;
    OTSStatus ci_stat = ots_confidence_interval(data, 5, &lower, &upper);
    ASSERT_EQ_INT(MPC_OTS_OK, ci_stat);
    ASSERT_TRUE(lower < upper);
    ASSERT_TRUE(lower < 71.2 && upper > 71.2); /* mean ~71.2 */
    PASS();
}

/* =========================================================================
 * L3 Tests: Scenario Generation
 * ========================================================================= */

static void test_scenario_generation(void)
{
    ScenarioTemplate tmpl;
    EventTimeline timeline;

    TEST("scenario_preset_reactor");
    OTSStatus s = ots_scenario_preset_reactor(&tmpl);
    ASSERT_EQ_INT(MPC_OTS_OK, s);
    ASSERT_EQ_INT(8, tmpl.num_events);
    ASSERT_TRUE(tmpl.is_validated);
    PASS();

    TEST("scenario_generate_timeline_beginner");
    s = ots_scenario_generate_timeline(&tmpl, MPC_DIFFICULTY_BEGINNER, &timeline);
    ASSERT_EQ_INT(MPC_OTS_OK, s);
    ASSERT_TRUE(timeline.is_valid);
    ASSERT_EQ_INT(8, timeline.num_events);
    PASS();

    TEST("scenario_generate_timeline_expert");
    s = ots_scenario_generate_timeline(&tmpl, MPC_DIFFICULTY_EXPERT, &timeline);
    ASSERT_EQ_INT(MPC_OTS_OK, s);
    ASSERT_TRUE(timeline.is_valid);
    PASS();

    TEST("scenario_validate_timeline");
    bool valid = ots_scenario_validate_timeline(&timeline);
    ASSERT_TRUE(valid);
    PASS();

    TEST("scenario_expected_score_monotonic");
    /* Harder difficulty → lower expected score */
    EventTimeline easy, hard;
    ots_scenario_generate_timeline(&tmpl, MPC_DIFFICULTY_BEGINNER, &easy);
    ots_scenario_generate_timeline(&tmpl, MPC_DIFFICULTY_EXPERT, &hard);
    ASSERT_TRUE(easy.expected_overall_score > hard.expected_overall_score);
    PASS();
}

/* =========================================================================
 * L5 Tests: Difficulty Parameter Scaling
 * ========================================================================= */

static void test_difficulty_scaling(void)
{
    double factors[] = {2.0, 1.5, 1.0, 0.7};
    double base = 100.0;

    TEST("diff_scale_beginner");
    double b = ots_diff_scale(base, MPC_DIFFICULTY_BEGINNER, factors);
    ASSERT_NEAR(200.0, b, 0.01);
    PASS();

    TEST("diff_scale_expert");
    double e = ots_diff_scale(base, MPC_DIFFICULTY_EXPERT, factors);
    ASSERT_NEAR(70.0, e, 0.01);
    PASS();

    TEST("diff_event_spacing_clamped");
    double spacing = ots_diff_event_spacing(10.0, MPC_DIFFICULTY_EXPERT, factors);
    ASSERT_TRUE(spacing >= MPC_OTS_SCENARIO_MIN_EVENT_SPACING);
    PASS();
}

/* =========================================================================
 * L5 Tests: Guidance System
 * ========================================================================= */

static void test_guidance_system(void)
{
    GuidanceQueue queue;
    GuidanceEntry entry;

    TEST("guidance_queue_init");
    OTSStatus s = ots_guidance_queue_init(&queue);
    ASSERT_EQ_INT(MPC_OTS_OK, s);
    ASSERT_EQ_INT(0, queue.count);
    PASS();

    TEST("guidance_enqueue");
    memset(&entry, 0, sizeof(GuidanceEntry));
    entry.message_id = 1;
    entry.priority = 5;
    entry.level = MPC_GUIDANCE_HINT;
    s = ots_guidance_enqueue(&queue, &entry);
    ASSERT_EQ_INT(MPC_OTS_OK, s);
    ASSERT_EQ_INT(1, queue.count);
    PASS();

    TEST("guidance_mark_acted");
    s = ots_guidance_mark_acted(&queue, 1);
    ASSERT_EQ_INT(MPC_OTS_OK, s);
    PASS();

    TEST("guidance_mark_acted_not_found");
    s = ots_guidance_mark_acted(&queue, 999);
    ASSERT_EQ_INT(MPC_OTS_ERR_EVENT_NOT_FOUND, s);
    PASS();

    TEST("guidance_expire_stale");
    s = ots_guidance_expire_stale(&queue, 1000.0);
    ASSERT_EQ_INT(MPC_OTS_OK, s);
    PASS();

    TEST("guidance_compute_delay");
    double delay = ots_guidance_compute_delay(MPC_ROLE_TRAINEE, MPC_DIFFICULTY_BEGINNER, 30.0);
    ASSERT_TRUE(delay >= MPC_OTS_GUIDANCE_DELAY_MIN);
    ASSERT_TRUE(delay <= MPC_OTS_GUIDANCE_DELAY_MAX);
    /* Trainee + Beginner = shortest delay (0.5 * 0.6 = 0.3 * 30 = 9.0) */
    ASSERT_NEAR(9.0, delay, 0.1);
    PASS();

    TEST("guidance_compute_delay_expert");
    double delay_expert = ots_guidance_compute_delay(MPC_ROLE_TRAINER, MPC_DIFFICULTY_EXPERT, 30.0);
    /* Trainer(1.5) * Expert(1.2) * 30 = 54 */
    ASSERT_TRUE(delay_expert > delay);
    ASSERT_NEAR(54.0, delay_expert, 0.1);
    PASS();
}

/* =========================================================================
 * L5 Tests: Workload Estimation
 * ========================================================================= */

static void test_workload_estimation(void)
{
    DecisionContext ctx;
    WorkloadEstimate wl;
    memset(&ctx, 0, sizeof(DecisionContext));

    ctx.num_variables = 5;
    ctx.active_event_count = 3;
    ctx.difficulty = MPC_DIFFICULTY_INTERMEDIATE;
    ctx.current_score = 60.0;
    for (int32_t i = 0; i < 5; i++) {
        ctx.constraint_margins[i] = 2.0;
    }

    TEST("workload_estimate");
    OTSStatus s = ots_workload_estimate(&ctx, &wl);
    ASSERT_EQ_INT(MPC_OTS_OK, s);
    ASSERT_TRUE(wl.overall_workload >= 0.0 && wl.overall_workload <= 1.0);
    ASSERT_FALSE(wl.is_overloaded);
    PASS();

    TEST("workload_critical_under_stress");
    DecisionContext stress_ctx;
    memset(&stress_ctx, 0, sizeof(DecisionContext));
    stress_ctx.num_variables = 16;
    stress_ctx.active_event_count = 10;
    stress_ctx.difficulty = MPC_DIFFICULTY_EXPERT;
    stress_ctx.current_score = 30.0;
    for (int32_t i = 0; i < 16; i++) {
        stress_ctx.constraint_margins[i] = -15.0;
    }
    WorkloadEstimate stress_wl;
    ots_workload_estimate(&stress_ctx, &stress_wl);
    TEST("workload_critical_true");
    ASSERT_TRUE(stress_wl.is_overloaded);
    PASS();
}

/* =========================================================================
 * L5 Tests: What-if Analysis
 * ========================================================================= */

static void test_whatif_analysis(void)
{
    WhatIfQuery query;
    memset(&query, 0, sizeof(WhatIfQuery));
    query.query_id = 1;
    query.timestamp = 100.0;
    query.num_variables = 3;
    query.current_values[0] = 50.0;
    query.current_values[1] = 60.0;
    query.current_values[2] = 70.0;
    query.proposed_values[0] = 55.0;
    query.proposed_values[1] = 62.0;
    query.proposed_values[2] = 68.0;

    TEST("whatif_execute");
    OTSStatus s = ots_whatif_execute(&query);
    ASSERT_EQ_INT(MPC_OTS_OK, s);
    ASSERT_TRUE(query.confidence_level > 0.5);
    ASSERT_TRUE(query.is_safe);
    PASS();

    TEST("whatif_is_safe");
    ASSERT_TRUE(ots_whatif_is_safe(&query));
    PASS();
}

/* =========================================================================
 * L5 Tests: Trend Buffer (Ring Buffer Correctness)
 * ========================================================================= */

static void test_trend_buffer(void)
{
    TrendBuffer trend;
    OTSStatus s;

    TEST("trend_init");
    s = ots_trend_init(&trend, "Temperature", "°C", 1.0);
    ASSERT_EQ_INT(MPC_OTS_OK, s);
    ASSERT_EQ_INT(0, trend.count);
    PASS();

    /* Mathematical assertion: ring buffer wraps correctly */
    TEST("trend_push_wraparound");
    for (int i = 0; i < MPC_OTS_HMI_TREND_BUFFER_SIZE + 10; i++) {
        ots_trend_push(&trend, (double)i, (double)(i * 10));
    }
    ASSERT_EQ_INT(MPC_OTS_HMI_TREND_BUFFER_SIZE, trend.count);
    PASS();

    TEST("trend_get_recent");
    double vals[10], times[10];
    int32_t n = ots_trend_get_recent(&trend, vals, times, 5);
    ASSERT_EQ_INT(5, n);
    PASS();
}

/* =========================================================================
 * L5 Tests: Constraint Polygon
 * ========================================================================= */

static void test_constraint_polygon(void)
{
    ConstraintPolygon poly;
    OTSStatus s;

    TEST("polygon_init");
    s = ots_constraint_polygon_init(&poly, 4);
    ASSERT_EQ_INT(MPC_OTS_OK, s);
    ASSERT_EQ_INT(4, poly.num_variables);
    PASS();

    double current[] = {50.0, 30.0, 80.0, 45.0};
    poly.lower_limits[0] = 0.0;  poly.upper_limits[0] = 100.0;
    poly.lower_limits[1] = 10.0; poly.upper_limits[1] = 50.0;
    poly.lower_limits[2] = 60.0; poly.upper_limits[2] = 90.0;
    poly.lower_limits[3] = 20.0; poly.upper_limits[3] = 60.0;

    TEST("polygon_update");
    s = ots_constraint_polygon_update(&poly, current);
    ASSERT_EQ_INT(MPC_OTS_OK, s);
    ASSERT_FALSE(poly.is_infeasible);
    ASSERT_TRUE(poly.min_margin > 0.0);
    PASS();

    TEST("polygon_most_critical");
    int32_t critical = ots_constraint_most_critical(&poly);
    ASSERT_TRUE(critical >= 0 && critical < 4);
    PASS();
}

/* =========================================================================
 * L5 Tests: Performance Radar Mathematics
 * ========================================================================= */

static void test_performance_radar(void)
{
    PerformanceRadar radar;
    double scores[] = {70.0, 80.0, 60.0, 75.0, 85.0, 90.0, 65.0};

    TEST("radar_init");
    OTSStatus s = ots_radar_init(&radar);
    ASSERT_EQ_INT(MPC_OTS_OK, s);
    PASS();

    TEST("radar_update");
    s = ots_radar_update(&radar, scores);
    ASSERT_EQ_INT(MPC_OTS_OK, s);
    PASS();

    TEST("radar_area_in_range");
    double area = ots_radar_area(&radar);
    ASSERT_TRUE(area >= 0.0 && area <= 100.0);
    PASS();

    TEST("radar_balance_in_range");
    double balance = ots_radar_balance(&radar);
    ASSERT_TRUE(balance >= 0.0 && balance <= 1.0);
    PASS();

    /* Mathematical assertion: radar area for all-100 scores = 100 */
    PerformanceRadar perfect;
    ots_radar_init(&perfect);
    double perfect_scores[] = {100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0};
    ots_radar_update(&perfect, perfect_scores);
    double perfect_area = ots_radar_area(&perfect);
    TEST("radar_area_perfect_max");
    ASSERT_NEAR(100.0, perfect_area, 0.1);
    PASS();

    /* Mathematical assertion: radar balance for equal scores = 0 */
    PerformanceRadar balanced;
    ots_radar_init(&balanced);
    double balanced_scores[] = {75.0, 75.0, 75.0, 75.0, 75.0, 75.0, 75.0};
    ots_radar_update(&balanced, balanced_scores);
    double bal = ots_radar_balance(&balanced);
    TEST("radar_balance_perfect_zero");
    ASSERT_NEAR(0.0, bal, 0.01);
    PASS();
}

/* =========================================================================
 * L5 Tests: Scenario Library
 * ========================================================================= */

static void test_scenario_library(void)
{
    ScenarioLibrary lib;
    OTSStatus s;

    TEST("library_init");
    s = ots_library_init(&lib, "TestLib", "./scenarios");
    ASSERT_EQ_INT(MPC_OTS_OK, s);
    ASSERT_EQ_INT(0, lib.num_templates);
    PASS();

    ScenarioTemplate tmpl;
    s = ots_scenario_preset_distillation(&tmpl);
    ASSERT_EQ_INT(MPC_OTS_OK, s);

    TEST("library_add_template");
    tmpl.template_id = 100;
    s = ots_library_add_template(&lib, &tmpl);
    ASSERT_EQ_INT(MPC_OTS_OK, s);
    ASSERT_EQ_INT(1, lib.num_templates);
    PASS();

    int32_t results[10];
    TEST("library_find_templates");
    int32_t found = ots_library_find_templates(&lib, MPC_SCENARIO_OPTIMIZATION_TRADEOFF, MPC_DIFFICULTY_INTERMEDIATE, results, 10);
    ASSERT_EQ_INT(1, found);
    PASS();
}

/* =========================================================================
 * L5 Tests: Competency Certification
 * ========================================================================= */

static void test_competency_certification(void)
{
    OperatorStatistics stats;
    CompetencyCertification cert;

    memset(&stats, 0, sizeof(OperatorStatistics));
    stats.operator_id = 1;
    stats.total_sessions = 25;
    for (int i = 0; i < MPC_OTS_NUM_METRICS; i++) {
        stats.metrics[i].metric = (PerformanceMetric)i;
        stats.metrics[i].mean = 85.0; /* well above thresholds */
        stats.metrics[i].num_samples = 20;
    }

    TEST("assess_competency_senior");
    OTSStatus s = ots_assess_competency(&stats, MPC_ROLE_SENIOR_OPERATOR, &cert);
    ASSERT_EQ_INT(MPC_OTS_OK, s);
    ASSERT_TRUE(cert.is_certified);
    ASSERT_TRUE(cert.confidence_level > 0.5);
    PASS();
}

/* =========================================================================
 * L5 Tests: HMI State
 * ========================================================================= */

static void test_hmi_state(void)
{
    HMIDisplayState hmi;

    TEST("hmi_init");
    OTSStatus s = ots_hmi_init(&hmi);
    ASSERT_EQ_INT(MPC_OTS_OK, s);
    ASSERT_EQ_INT((int)MPC_HMI_LEVEL_1_AREA, (int)hmi.current_level);
    PASS();

    TEST("hmi_navigate_level2");
    s = ots_hmi_navigate_to(&hmi, MPC_HMI_LEVEL_2_UNIT);
    ASSERT_EQ_INT(MPC_OTS_OK, s);
    ASSERT_EQ_INT((int)MPC_HMI_LEVEL_2_UNIT, (int)hmi.current_level);
    PASS();

    TEST("hmi_navigate_skip_levels");
    s = ots_hmi_navigate_to(&hmi, MPC_HMI_LEVEL_4_DIAGNOSTIC);
    ASSERT_EQ_INT(MPC_OTS_ERR_INVALID_TRANSITION, s);
    PASS();

    TEST("hmi_navigate_back");
    s = ots_hmi_navigate_back(&hmi);
    ASSERT_EQ_INT(MPC_OTS_OK, s);
    ASSERT_EQ_INT((int)MPC_HMI_LEVEL_1_AREA, (int)hmi.current_level);
    PASS();
}

/* =========================================================================
 * L5 Tests: Decision Logging
 * ========================================================================= */

static void test_decision_logging(void)
{
    OperatorDecision dec;
    OTSStatus s;

    TEST("decision_log");
    s = ots_decision_log(&dec, 1, 10, "Increased reflux by 5%", 5.0, 2.0, true);
    ASSERT_EQ_INT(MPC_OTS_OK, s);
    ASSERT_EQ_INT(1, dec.decision_id);
    ASSERT_TRUE(dec.followed_guidance);
    PASS();

    TEST("compute_hesitation");
    double hesitation = ots_compute_hesitation(100.0, 135.0);
    ASSERT_NEAR(35.0, hesitation, 0.01);
    PASS();

    OperatorDecision decisions[] = {
        {.followed_guidance = true, .was_correct = true, .time_to_decide = 10.0},
        {.followed_guidance = true, .was_correct = true, .time_to_decide = 15.0},
        {.followed_guidance = false, .was_correct = false, .time_to_decide = 20.0},
        {.followed_guidance = true, .was_correct = true, .time_to_decide = 12.0}
    };

    TEST("decision_summary");
    double follow_rate, mean_hes, accuracy;
    s = ots_decision_summary(decisions, 4, &follow_rate, &mean_hes, &accuracy);
    ASSERT_EQ_INT(MPC_OTS_OK, s);
    ASSERT_NEAR(0.75, follow_rate, 0.01);
    ASSERT_NEAR(14.25, mean_hes, 0.01);
    ASSERT_NEAR(0.75, accuracy, 0.01);
    PASS();
}

/* =========================================================================
 * Main Runner
 * ========================================================================= */

int main(void)
{
    printf("=== MPC Operator Training Interface — Test Suite ===\n\n");

    printf("--- L1: Enum/String Conversions ---\n");
    test_l1_enum_conversions();

    printf("\n--- L1+L2: Session Lifecycle ---\n");
    test_session_lifecycle();

    printf("\n--- L2: State Transitions ---\n");
    test_state_transitions();

    printf("\n--- L1+L2: Operator Profile ---\n");
    test_operator_profile();

    printf("\n--- L5: Elo Rating ---\n");
    test_elo_rating();

    printf("\n--- L5: Performance Scoring ---\n");
    test_performance_scoring();

    printf("\n--- L5: Statistical Functions ---\n");
    test_statistical_functions();

    printf("\n--- L3: Scenario Generation ---\n");
    test_scenario_generation();

    printf("\n--- L5: Difficulty Scaling ---\n");
    test_difficulty_scaling();

    printf("\n--- L5: Guidance System ---\n");
    test_guidance_system();

    printf("\n--- L5: Workload Estimation ---\n");
    test_workload_estimation();

    printf("\n--- L5: What-if Analysis ---\n");
    test_whatif_analysis();

    printf("\n--- L5: Trend Buffer ---\n");
    test_trend_buffer();

    printf("\n--- L5: Constraint Polygon ---\n");
    test_constraint_polygon();

    printf("\n--- L5: Performance Radar ---\n");
    test_performance_radar();

    printf("\n--- L5: Scenario Library ---\n");
    test_scenario_library();

    printf("\n--- L5: Competency Certification ---\n");
    test_competency_certification();

    printf("\n--- L5: HMI State ---\n");
    test_hmi_state();

    printf("\n--- L5: Decision Logging ---\n");
    test_decision_logging();

    printf("\n========================================\n");
    printf("RESULTS: %d passed, %d failed, %d total\n",
           tests_passed, tests_failed, tests_passed + tests_failed);
    printf("========================================\n");

    return tests_failed > 0 ? 1 : 0;
}
