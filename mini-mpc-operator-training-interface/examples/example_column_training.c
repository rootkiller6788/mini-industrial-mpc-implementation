/**
 * @file example_column_training.c
 * @brief Distillation column MPC operator training — multi-variable optimization.
 *
 * Demonstrates a complete operator training session for binary distillation
 * column MPC operation. Covers feed composition upset, reboiler duty constraint,
 * product purity trade-off, and tray flooding approach.
 *
 * Knowledge points:
 *   L6: Distillation column MPC operation — canonical problem
 *   L5: Multi-metric scoring, what-if analysis
 *   L2: Constraint trade-off in multi-variable MPC
 *   L7: Honeywell UniSim distillation training module
 *
 * Reference:
 *   Luyben (2013), "Distillation Design and Control Using Aspen Simulation", 2nd ed., Ch.15
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../include/mpc_ots_defs.h"
#include "../include/mpc_ots_scenario.h"
#include "../include/mpc_ots_assessment.h"
#include "../include/mpc_ots_interface.h"

int main(void)
{
    printf("============================================================\n");
    printf("  Distillation Column MPC — Operator Training Session\n");
    printf("  Binary Separation: Benzene/Toluene, 30 Trays\n");
    printf("  Specs: Top Purity 99.5%%, Bottoms Recovery 98%%\n");
    printf("============================================================\n\n");

    /* Operator: experienced panel operator upgrading to MPC */
    OperatorProfile operator;
    ots_profile_init(&operator, 2002, "Bob Chen", MPC_ROLE_SENIOR_OPERATOR);
    operator.sessions_completed = 20;
    operator.current_elo_rating = 1350.0;
    operator.recommended_difficulty = MPC_DIFFICULTY_INTERMEDIATE;
    printf("[Profile] Operator: %s\n", operator.name);
    printf("          Sessions: %d, Elo: %.0f, Level: %s\n\n",
           operator.sessions_completed, operator.current_elo_rating,
           ots_difficulty_to_string(operator.recommended_difficulty));

    /* Create distillation scenario at INTERMEDIATE difficulty */
    ScenarioTemplate column_scenario;
    ots_scenario_preset_distillation(&column_scenario);
    printf("[Scenario] %s\n", column_scenario.name);
    printf("           Events: %d, Fidelity: High\n", column_scenario.num_events);
    printf("           MVs: Reflux ratio, Reboiler steam, Distillate rate\n");
    printf("           CVs: Top comp, Bottom comp, Column ΔP, Tray temps\n\n");

    /* Generate timeline */
    EventTimeline timeline;
    ots_scenario_generate_timeline(&column_scenario, MPC_DIFFICULTY_INTERMEDIATE, &timeline);
    printf("[Timeline] Difficulty: Intermediate\n");
    printf("           Duration: %.0f sec, Expected Score: %.1f\n\n",
           timeline.total_duration, timeline.expected_overall_score);

    /* Session setup */
    TrainingSession session;
    ots_session_init(&session, 6001, "Distillation_Column_MPC_Training");
    session.operator_id = operator.operator_id;
    session.scenario_type = column_scenario.type;
    session.difficulty = MPC_DIFFICULTY_INTERMEDIATE;
    session.total_events = timeline.num_events;
    session.interface_mode = MPC_IF_MODE_GUIDE;
    session.state = MPC_OTS_STATE_READY;

    ots_session_start(&session);

    /* Simulate training with constraint visualization */
    printf("=== Training Session: Distillation Column MPC ===\n");

    ConstraintPolygon poly;
    ots_constraint_polygon_init(&poly, 4);
    /* Top purity CV */
    snprintf(poly.variable_names[0], MPC_OTS_NAME_MAX, "Top Purity %%");
    poly.lower_limits[0] = 98.0; poly.upper_limits[0] = 100.0; poly.target_values[0] = 99.5;
    /* Bottoms recovery */
    snprintf(poly.variable_names[1], MPC_OTS_NAME_MAX, "Bottoms Rec %%");
    poly.lower_limits[1] = 95.0; poly.upper_limits[1] = 100.0; poly.target_values[1] = 98.0;
    /* Column ΔP (mbar) */
    snprintf(poly.variable_names[2], MPC_OTS_NAME_MAX, "Column ΔP mbar");
    poly.lower_limits[2] = 20.0; poly.upper_limits[2] = 150.0; poly.target_values[2] = 100.0;
    /* Reboiler duty (MW) */
    snprintf(poly.variable_names[3], MPC_OTS_NAME_MAX, "Reboiler MW");
    poly.lower_limits[3] = 1.0; poly.upper_limits[3] = 5.0; poly.target_values[3] = 3.5;

    double cv_values[4];
    double total_score = 0.0;
    int scored_events = 0;

    for (int32_t i = 0; i < timeline.num_events; i++) {
        ScenarioEvent *evt = &timeline.events[i];
        printf("\n  Event %d: %s\n", i + 1, evt->description);

        /* Simulated process response */
        double resp_actual = evt->optimal_response_time * (0.6 + (double)((i * 11 + 23) % 100) / 100.0);
        double dev_actual = evt->max_allowable_deviation * (0.2 + (double)((i * 7 + 19) % 100) / 200.0);

        PerformanceRecord rec;
        ots_event_evaluate_response(evt, resp_actual, dev_actual, &rec);
        total_score += rec.normalized_score;
        scored_events++;

        /* Update constraint polygon */
        cv_values[0] = 99.5 - dev_actual * 0.05;
        cv_values[1] = 98.0 - dev_actual * 0.03;
        cv_values[2] = 100.0 + dev_actual * 2.0;
        cv_values[3] = 3.5 + dev_actual * 0.1;
        ots_constraint_polygon_update(&poly, cv_values);

        printf("    Response: %.1fs (opt %.1fs) Score: %.1f\n",
               resp_actual, evt->optimal_response_time, rec.normalized_score);
        printf("    Min Constraint Margin: %.2f (var: %s)\n",
               poly.min_margin, poly.variable_names[poly.most_critical_axis]);
        if (poly.is_infeasible) printf("    ⚠ WARNING: Operating point INFEASIBLE!\n");
    }

    double avg_score = (scored_events > 0) ? total_score / (double)scored_events : 0.0;

    /* Session complete */
    DebriefReport report;
    ots_session_complete(&session, &report);
    report.overall_score = avg_score;

    /* Update profile with new Elo */
    ots_profile_update_after_session(&operator, &report);
    operator.current_elo_rating = ots_elo_update(
        operator.current_elo_rating, avg_score, timeline.expected_overall_score, 24.0);
    operator.recommended_difficulty = ots_recommend_difficulty(&operator);

    printf("\n============================================================\n");
    printf("  DISTILLATION TRAINING RESULTS\n");
    printf("============================================================\n");
    printf("  Average Event Score: %.1f/100\n", avg_score);
    printf("  Expected Score:      %.1f/100\n", timeline.expected_overall_score);
    printf("  Events Handled:      %d/%d\n", scored_events, timeline.num_events);
    printf("  Final Constraint Margin: %.2f\n", poly.min_margin);
    printf("  Operator Elo:        %.0f → %.0f\n",
           operator.current_elo_rating - ots_elo_update(0, 0, 0, 0) + 1350.0,
           operator.current_elo_rating);
    printf("  Recommended Next:    %s\n", ots_difficulty_to_string(operator.recommended_difficulty));
    printf("============================================================\n");

    return 0;
}
