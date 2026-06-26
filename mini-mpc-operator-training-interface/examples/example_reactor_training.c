/**
 * @file example_reactor_training.c
 * @brief End-to-end CSTR reactor operator training simulation.
 *
 * Demonstrates a complete operator training session for exothermic
 * CSTR reactor MPC operation. The scenario covers cooling failure
 * response, feed disturbance handling, and thermal runaway prevention.
 *
 * Knowledge points:
 *   L6: CSTR reactor operator training — canonical problem
 *   L5: Session lifecycle, scenario generation, scoring
 *   L2: Operator-in-the-loop MPC training
 *
 * Reference:
 *   Fogler (2016), "Elements of Chemical Reaction Engineering", 6th ed., Ch.12
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../include/mpc_ots_defs.h"
#include "../include/mpc_ots_scenario.h"
#include "../include/mpc_ots_assessment.h"

int main(void)
{
    printf("====================================================\n");
    printf("  CSTR Reactor MPC Operator Training Simulation\n");
    printf("  Exothermic Reaction: A -> B, ΔH = -200 kJ/mol\n");
    printf("====================================================\n\n");

    /* Step 1: Create operator profile (Trainee) */
    OperatorProfile operator;
    ots_profile_init(&operator, 1001, "Operator Alice", MPC_ROLE_TRAINEE);
    printf("[Profile] Operator: %s (ID: %d)\n", operator.name, operator.operator_id);
    printf("           Role: Trainee, Sessions: %d, Elo: %.0f\n\n",
           operator.sessions_completed, operator.current_elo_rating);

    /* Step 2: Create reactor training scenario */
    ScenarioTemplate reactor_scenario;
    ots_scenario_preset_reactor(&reactor_scenario);
    printf("[Scenario] Template: %s\n", reactor_scenario.name);
    printf("           Events: %d, Type: Disturbance Rejection\n", reactor_scenario.num_events);
    printf("           Fidelity: Medium (1st-principles dynamics)\n");
    printf("           Key MVs: Coolant flow, Feed temperature\n");
    printf("           Key CVs: Reactor temp, Conversion, Jacket temp\n\n");

    /* Step 3: Generate training timeline at BEGINNER difficulty */
    EventTimeline timeline;
    ots_scenario_generate_timeline(&reactor_scenario, MPC_DIFFICULTY_BEGINNER, &timeline);
    printf("[Timeline] Difficulty: %s\n", ots_difficulty_to_string(timeline.difficulty));
    printf("           Events: %d, Duration: %.0f seconds\n", timeline.num_events, timeline.total_duration);
    printf("           Expected Score: %.1f/100\n\n", timeline.expected_overall_score);

    /* Step 4: Create training session */
    TrainingSession session;
    ots_session_init(&session, 5001, "CSTR_Reactor_Training_001");
    session.operator_id = operator.operator_id;
    session.scenario_type = reactor_scenario.type;
    session.difficulty = timeline.difficulty;
    session.total_events = timeline.num_events;
    session.interface_mode = MPC_IF_MODE_ASSIST;
    session.state = MPC_OTS_STATE_READY;

    printf("[Session] ID: %d, Name: %s\n", session.session_id, session.session_name);
    printf("          State: %s\n\n", ots_state_to_string(session.state));

    /* Step 5: Start training session */
    ots_session_start(&session);
    printf("[Session] State: %s (STARTED)\n\n", ots_state_to_string(session.state));

    /* Step 6: Simulate operator responding to events */
    printf("=== Training Events ===\n");
    PerformanceRecord records[16];
    int32_t record_count = 0;

    for (int32_t i = 0; i < timeline.num_events; i++) {
        ScenarioEvent *event = &timeline.events[i];
        printf("Event %2d: %s at t=%.0f sec\n", i + 1, event->description, event->trigger_time);

        /* Simulate operator response (randomized for demo) */
        double response_time = event->optimal_response_time * (0.5 + ((double)(i * 17 + 31) / 100.0));
        if (response_time < 2.0) response_time = 2.0;
        double peak_dev = event->max_allowable_deviation * (0.3 + ((double)(i * 13 + 7) / 100.0));

        ots_event_evaluate_response(event, response_time, peak_dev, &records[record_count]);
        record_count++;

        printf("          Response: %.1f sec (optimal: %.1f), Deviation: %.2f\n",
               response_time, event->optimal_response_time, peak_dev);
        printf("          Score: %.1f/100\n",
               records[record_count - 1].normalized_score);
    }
    printf("\n");

    /* Step 7: Complete session and generate debrief */
    DebriefReport report;
    ots_session_complete(&session, &report);

    /* Compute overall scores */
    double metric_scores[7];
    double weights[] = {
        MPC_OTS_WEIGHT_RESPONSE_TIME,
        MPC_OTS_WEIGHT_STABILITY,
        MPC_OTS_WEIGHT_CONSTRAINTS,
        MPC_OTS_WEIGHT_ECONOMIC,
        MPC_OTS_WEIGHT_ALARM,
        MPC_OTS_WEIGHT_AWARENESS,
        MPC_OTS_WEIGHT_CONSISTENCY
    };

    for (int i = 0; i < 7; i++) {
        double sum = 0.0;
        int cnt = 0;
        for (int j = 0; j < record_count; j++) {
            if ((int)records[j].metric_type == i) { sum += records[j].normalized_score; cnt++; }
        }
        metric_scores[i] = (cnt > 0) ? sum / (double)cnt : 50.0;
    }

    double overall = ots_score_weighted_overall(metric_scores, weights, 7);

    report.session_id = session.session_id;
    report.overall_score = overall;
    memcpy(report.scores_by_metric, metric_scores, sizeof(metric_scores));

    /* Step 8: Update operator profile */
    ots_profile_update_after_session(&operator, &report);

    double new_elo = ots_elo_update(operator.current_elo_rating, overall, timeline.expected_overall_score, 32.0);
    operator.current_elo_rating = new_elo;
    operator.recommended_difficulty = ots_recommend_difficulty(&operator);

    printf("====================================================\n");
    printf("  TRAINING SESSION RESULTS\n");
    printf("====================================================\n");
    printf("Session: %s (#%d)\n", session.session_name, session.session_id);
    printf("State: %s\n", ots_state_to_string(session.state));
    printf("Events Completed: %d/%d\n", timeline.num_events, timeline.num_events);
    printf("\n--- Performance Scores ---\n");
    printf("  Response Time:     %.1f/100\n", metric_scores[0]);
    printf("  Stability:         %.1f/100\n", metric_scores[1]);
    printf("  Constraints:       %.1f/100\n", metric_scores[2]);
    printf("  Economic:          %.1f/100\n", metric_scores[3]);
    printf("  Alarm Management:  %.1f/100\n", metric_scores[4]);
    printf("  Situation Aware:   %.1f/100\n", metric_scores[5]);
    printf("  Consistency:       %.1f/100\n", metric_scores[6]);
    printf("  ---\n");
    printf("  OVERALL SCORE:     %.1f/100\n", overall);
    printf("\n--- Operator Progress ---\n");
    printf("  Sessions: %d (+1)\n", operator.sessions_completed);
    printf("  Elo Rating: %.0f\n", operator.current_elo_rating);
    printf("  Next Difficulty: %s\n", ots_difficulty_to_string(operator.recommended_difficulty));
    printf("====================================================\n");

    return 0;
}
