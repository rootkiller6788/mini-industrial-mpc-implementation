/**
 * @file example_whatif_analysis.c
 * @brief Operator what-if analysis and decision support demonstration.
 *
 * Demonstrates the MPC operator training interface's what-if analysis
 * capability. An operator explores alternative MV adjustments before
 * committing to an action, receiving safety assessments and economic
 * impact predictions.
 *
 * Knowledge points:
 *   L6: What-if analysis for operator decision support — canonical problem
 *   L5: What-if prediction, economic impact, safety assessment
 *   L2: Operator-in-the-loop decision making
 *   L7: Honeywell Profit Suite what-if, AspenTech DMC3 what-if mode
 *
 * Reference:
 *   Honeywell (2020), "Profit Suite — What-if Analysis"
 *   Seborg et al. (2016), "Process Dynamics and Control", 4th ed., Ch.16
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../include/mpc_ots_defs.h"
#include "../include/mpc_ots_assessment.h"
#include "../include/mpc_ots_guidance.h"

int main(void)
{
    printf("============================================================\n");
    printf("  MPC Operator What-If Analysis Demo\n");
    printf("  FCC Unit — Exploring MV Adjustments Safely\n");
    printf("============================================================\n\n");

    /* Setup decision context for FCC unit */
    DecisionContext ctx;
    memset(&ctx, 0, sizeof(DecisionContext));
    ctx.num_variables = 6;
    ctx.difficulty = MPC_DIFFICULTY_ADVANCED;
    ctx.operator_level = MPC_ROLE_SENIOR_OPERATOR;
    ctx.active_event_count = 2;
    ctx.elapsed_scenario_time = 300.0;
    ctx.current_score = 75.0;

    /* FCC operating point (normalized) */
    char cv_names[][32] = {"Riser T", "Regen T", "Conversion", "WGC Suction", "LPG Yield", "Gasoline"};
    for (int32_t i = 0; i < 6; i++) {
        ctx.current_values[i] = 50.0 + (double)i * 8.0;
        ctx.lower_limits[i] = 20.0;
        ctx.upper_limits[i] = 80.0 + (double)i * 5.0;
        ctx.optimal_targets[i] = 55.0 + (double)i * 10.0;
        ctx.constraint_margins[i] = 10.0 - (double)i * 1.5;
    }

    printf("[FCC Unit State]\n");
    printf("  Variable       Current  Lower   Upper   Optimum  Margin\n");
    printf("  ------------   -------  -----   -----   -------  ------\n");
    for (int32_t i = 0; i < ctx.num_variables; i++) {
        printf("  %-12s  %6.1f  %6.1f  %6.1f  %6.1f  %6.1f\n",
               cv_names[i], ctx.current_values[i], ctx.lower_limits[i],
               ctx.upper_limits[i], ctx.optimal_targets[i], ctx.constraint_margins[i]);
    }
    printf("\n");

    /* Workload estimation */
    WorkloadEstimate wl;
    ots_workload_estimate(&ctx, &wl);
    printf("[Workload] Mental: %.2f, Temporal: %.2f, Overall: %.2f (%s)\n\n",
           wl.mental_demand, wl.temporal_demand, wl.overall_workload,
           wl.is_overloaded ? "OVERLOADED" : "Normal");

    /* Generate guidance */
    GuidanceEntry guidance;
    ots_guidance_generate(&ctx, &wl, &guidance);
    printf("[Guidance] Level: %d, Priority: %d\n", (int)guidance.level, guidance.priority);
    printf("           Title: %s\n", guidance.title);
    printf("           Message: %s\n", guidance.message);
    printf("           Action: %s\n", guidance.action);
    printf("           Delay before display: %.0f sec\n\n", guidance.delay_before_display);

    /* ─── What-If Analysis #1: Conservative Adjustment ─── */
    printf("============================================================\n");
    printf("  WHAT-IF #1: Conservative Riser Temperature Increase (+5)\n");
    printf("============================================================\n");

    WhatIfQuery query1;
    memset(&query1, 0, sizeof(WhatIfQuery));
    query1.query_id = 1;
    query1.num_variables = ctx.num_variables;
    for (int32_t i = 0; i < ctx.num_variables; i++) {
        query1.current_values[i] = ctx.current_values[i];
        query1.proposed_values[i] = ctx.current_values[i];
        snprintf(query1.variable_names[i], MPC_OTS_NAME_MAX, "%s", cv_names[i]);
    }
    query1.proposed_values[0] = ctx.current_values[0] + 5.0; /* +5 on riser T */

    ots_whatif_execute(&query1);

    printf("  Proposed Action: Increase Riser Temperature by +5 units\n");
    printf("  Predicted CV Deviations:\n");
    for (int32_t i = 0; i < ctx.num_variables; i++) {
        printf("    %s: Δ = %+.2f\n", cv_names[i], query1.predicted_cv_deviations[i]);
    }
    printf("  Economic Impact: $%.1f/hr\n", query1.predicted_economic_impact);
    printf("  Constraint Violations: %.0f\n", query1.predicted_constraint_violations);
    printf("  Confidence: %.0f%%\n", query1.confidence_level * 100.0);
    printf("  SAFETY: %s\n", query1.is_safe ? "✅ SAFE" : "❌ UNSAFE");
    printf("  Recommendation: %s\n\n", query1.recommendation);

    /* ─── What-If Analysis #2: Aggressive Adjustment ─── */
    printf("============================================================\n");
    printf("  WHAT-IF #2: Aggressive Multi-Variable Push (+20 on all)\n");
    printf("============================================================\n");

    WhatIfQuery query2;
    memset(&query2, 0, sizeof(WhatIfQuery));
    query2.query_id = 2;
    query2.num_variables = ctx.num_variables;
    for (int32_t i = 0; i < ctx.num_variables; i++) {
        query2.current_values[i] = ctx.current_values[i];
        query2.proposed_values[i] = ctx.current_values[i] + 20.0;
        snprintf(query2.variable_names[i], MPC_OTS_NAME_MAX, "%s", cv_names[i]);
    }

    ots_whatif_execute(&query2);

    printf("  Proposed Action: +20 across all 6 variables\n");
    printf("  Predicted CV Deviations:\n");
    for (int32_t i = 0; i < ctx.num_variables; i++) {
        printf("    %s: Δ = %+.2f\n", cv_names[i], query2.predicted_cv_deviations[i]);
    }
    printf("  Economic Impact: $%.1f/hr\n", query2.predicted_economic_impact);
    printf("  Constraint Violations: %.0f\n", query2.predicted_constraint_violations);
    printf("  Confidence: %.0f%%\n", query2.confidence_level * 100.0);
    printf("  SAFETY: %s\n", query2.is_safe ? "✅ SAFE" : "❌ UNSAFE");
    printf("  Recommendation: %s\n\n", query2.recommendation);

    /* ─── What-If Analysis #3: Economic Optimization ─── */
    printf("============================================================\n");
    printf("  WHAT-IF #3: Economic Optimization Push\n");
    printf("============================================================\n");

    WhatIfQuery query3;
    memset(&query3, 0, sizeof(WhatIfQuery));
    query3.query_id = 3;
    query3.num_variables = ctx.num_variables;
    for (int32_t i = 0; i < ctx.num_variables; i++) {
        query3.current_values[i] = ctx.current_values[i];
        /* Move each CV halfway toward its economic optimum */
        query3.proposed_values[i] = ctx.current_values[i]
            + (ctx.optimal_targets[i] - ctx.current_values[i]) * 0.5;
        snprintf(query3.variable_names[i], MPC_OTS_NAME_MAX, "%s", cv_names[i]);
    }

    ots_whatif_execute(&query3);

    printf("  Proposed Action: Move 50%% toward economic optimum\n");
    printf("  Predicted CV Deviations:\n");
    for (int32_t i = 0; i < ctx.num_variables; i++) {
        printf("    %s: Δ = %+.2f (target at %.1f)\n",
               cv_names[i], query3.predicted_cv_deviations[i], ctx.optimal_targets[i]);
    }
    printf("  Economic Impact: $%.1f/hr\n", query3.predicted_economic_impact);
    printf("  Confidence: %.0f%%\n", query3.confidence_level * 100.0);
    printf("  SAFETY: %s\n", query3.is_safe ? "✅ SAFE" : "❌ UNSAFE");
    printf("  Recommendation: %s\n\n", query3.recommendation);

    /* ─── Decision Logging ─── */
    OperatorDecision dec;
    ots_decision_log(&dec, 1, 100, "Applied conservative riser T +5 adjustment", 5.0, query1.predicted_economic_impact, true);
    dec.time_to_decide = 12.5;
    dec.was_correct = true;
    dec.actual_outcome = query1.predicted_economic_impact * 0.9; /* slightly better */

    printf("============================================================\n");
    printf("  DECISION LOGGED\n");
    printf("============================================================\n");
    printf("  Action: %s\n", dec.action_description);
    printf("  Decision Time: %.1f sec\n", dec.time_to_decide);
    printf("  Followed Guidance: %s\n", dec.followed_guidance ? "Yes" : "No");
    printf("  Predicted Outcome: $%.1f/hr\n", dec.predicted_outcome);
    printf("  Actual Outcome:    $%.1f/hr\n", dec.actual_outcome);
    printf("  Prediction Error:  $%.1f/hr (%.1f%%)\n",
           dec.prediction_error, fabs(dec.prediction_error / dec.predicted_outcome) * 100.0);
    printf("============================================================\n");

    /* Summary */
    printf("\n=== Analysis Summary ===\n");
    printf("What-If #1 (Conservative): %s — Recommended for execution\n",
           ots_whatif_is_safe(&query1) ? "SAFE" : "UNSAFE");
    printf("What-If #2 (Aggressive):  %s — Reject, excessive violations\n",
           ots_whatif_is_safe(&query2) ? "SAFE" : "UNSAFE");
    printf("What-If #3 (Optimize):    %s — Optimal balance of safety & economics\n",
           ots_whatif_is_safe(&query3) ? "SAFE" : "UNSAFE");
    printf("Operator chose: What-If #1 (safe, conservative approach)\n");

    return 0;
}
