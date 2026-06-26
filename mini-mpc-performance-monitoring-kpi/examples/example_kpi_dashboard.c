/**
 * @file example_kpi_dashboard.c
 * @brief Multi-KPI dashboard for MPC performance at a petrochemical plant.
 *
 * L7 industrial application: Complete KPI monitoring dashboard simulating
 * AspenTech AspenWatch-style performance tracking across availability,
 * performance, quality, economic, and constraint categories.
 *
 * Demonstrates: multi-category dashboard aggregation, vendor-specific KPIs
 * (AspenWatch, Profit Sensor, Yokogawa MD), ISO 50001 energy monitoring,
 * CO2 emission tracking, and autonomous health assessment.
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "../include/mpc_kpi_defs.h"
#include "../include/mpc_kpi_metrics.h"
#include "../include/mpc_kpi_monitoring.h"
#include "../include/mpc_kpi_diagnosis.h"

static void print_category(const char *cat_name, double score)
{
    printf("  %-20s: %.3f [%s]\n", cat_name, score, mpc_kpi_health_score_to_tier(score)<=MPC_KPI_TIER_FAIR?"FAIR":"GOOD+");
}

int main(void)
{
    printf("=== MPC KPI Dashboard: Petrochemical Plant ===\n\n");

    mpc_kpi_dashboard_t db;
    mpc_kpi_dashboard_init(&db, 20);

    /* Register 12 KPIs across all categories */
    mpc_kpi_dashboard_register_kpi(&db, MPC_KPI_CONTROLLER_UTILIZATION, MPC_KPI_CAT_AVAILABILITY);
    mpc_kpi_dashboard_register_kpi(&db, MPC_KPI_HARRIS_INDEX, MPC_KPI_CAT_PERFORMANCE);
    mpc_kpi_dashboard_register_kpi(&db, MPC_KPI_SETPOINT_TRACKING_RMSE, MPC_KPI_CAT_PERFORMANCE);
    mpc_kpi_dashboard_register_kpi(&db, MPC_KPI_VARIANCE_REDUCTION, MPC_KPI_CAT_PERFORMANCE);
    mpc_kpi_dashboard_register_kpi(&db, MPC_KPI_CONSTRAINT_SATISFACTION, MPC_KPI_CAT_CONSTRAINT);
    mpc_kpi_dashboard_register_kpi(&db, MPC_KPI_MV_SATURATION_FRACTION, MPC_KPI_CAT_CONSTRAINT);
    mpc_kpi_dashboard_register_kpi(&db, MPC_KPI_ECONOMIC_BENEFIT, MPC_KPI_CAT_ECONOMIC);
    mpc_kpi_dashboard_register_kpi(&db, MPC_KPI_ENERGY_SAVINGS, MPC_KPI_CAT_ECONOMIC);
    mpc_kpi_dashboard_register_kpi(&db, MPC_KPI_MODEL_PLANT_MISMATCH, MPC_KPI_CAT_DIAGNOSTIC);
    mpc_kpi_dashboard_register_kpi(&db, MPC_KPI_QP_SOLVE_TIME, MPC_KPI_CAT_DIAGNOSTIC);
    mpc_kpi_dashboard_register_kpi(&db, MPC_KPI_YIELD_IMPROVEMENT, MPC_KPI_CAT_QUALITY);
    mpc_kpi_dashboard_register_kpi(&db, MPC_KPI_PREDICTION_ERROR_RMS, MPC_KPI_CAT_QUALITY);

    /* Initialize baselines */
    for(int i=0;i<db.num_kpi;i++){
        db.kpi_values[i].target_value=0.85;
        db.kpi_values[i].baseline_mean=0.80;
        db.kpi_values[i].baseline_std=0.12;
    }

    /* Simulate 8 monitoring cycles with different plant conditions */
    double kpi_profiles[8][12]={
        {0.95,0.82,0.15,0.60,0.98,0.05,0.12,0.08,0.03,25.0,0.04,0.02},  /* normal */
        {0.93,0.80,0.16,0.58,0.97,0.06,0.13,0.09,0.04,28.0,0.05,0.03},  /* normal */
        {0.88,0.75,0.22,0.55,0.95,0.08,0.10,0.07,0.05,32.0,0.06,0.04},  /* slight degrade */
        {0.82,0.68,0.30,0.48,0.90,0.12,0.09,0.06,0.08,40.0,0.07,0.06},  /* degrade */
        {0.75,0.60,0.40,0.40,0.85,0.18,0.07,0.05,0.12,55.0,0.09,0.08},  /* severe */
        {0.85,0.72,0.25,0.50,0.92,0.10,0.11,0.07,0.06,35.0,0.06,0.05},  /* recovery start */
        {0.90,0.78,0.18,0.56,0.95,0.07,0.12,0.08,0.04,28.0,0.05,0.03},  /* recovery */
        {0.94,0.81,0.15,0.59,0.97,0.05,0.12,0.08,0.03,24.0,0.04,0.02},  /* normal */
    };
    const char *phase_names[]={"Normal","Normal","Slight Degrade","Degrade","Severe",
                                "Recovery Start","Recovery","Normal"};

    for(int cycle=0;cycle<8;cycle++){
        printf("--- Cycle %d [%s] ---\n", cycle*50, phase_names[cycle]);
        for(int i=0;i<db.num_kpi;i++){
            double noise=0.02*((double)rand()/RAND_MAX-0.5);
            mpc_kpi_dashboard_update(&db, db.kpi_values[i].kpi_id,
                                       kpi_profiles[cycle][i]+noise, (uint64_t)(cycle*50));
        }
        mpc_kpi_dashboard_compute_health(&db);

        printf("  Category scores:\n");
        print_category("Availability", db.availability_score);
        print_category("Performance", db.performance_score);
        print_category("Quality", db.quality_score);
        print_category("Economic", db.economic_score);
        print_category("Constraint", db.constraint_score);
        printf("  ---> OVERALL: %.3f (%s) | Alarming: %d | Critical: %d\n\n",
               db.overall_health_score, mpc_kpi_tier_string(db.overall_tier),
               db.num_alarming, db.num_in_critical);
    }

    /* Generate comprehensive final report */
    printf("=== Final Report ===\n");
    mpc_kpi_report_t rpt;
    mpc_kpi_report_init(&rpt, db.num_kpi);
    mpc_kpi_report_snapshot(&rpt, &db, 400);
    printf("  Cycle: %llu\n", (unsigned long long)rpt.cycle);
    printf("  Overall Health: %.3f (%s)\n", rpt.overall_health, mpc_kpi_tier_string(rpt.tier));

    /* Vendor KPIs */
    double health_idx, service_factor;
    mpc_kpi_aspentech_aspenwatch_kpi(&db, &health_idx, &service_factor);
    printf("  AspenWatch: health=%.3f service=%.3f\n", health_idx, service_factor);

    double profit_impact, model_quality;
    double pred_errs[50], mv_costs_data[3]={2.5, 1.8, 3.2};
    for(int i=0;i<50;i++) pred_errs[i]=0.01*((double)rand()/RAND_MAX-0.5);
    mpc_kpi_honeywell_profit_sensor(pred_errs, 50, mv_costs_data, 3, &profit_impact, &model_quality);
    printf("  Profit Sensor: impact=%.3f model_quality=%.3f\n", profit_impact, model_quality);

    /* ISO 50001 & Environmental */
    double enpi, imp_pct;
    mpc_kpi_iso50001_enpi(1200.0, 1050.0, 100.0, 95.0, &enpi, &imp_pct);
    printf("  ISO 50001 EnPI: %.3f (%.1f%% improvement)\n", enpi, imp_pct);

    double co2 = mpc_kpi_co2_reduction_estimate(150.0, 0.48);
    printf("  CO2 Reduction: %.1f kg CO2\n", co2);

    double eii = mpc_kpi_energy_intensity_index(1050.0, 95.0, 12.0);
    printf("  Energy Intensity Index: %.3f\n", eii);

    /* Autonomous health (L9) */
    double auton_readiness, intervention_freq;
    mpc_kpi_autonomous_health_assessment(&db, &auton_readiness, &intervention_freq);
    printf("  Autonomy Readiness: %.3f (interv. freq: %.3f)\n", auton_readiness, intervention_freq);

    /* Pareto analysis of economic vs quality trade-off */
    double econ_kpis[5]={0.12,0.10,0.13,0.09,0.11};
    double qual_kpis[5]={0.04,0.05,0.03,0.06,0.04};
    int pareto_front[5], num_pareto;
    mpc_kpi_pareto_analysis(econ_kpis, qual_kpis, 5, pareto_front, &num_pareto, 5);
    printf("  Pareto Front: %d non-dominated solutions\n", num_pareto);

    /* IT/OT convergence readiness */
    double data_int, cyber, cloud;
    mpc_kpi_it_ot_convergence_readiness(&db, &data_int, &cyber, &cloud);
    printf("  IT/OT Convergence: data=%.2f cyber=%.2f cloud=%.2f\n", data_int, cyber, cloud);

    mpc_kpi_report_free(&rpt);
    mpc_kpi_dashboard_free(&db);
    printf("\nDashboard example complete.\n");
    return 0;
}
