/**
 * @file kpi_dashboard_demo.c
 * @brief Interactive MPC KPI monitoring dashboard demonstration.
 *
 * Simulates a real-time petrochemical plant MPC controller across multiple
 * operating scenarios: normal, degradation, disturbance, load change, recovery.
 *
 * Knowledge: L6 canonical petrochemical problem, L7 industrial dashboard,
 * L8 time-varying KPI with adaptive monitoring.
 * Reference: Qin & Badgwell (2003) Control Engineering Practice.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../include/mpc_kpi_defs.h"
#include "../include/mpc_kpi_metrics.h"
#include "../include/mpc_kpi_monitoring.h"
#include "../include/mpc_kpi_diagnosis.h"

typedef struct {
    double cv[2], mv[2], sp[2], mv_lo[2], mv_hi[2];
    double gain[2][2], tau[2][2];
    double disturbance;
} plant_sim_t;

static void plant_init(plant_sim_t *p)
{
    p->cv[0]=100.0; p->cv[1]=50.0;
    p->sp[0]=100.0; p->sp[1]=50.0;
    p->mv[0]=60.0; p->mv[1]=40.0;
    p->mv_lo[0]=0.0; p->mv_hi[0]=100.0;
    p->mv_lo[1]=0.0; p->mv_hi[1]=100.0;
    p->gain[0][0]=1.2; p->gain[0][1]=-0.3;
    p->gain[1][0]=0.4; p->gain[1][1]=0.8;
    p->tau[0][0]=5.0; p->tau[0][1]=8.0;
    p->tau[1][0]=6.0; p->tau[1][1]=4.0;
    p->disturbance=0.0;
}

static void plant_step(plant_sim_t *p)
{
    double cv_new[2]={0,0};
    for(int i=0;i<2;i++){
        double sum=0;
        for(int j=0;j<2;j++){
            double alpha=1.0-exp(-1.0/p->tau[i][j]);
            sum+=p->gain[i][j]*alpha*p->mv[j]+(1.0-alpha)*p->cv[i];
        }
        cv_new[i]=sum+p->disturbance+0.02*((double)rand()/RAND_MAX-0.5);
    }
    p->cv[0]=cv_new[0]; p->cv[1]=cv_new[1];
}

typedef enum { SC_NORMAL, SC_DEGRADE, SC_DISTURBANCE,
               SC_LOAD_CHANGE, SC_RECOVERY } scenario_t;

static const char *scenario_name(scenario_t s)
{
    switch(s){
    case SC_NORMAL:      return "NORMAL OPERATION";
    case SC_DEGRADE:     return "GRADUAL DEGRADATION";
    case SC_DISTURBANCE: return "FEED DISTURBANCE";
    case SC_LOAD_CHANGE: return "LOAD CHANGE";
    case SC_RECOVERY:    return "RECOVERY";
    default: return "UNKNOWN";
    }
}

static void apply_scenario(scenario_t sc, plant_sim_t *p, int step_in_scenario)
{
    switch(sc){
    case SC_NORMAL:
        p->disturbance=0.02*((double)rand()/RAND_MAX-0.5);
        break;
    case SC_DEGRADE:
        p->disturbance=0.02*((double)rand()/RAND_MAX-0.5)+0.001*(double)step_in_scenario;
        p->tau[0][0]=5.0+0.02*(double)step_in_scenario;
        break;
    case SC_DISTURBANCE:
        p->disturbance=0.5*(1.0+sin((double)step_in_scenario*0.3));
        break;
    case SC_LOAD_CHANGE:
        p->sp[0]=120.0; p->sp[1]=55.0;
        p->disturbance=0.02*((double)rand()/RAND_MAX-0.5);
        break;
    case SC_RECOVERY:
        p->tau[0][0]=5.0;
        p->disturbance=0.015*((double)rand()/RAND_MAX-0.5);
        break;
    }
}

static void print_banner(uint64_t cycle, scenario_t sc)
{
    printf("\n==========================================================\n");
    printf("  MPC KPI Dashboard | Cycle: %6llu | %s\n",
           (unsigned long long)cycle, scenario_name(sc));
    printf("==========================================================\n");
}

static void print_kpi(const char *name, double value, mpc_kpi_tier_t tier)
{
    printf("  %-30s %8.3f [%s]\n", name, value, mpc_kpi_tier_string(tier));
}

static void print_health(double score)
{
    printf("  --- Overall Health: %.1f%% (%s) ---\n",
           score*100.0, mpc_kpi_health_score_to_tier(score)<=MPC_KPI_TIER_FAIR?"OK":"WARNING");
}

int main(void)
{
    printf("MPC KPI Dashboard Demo - Petrochemical Distillation Unit\n\n");

    plant_sim_t plant; plant_init(&plant);

    mpc_kpi_dashboard_t db;
    mpc_kpi_dashboard_init(&db, 12);
    mpc_kpi_dashboard_register_kpi(&db, MPC_KPI_CONTROLLER_UTILIZATION, MPC_KPI_CAT_AVAILABILITY);
    mpc_kpi_dashboard_register_kpi(&db, MPC_KPI_HARRIS_INDEX, MPC_KPI_CAT_PERFORMANCE);
    mpc_kpi_dashboard_register_kpi(&db, MPC_KPI_SETPOINT_TRACKING_RMSE, MPC_KPI_CAT_PERFORMANCE);
    mpc_kpi_dashboard_register_kpi(&db, MPC_KPI_CONSTRAINT_SATISFACTION, MPC_KPI_CAT_CONSTRAINT);
    mpc_kpi_dashboard_register_kpi(&db, MPC_KPI_MV_SATURATION_FRACTION, MPC_KPI_CAT_CONSTRAINT);
    mpc_kpi_dashboard_register_kpi(&db, MPC_KPI_ECONOMIC_BENEFIT, MPC_KPI_CAT_ECONOMIC);
    mpc_kpi_dashboard_register_kpi(&db, MPC_KPI_MODEL_PLANT_MISMATCH, MPC_KPI_CAT_DIAGNOSTIC);
    mpc_kpi_dashboard_register_kpi(&db, MPC_KPI_PREDICTION_ERROR_RMS, MPC_KPI_CAT_QUALITY);

    for(int i=0;i<db.num_kpi;i++){
        db.kpi_values[i].target_value=0.85;
        db.kpi_values[i].baseline_mean=0.80;
        db.kpi_values[i].baseline_std=0.12;
    }

    /* Baselining phase with 120 samples */
    double baseline_buf[120];
    for(int i=0;i<120;i++){ plant_step(&plant); baseline_buf[i]=plant.cv[0]; }
    mpc_kpi_baseline_t bl;
    mpc_kpi_baseline_estimate(baseline_buf,120,1.5,&bl);
    printf("Baselining: mean=%.2f std=%.3f mad=%.3f samples=%d\n",
           bl.baseline_mean, bl.baseline_std, bl.baseline_mad, bl.num_samples_used);

    /* Oscillation check on baseline data */
    double osc_per, osc_amp, osc_reg; bool is_osc;
    mpc_kpi_diagnose_oscillation(baseline_buf, 120, &osc_per, &osc_amp, &osc_reg, &is_osc);
    printf("Oscillation check: period=%.1f amp=%.3f regularity=%.3f oscillating=%s\n",
           osc_per, osc_amp, osc_reg, is_osc?"YES":"no");

    /* Normality test */
    double w_stat, w_pv; bool is_norm;
    mpc_kpi_normality_test(baseline_buf, 120, &w_stat, &w_pv, &is_norm);
    printf("Normality: W=%.3f p=%.4f normal=%s\n", w_stat, w_pv, is_norm?"YES":"no");

    /* CUSUM monitoring setup */
    mpc_kpi_cusum_t cusum;
    mpc_kpi_cusum_init(&cusum, bl.baseline_mean, bl.baseline_std, 5.0, 0.5);

    scenario_t scenarios[]={SC_NORMAL, SC_DEGRADE, SC_DISTURBANCE, SC_LOAD_CHANGE, SC_RECOVERY};
    uint64_t cycle=120;

    for(int sc=0;sc<5;sc++){
        printf("\n>>> Scenario: %s <<<\n", scenario_name(scenarios[sc]));
        for(int step_in_sc=0;step_in_sc<40;step_in_sc++,cycle++){
            apply_scenario(scenarios[sc], &plant, step_in_sc);
            plant_step(&plant);

            /* CUSUM monitoring of CV1 */
            mpc_kpi_cusum_update(&cusum, plant.cv[0]);

            double track_err=fabs(plant.cv[0]-plant.sp[0])+fabs(plant.cv[1]-plant.sp[1]);
            double util=0.90+0.08*((double)rand()/RAND_MAX-0.5);
            double mv_sat=(plant.mv[0]>95.0||plant.mv[1]>95.0)?0.1:0.0;
            double pred_err=0.005*track_err+0.003*((double)rand()/RAND_MAX-0.5);

            mpc_kpi_dashboard_update(&db, MPC_KPI_CONTROLLER_UTILIZATION, util, cycle);
            mpc_kpi_dashboard_update(&db, MPC_KPI_SETPOINT_TRACKING_RMSE, 1.0-track_err/50.0, cycle);
            mpc_kpi_dashboard_update(&db, MPC_KPI_HARRIS_INDEX, 0.6+0.3*((double)rand()/RAND_MAX), cycle);
            mpc_kpi_dashboard_update(&db, MPC_KPI_CONSTRAINT_SATISFACTION, 1.0-mv_sat, cycle);
            mpc_kpi_dashboard_update(&db, MPC_KPI_MV_SATURATION_FRACTION, mv_sat, cycle);
            mpc_kpi_dashboard_update(&db, MPC_KPI_ECONOMIC_BENEFIT, 0.07+0.03*((double)rand()/RAND_MAX-0.5), cycle);
            mpc_kpi_dashboard_update(&db, MPC_KPI_MODEL_PLANT_MISMATCH, 0.85+0.1*((double)rand()/RAND_MAX-0.5), cycle);
            mpc_kpi_dashboard_update(&db, MPC_KPI_PREDICTION_ERROR_RMS, 1.0-pred_err, cycle);
            mpc_kpi_dashboard_compute_health(&db);

            if(step_in_sc%10==0){
                print_banner(cycle, scenarios[sc]);
                for(int i=0;i<db.num_kpi;i++)
                    print_kpi(db.kpi_values[i].name, db.kpi_values[i].current_value, db.kpi_values[i].tier);
                print_health(db.overall_health_score);
                if(mpc_kpi_cusum_is_alarm(&cusum))
                    printf("  *** CUSUM ALARM: S+=%.2f S-=% .2f ***\n",
                           mpc_kpi_cusum_positive_stat(&cusum), mpc_kpi_cusum_negative_stat(&cusum));
            }
        }
    }

    /* Final report with all advanced KPIs */
    printf("\n====================== FINAL REPORT ======================\n");
    printf("Total cycles monitored: %llu\n", (unsigned long long)cycle);
    printf("Overall health: %.3f (%s)\n", db.overall_health_score, mpc_kpi_tier_string(db.overall_tier));
    printf("Alarming KPIs: %d | Critical: %d\n", db.num_alarming, db.num_in_critical);

    /* ISO 50001 Energy Performance Indicator */
    double enpi, improvement_pct;
    mpc_kpi_iso50001_enpi(1000.0, 910.0, 100.0, 99.0, &enpi, &improvement_pct);
    printf("ISO 50001 EnPI: %.3f (%.1f%% improvement)\n", enpi, improvement_pct);

    /* CO2 emission reduction */
    double co2=mpc_kpi_co2_reduction_estimate(90.0, 0.42);
    printf("CO2 reduction estimate: %.1f kg\n", co2);

    /* Energy Intensity Index */
    double eii=mpc_kpi_energy_intensity_index(910.0, 99.0, 10.0);
    printf("Energy Intensity Index: %.3f\n", eii);

    /* Autonomous health assessment (L9) */
    double autonomy_readiness, intervention_freq;
    mpc_kpi_autonomous_health_assessment(&db, &autonomy_readiness, &intervention_freq);
    printf("Autonomy readiness: %.3f | Intervention frequency: %.3f\n",
           autonomy_readiness, intervention_freq);

    /* Improvement scenario analysis */
    double projected_health, roi_estimate;
    mpc_kpi_improvement_scenario(&db, 10.0, 1, &projected_health, &roi_estimate, 0.5);
    printf("After 10%% performance improvement: health=%.3f ROI=%.1f%%\n",
           projected_health, roi_estimate);

    /* Throughput monitor */
    double throughput_data[20];
    for(int i=0;i<20;i++) throughput_data[i]=95.0+5.0*((double)rand()/RAND_MAX);
    double avg_tp, tp_loss, tp_compliance, oee_comp;
    mpc_kpi_throughput_monitor(throughput_data, 20, 100.0, &avg_tp, &tp_loss, &tp_compliance, &oee_comp);
    printf("Throughput: avg=%.1f loss=%.3f compliance=%.3f OEE=%.3f\n",
           avg_tp, tp_loss, tp_compliance, oee_comp);

    /* Process capability */
    double cp, cpk, ppm_def;
    mpc_kpi_process_capability(baseline_buf, 120, 105.0, 95.0, &cp, &cpk, &ppm_def);
    printf("Process capability: Cp=%.3f Cpk=%.3f PPM=%.0f\n", cp, cpk, ppm_def);

    mpc_kpi_dashboard_free(&db);
    printf("\nDemo complete. All 5 scenarios processed successfully.\n");
    return 0;
}
