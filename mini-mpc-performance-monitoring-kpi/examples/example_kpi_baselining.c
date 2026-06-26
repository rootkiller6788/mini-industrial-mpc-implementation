/**
 * @file example_kpi_baselining.c
 * @brief KPI baselining and monitoring for an FCC reactor unit.
 *
 * L6 canonical problem: Establish performance baselines for an FCC unit's
 * MPC controller, then detect degradation via CUSUM monitoring. Demonstrates
 * baseline estimation with outlier removal, normality testing, process
 * capability analysis, and degradation rate estimation.
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "../include/mpc_kpi_defs.h"
#include "../include/mpc_kpi_metrics.h"
#include "../include/mpc_kpi_monitoring.h"
#include "../include/mpc_kpi_diagnosis.h"

int main(void)
{
    printf("=== MPC KPI Baselining: FCC Reactor ===\n\n");

    mpc_kpi_dashboard_t db;
    mpc_kpi_dashboard_init(&db,10);

    mpc_kpi_dashboard_register_kpi(&db,MPC_KPI_HARRIS_INDEX,MPC_KPI_CAT_PERFORMANCE);
    mpc_kpi_dashboard_register_kpi(&db,MPC_KPI_CONTROLLER_UTILIZATION,MPC_KPI_CAT_AVAILABILITY);
    mpc_kpi_dashboard_register_kpi(&db,MPC_KPI_CONSTRAINT_SATISFACTION,MPC_KPI_CAT_CONSTRAINT);
    mpc_kpi_dashboard_register_kpi(&db,MPC_KPI_ECONOMIC_BENEFIT,MPC_KPI_CAT_ECONOMIC);
    mpc_kpi_dashboard_register_kpi(&db,MPC_KPI_VARIANCE_REDUCTION,MPC_KPI_CAT_PERFORMANCE);

    /* Baselining phase */
    printf("--- Baselining Phase (200 cycles) ---\n");
    double baseline_data[200];
    for(int i=0;i<200;i++) baseline_data[i]=0.85+0.03*((double)rand()/RAND_MAX-0.5);
    /* Inject a few outliers */
    baseline_data[50]=0.45; baseline_data[120]=1.25;
    mpc_kpi_baseline_t bl;
    mpc_kpi_baseline_estimate(baseline_data,200,2.0,&bl);
    printf("  Mean:     %.4f\n",bl.baseline_mean);
    printf("  Std:      %.4f\n",bl.baseline_std);
    printf("  Median:   %.4f\n",bl.baseline_median);
    printf("  MAD:      %.4f\n",bl.baseline_mad);
    printf("  P5/P95:   %.4f / %.4f\n",bl.baseline_p5,bl.baseline_p95);
    printf("  Samples:  %d (outliers removed: %d)\n",bl.num_samples_used,bl.num_outliers_removed);
    printf("  SW Norm:  W=%.4f p=%.4f %s\n",bl.normality_sw_statistic,bl.normality_pvalue,bl.is_normal?"NORMAL":"non-normal");
    printf("  CV:       %.4f\n",bl.cv_statistic);

    /* Process capability */
    double cp,cpk,ppm;
    mpc_kpi_process_capability(baseline_data,200,1.05,0.65,&cp,&cpk,&ppm);
    printf("  Process Capability: Cp=%.3f Cpk=%.3f PPM=%.0f\n",cp,cpk,ppm);

    /* Oscillation check */
    double osc_per,osc_amp,osc_reg; bool is_osc;
    mpc_kpi_diagnose_oscillation(baseline_data,200,&osc_per,&osc_amp,&osc_reg,&is_osc);
    printf("  Oscillation: period=%.1f amp=%.3f reg=%.3f %s\n",osc_per,osc_amp,osc_reg,is_osc?"OSCILLATING":"stable");

    /* Set baselines for all KPIs */
    for(int i=0;i<db.num_kpi;i++){
        db.kpi_values[i].baseline_mean=bl.baseline_mean;
        db.kpi_values[i].baseline_std=bl.baseline_std;
        db.kpi_values[i].target_value=bl.baseline_mean;
    }

    /* Monitoring phase: gradual degradation */
    printf("\n--- Monitoring Phase (150 cycles with degradation) ---\n");
    double degradation[150], health_history[150];
    for(int i=0;i<150;i++){
        degradation[i]=0.85-0.001*(double)i+0.03*((double)rand()/RAND_MAX-0.5);
    }

    mpc_kpi_cusum_t cusum;
    mpc_kpi_cusum_init(&cusum,bl.baseline_mean,bl.baseline_std*2.0,5.0,1.0);
    bool cusum_triggered=false; int trigger_cycle=-1;

    for(int i=0;i<150;i+=5){
        for(int j=0;j<5&&(i+j)<150;j++)
            mpc_kpi_cusum_update(&cusum,degradation[i+j]);

        if(!cusum_triggered && mpc_kpi_cusum_is_alarm(&cusum)){
            cusum_triggered=true; trigger_cycle=i;
        }

        mpc_kpi_dashboard_update(&db,MPC_KPI_HARRIS_INDEX,degradation[i],200+(uint64_t)i);
        mpc_kpi_dashboard_update(&db,MPC_KPI_VARIANCE_REDUCTION,1.0-0.003*(double)i,200+(uint64_t)i);
        mpc_kpi_dashboard_compute_health(&db);
        health_history[i/5]=db.overall_health_score;

        printf("  Cycle %3d: CUSUM+=%6.2f CUSUM-=%6.2f Health=%.3f Alarm=%s\n",
               i,cusum.cusum_positive,cusum.cusum_negative,db.overall_health_score,
               mpc_kpi_cusum_is_alarm(&cusum)?"YES":"no");
    }

    if(cusum_triggered)
        printf("\n  *** CUSUM alarm triggered at cycle %d! ***\n",trigger_cycle);

    /* Degradation rate estimation */
    double deg_rate,cyc_to_crit;
    mpc_kpi_degradation_rate(health_history,30,&deg_rate,&cyc_to_crit,0.3);
    printf("  Degradation rate: %.5f/cycle\n",deg_rate);
    printf("  Estimated cycles to critical: %.0f\n",cyc_to_crit);

    /* Improvement scenario */
    double proj_health,roi;
    mpc_kpi_improvement_scenario(&db,15.0,1,&proj_health,&roi,0.5);
    printf("  With 15%% performance improvement: health=%.3f ROI=%.1f%%\n",proj_health,roi);

    /* Economic benefit */
    double benefit=mpc_kpi_economic_benefit(100.0,100.0*(1.0-db.overall_health_score));
    printf("  Normalized economic benefit: %.3f\n",benefit);

    mpc_kpi_dashboard_free(&db);
    printf("\nExample complete.\n");
    return 0;
}
