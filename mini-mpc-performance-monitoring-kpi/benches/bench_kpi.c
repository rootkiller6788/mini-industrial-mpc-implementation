/**
 * @file bench_kpi.c
 * @brief Performance benchmarks for MPC KPI monitoring operations.
 *
 * Benchmarks key computational paths: Harris index, autocorrelation,
 * mismatch detection, ring buffer operations, and dashboard aggregation.
 *
 * Benchmark results are reported in operations/second.
 * Reference: Standard benchmarking methodology for real-time control systems.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "../include/mpc_kpi_defs.h"
#include "../include/mpc_kpi_metrics.h"
#include "../include/mpc_kpi_monitoring.h"
#include "../include/mpc_kpi_diagnosis.h"

static double get_time_sec(void)
{
    return (double)clock() / (double)CLOCKS_PER_SEC;
}

static void bench_ringbuffer(int n_ops)
{
    mpc_kpi_ringbuffer_t rb;
    mpc_kpi_ringbuffer_init(&rb, 10000);

    double t0=get_time_sec();
    for(int i=0;i<n_ops;i++){
        mpc_kpi_ringbuffer_push(&rb, (double)(i%1000)*0.01, (uint64_t)i);
    }
    double t1=get_time_sec();
    printf("  Ring buffer push: %.0f ops/sec (%.2f ms for %d ops)\n",
           (double)n_ops/(t1-t0), (t1-t0)*1000.0, n_ops);

    t0=get_time_sec();
    volatile double dummy=0;
    for(int i=0;i<n_ops/100;i++){
        dummy+=mpc_kpi_ringbuffer_mean(&rb);
        dummy+=mpc_kpi_ringbuffer_variance(&rb);
        dummy+=mpc_kpi_ringbuffer_skewness(&rb);
        dummy+=mpc_kpi_ringbuffer_kurtosis(&rb);
    }
    t1=get_time_sec();
    (void)dummy;
    printf("  Ring buffer stats (mean+var+skew+kurt): %.0f ops/sec\n",
           (double)(n_ops/100*4)/(t1-t0));

    mpc_kpi_ringbuffer_free(&rb);
}

static void bench_harris_index(int n_ops)
{
    double data[500];
    for(int i=0;i<500;i++) data[i]=sin(i*0.1)+0.05*((double)rand()/RAND_MAX-0.5);

    mpc_kpi_harris_t hr;
    double t0=get_time_sec();
    for(int i=0;i<n_ops;i++){
        mpc_kpi_compute_harris_index(data, 500, 10, &hr);
        free(hr.impulse_response_coeffs);
    }
    double t1=get_time_sec();
    printf("  Harris index (n=500, lag=10): %.0f ops/sec (%.2f ms/op)\n",
           (double)n_ops/(t1-t0), (t1-t0)*1000.0/(double)n_ops);
}

static void bench_autocorrelation(int n_ops)
{
    double data[200];
    for(int i=0;i<200;i++) data[i]=(double)rand()/RAND_MAX;

    mpc_kpi_autocorr_t ac;
    double t0=get_time_sec();
    for(int i=0;i<n_ops;i++){
        mpc_kpi_autocorrelation(data, 200, 20, &ac);
        free(ac.autocorr);
        free(ac.partial_autocorr);
    }
    double t1=get_time_sec();
    printf("  Autocorrelation (n=200, lag=20): %.0f ops/sec (%.2f ms/op)\n",
           (double)n_ops/(t1-t0), (t1-t0)*1000.0/(double)n_ops);
}

static void bench_mismatch_detection(int n_ops)
{
    double pe[200], mv[200];
    for(int i=0;i<200;i++){pe[i]=(double)rand()/RAND_MAX-0.5; mv[i]=(double)rand()/RAND_MAX;}

    mpc_kpi_mismatch_t mm;
    double t0=get_time_sec();
    for(int i=0;i<n_ops;i++){
        mpc_kpi_detect_model_mismatch(pe, mv, 200, 15, &mm);
        free(mm.correlation_function);
    }
    double t1=get_time_sec();
    printf("  Mismatch detection (n=200, lag=15): %.0f ops/sec (%.2f ms/op)\n",
           (double)n_ops/(t1-t0), (t1-t0)*1000.0/(double)n_ops);
}

static void bench_dashboard(int n_ops)
{
    mpc_kpi_dashboard_t db;
    mpc_kpi_dashboard_init(&db, 20);

    mpc_kpi_dashboard_register_kpi(&db, MPC_KPI_CONTROLLER_UTILIZATION, MPC_KPI_CAT_AVAILABILITY);
    mpc_kpi_dashboard_register_kpi(&db, MPC_KPI_HARRIS_INDEX, MPC_KPI_CAT_PERFORMANCE);
    mpc_kpi_dashboard_register_kpi(&db, MPC_KPI_SETPOINT_TRACKING_RMSE, MPC_KPI_CAT_PERFORMANCE);
    mpc_kpi_dashboard_register_kpi(&db, MPC_KPI_CONSTRAINT_SATISFACTION, MPC_KPI_CAT_CONSTRAINT);
    mpc_kpi_dashboard_register_kpi(&db, MPC_KPI_MV_SATURATION_FRACTION, MPC_KPI_CAT_CONSTRAINT);
    mpc_kpi_dashboard_register_kpi(&db, MPC_KPI_ECONOMIC_BENEFIT, MPC_KPI_CAT_ECONOMIC);

    for(int i=0;i<db.num_kpi;i++){
        db.kpi_values[i].target_value=0.9;
        db.kpi_values[i].baseline_mean=0.85;
        db.kpi_values[i].baseline_std=0.1;
    }

    double t0=get_time_sec();
    for(int i=0;i<n_ops;i++){
        mpc_kpi_dashboard_update(&db, MPC_KPI_CONTROLLER_UTILIZATION, 0.92+(i%10)*0.01, (uint64_t)i);
        mpc_kpi_dashboard_update(&db, MPC_KPI_HARRIS_INDEX, 0.75+(i%10)*0.02, (uint64_t)i);
        mpc_kpi_dashboard_compute_health(&db);
    }
    double t1=get_time_sec();
    printf("  Dashboard (6 KPIs, update+health): %.0f cycles/sec (%.2f us/cycle)\n",
           (double)n_ops/(t1-t0), (t1-t0)*1e6/(double)n_ops);

    mpc_kpi_dashboard_free(&db);
}

static void bench_industrial_kpis(int n_ops)
{
    mpc_kpi_dashboard_t db;
    mpc_kpi_dashboard_init(&db, 10);
    mpc_kpi_dashboard_register_kpi(&db, MPC_KPI_HARRIS_INDEX, MPC_KPI_CAT_PERFORMANCE);
    mpc_kpi_dashboard_update(&db, MPC_KPI_HARRIS_INDEX, 0.85, 1);
    mpc_kpi_dashboard_compute_health(&db);

    double pe[50]; double mc[3]={10,5,8};
    for(int i=0;i<50;i++) pe[i]=0.01*((double)rand()/RAND_MAX-0.5);

    double t0=get_time_sec();
    for(int i=0;i<n_ops;i++){
        double hi,sf,pi,mq,pc,rb,oh,bi,enpi,imp;
        mpc_kpi_aspentech_aspenwatch_kpi(&db, &hi, &sf);
        mpc_kpi_honeywell_profit_sensor(pe, 50, mc, 3, &pi, &mq);
        double cv[3]={95,88,92}, ct[3]={96,90,91};
        mpc_kpi_yokogawa_md_diagnostic(cv, 3, ct, &pc, &rb);
        double mvu[2]={0.7,0.8}, cvv[2]={0.5,0.6};
        mpc_kpi_shell_mv_monitor(mvu, 2, cvv, 2, &oh, &bi);
        mpc_kpi_iso50001_enpi(1000, 900, 100, 105, &enpi, &imp);
    }
    double t1=get_time_sec();
    printf("  Industrial vendor KPIs (5 vendors): %.0f ops/sec (%.2f us/call)\n",
           (double)n_ops/(t1-t0), (t1-t0)*1e6/(double)n_ops);

    mpc_kpi_dashboard_free(&db);
}

static void bench_large_dashboard(int n_kpis)
{
    mpc_kpi_dashboard_t db;
    mpc_kpi_dashboard_init(&db, n_kpis);

    for(int i=0;i<n_kpis;i++){
        mpc_kpi_dashboard_register_kpi(&db, (mpc_kpi_id_t)(i%20), (mpc_kpi_category_t)(i%7));
        db.kpi_values[i].target_value=0.8;
        db.kpi_values[i].baseline_mean=0.75;
        db.kpi_values[i].baseline_std=0.15;
    }

    double t0=get_time_sec();
    for(int i=0;i<1000;i++){
        for(int k=0;k<n_kpis;k++)
            mpc_kpi_dashboard_update(&db, db.kpi_values[k].kpi_id, 0.7+0.1*((double)rand()/RAND_MAX), (uint64_t)i);
        mpc_kpi_dashboard_compute_health(&db);
    }
    double t1=get_time_sec();
    printf("  Large dashboard (%d KPIs, 1000 cycles): %.2f ms total (%.2f us/cycle/KPI)\n",
           n_kpis, (t1-t0)*1000.0, (t1-t0)*1e6/(1000.0*(double)n_kpis));

    mpc_kpi_dashboard_free(&db);
}

int main(void)
{
    printf("MPC KPI Monitoring Performance Benchmarks\n");
    printf("=========================================\n\n");

    printf("--- Ring Buffer Operations ---\n");
    bench_ringbuffer(500000);

    printf("\n--- Harris Index Computation ---\n");
    bench_harris_index(100);

    printf("\n--- Autocorrelation ---\n");
    bench_autocorrelation(500);

    printf("\n--- Model-Plant Mismatch Detection ---\n");
    bench_mismatch_detection(200);

    printf("\n--- Dashboard Aggregation ---\n");
    bench_dashboard(100000);

    printf("\n--- Industrial Vendor KPIs ---\n");
    bench_industrial_kpis(5000);

    printf("\n--- Scalability Test ---\n");
    bench_large_dashboard(50);
    bench_large_dashboard(100);

    printf("\nBenchmarks complete.\n");
    return 0;
}
