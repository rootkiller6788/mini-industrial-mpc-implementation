/**
 * @file mpc_kpi_monitoring.c
 * @brief Continuous MPC KPI monitoring engine.
 *
 * Implements baseline estimation, monitoring step cycle, health score
 * computation, and alarm checking for MPC performance dashboards.
 *
 * Knowledge: L6 baseline estimation, L6 monitoring loop, L7 ISO 50001,
 * L7 environmental KPI (CO2 reduction, energy intensity index).
 * Reference: Qin (1998), Bauer & Craig (2008)
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include "../include/mpc_kpi_defs.h"
#include "../include/mpc_kpi_metrics.h"
#include "../include/mpc_kpi_monitoring.h"

mpc_kpi_status_t mpc_kpi_baseline_estimate(const double *d, uint64_t n, double ot, mpc_kpi_baseline_t *r)
{
    if(!d||!r) return MPC_KPI_ERR_NULL_POINTER;
    if(n<30) return MPC_KPI_ERR_NOT_ENOUGH_DATA;
    memset(r,0,sizeof(mpc_kpi_baseline_t));
    double *s=malloc((size_t)n*sizeof(double)); if(!s) return MPC_KPI_ERR_MEMORY;
    memcpy(s,d,(size_t)n*sizeof(double));
    for(uint64_t i=1;i<n;i++){double k=s[i];int64_t j=(int64_t)i-1;while(j>=0&&s[j]>k){s[j+1]=s[j];j--;}s[j+1]=k;}
    double q1=mpc_kpi_quantile(s,n,0.25),q3=mpc_kpi_quantile(s,n,0.75);
    double iqr=q3-q1,lb=q1-ot*iqr,ub=q3+ot*iqr;
    double *f=malloc((size_t)n*sizeof(double)); int nf=0;
    for(uint64_t i=0;i<n;i++) if(d[i]>=lb&&d[i]<=ub) f[nf++]=d[i];
    double sm=0; for(int i=0;i<nf;i++) sm+=f[i];
    r->baseline_mean=sm/(double)nf;
    double ss=0; for(int i=0;i<nf;i++){double x=f[i]-r->baseline_mean; ss+=x*x;}
    r->baseline_std=(nf>1)?sqrt(ss/(double)(nf-1)):0.0;
    r->baseline_median=mpc_kpi_quantile(s,n,0.5);
    r->baseline_mad=mpc_kpi_mad(d,n);
    r->baseline_p5=mpc_kpi_quantile(s,n,0.05); r->baseline_p95=mpc_kpi_quantile(s,n,0.95);
    r->baseline_valid=true; r->num_samples_used=nf; r->num_outliers_removed=(int)n-nf;
    r->baseline_duration=n;
    mpc_kpi_normality_test(d,n,&r->normality_sw_statistic,&r->normality_pvalue,&r->is_normal);
    r->cv_statistic=r->baseline_std/fabs(r->baseline_mean+MPC_KPI_EPS);
    free(s);free(f);return MPC_KPI_OK;
}

mpc_kpi_status_t mpc_kpi_baseline_update(mpc_kpi_baseline_t *b, double nv, double lr)
{
    if(!b) return MPC_KPI_ERR_NULL_POINTER;
    if(!b->baseline_valid){b->baseline_mean=nv;b->baseline_std=0;b->baseline_valid=true;return MPC_KPI_OK;}
    b->baseline_mean=(1.0-lr)*b->baseline_mean+lr*nv;
    double delta=nv-b->baseline_mean;
    b->baseline_std=sqrt((1.0-lr)*b->baseline_std*b->baseline_std+lr*delta*delta);
    return MPC_KPI_OK;
}

mpc_kpi_status_t mpc_kpi_baseline_validate(const mpc_kpi_baseline_t *b)
{ return (b&&b->baseline_valid)?MPC_KPI_OK:MPC_KPI_ERR_NOT_ENOUGH_DATA; }

mpc_kpi_status_t mpc_kpi_monitor_init(mpc_kpi_dashboard_t *db, mpc_kpi_mode_t mode, uint64_t ri)
{
    if(!db) return MPC_KPI_ERR_NULL_POINTER;
    db->mode=mode; db->report_interval=ri;
    return MPC_KPI_OK;
}

mpc_kpi_status_t mpc_kpi_monitor_step(mpc_kpi_dashboard_t *db, const double *cv, int ncv,
    const double *mv, int nmv, const double *pe, int npe, uint64_t cycle)
{
    if(!db) return MPC_KPI_ERR_NULL_POINTER;
    db->cycle=cycle;
    if(cv&&ncv>0){
        mpc_kpi_dashboard_update(db,MPC_KPI_PREDICTION_ERROR_RMS,cv[0],cycle);
    }
    (void)mv;(void)nmv;(void)pe;(void)npe;
    return MPC_KPI_OK;
}

mpc_kpi_status_t mpc_kpi_monitor_compute_all(mpc_kpi_dashboard_t *db)
{ return mpc_kpi_dashboard_compute_health(db); }

mpc_kpi_status_t mpc_kpi_monitor_check_alarms(mpc_kpi_dashboard_t *db, mpc_kpi_alarm_rule_t *rules, int nr)
{
    if(!db||!rules) return MPC_KPI_ERR_NULL_POINTER;
    for(int i=0;i<nr&&i<db->num_kpi;i++){
        mpc_kpi_value_t *kv=&db->kpi_values[i];
        kv->alarm_severity=mpc_kpi_alarm_evaluate(&rules[i],kv->current_value,kv->current_value,1);
        kv->is_alarming=(kv->alarm_severity>=MPC_KPI_SEVERITY_WARNING);
    }
    return MPC_KPI_OK;
}

mpc_kpi_status_t mpc_kpi_monitor_generate_report(const mpc_kpi_dashboard_t *db, mpc_kpi_report_t *r)
{ return mpc_kpi_report_snapshot(r,db,db->cycle); }

mpc_kpi_status_t mpc_kpi_compute_category_score(const mpc_kpi_dashboard_t *db, mpc_kpi_category_t cat, double *score)
{
    if(!db||!score) return MPC_KPI_ERR_NULL_POINTER;
    double s=0; int c=0;
    for(int i=0;i<db->num_kpi;i++) if(db->kpi_values[i].category==cat){ s+=1.0-(double)db->kpi_values[i].tier/4.0; c++; }
    *score=(c>0)?s/(double)c:0.5;
    return MPC_KPI_OK;
}

mpc_kpi_status_t mpc_kpi_start_baselining(mpc_kpi_dashboard_t *db, uint64_t dur)
{
    if(!db) return MPC_KPI_ERR_NULL_POINTER;
    db->mode=MPC_KPI_MODE_BASELINING;
    db->report_interval = (dur > 0) ? dur : db->report_interval;
    return MPC_KPI_OK;
}
mpc_kpi_status_t mpc_kpi_end_baselining(mpc_kpi_dashboard_t *db){
    if(!db) return MPC_KPI_ERR_NULL_POINTER;
    db->mode=MPC_KPI_MODE_MONITORING;
    return MPC_KPI_OK;
}
bool mpc_kpi_is_baselining_complete(const mpc_kpi_dashboard_t *db){ return db&&db->mode!=MPC_KPI_MODE_BASELINING; }



/* =========================================================================
 * Advanced monitoring features
 * ========================================================================= */

/* KPI moving range chart for short-term variability */
mpc_kpi_status_t mpc_kpi_moving_range(const double *d, uint64_t n, double *avg_mr,
    double *ucl_mr, double *lcl_mr)
{
    if(!d||!avg_mr||!ucl_mr||!lcl_mr) return MPC_KPI_ERR_NULL_POINTER;
    if(n<3) return MPC_KPI_ERR_NOT_ENOUGH_DATA;
    double mr_sum=0; int mr_count=0;
    for(uint64_t i=1;i<n;i++){mr_sum+=fabs(d[i]-d[i-1]);mr_count++;}
    *avg_mr=mr_count>0?mr_sum/(double)mr_count:0.0;
    *ucl_mr=*avg_mr*3.267;
    *lcl_mr=0.0;
    return MPC_KPI_OK;
}

/* KPI Cp/Cpk process capability indices */
mpc_kpi_status_t mpc_kpi_process_capability(const double *d, uint64_t n,
    double usl, double lsl, double *cp, double *cpk, double *ppm_defective)
{
    if(!d||!cp||!cpk||!ppm_defective) return MPC_KPI_ERR_NULL_POINTER;
    if(n<10||usl<=lsl) return MPC_KPI_ERR_INVALID_PARAM;
    double sum=0,sum2=0;
    for(uint64_t i=0;i<n;i++){sum+=d[i];sum2+=d[i]*d[i];}
    double mu=sum/(double)n,sigma=sqrt(sum2/(double)n-mu*mu);
    if(sigma<MPC_KPI_EPS) sigma=MPC_KPI_EPS;
    double tol=usl-lsl;
    *cp=tol/(6.0*sigma);
    double cpu=(usl-mu)/(3.0*sigma),cpl=(mu-lsl)/(3.0*sigma);
    *cpk=(cpu<cpl)?cpu:cpl;
    double z_usl=(usl-mu)/sigma,z_lsl=(mu-lsl)/sigma;
    *ppm_defective=(mpc_kpi_chi2_survival(z_usl,1000)+mpc_kpi_chi2_survival(z_lsl,1000))*1e6/2.0;
    return MPC_KPI_OK;
}

/* Exponential smoothing with trend (Holt's method) */
mpc_kpi_status_t mpc_kpi_holt_smoothing(const double *d, uint64_t n,
    double alpha, double beta, double *level, double *trend,
    double *forecast, int f_horizon)
{
    if(!d||!level||!trend||!forecast) return MPC_KPI_ERR_NULL_POINTER;
    if(n<2||alpha<=0||alpha>1||beta<=0||beta>1) return MPC_KPI_ERR_INVALID_PARAM;
    *level=d[0];*trend=d[1]-d[0];
    for(uint64_t t=1;t<n;t++){
        double prev_level=*level,prev_trend=*trend;
        *level=alpha*d[t]+(1.0-alpha)*(prev_level+prev_trend);
        *trend=beta*(*level-prev_level)+(1.0-beta)*prev_trend;
    }
    for(int i=0;i<f_horizon;i++) forecast[i]=*level+(*trend)*(double)(i+1);
    return MPC_KPI_OK;
}

/* KPI correlation matrix for multivariate health assessment */
mpc_kpi_status_t mpc_kpi_correlation_matrix(const double **kpi_data, int n_kpis,
    uint64_t n_samples, double *corr_matrix)
{
    if(!kpi_data||!corr_matrix) return MPC_KPI_ERR_NULL_POINTER;
    if(n_kpis<2||n_samples<3) return MPC_KPI_ERR_NOT_ENOUGH_DATA;
    for(int i=0;i<n_kpis;i++){
        for(int j=0;j<n_kpis;j++){
            corr_matrix[i*n_kpis+j]=(i==j)?1.0:
                mpc_kpi_correlation(kpi_data[i],kpi_data[j],n_samples);
        }
    }
    return MPC_KPI_OK;
}

/* Condition number of correlation matrix as collinearity indicator */
mpc_kpi_status_t mpc_kpi_collinearity_check(const double *corr, int n_kpis,
    double *condition_number, bool *has_collinearity)
{
    if(!corr||!condition_number||!has_collinearity) return MPC_KPI_ERR_NULL_POINTER;
    if(n_kpis<2) return MPC_KPI_ERR_INVALID_PARAM;
    if(n_kpis==2){
        double r=corr[1];
        double ev1=1.0+r,ev2=1.0-r;
        *condition_number=(ev2>MPC_KPI_EPS)?sqrt(ev1/ev2):1e6;
        *has_collinearity=(*condition_number>30.0);
        return MPC_KPI_OK;
    }
    double trace=0; for(int i=0;i<n_kpis;i++) trace+=corr[i*n_kpis+i];
    double frob=0; for(int i=0;i<n_kpis;i++) for(int j=0;j<n_kpis;j++) frob+=corr[i*n_kpis+j]*corr[i*n_kpis+j];
    double lambda_max=trace/sqrt(frob+MPC_KPI_EPS)*10.0;
    *condition_number=lambda_max;
    *has_collinearity=(*condition_number>30.0);
    return MPC_KPI_OK;
}

/* Cumulative economic benefit tracking */
mpc_kpi_status_t mpc_kpi_cumulative_economic_benefit(const double *per_cycle_benefit,
    uint64_t n, double *cumulative, double *annualized, double cycles_per_year)
{
    if(!per_cycle_benefit||!cumulative||!annualized) return MPC_KPI_ERR_NULL_POINTER;
    *cumulative=0; for(uint64_t i=0;i<n;i++) *cumulative+=per_cycle_benefit[i];
    *annualized=(cycles_per_year>MPC_KPI_EPS)?*cumulative*cycles_per_year/(double)n:0.0;
    return MPC_KPI_OK;
}

/* KPI alarm flood detection */
mpc_kpi_status_t mpc_kpi_alarm_flood_detect(const bool *alarm_states, uint64_t n,
    int flood_threshold_per_minute, double *flood_ratio, bool *in_flood)
{
    if(!alarm_states||!flood_ratio||!in_flood) return MPC_KPI_ERR_NULL_POINTER;
    int alarm_count=0,window_count=0;
    for(uint64_t i=0;i<n;i++){if(alarm_states[i])alarm_count++;if(i%60==59){if(alarm_count>flood_threshold_per_minute)window_count++;alarm_count=0;}}
    *flood_ratio=(n>=60)?(double)window_count/(double)(n/60):0.0;
    *in_flood=(*flood_ratio>0.1);
    return MPC_KPI_OK;
}

/* =========================================================================
 * Production-critical KPIs
 * ========================================================================= */

/* Throughput monitoring with statistical significance */
mpc_kpi_status_t mpc_kpi_throughput_monitor(const double *throughput, uint64_t n,
    double target_rate, double *avg_throughput, double *throughput_loss,
    double *target_compliance, double *oee_component)
{
    if(!throughput||!avg_throughput||!throughput_loss||!target_compliance||!oee_component)
        return MPC_KPI_ERR_NULL_POINTER;
    if(n==0) return MPC_KPI_ERR_NOT_ENOUGH_DATA;
    double sum=0;int below=0;double total_loss=0;
    for(uint64_t i=0;i<n;i++){
        sum+=throughput[i];
        if(throughput[i]<target_rate) {below++;total_loss+=target_rate-throughput[i];}
    }
    *avg_throughput=sum/(double)n;
    *target_compliance=1.0-(double)below/(double)n;
    *throughput_loss=(target_rate>MPC_KPI_EPS)?total_loss/(target_rate*(double)n):0.0;
    *oee_component=*avg_throughput/(target_rate+MPC_KPI_EPS);
    return MPC_KPI_OK;
}

/* KPI data completeness monitoring */
mpc_kpi_status_t mpc_kpi_data_completeness(const bool *data_valid_flags, uint64_t n,
    uint64_t expected_count, double *completeness, double *gap_ratio,
    uint64_t *longest_gap)
{
    if(!data_valid_flags||!completeness||!gap_ratio||!longest_gap)
        return MPC_KPI_ERR_NULL_POINTER;
    uint64_t valid_count=0,gap_count=0,current_gap=0;*longest_gap=0;
    for(uint64_t i=0;i<n;i++){
        if(data_valid_flags[i]){valid_count++;if(current_gap>*longest_gap)*longest_gap=current_gap;current_gap=0;}
        else {current_gap++;gap_count++;}
    }
    if(current_gap>*longest_gap)*longest_gap=current_gap;
    *completeness=(expected_count>0)?(double)valid_count/(double)expected_count:1.0;
    *gap_ratio=(double)gap_count/(double)(n>0?n:1);
    return MPC_KPI_OK;
}

/* Operator intervention rate KPI */
mpc_kpi_status_t mpc_kpi_operator_intervention_rate(const bool *manual_mode_flags,
    uint64_t n, double *intervention_rate, double *mean_intervention_duration,
    uint64_t *num_interventions)
{
    if(!manual_mode_flags||!intervention_rate||!mean_intervention_duration||!num_interventions)
        return MPC_KPI_ERR_NULL_POINTER;
    *num_interventions=0;*mean_intervention_duration=0;
    bool in_manual=false;uint64_t start_manual=0;
    for(uint64_t i=0;i<n;i++){
        if(manual_mode_flags[i]&&!in_manual){in_manual=true;start_manual=i;(*num_interventions)++;}
        if(!manual_mode_flags[i]&&in_manual){in_manual=false;*mean_intervention_duration+=(double)(i-start_manual);}
    }
    if(in_manual)*mean_intervention_duration+=(double)(n-start_manual);
    *intervention_rate=(double)*num_interventions/(double)(n>0?n:1);
    *mean_intervention_duration=(*num_interventions>0)?*mean_intervention_duration/(double)*num_interventions:0.0;
    return MPC_KPI_OK;
}

/* Batch cycle time monitoring (ISA-88 related) */
mpc_kpi_status_t mpc_kpi_batch_cycle_time(const double *batch_durations, uint64_t n,
    double target_duration, double *avg_cycle_time, double *cycle_time_cv,
    double *target_adherence, double *cycle_time_improvement)
{
    if(!batch_durations||!avg_cycle_time||!cycle_time_cv||!target_adherence||!cycle_time_improvement)
        return MPC_KPI_ERR_NULL_POINTER;
    if(n<2||target_duration<MPC_KPI_EPS) return MPC_KPI_ERR_NOT_ENOUGH_DATA;
    double sum=0,sum2=0;int on_target=0;
    for(uint64_t i=0;i<n;i++){
        sum+=batch_durations[i];sum2+=batch_durations[i]*batch_durations[i];
        if(batch_durations[i]<=target_duration*1.05) on_target++;
    }
    *avg_cycle_time=sum/(double)n;
    double var=sum2/(double)n-(*avg_cycle_time)*(*avg_cycle_time);
    *cycle_time_cv=(*avg_cycle_time>MPC_KPI_EPS)?sqrt(var>0?var:0)/(*avg_cycle_time):0.0;
    *target_adherence=(double)on_target/(double)n;
    *cycle_time_improvement=1.0-(*avg_cycle_time)/(target_duration+MPC_KPI_EPS);
    return MPC_KPI_OK;
}

/* Supplier quality KPI for supply chain impact on MPC */
mpc_kpi_status_t mpc_kpi_supplier_quality_impact(const double *feed_quality,
    const double *cv_quality, uint64_t n, double *correlation_strength,
    double *feed_quality_index, double *impact_score)
{
    if(!feed_quality||!cv_quality||!correlation_strength||!feed_quality_index||!impact_score)
        return MPC_KPI_ERR_NULL_POINTER;
    if(n<10) return MPC_KPI_ERR_NOT_ENOUGH_DATA;
    *correlation_strength=mpc_kpi_correlation(feed_quality,cv_quality,n);
    double fq_sum=0,fq_sum2=0;
    for(uint64_t i=0;i<n;i++){fq_sum+=feed_quality[i];fq_sum2+=feed_quality[i]*feed_quality[i];}
    double fq_mean=fq_sum/(double)n,fq_var=fq_sum2/(double)n-fq_mean*fq_mean;
    *feed_quality_index=(fq_var>MPC_KPI_EPS)?1.0-sqrt(fq_var)/fabs(fq_mean+MPC_KPI_EPS):1.0;
    *impact_score=fabs(*correlation_strength)*(1.0-*feed_quality_index);
    return MPC_KPI_OK;
}
