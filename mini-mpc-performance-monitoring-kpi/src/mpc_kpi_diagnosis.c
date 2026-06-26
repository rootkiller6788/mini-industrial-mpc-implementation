/** @file mpc_kpi_diagnosis.c */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include "../include/mpc_kpi_defs.h"
#include "../include/mpc_kpi_metrics.h"
#include "../include/mpc_kpi_diagnosis.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include "../include/mpc_kpi_defs.h"
#include "../include/mpc_kpi_metrics.h"
#include "../include/mpc_kpi_diagnosis.h"

mpc_kpi_status_t mpc_kpi_diagnose_degradation(const mpc_kpi_dashboard_t *db,
    const double *pe, uint64_t npe, const double *mv, uint64_t nmv, int *pc, double *cf)
{
    if(!db||!pc||!cf) return MPC_KPI_ERR_NULL_POINTER;
    *pc=0; *cf=0.5;
    if(pe&&npe>30){
        mpc_kpi_autocorr_t ac; mpc_kpi_autocorrelation(pe,npe,10,&ac);
        if(!ac.is_white_noise){*pc=1; *cf=ac.ljung_box_pvalue;}
        free(ac.autocorr); free(ac.partial_autocorr);
    }
    if(mv&&nmv>30){
        double sx=0; for(uint64_t i=0;i<nmv;i++) sx+=mv[i];
        double mn=sx/(double)nmv, ss=0;
        for(uint64_t i=0;i<nmv;i++){double d=mv[i]-mn; ss+=d*d;}
        ss/=(double)nmv; if(ss<MPC_KPI_EPS&&*cf>0.3){*pc=3;*cf=0.8;}
    }
    if(db->overall_tier>=MPC_KPI_TIER_POOR&&*cf<0.5){*pc=2;*cf=0.6;}
    return MPC_KPI_OK;
}

mpc_kpi_status_t mpc_kpi_diagnose_oscillation(const double *d, uint64_t n,
    double *per, double *amp, double *reg, bool *is_osc)
{
    if(!d||!per||!amp||!reg||!is_osc) return MPC_KPI_ERR_NULL_POINTER;
    if(n<20) return MPC_KPI_ERR_NOT_ENOUGH_DATA;
    mpc_kpi_autocorr_t ac; mpc_kpi_autocorrelation(d,n,(int)(n/4),&ac);
    *per=0; int pk_l=0; double pk_v=0;
    for(int k=1;k<ac.num_lags/2;k++){
        if(ac.autocorr[k]>pk_v&&ac.autocorr[k-1]<ac.autocorr[k]&&ac.autocorr[k+1]<ac.autocorr[k])
        {pk_v=ac.autocorr[k]; pk_l=k;}
    }
    *per=(double)(pk_l>0?pk_l:1);
    double mn=0; for(uint64_t i=0;i<n;i++) mn+=d[i]; mn/=(double)n;
    double rms=0; for(uint64_t i=0;i<n;i++){double x=d[i]-mn; rms+=x*x;}
    *amp=sqrt(rms/(double)n)*2.0;
    *reg=pk_v; *is_osc=(pk_v>0.3);
    free(ac.autocorr); free(ac.partial_autocorr);
    return MPC_KPI_OK;
}

mpc_kpi_status_t mpc_kpi_diagnose_sluggishness(const double *sp, const double *cv, uint64_t n,
    double *st, double *rt, double *os)
{
    if(!sp||!cv||!st||!rt||!os) return MPC_KPI_ERR_NULL_POINTER;
    *st=0;*rt=0;*os=0;
    uint64_t cs=0; bool it=false; double mc=0,sv=0;
    for(uint64_t i=1;i<n;i++){
        if(fabs(sp[i]-sp[i-1])>MPC_KPI_EPS&&!it){cs=i;it=true;sv=cv[i-1];mc=cv[i];}
        if(it){
            if(cv[i]>mc) mc=cv[i];
            if(fabs(cv[i]-sp[i])<0.02*fabs(sp[i]-sv+MPC_KPI_EPS)&&i-cs>5){
                *st=(double)(i-cs); it=false;
                double dsp=sp[cs]-sv;
                *os=fabs(dsp)>MPC_KPI_EPS?fabs((mc-sp[cs])/dsp)*100.0:0.0;
                *rt=fabs(dsp)>MPC_KPI_EPS?fabs(*st*0.3):*st*0.3;
            }
        }
    }
    return MPC_KPI_OK;
}

mpc_kpi_status_t mpc_kpi_diagnose_stiction(const double *mv, const double *cv, uint64_t n,
    double *sb, bool *hs, double *sjr)
{
    if(!mv||!cv||!sb||!hs||!sjr) return MPC_KPI_ERR_NULL_POINTER;
    if(n<10) return MPC_KPI_ERR_NOT_ENOUGH_DATA;
    *sb=0;*hs=false;*sjr=0;
    int sc=0, sl=0;
    for(uint64_t i=1;i<n;i++){
        if(fabs(mv[i]-mv[i-1])>MPC_KPI_EPS&&fabs(cv[i]-cv[i-1])<MPC_KPI_EPS) sc++;
        if(fabs(cv[i]-cv[i-1])>MPC_KPI_EPS) sl++;
    }
    *sb=sc>0?fabs(mv[n-1]-mv[0])/(double)sc:0.0;
    *sjr=sl>0?(double)sc/(double)sl:0.0;
    *hs=(*sjr>0.5||sc>(int)n/4);
    return MPC_KPI_OK;
}

mpc_kpi_status_t mpc_kpi_isolate_mismatch_channel(const double *pe, uint64_t npe,
    const double *mv, uint64_t nmv, int ncv, int nmv2, int *mci, int *mmi, mpc_mismatch_type_t *types)
{
    if(!pe||!mci||!mmi) return MPC_KPI_ERR_NULL_POINTER;
    *mci=-1; *mmi=-1;
    if(ncv>0&&nmv2>0&&npe>10){
        mpc_kpi_mismatch_t mm;
        mpc_kpi_detect_model_mismatch(pe,mv,npe<nmv?npe:nmv,5,&mm);
        *mci=mm.correlation_lag%ncv; *mmi=0;
        if(types) types[0]=mm.primary_type;
        free(mm.correlation_function);
    }
    return MPC_KPI_OK;
}

mpc_kpi_status_t mpc_kpi_residual_whiteness_test(const double *res, uint64_t n,
    int ml, double sig, bool *iw, double *ts, double *cv)
{
    if(!res||!iw||!ts||!cv) return MPC_KPI_ERR_NULL_POINTER;
    double q,pv; int dof;
    mpc_kpi_ljung_box_test(res,n,ml,&q,&pv,&dof,iw);
    *ts=q; *cv=mpc_kpi_chi2_survival(q,dof);
    *iw=(pv>sig); return MPC_KPI_OK;
}

mpc_kpi_status_t mpc_kpi_forecast_kpi(const double *kh, uint64_t n, int fh,
    double *fv, double *lo, double *hi, double cl)
{
    if(!kh||!fv||n<5||fh<1) return MPC_KPI_ERR_NOT_ENOUGH_DATA;
    double sl,ic,r2; mpc_kpi_linear_trend(kh,n,&sl,&ic,&r2);
    double var=0; for(uint64_t i=0;i<n;i++){double e=kh[i]-(ic+sl*(double)i);var+=e*e;}
    var/=(double)(n-2);
    for(int i=0;i<fh;i++) fv[i]=ic+sl*((double)n+(double)i);
    double z=(cl+1.0)/2.0; if(z>0.99)z=0.99;
    double se=sqrt(var*(1.0+1.0/(double)n));
    for(int i=0;i<fh;i++){lo[i]=fv[i]-z*se; hi[i]=fv[i]+z*se;}
    return MPC_KPI_OK;
}

mpc_kpi_status_t mpc_kpi_forecast_degradation_time(const double *kh, uint64_t n,
    double at, double *eca, double *cf)
{
    if(!kh||!eca||!cf||n<5) return MPC_KPI_ERR_NOT_ENOUGH_DATA;
    double sl,ic,r2; mpc_kpi_linear_trend(kh,n,&sl,&ic,&r2);
    *cf=r2;
    if(fabs(sl)<MPC_KPI_EPS){*eca=-1.0;return MPC_KPI_OK;}
    *eca=(at-ic)/sl-(double)n; if(*eca<0) *eca=-1.0;
    return MPC_KPI_OK;
}

mpc_kpi_status_t mpc_kpi_generate_recommendations(const mpc_kpi_dashboard_t *db,
    const mpc_kpi_mismatch_t *mm, char *rec, int mc)
{
    if(!db||!rec) return MPC_KPI_ERR_NULL_POINTER;
    int p=0;
    if(db->overall_tier>=MPC_KPI_TIER_POOR)
        p+=snprintf(rec+p,(size_t)(mc-p),"OVERALL: Performance degraded. ");
    if(mm&&mm->is_significant)
        p+=snprintf(rec+p,(size_t)(mc-p),"MODEL: Mismatch %.1f%%. ",mm->mismatch_magnitude*100.0);
    if(db->num_alarming>0)
        p+=snprintf(rec+p,(size_t)(mc-p),"ALARM: %d KPIs alarming. ",db->num_alarming);
    if(p==0) snprintf(rec,(size_t)mc,"All KPIs normal.");
    return MPC_KPI_OK;
}

mpc_kpi_status_t mpc_kpi_model_update_urgency(const mpc_kpi_mismatch_t *mm, double ei,
    double *us, char *ul, int lm)
{
    if(!mm||!us||!ul) return MPC_KPI_ERR_NULL_POINTER;
    *us=mm->mismatch_magnitude*ei*100.0;
    if(*us>75.0) snprintf(ul,(size_t)lm,"CRITICAL: Update immediately");
    else if(*us>50.0) snprintf(ul,(size_t)lm,"HIGH: Schedule update this week");
    else if(*us>25.0) snprintf(ul,(size_t)lm,"MEDIUM: Plan update next month");
    else snprintf(ul,(size_t)lm,"LOW: Monitor, no urgent action");
    return MPC_KPI_OK;
}

/* =========================================================================
 * Advanced diagnosis functions
 * ========================================================================= */

/* Ratio control performance diagnosis */
mpc_kpi_status_t mpc_kpi_diagnose_ratio_violation(const double *pv, const double *sv,
    uint64_t n, double ratio_target, double tolerance_pct,
    double *actual_ratio_mean, double *ratio_violation_rate, bool *ratio_maintained)
{
    if(!pv||!sv||!actual_ratio_mean||!ratio_violation_rate||!ratio_maintained)
        return MPC_KPI_ERR_NULL_POINTER;
    if(n<2) return MPC_KPI_ERR_NOT_ENOUGH_DATA;
    double ratio_sum=0; int viol_count=0;
    for(uint64_t i=0;i<n;i++){
        double r=(fabs(sv[i])>MPC_KPI_EPS)?pv[i]/sv[i]:1.0;
        ratio_sum+=r;
        if(fabs(r-ratio_target)/fabs(ratio_target+MPC_KPI_EPS)>tolerance_pct*0.01) viol_count++;
    }
    *actual_ratio_mean=ratio_sum/(double)n;
    *ratio_violation_rate=(double)viol_count/(double)n;
    *ratio_maintained=(*ratio_violation_rate<0.05);
    return MPC_KPI_OK;
}

/* Valve travel histogram for maintenance prediction */
mpc_kpi_status_t mpc_kpi_valve_travel_histogram(const double *mv, uint64_t n,
    double *travel_total, double *travel_mean, double *travel_max,
    double *reversal_count, double *stiction_indicator)
{
    if(!mv||!travel_total||!travel_mean||!travel_max||!reversal_count||!stiction_indicator)
        return MPC_KPI_ERR_NULL_POINTER;
    if(n<3) return MPC_KPI_ERR_NOT_ENOUGH_DATA;
    *travel_total=0;*reversal_count=0;*travel_max=0;
    int direction=(mv[1]>mv[0])?1:((mv[1]<mv[0])?-1:0);
    for(uint64_t i=1;i<n;i++){
        double step=fabs(mv[i]-mv[i-1]);
        *travel_total+=step;
        if(step>*travel_max)*travel_max=step;
        int new_dir=(mv[i]>mv[i-1])?1:((mv[i]<mv[i-1])?-1:0);
        if(new_dir!=0&&new_dir!=direction){(*reversal_count)++;direction=new_dir;}
    }
    *travel_mean=*travel_total/(double)(n-1);
    *stiction_indicator=(*reversal_count>0)?(*travel_total/(*reversal_count+1.0)):0.0;
    return MPC_KPI_OK;
}

/* MV constraint activity analysis */
mpc_kpi_status_t mpc_kpi_constraint_activity_analysis(const double *lagrange_multipliers,
    uint64_t n, int n_constraints, double *activity_fraction, int *most_active_idx,
    double *avg_multiplier)
{
    if(!lagrange_multipliers||!activity_fraction||!most_active_idx||!avg_multiplier)
        return MPC_KPI_ERR_NULL_POINTER;
    if(n==0||n_constraints<1) return MPC_KPI_ERR_NOT_ENOUGH_DATA;
    *avg_multiplier=0; int act_count=0; *most_active_idx=0; double max_act=0;
    double *per_constr_act=calloc((size_t)n_constraints,sizeof(double));
    if(!per_constr_act) return MPC_KPI_ERR_MEMORY;
    for(uint64_t i=0;i<n;i++){
        for(int j=0;j<n_constraints;j++){
            double lm=lagrange_multipliers[i*(uint64_t)n_constraints+(uint64_t)j];
            if(fabs(lm)>MPC_KPI_EPS){per_constr_act[j]++;act_count++;}
            *avg_multiplier+=fabs(lm);
        }
    }
    *avg_multiplier/=(double)(n*(uint64_t)n_constraints+1);
    *activity_fraction=(double)act_count/(double)(n*(uint64_t)n_constraints+1);
    for(int j=0;j<n_constraints;j++) if(per_constr_act[j]>max_act){max_act=per_constr_act[j];*most_active_idx=j;}
    free(per_constr_act);
    return MPC_KPI_OK;
}

/* Setpoint change frequency analysis */
mpc_kpi_status_t mpc_kpi_setpoint_change_analysis(const double *sp, uint64_t n,
    double *changes_per_cycle, double *mean_change_magnitude, uint64_t *num_changes)
{
    if(!sp||!changes_per_cycle||!mean_change_magnitude||!num_changes)
        return MPC_KPI_ERR_NULL_POINTER;
    *num_changes=0;*mean_change_magnitude=0;
    for(uint64_t i=1;i<n;i++){
        double delta=fabs(sp[i]-sp[i-1]);
        if(delta>MPC_KPI_EPS){(*num_changes)++;*mean_change_magnitude+=delta;}
    }
    *changes_per_cycle=(double)*num_changes/(double)n;
    *mean_change_magnitude=(*num_changes>0)?*mean_change_magnitude/(double)*num_changes:0.0;
    return MPC_KPI_OK;
}

/* Performance degradation rate estimation */
mpc_kpi_status_t mpc_kpi_degradation_rate(const double *health_history, uint64_t n,
    double *degradation_per_cycle, double *cycles_to_critical, double critical_threshold)
{
    if(!health_history||!degradation_per_cycle||!cycles_to_critical)
        return MPC_KPI_ERR_NULL_POINTER;
    if(n<10) return MPC_KPI_ERR_NOT_ENOUGH_DATA;
    double sl,ic,r2; mpc_kpi_linear_trend(health_history,n,&sl,&ic,&r2);
    *degradation_per_cycle=-sl;
    double current=health_history[n-1];
    *cycles_to_critical=(sl<-MPC_KPI_EPS)?(critical_threshold-current)/sl:(sl>MPC_KPI_EPS)?1e9:-1.0;
    if(*cycles_to_critical<0)*cycles_to_critical=-1.0;
    return MPC_KPI_OK;
}

/* Interactive KPI improvement scenario analysis */
mpc_kpi_status_t mpc_kpi_improvement_scenario(const mpc_kpi_dashboard_t *db,
    double improvement_pct, int target_category, double *projected_health,
    double *roi_estimate, double cost_per_point)
{
    if(!db||!projected_health||!roi_estimate) return MPC_KPI_ERR_NULL_POINTER;
    double scores[5]={db->availability_score,db->performance_score,db->quality_score,
                      db->economic_score,db->constraint_score};
    if(target_category>=0&&target_category<5) scores[target_category]*=(1.0+improvement_pct*0.01);
    double weights[5]={0.25,0.25,0.15,0.20,0.15};
    *projected_health=mpc_kpi_weighted_health_score(scores,weights,5);
    double health_gain=*projected_health-db->overall_health_score;
    *roi_estimate=(cost_per_point>MPC_KPI_EPS)?(health_gain*100.0-cost_per_point)/cost_per_point*100.0:0.0;
    return MPC_KPI_OK;
}

/* =========================================================================
 * Additional diagnosis capabilities
 * ========================================================================= */

/* Nonlinearity detection via bispectrum surrogate */
mpc_kpi_status_t mpc_kpi_detect_nonlinearity(const double *data, uint64_t n,
    double *nonlinearity_index, bool *is_nonlinear, double *bicoherence_peak)
{
    if(!data||!nonlinearity_index||!is_nonlinear||!bicoherence_peak)
        return MPC_KPI_ERR_NULL_POINTER;
    if(n<50) return MPC_KPI_ERR_NOT_ENOUGH_DATA;
    double sum=0,sum2=0,sum3=0;
    for(uint64_t i=0;i<n;i++){sum+=data[i];sum2+=data[i]*data[i];sum3+=data[i]*data[i]*data[i];}
    double m1=sum/(double)n,m2=sum2/(double)n-m1*m1;
    double m3=sum3/(double)n-3.0*m1*sum2/(double)n+2.0*m1*m1*m1;
    double skew=(m2>MPC_KPI_EPS)?m3/pow(m2,1.5):0.0;
    *nonlinearity_index=fabs(skew);
    *is_nonlinear=(*nonlinearity_index>1.0);
    *bicoherence_peak=(*is_nonlinear)?*nonlinearity_index*0.5:0.0;
    return MPC_KPI_OK;
}

/* Controller tuning aggressiveness index */
mpc_kpi_status_t mpc_kpi_tuning_aggressiveness(const double *mv_moves, uint64_t n,
    const double *mv_bounds, double *aggressiveness, double *move_smoothness,
    bool *overly_aggressive)
{
    if(!mv_moves||!mv_bounds||!aggressiveness||!move_smoothness||!overly_aggressive)
        return MPC_KPI_ERR_NULL_POINTER;
    if(n<10) return MPC_KPI_ERR_NOT_ENOUGH_DATA;
    double move_rms=0,rms_change=0;
    for(uint64_t i=0;i<n;i++){move_rms+=mv_moves[i]*mv_moves[i];}
    move_rms=sqrt(move_rms/(double)n);
    for(uint64_t i=1;i<n;i++){double ch=mv_moves[i]-mv_moves[i-1];rms_change+=ch*ch;}
    rms_change=sqrt(rms_change/(double)(n-1));
    double mv_range=fabs(mv_bounds[1]-mv_bounds[0]);
    *aggressiveness=(mv_range>MPC_KPI_EPS)?move_rms/mv_range:0.0;
    *move_smoothness=(move_rms>MPC_KPI_EPS)?1.0-rms_change/(move_rms+MPC_KPI_EPS):1.0;
    *overly_aggressive=(*aggressiveness>0.3||*move_smoothness<0.3);
    return MPC_KPI_OK;
}

/* Idle time and changeover analysis (ISA-95 equipment) */
mpc_kpi_status_t mpc_kpi_idle_time_analysis(const bool *running_flags, uint64_t n,
    uint64_t cycle_time_sec, double *utilization_pct, double *mttf,
    double *mttr, uint64_t *num_stops)
{
    if(!running_flags||!utilization_pct||!mttf||!mttr||!num_stops)
        return MPC_KPI_ERR_NULL_POINTER;
    uint64_t run_time=0,stop_time=0;*num_stops=0;
    bool was_running=running_flags[0];
    uint64_t current_run=0,current_stop=0;
    double total_runs=0,total_stops=0;
    for(uint64_t i=0;i<n;i++){
        if(running_flags[i]){run_time++;current_run++;
            if(!was_running){total_stops+=(double)current_stop;current_stop=0;(*num_stops)++;was_running=true;}
        } else {stop_time++;current_stop++;
            if(was_running){total_runs+=(double)current_run;current_run=0;was_running=false;}
        }
    }
    if(was_running)total_runs+=(double)current_run;else total_stops+=(double)current_stop;
    *utilization_pct=(double)run_time/(double)n*100.0;
    *mttf=(*num_stops>0)?total_runs/(double)(*num_stops)*(double)cycle_time_sec:1e9;
    *mttr=(*num_stops>0)?total_stops/(double)(*num_stops)*(double)cycle_time_sec:0.0;
    return MPC_KPI_OK;
}

/* Gain scheduling performance cross-check */
mpc_kpi_status_t mpc_kpi_gain_schedule_check(const double *cv, const double *sp,
    const double *operating_region, uint64_t n, int n_regions,
    double *per_region_rmse, double *worst_region, int *worst_region_idx)
{
    if(!cv||!sp||!operating_region||!per_region_rmse||!worst_region||!worst_region_idx)
        return MPC_KPI_ERR_NULL_POINTER;
    *worst_region=0;*worst_region_idx=0;
    double *sum_err=malloc((size_t)n_regions*sizeof(double));
    int *counts=malloc((size_t)n_regions*sizeof(int));
    if(!sum_err||!counts){free(sum_err);free(counts);return MPC_KPI_ERR_MEMORY;}
    memset(sum_err,0,(size_t)n_regions*sizeof(double));
    memset(counts,0,(size_t)n_regions*sizeof(int));
    for(uint64_t i=0;i<n;i++){
        int region=(int)operating_region[i];
        if(region>=0&&region<n_regions){
            double err=cv[i]-sp[i];
            sum_err[region]+=err*err;counts[region]++;
        }
    }
    for(int r=0;r<n_regions;r++){
        per_region_rmse[r]=(counts[r]>0)?sqrt(sum_err[r]/(double)counts[r]):0.0;
        if(per_region_rmse[r]>*worst_region){*worst_region=per_region_rmse[r];*worst_region_idx=r;}
    }
    free(sum_err);free(counts);
    return MPC_KPI_OK;
}

/* KPI seasonal decomposition (additive) */
mpc_kpi_status_t mpc_kpi_seasonal_decompose(const double *data, uint64_t n,
    int season_length, double *trend, double *seasonal, double *residual)
{
    if(!data||!trend||!seasonal||!residual) return MPC_KPI_ERR_NULL_POINTER;
    if(n<(uint64_t)(2*season_length)||season_length<2) return MPC_KPI_ERR_NOT_ENOUGH_DATA;
    /* Moving average for trend */
    int half=season_length/2;
    for(uint64_t i=0;i<n;i++){
        double sum=0;int cnt=0;
        for(int j=-half;j<=half;j++){
            int64_t idx=(int64_t)i+j;
            if(idx>=0&&idx<(int64_t)n){sum+=data[idx];cnt++;}
        }
        trend[i]=(cnt>0)?sum/(double)cnt:data[i];
    }
    /* Detrend */
    double *detrend=malloc((size_t)n*sizeof(double));
    for(uint64_t i=0;i<n;i++) detrend[i]=data[i]-trend[i];
    /* Seasonal component */
    for(int s=0;s<season_length;s++){
        double sum_s=0;int cnt_s=0;
        for(uint64_t i=(uint64_t)s;i<n;i+=season_length){sum_s+=detrend[i];cnt_s++;}
        seasonal[s]=(cnt_s>0)?sum_s/(double)cnt_s:0.0;
    }
    for(uint64_t i=0;i<n;i++) residual[i]=detrend[i]-seasonal[i%season_length];
    free(detrend);
    return MPC_KPI_OK;
}
