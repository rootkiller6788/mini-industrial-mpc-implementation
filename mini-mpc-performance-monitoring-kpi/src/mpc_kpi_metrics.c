/**
 * @file mpc_kpi_metrics.c
 * @brief MPC KPI computation algorithms.
 *
 * Implements Harris index, autocorrelation, Ljung-Box test, model-plant
 * mismatch detection, constraint monitoring, and statistical utilities.
 *
 * Knowledge: L4 Harris index, L4 Ljung-Box test, L5 Yule-Walker AR,
 * L5 Levinson-Durbin, L5 cross-correlation mismatch detection.
 *
 * Reference: Desborough & Harris (1992), Ljung & Box (1978)
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include "../include/mpc_kpi_defs.h"
#include "../include/mpc_kpi_metrics.h"

/* ---- Harris Performance Index ---- */
mpc_kpi_status_t mpc_kpi_compute_harris_index(
    const double *d, uint64_t n, int ml, mpc_kpi_harris_t *r)
{
    if(!d||!r) return MPC_KPI_ERR_NULL_POINTER;
    if(n<(uint64_t)(2*ml)||ml<2) return MPC_KPI_ERR_NOT_ENOUGH_DATA;
    memset(r,0,sizeof(mpc_kpi_harris_t));
    double sy=0,sy2=0;
    for(uint64_t i=0;i<n;i++){sy+=d[i];sy2+=d[i]*d[i];}
    double my=sy/(double)n;
    r->actual_variance=(sy2/(double)n)-(my*my);
    if(r->actual_variance<MPC_KPI_EPS){r->harris_index=1.0;r->tier=MPC_KPI_TIER_EXCELLENT;return MPC_KPI_OK;}
    double *phi=calloc(ml+1,sizeof(double)),*bp=calloc(ml+1,sizeof(double));
    if(!phi||!bp){free(phi);free(bp);return MPC_KPI_ERR_MEMORY;}
    double ba=1e30,s2; int bo=1;
    for(int p=1;p<=ml&&p<(int)(n/2);p++){
        double aic; mpc_kpi_yule_walker_ar(d,n,p,phi,&s2,&aic);
        if(aic<ba&&s2>MPC_KPI_EPS){ba=aic;bo=p;memcpy(bp,phi,(p+1)*sizeof(double));}
    }
    mpc_kpi_yule_walker_ar(d,n,bo,bp,&s2,&ba); r->min_variance_achievable=s2;
    r->harris_index=1.0-s2/r->actual_variance;
    if(r->harris_index<0)r->harris_index=0;
    if(r->harris_index>1)r->harris_index=1;
    r->variance_reduction_pct=r->harris_index*100.0;
    r->deadtime_est=(double)bo; r->num_impulse_coeffs=bo; r->converged=true;
    if(r->harris_index>=0.80)r->tier=MPC_KPI_TIER_EXCELLENT;
    else if(r->harris_index>=0.60)r->tier=MPC_KPI_TIER_GOOD;
    else if(r->harris_index>=0.40)r->tier=MPC_KPI_TIER_FAIR;
    else if(r->harris_index>=0.20)r->tier=MPC_KPI_TIER_POOR;
    else r->tier=MPC_KPI_TIER_CRITICAL;
    free(phi);free(bp);return MPC_KPI_OK;
}

mpc_kpi_status_t mpc_kpi_estimate_impulse_response(
    const double *d, uint64_t n, int nl, double *imp, int *ni)
{
    if(!d||!imp||!ni) return MPC_KPI_ERR_NULL_POINTER;
    if(n<10||nl<1) return MPC_KPI_ERR_NOT_ENOUGH_DATA;
    *ni=nl; double *phi=calloc(nl+1,sizeof(double));
    if(!phi) return MPC_KPI_ERR_MEMORY;
    double s2,aic; mpc_kpi_yule_walker_ar(d,n,nl,phi,&s2,&aic);
    imp[0]=1.0; for(int i=1;i<nl;i++) imp[i]=-phi[i];
    free(phi); return MPC_KPI_OK;
}

/* Levinson-Durbin recursion for Yule-Walker */
mpc_kpi_status_t mpc_kpi_yule_walker_ar(
    const double *d, uint64_t n, int ar, double *phi, double *s2, double *aic)
{
    if(!d||!phi||!s2) return MPC_KPI_ERR_NULL_POINTER;
    if(ar<1||n<(uint64_t)(2*ar)) return MPC_KPI_ERR_NOT_ENOUGH_DATA;
    double *r=calloc(ar+1,sizeof(double)),*pp=calloc(ar+1,sizeof(double));
    if(!r||!pp){free(r);free(pp);return MPC_KPI_ERR_MEMORY;}
    for(int k=0;k<=ar;k++){double s=0; for(uint64_t t=0;t<n-(uint64_t)k;t++) s+=d[t]*d[t+k]; r[k]=s/(double)n;}
    memset(phi,0,(ar+1)*sizeof(double)); *s2=r[0];
    for(int m=0;m<ar;m++){
        double num=r[m+1]; for(int j=0;j<m;j++) num-=phi[j]*r[m-j];
        double refl=num/(*s2); memcpy(pp,phi,(m+1)*sizeof(double));
        phi[m]=refl;
        for(int j=0;j<m;j++) phi[j]=pp[j]-refl*pp[m-1-j];
        *s2*=(1.0-refl*refl); if(*s2<MPC_KPI_EPS) *s2=MPC_KPI_EPS;
    }
    if(aic) *aic=(double)n*log(*s2)+2.0*ar;
    free(r);free(pp);return MPC_KPI_OK;
}

/* ---- Autocorrelation ---- */
mpc_kpi_status_t mpc_kpi_autocorrelation(
    const double *d, uint64_t n, int ml, mpc_kpi_autocorr_t *r)
{
    if(!d||!r) return MPC_KPI_ERR_NULL_POINTER;
    if(n<10||ml<1) return MPC_KPI_ERR_NOT_ENOUGH_DATA;
    if(ml>=(int)n) ml=(int)n-1;
    memset(r,0,sizeof(mpc_kpi_autocorr_t)); r->num_lags=ml;
    r->autocorr=calloc(ml+1,sizeof(double)); r->partial_autocorr=calloc(ml,sizeof(double));
    if(!r->autocorr||!r->partial_autocorr){free(r->autocorr);free(r->partial_autocorr);return MPC_KPI_ERR_MEMORY;}
    double sx=0; for(uint64_t i=0;i<n;i++) sx+=d[i]; double mx=sx/(double)n;
    double v=0; for(uint64_t i=0;i<n;i++){double x=d[i]-mx; v+=x*x;} v/=(double)n; if(v<MPC_KPI_EPS)v=1.0;
    for(int k=0;k<=ml;k++){double s=0; for(uint64_t t=0;t<n-(uint64_t)k;t++) s+=(d[t]-mx)*(d[t+k]-mx); r->autocorr[k]=s/((double)n*v);}
    mpc_kpi_partial_autocorrelation(r->autocorr,ml,r->partial_autocorr);
    mpc_kpi_ljung_box_from_acf(r->autocorr,n,ml,&r->ljung_box_statistic,&r->ljung_box_pvalue,&r->ljung_box_dof,&r->is_white_noise);
    r->significance_level=0.05; return MPC_KPI_OK;
}

mpc_kpi_status_t mpc_kpi_partial_autocorrelation(const double *acf, int ml, double *pacf)
{
    if(!acf||!pacf) return MPC_KPI_ERR_NULL_POINTER;
    if(ml<1) return MPC_KPI_ERR_INVALID_PARAM;
    double *phi=calloc(ml+1,sizeof(double)),*pp=calloc(ml+1,sizeof(double));
    if(!phi||!pp){free(phi);free(pp);return MPC_KPI_ERR_MEMORY;}
    double v=acf[0];
    for(int m=0;m<ml&&v>MPC_KPI_EPS;m++){
        double num=acf[m+1]; for(int j=0;j<m;j++) num-=phi[j]*acf[m-j];
        double refl=num/v; memcpy(pp,phi,(m+1)*sizeof(double));
        phi[m]=refl; for(int j=0;j<m;j++) phi[j]=pp[j]-refl*pp[m-1-j];
        pacf[m]=refl; v*=(1.0-refl*refl);
    }
    free(phi);free(pp);return MPC_KPI_OK;
}

mpc_kpi_status_t mpc_kpi_ljung_box_test(const double *res, uint64_t n, int nl, double *q, double *pv, int *dof, bool *iw)
{
    if(!res||!q||!pv||!dof||!iw) return MPC_KPI_ERR_NULL_POINTER;
    mpc_kpi_autocorr_t ac; mpc_kpi_status_t st=mpc_kpi_autocorrelation(res,n,nl,&ac);
    if(st!=MPC_KPI_OK) return st;
    *q=ac.ljung_box_statistic; *pv=ac.ljung_box_pvalue; *dof=ac.ljung_box_dof; *iw=ac.is_white_noise;
    free(ac.autocorr);free(ac.partial_autocorr);return MPC_KPI_OK;
}

mpc_kpi_status_t mpc_kpi_ljung_box_from_acf(const double *acf, uint64_t n, int nl, double *q, double *pv, int *dof, bool *iw)
{
    if(!acf||!q||!pv||!dof||!iw) return MPC_KPI_ERR_NULL_POINTER;
    *q=0.0;
    for(int k=1;k<=nl;k++){ double r2=acf[k]*acf[k]; *q+=r2/(double)((int64_t)n-(int64_t)k); }
    *q=(double)n*((double)n+2.0)*(*q); *dof=nl;
    *pv=mpc_kpi_chi2_survival(*q,nl); *iw=(*pv>0.05); return MPC_KPI_OK;
}
/* ---- Model-Plant Mismatch Detection ---- */
mpc_kpi_status_t mpc_kpi_detect_model_mismatch(
    const double *pe, const double *mv, uint64_t n, int ml, mpc_kpi_mismatch_t *r)
{
    if(!pe||!mv||!r) return MPC_KPI_ERR_NULL_POINTER;
    if(n<10||ml<1) return MPC_KPI_ERR_NOT_ENOUGH_DATA;
    memset(r,0,sizeof(mpc_kpi_mismatch_t));
    r->correlation_length=ml;
    r->correlation_function=calloc(ml,sizeof(double));
    if(!r->correlation_function) return MPC_KPI_ERR_MEMORY;
    mpc_kpi_cross_correlation(pe,mv,n,ml,r->correlation_function);
    double peak=0.0; int lag=0;
    for(int k=0;k<ml;k++){ double a=fabs(r->correlation_function[k]); if(a>peak){peak=a;lag=k;} }
    r->correlation_peak=peak; r->correlation_lag=lag;
    r->mismatch_magnitude=peak;
    r->primary_type=mpc_kpi_classify_mismatch(r->correlation_function,ml,peak,lag,&r->confidence_level);
    r->is_significant=(peak>0.3);
    r->model_update_recommended=r->is_significant;
    return MPC_KPI_OK;
}

mpc_kpi_status_t mpc_kpi_cross_correlation(const double *x, const double *y, uint64_t n, int ml, double *ccf)
{
    if(!x||!y||!ccf) return MPC_KPI_ERR_NULL_POINTER;
    if(n<2||ml<0) return MPC_KPI_ERR_NOT_ENOUGH_DATA;
    double sx=0,sy=0,sx2=0,sy2=0;
    for(uint64_t i=0;i<n;i++){sx+=x[i];sy+=y[i];sx2+=x[i]*x[i];sy2+=y[i]*y[i];}
    double mx=sx/(double)n,my=sy/(double)n;
    double vx=sx2/(double)n-mx*mx,vy=sy2/(double)n-my*my;
    double den=sqrt(vx*vy); if(den<MPC_KPI_EPS) den=1.0;
    for(int k=0;k<ml;k++){
        double s=0; for(uint64_t i=0;i<n-(uint64_t)k;i++) s+=(x[i]-mx)*(y[i+k]-my);
        ccf[k]=s/((double)(n-k)*den);
    }
    return MPC_KPI_OK;
}

mpc_mismatch_type_t mpc_kpi_classify_mismatch(const double *c, int len, double pk, int lag, double *cf)
{
    (void)cf; (void)len;
    if(!c) return MPC_MISMATCH_NONE;
    if(pk<0.2) return MPC_MISMATCH_NONE;
    if(lag==0) return MPC_MISMATCH_GAIN_BIAS;
    if(lag==1) return MPC_MISMATCH_GAIN_SLOPE;
    if(lag<=3) return MPC_MISMATCH_DEADTIME;
    return MPC_MISMATCH_TIME_CONSTANT;
}

mpc_kpi_status_t mpc_kpi_f_test_mismatch(double bv, double cv, uint64_t nb, uint64_t nc, double *fs, double *pv, bool *sig, double alpha)
{
    if(!fs||!pv||!sig) return MPC_KPI_ERR_NULL_POINTER;
    if(bv<MPC_KPI_EPS||cv<MPC_KPI_EPS) return MPC_KPI_ERR_DIVISION_BY_ZERO;
    *fs=cv/bv; *pv=1.0-mpc_kpi_chi2_cdf_approx(*fs*(double)nb/(double)nc,(int)nc);
    *sig=(*pv<alpha); return MPC_KPI_OK;
}

/* ---- Constraint Monitoring KPIs ---- */
double mpc_kpi_constraint_satisfaction_rate(const bool *vf, uint64_t n)
{
    if(!vf||n==0) return 1.0;
    uint64_t c=0; for(uint64_t i=0;i<n;i++) if(!vf[i]) c++;
    return (double)c/(double)n;
}

mpc_kpi_status_t mpc_kpi_constraint_violation_stats(
    const double *vm, uint64_t n, double *mv, double *xv, double *rv, uint64_t *cv)
{
    if(!vm||!mv||!xv||!rv||!cv) return MPC_KPI_ERR_NULL_POINTER;
    *mv=0;*xv=0;*rv=0;*cv=0;
    for(uint64_t i=0;i<n;i++){
        double v=fabs(vm[i]); if(v<MPC_KPI_EPS) continue;
        (*cv)++; *mv+=v; if(v>*xv) *xv=v; *rv+=v*v;
    }
    if(*cv>0){*mv/=(double)*cv; *rv=sqrt(*rv/(double)*cv);}
    return MPC_KPI_OK;
}

double mpc_kpi_mv_saturation_fraction(const double *vv, const double *lb, const double *ub, uint64_t n, int mi, double tol)
{
    if(!vv||!lb||!ub||n==0) return 0.0;
    uint64_t c=0;
    for(uint64_t i=0;i<n;i++){
        double v=vv[i*(uint64_t)1+(uint64_t)mi];
        if(fabs(v-lb[mi])<tol||fabs(v-ub[mi])<tol) c++;
    }
    return (double)c/(double)n;
}

/* ---- Statistical Utilities ---- */
mpc_kpi_status_t mpc_kpi_linear_trend(const double *y, uint64_t n, double *sl, double *ic, double *r2)
{
    if(!y||!sl||!ic||!r2) return MPC_KPI_ERR_NULL_POINTER;
    if(n<2) return MPC_KPI_ERR_NOT_ENOUGH_DATA;
    double sx=0,sy=0,sxy=0,sx2=0,sy2=0;
    for(uint64_t i=0;i<n;i++){ double xi=(double)i; sx+=xi;sy+=y[i];sxy+=xi*y[i];sx2+=xi*xi;sy2+=y[i]*y[i]; }
    double den=(double)n*sx2-sx*sx; if(fabs(den)<MPC_KPI_EPS) return MPC_KPI_ERR_DIVISION_BY_ZERO;
    *sl=((double)n*sxy-sx*sy)/den; *ic=(sy-(*sl)*sx)/(double)n;
    double sst=sy2-sy*sy/(double)n,sse=0;
    for(uint64_t i=0;i<n;i++){ double pred=*ic+(*sl)*(double)i; double e=y[i]-pred; sse+=e*e; }
    *r2=(sst>MPC_KPI_EPS)?1.0-sse/sst:0.0;
    return MPC_KPI_OK;
}

double mpc_kpi_mad(const double *d, uint64_t n)
{
    if(!d||n==0) return 0.0;
    double *s=malloc((size_t)n*sizeof(double)); if(!s)return 0.0;
    memcpy(s,d,(size_t)n*sizeof(double));
    for(uint64_t i=1;i<n;i++){double k=s[i];int64_t j=(int64_t)i-1;while(j>=0&&s[j]>k){s[j+1]=s[j];j--;}s[j+1]=k;}
    double med=(n%2)?s[n/2]:(s[n/2-1]+s[n/2])/2.0;
    double *a=malloc((size_t)n*sizeof(double));
    for(uint64_t i=0;i<n;i++) a[i]=fabs(d[i]-med);
    for(uint64_t i=1;i<n;i++){double k=a[i];int64_t j=(int64_t)i-1;while(j>=0&&a[j]>k){a[j+1]=a[j];j--;}a[j+1]=k;}
    double mad=(n%2)?a[n/2]:(a[n/2-1]+a[n/2])/2.0;
    free(s);free(a);return mad*1.4826;
}

mpc_kpi_status_t mpc_kpi_normality_test(const double *d, uint64_t n, double *w, double *pv, bool *in)
{
    if(!d||!w||!pv||!in) return MPC_KPI_ERR_NULL_POINTER;
    if(n<3||n>5000) return MPC_KPI_ERR_NOT_ENOUGH_DATA;
    double *s=malloc((size_t)n*sizeof(double)); if(!s) return MPC_KPI_ERR_MEMORY;
    memcpy(s,d,(size_t)n*sizeof(double));
    for(uint64_t i=1;i<n;i++){double k=s[i];int64_t j=(int64_t)i-1;while(j>=0&&s[j]>k){s[j+1]=s[j];j--;}s[j+1]=k;}
    double sm=0; for(uint64_t i=0;i<n;i++) sm+=s[i]; double mn=sm/(double)n;
    double ss=0; for(uint64_t i=0;i<n;i++){double x=s[i]-mn; ss+=x*x;}
    if(ss<MPC_KPI_EPS){*w=1.0;*pv=1.0;*in=true;free(s);return MPC_KPI_OK;}
    double num=0; int nh=(int)n/2;
    for(int i=0;i<nh;i++){
        double a=s[(int)n-1-i]-s[i];
        double coeff=(n<=50)?0.5*pow((i+1.0-0.375)/((double)n+0.25),-0.5):1.0/sqrt((double)n);
        num+=coeff*a;
    }
    *w=num*num/ss; *w=(*w>1.0)?1.0:*w;
    *pv=exp(-(*w)*0.5); *in=(*pv>0.05);
    free(s);return MPC_KPI_OK;
}

double mpc_kpi_quantile(const double *sd, uint64_t n, double p)
{
    if(!sd||n==0||p<0||p>1) return 0.0;
    double idx=p*(double)(n-1); uint64_t lo=(uint64_t)idx,hi=(uint64_t)ceil(idx);
    return sd[lo]+(sd[hi]-sd[lo])*(idx-(double)lo);
}

double mpc_kpi_economic_benefit(double bc, double cc) { return (fabs(bc)>MPC_KPI_EPS)?(bc-cc)/bc:0.0; }
double mpc_kpi_variance_reduction(double vb, double vc) { return (fabs(vb)>MPC_KPI_EPS)?(vb-vc)/vb:0.0; }
double mpc_kpi_controller_utilization(const bool *ia, uint64_t n)
{ if(!ia||n==0)return 1.0; uint64_t c=0; for(uint64_t i=0;i<n;i++)if(ia[i])c++; return (double)c/(double)n; }

double mpc_kpi_chi2_cdf_approx(double x, int dof)
{
    if(x<=0||dof<=0) return 0.0;
    double t=x/(double)dof; double z=(t>0?log(t):0.0)-(t-1.0);
    double arg=((double)dof/2.0-0.5)*z+0.5*log(2.0*3.141592653589793*dof);
    double a=1.0-exp(-(double)dof/2.0*x-arg);
    if(a<0)a=0;
    if(a>1)a=1;
    return a;
}

double mpc_kpi_chi2_survival(double x, int dof) { return 1.0-mpc_kpi_chi2_cdf_approx(x,dof); }

double mpc_kpi_covariance(const double *x, const double *y, uint64_t n)
{
    if(!x||!y||n<2) return 0.0;
    double sx=0,sy=0,sxy=0;
    for(uint64_t i=0;i<n;i++){sx+=x[i];sy+=y[i];sxy+=x[i]*y[i];}
    return sxy/(double)n-(sx/(double)n)*(sy/(double)n);
}

double mpc_kpi_correlation(const double *x, const double *y, uint64_t n)
{
    if(!x||!y||n<2) return 0.0;
    double sx=0,sy=0,sxy=0,sx2=0,sy2=0;
    for(uint64_t i=0;i<n;i++){sx+=x[i];sy+=y[i];sxy+=x[i]*y[i];sx2+=x[i]*x[i];sy2+=y[i]*y[i];}
    double vx=sx2/(double)n-(sx/(double)n)*(sx/(double)n);
    double vy=sy2/(double)n-(sy/(double)n)*(sy/(double)n);
    double cv=sxy/(double)n-(sx/(double)n)*(sy/(double)n);
    double den=sqrt(vx*vy); return (den>MPC_KPI_EPS)?cv/den:0.0;
}

mpc_kpi_status_t mpc_kpi_detect_outliers(const double *d, uint64_t n, bool *of, uint64_t *no, double k)
{
    if(!d||!of||!no) return MPC_KPI_ERR_NULL_POINTER;
    if(n<4) return MPC_KPI_ERR_NOT_ENOUGH_DATA;
    double *s=malloc((size_t)n*sizeof(double)); if(!s) return MPC_KPI_ERR_MEMORY;
    memcpy(s,d,(size_t)n*sizeof(double));
    for(uint64_t i=1;i<n;i++){double kv=s[i];int64_t j=(int64_t)i-1;while(j>=0&&s[j]>kv){s[j+1]=s[j];j--;}s[j+1]=kv;}
    double q1=mpc_kpi_quantile(s,n,0.25),q3=mpc_kpi_quantile(s,n,0.75);
    double iqr=q3-q1,lb=q1-k*iqr,ub=q3+k*iqr; *no=0;
    for(uint64_t i=0;i<n;i++){ of[i]=(d[i]<lb||d[i]>ub); if(of[i])(*no)++; }
    free(s);return MPC_KPI_OK;
}

/* =========================================================================
 * Additional Statistical Utilities
 * ========================================================================= */

/* Augmented Dickey-Fuller test for stationarity (simplified) */
mpc_kpi_status_t mpc_kpi_adf_test(const double *d, uint64_t n, double *test_stat, double *p_value, bool *is_stationary)
{
    if(!d||!test_stat||!p_value||!is_stationary) return MPC_KPI_ERR_NULL_POINTER;
    if(n<10) return MPC_KPI_ERR_NOT_ENOUGH_DATA;
    double *dy=malloc((size_t)(n-1)*sizeof(double));
    double *y_lag=malloc((size_t)(n-1)*sizeof(double));
    if(!dy||!y_lag){free(dy);free(y_lag);return MPC_KPI_ERR_MEMORY;}
    for(uint64_t i=0;i<n-1;i++){dy[i]=d[i+1]-d[i];y_lag[i]=d[i];}
    double sx=0,sy=0,sxy=0,sxx=0,syy=0;
    for(uint64_t i=0;i<n-1;i++){sx+=y_lag[i];sy+=dy[i];sxy+=y_lag[i]*dy[i];sxx+=y_lag[i]*y_lag[i];syy+=dy[i]*dy[i];}
    double mx=sx/(double)(n-1),my=sy/(double)(n-1);
    double num=sxy-(double)(n-1)*mx*my,den=sxx-(double)(n-1)*mx*mx;
    double beta=(fabs(den)>MPC_KPI_EPS)?num/den:0.0;
    double sse=0; for(uint64_t i=0;i<n-1;i++){double e=dy[i]-beta*y_lag[i];sse+=e*e;}
    double se_beta=sqrt(sse/(double)(n-3))/sqrt(den+MPC_KPI_EPS);
    *test_stat=beta/(se_beta+MPC_KPI_EPS);
    *p_value=mpc_kpi_chi2_survival(fabs(*test_stat),(int)n-2);
    *is_stationary=(*test_stat<-2.86);
    free(dy);free(y_lag);return MPC_KPI_OK;
}

/* Bootstrap confidence interval for KPI mean */
mpc_kpi_status_t mpc_kpi_bootstrap_ci(const double *d, uint64_t n, int n_bootstrap,
    double *ci_lower, double *ci_upper, double alpha)
{
    if(!d||!ci_lower||!ci_upper) return MPC_KPI_ERR_NULL_POINTER;
    if(n<10||n_bootstrap<100) return MPC_KPI_ERR_NOT_ENOUGH_DATA;
    double *means=malloc((size_t)n_bootstrap*sizeof(double));
    if(!means) return MPC_KPI_ERR_MEMORY;
    for(int b=0;b<n_bootstrap;b++){
        double sum_b=0; for(uint64_t i=0;i<n;i++) sum_b+=d[(uint64_t)rand()%n];
        means[b]=sum_b/(double)n;
    }
    for(int i=1;i<n_bootstrap;i++){double k=means[i];int j=i-1;while(j>=0&&means[j]>k){means[j+1]=means[j];j--;}means[j+1]=k;}
    int lo_idx=(int)(alpha*0.5*(double)n_bootstrap);
    int hi_idx=(int)((1.0-alpha*0.5)*(double)n_bootstrap);
    if(lo_idx<0)lo_idx=0;
    if(hi_idx>=n_bootstrap)hi_idx=n_bootstrap-1;
    *ci_lower=means[lo_idx];*ci_upper=means[hi_idx];
    free(means);return MPC_KPI_OK;
}

/* Signal-to-noise ratio for KPI quality assessment */
double mpc_kpi_signal_to_noise(const double *d, uint64_t n)
{
    if(!d||n<2) return 0.0;
    double sum=0,sum2=0;
    for(uint64_t i=0;i<n;i++){sum+=d[i];sum2+=d[i]*d[i];}
    double mean=sum/(double)n,var=sum2/(double)n-mean*mean;
    return (var>MPC_KPI_EPS)?fabs(mean)/sqrt(var):0.0;
}

/* KPI stability index (inverse of coefficient of variation) */
double mpc_kpi_stability_index(const double *d, uint64_t n)
{
    if(!d||n<2) return 0.0;
    double sum=0,sum2=0;
    for(uint64_t i=0;i<n;i++){sum+=d[i];sum2+=d[i]*d[i];}
    double mean=sum/(double)n,var=sum2/(double)n-mean*mean;
    if(mean<MPC_KPI_EPS) mean=MPC_KPI_EPS;
    return mean/sqrt(var+MPC_KPI_EPS);
}

/* CUSUM of recursive residuals for model stability */
mpc_kpi_status_t mpc_kpi_recursive_residual_cusum(const double *residuals, uint64_t n,
    int start_k, double *cusum_values, uint64_t *n_values, double *max_deviation)
{
    if(!residuals||!cusum_values||!n_values||!max_deviation) return MPC_KPI_ERR_NULL_POINTER;
    if(n<(uint64_t)start_k||start_k<2) return MPC_KPI_ERR_NOT_ENOUGH_DATA;
    double sum_r=0,sum_r2=0;
    for(int i=0;i<start_k;i++){sum_r+=residuals[i];sum_r2+=residuals[i]*residuals[i];}
    double sigma=sqrt(sum_r2/(double)start_k-(sum_r/(double)start_k)*(sum_r/(double)start_k));
    if(sigma<MPC_KPI_EPS) sigma=MPC_KPI_EPS;
    double w_sum=0,c_max=0;uint64_t cnt=0;
    for(uint64_t t=(uint64_t)start_k;t<n&&cnt<MPC_KPI_HARRIS_MAX_LAG;t++){
        w_sum+=(residuals[t]-sum_r/(double)t)/sigma;
        cusum_values[cnt]=w_sum/sqrt((double)(t-start_k+1));
        if(fabs(cusum_values[cnt])>c_max) c_max=fabs(cusum_values[cnt]);
        sum_r+=residuals[t];sum_r2+=residuals[t]*residuals[t];cnt++;
    }
    *n_values=cnt;*max_deviation=c_max;
    return MPC_KPI_OK;
}

/* Mutual information between two KPIs (discrete approximation) */
double mpc_kpi_mutual_information(const double *x, const double *y, uint64_t n, int n_bins)
{
    if(!x||!y||n<10||n_bins<2) return 0.0;
    double xmin=x[0],xmax=x[0],ymin=y[0],ymax=y[0];
    for(uint64_t i=0;i<n;i++){
        if(x[i]<xmin)xmin=x[i];
        if(x[i]>xmax)xmax=x[i];
        if(y[i]<ymin)ymin=y[i];
        if(y[i]>ymax)ymax=y[i];
    }
    double xrng=xmax-xmin+MPC_KPI_EPS,yrng=ymax-ymin+MPC_KPI_EPS;
    double *jx=malloc((size_t)(n_bins*n_bins)*sizeof(double));
    double *mx=malloc((size_t)n_bins*sizeof(double)),*my=malloc((size_t)n_bins*sizeof(double));
    if(!jx||!mx||!my){free(jx);free(mx);free(my);return 0.0;}
    memset(jx,0,(size_t)(n_bins*n_bins)*sizeof(double));
    for(uint64_t i=0;i<n;i++){
        int bx=(int)((x[i]-xmin)/xrng*(double)(n_bins-1));
        int by=(int)((y[i]-ymin)/yrng*(double)(n_bins-1));
        if(bx>=n_bins)bx=n_bins-1;
        if(by>=n_bins)by=n_bins-1;
        jx[bx*n_bins+by]+=1.0;mx[bx]+=1.0;my[by]+=1.0;
    }
    double mi=0;for(int bi=0;bi<n_bins;bi++)for(int bj=0;bj<n_bins;bj++){
        double pij=jx[bi*n_bins+bj]/(double)n;
        double pi=mx[bi]/(double)n,pj=my[bj]/(double)n;
        if(pij>0&&pi>0&&pj>0) mi+=pij*log(pij/(pi*pj));
    }
    free(jx);free(mx);free(my);return mi/log(2.0);
}

/* =========================================================================
 * Additional time series analysis utilities
 * ========================================================================= */

/* Durbin-Watson statistic for autocorrelation of residuals */
double mpc_kpi_durbin_watson(const double *residuals, uint64_t n)
{
    if(!residuals||n<2) return 2.0;
    double num=0,den=0;
    for(uint64_t i=1;i<n;i++){double d=residuals[i]-residuals[i-1];num+=d*d;}
    for(uint64_t i=0;i<n;i++) den+=residuals[i]*residuals[i];
    return (den>MPC_KPI_EPS)?num/den:2.0;
}

/* Runs test for randomness of KPI deviations */
mpc_kpi_status_t mpc_kpi_runs_test(const double *data, uint64_t n, double median,
    int *n_runs, double *z_statistic, bool *is_random)
{
    if(!data||!n_runs||!z_statistic||!is_random) return MPC_KPI_ERR_NULL_POINTER;
    if(n<10) return MPC_KPI_ERR_NOT_ENOUGH_DATA;
    int n_above=0,n_below=0;*n_runs=1;
    bool prev_above=(data[0]>=median);
    if(prev_above)n_above++;else n_below++;
    for(uint64_t i=1;i<n;i++){
        bool curr_above=(data[i]>=median);
        if(curr_above)n_above++;else n_below++;
        if(curr_above!=prev_above)(*n_runs)++;
        prev_above=curr_above;
    }
    double mu=2.0*(double)n_above*(double)n_below/(double)n+1.0;
    double sigma=sqrt(2.0*(double)n_above*(double)n_below*(2.0*(double)n_above*(double)n_below-(double)n)/
                     ((double)n*(double)n*((double)n-1.0)));
    *z_statistic=(sigma>MPC_KPI_EPS)?((double)*n_runs-mu)/sigma:0.0;
    *is_random=(fabs(*z_statistic)<1.96);
    return MPC_KPI_OK;
}

/* Theil-Sen robust trend estimator */
mpc_kpi_status_t mpc_kpi_theil_sen_trend(const double *y, uint64_t n,
    double *slope, double *intercept)
{
    if(!y||!slope||!intercept) return MPC_KPI_ERR_NULL_POINTER;
    if(n<3) return MPC_KPI_ERR_NOT_ENOUGH_DATA;
    uint64_t npairs=n*(n-1)/2;
    double *slopes=malloc((size_t)npairs*sizeof(double));
    if(!slopes) return MPC_KPI_ERR_MEMORY;
    uint64_t idx=0;
    for(uint64_t i=0;i<n;i++) for(uint64_t j=i+1;j<n;j++) slopes[idx++]=(y[j]-y[i])/(double)(j-i);
    for(uint64_t i=1;i<npairs;i++){double k=slopes[i];int64_t jj=(int64_t)i-1;while(jj>=0&&slopes[jj]>k){slopes[jj+1]=slopes[jj];jj--;}slopes[jj+1]=k;}
    *slope=(npairs%2)?slopes[npairs/2]:(slopes[npairs/2-1]+slopes[npairs/2])/2.0;
    double sum_y=0;for(uint64_t i=0;i<n;i++) sum_y+=y[i];
    *intercept=sum_y/(double)n-(*slope)*(double)(n-1)/2.0;
    free(slopes);
    return MPC_KPI_OK;
}

/* Granger causality test between two KPIs (simplified) */
mpc_kpi_status_t mpc_kpi_granger_causality(const double *x, const double *y, uint64_t n,
    int max_lag, double *f_statistic, bool *x_causes_y)
{
    if(!x||!y||!f_statistic||!x_causes_y) return MPC_KPI_ERR_NULL_POINTER;
    if(n<(uint64_t)(2*max_lag+5)) return MPC_KPI_ERR_NOT_ENOUGH_DATA;
    /* Restricted model: y_t = a0 + a1*y_{t-1} + ... + e_t */
    double sse_r=0; uint64_t cnt=0;
    for(uint64_t t=(uint64_t)max_lag;t<n;t++){
        double yp=0; for(int k=1;k<=max_lag;k++) yp+=0.5*y[t-(uint64_t)k];
        double e=y[t]-yp; sse_r+=e*e; cnt++;
    }
    /* Unrestricted model: adds x lags */
    double sse_u=0;
    for(uint64_t t=(uint64_t)max_lag;t<n;t++){
        double yp=0; for(int k=1;k<=max_lag;k++){yp+=0.5*y[t-(uint64_t)k];yp+=0.3*x[t-(uint64_t)k];}
        double e=y[t]-yp; sse_u+=e*e;
    }
    double num=(sse_r-sse_u)/(double)max_lag;
    double den=sse_u/(double)(cnt-(uint64_t)(2*max_lag));
    *f_statistic=(den>MPC_KPI_EPS)?num/den:0.0;
    *x_causes_y=(*f_statistic>2.5);
    return MPC_KPI_OK;
}
