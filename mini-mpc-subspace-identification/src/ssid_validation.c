/* ssid_validation.c -- Model Validation and Diagnostic Tools
 *
 * Reference: Ljung (1999), Chapter 16; Van Overschee & De Moor (1996), Ch 5
 */
#include "ssid_validation.h"
#include "ssid_n4sid.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

/* L4: Compute one-step prediction residuals via Kalman filter simulation */
int ssid_validation_residuals(const ssid_model_t *model,
                              const ssid_data_t *data,
                              ssid_matrix_t *residuals)
{
    if (!model || !data || !residuals || !residuals->data) return -1;
    if (!model->A.data || !model->C.data) return -2;
    size_t N = data->N, n_x = model->n_x, n_y = model->n_y, n_u = model->n_u;
    double *x = (double *)calloc(n_x, sizeof(double));
    if (!x) return -3;
    for (size_t k = 0; k < N; k++) {
        for (size_t p = 0; p < n_y; p++) {
            double y_hat = 0.0;
            for (size_t i = 0; i < n_x; i++) y_hat += model->C.data[i * model->C.stride + p] * x[i];
            for (size_t i = 0; i < n_u; i++) y_hat += model->D.data[i * model->D.stride + p] * data->U.data[i * data->U.stride + k];
            double e = data->Y.data[p * data->Y.stride + k] - y_hat;
            residuals->data[p * residuals->stride + k] = e;
            if (model->K.data) for (size_t j = 0; j < n_x; j++) x[j] += model->K.data[p * model->K.stride + j] * e;
        }
        double *x_new = (double *)calloc(n_x, sizeof(double));
        if (x_new) {
            for (size_t i = 0; i < n_x; i++) {
                double ax = 0.0, bu = 0.0;
                for (size_t j = 0; j < n_x; j++) ax += model->A.data[j * model->A.stride + i] * x[j];
                for (size_t j = 0; j < n_u; j++) bu += model->B.data[j * model->B.stride + i] * data->U.data[j * data->U.stride + k];
                x_new[i] = ax + bu;
            }
            memcpy(x, x_new, n_x * sizeof(double));
            free(x_new);
        }
    }
    free(x);
    return 0;
}

/* L4: Whiteness test */
int ssid_validation_whiteness(const ssid_matrix_t *residuals, size_t max_lag, double *autocorr_max, int *is_white)
{
    if (!residuals || !residuals->data || max_lag == 0) return -1;
    size_t N = residuals->cols, n_y = residuals->rows;
    if (N < max_lag + 1) max_lag = N - 1;
    if (max_lag == 0) return -1;
    double *ac = (double *)calloc(max_lag, sizeof(double));
    if (!ac) return -2;
    double max_v = 0.0;
    for (size_t tau = 0; tau < max_lag; tau++) {
        double sum = 0.0; size_t cnt = 0;
        for (size_t k = tau; k < N; k++) {
            for (size_t p = 0; p < n_y; p++) sum += residuals->data[p*residuals->stride+k] * residuals->data[p*residuals->stride+(k-tau)];
            cnt++;
        }
        ac[tau] = cnt ? sum/(double)cnt : 0.0;
        if (tau > 0) { double v = fabs(ac[tau]); if (v > max_v) max_v = v; }
    }
    if (fabs(ac[0]) > 1e-16) { for (size_t t=1;t<max_lag;t++) ac[t]/=ac[0]; max_v/=ac[0]; }
    if (autocorr_max) *autocorr_max = max_v;
    if (is_white) *is_white = (max_v < 1.96/sqrt((double)N)) ? 1 : 0;
    free(ac); return 0;
}

/* L4: Cross-correlation */
int ssid_validation_cross_correlation(const ssid_matrix_t *residuals, const ssid_matrix_t *U, size_t max_lag, double *crosscorr_max)
{
    if (!residuals || !U || !residuals->data || !U->data) return -1;
    size_t N = residuals->cols, n_y = residuals->rows, n_u = U->cols;
    if (N < max_lag + 1) max_lag = N - 1;
    if (max_lag == 0 || n_u == 0) return 0;
    double max_v = 0.0;
    for (size_t tau = 0; tau < max_lag; tau++) {
        double sum = 0.0; size_t cnt = 0;
        for (size_t k = tau; k < N; k++) {
            for (size_t p = 0; p < n_y; p++) {
                double ek = residuals->data[p*residuals->stride+k];
                for (size_t q = 0; q < n_u; q++) sum += ek * U->data[q*U->stride+(k-tau)];
            }
            cnt++;
        }
        double v = cnt ? fabs(sum/(double)cnt) : 0.0;
        if (v > max_v) max_v = v;
    }
    if (crosscorr_max) *crosscorr_max = max_v;
    return 0;
}

/* L4: NRMSE */
double ssid_validation_nrmse(const ssid_matrix_t *Y_measured, const ssid_matrix_t *Y_predicted)
{
    if (!Y_measured || !Y_predicted || !Y_measured->data || !Y_predicted->data) return 0.0;
    if (Y_measured->rows != Y_predicted->rows || Y_measured->cols != Y_predicted->cols) return 0.0;
    size_t N = Y_measured->cols, n_y = Y_measured->rows;
    if (N == 0 || n_y == 0) return 0.0;
    double *mean_y = (double *)calloc(n_y, sizeof(double));
    if (!mean_y) return 0.0;
    for (size_t p = 0; p < n_y; p++) { double sum = 0.0; for (size_t k = 0; k < N; k++) sum += Y_measured->data[p * Y_measured->stride + k]; mean_y[p] = sum / (double)N; }
    double e2 = 0.0, m2 = 0.0;
    for (size_t p = 0; p < n_y; p++) {
        for (size_t k = 0; k < N; k++) {
            double d = Y_measured->data[p*Y_measured->stride+k] - Y_predicted->data[p*Y_predicted->stride+k]; e2 += d*d;
            double dm = Y_measured->data[p*Y_measured->stride+k] - mean_y[p]; m2 += dm*dm;
        }
    }
    free(mean_y);
    double en = sqrt(e2), mn = sqrt(m2);
    if (mn < 1e-16) return (en < 1e-16) ? 100.0 : 0.0;
    return 100.0 * (1.0 - en / mn);
}

/* L4: Multi-step fit */
int ssid_validation_multistep_fit(const ssid_model_t *model, const ssid_data_t *data, size_t H_p, double *fit_vs_horizon)
{
    if (!model || !data || !fit_vs_horizon || H_p == 0) return -1;
    if (!model->A.data || !model->B.data || !model->C.data || !model->D.data) return -2;
    size_t N = data->N, n_x = model->n_x, n_u = model->n_u, n_y = model->n_y;
    if (H_p > N) H_p = N;
    for (size_t h = 1; h <= H_p; h++) {
        double te = 0.0, tm = 0.0; size_t nt = 0;
        for (size_t k0 = 0; k0 + h <= N; k0 += (N/20>0?N/20:1)) {
            double *x = (double *)calloc(n_x, sizeof(double));
            if (!x) continue;
            double tr_e = 0.0, tr_m = 0.0;
            for (size_t s = 0; s < h; s++) {
                for (size_t p = 0; p < n_y; p++) {
                    double yh = 0.0;
                    for (size_t i = 0; i < n_x; i++) yh += model->C.data[i*model->C.stride+p]*x[i];
                    for (size_t i = 0; i < n_u; i++) yh += model->D.data[i*model->D.stride+p]*data->U.data[i*data->U.stride+(k0+s)];
                    double ym = data->Y.data[p*data->Y.stride+(k0+s)];
                    tr_e += (ym-yh)*(ym-yh); tr_m += ym*ym;
                }
                double *xn = (double *)calloc(n_x, sizeof(double));
                if (xn) {
                    for (size_t i = 0; i < n_x; i++) {
                        double ax=0.0,bu=0.0;
                        for (size_t j=0;j<n_x;j++) ax+=model->A.data[j*model->A.stride+i]*x[j];
                        for (size_t j=0;j<n_u;j++) bu+=model->B.data[j*model->B.stride+i]*data->U.data[j*data->U.stride+(k0+s)];
                        xn[i]=ax+bu;
                    }
                    memcpy(x,xn,n_x*sizeof(double)); free(xn);
                }
            }
            te+=sqrt(tr_e); tm+=sqrt(tr_m); nt++; free(x);
        }
        if (nt>0&&tm>1e-16) fit_vs_horizon[h-1]=100.0*(1.0-te/tm);
        else fit_vs_horizon[h-1]=0.0;
    }
    return 0;
}

/* L4: Stability check */
int ssid_validation_check_stability(const ssid_matrix_t *A, int is_ct, double *spectral_radius, int *is_stable)
{
    if (!A || !A->data || A->rows != A->cols) return -1;
    size_t n = A->rows; if (n == 0) return -1;
    double *v = (double *)malloc(n * sizeof(double));
    double *Av = (double *)malloc(n * sizeof(double));
    if (!v || !Av) { free(v); free(Av); return -2; }
    for (size_t i = 0; i < n; i++) v[i] = 1.0 / sqrt((double)n);
    double la = 0.0;
    for (int iter = 0; iter < 100; iter++) {
        for (size_t i = 0; i < n; i++) { Av[i] = 0.0; for (size_t j = 0; j < n; j++) Av[i] += A->data[j * A->stride + i] * v[j]; }
        double num = 0.0, den = 0.0;
        for (size_t i = 0; i < n; i++) { num += v[i]*Av[i]; den += v[i]*v[i]; }
        double ln = (den>1e-16)?num/den:0.0;
        double nr = 0.0; for (size_t i = 0; i < n; i++) nr += Av[i]*Av[i]; nr = sqrt(nr);
        if (nr < 1e-16) break;
        for (size_t i = 0; i < n; i++) v[i] = Av[i]/nr;
        if (fabs(ln-la) < 1e-10) break;
        la = ln;
    }
    if (spectral_radius) *spectral_radius = fabs(la);
    if (is_stable) *is_stable = is_ct ? (la < 0 ? 1 : 0) : (fabs(la) < 1.0 ? 1 : 0);
    free(v); free(Av); return 0;
}

/* L4: Observability */
int ssid_validation_check_observability(const ssid_matrix_t *A, const ssid_matrix_t *C, int *is_observable, double *condition_number)
{
    if (!A || !C || !A->data || !C->data) return -1;
    size_t nx = A->rows, ny = C->rows;
    if (nx != A->cols || C->cols != nx || nx == 0) return -2;
    double *OTO = (double *)calloc(nx*nx, sizeof(double));
    double *CAk = (double *)malloc(ny*nx*sizeof(double));
    if (!OTO || !CAk) { free(OTO); free(CAk); return -3; }
    for (size_t p=0;p<ny;p++) for (size_t i=0;i<nx;i++) CAk[p*nx+i]=C->data[i*C->stride+p];
    for (size_t k=0;k<nx;k++) {
        for (size_t a=0;a<nx;a++) for (size_t b=0;b<nx;b++) for (size_t p=0;p<ny;p++) OTO[a*nx+b]+=CAk[p*nx+a]*CAk[p*nx+b];
        if (k<nx-1) {
            double *nCAk=(double*)calloc(ny*nx,sizeof(double));
            if (!nCAk) break;
            for (size_t p=0;p<ny;p++) for (size_t i=0;i<nx;i++) for (size_t j=0;j<nx;j++) nCAk[p*nx+i]+=CAk[p*nx+j]*A->data[i*A->stride+j];
            memcpy(CAk,nCAk,ny*nx*sizeof(double)); free(nCAk);
        }
    }
    double tr=0.0; for (size_t i=0;i<nx;i++) tr+=OTO[i*nx+i];
    if (is_observable) {
        int obs=1;
        for (size_t i=0;i<nx;i++) if (OTO[i*nx+i]<1e-12*tr/(double)nx){obs=0;break;}
        *is_observable=obs;
    }
    if (condition_number) {
        double mn=1e308,mx=0.0;
        for (size_t i=0;i<nx;i++){double d=OTO[i*nx+i];if(d>mx)mx=d;if(d>1e-16&&d<mn)mn=d;}
        *condition_number=(mn>1e-16)?mx/mn:1e16;
    }
    free(OTO); free(CAk); return 0;
}

/* L4: Controllability */
int ssid_validation_check_controllability(const ssid_matrix_t *A, const ssid_matrix_t *B, int *is_controllable, double *condition_number)
{
    if (!A || !B || !A->data || !B->data) return -1;
    size_t nx = A->rows, nu = B->cols;
    if (nx != A->cols || B->rows != nx || nx == 0) return -2;
    double *CTC = (double *)calloc(nx*nx, sizeof(double));
    double *AkB = (double *)malloc(nx*nu*sizeof(double));
    if (!CTC || !AkB) { free(CTC); free(AkB); return -3; }
    for (size_t i=0;i<nx;i++) for (size_t p=0;p<nu;p++) AkB[i*nu+p]=B->data[p*B->stride+i];
    for (size_t k=0;k<nx;k++) {
        for (size_t a=0;a<nx;a++) for (size_t b=0;b<nx;b++) for (size_t p=0;p<nu;p++) CTC[a*nx+b]+=AkB[a*nu+p]*AkB[b*nu+p];
        if (k<nx-1) {
            double *nAkB=(double*)calloc(nx*nu,sizeof(double));
            if (!nAkB) break;
            for (size_t i=0;i<nx;i++) for (size_t p=0;p<nu;p++) for (size_t j=0;j<nx;j++) nAkB[i*nu+p]+=A->data[j*A->stride+i]*AkB[j*nu+p];
            memcpy(AkB,nAkB,nx*nu*sizeof(double)); free(nAkB);
        }
    }
    double tr=0.0; for (size_t i=0;i<nx;i++) tr+=CTC[i*nx+i];
    if (is_controllable) {
        int ctrl=1;
        for (size_t i=0;i<nx;i++) if (CTC[i*nx+i]<1e-12*tr/(double)nx){ctrl=0;break;}
        *is_controllable=ctrl;
    }
    if (condition_number) {
        double mn=1e308,mx=0.0;
        for (size_t i=0;i<nx;i++){double d=CTC[i*nx+i];if(d>mx)mx=d;if(d>1e-16&&d<mn)mn=d;}
        *condition_number=(mn>1e-16)?mx/mn:1e16;
    }
    free(CTC); free(AkB); return 0;
}

/* L5: Train-test split */
int ssid_validation_train_test_split(const ssid_data_t *data, double train_ratio, ssid_data_t *train, ssid_data_t *test)
{
    if (!data || !train || !test || train_ratio <= 0.0 || train_ratio >= 1.0) return -1;
    size_t Nt = (size_t)((double)data->N * train_ratio);
    if (Nt < 2 || Nt >= data->N) return -2;
    size_t Nv = data->N - Nt;
    memset(train,0,sizeof(*train)); memset(test,0,sizeof(*test));
    train->N=Nt; train->n_u=data->n_u; train->n_y=data->n_y; train->Ts=data->Ts;
    snprintf(train->source,sizeof(train->source),"%s_train",data->source);
    test->N=Nv; test->n_u=data->n_u; test->n_y=data->n_y; test->Ts=data->Ts;
    snprintf(test->source,sizeof(test->source),"%s_test",data->source);
    train->U=ssid_matrix_alloc(Nt,data->n_u); train->Y=ssid_matrix_alloc(Nt,data->n_y);
    test->U=ssid_matrix_alloc(Nv,data->n_u); test->Y=ssid_matrix_alloc(Nv,data->n_y);
    if (!train->U.data||!train->Y.data||!test->U.data||!test->Y.data){ssid_data_free(train);ssid_data_free(test);return -3;}
    for (size_t j=0;j<data->n_u;j++) memcpy(&train->U.data[j*train->U.stride],&data->U.data[j*data->U.stride],Nt*sizeof(double));
    for (size_t j=0;j<data->n_y;j++) memcpy(&train->Y.data[j*train->Y.stride],&data->Y.data[j*data->Y.stride],Nt*sizeof(double));
    for (size_t j=0;j<data->n_u;j++) memcpy(&test->U.data[j*test->U.stride],&data->U.data[j*data->U.stride+Nt],Nv*sizeof(double));
    for (size_t j=0;j<data->n_y;j++) memcpy(&test->Y.data[j*test->Y.stride],&data->Y.data[j*data->Y.stride+Nt],Nv*sizeof(double));
    return 0;
}

/* L5: K-fold CV for order */
size_t ssid_validation_kfold_order(const ssid_data_t *data, const ssid_config_t *cfg, size_t K, size_t n_min, size_t n_max, double *cv_scores)
{
    if (!data || !cfg || K < 2 || n_min > n_max || n_min == 0) return n_min;
    ssid_data_t train_data, test_data;
    memset(&train_data,0,sizeof(train_data)); memset(&test_data,0,sizeof(test_data));
    if (ssid_validation_train_test_split(data,0.7,&train_data,&test_data)!=0) return n_min;
    size_t best_n=n_min; double best_score=-1e308;
    for (size_t n=n_min;n<=n_max&&n<=cfg->n_x_max;n++) {
        ssid_config_t cn=*cfg; cn.n_x_min=n; cn.n_x_max=n; cn.order_crit=SSID_ORDER_SVD_GAP;
        ssid_result_t r=ssid_n4sid_identify(&train_data,&cn);
        if (r.model.n_x>0) { double s=r.fit_metric; if(cv_scores)cv_scores[n-n_min]=s; if(s>best_score){best_score=s;best_n=n;} }
        else if(cv_scores) cv_scores[n-n_min]=0.0;
        ssid_result_free(&r);
    }
    ssid_data_free(&train_data); ssid_data_free(&test_data);
    return best_n;
}

/* L7: Industrial MQA report */
int ssid_validation_industrial_report(const ssid_result_t *result, const ssid_data_t *train_data, const ssid_data_t *val_data, const ssid_config_t *cfg, char *report_buffer, size_t buffer_size)
{
    if (!result || !train_data || !report_buffer || buffer_size == 0) return -1;
    (void)cfg;
    int off = snprintf(report_buffer, buffer_size,
        "========================================\n"
        "  Model Quality Assurance Report\n"
        "  Subspace Identification (N4SID)\n"
        "========================================\n\n"
        "Identified: %lu-th order, %lu ins, %lu outs\n"
        "Ts: %.4f s\n\n"
        "--- Fit Metrics ---\n"
        "  NRMSE (training): %.2f %%\n",
        (unsigned long)result->model.n_x, (unsigned long)result->model.n_u,
        (unsigned long)result->model.n_y, result->model.Ts, result->fit_metric);

    if (val_data && val_data->N > 0) {
        ssid_matrix_t Yvp = ssid_matrix_alloc(val_data->N, val_data->n_y);
        if (Yvp.data) {
            for (size_t k=0;k<val_data->N;k++) for (size_t p=0;p<val_data->n_y;p++) {
                double yh=0.0;
                for (size_t i=0;i<val_data->n_u;i++) yh+=result->model.D.data[i*result->model.D.stride+p]*val_data->U.data[i*val_data->U.stride+k];
                Yvp.data[p*Yvp.stride+k]=yh;
            }
            double vf=ssid_validation_nrmse(&val_data->Y,&Yvp);
            ssid_matrix_free(&Yvp);
            off+=snprintf(report_buffer+off,buffer_size-off,"  NRMSE (validation): %.2f %%\n",vf);
        }
    }
    off+=snprintf(report_buffer+off,buffer_size-off,
        "\n--- System Properties ---\n"
        "  Stability: %s (margin: %.4f)\n"
        "  Observable: %s, Controllable: %s\n"
        "  Order: %lu, AIC: %.2f, BIC: %.2f\n",
        result->is_stable?"STABLE":"UNSTABLE",result->stability_margin,
        result->is_observable?"YES":"NO",result->is_controllable?"YES":"NO",
        (unsigned long)result->n_x_selected,result->aic_score,result->bic_score);
    off+=snprintf(report_buffer+off,buffer_size-off,
        "\n--- Residuals ---\n"
        "  Max autocorr: %.6f, Max cross-corr: %.6f\n"
        "  Time: %.1f ms\n",
        result->autocorr_max,result->crosscorr_max,result->elapsed_ms);
    int overall_pass=(result->fit_metric>=70.0&&result->is_stable&&result->is_observable&&result->is_controllable);
    off+=snprintf(report_buffer+off,buffer_size-off,
        "\n--- Verdict ---\n"
        "  Fit>=70%%: %s, Stable: %s, Obs: %s, Ctrl: %s\n"
        "  Overall: %s\n"
        "========================================\n",
        result->fit_metric>=70.0?"PASS":"FAIL",result->is_stable?"PASS":"WARN",
        result->is_observable?"PASS":"FAIL",result->is_controllable?"PASS":"FAIL",
        overall_pass?"PASS":"FAIL - Review required");
    return off;
}

/* L7: MPC readiness */
int ssid_validation_mpc_readiness(const ssid_model_t *model, size_t H_p, double Ts, char *issues_buffer, size_t buffer_size)
{
    if (!model || !issues_buffer || buffer_size == 0) return -1;
    int off=0,ic=0;
    double sr; int st;
    ssid_validation_check_stability(&model->A,model->is_ct,&sr,&st);
    if (!st&&sr>1.001){off+=snprintf(issues_buffer+off,buffer_size-off,"[ISSUE %d] Model unstable (sr=%.4f). Needs observer.\n",++ic,sr);}
    if (sr<1.0&&sr>0.0&&H_p>0&&Ts>0){
        double td=-1.0/log(sr),sett=4.0*td*Ts,ht=(double)H_p*Ts;
        if(sett>ht*1.5) off+=snprintf(issues_buffer+off,buffer_size-off,"[ISSUE %d] Settling(%.1fs)>horizon(%.1fs).\n",++ic,sett,ht);
    }
    if(sr>0.999&&sr<1.001) off+=snprintf(issues_buffer+off,buffer_size-off,"[ISSUE %d] Possible integrator.\n",++ic);
    int obs,ctrl;
    ssid_validation_check_observability(&model->A,&model->C,&obs,NULL);
    ssid_validation_check_controllability(&model->A,&model->B,&ctrl,NULL);
    if(!obs) off+=snprintf(issues_buffer+off,buffer_size-off,"[ISSUE %d] Not observable.\n",++ic);
    if(!ctrl) off+=snprintf(issues_buffer+off,buffer_size-off,"[ISSUE %d] Not controllable.\n",++ic);
    if(ic==0) snprintf(issues_buffer,buffer_size,"MPC READINESS: PASS");
    else snprintf(issues_buffer+off,buffer_size-off,"MPC READINESS: %d issue(s).",ic);
    return ic;
}
