/**
 * @file example_harris_index.c
 * @brief Harris Performance Index computation for a distillation column.
 *
 * L6 canonical problem: A distillation column temperature control loop
 * is evaluated using the minimum variance benchmark. The Harris index
 * quantifies how close the current controller is to optimal performance.
 *
 * Demonstrates: Harris index, impulse response estimation, Yule-Walker AR
 * modeling, autocorrelation analysis, and residual whiteness testing.
 *
 * Reference: Desborough & Harris (1992), Can. J. Chem. Eng.
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "../include/mpc_kpi_defs.h"
#include "../include/mpc_kpi_metrics.h"

static void analyze_loop(const char *name, const double *data, int n, int max_lag)
{
    printf("\n--- %s (n=%d) ---\n", name, n);

    /* Harris index */
    mpc_kpi_harris_t hr;
    mpc_kpi_status_t st=mpc_kpi_compute_harris_index(data,(uint64_t)n,max_lag,&hr);
    if(st!=MPC_KPI_OK){printf("  Error: %s\n",mpc_kpi_status_string(st));return;}

    printf("  Harris Index:        %.4f\n",hr.harris_index);
    printf("  Actual variance:     %.6f\n",hr.actual_variance);
    printf("  Min achievable var:  %.6f\n",hr.min_variance_achievable);
    printf("  Variance reduction:  %.1f%%\n",hr.variance_reduction_pct);
    printf("  Performance tier:    %s\n",mpc_kpi_tier_string(hr.tier));

    /* Impulse response estimate */
    double imp[20]; int n_imp;
    mpc_kpi_estimate_impulse_response(data,(uint64_t)n,max_lag,imp,&n_imp);
    printf("  Impulse response:    [");
    for(int i=0;i<(n_imp<8?n_imp:8);i++) printf("%.3f%s",imp[i],(i<(n_imp<8?n_imp:8)-1)?", ":"]\n");

    /* Autocorrelation check */
    mpc_kpi_autocorr_t ac;
    mpc_kpi_autocorrelation(data,(uint64_t)n,max_lag,&ac);
    printf("  Autocorr (lag 1-5):  [%.3f, %.3f, %.3f, %.3f, %.3f]\n",
           ac.autocorr[1],ac.autocorr[2],ac.autocorr[3],ac.autocorr[4],ac.autocorr[5]);
    printf("  Ljung-Box Q:         %.2f (df=%d, p=%.4f, white=%s)\n",
           ac.ljung_box_statistic,ac.ljung_box_dof,ac.ljung_box_pvalue,
           ac.is_white_noise?"YES":"no");

    /* Interpretation */
    printf("  Assessment:          ");
    if(hr.harris_index>=0.80) printf("EXCELLENT - near minimum variance\n");
    else if(hr.harris_index>=0.60) printf("GOOD - minor retuning may help\n");
    else if(hr.harris_index>=0.40) printf("FAIR - significant room for improvement\n");
    else printf("POOR - major controller retuning required\n");

    free(hr.impulse_response_coeffs);
    free(ac.autocorr); free(ac.partial_autocorr);
}

int main(void)
{
    printf("=== Harris Performance Index: Distillation Column ===\n");

    /* Scenario 1: Well-tuned controller (AR(2) with small deadtime) */
    {
        printf("\nScenario 1: Well-tuned temperature loop\n");
        double cv_data[500], innov[500];
        for(int i=0;i<500;i++) innov[i]=0.1*((double)rand()/RAND_MAX-0.5);
        cv_data[0]=innov[0];
        cv_data[1]=0.5*cv_data[0]+innov[1];
        for(int i=2;i<500;i++) cv_data[i]=0.5*cv_data[i-1]-0.2*cv_data[i-2]+innov[i];
        analyze_loop("Well-tuned", cv_data, 500, 10);
    }

    /* Scenario 2: Poorly tuned controller (AR(4) with longer memory) */
    {
        printf("\nScenario 2: Poorly tuned composition loop\n");
        double cv_data[500], innov[500];
        for(int i=0;i<500;i++) innov[i]=0.2*((double)rand()/RAND_MAX-0.5);
        cv_data[0]=innov[0];
        cv_data[1]=0.9*cv_data[0]+innov[1];
        cv_data[2]=0.8*cv_data[1]-0.3*cv_data[0]+innov[2];
        cv_data[3]=0.7*cv_data[2]-0.2*cv_data[1]+0.1*cv_data[0]+innov[3];
        for(int i=4;i<500;i++)
            cv_data[i]=0.6*cv_data[i-1]-0.1*cv_data[i-2]+0.05*cv_data[i-3]-0.02*cv_data[i-4]+innov[i];
        analyze_loop("Poorly-tuned", cv_data, 500, 15);
    }

    /* Scenario 3: Strong disturbance (AR(1) + external oscillation) */
    {
        printf("\nScenario 3: Disturbance-affected pressure loop\n");
        double cv_data[500];
        cv_data[0]=0.1*((double)rand()/RAND_MAX-0.5);
        for(int i=1;i<500;i++)
            cv_data[i]=0.3*cv_data[i-1]+0.5*sin(i*0.2)+0.1*((double)rand()/RAND_MAX-0.5);
        analyze_loop("Disturbed", cv_data, 500, 10);
    }

    printf("\n=== Summary ===\n");
    printf("The Harris index provides a theoretical lower bound on achievable\n");
    printf("output variance. Values near 0.0 indicate the controller is far\n");
    printf("from optimal; values near 1.0 indicate near-optimal performance.\n");
    printf("\nExample complete.\n");
    return 0;
}

