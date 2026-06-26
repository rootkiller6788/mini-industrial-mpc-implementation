/** @file variable_scaling.c
 * @brief Engineering Unit Scaling (L1, ISA-101 HMI Convention)
 * Converts between percent (0-100) and engineering units (EU).
 * Used for operator display and DCS interface.
 * Formula: eu = eu0 + (pct/100) * (eu100 - eu0)
 */
#include <stdlib.h>
#include <math.h>
#include "mpc_common.h"

double mpc_pct_to_eu(double pct, double eu0, double eu100)
{ return eu0 + (pct / 100.0) * (eu100 - eu0); }

double mpc_eu_to_pct(double eu, double eu0, double eu100)
{
    double span = eu100 - eu0;
    if (fabs(span) < MPC_EPS) return 0.0;
    return ((eu - eu0) / span) * 100.0;
}

double mpc_scale_mv(const mpc_mv_config_t *mv, double pct_val)
{
    if (!mv) return 0.0;
    return mpc_pct_to_eu(pct_val, mv->eu0, mv->eu100);
}

double mpc_unscale_mv(const mpc_mv_config_t *mv, double eu_val)
{
    if (!mv) return 0.0;
    return mpc_eu_to_pct(eu_val, mv->eu0, mv->eu100);
}

/* CV scaling helpers */
double mpc_scale_cv_eu_to_pct(const mpc_cv_config_t *cv, double eu_val)
{
    if (!cv) return 0.0;
    return mpc_eu_to_pct(eu_val, cv->eu0, cv->eu100);
}

double mpc_scale_cv_pct_to_eu(const mpc_cv_config_t *cv, double pct_val)
{
    if (!cv) return 0.0;
    return mpc_pct_to_eu(pct_val, cv->eu0, cv->eu100);
}

/* Validate EU configuration */
int mpc_validate_eu_config(double eu0, double eu100)
{
    if (!isfinite(eu0) || !isfinite(eu100)) return -1;
    double span = eu100 - eu0;
    if (fabs(span) < MPC_EPS) return -2;
    return 0;
}
