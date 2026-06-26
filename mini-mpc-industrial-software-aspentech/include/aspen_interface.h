/** @file aspen_interface.h
 * @brief AspenTech DMC3 Interface (L7)
 */
#ifndef ASPEN_INTERFACE_H
#define ASPEN_INTERFACE_H
#include "mpc_common.h"
#ifdef __cplusplus
extern "C" {
#endif

int mpc_dmc3_read_inputs(mpc_aspen_config_t *cfg, double *cv_values, double *dv_values, double *mv_feedback);
int mpc_dmc3_write_outputs(mpc_aspen_config_t *cfg, const double *mv_output);
int mpc_dmc3_simulate_process(mpc_aspen_config_t *cfg, double *cv_noisy, double noise_std);
int mpc_dmc3_closed_loop_simulation(mpc_aspen_config_t *cfg, int n_steps, double noise_std, double *cv_history, double *mv_history);
double mpc_scale_cv_eu_to_pct(const mpc_cv_config_t *cv, double eu_val);
double mpc_scale_cv_pct_to_eu(const mpc_cv_config_t *cv, double pct_val);
int mpc_validate_eu_config(double eu0, double eu100);

#ifdef __cplusplus
}
#endif
#endif
