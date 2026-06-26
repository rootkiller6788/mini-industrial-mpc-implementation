/** @file mpc_model_id.h
 * @brief System Identification for MPC Models (L5, L7)
 *
 * L5: Step test identification, subspace identification (N4SID)
 * L7: AspenTech DMC3 SmartStep - automated step testing for refinery models
 *
 * Ref: Ljung (1999) System Identification, Ch.10
 *      Van Overschee & De Moor (1996) Subspace Identification
 */
#ifndef MPC_MODEL_ID_H
#define MPC_MODEL_ID_H
#include "mpc_common.h"
#ifdef __cplusplus
extern "C" {
#endif

/* Step test configuration (AspenTech SmartStep equivalent) */
typedef struct {
    int      mv_index;
    double   step_size;
    double   step_duration_sec;
    int      n_steps;
    double   settling_time_sec;
    double   noise_threshold;
    int      is_complete;
} mpc_step_test_config_t;

/* Subspace identification data buffer */
typedef struct {
    int      n_inputs, n_outputs;
    int      max_samples;
    int      n_stored;
    double  *U;
    double  *Y;
    int      block_rows;
} mpc_n4sid_data_t;

/* Identified model result */
typedef struct {
    int      order;
    double  *A, *B, *C, *D;
    double  *K;
    double   fit_percent;
    double   aic_criterion;
} mpc_identified_model_t;

int  mpc_step_test_init(mpc_step_test_config_t *cfg, int mv_index, double step_size, double duration);
void mpc_step_test_free(mpc_step_test_config_t *cfg);
int  mpc_step_test_execute(mpc_step_test_config_t *cfg, mpc_step_model_t **models, int n_cv);

int  mpc_n4sid_init(mpc_n4sid_data_t *data, int n_inputs, int n_outputs, int max_samples, int block_rows);
void mpc_n4sid_free(mpc_n4sid_data_t *data);
int  mpc_n4sid_add_sample(mpc_n4sid_data_t *data, const double *u, const double *y);
int  mpc_n4sid_identify(mpc_n4sid_data_t *data, int order, mpc_identified_model_t *model);

int  mpc_identified_model_alloc(mpc_identified_model_t *m, int nx, int nu, int ny);
void mpc_identified_model_free(mpc_identified_model_t *m);
int  mpc_model_to_dmc(const mpc_identified_model_t *id_model, mpc_mimo_model_t *dmc_model, double sample_time);

#ifdef __cplusplus
}
#endif
#endif
