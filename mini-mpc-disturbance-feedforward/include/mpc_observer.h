/**
 * @file mpc_observer.h
 * @brief State and Disturbance Estimation for MPC
 *
 * Implements Kalman filter, extended Kalman filter, disturbance observer,
 * and moving horizon estimation for state/disturbance reconstruction
 * in MPC applications.
 *
 * Key insight: Offset-free MPC requires estimation of both the plant
 * state and the unmeasured disturbance. The Kalman filter provides
 * optimal state estimation; the disturbance observer provides
 * disturbance estimation.
 *
 * Reference: Simon, "Optimal State Estimation" (2006)
 * Reference: Muske and Badgwell, "Disturbance Modeling for Offset-Free
 *            Linear MPC", J. Process Control (2002)
 * Reference: Pannocchia and Rawlings, "Disturbance Models for Offset-Free
 *            MPC", AIChE Journal (2003)
 *
 * Course Mappings:
 * - MIT 6.302: Kalman filtering, observer design
 * - Stanford AA272: GNSS state estimation
 * - Berkeley ME233: Stochastic estimation
 * - Georgia Tech AE 6530: Optimal estimation
 *
 * @knowledge L1: State estimator definition
 * @knowledge L2: Observer concept (Luenberger, Kalman)
 * @knowledge L3: Observer gain computation structure
 * @knowledge L4: Separation principle for LQG
 * @knowledge L5: Kalman filter algorithm
 * @knowledge L5: Disturbance observer (DOB) algorithm
 * @knowledge L5: Moving horizon estimation
 */

#ifndef MPC_OBSERVER_H
#define MPC_OBSERVER_H

#include "mpc_common.h"
#include "mpc_model.h"

/* Observer type enumeration */
typedef enum {
    MPC_OBSERVER_LUENBERGER     = 0,  /* Static Luenberger observer */
    MPC_OBSERVER_KALMAN         = 1,  /* Time-varying Kalman filter */
    MPC_OBSERVER_STEADY_KALMAN  = 2,  /* Steady-state Kalman filter */
    MPC_OBSERVER_DOB            = 3,  /* Disturbance observer */
    MPC_OBSERVER_MHE            = 4   /* Moving horizon estimator */
} mpc_observer_type_t;

/* Noise covariance matrices for Kalman filter */
typedef struct {
    double  Q[MPC_MAX_NX + MPC_MAX_ND][MPC_MAX_NX + MPC_MAX_ND]; /* Process noise */
    double  R[MPC_MAX_NY][MPC_MAX_NY];                            /* Measurement noise */
    double  P[MPC_MAX_NX + MPC_MAX_ND][MPC_MAX_NX + MPC_MAX_ND]; /* Estimation error cov */
    double  P0[MPC_MAX_NX + MPC_MAX_ND][MPC_MAX_NX + MPC_MAX_ND];/* Initial error cov */
} mpc_kalman_params_t;

/* Luenberger observer gains */
typedef struct {
    double  Lx[MPC_MAX_NX][MPC_MAX_NY];       /* State observer gain */
    double  Ld[MPC_MAX_ND][MPC_MAX_NY];       /* Disturbance observer gain */
    double  poles[MPC_MAX_NX + MPC_MAX_ND];   /* Observer pole locations */
} mpc_luenberger_gain_t;

/* Disturbance Observer (DOB) structure */
typedef struct {
    double  Q_filter_num[MPC_MAX_NX + 5];     /* Q-filter numerator coefficients */
    double  Q_filter_den[MPC_MAX_NX + 5];     /* Q-filter denominator coefficients */
    int     filter_order;                      /* Q-filter order */
    double  filter_state[MPC_MAX_NX + 5];     /* Internal filter state */
    double  inv_gain;                          /* Inverse plant DC gain */
    double  cutoff_freq;                       /* Q-filter cutoff frequency [rad/s] */
} mpc_dob_params_t;

/* Moving Horizon Estimator (MHE) configuration */
typedef struct {
    int     horizon;         /* Estimation horizon length */
    double  arrival_cost_weight[MPC_MAX_NX + MPC_MAX_ND]
                               [MPC_MAX_NX + MPC_MAX_ND];
    double  y_data[MPC_MAX_NP][MPC_MAX_NY];    /* Stored measurements */
    double  u_data[MPC_MAX_NP][MPC_MAX_NU];    /* Stored inputs */
} mpc_mhe_config_t;

/* Observer state structure */
typedef struct {
    double  x_hat[MPC_MAX_NX];                 /* Estimated plant state */
    double  d_hat[MPC_MAX_ND];                 /* Estimated disturbance */
    double  x_aug_hat[MPC_MAX_NX + MPC_MAX_ND];/* Augmented state estimate */
    double  y_hat[MPC_MAX_NY];                 /* Predicted output */
    double  innovation[MPC_MAX_NY];             /* y_meas - y_hat */
    double  innovation_cov[MPC_MAX_NY];         /* Innovation variance (diagonal) */
    double  kf_gain[MPC_MAX_NX + MPC_MAX_ND][MPC_MAX_NY]; /* Current Kalman gain */
    int     initialized;
} mpc_observer_state_t;

/* ==========================================================================
 * Kalman Filter (L5: Optimal Estimation)
 * ========================================================================== */

/* Initialize Kalman filter parameters with default values */
void mpc_kalman_init(mpc_kalman_params_t *kf, int nx_aug, int ny);

/* Kalman filter time update (prediction step)
 * x[k|k-1] = A * x[k-1|k-1] + B * u[k-1]
 * P[k|k-1] = A * P[k-1|k-1] * A^T + Q
 */
int mpc_kalman_predict(const mpc_aug_model_t *model,
                        const mpc_kalman_params_t *kf,
                        const double u[MPC_MAX_NU],
                        mpc_observer_state_t *obs);

/* Kalman filter measurement update (correction step)
 * K[k] = P[k|k-1] * C^T * (C * P[k|k-1] * C^T + R)^{-1}
 * x[k|k] = x[k|k-1] + K[k] * (y_meas[k] - C * x[k|k-1])
 * P[k|k] = (I - K[k] * C) * P[k|k-1]
 */
int mpc_kalman_update(const mpc_aug_model_t *model,
                       mpc_kalman_params_t *kf,
                       const double y_meas[MPC_MAX_NY],
                       mpc_observer_state_t *obs);

/* Compute steady-state Kalman gain by solving DARE */
int mpc_kalman_steady_gain(const mpc_aug_model_t *model,
                            const mpc_kalman_params_t *kf,
                            double K_ss[MPC_MAX_NX + MPC_MAX_ND][MPC_MAX_NY]);

/* ==========================================================================
 * Luenberger Observer (L2: Observer Design)
 * ========================================================================== */

/* Design Luenberger observer via pole placement
 * Places observer poles at specified locations.
 * For the augmented system with disturbance, places:
 *   - nx poles for state estimation
 *   - nd poles for disturbance estimation (typically slower)
 */
int mpc_luenberger_design(const mpc_aug_model_t *model,
                           const double desired_poles[MPC_MAX_NX + MPC_MAX_ND],
                           mpc_luenberger_gain_t *gain);

/* Luenberger observer update step
 * x_hat[k+1] = A * x_hat[k] + B * u[k] + L * (y[k] - C * x_hat[k])
 */
int mpc_luenberger_update(const mpc_aug_model_t *model,
                           const mpc_luenberger_gain_t *gain,
                           const double u[MPC_MAX_NU],
                           const double y_meas[MPC_MAX_NY],
                           double x_hat[MPC_MAX_NX + MPC_MAX_ND]);

/* ==========================================================================
 * Disturbance Observer - DOB (L5: Disturbance Estimation)
 * ========================================================================== */

/* Initialize DOB parameters */
void mpc_dob_init(mpc_dob_params_t *dob, double cutoff_freq, double Ts,
                  double plant_dc_gain, int filter_order);

/* DOB update: estimate disturbance from input-output data
 * d_hat = Q(s) * (P_n^{-1}(s) * y - u)
 * where Q(s) is a low-pass filter, P_n(s) is the nominal plant model
 */
int mpc_dob_update(mpc_dob_params_t *dob,
                    const mpc_ss_model_t *plant,
                    const double u[MPC_MAX_NU],
                    const double y_meas[MPC_MAX_NY],
                    double d_hat[MPC_MAX_ND],
                    double Ts);

/* Compute frequency response of DOB Q-filter */
double mpc_dob_qfilter_response(const mpc_dob_params_t *dob, double frequency);

/* ==========================================================================
 * Moving Horizon Estimation - MHE (L5: Constrained Estimation)
 * ========================================================================== */

/* Initialize MHE configuration */
void mpc_mhe_init(mpc_mhe_config_t *mhe, int horizon, int nx_aug);

/* MHE optimization step
 * Solves constrained least-squares to estimate state trajectory
 * over the estimation horizon.
 */
int mpc_mhe_estimate(const mpc_aug_model_t *model,
                      const mpc_mhe_config_t *mhe,
                      const mpc_constraints_t *constraints,
                      mpc_observer_state_t *obs);

/* Update MHE arrival cost using EKF update */
int mpc_mhe_update_arrival_cost(mpc_mhe_config_t *mhe,
                                 const mpc_aug_model_t *model);

/* ==========================================================================
 * Observer diagnostics
 * ========================================================================== */

/* Check innovation sequence whiteness (autocorrelation test) */
int mpc_observer_whiteness_test(const double innovations[], int n_innovations,
                                  int ny, double confidence_level);

/* Compute observer bandwidth from pole locations */
double mpc_observer_bandwidth(const mpc_luenberger_gain_t *gain,
                               int nx_aug, double Ts);

#endif /* MPC_OBSERVER_H */
