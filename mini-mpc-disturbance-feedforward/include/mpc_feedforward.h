/**
 * @file mpc_feedforward.h
 * @brief Disturbance Feedforward Control for MPC
 *
 * Implements the feedforward component of the combined MPC + FF controller.
 * The feedforward path compensates for measurable disturbances before they
 * affect the process output, while MPC handles feedback regulation.
 *
 * Combined control law: u[k] = u_mpc[k] + u_ff[k]
 *
 * Feedforward design methods:
 *   1. Static feedforward: u_ff = -Kd * d_meas  (steady-state compensation)
 *   2. Dynamic feedforward: u_ff = -G_ff(s) * d_meas  (transient compensation)
 *   3. Preview feedforward: uses future disturbance trajectory in MPC
 *
 * Reference: Brosilow and Joseph, "Inferential Control" (1978)
 * Reference: Seborg, Edgar, Mellichamp, "Process Dynamics and Control" (2016)
 * Reference: Guzman and Hagglund, "Feedforward Control" (2020)
 *
 * Course Mappings:
 * - MIT 6.302: Feedforward compensation
 * - Stanford ENGR205: Combined FF/FB control
 * - CMU 24-677: Disturbance rejection strategies
 * - Purdue ME 575: Industrial feedforward design
 *
 * @knowledge L1: Feedforward control definition
 * @knowledge L2: Feedback vs feedforward comparison
 * @knowledge L2: Measured disturbance compensation
 * @knowledge L5: Static feedforward design
 * @knowledge L5: Dynamic feedforward design
 * @knowledge L5: Preview-based disturbance feedforward
 */

#ifndef MPC_FEEDFORWARD_H
#define MPC_FEEDFORWARD_H

#include "mpc_common.h"
#include "mpc_model.h"
#include "mpc_observer.h"

/* Feedforward design method */
typedef enum {
    FF_STATIC_GAIN       = 0,  /* Kd = -inv(C*inv(I-A)*B) * (C*inv(I-A)*Bd) */
    FF_DYNAMIC_INVERSION = 1,  /* Gff(s) = -Gp^{-1}(s) * Gd(s) */
    FF_PREVIEW_MPC       = 2,  /* Include future d in MPC prediction */
    FF_ADAPTIVE_GAIN     = 3   /* Online adaptation of feedforward gain */
} mpc_ff_method_t;

/* Feedforward controller configuration */
typedef struct {
    mpc_ff_method_t method;
    double  Kd[MPC_MAX_NU][MPC_MAX_ND];    /* Static feedforward gain matrix */
    double  ff_gain_scale;                  /* Manual scaling factor (0..2) */

    /* Dynamic feedforward (lead-lag filter per I/O pair) */
    int     enable_dynamic_ff;
    double  lead_time[MPC_MAX_NU][MPC_MAX_ND];   /* Lead time constant [s] */
    double  lag_time[MPC_MAX_NU][MPC_MAX_ND];    /* Lag time constant [s] */
    double  ff_deadtime[MPC_MAX_NU][MPC_MAX_ND]; /* Feedforward deadtime [s] */

    /* Filter state for dynamic FF */
    double  ff_state[MPC_MAX_NU * MPC_MAX_ND * 4];

    /* Preview feedforward */
    int     preview_horizon;
    double  d_preview[MPC_MAX_NP][MPC_MAX_ND];   /* Future disturbance preview */

    /* Adaptive gain */
    int     enable_adaptation;
    double  adapt_rate;
    double  adapt_deadzone;
} mpc_ff_config_t;

/* Industrial feedforward structure (ISA-5.1 compliant) */
typedef struct {
    double  bias;                            /* Feedforward bias/offset */
    double  output_limit_high;               /* FF output upper limit */
    double  output_limit_low;                /* FF output lower limit */
    int     enable_ratio;                    /* Ratio control mode */
    double  ratio_value;                     /* Ratio setpoint */
    double  ratio_var;                       /* Ratio variable measurement */
} mpc_ff_industrial_t;

/* ==========================================================================
 * Static Feedforward Design
 * ========================================================================== */

/* Compute ideal static feedforward gain matrix
 * Kd_ideal = -(C*(I-A)^{-1}*B)^{-1} * C*(I-A)^{-1}*Bd
 *
 * This achieves perfect steady-state disturbance rejection.
 * Theorem: For a stable LTI plant with invertible steady-state gain
 * from u to y, the above Kd eliminates the effect of d on y at DC.
 */
int mpc_ff_compute_static_gain(const mpc_ss_model_t *plant,
                                double Kd[MPC_MAX_NU][MPC_MAX_ND]);

/* Compute static feedforward input for current disturbance
 * u_ff = -Kd * d_meas
 */
int mpc_ff_compute_static(const mpc_ff_config_t *ff,
                           const double d_meas[MPC_MAX_ND],
                           double u_ff[MPC_MAX_NU]);

/* ==========================================================================
 * Dynamic Feedforward Design
 * ========================================================================== */

/* Design dynamic feedforward compensator
 * Gff(s) = -Gp_inv(s) * Gd(s)
 *
 * Implements lead-lag compensation:
 * Gff(s) = Kd * (T_lead*s + 1) / (T_lag*s + 1)
 *
 * Valid only for minimum-phase plants (Gp^{-1} must be stable).
 * For non-minimum-phase plants, approximate inversion is used.
 */
int mpc_ff_design_dynamic(const mpc_ss_model_t *plant,
                           const mpc_tuning_t *tuning,
                           mpc_ff_config_t *ff);

/* Update dynamic feedforward filter state and compute output
 * Implements discretized lead-lag filter using Tustin transform
 */
int mpc_ff_update_dynamic(mpc_ff_config_t *ff,
                           const double d_meas[MPC_MAX_ND],
                           const double d_prev[MPC_MAX_ND],
                           double u_ff[MPC_MAX_NU],
                           double Ts);

/* ==========================================================================
 * Preview Feedforward (L5: Anticipatory Control)
 * ========================================================================== */

/* Set disturbance preview trajectory */
void mpc_ff_set_preview(mpc_ff_config_t *ff,
                         const double d_trajectory[MPC_MAX_NP][MPC_MAX_ND],
                         int horizon);

/* Compute feedforward-augmented prediction
 * Modifies the predicted output to include disturbance effects:
 * Y_ff = Gamma_d * DeltaU_d_preview + Phi_d * d[k]
 */
int mpc_ff_augment_prediction(const mpc_prediction_t *pred,
                               const mpc_ff_config_t *ff,
                               const double d_current[MPC_MAX_ND],
                               double Y_ff[MPC_MAX_NP * MPC_MAX_NY]);

/* ==========================================================================
 * Adaptive Feedforward
 * ========================================================================== */

/* Initialize adaptive feedforward parameters */
void mpc_ff_adaptive_init(mpc_ff_config_t *ff, double adapt_rate, double deadzone);

/* Update adaptive feedforward gain using gradient descent
 * dKd/dt = -gamma * y * d^T   (LMS-like update)
 * where y is the output error and d is the disturbance measurement
 */
int mpc_ff_adaptive_update(mpc_ff_config_t *ff,
                            const double d_meas[MPC_MAX_ND],
                            const double y_error[MPC_MAX_NY],
                            double Ts);

/* ==========================================================================
 * Combined Feedforward + Feedback Integration
 * ========================================================================== */

/* Integrate FF and FB control signals with anti-windup
 * u_total = saturate(u_fb + u_ff, u_min, u_max)
 * Applies incremental anti-windup: if u_total saturates,
 * the FF component is not back-calculated (it is disturbance-driven),
 * only the FB component is limited.
 */
void mpc_ff_combine(const double u_fb[MPC_MAX_NU],
                     const double u_ff[MPC_MAX_NU],
                     const double u_min[MPC_MAX_NU],
                     const double u_max[MPC_MAX_NU],
                     double u_total[MPC_MAX_NU],
                     int nu);

/* Industrial feedforward initialization with ISA-5.1 parameters */
void mpc_ff_industrial_init(mpc_ff_industrial_t *iff,
                             double bias, double out_high, double out_low);

/* Apply industrial feedforward with ratio and bias */
double mpc_ff_industrial_compute(const mpc_ff_industrial_t *iff,
                                  double disturbance,
                                  double base_signal);

/* ==========================================================================
 * Feedforward Performance Metrics
 * ========================================================================== */

/* Compute feedforward contribution ratio
 * FF_ratio = ||u_ff|| / (||u_ff|| + ||u_fb||)
 * Indicates what fraction of control effort comes from feedforward
 */
double mpc_ff_contribution_ratio(const double u_fb[MPC_MAX_NU],
                                  const double u_ff[MPC_MAX_NU],
                                  int nu);

/* Compute disturbance rejection improvement
 * I_dr = 1 - (var(y_with_ff) / var(y_without_ff))
 * Returns improvement factor (0 = no improvement, 1 = perfect rejection)
 */
double mpc_ff_rejection_improvement(const double y_errors_with_ff[],
                                     const double y_errors_without_ff[],
                                     int n_samples, int ny);

/* Detect feedforward-feedback fighting (cancelling control actions) */
int mpc_ff_detect_fighting(const double u_fb[MPC_MAX_NU],
                            const double u_ff[MPC_MAX_NU],
                            int nu, double threshold);

#endif /* MPC_FEEDFORWARD_H */
