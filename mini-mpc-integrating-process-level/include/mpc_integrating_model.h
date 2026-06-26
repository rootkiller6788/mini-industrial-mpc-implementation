/**
 * mpc_integrating_model.h — Integrating Process Models for MPC
 *
 * Domain: Industrial Model Predictive Control — Process Modeling
 * Reference: Rawlings, Mayne, Diehl (2017) "Model Predictive Control" §2
 *            Seborg, Edgar, Mellichamp (2016) "Process Dynamics and Control" §4-7
 *            Ljung (1999) "System Identification" §4-5
 *
 * Provides model construction for integrating (non-self-regulating) processes
 * in forms suitable for MPC: continuous-time transfer functions, discrete-time
 * state-space realizations, step response models, and CARIMA representations.
 *
 * An integrating process is characterized by:
 *   lim_{s→0} s * G(s) = K ≠ 0   [non-zero velocity gain]
 *   No steady-state under constant input → ramp output
 *
 * Knowledge Coverage:
 *   L2 - Core Concepts: integrating process dynamics, discretization, ZOH
 *   L3 - Eng. Structures: state-space, CARIMA, step response models
 *   L5 - Algorithms: model conversion, discretization
 *
 * MIT 6.302 · Stanford ENGR205 · Berkeley ME233 · CMU 24-677
 */

#ifndef MPC_INTEGRATING_MODEL_H
#define MPC_INTEGRATING_MODEL_H

#include "mpc_level_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ─── Transfer Function Construction ──────────────────────────────────── */

/**
 * mpc_model_from_lag — Build integrating process with lag
 *
 * Constructs G(s) = K / [s * (τ*s + 1)]
 *
 * This is the most common industrial integrating process.
 * Examples:
 *   - Tank level: K = 1/(ρ*A), τ models actuator/sensor dynamics
 *   - Steam drum level: K models inverse response (shrink/swell)
 *   - Distillation column reboiler level
 *
 * @param model  Output model struct (populated by function)
 * @param gain   K, process velocity gain [(m/s)/%valve]
 * @param tau    τ, first-order lag time constant [s]
 * @param dt     Sampling time T_s [s]
 * @param area   Tank area [m²] (0.0 if not a tank)
 * Returns 0 on success, -1 on invalid parameters.
 *
 * Theorem (ZOH Discretization):
 *   Discrete A matrix for ġ = [0 1; 0 -1/τ] * x + [K/τ; K/τ] * u
 *   A_d = exp(A*T_s), B_d = ∫₀^Ts exp(A*s)*B ds
 *   Explicit: a₁₁=1, a₁₂=τ*(1-exp(-Ts/τ)), a₂₂=exp(-Ts/τ)
 */
int mpc_model_from_lag(integrating_process_t *model, double gain,
                        double tau, double dt, double area);

/**
 * mpc_model_pure_integrator — Build pure integrator model
 *
 * G(s) = K / s
 *
 * Simplest integrating process. Discrete-time state-space:
 *   x(k+1) = x(k) + K*T_s * u(k)
 *   y(k)   = x(k)
 *
 * The discrete pole is at z=1 (unit circle boundary).
 * Impulse response = step (unbounded).
 *
 * @param model  Output model struct
 * @param gain   K, integrator gain
 * @param dt     T_s, sampling time [s]
 * @param area   Tank area [m²] (if applicable)
 */
int mpc_model_pure_integrator(integrating_process_t *model,
                               double gain, double dt, double area);

/**
 * mpc_model_integrating_second_order — Integrator + two lags
 *
 * G(s) = K / [s * (τ₁*s + 1) * (τ₂*s + 1)]
 *
 * Used when actuator dynamics cannot be neglected.
 * Applied to large valves with significant stroking time.
 * State dimension = 3.
 */
int mpc_model_integrating_second_order(integrating_process_t *model,
    double gain, double tau1, double tau2, double dt);

/* ─── State-Space Conversion ──────────────────────────────────────────── */

/**
 * mpc_model_to_state_space — Convert integrating process to discrete SS
 *
 * Controllable canonical form for G(s) = K / [s * (τ*s + 1)]:
 *
 *   x(k+1) = [1  τ*(1-α); 0  α] * x(k) + [K*(Ts-τ*(1-α)); K*(1-α)] * u(k)
 *   y(k)   = [1  0] * x(k)
 *   where α = exp(-Ts/τ)
 *
 * For pure integrator (τ→0):
 *   x(k+1) = x(k) + K*Ts*u(k)
 *   y(k) = x(k)
 *
 * @param model   Integrating process model (input)
 * @param ss      State-space structure (output, pre-allocated)
 * @param add_dist  If 1, augment with integrating output disturbance
 * Returns 0 on success.
 *
 * Disturbance augmentation (Muske & Badgwell 2002):
 *   For offset-free control, add d(k+1) = d(k) and y(k) = C*x(k) + d(k)
 *   giving augmented system with n+1 states.
 */
int mpc_model_to_state_space(const integrating_process_t *model,
                              integrating_state_t *ss, int add_dist);

/**
 * mpc_carima_from_ss — Convert discrete state-space to CARIMA model
 *
 * CARIMA: A(z⁻¹)*y(k) = z^{-d}*B(z⁻¹)*u(k-1) + C(z⁻¹)*ξ(k)/Δ
 *
 * Algorithm: Compute transfer function G(z) = C*(zI-A)⁻¹*B using
 * Leverrier-Faddeev method, then apply Δ = 1-z⁻¹ differencing.
 *
 * @param ss     Discrete state-space model (input)
 * @param gpc    GPC configuration structure (output)
 * Returns order of A polynomial on success, -1 on error.
 */
int mpc_carima_from_ss(const integrating_state_t *ss, gpc_config_t *gpc);

/**
 * mpc_carima_integrating_default — Build CARIMA for integrating process
 *
 * For a pure tank: A(z) = 1 - 2z⁻¹ + z⁻² = (1-z⁻¹)²
 *                   B(z) = -(T_s/A) * z⁻¹
 *
 * This is the integrating ARX (ARIX) form.
 * The (1-z⁻¹) factor in A provides integral action natively.
 *
 * @param gpc   Output GPC config
 * @param gain  K (velocity gain)
 * @param dt    T_s sampling time
 * @param d     delay in samples
 * Returns 0 on success.
 */
int mpc_carima_integrating_default(gpc_config_t *gpc, double gain,
                                    double dt, int d);

/* ─── Step Response Model Construction ────────────────────────────────── */

/**
 * mpc_step_response_from_model — Generate step response for DMC
 *
 * Computes s_i for i = 0, 1, ..., N-1 from the integrating process model.
 *
 * Method: Simulate the discrete state-space model with u(k) = 1 for all k.
 * For integrating process, s_i grows without bound; truncation at N.
 *
 * The DMC prediction assumes:
 *   y(k+j|k) = y_base + Σ_{i=1}^{j} s_i * Δu(k+j-i) + Σ_{i=j+1}^{N} s_i * Δu(k+j-i)
 *
 * Alternative form (commonly used):
 *   y(k+j|k) = y_base + Σ_{i=1}^{N} s_i * Δu(k+j-i)
 *   with Δu(k+j-i)=0 for j-i ≥ N_c (beyond control horizon)
 *
 * Step response truncation error bound:
 *   For integrating process, |s_N - s_{N-1}| ≈ K*Ts, linear growth.
 *   Choose N s.t. N*Ts ≫ τ (at least 5*τ beyond delay)
 *
 * @param step   Output step response struct
 * @param model  Integrating process model
 * @param N      Truncation length (≤ MPC_MAX_STEP_HORIZON)
 * Returns N on success, -1 on error.
 */
int mpc_step_response_from_model(step_response_t *step,
                                  const integrating_process_t *model, int N);

/**
 * mpc_step_response_ramp — Analytic step response for pure integrator
 *
 * For G(s) = K/s:
 *   s_i = K * Ts * i   (i = 1, 2, ..., N)
 *
 * For G(s) = K/[s*(τ*s+1)]:
 *   s_i = K * [i*Ts - τ*(1 - exp(-i*Ts/τ))]
 *
 * The analytic form avoids numerical integration errors.
 *
 * @param step   Output step response
 * @param gain   K
 * @param tau    τ (0.0 for pure integrator)
 * @param dt     Ts
 * @param N      number of coefficients
 * Returns N on success.
 */
int mpc_step_response_ramp(step_response_t *step, double gain, double tau,
                            double dt, int N);

/* ─── Discretization Utilities ────────────────────────────────────────── */

/**
 * mpc_discretize_zoh — Zero-order-hold discretization
 *
 * Given continuous state-space (A_c, B_c, C_c):
 *   A_d = exp(A_c * Ts)     [matrix exponential]
 *   B_d = A_c⁻¹ * (A_d - I) * B_c   [if A_c invertible]
 *   C_d = C_c
 *
 * For integrating process, A_c is singular (has eigenvalue 0).
 * Use:
 *   A_d = I + A_c*Ts + A_c²*Ts²/2! + ...  (series expansion)
 *   B_d = (I*Ts + A_c*Ts²/2! + ...) * B_c
 *
 * Implementation uses truncated Taylor series (5 terms, O(Ts⁶)).
 *
 * Reference: Franklin, Powell, Workman (1998) "Digital Control of Dynamic
 *            Systems" §4.3
 *
 * @param A_c     Continuous A matrix (n×n, column-major)
 * @param B_c     Continuous B matrix (n×1)
 * @param A_d     Output discrete A (n×n, column-major, pre-allocated)
 * @param B_d     Output discrete B (n×1, pre-allocated)
 * @param n       State dimension
 * @param dt      Sampling time
 * Returns 0 on success.
 */
int mpc_discretize_zoh(const double *A_c, const double *B_c,
                        double *A_d, double *B_d, int n, double dt);

/**
 * mpc_matrix_exponential — Matrix exponential via scaling-and-squaring
 *
 * Reference: Higham (2005) "The scaling and squaring method for the
 *            matrix exponential revisited", SIAM Review
 *
 * Pade approximation with scaling: exp(A) ≈ [R_{mm}(A/2^s)]^{2^s}
 * where R_{mm} is the (m,m) diagonal Padé approximant.
 *
 * @param A       Input matrix (n×n, column-major)
 * @param expA    Output exp(A) (n×n, column-major)
 * @param n       Dimension
 * Returns 0 on success.
 */
int mpc_matrix_exponential(const double *A, double *expA, int n);

/* ─── Model Validation ────────────────────────────────────────────────── */

/**
 * mpc_model_validate — Check model properties for MPC suitability
 *
 * Checks:
 *   1. Stability of discrete model (all |eig| ≤ 1 for integrating)
 *   2. Controllability: rank(ctrb(A,B)) == n
 *   3. Observability: rank(obsv(A,C)) == n
 *   4. Sampling time adequacy: Ts ≤ τ/5
 *   5. Gain sign consistency
 *
 * Returns bitmask of issues found (0 = valid):
 *   bit 0: unstable (>1 eigenvalues)
 *   bit 1: uncontrollable
 *   bit 2: unobservable
 *   bit 3: sampling too slow
 *   bit 4: zero gain
 *
 * @param model  Integrating process model
 * Returns issue bitmask.
 */
int mpc_model_validate(const integrating_process_t *model);

/* ─── Utility Functions ───────────────────────────────────────────────── */

/**
 * mpc_tank_area_to_gain — Convert physical tank parameters to process gain
 *
 * For a tank with level h, area A, outlet flow F_out = C_v*sqrt(h)*valve:
 *   dh/dt = (F_in - F_out) / A
 *   ∂h/∂(F_out) = -1/A → K = -1/A  [(m)/(m³/s)]
 *
 * For valve manipulation (valve% → flow):
 *   F_out = C_v * sqrt(h) * (valve%/100)
 *   K = -C_v*sqrt(h) / (100*A)   [(m/s)/%valve]
 *
 * @param area          A [m²]
 * @param valve_coeff   C_v [m^(5/2)/s]
 * @param level         h [m], current operating level
 * Returns process gain K.
 */
double mpc_tank_area_to_gain(double area, double valve_coeff, double level);

/**
 * mpc_residence_time — Compute tank residence time
 *
 * τ_res = V / F_out = (A * h) / F_out  [s]
 *
 * Key for understanding disturbance rejection capability.
 * Longer τ_res → better filtering of inlet flow disturbances.
 * For surge tanks: τ_res should be at least 2-5x disturbance period.
 *
 * @param area    Tank cross-sectional area [m²]
 * @param level   Current level [m]
 * @param outflow Outlet flow [m³/s]
 * Returns residence time [s].
 */
double mpc_residence_time(double area, double level, double outflow);

#ifdef __cplusplus
}
#endif

#endif /* MPC_INTEGRATING_MODEL_H */
