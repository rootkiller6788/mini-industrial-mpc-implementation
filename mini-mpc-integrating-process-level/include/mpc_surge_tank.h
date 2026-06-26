/**
 * mpc_surge_tank.h — Surge Tank Level Control with MPC
 *
 * Domain: Industrial Process Control — Surge Tank & Level Management
 *
 * Surge tanks are critical buffer vessels between process units. They absorb
 * flow fluctuations, allowing upstream and downstream units to operate
 * independently. The level control objective is fundamentally different
 * from setpoint tracking: the goal is flow smoothing / averaging, not
 * maintaining a precise level.
 *
 * Key references:
 *   McDonald & McAvoy (1986) "Application of dynamic matrix control to
 *     moderate-purity distillation columns" (introduced surge tank MPC concept)
 *   Luyben (2007) "Chemical Reactor Design and Control" §8 "Surge Tanks"
 *   Skogestad (2004) "Control structure design for complete chemical plants"
 *   Shinskey (1996) "Process Control Systems" §10 "Level Control"
 *
 * Physical Model:
 *   A * dh/dt = F_in(t) - F_out(t)
 *
 *   F_out = C_v * sqrt(h) * u  (gravity flow through valve)
 *   or F_out = u * pump_capacity  (pumped outflow)
 *
 * Knowledge Coverage:
 *   L6 - Canonical Problems: Surge tank level control, flow averaging
 *   L2 - Core Concepts: Buffer capacity, flow variability, residence time
 *   L7 - Applications: Refinery distillation, chemical plant surge
 *
 * Purdue ME 575 · RWTH Aachen Industrial Control · Tsinghua Process Control
 */

#ifndef MPC_SURGE_TANK_H
#define MPC_SURGE_TANK_H

#include "mpc_level_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ─── Tank Physical Modeling ──────────────────────────────────────────── */

/**
 * surge_tank_init — Initialize surge tank model from physical parameters
 *
 * Creates a physical model of a cylindrical vertical surge tank.
 *
 * Cross-sectional area: A = π * D² / 4  [m²]
 * Holdup volume: V = A * h  [m³]
 * Residence time: τ_res = V / F_out  [s]
 *
 * @param tank         Tank structure to initialize
 * @param diameter     D [m]
 * @param max_level    H_max [m]
 * @param initial_level h₀ [m]
 * @param valve_coeff  C_v [m^(5/2)/s] for gravity flow
 *                     (use arbitrary large for pumped)
 * Returns 0 on success.
 */
int surge_tank_init(surge_tank_t *tank, double diameter, double max_level,
                     double initial_level, double valve_coeff);

/**
 * surge_tank_outflow — Compute outflow given level and valve position
 *
 * Gravity flow (Torricelli's law):
 *   F_out = C_v * sqrt(h) * (u/100)
 *   where u = valve position [%], h = level [m]
 *
 * For pumped flow:
 *   F_out = u/100 * pump_capacity  (modeled in caller)
 *
 * @param tank    Tank state
 * @param valve   Valve position [0-100%]
 * Returns F_out [m³/s].
 */
double surge_tank_outflow(const surge_tank_t *tank, double valve);

/**
 * surge_tank_simulate — Simulate one time step of tank dynamics
 *
 * Euler integration:
 *   h(k+1) = h(k) + (Ts/A) * (F_in(k) - F_out(k))
 *
 * Clamped to [0, max_level].
 *
 * @param tank   Tank state (level updated)
 * @param F_in   Inlet flow [m³/s]
 * @param F_out  Outlet flow [m³/s]
 * @param Ts     Time step [s]
 */
void surge_tank_simulate(surge_tank_t *tank, double F_in, double F_out,
                          double Ts);

/* ─── Level Control Modes ─────────────────────────────────────────────── */

/**
 * surge_level_filter_factor — Compute φ (flow filter factor)
 *
 * φ quantifies the degree of inlet flow filtering:
 *
 *   F_out_ss = F_in_avg + φ * (F_in - F_in_avg)
 *
 *   φ = 1.0: Tight control. F_out follows F_in exactly.
 *             Level stays at setpoint. No filtering.
 *   φ = 0.0: Pure averaging. F_out = F_in_avg (constant).
 *             Level absorbs all variability.
 *   φ ∈ (0,1): Partial filtering. Trade-off between level variance
 *              and flow variance.
 *
 * For MPC: φ is achieved by tuning output weight Q vs. input weight R.
 * High Q → tight level → high φ (little filtering)
 * Low Q → relaxed level → low φ (more filtering)
 *
 * The optimal φ for a given tank depends on residence time and
 * disturbance period. From frequency domain (Shinskey 1996):
 *   φ_opt ≈ 1 / (1 + ω_dist * τ_res)
 *
 * @param bandwidth    Disturbance frequency to attenuate [rad/s]
 * @param tau_res      Tank residence time [s]
 * Returns optimal filter factor φ.
 */
double surge_level_filter_factor(double bandwidth, double tau_res);

/**
 * surge_capacity_utilization — Tank capacity utilization analysis
 *
 * For sinusoidal inlet flow: F_in = F_avg + A_f * sin(ω*t)
 * The level amplitude is:
 *   Δh_amplitude = A_f / (ω * A)  [pure integrator, no control]
 *
 * With level control: Δh_controlled = φ * A_f / (ω * A * (1 - φ))
 *
 * Maximum flow amplitude that can be handled without hitting limits:
 *   A_f_max = Δh_allowable * ω * A * (1 - φ) / φ
 *
 * @param tank        Tank parameters
 * @param bandwidth   Dominant disturbance frequency [rad/s]
 * @param phi         Filter factor
 * @param ampl_out    Output: expected level amplitude [m]
 * Returns maximum allowable flow amplitude [m³/s].
 */
double surge_capacity_utilization(const surge_tank_t *tank, double bandwidth,
                                   double phi, double *ampl_out);

/* ─── MPC Surge Tank Configuration ────────────────────────────────────── */

/**
 * surge_mpc_config — Configure MPC tuning for surge tank operation
 *
 * Derives MPC tuning parameters from surge tank physical properties
 * and desired operating mode (tight, averaging, surge).
 *
 * Tuning rules (McDonald & McAvoy heuristics + Shinskey):
 *   1. Sampling time: Ts = min(τ_res/20, 1 sec) but ≥ 0.1 sec
 *   2. Prediction horizon: N_p * Ts ≥ 3 * τ_res (see far enough)
 *   3. Control horizon: N_c = max(3, N_p/4) (standard industrial)
 *   4. Move suppression: λ = (1/φ - 1) * (A/K)² (derived from desired φ)
 *   5. Output weight: Q = 1 (normalized)
 *
 * @param tuning    Output MPC tuning
 * @param config    Output MPC level configuration
 * @param tank      Surge tank model
 * @param mode      Desired level control mode
 * @param inflow_nom Nominal inlet flow [m³/s] (design point)
 * Returns 0 on success.
 */
int surge_mpc_config(mpc_tuning_t *tuning, mpc_level_config_t *config,
                      const surge_tank_t *tank, level_control_mode_t mode,
                      double inflow_nom);

/* ─── Feedforward for Surge Tanks ──────────────────────────────────────── */

/**
 * surge_feedforward — Inlet flow feedforward compensation
 *
 * For surge tanks, inlet flow is often measurable and can be used as
 * feedforward in MPC. The feedforward action preemptively adjusts
 * outlet flow to match inlet changes:
 *
 *   Δu_ff = (ΔF_in * 100) / (C_v * sqrt(h))
 *
 * By incorporating feedforward, MPC can achieve much better flow
 * smoothing since the outlet flow change is coordinated with the
 * disturbance rather than reacting after level deviation is detected.
 *
 * In DMC with feedforward:
 *   ŷ = G_u * Δu + G_d * ΔF_in + f
 *
 * where G_d is the feedforward dynamic matrix from F_in to level.
 * For a pure tank: G_d = Ts/A * ones(N_p) (ramping response to flow step).
 *
 * @param tank        Tank current state
 * @param delta_F_in  Change in inlet flow [m³/s]
 * Returns feedforward valve move [%].
 */
double surge_feedforward(const surge_tank_t *tank, double delta_F_in);

/**
 * surge_feedforward_matrix — Build feedforward dynamic matrix G_d
 *
 * Step response of level to inlet flow step:
 *   For G(s) = 1/(A*s): step response = t/A → s_d,i = i*Ts/A
 *
 * @param G_d    Output: feedforward matrix (N_p elements, column vector)
 * @param tank   Tank model (area)
 * @param N_p    Prediction horizon
 * @param Ts     Sampling time
 */
void surge_feedforward_matrix(double *G_d, const surge_tank_t *tank,
                               int N_p, double Ts);

/* ─── Surge Tank Sizing ────────────────────────────────────────────────── */

/**
 * surge_tank_sizing — Determine minimum tank size for flow smoothing
 *
 * Given desired flow smoothing ratio φ and disturbance characteristics,
 * computes the minimum tank volume needed.
 *
 * From Shinskey (1996):
 *   V_min = (F_avg * T_d) / (2 * π * φ * Δh_frac)
 *
 * where T_d = disturbance period [s], F_avg = average flow [m³/s],
 * Δh_frac = allowable level variation as fraction of max level.
 *
 * Industrial rule of thumb for refinery surge drums:
 *   τ_res ≥ 5-10 minutes for good flow smoothing
 *   τ_res ≥ 30 minutes for feed surge to distillation columns
 *
 * Reference: Watkins (1979) Hydrocarbon Processing
 *            Branan (2005) "Rules of Thumb for Chemical Engineers"
 *
 * @param F_avg        Average flow [m³/s]
 * @param T_disturb    Disturbance period [s]
 * @param phi          Desired filter factor
 * @param delta_h_frac Allowable level fraction (e.g., 0.2 = ±20%)
 * @returns Minimum volume [m³].
 */
double surge_tank_sizing(double F_avg, double T_disturb, double phi,
                          double delta_h_frac);

/* ─── MPC Surge Simulation ────────────────────────────────────────────── */

/**
 * surge_mpc_simulation_run — Run closed-loop MPC simulation for surge tank
 *
 * Simulates the surge tank under MPC level control with inlet flow
 * disturbances. This is the primary demonstration function.
 *
 * @param tank        Surge tank (initial state provided)
 * @param tuning      MPC tuning
 * @param config      MPC level constraints
 * @param step        Step response model
 * @param dyn         DMC state
 * @param F_in_hist   Inlet flow history [n_steps]
 * @param n_steps     Number of simulation steps
 * @param F_out_hist  Output: outlet flow history [n_steps]
 * @param level_hist  Output: level history [n_steps]
 * @param valve_hist  Output: valve position history [n_steps]
 * @param kpi         Output: performance KPIs
 * Returns 0 on success.
 */
int surge_mpc_simulation_run(surge_tank_t *tank, const mpc_tuning_t *tuning,
                              const mpc_level_config_t *config,
                              const step_response_t *step,
                              dmc_dynamic_t *dyn,
                              const double *F_in_hist, int n_steps,
                              double *F_out_hist, double *level_hist,
                              double *valve_hist, mpc_kpi_t *kpi);

/* ─── Utility ─────────────────────────────────────────────────────────── */

/**
 * surge_tank_volume — Compute tank volume at current level
 *
 * V = A * h = (π * D² / 4) * h
 *
 * @param tank  Tank state
 * Returns volume [m³].
 */
double surge_tank_volume(const surge_tank_t *tank);

/**
 * surge_tank_residence_time — Compute residence time
 *
 * τ_res = V / F_out
 * If F_out ≈ 0, returns a large sentinel value (1e6).
 *
 * @param tank  Tank state
 * @param F_out Outlet flow [m³/s]
 * Returns residence time [s].
 */
double surge_tank_residence_time(const surge_tank_t *tank, double F_out);

/**
 * surge_level_percent — Level as percentage of tank height
 *
 * @param tank  Tank state
 * Returns level [% of max].
 */
double surge_level_percent(const surge_tank_t *tank);

#ifdef __cplusplus
}
#endif

#endif /* MPC_SURGE_TANK_H */
