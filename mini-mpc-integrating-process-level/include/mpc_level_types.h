/**
 * mpc_level_types.h — Core Type Definitions for MPC Integrating Level Control
 *
 * Domain: Industrial Model Predictive Control — Integrating Process Level
 * Standards: ISA-88 Batch Control, API 2350 Overfill Prevention, IEC 61511 SIS
 * Reference: Rawlings, Mayne, Diehl (2017) "Model Predictive Control"
 *            Seborg, Edgar, Mellichamp (2016) "Process Dynamics and Control"
 *            Qin & Badgwell (2003) "A survey of industrial MPC technology"
 *
 * This header defines fundamental data structures for MPC applied to
 * integrating (non-self-regulating) processes, particularly level control
 * in surge tanks, distillation columns, steam drums, and reactor vessels.
 *
 * An integrating process has a pole at the origin (1/s). Without feedback
 * control, the output drifts without bound. The transfer function is:
 *   G(s) = K / s  or  G(s) = K / [s * (τ*s + 1)]
 *
 * Knowledge Coverage:
 *   L1 - Definitions: All core MPC-integrating process types
 *   L3 - Eng. Structures: MPC data layout, prediction/control horizon
 *
 * MIT 2.171 · Stanford ENGR205 · Purdue ME 575 · RWTH Industrial Control
 * CMU 24-677 · Georgia Tech ECE 6550 · Tsinghua Process Control Eng.
 */

#ifndef MPC_LEVEL_TYPES_H
#define MPC_LEVEL_TYPES_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ─── Process Classification ─────────────────────────────────────────── */

/**
 * process_dynamics_t — Process type classification
 *
 * INTEGRATING:  Pole at origin, output drifts w/o control.
 *               G(s) = K / [s * (τ*s + 1)^n]
 *               Examples: tank level, distillation sump, steam drum
 * SELF_REGULATING: Stable, reaches steady state.
 *               G(s) = K / [(τ₁*s + 1)(τ₂*s + 1)]
 *               Examples: heat exchanger temperature, flow loop
 * RUN_AWAY:     Positive pole, unstable open-loop.
 *               G(s) = K / [(τ₁*s - 1)]
 *               Examples: exothermic CSTR, polymerization reactor
 * DOUBLE_INT:   Two poles at origin, ramping output.
 *               G(s) = K / s²
 *               Examples: satellite attitude, crane position
 */
typedef enum {
    PROCESS_INTEGRATING    = 0,
    PROCESS_SELF_REGULATING = 1,
    PROCESS_RUN_AWAY       = 2,
    PROCESS_DOUBLE_INT     = 3
} process_dynamics_t;

/**
 * level_control_mode_t — Level control strategy taxonomy
 *
 * TIGHT_CONTROL:      Maintain level exactly at setpoint. Uses full valve
 *                     movement. Downstream units see flow variability.
 *                     φ = 1.0 (no filtering of inlet disturbances)
 *
 * AVERAGING_CONTROL:  Let level float within bounds. Smooths outflow to
 *                     downstream. Trade-off between capacity and filtering.
 *                     0 < φ < 1.0 (partial filtering)
 *
 * SURGE_CONTROL:      Maximize use of tank capacity. Flow smoothing is
 *                     primary objective. Level becomes secondary.
 *                     φ ≈ 0.0 (maximum filtering)
 *
 * Reference: McDonald & McAvoy (1986), Skogestad (2003) "Simple analytic
 *            rules for model reduction and PID controller tuning"
 *            Luyben (2007) "Chemical Reactor Design and Control"
 */
typedef enum {
    LEVEL_MODE_TIGHT     = 0,
    LEVEL_MODE_AVERAGING = 1,
    LEVEL_MODE_SURGE     = 2
} level_control_mode_t;

/**
 * mpc_solver_type_t — QP solver selection
 *
 * ACTIVE_SET:   Classical primal active-set. Goldfarb-Idnani (1983).
 *               Guarantees optimal solution in finite steps. O(m³) worst case.
 * HILDRETH:     Hildreth's quadratic programming (1957) for simple bounds
 *               only. Iterative, suitable for embedded.
 * INTERIOR_POINT: Mehrotra predictor-corrector (1992). Best for large
 *               problems. Quadratic convergence.
 * GRADIENT_PROJ: Gradient projection (Rosen 1960). Fast for box constraints.
 */
typedef enum {
    MPC_SOLVER_ACTIVE_SET    = 0,
    MPC_SOLVER_HILDRETH      = 1,
    MPC_SOLVER_INTERIOR_POINT = 2,
    MPC_SOLVER_GRADIENT_PROJ = 3
} mpc_solver_type_t;

/**
 * qp_status_t — QP solver return status
 */
typedef enum {
    QP_OPTIMAL        = 0,
    QP_INFEASIBLE     = 1,
    QP_UNBOUNDED      = 2,
    QP_MAX_ITERATIONS = 3,
    QP_NUMERICAL_ERROR = 4,
    QP_NOT_SOLVED     = 5
} qp_status_t;

/**
 * constraint_type_t — Constraint classification for MPC
 *
 * HARD:  Must be satisfied. Violation = unsafe or physically impossible.
 *        E.g., valve fully closed (0%) or fully open (100%).
 * SOFT:  Preferred. Violation penalized. E.g., level high-high alarm.
 *        Allows feasibility recovery.
 * RATE:  Rate-of-change constraint on MV. Δu_min ≤ Δu(k) ≤ Δu_max.
 *
 * Reference: Rawlings et al. (2017) §2.5, §2.7
 *            Scokaert & Rawlings (1999) "Feasibility issues in MPC with
 *            state and input constraints"
 */
typedef enum {
    CONSTRAINT_HARD_INPUT       = 0,
    CONSTRAINT_SOFT_INPUT       = 1,
    CONSTRAINT_HARD_OUTPUT      = 2,
    CONSTRAINT_SOFT_OUTPUT      = 3,
    CONSTRAINT_RATE_OF_CHANGE   = 4,
    CONSTRAINT_TERMINAL         = 5,
    CONSTRAINT_TERMINAL_SET     = 6
} constraint_type_t;

/* ─── MPC Configuration Structures ────────────────────────────────────── */

/**
 * mpc_tuning_t — MPC horizon and tuning parameters
 *
 * Prediction Horizon N_p: How far into the future we predict (samples).
 *   Longer N_p → more foresight, more computation. Typically N_p*T_s
 *   covers rise time + dead time of the process.
 *
 * Control Horizon N_c: How many free MV moves we compute (samples).
 *   N_c << N_p typically. N_c = 3-5 is common industrial practice.
 *   Larger N_c → more aggressive control.
 *
 * Sampling Time T_s: must satisfy T_s ≤ τ_dom/5 (dominant time constant/5)
 *   for adequate discretization. Nyquist: T_s ≤ π/ω_max.
 *
 * Move Suppression λ (lambda): Penalizes Δu changes.
 *   λ → 0: aggressive, minimal penalty on MV movement
 *   λ → ∞: conservative, large penalty → slow control
 *
 * Output Weight Q: Penalizes setpoint deviation.
 *   Q → ∞: tight level control, aggressive valve movement
 *   Q → 0: relaxed level, smooth MV
 */
typedef struct {
    int     prediction_horizon;   /* N_p [samples], ≥ 10 */
    int     control_horizon;      /* N_c [samples], ≥ 1, ≤ N_p */
    double  sampling_time;        /* T_s [seconds], > 0 */
    double  move_suppression;     /* λ ≥ 0, typical 0.1–10 */
    double  output_weight;        /* Q > 0 */
    double  input_weight;         /* R ≥ 0, MV penalty */
    double  terminal_weight;      /* P ≥ 0, terminal cost (stability) */
    double  reference_trajectory_alpha; /* α ∈ [0,1], exponential filter to SP */
    int     move_blocking_enabled; /* Group free moves into blocks */
    int     move_blocking_divisor; /* Block size divisor for N_c */
} mpc_tuning_t;

/**
 * mpc_level_config_t — Complete MPC level controller configuration
 *
 * level_setpoint:   Desired level [% of span] or [m]
 * level_lo_limit:   Low-level constraint (pump cavitation protection)
 * level_hi_limit:   High-level constraint (overflow prevention, API 2350)
 * level_hihi_limit: High-high alarm trip point (SIS activation, IEC 61511)
 * valve_min:        Minimum valve position [%]
 * valve_max:        Maximum valve position [%]
 * valve_rate_max:   Maximum valve slew rate [%/s]
 * inflow_disturbance: Measured disturbance (MV feedforward)
 */
typedef struct {
    double  level_setpoint;
    double  level_lo_limit;
    double  level_hi_limit;
    double  level_hihi_limit;
    double  valve_min;
    double  valve_max;
    double  valve_rate_max;
    double  inflow_disturbance;
} mpc_level_config_t;

/* ─── Process Model Structures ────────────────────────────────────────── */

/**
 * integrating_process_t — Continuous-time integrating process model
 *
 * Standard form:
 *   G(s) = K / [s * (τ*s + 1)]
 *   where K = gain [m/%/s] or [%/%/s], τ = time constant [s]
 *
 * For a tank: K = 1/(ρ*A), where A = cross-sectional area [m²]
 *   dh/dt = (F_in - F_out) / A
 *   → H(s) / F_out(s) = -1/(A*s)  (integrator)
 *
 * Discrete-time (ZOH, T_s sampling):
 *   G(z) = K * [(τ*(1-α) + T_s*α)*z - (τ*(1-α) - τ*α - T_s*α)]
 *          / [(z-1)*(z-α)]
 *   where α = exp(-T_s/τ)
 */
typedef struct {
    double  gain;           /* K, process gain */
    double  time_constant;  /* τ [s], lag time constant */
    double  dead_time;      /* θ [s], pure transport delay */
    double  sampling_time;  /* T_s [s], sampling period */
    double  area;           /* A [m²], tank cross-sectional area (optional) */
    double  max_inflow;     /* F_in_max [m³/s] */
    double  max_outflow;    /* F_out_max [m³/s] */
} integrating_process_t;

/**
 * step_response_t — Unit step response model (DMC form)
 *
 * DMC uses a finite step response (FSR) model:
 *   y(k) = y₀ + Σᵢ₌₁ꜛᴺ s_i * Δu(k-i)
 * where s_i are step response coefficients, truncated at N.
 *
 * N_model: Truncation horizon. Must be large enough that s_N ≈ s_∞
 *          (steady-state gain reached). For integrating: N must cover
 *          the region before ramp dominates.
 *
 * s_i: Step response coefficient at sample i.
 *      For integrating process: s_i grows approx linearly for i > τ/T_s.
 */
#define MPC_MAX_STEP_HORIZON 120
typedef struct {
    double  coeff[MPC_MAX_STEP_HORIZON];
    int     n_coeffs;
    double  steady_state_gain;
    double  dead_time_samples;
} step_response_t;

/**
 * integrating_state_t — State-space representation
 *
 * x1: level deviation from setpoint [m] or [%]
 * x2: integrating disturbance state (modeled as random walk)
 * x3: unmeasured inlet flow disturbance (if estimated)
 *
 * State-space model (augmented with integrating disturbance):
 *   x(k+1) = A * x(k) + B * Δu(k) + B_d * d(k)
 *     y(k) = C * x(k)
 *
 * Reference: Muske & Badgwell (2002) "Disturbance modeling for offset-free
 *            linear MPC", AIChE Journal
 */
#define MPC_MAX_STATES 6
typedef struct {
    double  A[MPC_MAX_STATES * MPC_MAX_STATES];
    double  B[MPC_MAX_STATES];
    double  B_d[MPC_MAX_STATES];
    double  C[MPC_MAX_STATES];
    double  x[MPC_MAX_STATES];
    int     n_states;
    int     n_outputs;
    int     n_inputs;
    int     n_disturbances;
} integrating_state_t;

/* ─── DMC Specific Structures ─────────────────────────────────────────── */

/**
 * dmc_dynamic_matrix_t — DMC prediction structure
 *
 * The dynamic matrix G (N_p × N_c) maps future MV moves to predicted outputs:
 *   ŷ = G * Δu + f
 *
 * G is built from step response coefficients (Toeplitz structure):
 *   G_{i,j} = s_{i-j+1}  for i ≥ j, else 0
 *
 * f is the free response (prediction with no future moves)
 *   f_i = y(k) + Σ_{l=1}^{N} (s_{i+l} - s_l) * Δu(k-l)
 *
 * Reference: Cutler & Ramaker (1979) "Dynamic Matrix Control"
 *            Shell Oil Company (proprietary, became public domain)
 */
typedef struct {
    double  dynamic_matrix[MPC_MAX_STEP_HORIZON * 20]; /* G, N_p×N_c */
    double  free_response[MPC_MAX_STEP_HORIZON];       /* f */
    double  error_correction[MPC_MAX_STEP_HORIZON];    /* bias correction */
    double  reference_trajectory[MPC_MAX_STEP_HORIZON]; /* w, desired path */
    double  delta_u_past[MPC_MAX_STEP_HORIZON];        /* past MV moves */
    int     n_prediction;   /* N_p */
    int     n_control;      /* N_c */
    int     n_model;        /* N, truncation */
} dmc_dynamic_t;

/* ─── QP Problem Structure ────────────────────────────────────────────── */

/**
 * qp_problem_t — Quadratic Program formulation
 *
 * Standard form:
 *   min  0.5 * Δu^T * H * Δu + c^T * Δu
 *   s.t. A_ineq * Δu ≤ b_ineq
 *        Δu_min ≤ Δu ≤ Δu_max
 *
 * For DMC-MPC tracking objective:
 *   H = G^T * Q_mat * G + R_mat   (N_c × N_c, SPD)
 *   c = G^T * Q_mat * (f - w)    (N_c × 1)
 *
 * Reference: Nocedal & Wright (2006) "Numerical Optimization" §16
 *            Fletcher (1987) "Practical Methods of Optimization"
 */
#define MPC_MAX_CONSTRAINTS 80
#define MPC_MAX_QP_VARS 20
typedef struct {
    double  H[MPC_MAX_QP_VARS * MPC_MAX_QP_VARS]; /* Hessian (SPD) */
    double  c[MPC_MAX_QP_VARS];                   /* gradient */
    double  A_ineq[MPC_MAX_CONSTRAINTS * MPC_MAX_QP_VARS]; /* inequality matrix */
    double  b_ineq[MPC_MAX_CONSTRAINTS];                   /* inequality bounds */
    double  x_lower[MPC_MAX_QP_VARS];                      /* var lower bound */
    double  x_upper[MPC_MAX_QP_VARS];                      /* var upper bound */
    int     n_vars;
    int     n_ineq_constraints;
    int     n_eq_constraints;
} qp_problem_t;

/**
 * mpc_solution_t — MPC solution at each sampling instant
 *
 * delta_u_plan: Computed future MV moves [Δu(k), ..., Δu(k+N_c-1)]
 * u_plan:       Resulting MV values [u(k), ..., u(k+N_c-1)]
 * y_pred:       Predicted output trajectory [ŷ(k+1), ..., ŷ(k+N_p)]
 * objective:    Optimal objective value J*
 * solve_status: QP solver result
 * cpu_time_ms:  Computation time for performance monitoring
 */
typedef struct {
    double  delta_u_plan[MPC_MAX_QP_VARS];
    double  u_plan[MPC_MAX_QP_VARS];
    double  y_pred[MPC_MAX_STEP_HORIZON];
    double  objective;
    qp_status_t solve_status;
    int     iterations;
    double  cpu_time_ms;
} mpc_solution_t;

/* ─── Kalman Filter Structures ────────────────────────────────────────── */

/**
 * kalman_config_t — Kalman filter configuration for integrating state
 *
 * For offset-free MPC, the state is augmented with an integrating
 * disturbance d(k) modeled as:
 *   d(k+1) = d(k) + w_d(k)    [random walk]
 *   y(k)   = C*x(k) + d(k) + v(k)
 *
 * Q_w: Process noise covariance matrix (tuning parameter)
 * R_v: Measurement noise covariance (scalar for SISO)
 * P_prior, P_post: Estimation error covariance
 *
 * Reference: Pannocchia & Rawlings (2003) "Disturbance models for
 *            offset-free MPC", IEEE TAC
 */
#define KALMAN_MAX_STATES 8
typedef struct {
    double  A[KALMAN_MAX_STATES * KALMAN_MAX_STATES];
    double  B[KALMAN_MAX_STATES];
    double  C[KALMAN_MAX_STATES];
    double  Q_w[KALMAN_MAX_STATES * KALMAN_MAX_STATES];
    double  R_v;
    double  P_prior[KALMAN_MAX_STATES * KALMAN_MAX_STATES];
    double  P_post[KALMAN_MAX_STATES * KALMAN_MAX_STATES];
    double  K_gain[KALMAN_MAX_STATES];
    double  x_prior[KALMAN_MAX_STATES];
    double  x_post[KALMAN_MAX_STATES];
    int     n_states;
    int     n_outputs;
} kalman_config_t;

/* ─── GPC Model Structures ─────────────────────────────────────────────── */

/**
 * gpc_config_t — Generalized Predictive Control (GPC) model
 *
 * GPC uses CARIMA model (Controlled AR Integrated Moving Average):
 *   A(z⁻¹) * y(k) = B(z⁻¹) * z^{-d} * u(k-1) + C(z⁻¹) * ξ(k) / Δ
 *
 * where Δ = 1 - z⁻¹ is the differencing operator, ξ is white noise.
 * The Δ operator implicitly introduces integral action for offset-free
 * tracking.
 *
 *   A(z⁻¹) = 1 + a₁z⁻¹ + ... + a_{na} z^{-na}
 *   B(z⁻¹) = b₀ + b₁z⁻¹ + ... + b_{nb} z^{-nb}
 *   C(z⁻¹) = 1 (T-filter for robustness, normally set to 1)
 *
 * Reference: Clarke, Mohtadi, Tuffs (1987) "Generalized Predictive Control"
 *            Automatica, Parts I & II
 *            Bitmead, Gevers, Wertz (1990) "Adaptive Optimal Control"
 */
#define GPC_MAX_ORDER 10
typedef struct {
    double  A_coeff[GPC_MAX_ORDER + 1]; /* A polynomial, starts with a₀=1 */
    double  B_coeff[GPC_MAX_ORDER + 1]; /* B polynomial */
    double  C_coeff[GPC_MAX_ORDER + 1]; /* C (noise coloring) polynomial */
    int     na;           /* order of A */
    int     nb;           /* order of B */
    int     nc;           /* order of C */
    int     delay;        /* d, pure delay in samples */
    double  lambda_gpc;   /* control weighting λ, typical 0.1-10 */
    int     n1;            /* minimum prediction horizon (≥ d+1) */
    int     n2;            /* maximum prediction horizon = N_p */
    int     nu;            /* control horizon = N_c */
} gpc_config_t;

/* ─── Surge Tank Structures ────────────────────────────────────────────── */

/**
 * surge_tank_t — Physical surge tank model
 *
 * Level dynamics (mass balance):
 *   A * dh/dt = F_in(t) - F_out(t)
 *
 * where A = tank cross-sectional area [m²]
 *       h = liquid level [m]
 *       F_in = inlet volumetric flow [m³/s]
 *       F_out = outlet volumetric flow [m³/s] = C_v * sqrt(h) * valve%
 *
 * For a cylindrical vertical tank: A = π * D² / 4
 *
 * Reference: Luyben (2007) "Chemical Reactor Design and Control"
 *            Shinskey (1996) "Process Control Systems"
 */
typedef struct {
    double  diameter;          /* D [m], tank diameter */
    double  cross_area;        /* A = π*D²/4 [m²] */
    double  max_level;         /* H_max [m], tank height */
    double  level;             /* h [m], current level */
    double  inflow;            /* F_in [m³/s], disturbance */
    double  outflow;           /* F_out [m³/s], manipulated */
    double  valve_coeff;       /* C_v, outflow valve coefficient */
    double  valve_position;    /* u [%], valve opening 0-100% */
    double  holdup_volume;     /* V = A*h [m³] */
    double  residence_time;    /* τ_res = V/F_out [s] */
} surge_tank_t;

/* ─── Performance Monitoring Structures ────────────────────────────────── */

/**
 * mpc_kpi_t — MPC performance KPIs (Key Performance Indicators)
 *
 * ISE:   Integral Squared Error. Σ e(k)²
 * IAE:   Integral Absolute Error. Σ |e(k)|
 * ITAE:  Integral Time-weighted Absolute Error. Σ k * T_s * |e(k)|
 * MV_Variance: Variance of moves, indicates control effort
 * ValveTravel: Total valve movement Σ |Δu(k)|
 * CpK:   Process capability index = min((USL-μ)/(3σ), (μ-LSL)/(3σ))
 * Utilization: Percent time within bounds
 *
 * Reference: ASTM D6299-21 "Applying Statistical Quality Assurance"
 *            Shardt (2015) "Control Performance Assessment"
 */
typedef struct {
    double  integral_squared_error;
    double  integral_absolute_error;
    double  integral_time_absolute_error;
    double  mv_variance;
    double  valve_travel_total;
    double  process_capability_cpk;
    double  level_utilization_pct;
    double  constraint_violation_pct;
    long    sample_count;
} mpc_kpi_t;

/* ─── Operator Interface Structures ────────────────────────────────────── */

/**
 * mpc_operator_data_t — Real-time operator display data
 *
 * Human-Machine Interface (HMI) values for DCS operator console.
 * Follows ISA-101 HMI standard for process automation displays.
 *
 * Reference: ISA-101.01-2015 "Human Machine Interfaces for Process
 *            Automation Systems"
 *            ASM (Abnormal Situation Management) Consortium Guidelines
 */
typedef struct {
    double  current_level;         /* PV [m] or [%] */
    double  level_setpoint;        /* SP */
    double  valve_position;        /* MV [%] */
    double  predicted_level_1min;  /* 1-minute ahead prediction */
    double  predicted_level_5min;  /* 5-minute ahead prediction */
    double  predicted_level_15min; /* 15-minute ahead prediction */
    double  time_to_hihi;          /* estimated seconds to reach hi-hi alarm */
    double  time_to_lolo;          /* estimated seconds to reach lo-lo alarm */
    double  remaining_capacity_pct;/* buffer remaining [%] */
    int     mpc_active;            /* 1=ON, 0=manual/bypass */
    int     constraint_active;     /* bitmask of active constraints */
    int     alarm_status;          /* 0=normal, 1=warning, 2=critical */
} mpc_operator_data_t;

/* ─── Utility Declarations ────────────────────────────────────────────── */

const char* process_dynamics_name(process_dynamics_t type);
const char* level_control_mode_name(level_control_mode_t mode);
const char* mpc_solver_name(mpc_solver_type_t solver);
const char* qp_status_string(qp_status_t status);
const char* constraint_type_name(constraint_type_t ct);

#ifdef __cplusplus
}
#endif

#endif /* MPC_LEVEL_TYPES_H */
