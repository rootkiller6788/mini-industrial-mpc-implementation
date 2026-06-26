/**
 * mpc_level_constraints.h — Level Constraint Handling for MPC
 *
 * Domain: Industrial Process Safety — Overfill & Dry-Run Prevention
 *
 * Level control in industrial vessels involves critical safety constraints
 * governed by multiple international standards. This module implements
 * constraint formulation, management, and feasibility recovery for MPC.
 *
 * Key Standards:
 *   API 2350 (5th Ed, 2020): Overfill Prevention for Storage Tanks
 *     - Categories I/II/III based on consequence
 *     - Automatic Overfill Prevention System (AOPS)
 *     - Level of Concern (LOC) and Response Time
 *   IEC 61511 (2016): Functional Safety — SIS for Process Industry
 *     - SIF design for high-high and low-low trips
 *     - SIL determination for level protection loops
 *   ISA-18.2 (2016): Alarm Management
 *     - Alarm rationalization for level alarms
 *   AWWA D100: Welded Steel Tanks for Water Storage
 *     - Structural requirements for surge tank design
 *
 * Knowledge Coverage:
 *   L4 - Standards: API 2350, IEC 61511, ISA-18.2, AWWA D100
 *   L2 - Core Concepts: Constraint types, feasibility, soft constraints
 *   L3 - Eng. Structures: Constraint formulation for QP, level margins
 *
 * MIT 6.302 · Purdue ME 575 · ISA/IEC Standards
 */

#ifndef MPC_LEVEL_CONSTRAINTS_H
#define MPC_LEVEL_CONSTRAINTS_H

#include "mpc_level_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ─── API 2350 Overfill Prevention ────────────────────────────────────── */

/**
 * api2350_category_t — API 2350 Overfill Prevention Category
 *
 * CAT_I:   Lowest risk. Small tanks, benign fluids. Operator-only response.
 *          No automatic system required.
 * CAT_II:  Moderate risk. AOPS required. Manual shutdown time acceptable.
 *          Single-point level sensor + independent high-high switch.
 * CAT_III: Highest risk. Large tanks, hazardous fluids, populated areas.
 *          Full AOPS with redundant sensors, voting logic, remote monitoring.
 *          Automated shutdown mandatory.
 *
 * Reference: API 2350 5th Ed., §4.5 "Hazard and Risk Assessment"
 */
typedef enum {
    API2350_CAT_I   = 0,
    API2350_CAT_II  = 1,
    API2350_CAT_III = 2
} api2350_category_t;

/**
 * api2350_overfill_config — API 2350 compliant overfill configuration
 *
 * Level of Concern (LOC): Highest level before overfill. Must not be reached.
 * Critical High (LAHH): SIS activation setpoint (hard trip).
 * High Alarm (LAH): Operator alarm setpoint.
 * Max Working (LAH_working): MPC constraint ceiling.
 *
 * Response Time (API 2350 §6.3): Total time from level reaching LOC
 * condition to completion of all required actions (valve closure, pump stop).
 * MPC must ensure predicted level stays ≤ LAH_working - (response_time * max_inflow/A).
 */
typedef struct {
    api2350_category_t  category;
    double  level_of_concern;       /* LOC [m or %], absolute physical limit */
    double  critical_high_setpoint; /* LAHH, SIS trip [m or %] */
    double  high_alarm_setpoint;    /* LAH, operator alarm [m or %] */
    double  max_working_setpoint;  /* MPC constraint ceiling [m or %] */
    double  response_time;         /* Required response time [s] */
    int     num_sensors;           /* Redundant level sensors (2 or 3) */
    int     voting_logic;          /* 1oo2, 2oo3, etc. */
    double  sensor_tolerance;      /* Max sensor disagreement [m] */
} api2350_overfill_config_t;

/* ─── IEC 61511 SIS Level Protection ──────────────────────────────────── */

/**
 * iec61511_sil_config — SIL configuration for level SIF
 *
 * IEC 61511 requires Safety Integrity Level (SIL) analysis for
 * level protection loops that prevent overfill or dry-run with
 * safety consequences.
 *
 * SIL 1: RRF 10-100, PFDavg 10⁻¹-10⁻²
 *   Minor injury, limited environmental release.
 * SIL 2: RRF 100-1000, PFDavg 10⁻²-10⁻³
 *   Serious injury, significant release.
 * SIL 3: RRF 1000-10000, PFDavg 10⁻³-10⁻⁴
 *   Fatality, major environmental damage.
 * SIL 4: RRF ≥ 10000, PFDavg ≥ 10⁻⁴
 *   Catastrophic (rare in process industry for level alone).
 *
 * Reference: IEC 61511-1:2016 §8-11
 *            Marszal (2016) "SIL Determination for SIS"
 */
typedef enum {
    SIL_NONE = 0,
    SIL_1    = 1,
    SIL_2    = 2,
    SIL_3    = 3,
    SIL_4    = 4
} iec61511_sil_level_t;

typedef struct {
    iec61511_sil_level_t sil_level;
    double  proof_test_interval;     /* T_proof [hours], e.g., 8760 = 1yr */
    double  pfd_avg;                 /* Average probability of failure on demand */
    double  spurious_trip_rate;      /* [trips/year], false positive SIS activation */
    int     redundant_channels;      /* Sensor + logic solver + final element */
    double  safety_margin;           /* SIS setpoint margin below/above LOC */
} iec61511_sil_config_t;

/* ─── Constraint Types and Formulation ────────────────────────────────── */

/**
 * mpc_constraint_spec_t — Single constraint specification
 *
 * Generic constraint: a^T * Δu ≤ b  (linear inequality on MV moves)
 *
 * For input:   Δu_j ≤ u_max - u_{j-1},   -Δu_j ≤ -u_min + u_{j-1}
 * For output:  g_i^T * Δu ≤ y_max - f_i, -g_i^T * Δu ≤ -y_min + f_i
 * For rate:    Δu_j ≤ Δu_max,             -Δu_j ≤ -Δu_min
 */
typedef struct {
    double  coefficients[MPC_MAX_QP_VARS];  /* a vector */
    double  bound;                           /* b (RHS) */
    constraint_type_t type;
    int     soft;                            /* 1 = soft (slack penalized), 0 = hard */
    double  soft_weight;                     /* Penalty weight for soft violation */
} mpc_constraint_spec_t;

/**
 * mpc_constraint_set_t — Collection of all active constraints
 *
 * Built from level config, valve limits, and tuning for each MPC step.
 */
typedef struct {
    mpc_constraint_spec_t specs[MPC_MAX_CONSTRAINTS];
    int  num_constraints;
    int  num_hard;
    int  num_soft;
    int  num_feasible;
} mpc_constraint_set_t;

/* ─── Constraint Management Functions ─────────────────────────────────── */

/**
 * mpc_constraint_build_input — Build input (MV) constraints
 *
 * u_min ≤ u(k+j) ≤ u_max  for j = 0..N_c-1
 *
 * Using: u(k+j) = u(k-1) + Σ_{l=0}^{j} Δu(k+l)
 * →  Σ_{l=0}^{j} Δu(k+l) ≤ u_max - u(k-1)
 * → -Σ_{l=0}^{j} Δu(k+l) ≤ -u_min + u(k-1)
 *
 * @param cset       Constraint set (appended to)
 * @param u_prev     Previous MV u(k-1)
 * @param u_min      Minimum MV
 * @param u_max      Maximum MV
 * @param N_c        Control horizon
 * Returns number of constraints added.
 */
int mpc_constraint_build_input(mpc_constraint_set_t *cset, double u_prev,
                                double u_min, double u_max, int N_c);

/**
 * mpc_constraint_build_rate — Build rate-of-change constraints
 *
 * Δu_min ≤ Δu(k+j) ≤ Δu_max  for j = 0..N_c-1
 *
 * Box constraints on Δu_j directly.
 * These are the simplest constraints (no summation needed).
 *
 * @param cset         Constraint set (appended to)
 * @param du_min       Minimum Δu per step
 * @param du_max       Maximum Δu per step
 * @param N_c          Control horizon
 * Returns number of constraints added.
 */
int mpc_constraint_build_rate(mpc_constraint_set_t *cset, double du_min,
                               double du_max, int N_c);

/**
 * mpc_constraint_build_output — Build output (level) constraints
 *
 * y_min ≤ ŷ(k+i) ≤ y_max  for i = 1..N_p
 *
 * Using: ŷ = G*Δu + f
 * →  G_i^T * Δu ≤ y_max - f_i   (row i of G)
 * → -G_i^T * Δu ≤ -y_min + f_i
 *
 * For integrating processes, early output constraints (small i) may be
 * impossible to satisfy if the level is already outside bounds.
 * In that case, constraint softening is required.
 *
 * @param cset      Constraint set (appended to)
 * @param G         Dynamic matrix (col-major, N_p × N_c)
 * @param f         Free response vector (N_p)
 * @param y_min     Minimum level
 * @param y_max     Maximum level
 * @param N_p       Prediction horizon
 * @param N_c       Control horizon
 * @param start_i   First prediction step to constrain (≥ 1, skip dead-time)
 * Returns number of constraints added.
 */
int mpc_constraint_build_output(mpc_constraint_set_t *cset,
                                 const double *G, const double *f,
                                 double y_min, double y_max,
                                 int N_p, int N_c, int start_i);

/**
 * mpc_constraint_build_terminal — Terminal constraint for stability
 *
 * ŷ(k+N_p) = r  (end-point at setpoint)
 * →  g_Np^T * Δu = r - f_Np
 *
 * For integrating processes, this equality constraint (or very heavily
 * weighted soft constraint) guarantees recursive feasibility and nominal
 * stability under the dual-mode prediction paradigm.
 *
 * Reference: Rawlings & Muske (1993), Mayne et al. (2000)
 *
 * @param cset      Constraint set (appended to)
 * @param G_last    Last row of dynamic matrix g_Np (N_c elements)
 * @param f_last    Last free response element f_Np
 * @param setpoint  Target setpoint r
 * @param N_c       Control horizon
 * Returns number of constraints added.
 */
int mpc_constraint_build_terminal(mpc_constraint_set_t *cset,
                                   const double *G_last, double f_last,
                                   double setpoint, int N_c);

/* ─── Feasibility and Recovery ────────────────────────────────────────── */

/**
 * mpc_feasibility_check — Check if constraints can be satisfied
 *
 * Determines if there exists Δu satisfying all hard constraints.
 * Uses a simple LP feasibility test (phase I of simplex).
 *
 * Returns 1 if feasible (∃ Δu), 0 if infeasible.
 *
 * Common infeasibility causes for integrating level:
 *   1. Level already beyond bounds + slow actuator: can't recover within N_p
 *   2. Rate limits too restrictive for required response
 *   3. Output constraint too tight immediately after setpoint change
 *
 * Recovery strategies (applied in order):
 *   1. Relax output constraints (softening)
 *   2. Extend prediction horizon (give more time)
 *   3. Relax input constraints (allow larger valve movement)
 *   4. Setpoint adjustment (temporary deviation to recover)
 *
 * @param cset  Constraint set
 * @param G     Dynamic matrix (N_p × N_c)
 * @param f     Free response (N_p)
 * @param N_p   Prediction horizon
 * @param N_c   Control horizon
 * Returns 1 if feasible.
 */
int mpc_feasibility_check(const mpc_constraint_set_t *cset,
                           const double *G, const double *f,
                           int N_p, int N_c);

/**
 * mpc_feasibility_recover — Attempt to recover feasibility
 *
 * Strategy 1: Soften output constraints (add slack with penalty)
 * Strategy 2: Temporarily widen output bounds (emergency margin)
 * Strategy 3: Minimize constraint violation as primary objective
 *
 * Returns 1 if feasible solution found, 0 if truly infeasible.
 *
 * For API 2350 Category III tanks, infeasibility on hard high constraints
 * means the system must escalate to SIS shutdown — MPC cannot resolve it.
 *
 * @param cset   Constraint set (modified: some hard → soft)
 * @param config Level configuration
 * @returns 1 if recovery successful.
 */
int mpc_feasibility_recover(mpc_constraint_set_t *cset,
                             const mpc_level_config_t *config);

/* ─── Safety Margin Calculation ───────────────────────────────────────── */

/**
 * mpc_safety_margin_api2350 — Calculate safety margin per API 2350
 *
 * Safety margin [m] = LOC - LAHH_setpoint
 *
 * Must account for:
 *   - Max fill rate [m/s]
 *   - Total response time [s] (operator detection + action + valve closure)
 *   - Measurement uncertainty [m] (sensor accuracy bands)
 *   - Foam/froth allowance (for agitated tanks)
 *
 *   margin ≥ max_fill_rate * response_time + 2*sensor_uncertainty
 *
 * @param config  API 2350 overfill configuration
 * @param max_fill_rate  Worst-case inlet flow / cross_area [m/s]
 * @returns Required safety margin [m or %].
 */
double mpc_safety_margin_api2350(const api2350_overfill_config_t *config,
                                  double max_fill_rate);

/**
 * mpc_sil_margin — SIS trip setpoint margin from normal operating range
 *
 * Per IEC 61511, the SIS trip setpoint must be:
 *   1. Far enough from normal range to avoid spurious trips
 *   2. Close enough to LOC to provide protection
 *
 * Trip setpoint = max_working ± safety_margin
 *
 * @param sil     SIL configuration
 * @param max_fill_rate  Max rate toward trip direction [m/s]
 * @returns Safety margin [m or %].
 */
double mpc_sil_margin(const iec61511_sil_config_t *sil, double max_fill_rate);

/* ─── Utility ─────────────────────────────────────────────────────────── */

/**
 * mpc_constraint_set_init — Initialize empty constraint set
 *
 * @param cset  Constraint set to initialize
 */
void mpc_constraint_set_init(mpc_constraint_set_t *cset);

/**
 * mpc_constraint_count_active — Count active (binding) constraints
 *
 * For a given Δu solution, checks which inequality constraints
 * are within tolerance of their bounds.
 *
 * Active constraints define the optimal face of the feasible polyhedron.
 * Monitoring active constraint count helps diagnose overly constrained
 * operation (de-tuning signal).
 *
 * @param cset      Constraint set
 * @param x_sol     Solution Δu* (n_vars)
 * @param tol       Active tolerance
 * Returns number of active constraints.
 */
int mpc_constraint_count_active(const mpc_constraint_set_t *cset,
                                 const double *x_sol, double tol);

#ifdef __cplusplus
}
#endif

#endif /* MPC_LEVEL_CONSTRAINTS_H */
