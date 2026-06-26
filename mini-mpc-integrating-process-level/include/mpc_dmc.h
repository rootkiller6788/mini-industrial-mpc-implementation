/**
 * mpc_dmc.h — Dynamic Matrix Control for Integrating Processes
 *
 * Domain: Industrial MPC — DMC Algorithm for Level Control
 *
 * DMC (Dynamic Matrix Control) was developed at Shell Oil in the 1970s
 * (Cutler & Ramaker, 1979) and is the foundation of industrial MPC.
 * It uses a finite step response (FSR) model and solves a quadratic
 * program at each sampling instant to compute optimal MV moves.
 *
 * Key references:
 *   Cutler & Ramaker (1979) "Dynamic Matrix Control — A computer control
 *     algorithm" (Shell internal, later AIChE 1980)
 *   Garcia & Morshedi (1984) "Quadratic Programming Solution of DMC"
 *   Lundström et al. (1995) "Limitations of DMC for integrating processes"
 *
 * For integrating processes, DMC requires special handling:
 *   1. The step response does not settle → truncation introduces error
 *   2. The dynamic matrix is not strictly diagonally dominant
 *   3. The prediction horizon must extend far enough to capture ramp
 *   4. Terminal constraint or weight required for nominal stability
 *
 * Knowledge Coverage:
 *   L2 - Core Concepts: Receding horizon, DMC prediction
 *   L3 - Eng. Structures: Dynamic matrix, Toeplitz structure
 *   L5 - Algorithms: DMC control law, QP setup, move suppression
 *
 * MIT 2.171 · CMU 24-677 · Purdue ME 575 · Tsinghua Process Control Eng.
 */

#ifndef MPC_DMC_H
#define MPC_DMC_H

#include "mpc_level_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ─── Dynamic Matrix Construction ─────────────────────────────────────── */

/**
 * dmc_build_dynamic_matrix — Construct the DMC dynamic matrix G
 *
 * G is N_p × N_c Toeplitz matrix of step response coefficients:
 *
 *   G[i][j] = 0                    for i < j
 *   G[i][j] = s_{i-j+1}           for i ≥ j
 *
 * Visually:
 *   G = [s₁     0     0  ...  0  ]
 *       [s₂    s₁     0  ...  0  ]
 *       [s₃    s₂    s₁  ...  0  ]
 *       [  :            :   :  :  ]
 *       [s_Np  s_{Np-1}  ...  s_{Np-Nc+1}]
 *
 * For integrating processes: s_i grows with i.
 * The tail elements s_{i-j+1} for large (i-j) are large → strong coupling.
 * This means future moves have large impact on far-future predictions.
 *
 * @param G     Output dynamic matrix (column-major, N_p × N_c)
 * @param step  Step response coefficients
 * @param N_p   Prediction horizon
 * @param N_c   Control horizon
 * Returns 0 on success.
 */
int dmc_build_dynamic_matrix(double *G, const step_response_t *step,
                              int N_p, int N_c);

/**
 * dmc_build_free_response — Compute free response f (no future moves)
 *
 * The free response f_i is the predicted output at time k+i assuming
 * no future control moves (Δu(k) = Δu(k+1) = ... = 0):
 *
 *   f_i = y_m(k) + Σ_{l=1}^{N} (s_{i+l} - s_l) * Δu(k-l)
 *
 * where y_m(k) is the current measured output.
 *
 * For integrating processes, the free response typically ramps due to
 * the non-zero velocity gain. The "coasting" prediction can be visualized
 * as: if we hold the valve steady, where does the level go?
 *
 * Implementation uses the recursive form:
 *   f_{i+1} = f_i + Σ_{l=1}^{N-1} Δs_{l+1} * Δu(k+i-l)
 *   where Δs_{l} = s_{l} - s_{l-1} (impulse response)
 *
 * @param dyn       DMC dynamic structure (output f populated)
 * @param step      Step response
 * @param y_meas    Current measured output y_m(k)
 * @param delta_u_past  Array of past Δu values [Δu(k-1), ..., Δu(k-N)]
 * @param n_past    Length of delta_u_past
 * Returns 0 on success.
 */
int dmc_build_free_response(dmc_dynamic_t *dyn, const step_response_t *step,
                             double y_meas, const double *delta_u_past,
                             int n_past);

/**
 * dmc_reference_trajectory — Build exponential reference trajectory
 *
 * Smooth approach to setpoint from current output:
 *   w(k+i) = α^i * y(k) + (1 - α^i) * r
 *
 * where r = setpoint, α = reference_trajectory_alpha ∈ [0,1).
 * α = 0: immediate setpoint tracking (aggressive)
 * α ≈ 0.9: slow approach (conservative, smooth MV)
 *
 * The reference trajectory avoids aggressive setpoint changes that would
 * violate rate constraints. Standard industrial practice (AspenTech DMC3,
 * Honeywell Profit Controller) sets α based on desired closed-loop τ_cl:
 *   α = exp(-Ts / τ_cl)
 *
 * @param dyn    DMC structure (reference_trajectory populated)
 * @param y_k    Current output
 * @param r      Setpoint target
 * @param alpha  Exponential filter factor
 * @param N_p    Prediction horizon length
 */
void dmc_reference_trajectory(dmc_dynamic_t *dyn, double y_k, double r,
                               double alpha, int N_p);

/**
 * dmc_error_correction — Compute DMC bias correction term
 *
 * DMC uses an additive output bias to compensate for plant-model mismatch:
 *   d(k) = y_m(k) - ŷ(k|k-1)
 *
 * where ŷ(k|k-1) is the one-step-ahead prediction from the previous step.
 * This bias is added to all future predictions (constant bias assumption):
 *   ŷ(k+i|k) = ŷ_model(k+i|k) + d(k)
 *
 * The constant output disturbance assumption is the simplest feedback
 * mechanism in DMC. It provides integral action for step disturbances.
 *
 * For integrating processes, the constant bias assumption can be violated
 * when disturbances are ramping. In that case, the integrating disturbance
 * model (Kalman filter) is preferred.
 *
 * @param dyn       DMC structure
 * @param y_meas    Current measurement
 * @param y_pred_1  Previous one-step prediction ŷ(k|k-1)
 * @param N_p       Prediction horizon (error applied to all)
 */
void dmc_error_correction(dmc_dynamic_t *dyn, double y_meas,
                           double y_pred_1, int N_p);

/* ─── DMC Control Law (Unconstrained) ────────────────────────────────── */

/**
 * dmc_unconstrained_solution — Compute unconstrained DMC control law
 *
 * Minimizes:
 *   J = (G*Δu + f - w)^T * Q * (G*Δu + f - w) + Δu^T * R * Δu
 *
 * Closed-form solution (R = λ*I, Q = I for simplicity):
 *   Δu* = (G^T*G + λ*I)^{-1} * G^T * (w - f)
 *
 * Only the first element Δu*(0) is applied to the plant (receding horizon).
 *
 * Implementation uses Cholesky factorization of (G^T*G + λ*I) for numerical
 * stability. For integrating processes, G^T*G is positive definite for
 * λ > 0 and N_c ≥ 1.
 *
 * Condition number analysis:
 *   cond(G^T*G + λ*I) ≤ (σ_max² + λ) / λ
 *   For integrating processes, σ_max grows with N_p → larger λ needed.
 *
 * @param solution   Output solution (delta_u_plan populated)
 * @param dyn        DMC dynamic structure
 * @param tuning     MPC tuning parameters (λ, Q, R)
 * Returns 0 on success, -1 on numerical error.
 */
int dmc_unconstrained_solution(mpc_solution_t *solution,
                                const dmc_dynamic_t *dyn,
                                const mpc_tuning_t *tuning);

/**
 * dmc_solve_cholesky — Solve Ax = b by Cholesky for SPD matrix A
 *
 * G^T*G + λ*I is symmetric positive definite for λ > 0.
 * Cholesky: A = L*L^T, then L*y = b (forward), L^T*x = y (backward).
 *
 * O(n³/6) for factorization, O(n²) for solve.
 * Numerically backward stable per Higham (2002) §10.
 *
 * @param A    Input SPD matrix (n×n col-major, overwritten)
 * @param b    Input RHS vector, overwritten with solution
 * @param n    Dimension
 * Returns 0 on success, -1 if matrix not SPD.
 */
int dmc_solve_cholesky(double *A, double *b, int n);

/* ─── DMC with Hard Constraints (QP Setup) ────────────────────────────── */

/**
 * dmc_setup_qp — Build QP from DMC prediction for constrained problem
 *
 * The constrained DMC problem is:
 *   min  0.5 * Δu^T * H * Δu + c^T * Δu
 *   s.t. u_min ≤ u(k+j) ≤ u_max,     j = 0, ..., N_c-1
 *        Δu_min ≤ Δu(k+j) ≤ Δu_max,  j = 0, ..., N_c-1
 *        y_min ≤ ŷ(k+i) ≤ y_max,     i = 1, ..., N_p
 *
 * where H = G^T*Q*G + R,  c = G^T*Q*(f - w)
 *
 * Input constraints (u_min, u_max) are transformed to Δu constraints:
 *   u(k+j) = u(k-1) + Σ_{l=0}^{j} Δu(k+l)
 *   → L * Δu ≤ u_max - u(k-1)*1,  -L * Δu ≤ u_min + u(k-1)*1
 *   where L is lower-triangular of ones.
 *
 * Output constraints (y_min, y_max):
 *   G * Δu ≤ y_max - f,  -G * Δu ≤ -y_min + f
 *
 * Rate constraints are box constraints on Δu directly.
 *
 * @param qp       Output QP problem (H, c, constraints)
 * @param dyn      DMC dynamic structure (G, f, w)
 * @param tuning   MPC tuning (Q, R weights)
 * @param config   Level configuration (constraint limits)
 * @param u_prev   Previous MV value u(k-1)
 * Returns 0 on success, -1 if over-constrained.
 */
int dmc_setup_qp(qp_problem_t *qp, const dmc_dynamic_t *dyn,
                  const mpc_tuning_t *tuning,
                  const mpc_level_config_t *config, double u_prev);

/**
 * dmc_compute_prediction — Compute predicted output trajectory
 *
 * After solving for Δu*:
 *   ŷ = G * Δu* + f
 *
 * @param dyn       DMC structure (y_pred populated)
 * @param solution  Optimal Δu solution
 */
void dmc_compute_prediction(dmc_dynamic_t *dyn,
                             mpc_solution_t *solution);

/* ─── Integrating-Specific DMC Extensions ─────────────────────────────── */

/**
 * dmc_integrating_terminal_penalty — Add end-point penalty for stability
 *
 * For integrating processes, nominal stability requires that the predicted
 * level at the end of the horizon is at setpoint:
 *   ŷ(k+N_p|k) = r
 *
 * This is enforced via a large terminal weight (P → ∞) or hard terminal
 * equality constraint. In unconstrained form:
 *   J += P * (ŷ(k+N_p) - r)²
 *
 * with sufficiently large P ensuring the integrator does not drift.
 *
 * Implementation adds terminal row to G and terminal weight to H:
 *   H_term = H + P * g_Np * g_Np^T
 *   c_term = c + P * (f_Np - w_Np) * g_Np
 *
 * where g_Np is the last row of G.
 *
 * Reference: Rawlings & Muske (1993) "Stability of constrained receding
 *            horizon control", IEEE TAC
 *
 * @param dyn      DMC structure
 * @param qp       QP problem (H, c modified)
 * @param tuning   Terminal weight P in tuning
 * Returns 0 on success.
 */
int dmc_integrating_terminal_penalty(dmc_dynamic_t *dyn, qp_problem_t *qp,
                                      const mpc_tuning_t *tuning);

/**
 * dmc_move_blocking — Apply move blocking to reduce QP dimension
 *
 * Move blocking groups future control moves into blocks:
 *   Δu(k) = Δu(k+1) = ... = Δu(k+b-1)  (block 1)
 *   Δu(k+b) = ... = Δu(k+2b-1)         (block 2)
 *   etc.
 *
 * Reduces N_c to N_c / divisor by enforcing piecewise constant Δu.
 * Reduces QP dimension substantially for long horizons.
 *
 * Blocked dynamic matrix: G_blocked = G * M where M is the blocking matrix.
 *
 * Reference: Qin & Badgwell (2003) §3.4 "Move suppression and blocking"
 *            Ricker (1985) "DMC with blocking"
 *
 * @param dyn      DMC structure (dynamic matrix modified)
 * @param tuning   move_blocking_divisor in tuning
 * @param qp       QP problem (dimension reduced)
 * Returns new control horizon on success, -1 on error.
 */
int dmc_move_blocking(dmc_dynamic_t *dyn, const mpc_tuning_t *tuning,
                       qp_problem_t *qp);

/* ─── DMC Initialization and Execution ────────────────────────────────── */

/**
 * dmc_init — Initialize DMC controller state
 *
 * Clears past moves, sets initial free response.
 * Must be called before first dmc_step().
 *
 * @param dyn      DMC structure to initialize
 * @param step     Step response model
 * @param tuning   MPC tuning parameters
 */
void dmc_init(dmc_dynamic_t *dyn, const step_response_t *step,
              const mpc_tuning_t *tuning);

/**
 * dmc_step — Execute one DMC control step (receding horizon)
 *
 * Full DMC algorithm at each sampling instant:
 *   1. Measure current output y(k)
 *   2. Compute free response f
 *   3. Compute reference trajectory w
 *   4. Apply bias correction d(k)
 *   5. Set up QP (H, c, constraints)
 *   6. Solve QP → Δu*
 *   7. Apply Δu*(0) to plant
 *   8. Shift past move buffer for next step
 *
 * Returns the computed Δu to apply.
 *
 * @param solution  Output solution structure
 * @param dyn       DMC state (updated in place)
 * @param step      Step response model
 * @param tuning    MPC tuning
 * @param config    Level constraints
 * @param y_meas    Current measured level
 * @param u_prev    Previous applied MV
 * @param solver    QP solver type
 * Returns computed Δu on success, huge value on error.
 */
double dmc_step(mpc_solution_t *solution, dmc_dynamic_t *dyn,
                const step_response_t *step, const mpc_tuning_t *tuning,
                const mpc_level_config_t *config, double y_meas,
                double u_prev, mpc_solver_type_t solver);

/* ─── Utility ─────────────────────────────────────────────────────────── */

/**
 * dmc_condition_number — Estimate condition number of dynamic matrix
 *
 * Uses power iteration and Rayleigh quotient for σ_max estimate,
 * and inverse power iteration for σ_min estimate.
 *
 * Good for monitoring numerical health of the DMC QP.
 * For integrating processes, cond(G) typically grows as O(N_p²).
 *
 * @param G     Dynamic matrix (column-major)
 * @param N_p   Row dimension
 * @param N_c   Column dimension
 * @param cond  Output: estimated condition number
 * Returns 0 on success.
 */
int dmc_condition_number(const double *G, int N_p, int N_c, double *cond);

#ifdef __cplusplus
}
#endif

#endif /* MPC_DMC_H */
