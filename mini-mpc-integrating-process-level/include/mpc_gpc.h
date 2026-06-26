/**
 * mpc_gpc.h — Generalized Predictive Control for Integrating Processes
 *
 * Domain: Industrial MPC — GPC Algorithm
 *
 * Generalized Predictive Control (Clarke, Mohtadi, Tuffs 1987) is the
 * most influential MPC formulation in academia. Unlike DMC (which uses
 * step response), GPC uses a transfer function (CARIMA) model.
 *
 * The CARIMA model provides inherent integral action through the Δ operator:
 *   A(z⁻¹) * y(k) = z^{-d} * B(z⁻¹) * u(k-1) + ξ(k) / Δ
 *
 * For integrating processes, GPC has specific advantages:
 *   1. The CARIMA differencing operator naturally models integrating dynamics
 *   2. Diophantine recursion is numerically efficient (O(N_p*na))
 *   3. The T-filter provides robustness to model mismatch
 *   4. Pre-stabilized GPC variants fit naturally
 *
 * Knowledge Coverage:
 *   L5 - Algorithms: GPC, Diophantine recursion, CARIMA prediction
 *   L2 - Core Concepts: Transfer function MPC, integral action
 *   L6 - Canonical Problems: Integrating process level GPC
 *
 * Reference: Clarke, Mohtadi, Tuffs (1987) Automatica 23(2):137-160
 *            Clarke & Mohtadi (1989) Automatica 25(6):859-875
 *            Bitmead, Gevers, Wertz (1990) "Adaptive Optimal Control"
 *
 * MIT 2.171 · CMU 24-677 · Oxford (Clarke) · RWTH Aachen
 */

#ifndef MPC_GPC_H
#define MPC_GPC_H

#include "mpc_level_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ─── CARIMA Model Operations ─────────────────────────────────────────── */

/**
 * gpc_carima_predict — Compute CARIMA-based j-step ahead predictions
 *
 * The j-step predictor separates into forced + free responses:
 *   ŷ(k+j|k) = G_j(z⁻¹)*Δu(k+j-1) + F_j(z⁻¹)*y(k) + H_j(z⁻¹)*Δu(k-1)
 *
 * where F_j, G_j, H_j are solved via the Diophantine identity:
 *   1 = E_j(z⁻¹)*A(z⁻¹)*Δ + z^{-j}*F_j(z⁻¹)        [for F_j, E_j]
 *   E_j(z⁻¹)*B(z⁻¹) = G_j(z⁻¹) + z^{-j}*H_j(z⁻¹)    [for G_j, H_j]
 *
 *   deg(E_j) = j-1, deg(F_j) = na
 *   deg(G_j) = j-1, deg(H_j) = nb-1
 *
 * The Diophantine equations are solved recursively:
 *   E_{j+1} = E_j + f_{j,0} * z^{-j}
 *   F_{j+1}(z⁻¹) = z*(F_j(z⁻¹) - f_{j,0}*A(z⁻¹)*Δ(z⁻¹))
 *
 * @param gpc    GPC configuration (A, B polynomials)
 * @param y_meas Current output y(k)
 * @param du_past Array of past Δu values (oldest first)
 * @param n_past  Length of du_past array
 * @param N_p    Prediction horizon
 * @param y_pred Output: predicted outputs [ŷ(k+1), ..., ŷ(k+N_p)]
 * @param f_free Output: free response part (no future Δu) [f₁, ..., f_Np]
 * @param G_mat  Output: G matrix (N_p × N_c, col-major), forced response map
 * @param N_c    Control horizon
 * Returns 0 on success.
 *
 * Complexity: O(N_p * (na + nb)), dominated by recursion.
 */
int gpc_carima_predict(const gpc_config_t *gpc, double y_meas,
                        const double *du_past, int n_past,
                        int N_p, double *y_pred, double *f_free,
                        double *G_mat, int N_c);

/**
 * gpc_diophantine_recursion — Solve Diophantine equation for GPC
 *
 * Solves: 1 = E_j * A̅ + z^{-j} * F_j
 * where A̅ = A(z⁻¹)*Δ = A(z⁻¹)*(1-z⁻¹)
 *
 * The recursion (Clarke et al. 1987 §4):
 *   For j = 1:
 *     E₁ = 1
 *     F₁ = z*(1 - A̅)  →  f₁(z⁻¹) = polynomial in z⁻¹ of degree na
 *
 *   For j > 1:
 *     E_j = E_{j-1} + f_{j-1,0} * z^{-(j-1)}
 *     F_j(z⁻¹) = z*(F_{j-1}(z⁻¹) - f_{j-1,0}*A̅(z⁻¹))
 *     where f_{j-1,0} is the constant term of F_{j-1}
 *
 * This recursion is the computational core of GPC, avoiding explicit
 * polynomial division at each step.
 *
 * @param A_tilde   Augmented A polynomial A̅ = A*Δ (na+2 coefs)
 * @param na_tilde  Degree of A̅ (= na + 1)
 * @param j         Step index
 * @param E         Output: E_j coefficients [j elements]
 * @param F         Output: F_j coefficients [na_tilde+1 elements]
 * @param f0        Output: leading coefficient f_{j,0}
 * Returns 0 on success.
 */
int gpc_diophantine_recursion(const double *A_tilde, int na_tilde, int j,
                               double *E, double *F, double *f0);

/**
 * gpc_build_G_matrix — Build the GPC forced response matrix G
 *
 * G_ij = g_{i-j+1} where g_k are the step response coefficients of
 * the CARIMA model (B/A*Δ in the forward path).
 *
 * For j-step ahead:
 *   ŷ(k+j|k) = Σ_{i=1}^{j} g_i * Δu(k+j-i) + f_j
 *
 * So G is lower-triangular Toeplitz (same structure as DMC).
 * The g_i are computed from: E_j(z⁻¹)*B(z⁻¹) = Σ g_i*z^{-i}
 *
 * @param gpc     GPC configuration (B polynomial)
 * @param E_coeffs E_j polynomial coefficients [N_p × (N_p)] (row-major)
 * @param N_p     Prediction horizon
 * @param N_c     Control horizon
 * @param G       Output: G matrix (N_p × N_c, col-major)
 */
void gpc_build_G_matrix(const gpc_config_t *gpc, const double *E_coeffs,
                         int N_p, int N_c, double *G);

/**
 * gpc_free_response — Compute GPC free response f_j
 *
 * f_j = F_j(z⁻¹)*y(k) + H_j(z⁻¹)*Δu(k-1)
 *
 * where F_j and H_j come from the Diophantine identities.
 * f_j is what ŷ(k+j) would be if all future Δu = 0.
 *
 * @param gpc     GPC config
 * @param y_meas  y(k)
 * @param du_past Δu(k-1), ... (nb elements needed)
 * @param n_past  Length of past data
 * @param F_coeffs F_j polynomials [N_p × (na+2)] (row-major)
 * @param N_p     Prediction horizon
 * @param f       Output: free response vector
 */
void gpc_free_response(const gpc_config_t *gpc, double y_meas,
                        const double *du_past, int n_past,
                        const double *F_coeffs, int N_p, double *f);

/* ─── GPC Control Law ────────────────────────────────────────────────── */

/**
 * gpc_control_law — Compute GPC control move
 *
 * Minimizes cost (unconstrained case):
 *   J = Σ_{j=N1}^{N2} [ŷ(k+j|k) - w(k+j)]² + λ * Σ_{j=1}^{Nu} [Δu(k+j-1)]²
 *
 * Solution (receding horizon, only first move applied):
 *   Δu(k) = k_gpc^T * (w - f)
 *   where k_gpc^T = first row of (G^T*G + λ*I)^{-1}*G^T
 *
 * The gain vector k_gpc is pre-computed (constant for linear time-invariant
 * model). For integrating processes, k_gpc exists and is finite as long as
 * λ > 0 (guarantees G^T*G + λ*I is invertible).
 *
 * @param gpc     GPC configuration
 * @param G       Forced response matrix
 * @param f       Free response
 * @param w       Reference trajectory
 * @param N_p     Prediction horizon
 * @param N_c     Control horizon
 * @param lambda  Control weighting λ
 * @param du      Output: computed Δu(k)
 * Returns 0 on success.
 */
int gpc_control_law(const gpc_config_t *gpc, const double *G,
                     const double *f, const double *w,
                     int N_p, int N_c, double lambda, double *du);

/**
 * gpc_compute_gain — Pre-compute the GPC gain vector
 *
 * K_gpc = e₁^T * (G^T*G + λ*I)^{-1} * G^T
 *
 * where e₁ = [1, 0, ..., 0]^T selects first row.
 *
 * This gain is constant for fixed model and tuning.
 * In industrial GPC (e.g., Adersa PFC, AspenTech Apollo),
 * the gain vector is computed offline and stored.
 *
 * @param G       G matrix (N_p × N_c)
 * @param N_p     Prediction horizon
 * @param N_c     Control horizon
 * @param lambda  Control weight
 * @param K       Output: gain vector [N_p], K[j] = weight on w_{j+1} - f_{j+1}
 * Returns 0 on success.
 */
int gpc_compute_gain(const double *G, int N_p, int N_c, double lambda,
                      double *K);

/* ─── GPC for Integrating Processes ───────────────────────────────────── */

/**
 * gpc_integrating_setup — Setup GPC for pure integrating process
 *
 * For G(s) = K/s:
 *   Discrete ZOH: G(z) = K*Ts / (z-1)
 *   CARIMA: A(z⁻¹) = 1 - z⁻¹, B(z⁻¹) = K*Ts, delay d = 1
 *
 * Augmented A̅ = A*Δ = (1-z⁻¹)² = 1 - 2z⁻¹ + z⁻²
 * This double differencing naturally handles the integrating dynamics.
 *
 * @param gpc    Output: populated GPC config
 * @param K      Process gain
 * @param Ts     Sampling time
 * @param N_p    Prediction horizon
 * @param N_c    Control horizon
 * @param lambda Control weight
 * Returns 0 on success.
 */
int gpc_integrating_setup(gpc_config_t *gpc, double K, double Ts,
                           int N_p, int N_c, double lambda);

/**
 * gpc_integrating_with_lag_setup — GPC for integrating + lag process
 *
 * G(s) = K / [s*(τ*s+1)]
 *
 * Discrete (ZOH): G(z) = K * [Ts*(z-α) - τ*(1-α)*(z-1)] / [(z-1)*(z-α)]
 * where α = exp(-Ts/τ)
 *
 * CARIMA: A(z⁻¹) = (1-z⁻¹)*(1-α*z⁻¹), B(z⁻¹) = b₀ + b₁*z⁻¹
 *
 * @param gpc    Output GPC config
 * @param K      Gain
 * @param tau    Time constant [s]
 * @param Ts     Sampling time
 * @param N_p    Prediction horizon
 * @param N_c    Control horizon
 * @param lambda Control weight
 * Returns 0 on success.
 */
int gpc_integrating_with_lag_setup(gpc_config_t *gpc, double K, double tau,
                                    double Ts, int N_p, int N_c, double lambda);

/* ─── GPC Step Execution ──────────────────────────────────────────────── */

/**
 * gpc_step — Single GPC control step
 *
 * Full cycle at each sampling instant for integrating process level control:
 *   1. Read current level y(k)
 *   2. Compute free response f_j using Diophantine recursion
 *   3. Build reference trajectory w (exponential filter to SP)
 *   4. Compute Δu*(k) using pre-computed GPC gain K
 *   5. (Optional) Check constraints → if violated, solve constrained GPC QP
 *   6. Apply Δu(k) = max(du_min, min(du_max, Δu*(k)))
 *   7. Shift past data buffers
 *
 * @param solution  Output solution
 * @param gpc       GPC configuration (updated past data internally)
 * @param y_meas    Current level measurement
 * @param y_sp      Level setpoint
 * @param u_prev    Previous valve position
 * @param du_past   Array of past Δu (length ≥ nb+1)
 * @param y_past    Array of past y (length ≥ na+2)
 * Returns Δu to apply.
 */
double gpc_step(mpc_solution_t *solution, gpc_config_t *gpc,
                double y_meas, double y_sp, double u_prev,
                double *du_past, double *y_past);

/* ─── GPC T-Filter Design ─────────────────────────────────────────────── */

/**
 * gpc_design_T_filter — Design T-filter for robustness
 *
 * The T-filter modifies the noise model C(z⁻¹)/Δ to T(z⁻¹)/Δ where:
 *   T(z⁻¹) = (1 - β*z⁻¹)^{n_T}
 *
 * T-filter effects:
 *   - β → 1: narrow bandwidth → robust to high-freq mismatch → sluggish
 *   - β → 0: wide bandwidth → aggressive → sensitive to mismatch
 *
 * The T-filter provides a "detuning knob" for robustness against
 * unmodeled dynamics, analogous to λ in the objective function but
 * with better frequency-domain interpretation.
 *
 * Reference: Yoon & Clarke (1995) Automatica 31(3):403-420
 *
 * @param gpc    GPC config (C_coeff modified)
 * @param beta   Filter pole, e.g., 0.8 (typical)
 * @param order  Filter order n_T, typically 1 or 2
 */
void gpc_design_T_filter(gpc_config_t *gpc, double beta, int order);

#ifdef __cplusplus
}
#endif

#endif /* MPC_GPC_H */
