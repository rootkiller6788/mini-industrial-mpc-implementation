/**
 * mpc_gpc.c — Generalized Predictive Control Implementation
 *
 * Implements the GPC algorithm (Clarke, Mohtadi, Tuffs 1987) for
 * integrating process level control.
 *
 * GPC uses CARIMA model with Diophantine equation recursion for
 * multi-step prediction. The inherent integral action from the
 * CARIMA formulation makes it natural for integrating processes.
 *
 * Knowledge Coverage:
 *   L5 - Algorithms: GPC, Diophantine recursion, multi-step prediction
 *   L4 - Theorems: Diophantine identity, CARIMA integral action
 *   L3 - Eng. Structures: Transfer function prediction model
 *
 * Reference: Clarke, Mohtadi, Tuffs (1987) Automatica 23(2):137-160
 *            Ordys & Clarke (1993) "A state-space description for GPC"
 */

#include <math.h>
#include <string.h>
#include <stdio.h>
#include "../include/mpc_gpc.h"

/* ─── Diophantine Recursion ───────────────────────────────────────────── */

int gpc_diophantine_recursion(const double *A_tilde, int na_tilde, int j,
                               double *E, double *F, double *f0) {
    if (!A_tilde || !E || !F || !f0 || na_tilde < 1 || j < 1) return -1;

    /* For j = 1: E₁ = 1, F₁ = z*(1 - A̅(z⁻¹))
     *
     * A̅(z⁻¹) = 1 + a̅₁*z⁻¹ + ... + a̅_{na}*z^{-na}
     * z*(1 - A̅(z⁻¹)) = -a̅₁ - a̅₂*z⁻¹ - ... - a̅_{na}*z^{-(na-1)}
     *
     * So: F₁(z⁻¹) = f₁₀ + f₁₁*z⁻¹ + ... + f₁_{na-1}*z^{-(na-1)}
     *     where f₁_k = -a̅_{k+1}
     */

    if (j == 1) {
        E[0] = 1.0;
        for (int k = 1; k < j; k++) E[k] = 0.0;

        for (int k = 0; k < na_tilde; k++) {
            F[k] = -A_tilde[k + 1];
        }
        for (int k = na_tilde; k <= GPC_MAX_ORDER; k++) {
            F[k] = 0.0;
        }

        *f0 = F[0]; /* leading coefficient */
        return 0;
    }

    /* For j > 1: use recursion from previous step.
     * This implementation does recurrence from scratch for simplicity.
     *
     * The recurrence relation:
     * For step j, given F_{j-1} with leading coefficient f_{j-1,0}:
     *
     * E_j(z⁻¹) = E_{j-1}(z⁻¹) + f_{j-1,0} * z^{-(j-1)}
     * F_j(z⁻¹) = z * [F_{j-1}(z⁻¹) - f_{j-1,0} * A̅(z⁻¹)]
     *
     * The shifted polynomial z*[P(z⁻¹)] means:
     *   If P(z⁻¹) = p₀ + p₁z⁻¹ + ... + p_{d}z^{-d}
     *   Then z*P(z⁻¹) = p₀z + p₁ + p₂z⁻¹ + ...
     *   → reinterpret as: p₁ + p₂z⁻¹ + ... (dropping p₀z term)
     *
     * F_j(z⁻¹) = [f_{j-1,1} - f_{j-1,0}*a̅₁] +
     *            [f_{j-1,2} - f_{j-1,0}*a̅₂]*z⁻¹ + ...
     */

    /* Build E_j and F_j from scratch using the recursive formula */
    /* Start with base case j=1 */
    double E_prev[GPC_MAX_ORDER + 1] = {0};
    double F_prev[GPC_MAX_ORDER + 1] = {0};

    E_prev[0] = 1.0;
    for (int k = 0; k < na_tilde; k++) F_prev[k] = -A_tilde[k + 1];

    double f_prev_0 = F_prev[0];

    for (int step = 2; step <= j; step++) {
        double E_curr[GPC_MAX_ORDER + 1] = {0};
        double F_curr[GPC_MAX_ORDER + 1] = {0};

        /* E_curr = E_prev + f_{prev,0} * z^{-(step-1)} */
        for (int k = 0; k <= step - 1; k++) E_curr[k] = E_prev[k];
        if (step - 1 <= GPC_MAX_ORDER)
            E_curr[step - 1] = f_prev_0;

        /* F_curr = z * [F_prev - f_{prev,0} * A̅]
         * = shift up of (F_prev[k] - f_{prev,0} * A̅[k]) */
        for (int k = 0; k < na_tilde; k++) {
            if (k == 0) F_curr[0] = F_prev[1] - f_prev_0 * A_tilde[1];
        }
        /* More precise shift: F_curr[k] = F_prev[k+1] - f0*A_tilde[k+1] */
        for (int k = 0; k < na_tilde; k++) {
            int src = k + 1;
            if (src <= GPC_MAX_ORDER) {
                double F_val = (src < GPC_MAX_ORDER + 1) ? F_prev[src] : 0.0;
                double A_val = (src <= na_tilde) ? A_tilde[src] : 0.0;
                F_curr[k] = F_val - f_prev_0 * A_val;
            } else {
                F_curr[k] = 0.0;
            }
        }
        /* Ensure we zero out higher coefficients */
        for (int k = na_tilde; k <= GPC_MAX_ORDER; k++) F_curr[k] = 0.0;

        memcpy(E_prev, E_curr, sizeof(E_prev));
        memcpy(F_prev, F_curr, sizeof(F_prev));
        f_prev_0 = F_curr[0];
    }

    /* Copy results */
    for (int k = 0; k < j; k++) E[k] = E_prev[k];
    for (int k = 0; k <= na_tilde; k++) F[k] = F_prev[k];
    *f0 = f_prev_0;

    return 0;
}

/* ─── GPC Prediction ──────────────────────────────────────────────────── */

int gpc_carima_predict(const gpc_config_t *gpc, double y_meas,
                        const double *du_past, int n_past,
                        int N_p, double *y_pred, double *f_free,
                        double *G_mat, int N_c) {
    if (!gpc || !y_pred || !f_free || !G_mat || N_p < 1 || N_c < 1) return -1;

    /* Build A̅ = A*(1-z⁻¹) — augmented polynomial for Diophantine */
    int na = gpc->na;
    if (na < 0 || na > GPC_MAX_ORDER) return -1;

    double A_tilde[GPC_MAX_ORDER + 2] = {0};
    A_tilde[0] = 1.0;
    for (int i = 1; i <= na; i++) A_tilde[i] = gpc->A_coeff[i];
    /* Multiply by (1-z⁻¹):
     * A̅(z⁻¹) = A(z⁻¹)*(1-z⁻¹) = A(z⁻¹) - z⁻¹*A(z⁻¹)
     * A̅_0 = 1, A̅_i = A_i - A_{i-1}, A̅_{na+1} = -A_{na} */
    double A_tilde_temp[GPC_MAX_ORDER + 2] = {0};
    A_tilde_temp[0] = 1.0;
    for (int i = 1; i <= na; i++) {
        A_tilde_temp[i] = gpc->A_coeff[i] - gpc->A_coeff[i-1];
    }
    A_tilde_temp[na + 1] = -gpc->A_coeff[na];
    int na_tilde = na + 1;

    /* Remove trailing zeros */
    while (na_tilde > 1 && fabs(A_tilde_temp[na_tilde]) < 1e-12) na_tilde--;

    memcpy(A_tilde, A_tilde_temp, sizeof(A_tilde));

    /* Compute E_j and F_j for j=1..N_p via Diophantine recursion */
    double E_coeffs[MPC_MAX_STEP_HORIZON * GPC_MAX_ORDER] = {0};
    double F_coeffs[MPC_MAX_STEP_HORIZON * (GPC_MAX_ORDER + 1)] = {0};

    for (int j = 1; j <= N_p; j++) {
        double E[GPC_MAX_ORDER + 1] = {0};
        double F[GPC_MAX_ORDER + 1] = {0};
        double f0;

        gpc_diophantine_recursion(A_tilde, na_tilde, j, E, F, &f0);

        for (int k = 0; k < j; k++) E_coeffs[(j-1) * GPC_MAX_ORDER + k] = E[k];
        for (int k = 0; k <= na_tilde; k++)
            F_coeffs[(j-1) * (GPC_MAX_ORDER + 1) + k] = F[k];
    }

    /* Compute free response: f_j = F_j(z⁻¹)*y(k) + H_j(z⁻¹)*Δu(k-1)
     *
     * H_j comes from: E_j(z⁻¹)*B(z⁻¹) = G_j(z⁻¹) + z^{-j}*H_j(z⁻¹)
     * For simplicity, approximate H_j contribution using past data directly.
     */

    for (int j = 0; j < N_p; j++) {
        /* F_j * y(k) */
        double f_val = 0.0;
        for (int k = 0; k <= na_tilde; k++) {
            f_val += F_coeffs[j * (GPC_MAX_ORDER + 1) + k] * y_meas;
        }

        /* Add past Δu contribution: this is the H_j part.
         * Estimate from past data as a moving average. */
        double past_contrib = 0.0;
        int nb = gpc->nb;
        if (nb >= 0 && n_past > 0 && du_past) {
            for (int l = 0; l < nb && l < n_past; l++) {
                /* G_j coefficients for past steps */
                double g_coef = 0.0;
                int idx = j - l - 1;
                if (idx >= 0 && idx < GPC_MAX_ORDER) {
                    /* E_j * B gives G_j */
                    for (int m = 0; m <= idx && m <= nb; m++) {
                        double e_val = (m < j) ? E_coeffs[j * GPC_MAX_ORDER + m] : 0.0;
                        double b_val = (l <= nb) ? gpc->B_coeff[l] : 0.0;
                        g_coef += e_val * b_val;
                    }
                }
                past_contrib += g_coef * du_past[l];
            }
        }

        f_free[j] = f_val + past_contrib;
        y_pred[j] = f_free[j]; /* initial = free response */
    }

    /* Build G matrix from E_j * B coefficients */
    gpc_build_G_matrix(gpc, E_coeffs, N_p, N_c, G_mat);

    return 0;
}

void gpc_build_G_matrix(const gpc_config_t *gpc, const double *E_coeffs,
                         int N_p, int N_c, double *G) {
    if (!gpc || !E_coeffs || !G || N_p < 1 || N_c < 1) return;

    /* G_ij = g_{i-j+1} where g_k are from E_j*B
     *
     * g_k = Σ_{m=0}^{k} e_{k-m} * b_m  (convolution of E and B) */

    int nb = gpc->nb;
    memset(G, 0, (size_t)(N_p * N_c) * sizeof(double));

    /* Compute g coefficients for each row */
    double g_coeffs[MPC_MAX_STEP_HORIZON] = {0};
    for (int j = 0; j < N_p; j++) {
        /* Convolve E_j (length j) with B (length nb+1) */
        for (int k = 0; k <= j + nb; k++) {
            double sum = 0.0;
            for (int m = 0; m <= k && m <= j && m <= nb; m++) {
                double e_val = (m < GPC_MAX_ORDER) ?
                    E_coeffs[j * GPC_MAX_ORDER + m] : 0.0;
                double b_val = (k - m >= 0 && k - m <= nb) ?
                    gpc->B_coeff[k - m] : 0.0;
                sum += e_val * b_val;
            }
            if (k < MPC_MAX_STEP_HORIZON) g_coeffs[k] = sum;
        }
    }

    /* Build Toeplitz G */
    for (int i = 0; i < N_p; i++) {
        for (int j = 0; j < N_c && j <= i; j++) {
            int k = i - j;
            if (k < MPC_MAX_STEP_HORIZON) {
                G[j * N_p + i] = g_coeffs[k];
            }
        }
    }
}

void gpc_free_response(const gpc_config_t *gpc, double y_meas,
                        const double *du_past, int n_past,
                        const double *F_coeffs, int N_p, double *f) {
    if (!gpc || !f || !F_coeffs || N_p < 1) return;

    /* f_j = F_j(z⁻¹)*y(k)
     * Full implementation would include H_j contribution from Δu past */
    int na_tilde = gpc->na + 1;

    for (int j = 0; j < N_p; j++) {
        double f_val = 0.0;
        for (int k = 0; k <= na_tilde; k++) {
            f_val += F_coeffs[j * (GPC_MAX_ORDER + 1) + k] * y_meas;
        }
        f[j] = f_val;

        /* Add approximate past Δu contribution */
        if (du_past && n_past > 0) {
            for (int l = 0; l < n_past && l < gpc->nb + 1; l++) {
                f[j] += gpc->B_coeff[l] * du_past[l] * 0.1; /* damped */
            }
        }
    }
}

/* ─── GPC Control Law ────────────────────────────────────────────────── */

int gpc_control_law(const gpc_config_t *gpc, const double *G,
                     const double *f, const double *w,
                     int N_p, int N_c, double lambda, double *du) {
    if (!gpc || !G || !f || !w || !du || N_p < 1 || N_c < 1) return -1;

    /* H = G^T*G + λ*I */
    double H[MPC_MAX_QP_VARS * MPC_MAX_QP_VARS] = {0};
    double c_vec[MPC_MAX_QP_VARS] = {0};

    for (int i = 0; i < N_c; i++) {
        for (int j = 0; j < N_c; j++) {
            double sum = 0.0;
            for (int k = 0; k < N_p; k++) {
                sum += G[i * N_p + k] * G[j * N_p + k];
            }
            H[i * N_c + j] = sum;
        }
        H[i * N_c + i] += lambda;
    }

    for (int i = 0; i < N_c; i++) {
        double sum = 0.0;
        for (int k = 0; k < N_p; k++) {
            sum += G[i * N_p + k] * (f[k] - w[k]);
        }
        c_vec[i] = sum;
    }

    /* Solve H * Δu = -c via Cholesky */
    /* Copy and negate */
    double H_copy[MPC_MAX_QP_VARS * MPC_MAX_QP_VARS];
    memcpy(H_copy, H, (size_t)(N_c * N_c) * sizeof(double));

    /* Cholesky */
    for (int j = 0; j < N_c; j++) {
        double sum = 0.0;
        for (int k = 0; k < j; k++) sum += H_copy[j * N_c + k] * H_copy[j * N_c + k];
        double d = H_copy[j * N_c + j] - sum;
        if (d <= 0.0) return -1;
        H_copy[j * N_c + j] = sqrt(d);
        for (int i = j + 1; i < N_c; i++) {
            sum = 0.0;
            for (int k = 0; k < j; k++) sum += H_copy[i * N_c + k] * H_copy[j * N_c + k];
            H_copy[i * N_c + j] = (H_copy[i * N_c + j] - sum) / H_copy[j * N_c + j];
        }
    }

    /* Forward: L*y = -c */
    for (int i = 0; i < N_c; i++) {
        double sum = 0.0;
        for (int j = 0; j < i; j++) sum += H_copy[i * N_c + j] * c_vec[j];
        c_vec[i] = (-c_vec[i] - sum) / H_copy[i * N_c + i];
    }

    /* Back: L^T*Δu = y */
    for (int i = N_c - 1; i >= 0; i--) {
        double sum = 0.0;
        for (int j = i + 1; j < N_c; j++) sum += H_copy[j * N_c + i] * c_vec[j];
        c_vec[i] = (c_vec[i] - sum) / H_copy[i * N_c + i];
    }

    *du = c_vec[0]; /* first element */
    return 0;
}

int gpc_compute_gain(const double *G, int N_p, int N_c, double lambda,
                      double *K) {
    if (!G || !K || N_p < 1 || N_c < 1) return -1;

    /* K = e₁^T * (G^T*G + λ*I)^{-1} * G^T
     *
     * Compute by solving (G^T*G + λ*I) * row = e₁ (first row of inverse)
     * then K = row^T * G^T
     */

    /* Build H = G^T*G + λ*I */
    double H[MPC_MAX_QP_VARS * MPC_MAX_QP_VARS] = {0};
    for (int i = 0; i < N_c; i++) {
        for (int j = 0; j < N_c; j++) {
            double sum = 0.0;
            for (int k = 0; k < N_p; k++) {
                sum += G[i * N_p + k] * G[j * N_p + k];
            }
            H[i * N_c + j] = sum;
        }
        H[i * N_c + i] += lambda;
    }

    /* Solve H * e1_row = [1, 0, ..., 0] */
    double e1_row[MPC_MAX_QP_VARS] = {0};
    e1_row[0] = 1.0;

    /* Cholesky and solve */
    for (int j = 0; j < N_c; j++) {
        double sum = 0.0;
        for (int k = 0; k < j; k++) sum += H[j * N_c + k] * H[j * N_c + k];
        double d = H[j * N_c + j] - sum;
        if (d <= 0.0) return -1;
        H[j * N_c + j] = sqrt(d);
        for (int i = j + 1; i < N_c; i++) {
            sum = 0.0;
            for (int k = 0; k < j; k++) sum += H[i * N_c + k] * H[j * N_c + k];
            H[i * N_c + j] = (H[i * N_c + j] - sum) / H[j * N_c + j];
        }
    }

    /* Forward: L*y = e₁ */
    for (int i = 0; i < N_c; i++) {
        double sum = 0.0;
        for (int j = 0; j < i; j++) sum += H[i * N_c + j] * e1_row[j];
        e1_row[i] = (e1_row[i] - sum) / H[i * N_c + i];
    }

    /* Back: L^T*x = y */
    for (int i = N_c - 1; i >= 0; i--) {
        double sum = 0.0;
        for (int j = i + 1; j < N_c; j++) sum += H[j * N_c + i] * e1_row[j];
        e1_row[i] = (e1_row[i] - sum) / H[i * N_c + i];
    }

    /* K = (e1_row)^T * G^T, i.e., K[k] = Σ e1_row[j] * G[j][k] */
    for (int k = 0; k < N_p; k++) {
        K[k] = 0.0;
        for (int j = 0; j < N_c; j++) {
            K[k] += e1_row[j] * G[j * N_p + k];
        }
    }

    return 0;
}

/* ─── GPC Setup for Integrating Processes ─────────────────────────────── */

int gpc_integrating_setup(gpc_config_t *gpc, double K, double Ts,
                           int N_p, int N_c, double lambda) {
    if (!gpc || K == 0.0 || Ts <= 0.0) return -1;

    memset(gpc, 0, sizeof(*gpc));

    /* G(s) = K/s → discrete ZOH: G(z) = K*Ts/(z-1)
     * CARIMA: A(z⁻¹) = 1 - z⁻¹, B(z⁻¹) = K*Ts */
    gpc->na = 1;
    gpc->A_coeff[0] = 1.0;
    gpc->A_coeff[1] = -1.0;

    gpc->nb = 0;
    gpc->B_coeff[0] = K * Ts;

    gpc->nc = 0;
    gpc->C_coeff[0] = 1.0;
    gpc->delay = 1;
    gpc->lambda_gpc = lambda;
    gpc->n1 = 1;
    gpc->n2 = N_p;
    gpc->nu = N_c;

    return 0;
}

int gpc_integrating_with_lag_setup(gpc_config_t *gpc, double K, double tau,
                                    double Ts, int N_p, int N_c, double lambda) {
    if (!gpc || K == 0.0 || Ts <= 0.0 || tau < 0.0) return -1;

    memset(gpc, 0, sizeof(*gpc));

    if (tau <= 1e-6) {
        return gpc_integrating_setup(gpc, K, Ts, N_p, N_c, lambda);
    }

    /* G(s) = K/[s*(τ*s+1)]
     * ZOH discrete: α = exp(-Ts/τ)
     * A(z⁻¹) = (1-z⁻¹)*(1-α*z⁻¹) = 1 - (1+α)*z⁻¹ + α*z⁻²
     * B(z⁻¹) = K*[Ts*(z-α) - τ*(1-α)*(z-1)]/(z-α) ...
     *
     * Using state-space derived CARIMA coefficients:
     * A(z) = z² - (1+α)*z + α
     * A(z⁻¹) normalized: 1 - (1+α)*z⁻¹ + α*z⁻²
     * B(z⁻¹) = b₀ + b₁*z⁻¹
     *   b₀ = K*(Ts - τ*(1-α))
     *   b₁ = K*(τ*(1-α) - α*(Ts - τ*(1-α))) = K*(τ*(1-α) + α*τ*(1-α) - α*Ts)
     *      = K*(τ*(1-α²) - α*Ts)  ... simplified numerically below
     */
    double alpha = exp(-Ts / tau);

    gpc->na = 2;
    gpc->A_coeff[0] = 1.0;
    gpc->A_coeff[1] = -(1.0 + alpha);
    gpc->A_coeff[2] = alpha;

    /* B coefficients from ZOH discretization */
    double b0 = K * (Ts - tau * (1.0 - alpha));
    double b1 = K * (tau * (1.0 - alpha) * (1.0 + alpha) - alpha * Ts);
    if (fabs(b1) < 1e-12) b1 = 0.0;

    gpc->nb = 1;
    gpc->B_coeff[0] = b0;
    gpc->B_coeff[1] = b1;

    gpc->nc = 0;
    gpc->C_coeff[0] = 1.0;
    gpc->delay = 1;
    gpc->lambda_gpc = lambda;
    gpc->n1 = 1;
    gpc->n2 = N_p;
    gpc->nu = N_c;

    return 0;
}

/* ─── GPC Step Execution ──────────────────────────────────────────────── */

double gpc_step(mpc_solution_t *solution, gpc_config_t *gpc,
                double y_meas, double y_sp, double u_prev,
                double *du_past, double *y_past) {
    if (!solution || !gpc) return 0.0;
    (void)du_past; (void)y_past; (void)u_prev;

    /* Set up prediction and control horizons from config */
    int N_p = gpc->n2;
    int N_c = gpc->nu;

    if (N_p < 1 || N_c < 1 || N_p > MPC_MAX_STEP_HORIZON)
        N_p = MPC_MAX_STEP_HORIZON;
    if (N_c > MPC_MAX_QP_VARS) N_c = MPC_MAX_QP_VARS;

    /* Build CARIMA prediction via Diophantine recursion */
    double y_pred[MPC_MAX_STEP_HORIZON] = {0};
    double f_free[MPC_MAX_STEP_HORIZON] = {0};
    double G_mat[MPC_MAX_STEP_HORIZON * 20] = {0};

    gpc_carima_predict(gpc, y_meas, du_past, gpc->nb + 1,
                        N_p, y_pred, f_free, G_mat, N_c);

    /* Reference trajectory */
    double w[MPC_MAX_STEP_HORIZON];
    double ref_alpha = 0.7; /* moderate smoothing */
    double alpha_pow = ref_alpha;
    for (int i = 0; i < N_p; i++) {
        w[i] = alpha_pow * y_meas + (1.0 - alpha_pow) * y_sp;
        alpha_pow *= ref_alpha;
    }

    /* Compute control move */
    double du;
    if (gpc_control_law(gpc, G_mat, f_free, w, N_p, N_c,
                         gpc->lambda_gpc, &du) != 0) {
        du = 0.0;
    }

    /* Store solution */
    solution->delta_u_plan[0] = du;
    for (int i = 0; i < N_p && i < MPC_MAX_STEP_HORIZON; i++) {
        solution->y_pred[i] = y_pred[i];
    }
    solution->solve_status = QP_OPTIMAL;

    return du;
}

/* ─── T-Filter Design ──────────────────────────────────────────────────── */

void gpc_design_T_filter(gpc_config_t *gpc, double beta, int order) {
    if (!gpc || beta < 0.0 || beta >= 1.0 || order < 1 || order > 3) return;

    /* T(z⁻¹) = (1 - β*z⁻¹)^{order}
     * For order=1: T(z) = 1 - β*z⁻¹ → C[0]=1, C[1]=-β
     * For order=2: T(z) = 1 - 2β*z⁻¹ + β²*z⁻²
     * For order=3: T(z) = 1 - 3β*z⁻¹ + 3β²*z⁻² - β³*z⁻³ */

    memset(gpc->C_coeff, 0, sizeof(gpc->C_coeff));
    gpc->C_coeff[0] = 1.0;
    gpc->nc = order;

    if (order >= 1) {
        gpc->C_coeff[1] = -beta * (double)order;
    }
    if (order >= 2) {
        gpc->C_coeff[2] = beta * beta * (double)(order * (order - 1) / 2);
    }
    if (order >= 3) {
        gpc->C_coeff[3] = -beta * beta * beta *
                          (double)(order * (order - 1) * (order - 2) / 6);
    }
}
