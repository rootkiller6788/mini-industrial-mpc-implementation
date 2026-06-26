/**
 * mpc_dmc.c — Dynamic Matrix Control Implementation
 *
 * Implements the DMC algorithm for integrating processes.
 * Core features: dynamic matrix construction, free response computation,
 * unconstrained and constrained QP setup, receding horizon execution.
 *
 * Knowledge Coverage:
 *   L2 - Core Concepts: receding horizon, prediction, feedforward bias
 *   L3 - Eng. Structures: Toeplitz dynamic matrix, QP setup
 *   L5 - Algorithms: DMC, Cholesky, constrained QP assembly, move blocking
 *   L4 - Theorems: terminal penalty for stability, condition number analysis
 *
 * Reference: Cutler & Ramaker (1979), Garcia & Morshedi (1984)
 */

#include <math.h>
#include <string.h>
#include <stdio.h>
#include "../include/mpc_dmc.h"
#include "../include/mpc_qp_solver.h"
#include "../include/mpc_level_constraints.h"

/* ─── Dynamic Matrix Construction ─────────────────────────────────────── */

int dmc_build_dynamic_matrix(double *G, const step_response_t *step,
                              int N_p, int N_c) {
    if (!G || !step || N_p < 1 || N_c < 1 || N_c > N_p) return -1;
    if (N_p > MPC_MAX_STEP_HORIZON || N_c > MPC_MAX_QP_VARS) return -1;

    /* G[i][j] = s_{i-j+1} for i >= j, else 0
     * Column-major storage: G[j*N_p + i] = element (i, j) */
    memset(G, 0, (size_t)(N_p * N_c) * sizeof(double));

    for (int j = 0; j < N_c; j++) {
        for (int i = j; i < N_p; i++) {
            int s_idx = i - j;
            if (s_idx < step->n_coeffs) {
                G[j * N_p + i] = step->coeff[s_idx];
            } else {
                /* Beyond model horizon: extrapolate using ramp
                 * For integrating: s_{k} ≈ s_{N} + K*Ts*(k-N) */
                double slope = (step->coeff[step->n_coeffs - 1] -
                                (step->n_coeffs > 1 ?
                                 step->coeff[step->n_coeffs - 2] : 0.0));
                G[j * N_p + i] = step->coeff[step->n_coeffs - 1] +
                                 slope * (double)(s_idx - step->n_coeffs + 1);
            }
        }
    }

    return 0;
}

int dmc_build_free_response(dmc_dynamic_t *dyn, const step_response_t *step,
                             double y_meas, const double *delta_u_past,
                             int n_past) {
    if (!dyn || !step) return -1;

    int N_p = dyn->n_prediction;
    if (N_p > MPC_MAX_STEP_HORIZON) return -1;

    /* f_i = y_meas + Σ_{l=1}^{N} (s_{i+l} - s_l) * Δu(k-l)
     *
     * This formula uses the "additive form" of the DMC prediction.
     * Implementation uses the impulse response coefficients:
     *   Δs_l = s_l - s_{l-1}  (with s₀ = 0)
     *
     * For integrating process, the impulse response does not decay to zero,
     * but the summation over finite past data remains well-defined.
     */

    int N = step->n_coeffs;
    int n_eff = (n_past < N) ? n_past : N;

    for (int i = 0; i < N_p; i++) {
        double f_i = y_meas;

        for (int l = 0; l < n_eff; l++) {
            int idx_large = i + l + 1;
            double s_large, s_small;

            if (idx_large < N) {
                s_large = step->coeff[idx_large];
            } else {
                /* Extrapolate beyond model */
                double slope = (step->coeff[N-1] -
                               (N > 1 ? step->coeff[N-2] : 0.0));
                s_large = step->coeff[N-1] +
                          slope * (double)(idx_large - N + 1);
            }

            if (l < N) {
                s_small = step->coeff[l];
            } else {
                double slope = (step->coeff[N-1] -
                               (N > 1 ? step->coeff[N-2] : 0.0));
                s_small = step->coeff[N-1] +
                          slope * (double)(l - N + 1);
            }

            f_i += (s_large - s_small) * delta_u_past[l];
        }

        dyn->free_response[i] = f_i;

        /* Add bias correction (persistent from previous step) */
        f_i += dyn->error_correction[i];
        dyn->free_response[i] = f_i;
    }

    return 0;
}

void dmc_reference_trajectory(dmc_dynamic_t *dyn, double y_k, double r,
                               double alpha, int N_p) {
    if (!dyn || N_p > MPC_MAX_STEP_HORIZON) return;

    double alpha_pow = alpha;
    for (int i = 0; i < N_p; i++) {
        /* w(k+i) = α^i * y_k + (1 - α^i) * r */
        dyn->reference_trajectory[i] = alpha_pow * y_k + (1.0 - alpha_pow) * r;
        alpha_pow *= alpha;
    }
}

void dmc_error_correction(dmc_dynamic_t *dyn, double y_meas,
                           double y_pred_1, int N_p) {
    if (!dyn || N_p > MPC_MAX_STEP_HORIZON) return;

    /* d(k) = y_meas - ŷ(k|k-1) — constant output disturbance */
    double d = y_meas - y_pred_1;
    for (int i = 0; i < N_p; i++) {
        dyn->error_correction[i] = d;
    }
}

/* ─── Cholesky Solver ─────────────────────────────────────────────────── */

int dmc_solve_cholesky(double *A, double *b, int n) {
    if (!A || !b || n < 1 || n > MPC_MAX_QP_VARS) return -1;

    /* In-place Cholesky: A = L*L^T, L stored in lower triangle */
    for (int j = 0; j < n; j++) {
        double sum = 0.0;
        for (int k = 0; k < j; k++) {
            double L_jk = A[j * n + k];
            sum += L_jk * L_jk;
        }
        double diag = A[j * n + j] - sum;
        if (diag <= 0.0) return -1; /* Not SPD */
        A[j * n + j] = sqrt(diag);

        for (int i = j + 1; i < n; i++) {
            sum = 0.0;
            for (int k = 0; k < j; k++) {
                sum += A[i * n + k] * A[j * n + k];
            }
            A[i * n + j] = (A[i * n + j] - sum) / A[j * n + j];
        }
    }

    /* Forward substitution: L*y = b */
    for (int i = 0; i < n; i++) {
        double sum = 0.0;
        for (int j = 0; j < i; j++) {
            sum += A[i * n + j] * b[j];
        }
        b[i] = (b[i] - sum) / A[i * n + i];
    }

    /* Back substitution: L^T*x = y (y is now in b) */
    for (int i = n - 1; i >= 0; i--) {
        double sum = 0.0;
        for (int j = i + 1; j < n; j++) {
            sum += A[j * n + i] * b[j];
        }
        b[i] = (b[i] - sum) / A[i * n + i];
    }

    return 0;
}

/* ─── Unconstrained DMC ────────────────────────────────────────────────── */

int dmc_unconstrained_solution(mpc_solution_t *solution,
                                const dmc_dynamic_t *dyn,
                                const mpc_tuning_t *tuning) {
    if (!solution || !dyn || !tuning) return -1;

    int N_p = dyn->n_prediction;
    int N_c = dyn->n_control;
    if (N_p < 1 || N_c < 1) return -1;

    /* Build H = G^T*G + λ*I  (N_c × N_c)
     * Build c = G^T*(f - w)  (N_c)
     *
     * Then solve H * Δu = -c via Cholesky.
     */

    /* Access G from dyn: column-major, N_p × N_c */
    const double *G = dyn->dynamic_matrix;
    double H[MPC_MAX_QP_VARS * MPC_MAX_QP_VARS] = {0};
    double c_vec[MPC_MAX_QP_VARS] = {0};

    double lambda = tuning->move_suppression;
    double Q = tuning->output_weight;

    /* H = G^T * Q * G + λ*I */
    for (int i = 0; i < N_c; i++) {
        for (int j = 0; j < N_c; j++) {
            double sum = 0.0;
            for (int k = 0; k < N_p; k++) {
                /* G row k, column i = G[i*N_p + k] */
                sum += G[i * N_p + k] * G[j * N_p + k] * Q;
            }
            H[i * N_c + j] = sum;
        }
        H[i * N_c + i] += lambda;
    }

    /* c = G^T * Q * (f - w) */
    for (int i = 0; i < N_c; i++) {
        double sum = 0.0;
        for (int k = 0; k < N_p; k++) {
            double diff = dyn->free_response[k] - dyn->reference_trajectory[k];
            sum += G[i * N_p + k] * diff * Q;
        }
        c_vec[i] = sum;
    }

    /* Solve H * x = -c */
    /* Copy H and negate c */
    double H_copy[MPC_MAX_QP_VARS * MPC_MAX_QP_VARS];
    memcpy(H_copy, H, (size_t)(N_c * N_c) * sizeof(double));
    for (int i = 0; i < N_c; i++) c_vec[i] = -c_vec[i];

    if (dmc_solve_cholesky(H_copy, c_vec, N_c) != 0) {
        solution->solve_status = QP_NUMERICAL_ERROR;
        return -1;
    }

    /* Store solution */
    for (int i = 0; i < N_c && i < MPC_MAX_QP_VARS; i++) {
        solution->delta_u_plan[i] = c_vec[i];
    }

    /* Compute objective: J = 0.5*Δu^T*H*Δu + c^T*Δu
     * where original c (before negation) */
    double J = 0.0;
    for (int i = 0; i < N_c; i++) {
        double sum_H = 0.0;
        for (int j = 0; j < N_c; j++) {
            sum_H += H[i * N_c + j] * solution->delta_u_plan[j];
        }
        J += 0.5 * solution->delta_u_plan[i] * sum_H;
        /* Original c contribution (before negate): */
        double orig_c = 0.0;
        for (int k = 0; k < N_p; k++) {
            double diff = dyn->free_response[k] - dyn->reference_trajectory[k];
            orig_c += G[i * N_p + k] * diff * Q;
        }
        J += orig_c * solution->delta_u_plan[i];
    }
    solution->objective = J;
    solution->iterations = 1;
    solution->solve_status = QP_OPTIMAL;

    return 0;
}

/* ─── Constrained QP Setup ────────────────────────────────────────────── */

int dmc_setup_qp(qp_problem_t *qp, const dmc_dynamic_t *dyn,
                  const mpc_tuning_t *tuning,
                  const mpc_level_config_t *config, double u_prev) {
    if (!qp || !dyn || !tuning || !config) return -1;

    int N_p = dyn->n_prediction;
    int N_c = dyn->n_control;
    if (N_p < 1 || N_c < 1 || N_c > MPC_MAX_QP_VARS) return -1;

    const double *G = dyn->dynamic_matrix;
    double lambda = tuning->move_suppression;
    double Q = tuning->output_weight;

    qp_init(qp);
    qp->n_vars = N_c;

    /* Build H and c */
    for (int i = 0; i < N_c; i++) {
        for (int j = 0; j < N_c; j++) {
            double sum = 0.0;
            for (int k = 0; k < N_p; k++) {
                sum += G[i * N_p + k] * G[j * N_p + k] * Q;
            }
            qp->H[i * N_c + j] = sum;
        }
        qp->H[i * N_c + i] += lambda;
    }

    for (int i = 0; i < N_c; i++) {
        double sum = 0.0;
        for (int k = 0; k < N_p; k++) {
            double diff = dyn->free_response[k] - dyn->reference_trajectory[k];
            sum += G[i * N_p + k] * diff * Q;
        }
        qp->c[i] = sum;
    }

    /* Input constraint: u_min - u_prev ≤ Σ Δu_j ≤ u_max - u_prev */
    double u_min = config->valve_min;
    double u_max = config->valve_max;

    if (u_min < u_max) {
        /* Build cumulative sum matrix L: L[i,j] = 1 if j ≤ i, else 0 */
        /* L * Δu ≤ u_max - u_prev (for each cumulative sum) */
        for (int i = 0; i < N_c; i++) {
            int row = qp->n_ineq_constraints;
            if (row >= MPC_MAX_CONSTRAINTS) break;
            for (int j = 0; j <= i; j++) {
                qp->A_ineq[row * MPC_MAX_QP_VARS + j] = 1.0;
            }
            qp->b_ineq[row] = u_max - u_prev;
            qp->n_ineq_constraints++;

            /* -L * Δu ≤ -(u_min - u_prev) */
            row = qp->n_ineq_constraints;
            if (row >= MPC_MAX_CONSTRAINTS) break;
            for (int j = 0; j <= i; j++) {
                qp->A_ineq[row * MPC_MAX_QP_VARS + j] = -1.0;
            }
            qp->b_ineq[row] = -(u_min - u_prev);
            qp->n_ineq_constraints++;
        }
    }

    /* Rate constraint: box on Δu */
    double du_max = config->valve_rate_max;
    if (du_max > 0.0) {
        for (int i = 0; i < N_c; i++) {
            qp->x_lower[i] = -du_max;
            qp->x_upper[i] = du_max;
        }
    } else {
        for (int i = 0; i < N_c; i++) {
            qp->x_lower[i] = -1e6;
            qp->x_upper[i] = 1e6;
        }
    }

    /* Output constraints: y_min ≤ G*Δu + f ≤ y_max */
    double y_min = config->level_lo_limit;
    double y_max = config->level_hi_limit;

    if (y_min < y_max) {
        /* Start from i > dead_time to avoid unachievable early steps */
        int start_i = 1; /* at least 1 step delay */

        for (int i = start_i; i < N_p; i++) {
            if (qp->n_ineq_constraints + 2 > MPC_MAX_CONSTRAINTS) break;

            /* G_i^T * Δu ≤ y_max - f_i */
            int row = qp->n_ineq_constraints;
            for (int j = 0; j < N_c; j++) {
                qp->A_ineq[row * MPC_MAX_QP_VARS + j] = G[j * N_p + i];
            }
            qp->b_ineq[row] = y_max - dyn->free_response[i];
            qp->n_ineq_constraints++;

            /* -G_i^T * Δu ≤ -(y_min - f_i) */
            row = qp->n_ineq_constraints;
            if (row >= MPC_MAX_CONSTRAINTS) break;
            for (int j = 0; j < N_c; j++) {
                qp->A_ineq[row * MPC_MAX_QP_VARS + j] = -G[j * N_p + i];
            }
            qp->b_ineq[row] = -(y_min - dyn->free_response[i]);
            qp->n_ineq_constraints++;
        }
    }

    return 0;
}

void dmc_compute_prediction(dmc_dynamic_t *dyn,
                             mpc_solution_t *solution) {
    if (!dyn || !solution) return;

    int N_p = dyn->n_prediction;
    int N_c = dyn->n_control;
    const double *G = dyn->dynamic_matrix;

    for (int i = 0; i < N_p && i < MPC_MAX_STEP_HORIZON; i++) {
        double y_hat = dyn->free_response[i];
        for (int j = 0; j < N_c; j++) {
            y_hat += G[j * N_p + i] * solution->delta_u_plan[j];
        }
        solution->y_pred[i] = y_hat;
    }
}

/* ─── Integrating-Specific Extensions ─────────────────────────────────── */

int dmc_integrating_terminal_penalty(dmc_dynamic_t *dyn, qp_problem_t *qp,
                                      const mpc_tuning_t *tuning) {
    if (!dyn || !qp || !tuning) return -1;

    int N_p = dyn->n_prediction;
    int N_c = dyn->n_control;
    if (N_p < 1 || N_c < 1) return -1;

    const double *G = dyn->dynamic_matrix;
    double P = tuning->terminal_weight;
    if (P <= 0.0) return 0; /* No terminal penalty needed */

    /* Add P * g_Np * g_Np^T to H */
    int last_row = N_p - 1;
    for (int i = 0; i < N_c; i++) {
        double g_i = G[i * N_p + last_row];
        for (int j = 0; j < N_c; j++) {
            double g_j = G[j * N_p + last_row];
            qp->H[i * N_c + j] += P * g_i * g_j;
        }
        /* Add P * (f_Np - w_Np) * g_Np to c */
        qp->c[i] += P * (dyn->free_response[last_row] -
                          dyn->reference_trajectory[last_row]) *
                     G[i * N_p + last_row];
    }

    return 0;
}

int dmc_move_blocking(dmc_dynamic_t *dyn, const mpc_tuning_t *tuning,
                       qp_problem_t *qp) {
    if (!dyn || !tuning || !qp) return -1;
    if (!tuning->move_blocking_enabled || tuning->move_blocking_divisor <= 1)
        return dyn->n_control;

    int N_p = dyn->n_prediction;
    int N_c_orig = dyn->n_control;
    int divisor = tuning->move_blocking_divisor;
    int N_c_new = (N_c_orig + divisor - 1) / divisor;
    if (N_c_new < 1) return N_c_orig;

    /* Build blocking matrix M (N_c_orig × N_c_new):
     * Block i maps to new variable floor(i/block_size) */
    double M[MPC_MAX_QP_VARS * MPC_MAX_QP_VARS] = {0};

    for (int i = 0; i < N_c_orig; i++) {
        int block = i / divisor;
        if (block < N_c_new) {
            M[block * N_c_orig + i] = 1.0; /* Row block, col i */
        }
    }

    /* G_blocked = G * M^T  (N_p × N_c_new) */
    double G_blocked[MPC_MAX_STEP_HORIZON * 20] = {0};
    for (int row = 0; row < N_p; row++) {
        for (int new_col = 0; new_col < N_c_new; new_col++) {
            double sum = 0.0;
            for (int old_col = 0; old_col < N_c_orig; old_col++) {
                sum += dyn->dynamic_matrix[old_col * N_p + row] *
                       M[new_col * N_c_orig + old_col];
            }
            G_blocked[new_col * N_p + row] = sum;
        }
    }

    /* Update dynamic matrix and dimensions */
    memcpy(dyn->dynamic_matrix, G_blocked,
           (size_t)(N_p * N_c_new) * sizeof(double));
    dyn->n_control = N_c_new;
    qp->n_vars = N_c_new;

    return N_c_new;
}

/* ─── DMC Lifecycle ────────────────────────────────────────────────────── */

void dmc_init(dmc_dynamic_t *dyn, const step_response_t *step,
              const mpc_tuning_t *tuning) {
    if (!dyn || !step || !tuning) return;

    memset(dyn, 0, sizeof(*dyn));
    dyn->n_prediction = tuning->prediction_horizon;
    dyn->n_control = tuning->control_horizon;
    dyn->n_model = step->n_coeffs;

    /* Build initial dynamic matrix */
    dmc_build_dynamic_matrix(dyn->dynamic_matrix, step,
                              dyn->n_prediction, dyn->n_control);
}

double dmc_step(mpc_solution_t *solution, dmc_dynamic_t *dyn,
                const step_response_t *step, const mpc_tuning_t *tuning,
                const mpc_level_config_t *config, double y_meas,
                double u_prev, mpc_solver_type_t solver) {
    if (!solution || !dyn || !step || !tuning || !config) return 0.0;

    /* Pre-compute one-step prediction for bias correction from previous
     * solution, if available */
    double y_pred_1 = y_meas; /* fallback: no prediction error */
    if (solution->solve_status == QP_OPTIMAL && dyn->n_prediction > 0) {
        y_pred_1 = solution->y_pred[0];
    }

    /* 1. Build free response */
    dmc_build_free_response(dyn, step, y_meas, dyn->delta_u_past,
                             dyn->n_model);

    /* 2. Bias correction */
    dmc_error_correction(dyn, y_meas, y_pred_1, dyn->n_prediction);

    /* Re-apply free response with bias */
    dmc_build_free_response(dyn, step, y_meas, dyn->delta_u_past,
                             dyn->n_model);

    /* 3. Reference trajectory */
    dmc_reference_trajectory(dyn, y_meas, config->level_setpoint,
                              tuning->reference_trajectory_alpha,
                              dyn->n_prediction);

    /* 4. Set up QP */
    qp_problem_t qp;
    dmc_setup_qp(&qp, dyn, tuning, config, u_prev);

    /* 5. Terminal penalty for stability (integrating processes) */
    if (tuning->terminal_weight > 0.0) {
        dmc_integrating_terminal_penalty(dyn, &qp, tuning);
    }

    /* 6. Move blocking */
    if (tuning->move_blocking_enabled) {
        dmc_move_blocking(dyn, tuning, &qp);
    }

    /* 7. Solve QP */
    solution->solve_status = qp_solve(solution, &qp, solver);
    if (solution->solve_status != QP_OPTIMAL) {
        /* Fallback: use zero move */
        return 0.0;
    }

    /* 8. First move */
    double du = solution->delta_u_plan[0];

    /* 9. Compute predicted trajectory */
    dmc_compute_prediction(dyn, solution);

    /* 10. Shift past move buffer */
    if (dyn->n_model > 1) {
        memmove(&dyn->delta_u_past[1], dyn->delta_u_past,
                (size_t)(dyn->n_model - 1) * sizeof(double));
    }
    dyn->delta_u_past[0] = du;

    /* 11. Compute intermediate u values for constraint checking */
    double u_cumulative = u_prev + du;
    for (int j = 0; j < dyn->n_control && j < MPC_MAX_QP_VARS; j++) {
        if (j > 0) u_cumulative += solution->delta_u_plan[j];
        solution->u_plan[j] = u_cumulative;
    }

    return du;
}

/* ─── Utility ─────────────────────────────────────────────────────────── */

int dmc_condition_number(const double *G, int N_p, int N_c, double *cond) {
    if (!G || !cond || N_p < 1 || N_c < 1) return -1;

    /* Estimate via power iteration */
    double sigma_max_sq = 0.0;
    double v[MPC_MAX_QP_VARS];
    double u[MPC_MAX_STEP_HORIZON];

    /* Initialize v */
    for (int i = 0; i < N_c; i++) v[i] = 1.0 / sqrt((double)N_c);

    /* Power iteration for G^T*G (find σ_max²) */
    for (int iter = 0; iter < 20; iter++) {
        /* u = G * v */
        for (int i = 0; i < N_p; i++) {
            u[i] = 0.0;
            for (int j = 0; j < N_c; j++) {
                u[i] += G[j * N_p + i] * v[j];
            }
        }
        /* v = G^T * u */
        double v_norm = 0.0;
        for (int j = 0; j < N_c; j++) {
            v[j] = 0.0;
            for (int i = 0; i < N_p; i++) {
                v[j] += G[j * N_p + i] * u[i];
            }
            v_norm += v[j] * v[j];
        }
        v_norm = sqrt(v_norm);
        if (v_norm > 1e-10) {
            sigma_max_sq = v_norm;
            for (int j = 0; j < N_c; j++) v[j] /= v_norm;
        }
    }

    double sigma_max = sqrt(sigma_max_sq);

    /* For min singular value, use inverse iteration on G^T*G + λ*I
     * with small λ, or bound by smallest diagonal element.
     * Simplified: σ_min ≈ min diagonal of G^T*G after deflation */
    double sigma_min = sigma_max;
    for (int j = 0; j < N_c; j++) {
        double diag = 0.0;
        for (int i = 0; i < N_p; i++) {
            diag += G[j * N_p + i] * G[j * N_p + i];
        }
        if (diag < sigma_min && diag > 1e-10) sigma_min = sqrt(diag);
    }

    if (sigma_min < 1e-10) sigma_min = 1e-10;
    *cond = sigma_max / sigma_min;

    return 0;
}
