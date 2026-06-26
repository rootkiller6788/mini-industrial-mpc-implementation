/**
 * mpc_integrating_model.c — Integrating Process Model Implementation
 *
 * Implements model construction, discretization, and validation for
 * integrating (non-self-regulating) processes used in MPC.
 *
 * Knowledge Coverage:
 *   L2 - Core Concepts: integrating dynamics, ZOH discretization
 *   L3 - Eng. Structures: state-space, CARIMA, step response models
 *   L4 - Theorems: ZOH equivalence, controllability rank condition
 *   L5 - Algorithms: matrix exponential (scaling+squaring), polynomial ops
 *
 * Reference: Franklin, Powell, Workman (1998) "Digital Control"
 *            Åström & Wittenmark (1997) "Computer-Controlled Systems"
 */

#include <math.h>
#include <string.h>
#include <stdio.h>
#include "../include/mpc_integrating_model.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ─── Model Construction ──────────────────────────────────────────────── */

int mpc_model_from_lag(integrating_process_t *model, double gain,
                        double tau, double dt, double area) {
    if (!model || gain == 0.0 || dt <= 0.0) return -1;
    if (tau < 0.0) tau = 0.0; /* pure integrator limit */
    if (dt > 10.0 * (tau > 0.0 ? tau : 1.0)) return -1; /* dt too large */

    model->gain          = gain;
    model->time_constant = tau;
    model->dead_time     = 0.0;
    model->sampling_time = dt;
    model->area          = area;
    model->max_inflow    = 1.0;
    model->max_outflow   = 1.0;
    return 0;
}

int mpc_model_pure_integrator(integrating_process_t *model,
                               double gain, double dt, double area) {
    if (!model || gain == 0.0 || dt <= 0.0) return -1;

    model->gain          = gain;
    model->time_constant = 0.0;
    model->dead_time     = 0.0;
    model->sampling_time = dt;
    model->area          = area;
    model->max_inflow    = 1.0;
    model->max_outflow   = 1.0;
    return 0;
}

int mpc_model_integrating_second_order(integrating_process_t *model,
    double gain, double tau1, double tau2, double dt) {
    if (!model || gain == 0.0 || dt <= 0.0 || tau1 < 0.0 || tau2 < 0.0)
        return -1;

    /* Approximate as single effective lag:
     * τ_eff = τ₁ + τ₂ (dominant pole approximation for slow dynamics) */
    double tau_eff = tau1 + tau2;
    return mpc_model_from_lag(model, gain, tau_eff, dt, 0.0);
}

/* ─── State-Space Conversion ──────────────────────────────────────────── */

int mpc_model_to_state_space(const integrating_process_t *model,
                              integrating_state_t *ss, int add_dist) {
    if (!model || !ss) return -1;

    double K = model->gain;
    double tau = model->time_constant;
    double Ts = model->sampling_time;
    double alpha;

    /* For pure integrator (τ=0): 1-state model */
    if (tau <= 0.0) {
        /* x(k+1) = x(k) + K*Ts*u(k) */
        ss->n_states = (add_dist) ? 2 : 1;
        ss->n_inputs = 1;
        ss->n_outputs = 1;
        ss->n_disturbances = (add_dist) ? 1 : 0;

        memset(ss->A, 0, sizeof(ss->A));
        memset(ss->B, 0, sizeof(ss->B));
        memset(ss->B_d, 0, sizeof(ss->B_d));
        memset(ss->C, 0, sizeof(ss->C));
        memset(ss->x, 0, sizeof(ss->x));

        if (add_dist) {
            /* Augmented: [x; d], A=[1 0; 0 1], B=[K*Ts; 0], C=[1 1] */
            ss->A[0] = 1.0;  /* A[0,0] = 1 */
            ss->A[3] = 1.0;  /* A[1,1] = 1 */
            ss->B[0] = K * Ts;
            ss->B_d[1] = 1.0; /* disturbance enters as random walk */
            ss->C[0] = 1.0;   /* C[0] = 1 (level) */
            ss->C[1] = 1.0;   /* C[1] = 1 (disturbance) */
        } else {
            ss->A[0] = 1.0;
            ss->B[0] = K * Ts;
            ss->C[0] = 1.0;
        }
        return 0;
    }

    /* Integrating + lag: G(s) = K/[s*(τ*s+1)]
     * Controllable canonical form in continuous time:
     *   ẋ₁ = x₂ + K*u
     *   ẋ₂ = -x₂/τ
     *   y  = x₁
     *
     * A_c = [0 1; 0 -1/τ], B_c = [K; K/τ?]
     * Actually: d/dt[y; ẏ] depends on specific realization.
     *
     * Using direct ZOH discretization of state-space:
     * Let state x1 = y (level), x2 = τ*ẏ (scaled rate)
     *
     * Discrete: α = exp(-Ts/τ)
     *   x1(k+1) = x1(k) + τ*(1-α)*x2(k) + K*(Ts - τ*(1-α))*u(k)
     *   x2(k+1) = α*x2(k) + K*(1-α)*u(k)
     *   y(k) = x1(k)
     *
     * This is exact ZOH discretization of the controllable canonical form.
     */

    alpha = exp(-Ts / tau);

    ss->n_states = (add_dist) ? 3 : 2;
    ss->n_inputs = 1;
    ss->n_outputs = 1;
    ss->n_disturbances = (add_dist) ? 1 : 0;

    memset(ss->A, 0, sizeof(ss->A));
    memset(ss->B, 0, sizeof(ss->B));
    memset(ss->B_d, 0, sizeof(ss->B_d));
    memset(ss->C, 0, sizeof(ss->C));
    memset(ss->x, 0, sizeof(ss->x));

    /* 2-state base model */
    /* A: row-major order in column-major storage for 2x2:
     * col-major: A[0]=A[0,0], A[1]=A[1,0], A[2]=A[0,1], A[3]=A[1,1] */
    ss->A[0] = 1.0;                    /* A[0,0] */
    ss->A[1] = 0.0;                    /* A[1,0] */
    ss->A[2] = tau * (1.0 - alpha);    /* A[0,1] */
    ss->A[3] = alpha;                  /* A[1,1] */

    ss->B[0] = K * (Ts - tau * (1.0 - alpha));  /* B[0] */
    ss->B[1] = K * (1.0 - alpha);                /* B[1] */

    ss->C[0] = 1.0;   /* output = x1 (level) */
    ss->C[1] = 0.0;

    if (add_dist) {
        /* Augment with integrating output disturbance: 3x3 */
        /* A_aug = [A  0; 0 1], B_aug = [B; 0], C_aug = [C 1] */
        /* A stored col-major: */
        ss->A[0] = 1.0;                    /* (0,0) */
        ss->A[1] = 0.0;                    /* (1,0) */
        ss->A[2] = 0.0;                    /* (2,0) */
        ss->A[3] = tau * (1.0 - alpha);    /* (0,1) */
        ss->A[4] = alpha;                  /* (1,1) */
        ss->A[5] = 0.0;                    /* (2,1) */
        ss->A[6] = 0.0;                    /* (0,2) */
        ss->A[7] = 0.0;                    /* (1,2) */
        ss->A[8] = 1.0;                    /* (2,2) */

        ss->B[0] = K * (Ts - tau * (1.0 - alpha));
        ss->B[1] = K * (1.0 - alpha);
        ss->B[2] = 0.0;

        ss->B_d[2] = 1.0; /* disturbance random walk */

        ss->C[0] = 1.0;   /* level */
        ss->C[1] = 0.0;
        ss->C[2] = 1.0;   /* disturbance */
    }

    return 0;
}

int mpc_carima_from_ss(const integrating_state_t *ss, gpc_config_t *gpc) {
    if (!ss || !gpc) return -1;
    if (ss->n_states < 1 || ss->n_states > GPC_MAX_ORDER) return -1;

    /* For integrating state-space, compute transfer function
     * G(z) = C*(zI-A)^{-1}*B
     *
     * For the 2-state model (integrator + lag):
     * G(z) = (b₀*z + b₁) / (z² - (1+α)*z + α)
     * A(z) = 1 - (1+α)*z^{-1} + α*z^{-2}
     * B(z) = b₀ + b₁*z^{-1}
     *
     * For pure integrator:
     * G(z) = K*Ts / (z-1)
     * A(z) = 1 - z^{-1}
     * B(z) = K*Ts * z^{-1}  →  b₀=0, b₁=K*Ts
     */

    int n = ss->n_states;
    int na, nb;

    memset(gpc, 0, sizeof(*gpc));

    if (n <= 2) {
        /* Pure integrator or integrator + first-order lag */
        double a11 = ss->A[0];      /* A[0,0] */
        double a21 = ss->A[1];      /* A[1,0] */
        double a12 = ss->A[2];      /* A[0,1] */
        double a22 = ss->A[3];      /* A[1,1] */
        double b1  = ss->B[0];
        double b2  = ss->B[1];

        /* Characteristic polynomial: z² - tr(A)*z + det(A) */
        double trace_A = a11 + a22;
        double det_A   = a11*a22 - a12*a21;

        if (n == 1) {
            /* Single integrator: A=1 */
            na = 1;
            gpc->A_coeff[0] = 1.0;
            gpc->A_coeff[1] = -1.0;
            nb = 1;
            gpc->B_coeff[0] = b1;  /* b₀ = K*Ts for pure integrator */
            gpc->B_coeff[1] = 0.0;
        } else {
            na = 2;
            gpc->A_coeff[0] = 1.0;
            gpc->A_coeff[1] = -trace_A;  /* a₁ */
            gpc->A_coeff[2] = det_A;      /* a₂ */

            /* B polynomial: numerator = C*adj(zI-A)*B
             * For output y = x1: b₀ = b1, b₁ = a12*b2 - a22*b1 */
            nb = 1;
            gpc->B_coeff[0] = b1;
            gpc->B_coeff[1] = a12*b2 - a22*b1;
        }
    } else {
        /* Higher-order: use Leverrier-Faddeev algorithm */
        na = n;
        nb = n - 1;
        /* Simplified: set A to identity minus state matrix eigenvalues.
         * Full Leverrier implementation would be needed for general case. */
        gpc->A_coeff[0] = 1.0;
        for (int i = 1; i <= n; i++) gpc->A_coeff[i] = 0.0;
        gpc->A_coeff[n] = 1.0; /* z^n - 1 (approximate) */
        gpc->B_coeff[0] = ss->B[0];
    }

    gpc->na = na;
    gpc->nb = nb;
    gpc->nc = 0;
    gpc->delay = 1;
    gpc->C_coeff[0] = 1.0; /* C(z) = 1 */
    gpc->lambda_gpc = 1.0;
    gpc->n1 = 1;
    gpc->n2 = 20;
    gpc->nu = 5;

    return na;
}

int mpc_carima_integrating_default(gpc_config_t *gpc, double gain,
                                    double dt, int d) {
    if (!gpc || gain == 0.0 || dt <= 0.0 || d < 1) return -1;

    memset(gpc, 0, sizeof(*gpc));

    /* For pure integrator: G(z) = K*Ts / (z-1) */
    gpc->na = 1;
    gpc->A_coeff[0] = 1.0;
    gpc->A_coeff[1] = -1.0;

    gpc->nb = 0;
    gpc->B_coeff[0] = gain * dt;

    gpc->nc = 0;
    gpc->C_coeff[0] = 1.0;
    gpc->delay = d;
    gpc->lambda_gpc = 1.0;
    gpc->n1 = d;
    gpc->n2 = 30;
    gpc->nu = 5;

    return 0;
}

/* ─── Step Response Model ──────────────────────────────────────────────── */

int mpc_step_response_from_model(step_response_t *step,
                                  const integrating_process_t *model, int N) {
    if (!step || !model || N < 1 || N > MPC_MAX_STEP_HORIZON) return -1;

    /* Simulate discrete state-space with unit step input u(k)=1 for all k */
    double K = model->gain;
    double tau = model->time_constant;
    double Ts = model->sampling_time;

    step->n_coeffs = N;
    step->dead_time_samples = 0.0;

    if (tau <= 0.0) {
        /* Pure integrator: s_i = K * Ts * i */
        for (int i = 0; i < N; i++) {
            step->coeff[i] = K * Ts * (double)(i + 1);
        }
        step->steady_state_gain = step->coeff[N-1];
    } else {
        /* Integrating + lag: s_i = K*[i*Ts - τ*(1-exp(-i*Ts/τ))] */
        double alpha = exp(-Ts / tau);
        double x1 = 0.0; /* level */
        double x2 = 0.0; /* lag state */

        for (int i = 0; i < N; i++) {
            /* Step the discrete state-space with u=1 */
            double x1_next = x1 + tau*(1.0-alpha)*x2 + K*(Ts - tau*(1.0-alpha));
            double x2_next = alpha*x2 + K*(1.0-alpha);
            x1 = x1_next;
            x2 = x2_next;
            step->coeff[i] = x1;
        }
        step->steady_state_gain = step->coeff[N-1];
    }

    return N;
}

int mpc_step_response_ramp(step_response_t *step, double gain, double tau,
                            double dt, int N) {
    if (!step || gain == 0.0 || dt <= 0.0 || N < 1 || N > MPC_MAX_STEP_HORIZON)
        return -1;

    step->n_coeffs = N;
    step->dead_time_samples = 0.0;

    if (tau <= 0.0) {
        for (int i = 0; i < N; i++) {
            step->coeff[i] = gain * dt * (double)(i + 1);
        }
    } else {
        for (int i = 0; i < N; i++) {
            double t = dt * (double)(i + 1);
            step->coeff[i] = gain * (t - tau * (1.0 - exp(-t / tau)));
        }
    }
    step->steady_state_gain = step->coeff[N-1];
    return N;
}

/* ─── Discretization ──────────────────────────────────────────────────── */

int mpc_discretize_zoh(const double *A_c, const double *B_c,
                        double *A_d, double *B_d, int n, double dt) {
    if (!A_c || !B_c || !A_d || !B_d || n < 1 || n > MPC_MAX_STATES || dt <= 0.0)
        return -1;

    /* Compute A_d = exp(A_c * dt) via truncated Taylor series.
     * A_d = I + A_c*dt + A_c²*dt²/2 + A_c³*dt³/6 + A_c⁴*dt⁴/24 + A_c⁵*dt⁵/120
     *
     * For integrating process: A_c has eigenvalue 0 → singular.
     * Taylor series avoids inversion needed by A_c⁻¹*(A_d - I) formula.
     */

    /* Initialize I to A_d */
    memset(A_d, 0, (size_t)(n * n) * sizeof(double));
    for (int i = 0; i < n; i++) A_d[i * n + i] = 1.0;

    /* Accumulate power series: A_d += A_c*dt + A_c²*dt²/2 + ... */
    double term[MPC_MAX_STATES * MPC_MAX_STATES];
    double temp[MPC_MAX_STATES * MPC_MAX_STATES];

    /* term = I (initial) */
    memset(term, 0, (size_t)(n * n) * sizeof(double));
    for (int i = 0; i < n; i++) term[i * n + i] = 1.0;

    double factorial = 1.0;
    double dt_pow = 1.0;
    for (int k = 1; k <= 5; k++) {
        /* term = term * A_c (matrix multiply) */
        memset(temp, 0, (size_t)(n * n) * sizeof(double));
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                for (int l = 0; l < n; l++)
                    temp[i * n + j] += term[i * n + l] * A_c[l * n + j];
        memcpy(term, temp, (size_t)(n * n) * sizeof(double));

        factorial *= (double)k;
        dt_pow *= dt;

        double scale = dt_pow / factorial;
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                A_d[i * n + j] += scale * term[i * n + j];
    }

    /* B_d = ∫₀^{Ts} exp(A_c*s) ds * B_c
     *     = (I*Ts + A_c*Ts²/2! + A_c²*Ts³/3! + ...) * B_c */
    memset(B_d, 0, (size_t)n * sizeof(double));

    /* term = I */
    memset(term, 0, (size_t)(n * n) * sizeof(double));
    for (int i = 0; i < n; i++) term[i * n + i] = 1.0;

    double int_matrix[MPC_MAX_STATES * MPC_MAX_STATES] = {0};
    dt_pow = 1.0;
    for (int k = 1; k <= 5; k++) {
        /* term = term * A_c */
        memset(temp, 0, (size_t)(n * n) * sizeof(double));
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                for (int l = 0; l < n; l++)
                    temp[i * n + j] += term[i * n + l] * A_c[l * n + j];
        memcpy(term, temp, (size_t)(n * n) * sizeof(double));

        dt_pow *= dt;
        double scale = dt_pow / (double)(k + 1);
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                int_matrix[i * n + j] += scale * term[i * n + j];
    }
    /* Add I*Ts term */
    dt_pow = dt;
    for (int i = 0; i < n; i++)
        int_matrix[i * n + i] += dt_pow;

    /* B_d = int_matrix * B_c */
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            B_d[i] += int_matrix[i * n + j] * B_c[j];

    return 0;
}

int mpc_matrix_exponential(const double *A, double *expA, int n) {
    if (!A || !expA || n < 1 || n > MPC_MAX_STATES) return -1;

    /* Simple scaling-and-squaring:
     * exp(A) ≈ [R_{mm}(A/2^s)]^{2^s}
     * where R_{22}(X) = I + X + X²/2 is the (2,2) Pade approximant.
     */

    /* Scale: s = ceil(log2(||A||₁)) */
    double norm = 0.0;
    for (int j = 0; j < n; j++) {
        double col_sum = 0.0;
        for (int i = 0; i < n; i++) col_sum += fabs(A[i * n + j]);
        if (col_sum > norm) norm = col_sum;
    }

    int s = 0;
    double scale = norm;
    while (scale > 0.5 && s < 10) {
        scale /= 2.0;
        s++;
    }

    /* Compute A_scaled = A / 2^s */
    double divisor = pow(2.0, (double)s);
    double A_s[MPC_MAX_STATES * MPC_MAX_STATES];
    double temp[MPC_MAX_STATES * MPC_MAX_STATES];
    for (int i = 0; i < n * n; i++) A_s[i] = A[i] / divisor;

    /* R_{22}(X) = I + X + X²/2 */
    memset(expA, 0, (size_t)(n * n) * sizeof(double));
    for (int i = 0; i < n; i++) expA[i * n + i] = 1.0;

    /* Add X */
    for (int i = 0; i < n * n; i++) expA[i] += A_s[i];

    /* Add X²/2 */
    memset(temp, 0, (size_t)(n * n) * sizeof(double));
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            for (int k = 0; k < n; k++)
                temp[i * n + j] += A_s[i * n + k] * A_s[k * n + j];
    for (int i = 0; i < n * n; i++) expA[i] += 0.5 * temp[i];

    /* Square s times: expA = expA^2 repeated */
    for (int sq = 0; sq < s; sq++) {
        memcpy(temp, expA, (size_t)(n * n) * sizeof(double));
        memset(expA, 0, (size_t)(n * n) * sizeof(double));
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                for (int k = 0; k < n; k++)
                    expA[i * n + j] += temp[i * n + k] * temp[k * n + j];
    }

    return 0;
}

/* ─── Model Validation ────────────────────────────────────────────────── */

int mpc_model_validate(const integrating_process_t *model) {
    if (!model) return (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4);

    int issues = 0;

    /* Check gain zero */
    if (fabs(model->gain) < 1e-12) issues |= (1 << 4);

    /* Check sampling time adequacy */
    double tau = model->time_constant;
    if (tau > 0.0 && model->sampling_time > tau / 5.0) issues |= (1 << 3);

    /* For integrating process, A has eigenvalue at 1.0 → not strictly stable,
     * but that's expected. Check if A has eigenvalue > 1 + eps (unstable) */
    double alpha = (tau > 0.0) ? exp(-model->sampling_time / tau) : 1.0;
    if (alpha > 1.0 + 1e-9) issues |= (1 << 0);

    /* Controllability: For 2-state integrating+lag, rank([B AB])==2 unless K=0 */
    /* Observability: [C; CA] full rank for level output (C=[1 0]) */
    /* Both are satisfied for our construction, so only flag if K==0 */

    return issues;
}

/* ─── Utility Functions ───────────────────────────────────────────────── */

double mpc_tank_area_to_gain(double area, double valve_coeff, double level) {
    if (area <= 0.0 || level < 0.0) return 0.0;

    /* K = -C_v * sqrt(h) / (100 * A)  [m/(s·%valve)]
     * Sign: opening valve decreases level (outflow increases) */
    if (valve_coeff <= 0.0) {
        /* Pure integrator gain: K = -1/A [m/(m³/s)] */
        return -1.0 / area;
    }
    return -valve_coeff * sqrt(level) / (100.0 * area);
}

double mpc_residence_time(double area, double level, double outflow) {
    if (area <= 0.0 || outflow <= 0.0) return 1e6;
    return (area * level) / outflow;
}
