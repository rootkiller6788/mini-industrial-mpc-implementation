/**
 * mpc_surge_tank.c — Surge Tank Level Control with MPC
 *
 * Implements surge tank physical modeling, flow dynamics, level control
 * modes (tight/averaging/surge), feedforward compensation, and closed-loop
 * MPC simulation for surge tank level management.
 *
 * Knowledge Coverage:
 *   L6 - Canonical Problems: Surge tank level control, flow averaging
 *   L2 - Core Concepts: buffer capacity, flow smoothing, residence time
 *   L3 - Eng. Structures: tank dynamics, valve flow equation, feedforward
 *   L7 - Applications: Refinery surge drums, distillation feed tanks
 *
 * Reference: Luyben (2007), Shinskey (1996), McDonald & McAvoy (1986)
 */

#include <math.h>
#include <string.h>
#include <stdio.h>
#include "../include/mpc_surge_tank.h"
#include "../include/mpc_dmc.h"
#include "../include/mpc_integrating_model.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ─── Tank Physical Modeling ──────────────────────────────────────────── */

int surge_tank_init(surge_tank_t *tank, double diameter, double max_level,
                     double initial_level, double valve_coeff) {
    if (!tank || diameter <= 0.0 || max_level <= 0.0) return -1;
    if (initial_level < 0.0) initial_level = 0.0;
    if (initial_level > max_level) initial_level = max_level;

    tank->diameter    = diameter;
    tank->cross_area  = M_PI * diameter * diameter / 4.0;
    tank->max_level   = max_level;
    tank->level       = initial_level;
    tank->inflow      = 0.0;
    tank->outflow     = 0.0;
    tank->valve_coeff = valve_coeff;
    tank->valve_position = 50.0; /* default: 50% open */
    tank->holdup_volume = tank->cross_area * initial_level;
    tank->residence_time = (valve_coeff > 0.0 && initial_level > 0.0)
        ? tank->holdup_volume / (valve_coeff * sqrt(initial_level))
        : 1e6;

    /* Set initial outflow based on valve position */
    tank->outflow = surge_tank_outflow(tank, tank->valve_position);

    return 0;
}

double surge_tank_outflow(const surge_tank_t *tank, double valve) {
    if (!tank) return 0.0;
    if (valve < 0.0) valve = 0.0;
    if (valve > 100.0) valve = 100.0;

    if (tank->valve_coeff <= 0.0) {
        /* Pumped flow: direct proportion */
        return valve / 100.0;
    }

    /* Gravity flow: Torricelli's law with valve C_v
     * F_out = C_v * sqrt(h) * (valve/100)
     * C_v has units [m^(5/2)/s] for dimensional consistency */
    double effective_level = tank->level;
    if (effective_level < 0.0) effective_level = 0.0;

    return tank->valve_coeff * sqrt(effective_level) * (valve / 100.0);
}

void surge_tank_simulate(surge_tank_t *tank, double F_in, double F_out,
                          double Ts) {
    if (!tank || Ts <= 0.0) return;

    /* Euler integration of mass balance:
     * dh/dt = (F_in - F_out) / A */
    double dh = (F_in - F_out) * Ts / tank->cross_area;
    tank->level += dh;

    /* Clamp to physical limits */
    if (tank->level < 0.0) tank->level = 0.0;
    if (tank->level > tank->max_level) tank->level = tank->max_level;

    /* Update derived quantities */
    tank->inflow = F_in;
    tank->outflow = F_out;
    tank->holdup_volume = tank->cross_area * tank->level;

    if (F_out > 1e-10) {
        tank->residence_time = tank->holdup_volume / F_out;
    } else {
        tank->residence_time = 1e6;
    }
}

/* ─── Level Control Modes ─────────────────────────────────────────────── */

double surge_level_filter_factor(double bandwidth, double tau_res) {
    if (bandwidth <= 0.0 || tau_res <= 0.0) return 1.0;

    /* Optimal filter factor from frequency-domain analysis:
     * φ = 1 / (1 + ω * τ_res)
     * For ω = 0 (steady state): φ = 1 (tight control)
     * For ω → ∞: φ → 0 (pure averaging)
     */
    double phi = 1.0 / (1.0 + bandwidth * tau_res);
    return phi;
}

double surge_capacity_utilization(const surge_tank_t *tank, double bandwidth,
                                   double phi, double *ampl_out) {
    if (!tank || bandwidth <= 0.0 || phi <= 0.0) {
        if (ampl_out) *ampl_out = 0.0;
        return 0.0;
    }

    /* Amplitude of level oscillation for flow disturbance amplitude A_f:
     * Δh = A_f / (ω*A) for uncontrolled integrator
     * With control (φ): Δh_controlled = φ * A_f / (ω * A * (1 - φ))
     *
     * Maximum A_f before hitting limits:
     * A_f_max = Δh_allowable * ω * A * (1-φ) / φ
     *
     * where Δh_allowable = 0.8 * max_level (use 80% as working range)
     */
    double A = tank->cross_area;
    double dh_allowable = 0.8 * tank->max_level;

    double A_f_max = dh_allowable * bandwidth * A * (1.0 - phi) / phi;

    /* Amplitude at maximum allowable flow */
    if (ampl_out) {
        *ampl_out = phi * A_f_max / (bandwidth * A * (1.0 - phi));
    }

    return A_f_max;
}

/* ─── MPC Surge Tank Configuration ────────────────────────────────────── */

int surge_mpc_config(mpc_tuning_t *tuning, mpc_level_config_t *config,
                      const surge_tank_t *tank, level_control_mode_t mode,
                      double inflow_nom) {
    if (!tuning || !config || !tank) return -1;

    double A = tank->cross_area;
    double valve_Cv = tank->valve_coeff;
    double h = tank->level;
    if (h < 0.01) h = 0.01;

    /* Process gain: K = -Cv*sqrt(h)/(100*A) [m/s per %valve] */
    (void)valve_Cv; /* Used in feedforward, gain implicit in step response */

    /* Residence time */
    double F_out = surge_tank_outflow(tank, tank->valve_position);
    double tau_res = (F_out > 1e-10)
        ? (A * h) / F_out : 1000.0;

    /* Sampling time: 1/20 of residence time, min 0.5s, max 30s */
    double Ts = tau_res / 20.0;
    if (Ts < 0.5) Ts = 0.5;
    if (Ts > 30.0) Ts = 30.0;

    /* Prediction horizon: cover at least 3*residence time */
    int N_p = (int)(3.0 * tau_res / Ts + 0.5);
    if (N_p < 10) N_p = 10;
    if (N_p > MPC_MAX_STEP_HORIZON) N_p = MPC_MAX_STEP_HORIZON;

    /* Control horizon: ~N_p/4, min 3 */
    int N_c = N_p / 4;
    if (N_c < 3) N_c = 3;
    if (N_c > MPC_MAX_QP_VARS) N_c = MPC_MAX_QP_VARS;

    /* Tuning based on mode */
    double lambda, Q_weight, ref_alpha;
    switch (mode) {
        case LEVEL_MODE_TIGHT:
            /* Aggressive: tight level, high Q, low lambda */
            Q_weight = 10.0;
            lambda = 0.1;
            ref_alpha = 0.3; /* fast approach to SP */
            break;
        case LEVEL_MODE_AVERAGING:
            /* Balanced: moderate level variance, some flow smoothing */
            Q_weight = 1.0;
            lambda = 1.0;
            ref_alpha = 0.7;
            break;
        case LEVEL_MODE_SURGE:
            /* Conservative: prioritize flow smoothing over level */
            Q_weight = 0.1;
            lambda = 10.0;
            ref_alpha = 0.95; /* slow approach, absorb flow changes */
            break;
        default:
            Q_weight = 1.0;
            lambda = 1.0;
            ref_alpha = 0.7;
            break;
    }

    /* Populate tuning */
    memset(tuning, 0, sizeof(*tuning));
    tuning->prediction_horizon = N_p;
    tuning->control_horizon = N_c;
    tuning->sampling_time = Ts;
    tuning->move_suppression = lambda;
    tuning->output_weight = Q_weight;
    tuning->input_weight = 0.01;
    tuning->terminal_weight = 100.0; /* terminal penalty for integrating process */
    tuning->reference_trajectory_alpha = ref_alpha;
    tuning->move_blocking_enabled = (N_c > 6) ? 1 : 0;
    tuning->move_blocking_divisor = 3;

    /* Populate level configuration */
    memset(config, 0, sizeof(*config));
    config->level_setpoint = 0.5 * tank->max_level; /* 50% as default SP */
    config->level_lo_limit = 0.1 * tank->max_level; /* 10% low */
    config->level_hi_limit = 0.9 * tank->max_level; /* 90% high */
    config->level_hihi_limit = 0.95 * tank->max_level; /* 95% hi-hi */
    config->valve_min = 5.0;   /* 5% minimum (avoid deadband) */
    config->valve_max = 95.0;  /* 95% maximum */
    config->valve_rate_max = 5.0; /* 5%/s slew rate limit */
    config->inflow_disturbance = inflow_nom;

    return 0;
}

/* ─── Feedforward ──────────────────────────────────────────────────────── */

double surge_feedforward(const surge_tank_t *tank, double delta_F_in) {
    if (!tank) return 0.0;

    /* For a change in inlet flow ΔF_in, the outlet flow must
     * change by the same amount to maintain level:
     * ΔF_out_ff = ΔF_in
     * Δu_ff = (ΔF_out_ff * 100) / (C_v * sqrt(h))
     */
    if (tank->valve_coeff <= 0.0 || tank->level <= 0.0) {
        /* Pumped flow: Δu_ff = ΔF_in * 100 / pump_capacity */
        return delta_F_in * 100.0; /* assume pump capacity = 1.0 */
    }

    double denom = tank->valve_coeff * sqrt(tank->level);
    if (fabs(denom) < 1e-10) return 0.0;

    return (delta_F_in * 100.0) / denom;
}

void surge_feedforward_matrix(double *G_d, const surge_tank_t *tank,
                               int N_p, double Ts) {
    if (!G_d || !tank || N_p < 1 || N_p > MPC_MAX_STEP_HORIZON) return;

    /* Step response of level to inlet flow step:
     * For pure tank: dh/dt = F_in/A → h(t) = t/A
     * Discrete: s_i = i * Ts / A
     */
    double A = tank->cross_area;
    if (A <= 0.0) {
        memset(G_d, 0, (size_t)N_p * sizeof(double));
        return;
    }

    for (int i = 0; i < N_p; i++) {
        G_d[i] = (double)(i + 1) * Ts / A;
    }
}

/* ─── Surge Tank Sizing ────────────────────────────────────────────────── */

double surge_tank_sizing(double F_avg, double T_disturb, double phi,
                          double delta_h_frac) {
    if (F_avg <= 0.0 || T_disturb <= 0.0 || phi <= 0.0 || delta_h_frac <= 0.0) {
        return 0.0;
    }

    /* V_min = (F_avg * T_disturb) / (2 * π * φ * Δh_frac)
     *
     * This formula comes from the sinusoidal flow disturbance assumption:
     * F_in = F_avg + A_f * sin(2π*t/T)
     *
     * Level amplitude: Δh = (A_f * T) / (2π * A * φ)
     * Requiring Δh < Δh_frac * h_max and V = A * h_max:
     * V_min = (F_avg * T) / (2π * φ * Δh_frac)
     */
    double V_min = (F_avg * T_disturb) / (2.0 * M_PI * phi * delta_h_frac);
    return V_min;
}

/* ─── MPC Surge Simulation ────────────────────────────────────────────── */

int surge_mpc_simulation_run(surge_tank_t *tank, const mpc_tuning_t *tuning,
                              const mpc_level_config_t *config,
                              const step_response_t *step,
                              dmc_dynamic_t *dyn,
                              const double *F_in_hist, int n_steps,
                              double *F_out_hist, double *level_hist,
                              double *valve_hist, mpc_kpi_t *kpi) {
    if (!tank || !tuning || !config || !step || !dyn) return -1;
    if (!F_in_hist || !F_out_hist || !level_hist || !valve_hist) return -1;
    if (n_steps < 1) return -1;

    /* Initialize KPI */
    if (kpi) memset(kpi, 0, sizeof(*kpi));

    double u_prev = tank->valve_position;

    for (int k = 0; k < n_steps; k++) {
        /* Current measurement */
        double y_meas = tank->level;

        /* MPC step */
        mpc_solution_t solution;
        memset(&solution, 0, sizeof(solution));

        double du = dmc_step(&solution, dyn, step, tuning, config,
                              y_meas, u_prev, MPC_SOLVER_HILDRETH);

        /* Apply move with clamping */
        double u_new = u_prev + du;
        if (u_new < config->valve_min) u_new = config->valve_min;
        if (u_new > config->valve_max) u_new = config->valve_max;

        double actual_du = u_new - u_prev;
        u_prev = u_new;
        tank->valve_position = u_new;

        /* Compute outflow */
        double F_out = surge_tank_outflow(tank, u_new);

        /* Feedforward addition for measured inlet disturbance change */
        if (k > 0) {
            double dF_in = F_in_hist[k] - F_in_hist[k-1];
            double du_ff = surge_feedforward(tank, dF_in);
            /* Blend feedforward with MPC: 30% FF */
            u_new += 0.3 * du_ff;
            if (u_new < config->valve_min) u_new = config->valve_min;
            if (u_new > config->valve_max) u_new = config->valve_max;
            tank->valve_position = u_new;
            F_out = surge_tank_outflow(tank, u_new);
        }

        /* Simulate tank dynamics */
        surge_tank_simulate(tank, F_in_hist[k], F_out, tuning->sampling_time);

        /* Record history */
        F_out_hist[k]  = F_out;
        level_hist[k]  = tank->level;
        valve_hist[k]  = tank->valve_position;

        /* Update KPIs */
        if (kpi) {
            double error = tank->level - config->level_setpoint;
            kpi->integral_squared_error += error * error;
            kpi->integral_absolute_error += fabs(error);
            kpi->integral_time_absolute_error +=
                (double)(k + 1) * tuning->sampling_time * fabs(error);
            kpi->mv_variance += actual_du * actual_du;
            kpi->valve_travel_total += fabs(actual_du);
            kpi->sample_count++;

            /* Constraint violation */
            if (tank->level > config->level_hi_limit ||
                tank->level < config->level_lo_limit) {
                kpi->constraint_violation_pct += 1.0;
            }
        }
    }

    /* Finalize KPIs */
    if (kpi && kpi->sample_count > 0) {
        kpi->mv_variance /= (double)kpi->sample_count;
        kpi->constraint_violation_pct =
            100.0 * kpi->constraint_violation_pct / (double)kpi->sample_count;
        kpi->level_utilization_pct = 100.0 - kpi->constraint_violation_pct;

        /* Process capability estimate:
         * CpK = min((USL-μ)/(3σ), (μ-LSL)/(3σ)) */
        double std_dev = 0.0, mean = 0.0;
        for (int k = 0; k < n_steps; k++) mean += level_hist[k];
        mean /= (double)n_steps;
        for (int k = 0; k < n_steps; k++) {
            double diff = level_hist[k] - mean;
            std_dev += diff * diff;
        }
        std_dev = sqrt(std_dev / (double)n_steps);

        if (std_dev > 1e-10) {
            double cpk_hi = (config->level_hi_limit - mean) / (3.0 * std_dev);
            double cpk_lo = (mean - config->level_lo_limit) / (3.0 * std_dev);
            kpi->process_capability_cpk = (cpk_hi < cpk_lo) ? cpk_hi : cpk_lo;
        }
    }

    return 0;
}

/* ─── Utility Functions ────────────────────────────────────────────────── */

double surge_tank_volume(const surge_tank_t *tank) {
    if (!tank) return 0.0;
    return tank->cross_area * tank->level;
}

double surge_tank_residence_time(const surge_tank_t *tank, double F_out) {
    if (!tank || F_out <= 1e-10) return 1e6;
    return (tank->cross_area * tank->level) / F_out;
}

double surge_level_percent(const surge_tank_t *tank) {
    if (!tank || tank->max_level <= 0.0) return 0.0;
    return 100.0 * tank->level / tank->max_level;
}
