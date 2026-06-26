/**
 * mpc_level_types.c — Core Type Definitions Implementation
 *
 * Implements string conversion utilities for MPC types.
 * Each enum has a corresponding name function for operator displays
 * and diagnostic/logging purposes.
 *
 * Knowledge: L1 - Definitions (enum → string mapping for HMI/diagnostics)
 */

#include "../include/mpc_level_types.h"

const char* process_dynamics_name(process_dynamics_t type) {
    switch (type) {
        case PROCESS_INTEGRATING:    return "Integrating";
        case PROCESS_SELF_REGULATING: return "Self-Regulating";
        case PROCESS_RUN_AWAY:       return "Run-Away (Unstable)";
        case PROCESS_DOUBLE_INT:     return "Double Integrating";
        default:                     return "Unknown";
    }
}

const char* level_control_mode_name(level_control_mode_t mode) {
    switch (mode) {
        case LEVEL_MODE_TIGHT:     return "Tight Level Control";
        case LEVEL_MODE_AVERAGING: return "Level Averaging";
        case LEVEL_MODE_SURGE:     return "Surge Tank Mode";
        default:                   return "Unknown";
    }
}

const char* mpc_solver_name(mpc_solver_type_t solver) {
    switch (solver) {
        case MPC_SOLVER_ACTIVE_SET:    return "Goldfarb-Idnani Active Set";
        case MPC_SOLVER_HILDRETH:      return "Hildreth QP";
        case MPC_SOLVER_INTERIOR_POINT: return "Mehrotra Interior Point";
        case MPC_SOLVER_GRADIENT_PROJ: return "Gradient Projection";
        default:                        return "Unknown";
    }
}

const char* qp_status_string(qp_status_t status) {
    switch (status) {
        case QP_OPTIMAL:         return "Optimal";
        case QP_INFEASIBLE:      return "Infeasible";
        case QP_UNBOUNDED:       return "Unbounded";
        case QP_MAX_ITERATIONS:  return "Max Iterations";
        case QP_NUMERICAL_ERROR: return "Numerical Error";
        case QP_NOT_SOLVED:      return "Not Solved";
        default:                 return "Unknown";
    }
}

const char* constraint_type_name(constraint_type_t ct) {
    switch (ct) {
        case CONSTRAINT_HARD_INPUT:     return "Hard Input";
        case CONSTRAINT_SOFT_INPUT:     return "Soft Input";
        case CONSTRAINT_HARD_OUTPUT:    return "Hard Output";
        case CONSTRAINT_SOFT_OUTPUT:    return "Soft Output";
        case CONSTRAINT_RATE_OF_CHANGE: return "Rate of Change";
        case CONSTRAINT_TERMINAL:       return "Terminal Equality";
        case CONSTRAINT_TERMINAL_SET:   return "Terminal Set";
        default:                        return "Unknown";
    }
}
