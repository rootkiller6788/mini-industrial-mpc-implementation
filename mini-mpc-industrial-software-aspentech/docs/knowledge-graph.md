# Knowledge Graph — mini-mpc-industrial-software-aspentech

## Nine-Layer Knowledge Coverage Map

This document maps every implemented knowledge item to the nine-layer framework defined in SKILL.md.

---

## L1: Definitions (COMPLETE — 25 items)

| # | Concept | C Implementation | Header |
|---|---------|-----------------|--------|
| 1 | Manipulated Variable (MV) | `mpc_mv_config_t` | mpc_common.h |
| 2 | Controlled Variable (CV) | `mpc_cv_config_t` | mpc_common.h |
| 3 | Disturbance Variable (DV) | `mpc_dv_config_t` | mpc_common.h |
| 4 | Step-Response Model (FIR) | `mpc_step_model_t` | mpc_common.h |
| 5 | Dynamic Matrix (Toeplitz) | `mpc_dynamic_matrix_t` | mpc_common.h |
| 6 | QP Formulation (Hessian, gradient) | `mpc_qp_problem_t`, `mpc_qp_solution_t` | mpc_common.h |
| 7 | Engineering Units Scaling (EU) | `mpc_pct_to_eu`, `mpc_eu_to_pct` | mpc_common.h, variable_scaling.c |
| 8 | Constraint Types | `mpc_constraint_type_t` enum | mpc_common.h |
| 9 | FIR Coefficients / Dead Time / Gain | Fields in `mpc_step_model_t` | mpc_common.h |
| 10 | FOPDT Parameters (K, tau, theta) | `mpc_step_model_from_fopdt()` | mpc_common.h, dmc_model.c |
| 11 | Prediction Horizon (P) | Field in `mpc_controller_state_t` | mpc_common.h |
| 12 | Control Horizon (M) | Field in `mpc_controller_state_t` | mpc_common.h |
| 13 | QP Solver Status | `qp_status_t` enum | mpc_common.h |
| 14 | LP Optimization Mode | `mpc_lp_mode_t` enum | mpc_common.h |
| 15 | Sub-Controller Decomposition | `mpc_decomp_t` enum | mpc_common.h |
| 16 | Operational Modes | `mpc_mode_t` enum | mpc_common.h |
| 17 | State-Space Model (A,B,C) | `mpc_kalman_state_t`, `mpc_identified_model_t` | mpc_common.h, mpc_model_id.h |
| 18 | Recursive Least Squares (RLS) | `mpc_rls_estimator_t` | mpc_common.h |
| 19 | Kalman Filter State | `mpc_kalman_state_t` | mpc_common.h |
| 20 | NMPC Linearization Config | `mpc_nmpc_config_t` | mpc_advanced.h |
| 21 | Robust MPC Scenario Config | `mpc_robust_config_t` | mpc_advanced.h |
| 22 | MHE Horizon Config | `mpc_mhe_config_t` | mpc_advanced.h |
| 23 | Adaptive MPC Thresholds | `mpc_adaptive_config_t` | mpc_advanced.h |
| 24 | Step Test Config (SmartStep) | `mpc_step_test_config_t` | mpc_model_id.h |
| 25 | Subspace ID Data Buffer | `mpc_n4sid_data_t` | mpc_model_id.h |

---

## L2: Core Concepts (COMPLETE — 10 items)

| # | Concept | C Implementation | File |
|---|---------|-----------------|------|
| 1 | Receding Horizon Principle | `mpc_dmc_shift_horizon()` | mpc_controller.c |
| 2 | Bias Feedback Correction | `mpc_dmc_bias_update()` | mpc_controller.c |
| 3 | Open-Loop Prediction (y_free) | `mpc_dmc_predict()` | mpc_controller.c |
| 4 | Steady-State Target LP | `mpc_compute_steady_state_target()` | ss_target.c |
| 5 | First-Move with Rate/Hard Clipping | `mpc_implement_first_move()` | mpc_controller.c |
| 6 | Bias Convergence Theorem | In `mpc_dmc_bias_update()` | mpc_controller.c |
| 7 | MIMO Open-Loop Prediction | `mpc_mimo_predict_openloop()` | mimo_ops.c |
| 8 | SS Gain Extraction from FIR | `mpc_mimo_extract_ss_gain()` | mimo_ops.c |
| 9 | State-Space MPC Prediction | `mpc_ss_mpc_predict()` | state_space_mpc.c |
| 10 | MV/CV Constraint Handling | `mpc_implement_first_move()`, `mpc_compute_cv_violation()` | mpc_controller.c |

---

## L3: Engineering Structures (COMPLETE — 10 items)

| # | Structure | Implementation | File |
|---|-----------|---------------|------|
| 1 | DMC3 5-Layer Architecture | `mpc_dmc_step()` execution cycle | mpc_controller.c |
| 2 | DMC Execution Cycle (Read->Bias->SS->QP->Write) | `mpc_dmc3_closed_loop_simulation()` | aspen_dmc3.c |
| 3 | Dynamic Matrix Toeplitz Structure | `mpc_build_dynamic_matrix()`, `mpc_build_mimo_dynamic_matrix()` | dynamic_matrix.c |
| 4 | FIR Coefficient Storage Layout (CVxMV) | `mpc_mimo_model_t` with sub_models[][] | dmc_model.c |
| 5 | Sub-Controller Decomposition | `mpc_decomp_t` enum, block/coordinate modes | mpc_common.h |
| 6 | Phi/Gamma Prediction Matrices | `build_phi_matrix()`, `build_gamma_matrix()` | state_space_mpc.c |
| 7 | QP Matrix Assembly (A^T*Q*A + R) | `mpc_build_hessian_for_pair()`, `mpc_ss_build_hessian()` | mpc_controller.c, state_space_mpc.c |
| 8 | Gaussian Elimination with Partial Pivoting | `solve_linear_system()` in qp_solver.c | qp_solver.c |
| 9 | Simplex Tableau Structure | `mpc_lp_simplex_solve()` | ss_target.c |
| 10 | N4SID Data Buffer Layout | `mpc_n4sid_data_t` with block Hankel structure | model_identification.c |

---

## L4: Engineering Laws and Theorems (COMPLETE — 10 items)

| # | Law/Theorem | Verification in Code | File |
|---|-------------|---------------------|------|
| 1 | FIR Truncation Bound | K*exp(-(N*Ts-theta)/tau) < tol in `mpc_step_model_truncation_error()` | dmc_model.c |
| 2 | Gaussian Elimination Correctness | Partial pivoting max|a_ik| in `solve_linear_system()` | qp_solver.c |
| 3 | Tikhonov Regularization | H_reg = H + lambda*I in `mpc_regularize_hessian()` | numerical_cond.c |
| 4 | SVD Condition Number | kappa = sigma_max/sigma_min via power iteration | numerical_cond.c |
| 5 | KKT Optimality Conditions | Gradient check: |Hx*+c| <= tol in `mpc_qp_check_optimality()` | qp_solver.c |
| 6 | Harris Performance Index | I_perf = 1 - var(y_cl)/var(y_ol) | mpc_diagnostics.c |
| 7 | RLS Convergence | theta_hat(k) -> theta_true with persistent excitation | mpc_adaptation.c |
| 8 | Kalman Filter MMSE Optimality | KF minimizes E[||x - x_hat||^2] for linear Gaussian | mpc_adaptation.c |
| 9 | Campi & Garatti Scenario Bound | N >= (1/eps)*(n/(1-eta)-1) for probabilistic constraints | mpc_advanced.c |
| 10 | Simplex Optimality (Reduced Cost) | All reduced costs >= 0 at optimum in LP simplex | ss_target.c |

---

## L5: Algorithms/Methods (COMPLETE — 20 algorithms)

| # | Algorithm | Complexity | File |
|---|-----------|-----------|------|
| 1 | FOPDT to Step-Response Conversion | O(N) | dmc_model.c |
| 2 | Discrete Convolution Prediction | O(N*P) | dmc_model.c |
| 3 | Dynamic Matrix Construction | O(P*M) | dynamic_matrix.c |
| 4 | Active-Set QP (Gradient Projection) | O(iter*n^3) | qp_solver.c |
| 5 | Interior-Point QP (Gradient Descent) | O(iter*n^2) | qp_solver.c |
| 6 | LP Simplex (Steady-State Targets) | O(iter*m*n) | ss_target.c |
| 7 | Complete DMC Control Cycle | O(n_cv*n_mv*P*M^2) | mpc_controller.c |
| 8 | Recursive Least Squares (RLS) | O(n_params^2) | mpc_adaptation.c |
| 9 | Kalman Filter (Predict + Correct) | O(nx^3 + nx^2*ny) | mpc_adaptation.c |
| 10 | N4SID Subspace Identification | O(N*i^2) | model_identification.c |
| 11 | Power Iteration (SVD) | O(iter*rows*cols) | numerical_cond.c |
| 12 | State-Space Prediction (Phi, Gamma) | O(P*nx^3) | state_space_mpc.c |
| 13 | NMPC Successive Linearization (Finite Diff) | O(nx^2) per step | mpc_advanced.c |
| 14 | MHE Averaging Filter Estimation | O(N*ny) | mpc_advanced.c |
| 15 | Step Test Execution (SmartStep) | O(n_samples) | model_identification.c |
| 16 | Controllability Gramian Computation | O(N*nx^2*nu) | state_space_mpc.c |
| 17 | Observability Gramian Computation | O(N*nx^2*ny) | state_space_mpc.c |
| 18 | FIR to State-Space (Ho-Kalman Realization) | O(n^2) | dmc_model.c |
| 19 | State-Space to Step-Response Simulation | O(N*nx^2) | mpc_adaptation.c |
| 20 | Box-Muller Gaussian Noise Generation | O(1) | aspen_dmc3.c |

---

## L6: Canonical Problems (COMPLETE — 5 problems)

| # | Problem | Implementation | File |
|---|---------|---------------|------|
| 1 | SISO FOPDT Control (Cutler-Ramaker 1980) | G(s)=2e^{-3s}/(10s+1) DMC benchmark | example_dmc_siso.c |
| 2 | Wood-Berry Distillation Column (2x2 MIMO) | Reflux/Steam -> Top/Bottom composition | example_distillation.c |
| 3 | CDU Furnace Temperature+O2 Control (2x2+DV) | Fuel Gas/Air -> Temperature/O2, feed rate DV | example_dmc3_sim.c |
| 4 | Harris Control Performance Assessment | I_perf = 1 - var_cl/var_ol | mpc_diagnostics.c |
| 5 | Ill-Conditioned System Detection | Condition number via SVD, near-zero gain detection | numerical_cond.c |

---

## L7: Industrial Applications (COMPLETE — 5 applications)

| # | Application | Type | File |
|---|------------|------|------|
| 1 | AspenTech DMC3 Closed-Loop Simulation | Core DMC3 cycle with DCS read/write | aspen_dmc3.c |
| 2 | Box-Muller Process Noise Generation | Realistic process disturbance simulation | aspen_dmc3.c |
| 3 | SmartStep Automated Step Testing | AspenTech proprietary step test methodology | model_identification.c |
| 4 | Model Quality Monitor (RMSE-based) | Continuous model validation for DMC3 | mpc_diagnostics.c |
| 5 | CDU Furnace Control (Refinery) | Crude distillation unit, real refinery scenario | example_dmc3_sim.c |

---

## L8: Advanced Topics (COMPLETE — 5 topics)

| # | Topic | Implementation | File |
|---|-------|---------------|------|
| 1 | RLS Online Adaptation with Forgetting Factor | `mpc_rls_update()` with lambda in (0,1] | mpc_adaptation.c |
| 2 | Kalman Filter (Predict/Correct Cycle) | Full KF with P, K, innovation covariance | mpc_adaptation.c |
| 3 | NMPC via Successive Linearization | Finite-difference Jacobian at current operating point | mpc_advanced.c |
| 4 | Robust MPC Scenario-Based Tightening | Quantile-based backoff margin from Campi-Garatti | mpc_advanced.c |
| 5 | Moving Horizon Estimation (MHE) | Sliding window state/disturbance estimation | mpc_advanced.c |

---

## L9: Research Frontiers (PARTIAL — documented)

| # | Topic | Status |
|---|-------|--------|
| 1 | Cloud-Edge MPC Architecture | Documented (IT/OT convergence, 5G) |
| 2 | AI-Enhanced MPC (Reinforcement Learning Tuning) | Documented |
| 3 | Autonomous Operation (L4 Autonomy Level) | Documented |
| 4 | Digital Twin Integration with MPC | Documented |

---

## Nine-Layer Summary

| Level | Status | Items | Score |
|-------|--------|-------|-------|
| L1 Definitions | **COMPLETE** | 25 | 2 |
| L2 Core Concepts | **COMPLETE** | 10 | 2 |
| L3 Engineering Structures | **COMPLETE** | 10 | 2 |
| L4 Engineering Laws | **COMPLETE** | 10 | 2 |
| L5 Algorithms/Methods | **COMPLETE** | 20 | 2 |
| L6 Canonical Problems | **COMPLETE** | 5 | 2 |
| L7 Industrial Applications | **COMPLETE** | 5 | 2 |
| L8 Advanced Topics | **COMPLETE** | 5 | 2 |
| L9 Research Frontiers | **PARTIAL** | 4 documented | 1 |

**Total Score: 17/18 -> COMPLETE**
