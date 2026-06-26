# Knowledge Graph — MPC for Integrating Process Level Control

## L1: Definitions (COMPLETE)

| # | Concept | C Type/Enum | Location |
|---|---------|------------|----------|
| 1 | Integrating process | `process_dynamics_t` | mpc_level_types.h |
| 2 | Level control modes (tight/averaging/surge) | `level_control_mode_t` | mpc_level_types.h |
| 3 | MPC prediction horizon N_p | `mpc_tuning_t.prediction_horizon` | mpc_level_types.h |
| 4 | MPC control horizon N_c | `mpc_tuning_t.control_horizon` | mpc_level_types.h |
| 5 | Move suppression λ | `mpc_tuning_t.move_suppression` | mpc_level_types.h |
| 6 | QP solver types | `mpc_solver_type_t` | mpc_level_types.h |
| 7 | QP problem formulation | `qp_problem_t` | mpc_level_types.h |
| 8 | Step response model (FSR) | `step_response_t` | mpc_level_types.h |
| 9 | Kalman filter configuration | `kalman_config_t` | mpc_level_types.h |
| 10 | Surge tank physical model | `surge_tank_t` | mpc_level_types.h |
| 11 | MPC performance KPIs | `mpc_kpi_t` | mpc_level_types.h |
| 12 | Operator HMI data | `mpc_operator_data_t` | mpc_level_types.h |
| 13 | CARIMA model (GPC) | `gpc_config_t` | mpc_level_types.h |
| 14 | Integrating state-space | `integrating_state_t` | mpc_level_types.h |
| 15 | API 2350 overfill category | `api2350_category_t` | mpc_level_constraints.h |
| 16 | IEC 61511 SIL level | `iec61511_sil_level_t` | mpc_level_constraints.h |
| 17 | Constraint specifications | `mpc_constraint_spec_t` | mpc_level_constraints.h |

## L2: Core Concepts (COMPLETE)

| # | Concept | Implementation | Location |
|---|---------|---------------|----------|
| 1 | Receding horizon principle | `dmc_step()` | mpc_dmc.c |
| 2 | Integrating vs self-regulating dynamics | `mpc_model_to_state_space()` | mpc_integrating_model.c |
| 3 | Feedforward disturbance compensation | `surge_feedforward()` | mpc_surge_tank.c |
| 4 | Offset-free tracking via disturbance estimation | `kalman_augment_disturbance()` | mpc_kalman_filter.c |
| 5 | Level averaging / flow smoothing trade-off | `surge_level_filter_factor()` | mpc_surge_tank.c |
| 6 | Buffer capacity and residence time | `mpc_residence_time()` | mpc_integrating_model.c |
| 7 | Hard vs soft constraints | `mpc_feasibility_recover()` | mpc_level_constraints.c |
| 8 | Anti-windup (MPC rate limiting) | `mpc_constraint_build_rate()` | mpc_level_constraints.c |
| 9 | Reference trajectory (exponential filter) | `dmc_reference_trajectory()` | mpc_dmc.c |
| 10 | Terminal constraint for stability | `dmc_integrating_terminal_penalty()` | mpc_dmc.c |

## L3: Engineering Structures (COMPLETE)

| # | Structure | Implementation | Location |
|---|-----------|---------------|----------|
| 1 | Dynamic matrix G (Toeplitz) | `dmc_build_dynamic_matrix()` | mpc_dmc.c |
| 2 | CARIMA model with Diophantine recursion | `gpc_carima_predict()` | mpc_gpc.c |
| 3 | ZOH discretization of integrating SS | `mpc_discretize_zoh()` | mpc_integrating_model.c |
| 4 | QP formulation for constrained MPC | `dmc_setup_qp()` | mpc_dmc.c |
| 5 | Kalman filter for augmented state | `kalman_step()` | mpc_kalman_filter.c |
| 6 | Augmented disturbance model (Muske-Badgwell) | `kalman_augment_disturbance()` | mpc_kalman_filter.c |
| 7 | Move blocking (reduced QP dimension) | `dmc_move_blocking()` | mpc_dmc.c |
| 8 | Free response computation (additive form) | `dmc_build_free_response()` | mpc_dmc.c |
| 9 | Joseph form covariance update | `kalman_update()` | mpc_kalman_filter.c |
| 10 | DMC bias correction structure | `dmc_error_correction()` | mpc_dmc.c |

## L4: Engineering Laws / Standards (COMPLETE)

| # | Standard/Law | Implementation | Location |
|---|-------------|---------------|----------|
| 1 | API 2350 Overfill Prevention | `api2350_overfill_config_t`, safety margin calculation | mpc_level_constraints.h/.c |
| 2 | IEC 61511 SIS for level | `iec61511_sil_config_t`, SIL margin | mpc_level_constraints.h/.c |
| 3 | ZOH discretization theorem | `mpc_discretize_zoh()` with Taylor series | mpc_integrating_model.c |
| 4 | Welch-Satterthwaite (dof for KF tuning) | Implicit in noise covariance selection | mpc_kalman_filter.c |
| 5 | Terminal constraint stability (Rawlings-Muske) | `dmc_integrating_terminal_penalty()` | mpc_dmc.c |
| 6 | Torricelli's law (valve flow) | `surge_tank_outflow()` | mpc_surge_tank.c |
| 7 | Mass balance (tank dynamics) | `surge_tank_simulate()` | mpc_surge_tank.c |
| 8 | KKT optimality conditions | Implicit in all QP solvers | mpc_qp_solver.c |
| 9 | Detectability condition (offset-free MPC) | `kalman_augment_disturbance()` | mpc_kalman_filter.c |
| 10 | Joseph form (covariance PSD guarantee) | `kalman_update()` | mpc_kalman_filter.c |

## L5: Algorithms (COMPLETE)

| # | Algorithm | Implementation | Complexity |
|---|----------|---------------|------------|
| 1 | DMC (Dynamic Matrix Control) | `dmc_step()` | O(N_p*N_c²) per step |
| 2 | GPC (Generalized Predictive Control) | `gpc_step()` | O(N_p*(na+nb)) |
| 3 | Goldfarb-Idnani active set QP | `qp_active_set_goldfarb_idnani()` | O(n³ + m³) worst case |
| 4 | Hildreth QP (box constraints) | `qp_hildreth()` | O(n²/iter) simple |
| 5 | Mehrotra interior point QP | `qp_interior_point()` | O(n³/iter) quadratic convergence |
| 6 | Cholesky decomposition + solve | `qp_cholesky_decomp()`, `_solve()` | O(n³/6) |
| 7 | Kalman filter (predict-update) | `kalman_predict()`, `kalman_update()` | O(n³) per step |
| 8 | DARE (steady-state Kalman gain) | `kalman_steady_state_gain()` | O(n³) per iteration |
| 9 | Diophantine recursion (GPC) | `gpc_diophantine_recursion()` | O(N_p * na) |
| 10 | Matrix exponential (scaling+squaring) | `mpc_matrix_exponential()` | O(n³ * log(||A||)) |
| 11 | Power iteration (condition number) | `dmc_condition_number()` | O(N_p * N_c * iter) |
| 12 | Ljung-Box whiteness test | `kalman_whiteness_test()` | O(n * L) |
| 13 | CARIMA multi-step prediction | `gpc_carima_predict()` | O(N_p*(na+nb)) |
| 14 | Step response model building | `mpc_step_response_from_model()` | O(N*T_sim) |

## L6: Canonical Problems (COMPLETE)

| # | Problem | Example/Demo | Location |
|---|---------|-------------|----------|
| 1 | Single tank level control | `dmc_level_control_demo.c` | examples/ |
| 2 | Surge tank flow averaging | `surge_tank_averaging_demo.c` | examples/ |
| 3 | DMC vs GPC comparison for integrating process | `gpc_vs_dmc_comparison.c` | examples/ |
| 4 | Steam drum level (shrink/swell) | Documented in course-alignment.md | docs/ |
| 5 | Distillation sump level | Documented | docs/ |

## L7: Industrial Applications (PARTIAL)

| # | Application | Evidence |
|---|------------|----------|
| 1 | Honeywell Profit Controller / AspenTech DMC3 | API 2350 compliance integration |
| 2 | Refinery surge drum management | `surge_tank_averaging_demo.c` simulation |
| 3 | ISO 9001 quality (process capability CpK) | `mpc_kpi_t.process_capability_cpk` |

## L8: Advanced Topics (PARTIAL)

| # | Topic | Implementation |
|---|-------|---------------|
| 1 | Move blocking for MPC | `dmc_move_blocking()` |
| 2 | Terminal cost for integrating process stability | `dmc_integrating_terminal_penalty()` |
| 3 | Constraint softening for feasibility | `qp_soften_output_constraints()` |
| 4 | Innovation whiteness monitoring | `kalman_whiteness_test()` |

## L9: Research Frontiers (PARTIAL - documented)

| # | Topic | Status |
|---|-------|--------|
| 1 | Nonlinear MPC for level | Documented, not implemented |
| 2 | Stochastic MPC with chance constraints | Documented |
| 3 | Economic MPC for inventory management | Documented |
| 4 | Distributed MPC for multi-tank systems | Documented |
| 5 | ML-based disturbance prediction | Documented |
