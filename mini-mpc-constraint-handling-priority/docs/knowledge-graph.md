# Knowledge Graph — mini-mpc-constraint-handling-priority

## L1: Definitions ✅ Complete

| # | Concept | C Implementation | Lean Formalization |
|---|---------|-----------------|-------------------|
| 1 | Constraint Type (HARD/SOFT/COUPLING) | `mpc_constraint_type_t` enum | `ConstraintCategory` inductive |
| 2 | Priority Level (CRITICAL/HIGH/MEDIUM/LOW/MONITOR) | `mpc_priority_level_t` enum | `PriorityLevel` inductive |
| 3 | Feasibility Status (FEASIBLE/INFEASIBLE/RECOVERABLE) | `mpc_feasibility_status_t` enum | — |
| 4 | Constraint Scope (INPUT/RATE/OUTPUT/TERMINAL) | `mpc_constraint_scope_t` enum | `ConstraintScope` inductive |
| 5 | Relaxation Policy (NEVER/IF_NEEDED/ALWAYS_SOFT/SEQUENTIAL) | `mpc_relaxation_policy_t` enum | `RelaxationPolicy` inductive |
| 6 | Single Constraint (bounds + coefficients + slack) | `mpc_constraint_t` struct | `Constraint` structure |
| 7 | Constraint Set (collection with priority groups) | `mpc_constraint_set_t` struct | `ConstraintSet` structure |
| 8 | QP Solution with Priority | `mpc_qp_solution_t` struct | — |
| 9 | Input Saturation Info | `mpc_input_saturation_t` struct | `InputSaturation` structure |
| 10 | Output Prioritization | `mpc_output_prioritization_t` struct | `OutputPrioritization` structure |

## L2: Core Concepts ✅ Complete

| # | Concept | Implementation |
|---|---------|---------------|
| 1 | Hard vs Soft constraint classification | `mpc_constraint_is_hard()` / `mpc_constraint_is_soft()` |
| 2 | Constraint violation detection | `mpc_constraint_check_violation()` |
| 3 | Priority ordering (lower = more important) | `isHigherPriority` in Lean, `mpc_constraint_sort_by_priority()` in C |
| 4 | Slack variable concept (absorb violation at penalty) | `mpc_relaxation_alloc_slacks()` |
| 5 | Exact penalty theorem (Fletcher, 1987) | `mpc_relaxation_set_exact_penalty()` |
| 6 | Lexicographic optimization (priorities sequentially) | `mpc_qp_solve_prioritized_sequential()` |
| 7 | Farkas' Lemma for infeasibility certificate | `mpc_feasibility_farkas_certificate()` |
| 8 | Hoffman's bound (distance to feasible set) | `mpc_feasibility_hoffman_bound()` |

## L3: Engineering Structures ✅ Complete

| # | Structure | Implementation |
|---|-----------|---------------|
| 1 | Priority group indexing | `mpc_constraint_build_priority_index()` |
| 2 | Active set for QP | `mpc_active_set_t` struct |
| 3 | Slack variable arrays | `mpc_relaxation_state_t` |
| 4 | Constraint propagation in prediction horizon | `mpc_constraint_propagation_t` |
| 5 | Vendor configuration | `mpc_vendor_config_t` |
| 6 | QP problem formulation | `mpc_qp_problem_t` |

## L4: Engineering Laws/Theorems ✅ Complete

| # | Theorem/Law | Implementation |
|---|------------|---------------|
| 1 | KKT optimality conditions for QP | `mpc_qp_verify_kkt()` |
| 2 | Fletcher's exact penalty theorem | `mpc_relaxation_set_exact_penalty()` |
| 3 | Farkas' Lemma (infeasibility alternative) | `mpc_feasibility_farkas_certificate()` |
| 4 | Hoffman's error bound | `mpc_feasibility_hoffman_bound()` |
| 5 | Complementary slackness | `mpc_relaxation_check_kkt_slacks()` |
| 6 | Priority transitivity | `priority_transitive` theorem (Lean) |
| 7 | Priority antisymmetry | `priority_antisymmetric` theorem (Lean) |

## L5: Algorithms/Methods ✅ Complete

| # | Algorithm | Implementation |
|---|-----------|---------------|
| 1 | Primal active-set QP solver | `mpc_qp_solve_active_set()` |
| 2 | Sequential priority relaxation | `mpc_relaxation_sequential_by_priority()` |
| 3 | Deletion filter for IIS | `mpc_feasibility_find_iis()` |
| 4 | Constraint sorting by priority (qsort) | `mpc_constraint_sort_by_priority()` |
| 5 | Penalty weight auto-tuning | `mpc_relaxation_auto_tune_penalty_weights()` |
| 6 | Priority-based conflict resolution | `mpc_constraint_resolve_conflict()` |
| 7 | Lexicographic MPC | `mpc_lexicographic_solve()` |
| 8 | Input saturation detection | `mpc_detect_input_saturation()` |
| 9 | Desaturation path planning | `mpc_compute_desaturation_path()` |
| 10 | Shadow price / sensitivity analysis | `mpc_constraint_shadow_price()` |

## L6: Canonical Problems ✅ Complete

| # | Problem | Implementation |
|---|---------|---------------|
| 1 | Input saturation management | `mpc_detect_input_saturation()` + example |
| 2 | Multi-CV output prioritization (FCC unit) | `mpc_output_prioritization_*()` + example |
| 3 | Constraint relaxation for feasibility recovery | `mpc_relaxation_sequential_by_priority()` + example |
| 4 | Priority inheritance for cascaded constraints | `mpc_constraint_inherit_priority()` |
| 5 | Feasibility recovery planning | `mpc_feasibility_recovery_plan()` |

## L7: Industrial Applications ⚡ Partial (4/7)

| # | Application | Implementation |
|---|------------|---------------|
| 1 | AspenTech DMC3 sequential QP | `mpc_aspen_dmc3_priority_solve()` ✅ |
| 2 | Honeywell RMPCT range control | `mpc_honeywell_rmpct_range_control()` ✅ |
| 3 | Shell SMOC soft constraint setup | `mpc_shell_smoc_constraint_setup()` ✅ |
| 4 | ABB Predict & Control economic ranking | `mpc_abb_predict_economic_ranking()` ✅ |
| 5 | Rockwell Pavilion8 | Documented only 📋 |
| 6 | Siemens SIMATIC PCS 7 APC | Documented only 📋 |
| 7 | Yokogawa Exasmoc | Not covered |

## L8: Advanced Topics ⚡ Partial (5/8)

| # | Topic | Implementation |
|---|-------|---------------|
| 1 | Explicit MPC critical regions | `mpc_qp_compute_critical_region()` ✅ |
| 2 | Constraint funnel management | `mpc_constraint_funnel_update()` ✅ |
| 3 | Lexicographic MPC | `mpc_lexicographic_solve()` ✅ |
| 4 | Constraint conditioning analysis | `mpc_constraint_conditioning_analysis()` ✅ |
| 5 | KKT slack optimality verification | `mpc_relaxation_check_kkt_slacks()` ✅ |
| 6 | Stochastic constraint handling | Documented only 📋 |
| 7 | Learning constraint sets from data | Not covered |
| 8 | Distributed MPC constraint coordination | Not covered |

## L9: Industry Frontiers 📋 Partial

- IT/OT convergence for constraint management
- 5G-enabled real-time constraint updates
- Autonomous constraint tuning (L4 Autonomous Operations)
- Zero-trust constraint validation
