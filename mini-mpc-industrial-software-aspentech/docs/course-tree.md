# Course Tree — mini-mpc-industrial-software-aspentech

## Prerequisite Dependency Tree

This module builds on knowledge from earlier mini-modules. Below is the dependency graph.

---

## Direct Dependencies

```
mini-mpc-industrial-software-aspentech
├── 1. mini-pid-control-engineering
│   └── Feedback control principles, setpoint tracking, disturbance rejection
├── 2. mini-advanced-pid-tuning
│   └── Controller tuning methodology, performance assessment
├── 3. mini-feedforward-cascade-ratio
│   └── Feedforward DV handling, cascade structure
├── 10. mini-safety-instrumented-system
│   └── Constraint handling philosophy, hard vs soft limits
├── 11. mini-industrial-real-time-database
│   └── Data buffering, historian concepts for MHE
├── 12. mini-advanced-process-control-apc
│   └── APC architecture, DMC theory fundamentals
└── 14. mini-soft-sensor-inferential
    └── Inferential CVs, Kalman filter concepts
```

---

## Internal Knowledge Dependencies

```
L1: Definitions (no prerequisites)
├── L2: Core Concepts
│   ├── Requires: L1 (MV, CV, DV, P, M, FIR)
│   ├── L3: Engineering Structures
│   │   ├── Requires: L1, L2
│   │   ├── L4: Engineering Laws
│   │   │   ├── Requires: L1-L3
│   │   │   ├── L5: Algorithms
│   │   │   │   ├── Requires: L1-L4
│   │   │   │   ├── L6: Canonical Problems
│   │   │   │   │   ├── Requires: L1-L5
│   │   │   │   │   ├── L7: Industrial Applications
│   │   │   │   │   │   ├── Requires: L1-L6
│   │   │   │   │   │   ├── L8: Advanced Topics
│   │   │   │   │   │   │   ├── Requires: L1-L7
│   │   │   │   │   │   │   └── L9: Research Frontiers
│   │   │   │   │   │   │       └── Requires: L1-L8
```

---

## Mathematical Prerequisites

| Topic | Required For | Reference |
|-------|-------------|-----------|
| Linear Algebra (matrix mult, inverse, eigenvalues) | Dynamic matrix, SVD, Gramians | Golub & Van Loan (2013) |
| Convex Optimization (QP, LP KKT) | QP solvers, LP targets | Nocedal & Wright (2006) |
| Probability/Statistics (Gaussian, convergence) | Kalman filter, RLS | Ljung (1999) |
| Difference Equations (z-transform) | Discrete convolution | Astrom & Wittenmark (2008) |
| Numerical Methods (pivoting, regularization) | Linear system solver, conditioning | Golub & Van Loan (2013) |
| System Identification (persistent excitation) | N4SID, step testing | Ljung (1999) |

---

## Topic Dependency Map

```
FOPDT Model ──> Step-Response Model ──> Dynamic Matrix
                                           │
                                           v
                            DMC Prediction ──> Bias Update
                                  │
                                  v
                            QP Formulation ──> QP Solver ──> First Move
                                  │                             │
                                  v                             v
                            LP Targets                    Horizon Shift
                                  │                             │
                                  └──────> DMC Step <───────────┘
                                               │
                                               v
                                        DMC3 Simulation
                                               │
                         ┌─────────────────────┼─────────────────────┐
                         v                     v                     v
                   State-Space MPC      Adaptive MPC (RLS)    Robust MPC
                         │                     │                     │
                         v                     v                     v
                  Observability         Kalman Filter        MHE (Estimation)
                         │
                         v
                   N4SID ID ──> Model Conversion
```

---

## L9 Research Frontier Dependencies

| Frontier | Prerequisites in This Module |
|----------|------------------------------|
| Cloud-Edge MPC | DMC3 simulation, DCS interface |
| AI-Enhanced MPC | RLS adaptation, Kalman filter, QP solver |
| Autonomous L4 | All L1-L8 content, diagnostics, model quality monitor |
| Digital Twin | State-space MPC, N4SID, MHE |

---

## Recommended Learning Path

1. Start with L1 (mpc_common.h) — understand all structs and enums
2. Read dmc_model.c — grasp FOPDT -> FIR conversion and convolution prediction
3. Study dynamic_matrix.c — understand Toeplitz structure
4. Work through mpc_controller.c — master the complete DMC cycle
5. Understand qp_solver.c and ss_target.c — optimization layer
6. Explore state_space_mpc.c — alternative formulation
7. Advanced: mpc_adaptation.c (RLS, KF), mpc_advanced.c (NMPC, Robust, MHE)
8. Application: aspen_dmc3.c (closed-loop), model_identification.c (SmartStep)
9. Diagnostics: mpc_diagnostics.c (Harris index, model quality)
