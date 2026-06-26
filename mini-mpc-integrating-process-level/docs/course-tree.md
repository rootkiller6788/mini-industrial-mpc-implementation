# Course Tree — MPC for Integrating Process Level Control

## Prerequisites

```
L1: Control Theory Basics
├── L1.1 Transfer functions and state-space
├── L1.2 Stability (Lyapunov, pole placement)
├── L1.3 PID control fundamentals
└── L1.4 Linear algebra (matrix ops, Cholesky)

L2: Process Dynamics
├── L2.1 First/second-order systems
├── L2.2 Integrating (non-self-regulating) processes
├── L2.3 Discretization (ZOH, Tustin)
└── L2.4 Time delay systems

L3: Optimization
├── L3.1 Convex optimization
├── L3.2 Quadratic programming (KKT conditions)
├── L3.3 Active set methods
└── L3.4 Interior point methods

L4: State Estimation
├── L4.1 Observability and detectability
├── L4.2 Kalman filter (predict-update)
├── L4.3 DARE and steady-state gain
└── L4.4 Disturbance modeling (Muske-Badgwell)
```

## This Module's Position

```
mini-mpc-integrating-process-level/
├── Prerequisites above
├── → mini-pid-control-engineering (PID for baseline comparison)
├── → mini-advanced-process-control-apc (APC fundamentals)
└── Leads to:
    ├── mini-mpc-constraint-handling-priority
    ├── mini-mpc-disturbance-feedforward
    └── mini-mpc-performance-monitoring-kpi
```

## Knowledge Dependencies

| This Module's Topic | Depends On |
|--------------------|-----------|
| DMC algorithm | Step response models, QP solver |
| GPC algorithm | CARIMA models, Diophantine equation |
| Kalman filter | State-space models, linear algebra |
| QP solver | Convex optimization, KKT conditions |
| Surge tank | Mass balance, Torricelli's law |
| API 2350 | Hazard analysis, SIS design (IEC 61511) |
| Terminal stability | Lyapunov theory, invariant sets |
