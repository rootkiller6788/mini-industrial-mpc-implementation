# Course Tree — mini-mpc-constraint-handling-priority

## Prerequisites

```
Linear Algebra (matrix operations, norms)
    │
    ├── Convex Optimization (QP, KKT conditions)
    │       │
    │       └── Numerical Optimization (active-set, interior-point)
    │
    ├── Model Predictive Control (MPC fundamentals)
    │       │
    │       ├── DMC/GPC formulation
    │       ├── Prediction & control horizons
    │       └── State-space MPC
    │
    └── Process Control (industrial context)
            │
            ├── PID control basics
            ├── MIMO systems
            └── Process constraints & safety
```

## Module Dependency Tree

```
mini-mpc-constraint-handling-priority
│
├── [DEPENDS ON]
│   ├── mini-dmc-dynamic-matrix-control           (MPC algorithm fundamentals)
│   ├── mini-gpc-generalized-predictive-control   (GPC formulation)
│   ├── mini-pid-control-engineering              (basic constrained control)
│   └── mini-advanced-process-control-apc         (APC architecture)
│
├── [FEEDS INTO]
│   ├── mini-mpc-performance-monitoring-kpi       (constraint violation KPIs)
│   ├── mini-mpc-operator-training-interface      (operator display of constraints)
│   ├── mini-mpc-industrial-software-aspentech    (DMC3 deep dive)
│   ├── mini-mpc-disturbance-feedforward          (constraints under disturbance)
│   ├── mini-mpc-integrating-process-level        (level control constraints)
│   ├── mini-mpc-ill-conditioned-process          (conditioning analysis)
│   └── mini-mpc-subspace-identification          (model constraints)
│
├── [KNOWLEDGE AREAS COVERED]
│   ├── L1: Constraint types, priorities, feasibility status
│   ├── L2: Hard/soft classification, violation detection, slack variables
│   ├── L3: Priority indexing, active sets, QP formulations
│   ├── L4: KKT conditions, Farkas lemma, Hoffman bound, exact penalty
│   ├── L5: Active-set QP, sequential relaxation, IIS detection, lexicographic MPC
│   ├── L6: Input saturation, output prioritization, feasibility recovery
│   ├── L7: AspenTech DMC3, Honeywell RMPCT, Shell SMOC, ABB Predict
│   ├── L8: Explicit MPC regions, constraint funnels, conditioning analysis
│   └── L9: IT/OT convergence, autonomous operations (documented)
│
└── [LEARNING PATH]
    1. Understand constraint types and priorities (L1)
    2. Grasp hard vs soft constraint tradeoffs (L2)
    3. Review QP formulation and KKT conditions (L3-L4)
    4. Study active-set QP algorithm (L5)
    5. Practice with input saturation and output prioritization (L6)
    6. Compare industrial vendor approaches (L7)
    7. Explore advanced topics: explicit MPC, funnels (L8)
    8. Read about future directions (L9)
