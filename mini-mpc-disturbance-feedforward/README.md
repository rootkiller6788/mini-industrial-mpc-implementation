# mini-mpc-disturbance-feedforward

**Model Predictive Control with Disturbance Feedforward**

## Module Status: COMPLETE

- L1 (Definitions): Complete
- L2 (Core Concepts): Complete
- L3 (Engineering Structures): Complete
- L4 (Engineering Laws/Theorems): Complete
- L5 (Algorithms/Methods): Complete
- L6 (Canonical Problems): Complete
- L7 (Industrial Applications): Partial (2 applications)
- L8 (Advanced Topics): Partial (3/5 topics implemented)
- L9 (Research Frontiers): Partial (documented)

## Line Count
- `include/` + `src/`: 3650+ lines
- Meets 3000-line minimum requirement

## Core Definitions
- `mpc_ss_model_t`: Discrete-time state-space model with disturbance channels
- `mpc_tuning_t`: MPC horizons and weighting matrices
- `mpc_constraints_t`: Box constraints on inputs, rates, outputs, states
- `mpc_aug_model_t`: Augmented model for offset-free control
- `mpc_prediction_t`: Prediction matrices (Phi, Gamma, Phi_d)
- `mpc_qp_t`: Quadratic Programming problem structure
- `mpc_kalman_params_t`: Kalman filter noise covariances
- `mpc_ff_config_t`: Feedforward controller configuration

## Core Theorems
- **Internal Model Principle** (Francis & Wonham 1976): Integrating disturbances for offset-free tracking
- **Van Loan** (1978): Exact ZOH discretization via augmented matrix exponential
- **Kleinman** (1968): DARE iterative solution convergence
- **KKT Conditions**: Optimality for constrained QP
- **Separation Principle**: LQG = optimal estimation + deterministic control
- **Kalman Duality**: Observability rank condition

## Core Algorithms
1. Matrix exponential (Pade [6/6] + scaling-and-squaring)
2. ZOH/Tustin discretization
3. Prediction matrix construction (Phi, Gamma)
4. Active-set QP solver
5. Kalman filter (predict + update)
6. Disturbance observer (DOB)
7. Static feedforward gain computation
8. DARE solution (Kleinman iteration)
9. Step response model construction

## Canonical Problems
1. CSTR temperature control with feed disturbance
2. Level control with inflow disturbance
3. Distillation column composition control
4. Quadcopter with wind disturbance

## Course Mappings
| Course | School | Topics Covered |
|--------|--------|---------------|
| 6.302 Feedback Systems | MIT | State-space, optimal control |
| ENGR205 Process Control | Stanford | MPC formulation, disturbance models |
| ME233 Advanced Control | Berkeley | Discrete-time LTI, estimation |
| 24-677 Adv Ctrl Systems | CMU | MPC with disturbance models |
| ECE 6550 Nonlinear Control | Georgia Tech | Constrained optimization |
| ME 575 Industrial Control | Purdue | Feedforward design |
| Industrial Control Systems | RWTH Aachen | PLC/DCS integration |
| Process Control Engineering | Tsinghua | Industrial MPC applications |

## Build and Test
```
make          # Build static library libmpc.a
make test     # Build and run tests
make examples # Build examples
make clean    # Clean build artifacts
```

## References
- Rawlings, Mayne, Diehl, "Model Predictive Control: Theory, Computation, and Design" (2017)
- Maciejowski, "Predictive Control with Constraints" (2002)
- Camacho & Bordons, "Model Predictive Control", 2nd Ed. (2007)
- Nocedal & Wright, "Numerical Optimization" (2006)
- Simon, "Optimal State Estimation" (2006)
- Qin & Badgwell, "A Survey of Industrial MPC Technology" (2003)
