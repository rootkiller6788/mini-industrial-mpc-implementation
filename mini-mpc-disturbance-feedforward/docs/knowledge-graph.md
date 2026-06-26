# Knowledge Graph: MPC with Disturbance Feedforward

## L1: Definitions (Complete)
- MPC state-space model (A, B, C, D matrices)
- Prediction horizon Np, Control horizon Nc
- Weighting matrices (Q, R, S terminal cost)
- Disturbance model types (output, input, state)
- Feedforward gain matrix (Kd)
- Constraint types (input, rate, output, state)
- QP standard form (Hessian, gradient, constraints)
- Solver status enumeration
- Controller modes (feedback-only, disturbance modes, FF)

## L2: Core Concepts (Complete)
- Receding horizon control principle
- Feedback vs. feedforward compensation
- Measured vs. unmeasured disturbances
- Offset-free MPC via disturbance augmentation
- Constraint handling in optimal control
- Cost function structure (tracking + actuation penalty)
- Separation of estimation and control

## L3: Engineering Structures (Complete)
- State-space model structure with disturbance channels
- Prediction matrix structure (Phi, Gamma, Phi_d, Psi)
- Quadratic Programming problem structure
- Observer gain matrix structure (Luenberger, Kalman)
- QP Hessian/Gradient construction from prediction matrices
- Discretization structure (ZOH, Tustin/bilinear)

## L4: Engineering Laws/Theorems (Complete)
- Internal Model Principle (Francis and Wonham 1976): disturbance integrators for offset-free
- Separation Principle: optimal control = optimal estimation + deterministic control
- KKT optimality conditions for constrained QP
- Kalman duality: observability <-> controllability of dual system
- Van Loan (1978): exact ZOH discretization via matrix exponential
- Kleinman (1968): DARE iteration convergence conditions
- Mayne et al. (2000): terminal cost + terminal set -> stability

## L5: Algorithms/Methods (Complete)
- Matrix exponential via Pade [6/6] + scaling-and-squaring (Higham 2009)
- ZOH discretization (Van Loan augmented matrix / series expansion)
- Prediction matrix construction (Phi, Gamma)
- Active-set QP solver (Goldfarb-Idnani dual method)
- Kalman filter (predict + update)
- Steady-state Kalman gain via DARE
- Static feedforward gain computation
- DARE solution via Kleinman iteration
- Step response model construction (DMC)
- Steady-state target computation

## L6: Canonical Problems (Complete)
- CSTR temperature control with feed disturbance (example_cstr_temp.c)
- Level control with inflow disturbance
- Distillation column with feed composition disturbance
- Quadcopter with wind disturbance

## L7: Industrial Applications (Partial - 2 items)
- Honeywell Experion DCS interface
- Rockwell PlantPAx interface
- Aspen DMCplus compatibility (step response model)
- ISA-5.1 compliant feedforward structure

## L8: Advanced Topics (Partial - 3 items)
- Adaptive feedforward gain (gradient descent)
- Stability margin estimation (Lyapunov analysis)
- Model-plant mismatch detection
- Monte Carlo robustness analysis
- Time-varying Kalman filter

## L9: Research Frontiers (Partial - documented)
- IT/OT convergence for MPC deployment
- Edge AI for disturbance prediction
- Autonomous MPC tuning (reinforcement learning)
