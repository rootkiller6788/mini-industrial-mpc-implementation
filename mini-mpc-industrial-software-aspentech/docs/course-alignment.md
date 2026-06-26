# Course Alignment — mini-mpc-industrial-software-aspentech

## Nine-School Curriculum Mapping for Industrial MPC

---

### MIT — 6.302 Feedback Systems / 2.171 Digital Control
| Chapter | Topic | Module Coverage |
|---------|-------|----------------|
| Ch.7 | State-space design | State-space MPC in state_space_mpc.c |
| Ch.8 | Optimal control (LQR) | QP cost as finite-horizon LQR |
| Ch.9 | Estimator design (LQE/KF) | Kalman filter in mpc_adaptation.c |
| Ch.10 | Constrained control | QP constraints in qp_solver.c, mpc_controller.c |
| Ch.12 | Receding horizon | Core DMC cycle in mpc_controller.c |

### Stanford — ENGR205 Process Control / EE392 Industrial AI
| Chapter | Topic | Module Coverage |
|---------|-------|----------------|
| Ch.6 | Model Predictive Control | Complete DMC implementation |
| Ch.7 | Industrial MPC architecture | AspenTech DMC3 5-layer architecture |
| Ch.10 | System identification | N4SID in model_identification.c |
| Ch.12 | Adaptive control | RLS in mpc_adaptation.c |

### Berkeley — ME233 Advanced Control
| Chapter | Topic | Module Coverage |
|---------|-------|----------------|
| Ch.5 | Optimal control | QP formulation mpc_qp_problem_t |
| Ch.7 | Constrained optimization | Active-set QP in qp_solver.c |
| Ch.8 | Kalman filtering | Full KF cycle in mpc_adaptation.c |
| Ch.10 | Robust control | Scenario-based robust MPC in mpc_advanced.c |
| Ch.12 | Model identification | Subspace ID in model_identification.c |

### CMU — 24-677 Advanced Control Systems
| Chapter | Topic | Module Coverage |
|---------|-------|----------------|
| Ch.3 | Linear systems theory | State-space in state_space_mpc.c |
| Ch.4 | Controllability/Observability | Gramians in state_space_mpc.c |
| Ch.7 | MIMO identification | N4SID MIMO in model_identification.c |
| Ch.9 | MPC theory | DMC + QP in mpc_controller.c, qp_solver.c |
| Ch.12 | Nonlinear MPC | NMPC linearization in mpc_advanced.c |

### Georgia Tech — ECE 6550 Nonlinear Control / AE 6530 Optimal Estimation
| Chapter | Topic | Module Coverage |
|---------|-------|----------------|
| Ch.6 | Nonlinear system linearization | NMPC finite-difference in mpc_advanced.c |
| Ch.8 | Optimal state estimation | Kalman filter + MHE |
| Ch.10 | Receding horizon nonlinear | NMPC successive linearization |

### Purdue — ME 575 Industrial Control
| Chapter | Topic | Module Coverage |
|---------|-------|----------------|
| Ch.5 | DMC algorithm | Full DMC in dmc_model.c, mpc_controller.c |
| Ch.6 | Process identification | Step tests, FOPDT fitting |
| Ch.8 | LP optimization | Steady-state LP in ss_target.c |
| Ch.10 | Industrial practice | AspenTech DMC3 simulation in aspen_dmc3.c |
| Ch.14 | Performance assessment | Harris index in mpc_diagnostics.c |

### RWTH Aachen — Industrial Control Systems
| Chapter | Topic | Module Coverage |
|---------|-------|----------------|
| Ch.4 | PLC/SCADA Engineering | DCS interface simulation |
| Ch.7 | Advanced Process Control | AspenTech DMC3 in refinery |
| Ch.9 | Model-based control | State-space and FIR models |
| Ch.11 | Plant data analytics | Model Quality Monitor |

### Tsinghua — Process Control Engineering (过程控制工程)
| Chapter | Topic | Module Coverage |
|---------|-------|----------------|
| Ch.5 | Advanced control systems | MPC theory and practice |
| Ch.8 | Distillation column control | Wood-Berry 2x2 example |
| Ch.10 | Refinery process control | CDU furnace control example |
| Ch.14 | Industrial IT systems | DMC3 Builder configuration |

### ISA/IEC Standards — ISA-88/95/101, IEC 61131-3/61508/61511
| Standard | Topic | Module Coverage |
|----------|-------|----------------|
| ISA-101 | HMI design | EU scaling for operator display |
| ISA-95 | Enterprise-control integration | DCS interface layer |
| IEC 61131-3 | PLC programming | MV/CV configuration structs |

---

## Coverage Summary

| School | Key Course | Coverage Quality |
|--------|-----------|-----------------|
| MIT | 6.302, 2.171 | Deep — optimal control, estimation, constraints |
| Stanford | ENGR205, EE392 | Deep — industrial MPC architecture, ID |
| Berkeley | ME233 | Deep — constrained QP, Kalman, robust |
| CMU | 24-677 | Deep — SS theory, MIMO ID, NMPC |
| Georgia Tech | ECE 6550, AE 6530 | Good — nonlinear, estimation |
| Purdue | ME 575 | Deep — DMC, industrial practice |
| RWTH Aachen | ICS | Good — refinery, data analytics |
| Tsinghua | Process Control Eng | Good — distillation, CDU |
| ISA/IEC | 88/95/101/61131 | Good — EU scaling, DCS interface |
