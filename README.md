# Mini Industrial MPC Implementation

A collection of **from-scratch, zero-dependency C implementations** of industrial Model Predictive Control (MPC) algorithms and engineering practice. Each module maps to MIT and Stanford graduate-level courses, bridging control theory and industrial deployment—from subspace identification and constraint handling to AspenTech DMC3 integration and operator training interfaces.

## Sub-Modules

| Sub-Module | Topics | Key Courses |
|------------|--------|-------------|
| [mini-mpc-constraint-handling-priority](mini-mpc-constraint-handling-priority/) | Constraint prioritization, feasibility analysis, relaxation strategies, QP optimization | MIT 6.251J, Stanford AA203 |
| [mini-mpc-disturbance-feedforward](mini-mpc-disturbance-feedforward/) | Measured/unmeasured disturbance feedforward, Kalman observer, disturbance modeling | MIT 6.241J, MIT 10.492 |
| [mini-mpc-ill-conditioned-process](mini-mpc-ill-conditioned-process/) | Condition number estimation, SVD (Golub-Reinsch), preconditioning, Tikhonov regularization, RGA sensitivity | MIT 18.065, MIT 10.492 |
| [mini-mpc-industrial-software-aspentech](mini-mpc-industrial-software-aspentech/) | AspenTech DMC3 interface, NMPC, robust MPC, adaptive MPC, moving horizon estimation (MHE) | MIT 10.492, Stanford AA203 |
| [mini-mpc-integrating-process-level](mini-mpc-integrating-process-level/) | DMC, GPC (CARIMA), integrating process models, level constraints, surge tank dynamics | MIT 10.492, MIT 10.450 |
| [mini-mpc-operator-training-interface](mini-mpc-operator-training-interface/) | Operator Training Simulator (OTS), performance assessment, scenario generation, operator guidance | MIT 16.842, MIT 2.154 |
| [mini-mpc-performance-monitoring-kpi](mini-mpc-performance-monitoring-kpi/) | Harris performance index, minimum variance benchmark, KPI monitoring, root cause diagnosis | MIT 10.492, Stanford EE364A |
| [mini-mpc-subspace-identification](mini-mpc-subspace-identification/) | N4SID, MOESP, CVA, block Hankel matrices, oblique projections, model validation | Stanford EE364B, MIT 6.241J |

## Design Philosophy

- **Zero external dependencies** — pure C (C99/C11), only `libc` and `libm`
- **Self-contained modules** — each directory has its own `Makefile`, `include/`, `src/`, `examples/`, `tests/`, `docs/`
- **Industry-to-code mapping** — every module includes `docs/` with industrial standard references (ISA-88, ISA-95, IEC 61511, ISA-18.2)
- **Practical demos** — DMC controller, GPC controller, AspenTech DMC3 interface, N4SID identification, OTS scenarios, KPI dashboards

## Building

Each module is standalone. Navigate to a module directory and run:

```bash
cd mini-mpc-constraint-handling-priority
make all    # build everything
make test   # run tests
```

Requires **GCC** and **GNU Make**.

## Project Structure

```
mini-industrial-mpc-implementation/
├── mini-mpc-constraint-handling-priority/  # Constraint hierarchy, feasibility, relaxation
├── mini-mpc-disturbance-feedforward/       # MPC with measured/unmeasured disturbance compensation
├── mini-mpc-ill-conditioned-process/       # SVD, preconditioning, Tikhonov, RGA for ill-conditioned systems
├── mini-mpc-industrial-software-aspentech/ # AspenTech DMC3 interface, NMPC, robust/adaptive MPC, MHE
├── mini-mpc-integrating-process-level/     # DMC, GPC, CARIMA for integrating (non-self-regulating) processes
├── mini-mpc-operator-training-interface/   # Operator Training Simulator with assessment & guidance
├── mini-mpc-performance-monitoring-kpi/    # Harris index, minimum variance benchmark, KPI diagnostics
├── mini-mpc-subspace-identification/       # N4SID, MOESP, CVA — data-driven state-space identification
├── .gitignore                              # Build artifact and IDE ignore rules
├── README.md                               # This file (English)
└── README-CN.md                            # Chinese version of this file
```

## License

MIT
