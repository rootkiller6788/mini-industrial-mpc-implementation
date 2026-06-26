# Course Tree — mini-mpc-operator-training-interface

## Prerequisite Dependencies

```
mini-mpc-operator-training-interface
├── Requires:
│   ├── mini-mpc-implementation (parent module — MPC fundamentals)
│   ├── mini-pid-control-engineering (control basics)
│   ├── mini-industrial-measurement-actuator (process variables)
│   ├── mini-scada-hmi-engineering (ISA-101 HMI principles)
│   └── mini-advanced-process-control-apc (APC concepts)
│
├── Knowledge Tree (topological order):
│   L1: OTS state machine, scenarios, roles, metrics
│   ├── L2: Operator-in-the-loop, scenario-based training, guidance
│   │   ├── L3: State machine, event timeline, priority queue, ring buffer
│   │   │   ├── L4: ISA-101, EEMUA 201, Kirkpatrick, Elo, NASA-TLX
│   │   │   │   ├── L5: Weighted scoring, OLS, EWMA, plateau detection
│   │   │   │   │   ├── L6: Reactor, Column, What-if, Boiler, Emergency
│   │   │   │   │   │   ├── L7: Honeywell, AspenTech, Siemens, ABB, Yokogawa
│   │   │   │   │   │   │   ├── L8: BKT, Adaptive learning, VR/AR
│   │   │   │   │   │   │   │   └── L9: Autonomous training, Cloud OTS
│
└── Dependent modules:
    └── mini-mes-digital-factory (operator training data → MES)
    └── mini-industrial-ai-control-fusion (AI-based operator assistance)
```

## Learning Path

1. **Beginner**: Understand OTS state machine and basic session lifecycle
2. **Intermediate**: Multi-variable scenario training with constraint trade-offs
3. **Advanced**: Industrial vendor integration, workload management
4. **Expert**: Adaptive difficulty, Bayesian knowledge tracing, digital twin OTS
