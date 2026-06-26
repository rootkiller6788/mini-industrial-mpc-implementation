# Course Alignment — mini-mpc-constraint-handling-priority

## Nine-School Curriculum Mapping

| School | Course | Relevant Chapters | This Module's Coverage |
|--------|--------|-------------------|----------------------|
| **MIT** | 6.302 Feedback Systems | Ch.7: Constraints in Control | Active-set QP, KKT verification |
| **Stanford** | ENGR205 Process Control | Ch.8: MPC Implementation | Lexicographic MPC, priority ordering |
| **Berkeley** | ME233 Advanced Control | Module 5: Constrained Optimization | QP solver, constraint sensitivity |
| **CMU** | 24-677 Adv Ctrl Systems | Module 6: Industrial MPC | AspenTech DMC3, Honeywell RMPCT |
| **Georgia Tech** | ECE 6550 Nonlinear Control | Ch.9: Constraint Handling | Farkas' lemma, feasibility recovery |
| **Purdue** | ME 575 Industrial Control | Ch.7: Process Constraints | Input saturation, output prioritization |
| **RWTH Aachen** | Industrial Control Systems | PLC/SCADA Constraints | Vendor-specific constraint policies |
| **清华** | 过程控制工程 | 第6章: 约束预测控制 | Priority-based relaxation |
| **ISA/IEC** | ISA-88/95, IEC 61131-3 | Constraint Management | Hard/soft classification, relaxation policies |

## Key Reference Textbooks

| Textbook | Chapters | Topics Covered |
|----------|---------|---------------|
| Rawlings, Mayne & Diehl (2017) | Ch.5-6 | Constrained optimal control, feasibility, stability |
| Nocedal & Wright (2006) | Ch.16 | Quadratic programming, active-set methods |
| Maciejowski (2002) | Ch.6-7 | Constraint handling in predictive control |
| Camacho & Bordons (2007) | Ch.7 | Constrained MPC implementation |
| Qin & Badgwell (2003) | Sec.5 | Industrial MPC constraint handling survey |
| Bemporad et al. (2002) | — | Explicit MPC, critical regions |
| Chinneck (2008) | — | Feasibility and infeasibility in optimization |
| Fletcher (1987) | Ch.10 | Exact penalty methods |

## Key Papers

1. Bemporad, A., Morari, M., Dua, V., & Pistikopoulos, E. N. (2002). The explicit linear quadratic regulator for constrained systems. *Automatica*, 38(1), 3-20.
2. Qin, S. J., & Badgwell, T. A. (2003). A survey of industrial model predictive control technology. *Control Engineering Practice*, 11(7), 733-764.
3. Chinneck, J. W., & Dravnieks, E. W. (1991). Locating minimal infeasible constraint sets in linear programs. *ORSA Journal on Computing*, 3(2), 120-130.
4. de Oliveira, N. M. C., & Biegler, L. T. (1994). Constraint handling and stability properties of model-predictive control. *Automatica*, 30(8), 1303-1313.
5. Kerrigan, E. C., & Maciejowski, J. M. (2002). Designing model predictive controllers with prioritised constraints and objectives. *IEEE Int. Symp. CACSD*.
