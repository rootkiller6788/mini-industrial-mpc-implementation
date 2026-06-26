# Mini MPC Ill-Conditioned Process

**Numerical conditioning analysis and stabilization for industrial Model Predictive Control.**

---

## Module Status: COMPLETE

- **L1-L6**: Complete
- **L7**: Partial+ (3 canonical applications, industrial software references)
- **L8**: Partial (Bayesian regularization, GCV, L-curve, adaptive LM)
- **L9**: Partial (documented frontiers)

---

## Line Count
| Component | Lines |
|-----------|-------|
| include/ (6 headers) | 639 |
| src/ (7 .c files) | 2422 |
| src/ (.lean) | 187 |
| **Total include+src** | **3061** |

---

## Knowledge Coverage

### L1: Definitions (Complete)
Condition number (1-norm, 2-norm, inf-norm), numerical rank, stiffness ratio, collinearity index, RGA (Bristol, 1966), regularization parameter lambda, preconditioner types, sensitivity classification, ill-conditioning threshold (1e8).

### L2: Core Concepts (Complete)
Numerical stability framework, regularization paradigm, preconditioning theory, sensitivity analysis pipeline, root cause taxonomy (7 causes), condition quality grading, diagnose-recommend-verify loop.

### L3: Engineering Structures (Complete)
`mpc_matrix_t`, `mpc_svd_t`, `mpc_illcond_model_t`, `mpc_regularization_t`, `mpc_preconditioner_t`, `mpc_illcond_diagnostic_t`, `mpc_illcond_qp_t`.

### L4: Fundamental Theorems (Complete)
- **Weyl (1912)**: |sigma_i(A+E) - sigma_i(A)| <= ||E||_2
- **Eckart-Young-Mirsky**: min_{rank=k} ||A-B||_2 = sigma_{k+1}
- **Gershgorin (1931)**: Eigenvalue localization in discs
- **Bristol (1966)**: RGA = G .* (G^{-1})^T
- **Wilkinson (1963)**: ||dx||/||x|| <= c * kappa * eps
- **Morozov (1966)**: lambda_opt = sigma_min^2
- **Sylvester**: SPD iff leading principal minors > 0

All theorems have both C code verification (tests/) and formal Lean 4 statements (src/mpc_illcond_theory.lean).

### L5: Algorithms (Complete)
- Hager-Higham 1-norm condition estimator
- Gershgorin circle condition bound
- Trace-ratio eigenvalue bound (Wolkowicz & Styan, 1980)
- Tikhonov regularization (Phillips 1962, Tikhonov 1963)
- Truncated SVD regularization (Hansen 1998)
- Levenberg-Marquardt adaptive regularization
- Elastic net (Zou & Hastie 2005)
- Jacobi, SSOR, ILU(0) preconditioners
- Polynomial (Neumann series) preconditioner
- Block Jacobi for structured MIMO
- Additive Schwarz domain decomposition
- Preconditioned Conjugate Gradient (PCG)
- K-fold CV and GCV for lambda selection
- L-curve criterion (Hansen 1992)

### L6: Canonical Problems (Complete)
1. **High-purity distillation column** (kappa > 1e6, near-collinear gains)
2. **Stiff CSTR reactor** (stiffness ratio > 100, multi-rate dynamics)
3. **Collinear sensor arrays** (measurement redundancy, sensor selection)

### L7: Industrial Applications (Partial+)
- Distillation column MPC (Honeywell RMPCT, AspenTech DMCplus)
- Chemical reactor temperature/composition control
- Operator alarm thresholds for conditioning
- Condition monitoring in MPC commissioning

### L8: Advanced Topics (Partial)
- Bayesian regularization (MAP estimation interpretation)
- Generalized Cross-Validation (GCV)
- L-curve optimal lambda selection
- Relative condition number for model mismatch
- Adaptive Levenberg-Marquardt

### L9: Research Frontiers (Partial)
- ML for automatic lambda selection (documented)
- Randomized SVD (documented)
- Quantum-inspired condition estimation (documented)

---

## Core APIs

### Condition Estimation
```c
double mpc_condition_estimate(const mpc_matrix_t *A, mpc_illcond_estimation_method_t method);
int mpc_condition_is_illcond(double cond);
const char* mpc_condition_grade(double cond);
double mpc_condition_digits_lost(double cond);
int mpc_condition_diagnose(const mpc_matrix_t *A, mpc_illcond_diagnostic_t *diag);
```

### Regularization
```c
void mpc_regularize_tikhonov(mpc_matrix_t *A, double lambda);
int mpc_regularize_truncated_svd(const mpc_svd_t *svd, mpc_matrix_t *A_reg, double threshold);
double mpc_regularize_levenberg_marquardt(...);
void mpc_regularize_elastic_net(mpc_matrix_t *A_reg, double lambda, double alpha, int max_iter, double tol);
int mpc_regularize_qp(mpc_illcond_qp_t *qp, const mpc_regularization_t *config);
```

### Preconditioning
```c
int mpc_precond_build(const mpc_matrix_t *A, mpc_preconditioner_t *prec);
void mpc_precond_apply(const mpc_preconditioner_t *prec, const double *r, double *z);
int mpc_pcg_solve(const mpc_matrix_t *A, const double *b, double *x, const mpc_preconditioner_t *prec, int max_iter, double tol);
```

### Sensitivity Analysis
```c
int mpc_sensitivity_rga(const mpc_matrix_t *G, mpc_matrix_t *RGA);
double mpc_sensitivity_collinearity(const mpc_matrix_t *G, size_t *worst_i, size_t *worst_j);
int mpc_sensitivity_analyze(const mpc_illcond_model_t *model, mpc_illcond_diagnostic_t *diag);
```

---

## Building and Running

```bash
make          # build library and run tests
make test     # run tests only
make examples # build and run examples
make clean    # remove build artifacts
```

---

## Course Mapping

| University | Course | Coverage |
|------------|--------|----------|
| MIT | 6.302 Feedback Systems | Condition number, SVD robustness |
| Stanford | ENGR205 Process Control | Ill-conditioned MPC |
| Berkeley | ME233 Advanced Control | Numerical methods |
| CMU | 24-677 Adv Ctrl Systems | Preconditioning, large-scale |
| Purdue | ME575 Industrial Control | RGA, pairing, industrial MPC |
| RWTH Aachen | Industrial Control | Condition monitoring |
| Tsinghua | Process Control Eng. | Distillation, reactor MPC |
| ISA/IEC | ISA-101, IEC 61131-3 | HMI alarms, monitoring blocks |

---

## References

- Golub & Van Loan (2013) "Matrix Computations", 4th ed., Johns Hopkins
- Higham (2002) "Accuracy and Stability of Numerical Algorithms", 2nd ed., SIAM
- Rawlings, Mayne, Diehl (2017) "Model Predictive Control", 2nd ed., Nob Hill
- Skogestad & Postlethwaite (2005) "Multivariable Feedback Control", Wiley
- Saad (2003) "Iterative Methods for Sparse Linear Systems", 2nd ed., SIAM
- Hansen (1998) "Rank-Deficient and Discrete Ill-Posed Problems", SIAM
- Hager (1984) "Condition Estimates", SIAM J. Sci. Stat. Comput. 5(2)
- Bristol (1966) "On a New Measure of Interaction", IEEE TAC 11(1)
