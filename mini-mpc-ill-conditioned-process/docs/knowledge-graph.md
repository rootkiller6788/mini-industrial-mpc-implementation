# Knowledge Graph — Mini MPC Ill-Conditioned Process

## L1: Definitions (Complete)
- Condition number: kappa(A) = ||A|| * ||A^{-1}|| (2-norm, 1-norm, inf-norm)
- Numerical rank: count of singular values > eps * sigma_max
- Stiffness ratio: tau_max / tau_min for multi-rate systems
- Collinearity index: max |cos(theta)| between gain column vectors
- Relative Gain Array (RGA): G .* (G^{-1})^T (Bristol, 1966)
- Regularization parameter: lambda in Tikhonov A_reg = A + lambda*I
- Preconditioner: M ~ A such that M^{-1}A has clustered eigenvalues
- Ill-conditioning threshold: kappa > 1e8 for double precision
- Sensitivity classes: LOW, MODERATE, HIGH, EXTREME

## L2: Core Concepts (Complete)
- Numerical stability and conditioning in MPC optimization
- Regularization framework: L2 (Tikhonov), L1+L2 (elastic net), truncated SVD
- Preconditioning: transform to improve iterative solver convergence
- Sensitivity analysis: RGA, collinearity, minimum gain direction
- Root cause taxonomy: 7 types of ill-conditioning causes
- Condition quality grades: WELL, MODERATE, POOR, ILL
- Diagnose-recommend loop: detect -> classify -> regularize -> verify

## L3: Engineering Structures (Complete)
- mpc_matrix_t: dense row-major matrix with stride
- mpc_svd_t: SVD decomposition (U, S, V) with rank and condition
- mpc_illcond_model_t: process model with full conditioning diagnostics
- mpc_regularization_t: regularization configuration
- mpc_preconditioner_t: preconditioner matrix and metadata
- mpc_illcond_diagnostic_t: comprehensive diagnostic report
- mpc_illcond_qp_t: MPC QP with embedded conditioning

## L4: Fundamental Theorems (Complete)
- Weyl's perturbation bound: |sigma_i(A+E) - sigma_i(A)| <= ||E||_2 (1912)
- Eckart-Young-Mirsky: min_{rank(B)=k} ||A-B||_2 = sigma_{k+1}
- Gershgorin circle theorem: eigenvalue localization (1931)
- Bristol's RGA theorem: steady-state interaction quantification (1966)
- Sylvester's criterion: SPD iff leading principal minors > 0
- Wilkinson's error bound: ||dx||/||x|| <= c * kappa * eps (1963)
- Morozov discrepancy principle: optimal lambda = sigma_min^2 (1966)
- Condition number sensitivity: d(kappa)/d(A_ij) via singular vectors

## L5: Algorithms/Methods (Complete)
- Hager-Higham 1-norm condition estimator (1984/1988)
- Gershgorin circle condition bound (O(n^2))
- Trace-ratio eigenvalue bound (Wolkowicz & Styan, 1980)
- Tikhonov regularization (A + lambda*I)
- Truncated SVD regularization (Hansen, 1998)
- Levenberg-Marquardt adaptive regularization
- Elastic net (Zou & Hastie, 2005)
- Jacobi, SSOR, ILU(0) preconditioners
- Polynomial (Neumann series) preconditioner
- Block Jacobi for structured MIMO
- Additive Schwarz domain decomposition
- Preconditioned Conjugate Gradient (PCG)
- Coordinate descent for elastic net
- Cross-validation (K-fold + GCV) for lambda selection
- L-curve criterion (Hansen, 1992)

## L6: Canonical Problems (Complete)
- High-purity distillation column (kappa > 1e6)
- Stiff CSTR reactor (stiffness ratio > 1e5)
- Collinear sensor arrays (measurement redundancy)
- Near-singular gain matrices (zero-gain direction)
- Dynamic matrix condition growth with prediction horizon
- Effective condition number for MPC QP assessment

## L7: Industrial Applications (Partial+)
- Distillation column control (Honeywell RMPCT, AspenTech DMCplus)
- Chemical reactor temperature/composition MPC
- Refinery crude unit multivariable control
- Condition monitoring in MPC commissioning
- Operator alarm thresholds for conditioning degradation

## L8: Advanced Topics (Partial)
- Bayesian regularization (MAP interpretation of Tikhonov)
- Generalized Cross-Validation (GCV) for lambda
- L-curve optimal lambda selection
- Relative condition number for model mismatch
- Adaptive regularization for time-varying processes
- Domain decomposition preconditioners

## L9: Research Frontiers (Partial)
- Machine learning for automatic lambda selection
- Randomized SVD for large-scale MPC (Halko et al., 2011)
- Stochastic preconditioners for uncertain systems
- Quantum-inspired condition number estimation
