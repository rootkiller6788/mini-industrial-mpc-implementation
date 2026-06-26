# Course Dependency Tree — Mini MPC Ill-Conditioned Process

## Prerequisites
```
Linear Algebra (matrix operations, eigenvalues, SVD)
  |
Numerical Analysis (floating-point errors, condition numbers)
  |
Process Control Fundamentals (PID, MIMO, MPC basics)
  |
  +--> mini-mpc-ill-conditioned-process (this module)
         |
         +--> Condition Number Estimation
         |      |
         |      +--> Hager-Higham estimator
         |      +--> Gershgorin bounds
         |      +--> Trace-ratio estimate
         |
         +--> Sensitivity Analysis
         |      |
         |      +--> RGA computation
         |      +--> Collinearity detection
         |      +--> Stiffness ratio
         |      +--> Minimum gain direction
         |
         +--> Regularization
         |      |
         |      +--> Tikhonov (L2)
         |      +--> Truncated SVD
         |      +--> Levenberg-Marquardt
         |      +--> Elastic Net (L1+L2)
         |      +--> Cross-validation / GCV
         |
         +--> Preconditioning
         |      |
         |      +--> Jacobi / SSOR / ILU(0)
         |      +--> Polynomial
         |      +--> Block Jacobi
         |      +--> PCG solver
         |
         +--> SVD Computation
                |
                +--> Golub-Reinsch bidiagonalization
                +--> Householder reflections
                +--> Numerical rank / nullspace
```

## Depends On
- mini-dmc-dynamic-matrix-control (MPC QP structure)
- mini-multi-variable-process-id (model identification)
- mini-apc-model-identification-step-test (step response data)

## Feed Into
- mini-mpc-constraint-handling-priority (ill-conditioned constraints)
- mini-mpc-performance-monitoring-kpi (condition monitoring)
- mini-apc-economic-objective-function (regularized economic MPC)
