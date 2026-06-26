# Knowledge Graph: Subspace Identification for MPC

## L1: Definitions
- ssid_matrix_t: Column-major dense matrix
- ssid_model_t: State-space model {A,B,C,D,K}
- ssid_hankel_t: Block Hankel matrix (past/future)
- ssid_svd_t: SVD result (U,S,V)
- ssid_lq_t: LQ decomposition
- ssid_data_t: MIMO I/O data record
- ssid_result_t: Identification result bundle
- ssid_config_t: Identification configuration
- ssid_dim_t: System dimensions

## L2: Core Concepts
- Block Hankel matrix: past/future data decomposition
- Oblique projection: Y_f /_{U_f} W_p
- Persistent excitation: necessary identifiability condition
- Innovation form: combined deterministic-stochastic model
- Kalman filter state sequence recovery

## L3: Engineering Structures
- LQ decomposition (square-root algorithms)
- Column-major storage (BLAS/LAPACK compatibility)
- Householder reflections for orthogonalization
- Jacobi eigenvalue algorithm
- Matrix view vs. ownership model

## L4: Engineering Laws/Theorems
- Van Overschee & De Moor Theorem 2: Oblique projection factorizes as observability * state
- Van Overschee & De Moor Theorem 3: Unified 4SID framework
- Eckart-Young theorem: Optimal low-rank approximation via SVD
- Shift-invariance property: Gamma_i_up * A = Gamma_i_down
- KKT conditions in weighted subspace methods

## L5: Algorithms/Methods
- N4SID: Full identification pipeline
- MOESP: Instrumental variable subspace method
- CVA: Canonical variate analysis
- SVD-gap order selection
- AIC/BIC/MDL order selection
- Jacobi SVD algorithm
- Recursive N4SID (sliding window)
- Closed-loop N4SID (instrumental variables)

## L6: Canonical Problems
- 2nd-order system identification
- CSTR reactor temperature (MIMO)
- Order selection with multiple criteria

## L7: Industrial Applications
- AspenTech DMC3 identification wrapper
- Honeywell Profit Controller wrapper
- Industrial MQA validation report
- MPC readiness assessment

## L8: Advanced Topics
- Recursive subspace identification
- Closed-loop subspace identification
- Weighting matrix condition analysis

## L9: Research Frontiers
- Autonomous system identification
- Digital twin calibration
- IT/OT convergence for model management
