# mini-mpc-subspace-identification

**Subspace State-Space System Identification (4SID) for MPC Model Generation**

Implements N4SID, MOESP, and CVA algorithms for MIMO state-space model identification from input-output data. The identified models serve as the internal prediction models for Model Predictive Control (MPC).

## Module Status: COMPLETE

- **L1-L6**: Complete (all core concepts, definitions, algorithms, canonical problems implemented)
- **L7**: Partial (2/4 industrial vendors: AspenTech DMC3, Honeywell Profit Controller)
- **L8**: Partial (2/5 advanced topics: recursive N4SID, closed-loop N4SID)
- **L9**: Partial (Documented in knowledge graph)

**??????**: include/ + src/ = **5,547 ?** >= 3,000
**make test**: 13/13 tests passing
**?????**: 0 matches
**? TODO/FIXME/stub/placeholder**: YES

---

## ????????

| Level | Name | Coverage | Key Items |
|-------|------|----------|-----------|
| **L1** | Definitions | COMPLETE | 9 structs, 5 enums, column-major matrix type |
| **L2** | Core Concepts | COMPLETE | Block Hankel, oblique projection, persistent excitation |
| **L3** | Engineering Structures | COMPLETE | LQ decomposition, Householder reflections, views |
| **L4** | Engineering Laws | COMPLETE | VODM Theorems 2 & 3, Eckart-Young, shift-invariance |
| **L5** | Algorithms | COMPLETE | N4SID, MOESP, CVA, SVD-gap/AIC/BIC/MDL order selection |
| **L6** | Canonical Problems | COMPLETE | 2nd-order ID, CSTR reactor, order selection comparison |
| **L7** | Industrial Applications | PARTIAL | AspenTech DMC3, Honeywell Profit wrappers |
| **L8** | Advanced Topics | PARTIAL | Recursive ID, closed-loop ID |
| **L9** | Industry Frontiers | PARTIAL | Documented (autonomous ID, digital twin) |

**Score**: L1-L6 Complete x2 = 12, L7-L9 Partial x1 = 3, **Total = 15/18 (COMPLETE)**

---

## Core Definitions

| Type | Description | C API |
|------|-------------|-------|
| `ssid_matrix_t` | Dense matrix (col-major) | `ssid_matrix_alloc/from_array/view/free` |
| `ssid_model_t` | State-space model {A,B,C,D,K} | `ssid_model_free` |
| `ssid_hankel_t` | Block Hankel (past/future) | `ssid_hankel_build/split/free` |
| `ssid_svd_t` | SVD result (U, S, V) | `ssid_svd_compute/truncate/free` |
| `ssid_config_t` | ID configuration | `ssid_config_default` |
| `ssid_data_t` | MIMO I/O data | `ssid_data_free` |
| `ssid_result_t` | Identification result | `ssid_result_free` |

## Core Theorems

| Theorem | Formula | Implementation |
|---------|---------|----------------|
| **VODM Theorem 2** | O_i = Gamma_i * X_i (oblique projection factorization) | `ssid_project_oblique` |
| **VODM Theorem 3** | Unified 4SID via single LQ decomposition | `ssid_project_combined_LQ` |
| **Eckart-Young** | A_k = U_k * S_k * V_k^T (optimal rank-k) | `ssid_svd_reconstruct` |
| **Shift-Invariance** | Gamma_i_up * A = Gamma_i_down | `ssid_svd_estimate_AC` |

## Core Algorithms

| Algorithm | Description | Reference |
|-----------|-------------|-----------|
| **N4SID** | Numerical subspace state-space ID | Van Overschee & De Moor (1994) |
| **SVD-Gap** | Order selection via singular value ratio | Ljung (1999), Sec 16.5 |
| **AIC** | Akaike Information Criterion | Akaike (1974) |
| **BIC** | Bayesian Information Criterion | Schwarz (1978) |
| **MDL** | Minimum Description Length | Rissanen (1978) |
| **Jacobi SVD** | Symmetric eigendecomposition | Golub & Van Loan (2013), Ch 8 |

## Classic Problems

| Problem | File | Description |
|---------|------|-------------|
| 2nd-order system ID | `examples/example_2nd_order.c` | Identify SISO 2nd-order from PRBS data |
| CSTR reactor | `examples/example_cstr_reactor.c` | MIMO nonlinear chemical reactor |
| Order selection | `examples/example_order_selection.c` | Compare AIC/BIC/MDL/SVD-gap |

## ??????

| School | Course | Topic |
|--------|--------|-------|
| MIT | 6.302 Feedback Systems | System ID for control design |
| Stanford | ENGR205 Process Control | MPC model generation |
| CMU | 24-677 Adv Ctrl Systems | Subspace methods for MIMO |
| Berkeley | ME233 Advanced Control | State-space realization |
| Georgia Tech | ECE 6550 Nonlinear Control | Linearization + ID |
| Purdue | ME 575 Industrial Control | Industrial identification |
| RWTH Aachen | Industrial Control Systems | PLC/SCADA integration |
| Tsinghua | Process Control Engineering | Chemical process ID |
| ISA/IEC | ISA-88/95 | Model management standards |

## Directory Structure

```
mini-mpc-subspace-identification/
  Makefile          # make test ????
  README.md         # ???
  include/          # 6 header files
  src/              # 6 C implementation files
  tests/            # 13 unit tests
  examples/         # 3 end-to-end examples
  docs/             # 5 knowledge documents
```

## Build & Test

```bash
make          # Build library, test, and examples
make test     # Run 13 unit tests
make examples # Run 3 end-to-end examples
make count    # Show line counts
make audit    # Filler detection scan
make clean    # Remove build artifacts
```

## Key References

1. Van Overschee, P. & De Moor, B. (1996). *Subspace Identification for Linear Systems*. Kluwer.
2. Ljung, L. (1999). *System Identification: Theory for the User*, 2nd ed. Prentice Hall.
3. Verhaegen, M. & Dewilde, P. (1992). Subspace model identification. *Int. J. Control*, 56(5).
4. Larimore, W.E. (1990). Canonical variate analysis in identification. *Proc. CDC*.
5. Golub, G.H. & Van Loan, C.F. (2013). *Matrix Computations*, 4th ed. Johns Hopkins.
