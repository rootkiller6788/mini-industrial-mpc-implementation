#ifndef SSID_VALIDATION_H
#define SSID_VALIDATION_H

#include "ssid_defs.h"
#include "ssid_svd.h"

/* ============================================================================
 * ssid_validation.h -- Model Validation and Diagnostic Tools
 *
 * Reference: Ljung (1999), Chapter 16 (Model Validation)
 *            Van Overschee & De Moor (1996), Chapter 5
 *
 * Model validation in system identification answers the question:
 * "Is the identified model good enough for its intended purpose?"
 *
 * For MPC applications, this means:
 *   - Prediction accuracy over the control horizon
 *   - Residual whiteness (no unmodeled dynamics)
 *   - Stability of the identified model
 *   - Cross-validation on independent data
 *
 * Each function implements a distinct validation criterion.
 * ============================================================================ */

/* ---------------------------------------------------------------------------
 * L4: Residual Analysis
 * ---------------------------------------------------------------------------
 */

/* L4: Compute prediction residuals (one-step-ahead prediction errors).
 *
 * epsilon_k = y_k - y_k_hat  where y_k_hat is the optimal one-step
 * predictor from the identified Kalman filter model.
 *
 * For an innovation model: y_k_hat = C*x_k_hat + D*u_k
 *       x_{k+1}_hat = A*x_k_hat + B*u_k + K*(y_k - C*x_k_hat - D*u_k)
 *
 * Knowledge point: Residual analysis is the primary model validation
 *   tool. If the model captures all dynamics, residuals should be
 *   white noise uncorrelated with inputs. Autocorrelation and
 *   cross-correlation tests formalize this.
 *
 * Complexity: O(N * n_x^2). */
int ssid_validation_residuals(const ssid_model_t *model,
                              const ssid_data_t *data,
                              ssid_matrix_t *residuals);

/* L4: Whiteness test: compute autocorrelation of residuals.
 *
 * r_ee(tau) = (1/N) * sum_{k=0}^{N-tau-1} epsilon_k * epsilon_{k+tau}^T
 *
 * Under the null hypothesis of whiteness, |r_ee(tau)| should be less
 * than 1.96/sqrt(N) with 95% confidence.
 *
 * Knowledge point: The autocorrelation test checks if residuals
 *   contain unmodeled dynamics. Significant autocorrelation at
 *   certain lags indicates missing poles in the model.
 *
 * Complexity: O(N * M * n_y^2) where M is max_lag. */
int ssid_validation_whiteness(const ssid_matrix_t *residuals,
                              size_t max_lag,
                              double *autocorr_max,
                              int *is_white);

/* L4: Cross-correlation test between residuals and inputs.
 *
 * r_eu(tau) = (1/N) * sum epsilon_k * u_{k-tau}^T
 *
 * Under correct model structure, residuals should be uncorrelated
 * with past inputs. Significant correlation indicates missing
 * feedforward paths or feedback in residuals.
 *
 * Knowledge point: The cross-correlation test detects missing
 *   input channels or structural errors in the model. For MPC,
 *   this is critical because correlated residuals mean the
 *   optimizer will make biased decisions.
 *
 * Complexity: O(N * M * n_y * n_u). */
int ssid_validation_cross_correlation(const ssid_matrix_t *residuals,
                                      const ssid_matrix_t *U,
                                      size_t max_lag,
                                      double *crosscorr_max);

/* ---------------------------------------------------------------------------
 * L4: Prediction Accuracy
 * ---------------------------------------------------------------------------
 */

/* L4: Compute NRMSE fit metric (Normalized Root Mean Square Error).
 *
 * fit = 100 * (1 - ||y - y_hat|| / ||y - mean(y)||)
 *
 * This is the standard metric used in MATLAB System Identification
 * Toolbox's "compare" function. Values above 90% indicate excellent
 * fit; below 50% indicate poor model.
 *
 * Knowledge point: NRMSE is the de facto standard for reporting
 *   model quality in both academic literature and industrial
 *   practice. It normalizes by output variance so it is comparable
 *   across different signals.
 *
 * Complexity: O(N * n_y). */
double ssid_validation_nrmse(const ssid_matrix_t *Y_measured,
                             const ssid_matrix_t *Y_predicted);

/* L4: Compute k-step-ahead prediction accuracy.
 *
 * Unlike one-step-ahead prediction, k-step prediction is more
 * challenging and more relevant for MPC (which predicts over
 * the control horizon H_p).
 *
 * Knowledge point: Multi-step prediction accuracy degrades with
 *   horizon for models with unmodeled dynamics. The decay rate
 *   indicates the model's useful prediction horizon.
 *
 * Complexity: O(N * H_p * n_x^2). */
int ssid_validation_multistep_fit(const ssid_model_t *model,
                                  const ssid_data_t *data,
                                  size_t H_p,
                                  double *fit_vs_horizon);

/* ---------------------------------------------------------------------------
 * L4: System-Theoretic Properties
 * ---------------------------------------------------------------------------
 */

/* L4: Check stability: compute spectral radius of A matrix.
 *
 * For discrete-time: max|eig(A)| < 1  => stable
 * For continuous-time: max Re(eig(A)) < 0 => stable
 *
 * Uses the power iteration method to estimate the dominant
 * eigenvalue without full eigendecomposition.
 *
 * Knowledge point: Stability is the most basic system property.
 *   An unstable identified model may be correct (the process is
 *   open-loop unstable) or incorrect (numerical issues). The
 *   distinction requires engineering judgment.
 *
 * Complexity: O(n_x^2) per power iteration. */
int ssid_validation_check_stability(const ssid_matrix_t *A,
                                    int is_ct,
                                    double *spectral_radius,
                                    int *is_stable);

/* L4: Check observability of (C, A).
 *
 * Build the observability matrix O_n:
 *   O_n = [C; C*A; C*A^2; ...; C*A^{n_x-1}]
 *
 * The system is observable if rank(O_n) = n_x.
 *
 * Knowledge point: Observability determines whether the state can
 *   be reconstructed from outputs. For MPC, observability is
 *   required for the state estimator (Kalman filter) to work.
 *   Near-unobservable systems cause numerical issues.
 *
 * Complexity: O(n_x^2 * n_y * n_x) for Gramian computation. */
int ssid_validation_check_observability(const ssid_matrix_t *A,
                                        const ssid_matrix_t *C,
                                        int *is_observable,
                                        double *condition_number);

/* L4: Check controllability of (A, B).
 *
 * Build the controllability matrix C_n:
 *   C_n = [B, A*B, A^2*B, ..., A^{n_x-1}*B]
 *
 * The system is controllable if rank(C_n) = n_x.
 *
 * Knowledge point: Controllability determines whether the state
 *   can be driven to any desired value. For MPC, lack of
 *   controllability means some states cannot be influenced,
 *   which limits achievable control performance.
 *
 * Complexity: O(n_x^3 * n_u). */
int ssid_validation_check_controllability(const ssid_matrix_t *A,
                                          const ssid_matrix_t *B,
                                          int *is_controllable,
                                          double *condition_number);

/* ---------------------------------------------------------------------------
 * L5: Cross-Validation
 * ---------------------------------------------------------------------------
 */

/* L5: K-fold cross-validation for model order selection.
 *
 * Split data into K segments. For each candidate order n:
 *   Train on K-1 segments, validate on held-out segment.
 *   Average validation fit across all K folds.
 *   Select n that maximizes average fit.
 *
 * Knowledge point: Cross-validation provides a data-driven
 *   alternative to information criteria (AIC/BIC). It makes
 *   no distributional assumptions and is robust to outliers,
 *   at the cost of K-fold computation.
 *
 * Complexity: O(K * full_identification_time). */
size_t ssid_validation_kfold_order(const ssid_data_t *data,
                                   const ssid_config_t *cfg,
                                   size_t K,
                                   size_t n_min, size_t n_max,
                                   double *cv_scores);

/* L5: Split data into training and validation sets.
 *
 * train_ratio: fraction for training (e.g., 0.7)
 * The first train_ratio*N samples go to training;
 * the remainder to validation.
 *
 * Complexity: O(N) for copying. */
int ssid_validation_train_test_split(const ssid_data_t *data,
                                     double train_ratio,
                                     ssid_data_t *train,
                                     ssid_data_t *test);

/* ---------------------------------------------------------------------------
 * L7: Industrial Validation Reports
 * ---------------------------------------------------------------------------
 */

/* L7: Generate a validation report suitable for an industrial
 * Model Quality Assurance (MQA) document.
 *
 * Includes:
 *   - NRMSE fit on training and validation data
 *   - Residual whiteness (with significance levels)
 *   - Input-residual cross-correlation
 *   - Stability margin
 *   - Observability/controllability condition numbers
 *   - Multi-step prediction degradation
 *   - Pass/fail verdict against user-specified thresholds
 *
 * Knowledge point: In regulated industries (pharma GMP, FAA DO-178,
 *   nuclear IEC 61513), model validation must be documented with
 *   formal acceptance criteria. This function produces a structured
 *   report that can be audited.
 *
 * Complexity: O(N * n_x^2 + K * full_ID_time). */
int ssid_validation_industrial_report(const ssid_result_t *result,
                                      const ssid_data_t *train_data,
                                      const ssid_data_t *val_data,
                                      const ssid_config_t *cfg,
                                      char *report_buffer,
                                      size_t buffer_size);

/* L7: Check model suitability for MPC deployment.
 *
 * MPC-specific checks:
 *   - Step response settling time vs. control horizon
 *   - Inverse response detection (RHP zero warning)
 *   - Integrator count (offset-free tracking requirements)
 *   - Input-output delay estimation
 *   - Gain sign consistency (sign(C*B) should match process knowledge)
 *
 * Knowledge point: A model that passes statistical tests may
 *   still be unsuitable for MPC if it doesn't capture the
 *   physical characteristics the optimizer relies on.
 *
 * Complexity: O(n_x^3 + H_p * n_x^2). */
int ssid_validation_mpc_readiness(const ssid_model_t *model,
                                  size_t H_p, double Ts,
                                  char *issues_buffer,
                                  size_t buffer_size);

#endif /* SSID_VALIDATION_H */
