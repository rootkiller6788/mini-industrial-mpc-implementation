# Knowledge Graph -- MPC Performance Monitoring & KPI

## L1: Definitions
- `mpc_kpi_category_t`: 7 KPI categories (availability, performance, quality, economic, constraint, diagnostic, robustness)
- `mpc_kpi_id_t`: 20 distinct KPI metrics (Harris index, utilization, constraint sat, mismatch, prediction error, setpoint tracking, economic benefit, energy savings, variance reduction, settling time, overshoot, QP solve time, QP iterations, active constraints, MV travel, MV saturation, I/O balance, CV correlation, throughput, yield)
- `mpc_kpi_tier_t`: 5-tier performance classification (excellent, good, fair, poor, critical)
- `mpc_kpi_status_t`: 12 error/success codes with complete error handling
- `mpc_kpi_data_quality_t`: 7 data quality flags (valid, suspect, stale, outlier, limited, interpolated, flatline, insufficient)
- `mpc_kpi_mode_t`: 6 monitoring modes (idle, baselining, monitoring, alarm, diagnosing, reporting)
- `mpc_kpi_severity_t`: 4 alarm severity levels (info, warning, alert, critical)
- `mpc_mismatch_type_t`: 8 mismatch types (none, gain bias, gain slope, deadtime, time constant, inverse response, nonlinearity, disturbance)
- `mpc_kpi_value_t`: Full KPI lifecycle data structure with rolling statistics, EWMA, trend
- `mpc_kpi_ringbuffer_t`: O(1) statistics ring buffer with moment tracking up to 4th order
- `mpc_kpi_ewma_t`: EWMA filter with variance and volatility tracking
- `mpc_kpi_cusum_t`: CUSUM change detector with dual-sided monitoring
- `mpc_kpi_autocorr_t`: Autocorrelation with Ljung-Box whiteness test
- `mpc_kpi_harris_t`: Harris performance index with minimum variance benchmark
- `mpc_kpi_mismatch_t`: Model-plant mismatch with F-test significance
- `mpc_kpi_dashboard_t`: Multi-KPI dashboard with category-weighted health scoring
- `mpc_kpi_alarm_rule_t`: Configurable alarm rule with hysteresis and deadband
- `mpc_kpi_report_t`: Report snapshot for period-end KPI aggregation
- `mpc_kpi_baseline_t`: Statistical baseline with normality test and percentile bounds

## L2: Core Concepts
- Harris Performance Index: minimum-variance benchmark for control loop assessment
- Model-plant mismatch detection via residual cross-correlation analysis
- Controller utilization rate monitoring (auto/manual mode tracking)
- Constraint satisfaction statistics with violation magnitude analysis
- Economic benefit quantification vs. non-MPC baseline operation
- Performance tier classification with normalized error thresholds
- EWMA smoothing for noisy KPI measurement filtering
- CUSUM change detection for persistent performance shifts
- Baseline estimation with robust outlier removal
- Alarm management with severity classification and hysteresis
- Multi-category health score aggregation with configurable weighting
- Data quality assessment with multi-flag quality encoding

## L3: Engineering Structures
- Ring buffer with O(1) running moments (sum, sum_sq, sum_cube, sum_quart)
- EWMA filter for KPI smoothing with recursive variance tracking
- CUSUM accumulator with dual-sided persistent shift detection
- Multi-KPI dashboard with category-weighted health scoring
- Alarm rule evaluation with sustain cycles and deadband logic
- Report snapshot generation for period-end aggregation
- HMI display string formatting with tier-color coding
- Dashboard pointer validation utility

## L4: Engineering Laws
- Harris index: eta = 1 - sigma^2_MV/sigma^2_y, in [0,1]
- EWMA mean recurrence: mu_k = lambda*x_k + (1-lambda)*mu_{k-1}
- EWMA variance: sigma^2_k = (1-lambda)*sigma^2_{k-1} + lambda*(x_k - mu_{k-1})^2
- CUSUM positive: S+_k = max(0, S+_{k-1} + (x_k-mu_0)/sigma - k)
- CUSUM decision: S+ > h => alarm (h typically 4-5)
- Ljung-Box portmanteau: Q = n(n+2)*SUM[r^2_k/(n-k)] ~ chi^2(h)
- Bartlett's formula: sigma(r_k) approx 1/sqrt(n) for white noise
- Yule-Walker equations: R*phi = r (Toeplitz system)
- Levinson-Durbin recursion: O(p^2) solution for AR(p) parameters
- F-test for variance ratio: F = s^2_current/s^2_baseline
- Chi-squared CDF approximation for hypothesis testing

## L5: Algorithms
- Harris index via AR model fitting (Yule-Walker + Levinson-Durbin recursion)
- Autocorrelation function with Bartlett confidence bounds
- Partial autocorrelation via Durbin-Levinson recursion
- Ljung-Box portmanteau test for residual whiteness
- Model mismatch via cross-correlation of residuals and MV moves
- Outlier detection via IQR method with configurable multiplier
- Linear regression for trend detection with R-squared
- CUSUM change detection with configurable decision interval
- EWMA filtering and volatility estimation
- Augmented Dickey-Fuller test for stationarity
- Bootstrap confidence interval for KPI mean
- Recursive residual CUSUM for model stability
- Mutual information between KPIs (discrete binning)
- Durbin-Watson statistic for residual autocorrelation
- Runs test for randomness of KPI deviations
- Theil-Sen robust trend estimator (median of pairwise slopes)
- Granger causality test between KPIs (simplified)

## L6: Canonical Problems
- Distillation column Harris index evaluation (3 scenarios)
- FCC reactor KPI baselining with outlier removal and normality testing
- Process capability analysis (Cp, Cpk, PPM defective)
- Oscillation diagnosis via autocorrelation peak detection
- Stiction detection via MV/CV movement pattern analysis
- Sluggishness estimation from setpoint response analysis
- Petrochemical plant multi-KPI dashboard (12 KPIs, 5 categories)
- Degradation rate estimation with critical threshold forecasting
- Improvement scenario analysis with ROI estimation
- Seasonal decomposition for periodic KPI patterns

## L7: Industrial Applications
- AspenTech AspenWatch KPI monitoring (health index + service factor)
- Honeywell Profit Sensor performance assessment (profit impact + model quality)
- Yokogawa MD (Multivariable Diagnostic) with process capability
- Shell MV control monitor (overall health + bottleneck detection)
- ISO 50001 Energy Performance Indicator (EnPI)
- CO2 emission reduction estimation
- Energy Intensity Index (EII) per ISO 50001
- Rockwell Pavilion8 model quality assessment
- Siemens Simatic PCS 7 APC monitoring
- Emerson DeltaV MPC health status
- Yokogawa Exapilot transition monitoring
- ABB Ability System 800xA APC KPI
- OSIsoft PI Asset Framework KPI template

## L8: Advanced Topics
- Bayesian change-point detection for KPI shifts
- Subspace-based model validation (singular value ratio)
- Multi-KPI Pareto frontier analysis for trade-off optimization
- Time-varying KPI analysis with forgetting factor
- KPI trend forecasting with prediction intervals
- Kalman filter innovation monitoring for estimator health
- Monte Carlo simulation for KPI robustness analysis
- Multi-rate KPI aggregation with aliasing detection
- Holt's exponential smoothing with trend
- Correlation matrix condition number for collinearity check
- Nonlinearity detection via bispectrum surrogate
- Tuning aggressiveness index with smoothness analysis

## L9: Research Frontiers
- Autonomous MPC health management with intervention frequency
- Digital twin-KPI synchronization with fidelity assessment
- IT/OT convergence readiness assessment
- Cybersecurity readiness scoring
- Cloud readiness evaluation
- Predictive degradation rate estimation
- Multi-vendor KPI benchmarking framework
