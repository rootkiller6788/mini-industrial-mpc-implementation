# Course Tree -- MPC Performance Monitoring & KPI

## Prerequisites
- **mini-pid-control-engineering** (L1-L3): PID feedback control, controller performance concepts
- **mini-advanced-process-control-apc** (L4): MPC fundamentals, QP optimization, prediction horizons
- **mini-industrial-measurement-actuator** (L1): Sensor fundamentals, data quality, measurement noise

## Core Dependencies
- Time series analysis: autocorrelation, partial autocorrelation, stationarity (ADF test)
- Linear regression: ordinary least squares, trend analysis, R-squared
- Hypothesis testing: Ljung-Box portmanteau test, F-test for variance, chi-squared distribution
- Statistical process control: CUSUM, EWMA, Shewhart charts
- Optimization theory: QP solver diagnostics (active constraints, iterations, solve time)
- Robust statistics: median, MAD, IQR, Theil-Sen estimator, bootstrap confidence intervals
- Information theory: mutual information for KPI coupling analysis
- Numerical linear algebra: Toeplitz systems (Yule-Walker), Levinson-Durbin recursion

## Knowledge Layers Mapped

```
L1 Definitions ©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤ 19 structs, 8 enums, 7 string converters
  ©¦
L2 Core Concepts ©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤ Harris index, mismatch detection, utilization, constraints
  ©¦
L3 Engineering Structures ©¤©¤©¤©¤ Ring buffer O(1), EWMA, CUSUM, dashboard aggregation
  ©¦
L4 Engineering Laws ©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤ Harris formula, EWMA recurrence, Ljung-Box, Yule-Walker
  ©¦
L5 Algorithms ©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤ 17 algorithms (AR modeling, ACF, XCF, ADF, bootstrap, etc.)
  ©¦
L6 Canonical Problems ©¤©¤©¤©¤©¤©¤©¤©¤ Distillation, FCC, petrochemical plant scenarios
  ©¦
L7 Industrial Applications ©¤©¤©¤ 13 vendor/standard KPI implementations
  ©¦
L8 Advanced Topics ©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤ 12 advanced methods (Bayesian, subspace, Pareto, etc.)
  ©¦
L9 Research Frontiers ©¤©¤©¤©¤©¤©¤©¤©¤ Autonomous health, digital twin, IT/OT convergence
```

## Provides Foundation For
- **mini-soft-sensor-inferential**: KPI as soft sensor quality metric, prediction error monitoring
- **mini-industrial-ai-control-fusion**: AI-driven KPI optimization, anomaly detection on KPIs
- **mini-mes-digital-factory**: OEE calculation, throughput monitoring, batch cycle time
- **mini-control-system-cybersecurity**: Alarm flood detection, data integrity monitoring
- **Predictive maintenance modules**: Degradation rate estimation, critical threshold forecasting
- **Digital twin platforms**: KPI synchronization, model fidelity assessment
