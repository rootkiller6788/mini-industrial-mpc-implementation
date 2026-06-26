/**
 * @file mpc_ots_assessment.h
 * @brief Operator performance assessment and analytics for MPC OTS.
 *
 * Implements multi-dimensional operator performance evaluation,
 * learning trend analysis, and competency certification algorithms.
 *
 * @section L2_Core_Concepts
 * - Multi-metric performance scoring (7 dimensions)
 * - Response time analysis with penalty for delayed actions
 * - Situation awareness assessment (correct diagnosis weighting)
 * - Constraint compliance scoring (hard vs soft violation penalties)
 * - Economic optimality: deviation from LP optimum
 * - Progress tracking with statistical process control
 * - Competency certification thresholds
 *
 * @section L3_Engineering_Structures
 * - Weighted scoring with configurable metric weights
 * - Rolling window performance statistics
 * - Learning curve detection (logarithmic, exponential, plateau models)
 * - Statistical significance testing for improvement claims
 *
 * @section L4_Engineering_Laws
 * - Kirkpatrick Model Level 2 (Learning) and Level 3 (Behavior) evaluation
 * - EEMUA 201 competency assessment criteria
 * - ASM Consortium operator performance metrics
 * - ISA-101 alarm management performance KPIs
 *
 * @section L5_Algorithms
 * - Weighted geometric mean scoring (penalizes uneven performance)
 * - Ordinary least squares (OLS) trend line fitting
 * - Exponentially weighted moving average (EWMA) for stability
 * - Plateau detection via CUSUM or change-point analysis
 * - Confidence interval computation for performance bands
 * - Elo rating system adaptation for training difficulty
 *
 * Reference:
 *   Kirkpatrick & Kirkpatrick (2006), "Evaluating Training Programs", 3rd ed.
 *   ASM Consortium (2011), "Effective Operator Display Design"
 *   EEMUA 201 (2013), "Process plant control desks utilising HMI"
 *   Elo (1978), "The Rating of Chessplayers, Past and Present"
 *   Montgomery (2020), "Introduction to Statistical Quality Control", 8th ed.
 */

#ifndef MPC_OTS_ASSESSMENT_H
#define MPC_OTS_ASSESSMENT_H

#include "mpc_ots_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Constants
 * ========================================================================= */

#define MPC_OTS_NUM_METRICS       7       /**< Number of performance dimensions */
#define MPC_OTS_MIN_HISTORY       3       /**< Minimum sessions for trend analysis */
#define MPC_OTS_PLATEAU_WINDOW    8       /**< Sessions to detect plateau */
#define MPC_OTS_PLATEAU_THRESHOLD 0.02    /**< Score delta below which = plateau */
#define MPC_OTS_DEFAULT_ELO_K     32.0    /**< Default Elo K-factor */
#define MPC_OTS_CERTIFICATION_THRESHOLD 70.0 /**< Passing score for competency cert */
#define MPC_OTS_EXPERT_THRESHOLD  90.0     /**< Expert-level competency threshold */

/* =========================================================================
 * L2: Default Metric Weights
 * ========================================================================= */

/** Default weights for 7 performance metrics (sum to 1.0).
 *  Based on ASM Consortium operator assessment framework. */
#define MPC_OTS_WEIGHT_RESPONSE_TIME     0.20
#define MPC_OTS_WEIGHT_STABILITY         0.20
#define MPC_OTS_WEIGHT_CONSTRAINTS       0.20
#define MPC_OTS_WEIGHT_ECONOMIC          0.15
#define MPC_OTS_WEIGHT_ALARM             0.10
#define MPC_OTS_WEIGHT_AWARENESS         0.10
#define MPC_OTS_WEIGHT_CONSISTENCY       0.05

/* =========================================================================
 * L2+L3: Performance Statistics Structures
 * ========================================================================= */

/** Per-metric performance statistics.
 *  Captures central tendency and dispersion for a single metric. */
typedef struct {
    PerformanceMetric metric;              /**< Metric identifier */
    double            mean;                /**< Arithmetic mean */
    double            std_dev;             /**< Standard deviation */
    double            median;              /**< Median value */
    double            min;                 /**< Minimum observed */
    double            max;                 /**< Maximum observed */
    double            ewma_current;        /**< Exponentially weighted moving average */
    double            ewma_alpha;          /**< EWMA smoothing factor */
    int32_t           num_samples;         /**< Number of observations */
    double            last_value;          /**< Most recent value */
    double            second_last_value;   /**< Penultimate value (for delta) */
} MetricStatistics;

/** Comprehensive operator performance statistics.
 *  Aggregated across all metrics for an individual operator. */
typedef struct {
    int32_t           operator_id;         /**< Operator identifier */
    MetricStatistics  metrics[MPC_OTS_NUM_METRICS]; /**< Per-metric statistics */
    double            overall_mean;        /**< Mean overall score */
    double            overall_std_dev;     /**< Overall score standard deviation */
    double            overall_ewma;        /**< EWMA of overall scores */
    double            learning_rate;       /**< Estimated learning rate (score/session) */
    double            plateau_score;       /**< Detected plateau level */
    double            best_session_score;  /**< All-time best session score */
    int32_t           best_session_id;     /**< Session ID of best performance */
    int32_t           total_sessions;      /**< Total sessions analyzed */
    int32_t           sessions_since_improvement; /**< Plateau counter */
    bool              is_at_plateau;       /**< Plateau detection flag */
    bool              is_improving;        /**< Positive trend flag */
    double            confidence_improving; /**< Statistical confidence (p-value) */
} OperatorStatistics;

/** Competency certification assessment.
 *  Determines operator proficiency level from performance history. */
typedef struct {
    int32_t           operator_id;         /**< Operator identifier */
    OperatorRole       assessed_role;       /**< Assessed competency level */
    double            certification_score; /**< Overall competency score */
    bool              passes_response_time; /**< Metric-level pass/fail */
    bool              passes_stability;
    bool              passes_constraints;
    bool              passes_economic;
    bool              passes_alarm;
    bool              passes_awareness;
    bool              is_certified;        /**< Overall certification result */
    double            confidence_level;    /**< Certification confidence (0-1) */
    int32_t           required_sessions;   /**< Minimum sessions for cert */
    int32_t           sessions_completed;  /**< Actual sessions completed */
    char              comments[MPC_OTS_GUIDANCE_TEXT_MAX]; /**< Certification notes */
} CompetencyCertification;

/* =========================================================================
 * L5: Scoring Algorithms — Core API
 * ========================================================================= */

/**
 * Normalize a performance metric value to 0-100 score.
 *
 * For response_time: score decays exponentially beyond optimal.
 *   score = 100 * exp(-(actual - optimal)^2 / (2 * sigma^2))
 *   where sigma = optimal * 0.5 (half the optimal time as std dev).
 *
 * For deviation: score decays linearly with excess deviation.
 *   score = 100 * max(0, 1 - (actual - allowed) / (allowed * tolerance_factor))
 *
 * Complexity: O(1).
 */
double ots_normalize_response_time(double actual_time, double optimal_time);

/**
 * Normalize deviation metric to 0-100 score.
 * Score decreases as deviation exceeds allowed maximum.
 *
 * Complexity: O(1).
 */
double ots_normalize_deviation(double actual_deviation, double max_allowed);

/**
 * Compute single-metric normalized score for a performance record.
 * Routes to appropriate normalization function based on metric type.
 *
 * Complexity: O(1).
 */
double ots_score_metric(PerformanceMetric metric, double measured, double expected);

/**
 * Compute weighted overall score from per-metric sub-scores.
 *
 * Uses weighted geometric mean: score = ∏(s_i^w_i)
 * where s_i are normalized sub-scores and w_i are weights summing to 1.0.
 *
 * This formulation penalizes very poor performance in any single dimension
 * more severely than arithmetic mean, matching the ASM Consortium's findings
 * that operator effectiveness requires balanced competency.
 *
 * Complexity: O(k) where k = number of metrics.
 *
 * Reference: ASM Consortium (2011), multi-attribute utility for operator assessment.
 */
double ots_score_weighted_overall(const double scores[], const double weights[], int32_t num_metrics);

/**
 * Compute the exponentially weighted moving average of scores.
 * EWMA[t] = alpha * score[t] + (1 - alpha) * EWMA[t-1]
 * Default alpha = 0.3 for moderate smoothing.
 *
 * Complexity: O(n) where n = count.
 *
 * Reference: Montgomery (2020), "Statistical Quality Control", Ch.9.
 */
double ots_ewma_compute(const double scores[], int32_t count, double alpha);

/* =========================================================================
 * L5: Trend Analysis
 * ========================================================================= */

/**
 * Fit ordinary least squares regression to score history.
 *
 * Model: score = intercept + slope * session_index
 * Returns slope (learning rate) and r_squared (fit quality).
 *
 * Complexity: O(n) where n = history length.
 *
 * Formula:
 *   slope = (n * Σ(xy) - Σx * Σy) / (n * Σ(x²) - (Σx)²)
 *   r² = 1 - SS_residual / SS_total
 */
OTSStatus ots_fit_linear_trend(const double scores[], const int32_t session_indices[], int32_t count, double *slope, double *intercept, double *r_squared);

/**
 * Detect performance plateau using modified CUSUM.
 *
 * A plateau is detected when the absolute change in EWMA over the
 * plateau window is less than the threshold.
 *
 * Complexity: O(w) where w = plateau window size.
 *
 * Returns true if plateau detected.
 */
bool ots_detect_plateau(const double scores[], int32_t count, int32_t window, double threshold);

/**
 * Compute two-sided 95% confidence interval for mean performance.
 *
 * Uses Student's t-distribution critical value approximation:
 *   t_approx = 1.96 + 2.38/(n-1) for df > 2
 *   CI = mean ± t * std_dev / sqrt(n)
 *
 * Complexity: O(n) to compute mean and std_dev.
 */
OTSStatus ots_confidence_interval(const double scores[], int32_t count, double *lower, double *upper);

/* =========================================================================
 * L5: Competency Assessment
 * ========================================================================= */

/**
 * Assess operator competency based on performance history.
 *
 * Checks each metric against certification thresholds for the target role.
 * Thresholds vary by role:
 *   - TRAINEE → ≥60 on all metrics
 *   - JUNIOR → ≥70 on all metrics
 *   - SENIOR → ≥80 on all metrics
 *   - EXPERT → ≥90 on all metrics
 *
 * Complexity: O(m * n) where m = metrics, n = sessions.
 *
 * Reference: Kirkpatrick Model Level 3 (Behavior) evaluation criteria.
 */
OTSStatus ots_assess_competency(const OperatorStatistics *stats, OperatorRole target_role, CompetencyCertification *cert);

/**
 * Generate comprehensive operator statistics from performance records.
 * Computes mean, std_dev, median, EWMA for all 7 metrics.
 *
 * Complexity: O(m * n) where m = 7 metrics, n = records.
 */
OTSStatus ots_compute_statistics(const PerformanceRecord records[], int32_t count, OperatorStatistics *stats);

/**
 * Predict future performance using linear extrapolation of trend.
 * Returns predicted score after n_forward sessions.
 *
 * Complexity: O(1) given pre-computed slope and intercept.
 */
double ots_predict_future_score(double slope, double intercept, int32_t current_sessions, int32_t sessions_forward);

/* =========================================================================
 * L7: Industrial Reporting
 * ========================================================================= */

/**
 * Generate operator competency report suitable for regulatory documentation.
 * Includes: overall score, per-metric breakdown, trend analysis, plateau status.
 *
 * Complexity: O(m + n) where m = metrics, n = sessions in stats.
 */
OTSStatus ots_generate_competency_report(const OperatorStatistics *stats, const CompetencyCertification *cert, char *report_buffer, int32_t buffer_size);

/**
 * Compare operator performance against industry benchmarks.
 * Benchmark data from: ASM Consortium, Honeywell OTS studies, AspenTech benchmarks.
 *
 * Complexity: O(m) where m = number of metrics.
 *
 * Returns percentile ranking (0-100) relative to industry peers.
 */
OTSStatus ots_benchmark_operator(const OperatorStatistics *stats, double percentile_rankings[], int32_t num_metrics);

#ifdef __cplusplus
}
#endif

#endif /* MPC_OTS_ASSESSMENT_H */
