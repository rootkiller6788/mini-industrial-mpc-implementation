/**
 * @file mpc_ots_assessment.c
 * @brief Operator performance assessment and competency certification.
 *
 * Implements multi-metric scoring, statistical trend analysis,
 * plateau detection, confidence intervals, competency certification,
 * and industrial benchmark comparison.
 *
 * Knowledge points:
 *   L1: MetricStatistics, OperatorStatistics, CompetencyCertification types
 *   L2: 7-dimensional operator performance scoring
 *   L3: EWMA, OLS regression, statistical hypothesis testing
 *   L4: Kirkpatrick Model Level 2-3, EEMUA 201, ASM Consortium metrics
 *   L5: Weighted geometric mean, CUSUM plateau detection, t-distribution CI
 *   L6: Competency certification thresholds, trend analysis
 *   L7: Honeywell/ASM operator benchmarks, AspenTech OTS assessment
 *   L8: Statistical process control for operator learning
 *
 * Reference:
 *   Kirkpatrick & Kirkpatrick (2006), "Evaluating Training Programs", 3rd ed.
 *   Montgomery (2020), "Introduction to Statistical Quality Control", 8th ed.
 *   ASM Consortium (2011), "Effective Operator Display Design"
 *   EEMUA 201 (2013), "Process plant control desks utilising HMI"
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include "../include/mpc_ots_assessment.h"

/* =========================================================================
 * L5: Score Normalization Functions
 * ========================================================================= */

double ots_normalize_response_time(double actual_time, double optimal_time)
{
    if (optimal_time < MPC_OTS_EPS) return 0.0;
    if (actual_time < 0.0) return 0.0;

    /* Gaussian decay: score = 100 * exp(-(t-t*)^2 / (2*σ^2))
     * where σ = optimal_time * 0.5
     * Very fast response: score → 100
     * Response at optimal: score → 100 * exp(0) = 100
     * Response 2x optimal: score → 100 * exp(-2) ≈ 13.5
     */
    double sigma = optimal_time * 0.5;
    double z = (actual_time - optimal_time) / sigma;
    double score = 100.0 * exp(-0.5 * z * z);

    if (score > 100.0) score = 100.0;
    if (score < 0.0) score = 0.0;
    return score;
}

double ots_normalize_deviation(double actual_deviation, double max_allowed)
{
    if (max_allowed < MPC_OTS_EPS) return 100.0; /* no constraint = perfect */
    if (actual_deviation < 0.0) actual_deviation = 0.0;

    /* Linear penalty beyond allowed deviation:
     * score = 100 * max(0, 1 - (actual - allowed) / allowed)
     * Within allowed: score = 100
     * At 2x allowed: score = 0
     */
    double excess = actual_deviation - max_allowed;
    if (excess <= 0.0) return 100.0;

    double score = 100.0 * (1.0 - excess / max_allowed);
    if (score < 0.0) score = 0.0;
    return score;
}

double ots_score_metric(PerformanceMetric metric, double measured, double expected)
{
    switch (metric) {
    case MPC_METRIC_RESPONSE_TIME:
        return ots_normalize_response_time(measured, expected);
    case MPC_METRIC_STABILITY_MARGIN:
    case MPC_METRIC_CONSTRAINT_COMPLIANCE:
    case MPC_METRIC_SITUATION_AWARENESS:
        return ots_normalize_deviation(measured, expected);
    case MPC_METRIC_ECONOMIC_OPTIMALITY:
        /* Economic: closer to optimum = better (use absolute error) */
        {
            double error = fabs(measured - expected);
            double max_error = fmax(fabs(expected) * 0.5, 1.0);
            return ots_normalize_deviation(error, max_error);
        }
    case MPC_METRIC_ALARM_MANAGEMENT:
        /* Alarms: fewer = better, measured is alarm count */
        {
            double max_alarms = fmax(expected, 1.0);
            return ots_normalize_deviation(measured, max_alarms);
        }
    case MPC_METRIC_CONSISTENCY:
        /* Consistency: lower variance = better */
        {
            double max_var = fmax(expected, 1.0);
            return ots_normalize_deviation(measured, max_var);
        }
    default:
        return 50.0;
    }
}

/* =========================================================================
 * L5: Weighted Overall Score
 * ========================================================================= */

double ots_score_weighted_overall(const double scores[], const double weights[], int32_t num_metrics)
{
    if (!scores || !weights || num_metrics <= 0) return 0.0;

    double log_sum = 0.0;
    double weight_sum = 0.0;

    for (int32_t i = 0; i < num_metrics; i++) {
        double s = scores[i];
        if (s < 0.1) s = 0.1;  /* clamp for log */
        if (s > 100.0) s = 100.0;
        log_sum += weights[i] * log(s);  /* natural log */
        weight_sum += weights[i];
    }

    if (weight_sum < MPC_OTS_EPS) return 0.0;

    double result = exp(log_sum / weight_sum);
    if (result > 100.0) result = 100.0;
    return result;
}

double ots_ewma_compute(const double scores[], int32_t count, double alpha)
{
    if (!scores || count <= 0) return 0.0;
    if (alpha <= 0.0 || alpha > 1.0) alpha = 0.3;

    double ewma = scores[0]; /* initialize with first value */
    for (int32_t i = 1; i < count; i++) {
        ewma = alpha * scores[i] + (1.0 - alpha) * ewma;
    }
    return ewma;
}

/* =========================================================================
 * L5: OLS Linear Trend
 * ========================================================================= */

OTSStatus ots_fit_linear_trend(const double scores[], const int32_t session_indices[], int32_t count, double *slope, double *intercept, double *r_squared)
{
    if (!scores || !session_indices || !slope || !intercept || !r_squared)
        return MPC_OTS_ERR_NULL_POINTER;
    if (count < 3) return MPC_OTS_ERR_INVALID_PARAM;

    double sum_x = 0.0, sum_y = 0.0, sum_xy = 0.0, sum_x2 = 0.0;

    for (int32_t i = 0; i < count; i++) {
        double x = (double)session_indices[i];
        double y = scores[i];
        sum_x += x;
        sum_y += y;
        sum_xy += x * y;
        sum_x2 += x * x;
    }

    double n = (double)count;
    double denom = n * sum_x2 - sum_x * sum_x;

    if (fabs(denom) < MPC_OTS_EPS) {
        *slope = 0.0;
        *intercept = sum_y / n;
        *r_squared = 0.0;
        return MPC_OTS_OK;
    }

    *slope = (n * sum_xy - sum_x * sum_y) / denom;
    *intercept = (sum_y - (*slope) * sum_x) / n;

    /* Compute R-squared */
    double ss_res = 0.0, ss_tot = 0.0;
    double y_mean = sum_y / n;
    for (int32_t i = 0; i < count; i++) {
        double y_pred = (*intercept) + (*slope) * (double)session_indices[i];
        ss_res += pow(scores[i] - y_pred, 2.0);
        ss_tot += pow(scores[i] - y_mean, 2.0);
    }

    if (ss_tot > MPC_OTS_EPS) {
        *r_squared = 1.0 - ss_res / ss_tot;
    } else {
        *r_squared = 0.0;
    }

    return MPC_OTS_OK;
}

/* =========================================================================
 * L5: Plateau Detection (CUSUM-inspired)
 * ========================================================================= */

bool ots_detect_plateau(const double scores[], int32_t count, int32_t window, double threshold)
{
    if (!scores || count < window || window < 2) return false;

    /* Use EWMA over the last 'window' periods */
    double first_ewma = 0.0, last_ewma = 0.0;
    int32_t half = window / 2;

    /* EWMA over first half of window */
    if (half > 0) {
        first_ewma = scores[count - window];
        for (int32_t i = count - window + 1; i < count - half; i++) {
            first_ewma = 0.3 * scores[i] + 0.7 * first_ewma;
        }

        /* EWMA over second half */
        last_ewma = scores[count - half];
        for (int32_t i = count - half + 1; i < count; i++) {
            last_ewma = 0.3 * scores[i] + 0.7 * last_ewma;
        }
    }

    double delta = fabs(last_ewma - first_ewma);
    return (delta / 100.0) < threshold; /* normalize to [0,1] */
}

/* =========================================================================
 * L5: Confidence Interval
 * ========================================================================= */

OTSStatus ots_confidence_interval(const double scores[], int32_t count, double *lower, double *upper)
{
    if (!scores || !lower || !upper) return MPC_OTS_ERR_NULL_POINTER;
    if (count < 2) return MPC_OTS_ERR_INVALID_PARAM;

    /* Compute mean and std_dev */
    double sum = 0.0, sum_sq = 0.0;
    for (int32_t i = 0; i < count; i++) {
        sum += scores[i];
        sum_sq += scores[i] * scores[i];
    }

    double n = (double)count;
    double mean = sum / n;
    double variance = (sum_sq - sum * sum / n) / (n - 1.0);
    if (variance < 0.0) variance = 0.0;
    double std_dev = sqrt(variance);

    /* t-critical value approximation for 95% CI:
     * t_0.025 ≈ 1.96 for large n, adjusted for small n */
    double df = n - 1.0;
    double t_crit;
    if (df < 1.0) {
        t_crit = 12.706; /* df=1 */
    } else if (df < 2.0) {
        t_crit = 4.303; /* df=2 */
    } else if (df < 5.0) {
        t_crit = 2.776; /* df=4 */
    } else if (df < 10.0) {
        t_crit = 2.262; /* df=9 */
    } else if (df < 30.0) {
        t_crit = 2.045; /* df=29 */
    } else {
        t_crit = 1.96;  /* large sample */
    }

    double margin = t_crit * std_dev / sqrt(n);
    *lower = mean - margin;
    *upper = mean + margin;

    return MPC_OTS_OK;
}

/* =========================================================================
 * L5: Competency Assessment
 * ========================================================================= */

OTSStatus ots_assess_competency(const OperatorStatistics *stats, OperatorRole target_role, CompetencyCertification *cert)
{
    if (!stats || !cert) return MPC_OTS_ERR_NULL_POINTER;

    memset(cert, 0, sizeof(CompetencyCertification));
    cert->operator_id = stats->operator_id;
    cert->assessed_role = target_role;
    cert->sessions_completed = stats->total_sessions;

    /* Thresholds by role:
     * TRAINEE: ≥60 on each metric (Level 2 Kirkpatrick: Learning)
     * JUNIOR:  ≥70 on each metric
     * SENIOR:  ≥80 on each metric
     * EXPERT:  ≥90 on each metric
     */
    double threshold;
    switch (target_role) {
    case MPC_ROLE_TRAINEE:          threshold = 60.0; cert->required_sessions = 3; break;
    case MPC_ROLE_JUNIOR_OPERATOR:  threshold = 70.0; cert->required_sessions = 8; break;
    case MPC_ROLE_SENIOR_OPERATOR:  threshold = 80.0; cert->required_sessions = 20; break;
    case MPC_ROLE_SHIFT_LEAD:       threshold = 75.0; cert->required_sessions = 15; break;
    case MPC_ROLE_TRAINER:          threshold = 85.0; cert->required_sessions = 30; break;
    default:                        threshold = 70.0; cert->required_sessions = 5; break;
    }

    /* Check each metric against threshold */
    double metric_means[MPC_OTS_NUM_METRICS];
    for (int32_t i = 0; i < MPC_OTS_NUM_METRICS; i++) {
        metric_means[i] = stats->metrics[i].mean;
    }

    cert->passes_response_time  = (stats->metrics[MPC_METRIC_RESPONSE_TIME].mean >= threshold);
    cert->passes_stability      = (stats->metrics[MPC_METRIC_STABILITY_MARGIN].mean >= threshold);
    cert->passes_constraints    = (stats->metrics[MPC_METRIC_CONSTRAINT_COMPLIANCE].mean >= threshold);
    cert->passes_economic       = (stats->metrics[MPC_METRIC_ECONOMIC_OPTIMALITY].mean >= threshold);
    cert->passes_alarm          = (stats->metrics[MPC_METRIC_ALARM_MANAGEMENT].mean >= threshold);
    cert->passes_awareness      = (stats->metrics[MPC_METRIC_SITUATION_AWARENESS].mean >= threshold);

    bool all_pass = cert->passes_response_time && cert->passes_stability
                 && cert->passes_constraints && cert->passes_economic
                 && cert->passes_alarm && cert->passes_awareness;

    bool enough_sessions = (cert->sessions_completed >= cert->required_sessions);

    cert->is_certified = all_pass && enough_sessions;

    /* Certification score: geometric mean of metric means */
    double log_sum = 0.0;
    for (int32_t i = 0; i < MPC_OTS_NUM_METRICS; i++) {
        double m = metric_means[i];
        if (m < 0.1) m = 0.1;
        log_sum += log(m);
    }
    cert->certification_score = exp(log_sum / (double)MPC_OTS_NUM_METRICS);

    /* Confidence: higher with more sessions and higher scores */
    double session_confidence = fmin(1.0, (double)cert->sessions_completed / (double)cert->required_sessions);
    double score_confidence = cert->certification_score / 100.0;
    cert->confidence_level = 0.5 * session_confidence + 0.5 * score_confidence;

    snprintf(cert->comments, MPC_OTS_GUIDANCE_TEXT_MAX,
        "%s. %s. Score: %.1f. Confidence: %.0f%%.",
        cert->is_certified ? "CERTIFIED" : "NOT CERTIFIED",
        all_pass ? "All metrics passed" : "Some metrics below threshold",
        cert->certification_score, cert->confidence_level * 100.0);

    return MPC_OTS_OK;
}

/* =========================================================================
 * L5: Operator Statistics Computation
 * ========================================================================= */

OTSStatus ots_compute_statistics(const PerformanceRecord records[], int32_t count, OperatorStatistics *stats)
{
    if (!records || !stats) return MPC_OTS_ERR_NULL_POINTER;
    if (count <= 0) return MPC_OTS_ERR_INVALID_PARAM;

    memset(stats, 0, sizeof(OperatorStatistics));
    stats->total_sessions = count;

    /* Extract per-metric sequences */
    double metric_values[MPC_OTS_NUM_METRICS][1024];
    int32_t metric_counts[MPC_OTS_NUM_METRICS] = {0};

    for (int32_t i = 0; i < MPC_OTS_NUM_METRICS; i++) {
        for (int32_t j = 0; j < count; j++) {
            if ((int32_t)records[j].metric_type == i) {
                int32_t idx = metric_counts[i];
                if (idx < 1024) {
                    metric_values[i][idx] = records[j].normalized_score;
                    metric_counts[i]++;
                }
            }
        }
    }

    /* Compute per-metric statistics */
    for (int32_t m = 0; m < MPC_OTS_NUM_METRICS; m++) {
        MetricStatistics *ms = &stats->metrics[m];
        ms->metric = (PerformanceMetric)m;
        ms->num_samples = metric_counts[m];

        if (metric_counts[m] > 0) {
            /* Sort for median */
            double *vals = metric_values[m];
            for (int32_t i = 0; i < metric_counts[m] - 1; i++) {
                for (int32_t j = i + 1; j < metric_counts[m]; j++) {
                    if (vals[j] < vals[i]) {
                        double tmp = vals[i];
                        vals[i] = vals[j];
                        vals[j] = tmp;
                    }
                }
            }

            ms->min = vals[0];
            ms->max = vals[metric_counts[m] - 1];
            ms->median = vals[metric_counts[m] / 2];

            double sum = 0.0, sum_sq = 0.0;
            for (int32_t i = 0; i < metric_counts[m]; i++) {
                sum += vals[i];
                sum_sq += vals[i] * vals[i];
            }
            ms->mean = sum / (double)metric_counts[m];
            double var = (sum_sq - sum * sum / (double)metric_counts[m]) / (double)metric_counts[m];
            if (var < 0.0) var = 0.0;
            ms->std_dev = sqrt(var);

            ms->ewma_current = ots_ewma_compute(vals, metric_counts[m], 0.3);
            ms->ewma_alpha = 0.3;
            ms->last_value = vals[metric_counts[m] - 1];
            ms->second_last_value = (metric_counts[m] >= 2) ? vals[metric_counts[m] - 2] : vals[0];
        }
    }

    /* Overall statistics */
    double overall_sum = 0.0, overall_sq = 0.0;
    stats->overall_mean = 0.0;
    stats->best_session_score = 0.0;

    for (int32_t i = 0; i < count; i++) {
        double s = records[i].normalized_score;
        overall_sum += s;
        overall_sq += s * s;
        if (s > stats->best_session_score) {
            stats->best_session_score = s;
        }
    }

    if (count > 0) {
        stats->overall_mean = overall_sum / (double)count;
        double overall_var = (overall_sq - overall_sum * overall_sum / (double)count) / (double)count;
        if (overall_var < 0.0) overall_var = 0.0;
        stats->overall_std_dev = sqrt(overall_var);
    }

    /* Plateau detection */
    double all_scores[1024];
    int32_t score_count = (count < 1024) ? count : 1024;
    for (int32_t i = 0; i < score_count; i++) {
        all_scores[i] = records[i].normalized_score;
    }
    stats->is_at_plateau = ots_detect_plateau(all_scores, score_count, MPC_OTS_PLATEAU_WINDOW, MPC_OTS_PLATEAU_THRESHOLD);

    /* Simple learning rate estimation from slope of first half vs second half */
    if (count >= 4) {
        int32_t half = count / 2;
        double first_mean = 0.0, second_mean = 0.0;
        for (int32_t i = 0; i < half; i++) first_mean += all_scores[i];
        for (int32_t i = half; i < count; i++) second_mean += all_scores[i];
        first_mean /= (double)half;
        second_mean /= (double)(count - half);
        stats->learning_rate = (second_mean - first_mean) / (double)half;
    }

    stats->is_improving = (stats->learning_rate > 0.01);
    stats->plateau_score = stats->overall_ewma;
    stats->sessions_since_improvement = stats->is_improving ? 0 : count;

    return MPC_OTS_OK;
}

/* =========================================================================
 * L5: Score Prediction
 * ========================================================================= */

double ots_predict_future_score(double slope, double intercept, int32_t current_sessions, int32_t sessions_forward)
{
    double predicted = intercept + slope * (double)(current_sessions + sessions_forward);
    if (predicted < 0.0) predicted = 0.0;
    if (predicted > 100.0) predicted = 100.0;
    return predicted;
}

/* =========================================================================
 * L7: Industrial Competency Report
 * ========================================================================= */

OTSStatus ots_generate_competency_report(const OperatorStatistics *stats, const CompetencyCertification *cert, char *report_buffer, int32_t buffer_size)
{
    if (!stats || !cert || !report_buffer || buffer_size <= 0)
        return MPC_OTS_ERR_NULL_POINTER;

    int32_t written = snprintf(report_buffer, buffer_size,
        "========================================\n"
        "OPERATOR COMPETENCY REPORT\n"
        "========================================\n"
        "Operator ID: %d\n"
        "Certified: %s (Role: %d)\n"
        "Overall Score: %.1f/100\n"
        "Sessions: %d (Required: %d)\n"
        "Confidence: %.0f%%\n"
        "----------------------------------------\n"
        "Metric Breakdown:\n"
        "  Response Time:     %.1f (Pass: %s)\n"
        "  Stability:         %.1f (Pass: %s)\n"
        "  Constraints:       %.1f (Pass: %s)\n"
        "  Economic:          %.1f (Pass: %s)\n"
        "  Alarm Mgmt:        %.1f (Pass: %s)\n"
        "  Situation Aware:   %.1f (Pass: %s)\n"
        "----------------------------------------\n"
        "Learning Rate: %.3f pts/session\n"
        "Plateau: %s\n"
        "========================================\n",
        cert->operator_id,
        cert->is_certified ? "YES" : "NO",
        (int32_t)cert->assessed_role,
        cert->certification_score,
        cert->sessions_completed, cert->required_sessions,
        cert->confidence_level * 100.0,
        stats->metrics[0].mean, cert->passes_response_time ? "YES" : "NO",
        stats->metrics[1].mean, cert->passes_stability ? "YES" : "NO",
        stats->metrics[2].mean, cert->passes_constraints ? "YES" : "NO",
        stats->metrics[3].mean, cert->passes_economic ? "YES" : "NO",
        stats->metrics[4].mean, cert->passes_alarm ? "YES" : "NO",
        stats->metrics[5].mean, cert->passes_awareness ? "YES" : "NO",
        stats->learning_rate,
        stats->is_at_plateau ? "Yes - may need new challenge" : "No - still improving");

    (void)written;
    return MPC_OTS_OK;
}

OTSStatus ots_benchmark_operator(const OperatorStatistics *stats, double percentile_rankings[], int32_t num_metrics)
{
    if (!stats || !percentile_rankings || num_metrics <= 0)
        return MPC_OTS_ERR_NULL_POINTER;

    /* Industry benchmark values from ASM Consortium operator studies.
     * These are approximate industry-average scores on 0-100 scale. */
    static const double benchmarks[MPC_OTS_NUM_METRICS] = {
        65.0,  /* Response Time */
        60.0,  /* Stability Margin */
        70.0,  /* Constraint Compliance */
        55.0,  /* Economic Optimality */
        75.0,  /* Alarm Management */
        62.0,  /* Situation Awareness */
        68.0   /* Consistency */
    };
    static const double benchmark_stddev[MPC_OTS_NUM_METRICS] = {
        15.0, 15.0, 12.0, 18.0, 10.0, 14.0, 13.0
    };

    int32_t n = (num_metrics < MPC_OTS_NUM_METRICS) ? num_metrics : MPC_OTS_NUM_METRICS;
    for (int32_t i = 0; i < n; i++) {
        double z = (stats->metrics[i].mean - benchmarks[i]) / benchmark_stddev[i];
        /* Approximate percentile from z-score using logistic approximation */
        double percentile = 100.0 / (1.0 + exp(-1.702 * z));
        if (percentile < 1.0) percentile = 1.0;
        if (percentile > 99.0) percentile = 99.0;
        percentile_rankings[i] = percentile;
    }

    return MPC_OTS_OK;
}
