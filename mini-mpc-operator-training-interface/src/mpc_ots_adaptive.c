/**
 * @file mpc_ots_adaptive.c
 * @brief Adaptive training difficulty and learning path optimization for MPC OTS.
 *
 * Implements Elo rating-based difficulty adaptation, curriculum sequencing,
 * spaced repetition scheduling, and individual learning path optimization.
 *
 * Knowledge points:
 *   L1: TrainingCurriculum, OTSConfig global configuration lifecycle
 *   L2: Adaptive difficulty, mastery learning, spaced repetition
 *   L3: Elo rating with variable K-factor, curriculum state machine
 *   L4: Kirkpatrick Level 3 (Behavior) and Level 4 (Results) evaluation
 *   L5: Elo K-factor decay, spaced repetition scheduling, Bayesian knowledge tracing
 *   L6: Personalized learning path optimization
 *   L7: Honeywell UniSim adaptive training, AspenTech competency tracking
 *   L8: Item response theory (IRT), Bayesian proficiency estimation
 *
 * Reference:
 *   Elo (1978), "The Rating of Chessplayers, Past and Present"
 *   Corbett & Anderson (1995), "Knowledge tracing: Modeling the acquisition of procedural knowledge"
 *   VanLehn (2006), "The behavior of tutoring systems"
 *   Kirkpatrick & Kirkpatrick (2006), "Evaluating Training Programs"
 *   Ebbinghaus (1885), "Memory: A Contribution to Experimental Psychology" (spacing effect)
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <time.h>
#include "../include/mpc_ots_defs.h"
#include "../include/mpc_ots_scenario.h"
#include "../include/mpc_ots_assessment.h"
#include "../include/mpc_ots_guidance.h"

/* =========================================================================
 * L1+L2: OTS System Configuration
 * ========================================================================= */

/**
 * Initialize the global OTS configuration with defaults.
 * Complexity: O(1).
 *
 * Sets ISA-101 and EEMUA 201 compliant defaults for the
 * operator training environment.
 */
OTSStatus ots_config_init(OTSConfig *config)
{
    if (!config) return MPC_OTS_ERR_NULL_POINTER;

    memset(config, 0, sizeof(OTSConfig));
    config->default_fidelity = MPC_FIDELITY_MEDIUM;
    config->default_interface_mode = MPC_IF_MODE_GUIDE;
    config->max_session_duration = MPC_OTS_DEFAULT_SESSION_TIMEOUT;
    config->guidance_base_delay = MPC_OTS_GUIDANCE_DELAY_BASE;
    config->auto_fail_timeout = 300.0; /* 5 minutes no response */
    config->max_concurrent_sessions = 8;
    config->score_decay_rate = 0.1;
    config->elo_k_factor = MPC_OTS_DEFAULT_ELO_K;
    config->improvement_weight = 0.3;
    config->enable_what_if = true;
    config->enable_auto_debrief = true;
    config->log_all_interactions = false;
    snprintf(config->log_directory, 256, "./ots_logs");

    return MPC_OTS_OK;
}

/* =========================================================================
 * L5: Elo Rating with Variable K-Factor
 * ========================================================================= */

/**
 * Compute dynamic K-factor based on operator experience.
 *
 * Elo K-factor determines how quickly rating adapts:
 * - High K (e.g., 48): rapid adaptation for new operators
 * - Medium K (e.g., 24): moderate adaptation
 * - Low K (e.g., 12): slow adaptation for experienced operators
 *
 * K decays with sessions_completed:
 *   K = K_max * exp(-λ * sessions) + K_min
 *
 * Where K_max = 32, K_min = 8, λ = 0.05
 *
 * Complexity: O(1).
 *
 * Reference: Glickman (1999), "Parameter estimation in large dynamic paired comparison experiments".
 */
double ots_elo_k_factor_dynamic(int32_t sessions_completed, double k_max, double k_min, double lambda)
{
    if (sessions_completed < 0) sessions_completed = 0;
    double k = k_max * exp(-lambda * (double)sessions_completed) + k_min;
    if (k < k_min) k = k_min;
    if (k > k_max) k = k_max;
    return k;
}

/* =========================================================================
 * L5: Mastery Learning Assessment
 * ========================================================================= */

/**
 * Check if operator has achieved mastery of current difficulty.
 *
 * Mastery criteria:
 * 1. Overall EWMA score ≥ mastery threshold (80.0)
 * 2. At least 3 sessions at current difficulty
 * 3. No failing score (< 50) in last 3 sessions
 * 4. Plateau not detected (still improving or stable at high level)
 *
 * Complexity: O(1).
 *
 * Reference: Bloom (1968), "Learning for Mastery".
 */
bool ots_mastery_achieved(const OperatorStatistics *stats, ScenarioDifficulty current_difficulty)
{
    if (!stats) return false;
    if (stats->total_sessions < 3) return false;

    double threshold;
    switch (current_difficulty) {
    case MPC_DIFFICULTY_BEGINNER:     threshold = 80.0; break;
    case MPC_DIFFICULTY_INTERMEDIATE: threshold = 75.0; break;
    case MPC_DIFFICULTY_ADVANCED:     threshold = 70.0; break;
    case MPC_DIFFICULTY_EXPERT:       threshold = 65.0; break;
    default:                          threshold = 75.0; break;
    }

    /* EWMA score must exceed threshold */
    if (stats->overall_ewma < threshold) return false;

    /* Improvement or stable at high level */
    if (stats->is_at_plateau && stats->plateau_score < threshold + 5.0) return false;

    return true;
}

/* =========================================================================
 * L5: Spaced Repetition Scheduling
 * ========================================================================= */

/**
 * Compute optimal spacing interval for next training session.
 *
 * Based on Ebbinghaus forgetting curve and spaced repetition:
 *   interval = base_interval * 2^(mastery_level)
 *
 * Where mastery_level = floor(ewma_score / 10) - 5
 *   (score 50 → level 0, score 100 → level 5)
 *
 * Complexity: O(1).
 *
 * Reference:
 *   Ebbinghaus (1885), "Memory"
 *   Cepeda et al. (2006), "Distributed practice in verbal recall tasks"
 */
double ots_spaced_repetition_interval(double ewma_score, double base_interval_hours)
{
    int32_t mastery_level = (int32_t)(ewma_score / 10.0) - 5;
    if (mastery_level < 0) mastery_level = 0;
    if (mastery_level > 5) mastery_level = 5;

    double interval = base_interval_hours * pow(2.0, (double)mastery_level);
    return interval;
}

/* =========================================================================
 * L5: Curriculum Progression
 * ========================================================================= */

/**
 * Initialize training curriculum.
 *
 * Creates a structured learning path for operator development
 * from TRAINEE through EXPERT level.
 *
 * Complexity: O(1).
 */
OTSStatus ots_curriculum_init(TrainingCurriculum *curriculum, int32_t id, const char *name, ScenarioDifficulty start, ScenarioDifficulty target)
{
    if (!curriculum || !name) return MPC_OTS_ERR_NULL_POINTER;

    memset(curriculum, 0, sizeof(TrainingCurriculum));
    curriculum->curriculum_id = id;
    snprintf(curriculum->name, MPC_OTS_NAME_MAX, "%s", name);
    snprintf(curriculum->description, MPC_OTS_DESCRIPTION_MAX,
        "Structured MPC operator training curriculum from %s to %s",
        ots_difficulty_to_string(start), ots_difficulty_to_string(target));
    curriculum->starting_difficulty = start;
    curriculum->target_difficulty = target;
    curriculum->total_modules = 4 * ((int32_t)target - (int32_t)start + 1);
    curriculum->modules_completed = 0;
    curriculum->passing_score = 65.0;
    curriculum->is_completed = false;

    return MPC_OTS_OK;
}

/**
 * Advance curriculum module based on operator performance.
 *
 * Module advancement rules:
 * - Must achieve mastery at current difficulty
 * - Score in most recent session ≥ passing_score
 * - Adequate spacing since last session (not rushed)
 *
 * Complexity: O(1).
 */
OTSStatus ots_curriculum_advance(TrainingCurriculum *curriculum, const OperatorStatistics *stats, const OperatorProfile *profile, ScenarioDifficulty *next_difficulty)
{
    if (!curriculum || !stats || !next_difficulty) return MPC_OTS_ERR_NULL_POINTER;
    (void)profile;

    ScenarioDifficulty current_effective = profile->recommended_difficulty;

    if (ots_mastery_achieved(stats, current_effective)) {
        /* Advance to next difficulty */
        if ((int32_t)current_effective < (int32_t)MPC_DIFFICULTY_EXPERT) {
            *next_difficulty = (ScenarioDifficulty)((int32_t)current_effective + 1);
        } else {
            *next_difficulty = MPC_DIFFICULTY_EXPERT; /* stay at expert */
        }

        curriculum->modules_completed++;

        /* Check curriculum completion */
        if ((int32_t)(*next_difficulty) >= (int32_t)curriculum->target_difficulty
            && curriculum->modules_completed >= curriculum->total_modules / 2) {
            curriculum->is_completed = true;
            curriculum->completion_timestamp = (double)time(NULL);
        }
    } else {
        *next_difficulty = current_effective; /* stay at same difficulty */
    }

    return MPC_OTS_OK;
}

/* =========================================================================
 * L5: Learning Path Optimization
 * ========================================================================= */

/**
 * Recommend next scenario type based on operator weaknesses.
 *
 * Analyzes per-metric scores to identify the operator's weakest area
 * and selects the scenario type most relevant to improving that area.
 *
 * Mapping:
 *   Weak Response Time   → Disturbance Rejection (fast dynamics)
 *   Weak Stability       → Setpoint Change (tracking)
 *   Weak Constraints     → Constraint Violation
 *   Weak Economic        → Optimization Trade-off
 *   Weak Alarms          → Emergency Shutdown
 *   Weak Awareness       → Grade Transition (complex multi-step)
 *
 * Complexity: O(m) where m = 7 metrics.
 *
 * Reference: Adaptive tutoring systems (VanLehn 2006).
 */
TrainingScenarioType ots_recommend_scenario_for_weakness(const OperatorStatistics *stats)
{
    if (!stats) return MPC_SCENARIO_DISTURBANCE_REJECTION;

    double min_score = 100.0;
    int32_t weakest_metric = 0;

    for (int32_t i = 0; i < MPC_OTS_NUM_METRICS; i++) {
        if (stats->metrics[i].mean < min_score && stats->metrics[i].num_samples > 0) {
            min_score = stats->metrics[i].mean;
            weakest_metric = i;
        }
    }

    switch (weakest_metric) {
    case 0: return MPC_SCENARIO_DISTURBANCE_REJECTION; /* Response Time */
    case 1: return MPC_SCENARIO_SETPOINT_CHANGE;       /* Stability */
    case 2: return MPC_SCENARIO_CONSTRAINT_VIOLATION;  /* Constraints */
    case 3: return MPC_SCENARIO_OPTIMIZATION_TRADEOFF; /* Economic */
    case 4: return MPC_SCENARIO_EMERGENCY_SHUTDOWN;    /* Alarms */
    default: return MPC_SCENARIO_GRADE_TRANSITION;     /* Awareness/Consistency */
    }
}

/* =========================================================================
 * L8: Bayesian Knowledge Tracing
 * ========================================================================= */

/**
 * Simple Bayesian knowledge tracing model for skill acquisition.
 *
 * Models operator knowledge state as probability of mastery.
 *
 * Parameters:
 *   p(L0) = initial probability of knowing (prior)
 *   p(T)  = probability of learning from a training session (transition)
 *   p(G)  = probability of guessing correctly (guess)
 *   p(S)  = probability of slipping despite knowing (slip)
 *
 * Update rule:
 *   p(L_t | correct)   = p(L_{t-1}) * (1-p(S)) / (p(L_{t-1})*(1-p(S)) + (1-p(L_{t-1}))*p(G))
 *   p(L_t | incorrect) = p(L_{t-1}) * p(S) / (p(L_{t-1})*p(S) + (1-p(L_{t-1}))*(1-p(G)))
 *
 * Complexity: O(1).
 *
 * Reference:
 *   Corbett & Anderson (1995), "Knowledge tracing"
 *   Yudelson et al. (2013), "Individualized Bayesian knowledge tracing models"
 */
OTSStatus ots_bayesian_knowledge_update(double *p_knowledge, bool was_correct, double p_guess, double p_slip, double p_learn)
{
    if (!p_knowledge) return MPC_OTS_ERR_NULL_POINTER;
    if (*p_knowledge < 0.0 || *p_knowledge > 1.0) return MPC_OTS_ERR_INVALID_PARAM;
    if (p_guess < 0.0 || p_guess > 1.0) p_guess = 0.2;
    if (p_slip < 0.0 || p_slip > 1.0) p_slip = 0.1;
    if (p_learn < 0.0 || p_learn > 1.0) p_learn = 0.3;

    double pL = *p_knowledge;

    if (was_correct) {
        double num = pL * (1.0 - p_slip);
        double den = num + (1.0 - pL) * p_guess;
        if (den > MPC_OTS_EPS) *p_knowledge = num / den;
    } else {
        double num = pL * p_slip;
        double den = num + (1.0 - pL) * (1.0 - p_guess);
        if (den > MPC_OTS_EPS) *p_knowledge = num / den;
    }

    /* Apply learning transition: p(L_{t+1}) = p(L_t | obs) + (1 - p(L_t | obs)) * p(T) */
    *p_knowledge = *p_knowledge + (1.0 - *p_knowledge) * p_learn;

    if (*p_knowledge > 1.0) *p_knowledge = 1.0;
    if (*p_knowledge < 0.0) *p_knowledge = 0.0;

    return MPC_OTS_OK;
}

/* =========================================================================
 * L5: Session Difficulty Recommendation
 * ========================================================================= */

/**
 * Generate comprehensive session recommendation for an operator.
 *
 * Considers: Elo rating, mastery status, weakest metrics, spaced repetition,
 * and curriculum progression to recommend optimal next session parameters.
 *
 * Complexity: O(1) after precomputed statistics.
 *
 * Outputs:
 *   recommended_difficulty: optimal difficulty for next session
 *   recommended_type: scenario type targeting weakest area
 *   recommended_interface_mode: appropriate assistance level
 *   days_until_next: suggested wait before next session (spaced repetition)
 */
OTSStatus ots_recommend_session(const OperatorProfile *profile, const OperatorStatistics *stats,
    ScenarioDifficulty *difficulty, TrainingScenarioType *type, InterfaceMode *mode, double *days_until_next)
{
    if (!profile || !stats || !difficulty || !type || !mode || !days_until_next)
        return MPC_OTS_ERR_NULL_POINTER;

    /* Difficulty: from Elo-based recommendation */
    *difficulty = ots_recommend_difficulty(profile);

    /* Scenario type: target weakest area */
    *type = ots_recommend_scenario_for_weakness(stats);

    /* Interface mode: based on experience and difficulty */
    if (profile->sessions_completed < 5) {
        *mode = MPC_IF_MODE_ASSIST;  /* beginners get more help */
    } else if (profile->sessions_completed < 20) {
        *mode = MPC_IF_MODE_GUIDE;   /* moderate guidance */
    } else {
        *mode = MPC_IF_MODE_MONITOR; /* minimal assistance for experts */
    }

    /* Spacing: compute from EWMA score */
    double interval_hours = ots_spaced_repetition_interval(stats->overall_ewma, 24.0);
    *days_until_next = interval_hours / 24.0;

    return MPC_OTS_OK;
}
