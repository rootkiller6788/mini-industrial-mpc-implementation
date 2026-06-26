/**
 * @file mpc_ots_guidance.h
 * @brief Operator guidance and decision support during MPC training.
 *
 * Provides context-sensitive operator assistance, what-if analysis
 * integration, and adaptive hint delivery during training scenarios.
 *
 * @section L2_Core_Concepts
 * - Progressive guidance: Hint → Advice → Warning → Intervention
 * - Context-sensitive assistance based on scenario state
 * - What-if analysis: operator explores alternatives before acting
 * - Decision confidence scoring for operator actions
 * - Workload monitoring to prevent guidance overload
 * - Delayed guidance to encourage independent problem-solving
 * - Haptic-style non-intrusive attention directing
 *
 * @section L3_Engineering_Structures
 * - Guidance message priority queue with expiry
 * - Decision tree for diagnostic guidance
 * - What-if query processing pipeline
 * - Workload estimation model (NASA-TLX adapted)
 *
 * @section L4_Engineering_Laws
 * - ISA-101 alarm management principles applied to guidance
 * - EEMUA 201 alarm flood management for guidance messages
 * - ISO 9241-210 Human-centered design for interactive systems
 * - ASM Consortium operator support design guidelines
 *
 * @section L5_Algorithms
 * - Guidance selection: context-weighted decision tree
 * - Workload estimation: weighted sum of task demands
 * - What-if prediction: linear extrapolation with uncertainty bounds
 * - Adaptive delay: longer delays for experienced operators
 * - Guidance relevance scoring: event type × operator level × history
 * - Operator hesitation detection from interaction patterns
 *
 * Reference:
 *   Bainbridge (1983), "Ironies of automation", Automatica
 *   Endsley (1995), "Toward a theory of situation awareness"
 *   Parasuraman & Riley (1997), "Humans and automation: Use, misuse, disuse, abuse"
 *   Lee & See (2004), "Trust in automation", Human Factors
 *   Wickens (2008), "Multiple resources and mental workload", Human Factors
 */

#ifndef MPC_OTS_GUIDANCE_H
#define MPC_OTS_GUIDANCE_H

#include "mpc_ots_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Constants
 * ========================================================================= */

#define MPC_OTS_GUIDANCE_MAX_QUEUE      64     /**< Maximum queued guidance messages */
#define MPC_OTS_GUIDANCE_MAX_WHATIF     16     /**< Maximum concurrent what-if queues */
#define MPC_OTS_GUIDANCE_DELAY_MIN      5.0    /**< Minimum guidance delay (seconds) */
#define MPC_OTS_GUIDANCE_DELAY_BASE    30.0    /**< Base guidance delay */
#define MPC_OTS_GUIDANCE_DELAY_MAX    120.0    /**< Maximum guidance delay */
#define MPC_OTS_GUIDANCE_COOLDOWN      15.0    /**< Minimum time between guidance */
#define MPC_OTS_WORKLOAD_CRITICAL       0.85   /**< Workload threshold for intervention */

/* =========================================================================
 * L1+L2: Guidance Type Definitions
 * ========================================================================= */

/** Guidance escalation level.
 *  Progresses from subtle hints to explicit instruction. */
typedef enum {
    MPC_GUIDANCE_HINT          = 0,  /**< Subtle indication, requires diagnosis */
    MPC_GUIDANCE_ADVICE        = 1,  /**< Suggested course of action */
    MPC_GUIDANCE_WARNING       = 2,  /**< Impending violation or error */
    MPC_GUIDANCE_INTERVENTION  = 3   /**< Automatic corrective action */
} GuidanceLevel;

/** Guidance trigger condition.
 *  What prompted this guidance message. */
typedef enum {
    MPC_TRIGGER_CONSTRAINT_APPROACH = 0, /**< Nearing constraint boundary */
    MPC_TRIGGER_CONSTRAINT_VIOLATED  = 1, /**< Constraint already violated */
    MPC_TRIGGER_OPTIMAL_DEVIATION    = 2, /**< Drifting from economic optimum */
    MPC_TRIGGER_ALARM_OVERLOAD       = 3, /**< Too many active alarms */
    MPC_TRIGGER_OPERATOR_INACTIVITY  = 4, /**< No action for extended period */
    MPC_TRIGGER_POOR_PERFORMANCE     = 5, /**< Sub-par response to event */
    MPC_TRIGGER_WHATIF_REQUESTED     = 6, /**< Operator requested what-if */
    MPC_TRIGGER_SAFETY_CRITICAL      = 7  /**< Immediate safety threat */
} GuidanceTrigger;

/* =========================================================================
 * L3: Guidance System Structures
 * ========================================================================= */

/** Guidance message with context and delivery rules.
 *  Complete guidance unit for operator assistance. */
typedef struct {
    int32_t            message_id;            /**< Unique identifier */
    GuidanceLevel      level;                 /**< Escalation level */
    GuidanceTrigger    trigger;               /**< Trigger condition */
    int32_t            priority;              /**< Display priority (0=highest) */
    char               title[MPC_OTS_NAME_MAX]; /**< Brief title */
    char               message[MPC_OTS_GUIDANCE_TEXT_MAX]; /**< Detailed text */
    char               action[MPC_OTS_GUIDANCE_TEXT_MAX]; /**< Recommended action */
    int32_t            related_event_id;      /**< Associated scenario event */
    double             generation_time;       /**< Creation timestamp */
    double             delay_before_display;  /**< Delay before showing */
    double             expiry_time;           /**< Auto-expiry timestamp */
    bool               was_displayed;         /**< Shown to operator */
    bool               was_acted_upon;        /**< Operator followed advice */
    double             operator_response_time; /**< Time to respond after display */
    double             relevance_score;       /**< 0-1 relevance to current state */
    bool               is_expired;            /**< Expired without action */
} GuidanceEntry;

/** Guidance delivery queue.
 *  Priority-sorted queue with expiry management. */
typedef struct {
    GuidanceEntry      entries[MPC_OTS_GUIDANCE_MAX_QUEUE]; /**< Message storage */
    int32_t            count;                 /**< Number of pending messages */
    int32_t            total_delivered;       /**< Cumulative messages shown */
    int32_t            total_acted;           /**< Cumulative messages acted upon */
    int32_t            total_expired;         /**< Cumulative expired messages */
    double             last_delivery_time;    /**< Timestamp of last delivery */
    double             cooldown_remaining;    /**< Time until next delivery allowed */
    int32_t            current_displayed;     /**< Currently displayed message ID */
    bool               is_suppressed;         /**< Queue suppressed flag */
} GuidanceQueue;

/** Workload estimation model.
 *  Adapted from NASA-TLX for real-time operator state assessment. */
typedef struct {
    double             mental_demand;         /**< 0-1 cognitive load */
    double             physical_demand;       /**< 0-1 physical demand */
    double             temporal_demand;       /**< 0-1 time pressure */
    double             performance_self;      /**< 0-1 self-assessed performance */
    double             effort;                /**< 0-1 perceived effort */
    double             frustration;           /**< 0-1 frustration level */
    double             overall_workload;      /**< Weighted sum (0-1) */
    int32_t            active_alarms;         /**< Contributing alarm count */
    double             guidance_frequency;    /**< Guidance messages per minute */
    double             action_frequency;      /**< Operator actions per minute */
    double             time_since_last_action; /**< Seconds since last action */
    bool               is_overloaded;         /**< Workload critical flag */
    double             last_update_time;      /**< Last estimation timestamp */
} WorkloadEstimate;

/** Decision support context.
 *  Bundles current process state for what-if and guidance decisions. */
typedef struct {
    double             current_values[16];    /**< Current process values */
    double             lower_limits[16];      /**< Lower bounds */
    double             upper_limits[16];      /**< Upper bounds */
    double             optimal_targets[16];   /**< Economic optimum */
    double             constraint_margins[16]; /**< Distance to limits */
    int32_t            num_variables;         /**< Number of process variables */
    ScenarioDifficulty difficulty;            /**< Current difficulty */
    OperatorRole        operator_level;        /**< Current operator experience */
    double             elapsed_scenario_time; /**< Time in scenario */
    int32_t            active_event_count;    /**< Currently active events */
    double             current_score;         /**< Real-time score */
} DecisionContext;

/** Operator decision record.
 *  Logs operator's action choice for debriefing and learning analytics. */
typedef struct {
    int32_t            decision_id;           /**< Unique decision identifier */
    double             timestamp;             /**< Decision timestamp */
    int32_t            event_id;              /**< Related scenario event */
    char               action_description[MPC_OTS_DESCRIPTION_MAX]; /**< What operator did */
    double             action_magnitude;      /**< Magnitude of MV change */
    double             predicted_outcome;     /**< System-predicted result */
    double             actual_outcome;        /**< Actual observed result */
    double             prediction_error;      /**< absolute prediction error */
    bool               was_correct;           /**< Action was appropriate */
    bool               followed_guidance;     /**< Matched recommended action */
    double             confidence;            /**< System confidence in prediction */
    double             time_to_decide;        /**< Seconds spent deliberating */
} OperatorDecision;

/* =========================================================================
 * L5: Guidance Queue Management
 * ========================================================================= */

/**
 * Initialize guidance queue.
 * Complexity: O(1).
 */
OTSStatus ots_guidance_queue_init(GuidanceQueue *queue);

/**
 * Add guidance message to queue.
 * Inserts in priority order (lower priority number = higher importance).
 * Higher priority messages displace lower priority if queue is full.
 *
 * Complexity: O(n) where n = queue count (insertion sort).
 */
OTSStatus ots_guidance_enqueue(GuidanceQueue *queue, const GuidanceEntry *entry);

/**
 * Get next guidance message ready for display.
 * Considers: cooldown, expiry, delivery delay, and relevance.
 * Returns NULL if no message is ready for display.
 *
 * Complexity: O(n) where n = queue count.
 */
OTSStatus ots_guidance_dequeue_next(GuidanceQueue *queue, GuidanceEntry *entry);

/**
 * Mark a displayed guidance as acted upon by operator.
 * Updates queue statistics for debriefing.
 *
 * Complexity: O(1).
 */
OTSStatus ots_guidance_mark_acted(GuidanceQueue *queue, int32_t message_id);

/**
 * Expire stale guidance messages that exceeded their expiry time.
 * Called periodically by the training session timer.
 *
 * Complexity: O(n) where n = queue count.
 */
OTSStatus ots_guidance_expire_stale(GuidanceQueue *queue, double current_time);

/* =========================================================================
 * L5: Guidance Generation
 * ========================================================================= */

/**
 * Generate a context-appropriate guidance message.
 * Analyzes DecisionContext to create relevant, timely guidance.
 *
 * Strategy: select guidance based on most critical deviation from optimal,
 * adjusted for operator experience level and workload.
 *
 * Complexity: O(v) where v = number of variables in context.
 */
OTSStatus ots_guidance_generate(const DecisionContext *ctx, const WorkloadEstimate *wl, GuidanceEntry *entry);

/**
 * Compute guidance relevance score for current decision context.
 * Higher score = more relevant guidance.
 * Formula: relevance = Σ(w_i * s_i) where s_i are normalized state deviations.
 *
 * Complexity: O(v) where v = variables.
 */
double ots_guidance_relevance(const DecisionContext *ctx, const GuidanceEntry *entry);

/**
 * Determine optimal guidance delay based on operator experience and difficulty.
 * Experienced operators get longer delays to encourage independent thinking.
 * delay = base_delay * (1 + operator_level_factor) * difficulty_factor
 *
 * Complexity: O(1).
 *
 * Reference: Lee & See (2004), trust-appropriate automation timing.
 */
double ots_guidance_compute_delay(OperatorRole role, ScenarioDifficulty difficulty, double base_delay);

/* =========================================================================
 * L5: Workload Estimation
 * ========================================================================= */

/**
 * Estimate operator workload from current training state.
 * Based on NASA-TLX multi-dimensional workload model adapted for OTS.
 *
 * overall_workload = 0.25*m + 0.05*p + 0.25*t + 0.15*e + 0.10*f + 0.20*g
 * m=mental, p=physical, t=temporal, e=effort, f=frustration, g=guidance_freq
 *
 * Complexity: O(1).
 *
 * Reference: Hart & Staveland (1988), "NASA-TLX: Development of a multi-dimensional workload rating scale".
 */
OTSStatus ots_workload_estimate(const DecisionContext *ctx, WorkloadEstimate *wl);

/**
 * Check if operator workload is critically high and requires intervention.
 * Returns true if overall_workload > MPC_OTS_WORKLOAD_CRITICAL.
 *
 * Complexity: O(1).
 */
bool ots_workload_is_critical(const WorkloadEstimate *wl);

/* =========================================================================
 * L5: What-if Decision Support
 * ========================================================================= */

/**
 * Execute what-if prediction for operator-proposed MV changes.
 * Uses linearized process model approximation:
 *   ΔCV ≈ G * ΔMV  (where G is the steady-state gain matrix)
 *
 * For training purposes, a simplified diagonal-dominant gain model is used
 * to provide fast feedback without requiring a full first-principles model.
 *
 * Complexity: O(m * n) where m = CVs, n = MVs.
 *
 * Reference: Honeywell Profit Suite what-if; Seborg et al. (2016), Ch.16.
 */
OTSStatus ots_whatif_predict(const DecisionContext *ctx, const double proposed_mv[], int32_t num_mv, double predicted_cv[], double uncertainty[]);

/**
 * Evaluate economic impact of a what-if scenario.
 * Economic impact = Σ(c_i * ΔCV_i) where c_i are economic coefficients.
 *
 * Complexity: O(v).
 */
double ots_whatif_economic_impact(const double predicted_cv[], const double economic_coefficients[], int32_t num_cv);

/**
 * Log operator decision for debriefing.
 * Stores action, prediction, and actual outcome.
 *
 * Complexity: O(1).
 */
OTSStatus ots_decision_log(OperatorDecision *log, int32_t id, int32_t event_id, const char *action, double magnitude, double predicted, bool followed_guidance);

/**
 * Compute operator hesitation time from interaction timestamps.
 * Hesitation = time between event detection and first action.
 *
 * Complexity: O(1).
 */
double ots_compute_hesitation(double event_trigger_time, double first_action_time);

/**
 * Generate decision summary for debriefing.
 * Aggregates: decisions made, guidance followed (%), hesitation trend.
 *
 * Complexity: O(n) where n = decisions.
 */
OTSStatus ots_decision_summary(const OperatorDecision decisions[], int32_t count, double *guidance_follow_rate, double *mean_hesitation, double *decision_accuracy);

#ifdef __cplusplus
}
#endif

#endif /* MPC_OTS_GUIDANCE_H */
