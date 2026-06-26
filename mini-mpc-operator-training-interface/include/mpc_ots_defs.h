/**
 * @file mpc_ots_defs.h
 * @brief Core type definitions for MPC Operator Training Simulator (OTS).
 *
 * This header defines the foundational data types for operator training
 * interface systems used in industrial Model Predictive Control environments.
 *
 * @section L1_Definitions
 * - OTSState: Training session lifecycle (INIT→READY→RUNNING→PAUSED→COMPLETED)
 * - TrainingScenarioType: 6 industrial scenario categories
 * - OperatorRole: Trainee through Trainer hierarchy
 * - PerformanceMetric: 7 dimensions of operator performance
 * - InterfaceMode: 4 levels of operator assistance
 * - ScenarioDifficulty: 4-tier difficulty classification
 * - OTSFidelityLevel: Simulation fidelity scale
 * - TrainingMode: Individual, team, supervised modes
 *
 * @section L2_Core_Concepts
 * - Operator-in-the-loop simulation for MPC training
 * - Scenario-based training with configurable difficulty
 * - Real-time performance tracking with multi-metric scoring
 * - What-if analysis for operator decision support
 * - Guidance system with hint/advice/warning escalation
 * - Adaptive difficulty based on operator performance
 * - Debriefing analytics for after-action review
 *
 * @section L3_Engineering_Structures
 * - OTS session state machine with validated transitions
 * - Training scenario timeline with event scheduling
 * - Operator performance history with statistical aggregation
 * - ISA-101 compliant interface state management
 * - Guidance message priority queue
 *
 * Reference:
 *   Honeywell (2020), "UniSim Operations Suite — Operator Training Simulator"
 *   AspenTech (2019), "Aspen OTS Framework — Technical Reference"
 *   ISA-101.01-2015, "Human Machine Interfaces for Process Automation Systems"
 *   ISO 11064-1:2000, "Ergonomic design of control centres"
 *   EEMUA 201 (2013), "Process plant control desks utilising HMI"
 *   ASM Consortium (2011), "Effective Operator Display Design"
 *   Kirkpatrick & Kirkpatrick (2006), "Evaluating Training Programs", 3rd ed.
 */

#ifndef MPC_OTS_DEFS_H
#define MPC_OTS_DEFS_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <math.h>
#include <float.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Constants
 * ========================================================================= */

#define MPC_OTS_MAX_SCENARIOS        512
#define MPC_OTS_MAX_EVENTS_PER_SCENARIO 128
#define MPC_OTS_MAX_GUIDANCE_MSG     256
#define MPC_OTS_MAX_OPERATOR_PROFILES 128
#define MPC_OTS_MAX_HISTORY_PER_OP   1024
#define MPC_OTS_MAX_ALARMS_PER_SESSION 64
#define MPC_OTS_NAME_MAX             64
#define MPC_OTS_DESCRIPTION_MAX      256
#define MPC_OTS_GUIDANCE_TEXT_MAX    512
#define MPC_OTS_MAX_WHATIF_VARIABLES 32
#define MPC_OTS_PERFORMANCE_WINDOW   3600.0   /* seconds */
#define MPC_OTS_DEFAULT_SESSION_TIMEOUT 7200.0 /* seconds */
#define MPC_OTS_EPS                  1e-12
#define MPC_OTS_LARGE                1e30

/* =========================================================================
 * L1: Enumerations — Core Type Definitions
 * ========================================================================= */

/** Training session lifecycle state.
 *  Equivalent to Honeywell UniSim session states and ISA-101 activity states. */
typedef enum {
    MPC_OTS_STATE_INIT        = 0,  /**< Session created, not yet configured */
    MPC_OTS_STATE_READY       = 1,  /**< Configured, waiting to start */
    MPC_OTS_STATE_RUNNING     = 2,  /**< Active training in progress */
    MPC_OTS_STATE_PAUSED      = 3,  /**< Temporarily suspended */
    MPC_OTS_STATE_COMPLETED   = 4,  /**< Successfully finished */
    MPC_OTS_STATE_FAILED      = 5,  /**< Terminated due to error/timeout */
    MPC_OTS_STATE_DEBRIEFING  = 6   /**< Post-session review in progress */
} OTSState;

/** Training scenario category.
 *  Mapped to common industrial MPC operational challenges. */
typedef enum {
    MPC_SCENARIO_DISTURBANCE_REJECTION = 0, /**< Feed disturbance, unmeasured upset */
    MPC_SCENARIO_SETPOINT_CHANGE       = 1, /**< Production rate or quality target change */
    MPC_SCENARIO_CONSTRAINT_VIOLATION  = 2, /**< MV or CV hitting limits */
    MPC_SCENARIO_OPTIMIZATION_TRADEOFF = 3, /**< Multiple competing objectives */
    MPC_SCENARIO_EMERGENCY_SHUTDOWN    = 4, /**< Equipment failure, trip handling */
    MPC_SCENARIO_GRADE_TRANSITION      = 5  /**< Product grade change (polymers etc.) */
} TrainingScenarioType;

/** Operator role in training system.
 *  Follows typical industrial operator progression hierarchy. */
typedef enum {
    MPC_ROLE_TRAINEE          = 0,  /**< New hire, no prior MPC experience */
    MPC_ROLE_JUNIOR_OPERATOR  = 1,  /**< Some panel experience, learning MPC */
    MPC_ROLE_SENIOR_OPERATOR  = 2,  /**< Experienced panel operator */
    MPC_ROLE_SHIFT_LEAD       = 3,  /**< Supervisory role, coordinates team */
    MPC_ROLE_TRAINER          = 4   /**< Instructor, evaluates trainees */
} OperatorRole;

/** Performance metric dimensions.
 *  Based on ASM Consortium operator performance framework. */
typedef enum {
    MPC_METRIC_RESPONSE_TIME         = 0,  /**< Time to detect and respond to event */
    MPC_METRIC_STABILITY_MARGIN      = 1,  /**< Avoidance of oscillations/overshoot */
    MPC_METRIC_CONSTRAINT_COMPLIANCE = 2,  /**< Adherence to MV/CV limits */
    MPC_METRIC_ECONOMIC_OPTIMALITY   = 3,  /**< Proximity to optimal operating point */
    MPC_METRIC_ALARM_MANAGEMENT      = 4,  /**< Appropriate alarm response */
    MPC_METRIC_SITUATION_AWARENESS   = 5,  /**< Correct diagnosis of root cause */
    MPC_METRIC_CONSISTENCY           = 6   /**< Repeatable performance across runs */
} PerformanceMetric;

/** Interface assistance mode.
 *  ISA-101 compliant interaction levels for operator support. */
typedef enum {
    MPC_IF_MODE_MONITOR  = 0,  /**< Display only, no guidance */
    MPC_IF_MODE_GUIDE    = 1,  /**< Suggest actions after delay */
    MPC_IF_MODE_ASSIST   = 2,  /**< Actively prompt during events */
    MPC_IF_MODE_AUTO     = 3   /**< Automated demonstrations */
} InterfaceMode;

/** Scenario difficulty classification.
 *  Progressive difficulty aligned with Kirkpatrick training levels. */
typedef enum {
    MPC_DIFFICULTY_BEGINNER     = 0,  /**< Single-variable, clear feedback */
    MPC_DIFFICULTY_INTERMEDIATE = 1,  /**< Multi-variable, moderate coupling */
    MPC_DIFFICULTY_ADVANCED     = 2,  /**< Strong interaction, tight constraints */
    MPC_DIFFICULTY_EXPERT       = 3   /**< Realistic plant upset, ambiguous signals */
} ScenarioDifficulty;

/** OTS simulation fidelity level.
 *  Aligned with ISO 11064 control room simulator fidelity grades. */
typedef enum {
    MPC_FIDELITY_LOW       = 0,  /**< Simplified first-order + deadtime models */
    MPC_FIDELITY_MEDIUM    = 1,  /**< Nonlinear steady-state, linear dynamics */
    MPC_FIDELITY_HIGH      = 2,  /**< First-principles dynamic models */
    MPC_FIDELITY_FULL_SCALE = 3  /**< Rigorous digital twin integration */
} OTSFidelityLevel;

/** Training session mode. */
typedef enum {
    MPC_TRAINING_MODE_INDIVIDUAL = 0, /**< Single operator training */
    MPC_TRAINING_MODE_TEAM       = 1, /**< Multi-operator coordinated */
    MPC_TRAINING_MODE_SUPERVISED = 2, /**< Trainer-guided session */
    MPC_TRAINING_MODE_SELF_PACED = 3  /**< Operator controls pace */
} TrainingMode;

/* =========================================================================
 * L1: Structures — Core Data Types
 * ========================================================================= */

/** Operator profile with training history.
 *  Maintains operator identity, role, and aggregate performance statistics. */
typedef struct {
    char     name[MPC_OTS_NAME_MAX];        /**< Operator display name */
    int32_t  operator_id;                    /**< Unique operator identifier */
    OperatorRole role;                       /**< Current role classification */
    int32_t  years_experience;              /**< Years of plant operations experience */
    int32_t  mpc_training_hours;            /**< Cumulative MPC training hours */
    int32_t  sessions_completed;            /**< Total completed training sessions */
    double   overall_score;                  /**< Weighted aggregate score 0-100 */
    double   score_response_time;            /**< Response time sub-score */
    double   score_stability;               /**< Stability sub-score */
    double   score_constraints;             /**< Constraint compliance sub-score */
    double   score_economic;                /**< Economic optimality sub-score */
    double   score_situation_awareness;     /**< Situation awareness sub-score */
    double   current_elo_rating;            /**< Elo-like adaptive difficulty rating */
    ScenarioDifficulty recommended_difficulty; /**< AI-recommended next difficulty */
    int32_t  last_session_id;               /**< ID of most recent session */
    double   last_session_timestamp;        /**< Unix timestamp of last session */
    bool     is_active;                      /**< Currently in training */
} OperatorProfile;

/** Training session configuration and runtime state.
 *  Single training exercise instance from setup through debrief. */
typedef struct {
    int32_t           session_id;            /**< Unique session identifier */
    char              session_name[MPC_OTS_NAME_MAX]; /**< Descriptive session label */
    TrainingScenarioType scenario_type;     /**< Type of training scenario */
    ScenarioDifficulty difficulty;           /**< Current difficulty level */
    TrainingMode      mode;                  /**< Training session mode */
    OTSFidelityLevel  fidelity;              /**< Simulation model fidelity */
    InterfaceMode     interface_mode;        /**< Operator assistance level */
    OTSState          state;                 /**< Current session state */
    int32_t           operator_id;           /**< Trainee operator ID */
    int32_t           trainer_id;            /**< Supervising trainer ID (0 if none) */
    double            session_start_time;    /**< Wall-clock start timestamp */
    double            session_elapsed;       /**< Elapsed session time (seconds) */
    double            session_duration;      /**< Planned session duration */
    double            scenario_clock;        /**< Simulated process time */
    int32_t           current_event_index;   /**< Index into scenario event timeline */
    int32_t           total_events;          /**< Total events in this scenario */
    int32_t           events_completed;      /**< Events successfully handled */
    int32_t           events_failed;         /**< Events mishandled */
    int32_t           guidance_messages_shown; /**< Count of guidance interactions */
    int32_t           operator_interventions; /**< Count of operator actions */
    double            current_score;         /**< Real-time performance score */
    int32_t           active_alarms;         /**< Current alarm count */
    double            cpu_utilization;       /**< OTS CPU load (0-1) */
    bool              is_paused;             /**< Pause flag for training */
    bool              requires_debrief;      /**< Flag for after-action review */
} TrainingSession;

/** Training scenario event definition.
 *  A single disturbance, setpoint change, or equipment fault in the scenario timeline. */
typedef struct {
    int32_t           event_id;              /**< Event sequence identifier */
    TrainingScenarioType event_type;         /**< Category of event */
    double            trigger_time;          /**< Scenario clock time to trigger (seconds) */
    double            ramp_duration;         /**< Ramp duration for gradual events */
    char              description[MPC_OTS_DESCRIPTION_MAX]; /**< Event description */
    char              affected_variable[MPC_OTS_NAME_MAX];  /**< Process variable name */
    double            initial_value;         /**< Pre-event steady-state value */
    double            target_value;          /**< Post-event target value */
    double            disturbance_magnitude; /**< Disturbance amplitude */
    double            disturbance_decay_rate; /**< Disturbance decay time constant */
    double            optimal_response_time; /**< Expected response time for scoring */
    double            max_allowable_deviation; /**< Maximum deviation before failure */
    double            economic_penalty_rate; /**< $/hour penalty for delayed response */
    bool              is_critical;           /**< Safety-critical event flag */
    bool              requires_operator_action; /**< True if auto-correction not possible */
    int32_t           guidance_hint_id;      /**< Associated guidance message ID */
    double            actual_response_time;  /**< Measured operator response time */
    bool              was_handled;           /**< Event successfully resolved */
    double            deviation_peak;        /**< Peak deviation during event */
} ScenarioEvent;

/** Operator performance record for a single event.
 *  Captures response metrics for debriefing and analytics. */
typedef struct {
    int32_t           event_id;              /**< Corresponding scenario event */
    PerformanceMetric metric_type;           /**< Metric being measured */
    double            measured_value;        /**< Raw measured value */
    double            expected_value;        /**< Expected/optimal value */
    double            normalized_score;      /**< 0-100 normalized score */
    double            timestamp;             /**< Scenario time of measurement */
    char              notes[MPC_OTS_DESCRIPTION_MAX]; /**< Debrief annotations */
} PerformanceRecord;

/** Operator guidance message.
 *  Context-sensitive assistance provided during training. */
typedef struct {
    int32_t           message_id;            /**< Unique message identifier */
    int32_t           priority;              /**< Display priority (0=highest) */
    char              title[MPC_OTS_NAME_MAX]; /**< Brief guidance title */
    char              body[MPC_OTS_GUIDANCE_TEXT_MAX]; /**< Detailed guidance text */
    char              suggested_action[MPC_OTS_GUIDANCE_TEXT_MAX]; /**< Recommended action */
    double            display_delay;         /**< Delay before showing (seconds) */
    double            auto_dismiss_time;     /**< Auto-hide timeout (seconds) */
    bool              requires_acknowledgment; /**< Operator must acknowledge */
    bool              is_displayed;          /**< Currently visible flag */
    bool              was_acknowledged;      /**< Operator acknowledged */
    double            time_shown;            /**< Scenario time when displayed */
    double            time_acknowledged;     /**< Scenario time when acknowledged */
} GuidanceMessage;

/** What-if analysis query from operator.
 *  Operator explores alternative control actions before committing. */
typedef struct {
    int32_t           query_id;              /**< Unique query identifier */
    double            timestamp;             /**< Query timestamp */
    int32_t           num_variables;         /**< Number of manipulated variables */
    char              variable_names[MPC_OTS_MAX_WHATIF_VARIABLES][MPC_OTS_NAME_MAX];
    double            current_values[MPC_OTS_MAX_WHATIF_VARIABLES];
    double            proposed_values[MPC_OTS_MAX_WHATIF_VARIABLES];
    double            predicted_cv_deviations[MPC_OTS_MAX_WHATIF_VARIABLES];
    double            predicted_economic_impact;
    double            predicted_constraint_violations;
    double            confidence_level;      /**< Prediction confidence (0-1) */
    bool              is_safe;               /**< Safety assessment result */
    char              recommendation[MPC_OTS_GUIDANCE_TEXT_MAX];
} WhatIfQuery;

/** Alarm event during training session.
 *  Represents a process alarm requiring operator response. */
typedef struct {
    int32_t           alarm_id;              /**< Unique alarm identifier */
    char              tag[MPC_OTS_NAME_MAX]; /**< Alarm tag name */
    char              description[MPC_OTS_DESCRIPTION_MAX]; /**< Alarm description */
    int32_t           priority;              /**< Alarm priority (EEMUA 191) */
    double            activation_time;       /**< Scenario time of activation */
    double            required_response_time; /**< EEMUA-191 max response time */
    double            actual_response_time;  /**< Measured response time */
    double            acknowledge_time;      /**< Time operator acknowledged */
    double            clear_time;            /**< Time alarm was cleared */
    bool              is_suppressed;         /**< Alarm suppression flag */
    bool              was_missed;            /**< Exceeded response time limit */
    bool              was_acknowledged;      /**< Operator acknowledged */
    char              operator_action[MPC_OTS_DESCRIPTION_MAX]; /**< Action taken */
} OTSTrainingAlarm;

/** After-action debriefing report.
 *  Comprehensive training session summary for operator improvement. */
typedef struct {
    int32_t           session_id;            /**< Associated session */
    double            overall_score;         /**< Session overall score 0-100 */
    double            scores_by_metric[7];   /**< Per-metric breakdown */
    int32_t           strengths_count;       /**< Number of identified strengths */
    char              strengths[8][MPC_OTS_DESCRIPTION_MAX]; /**< Operator strengths */
    int32_t           weaknesses_count;      /**< Number of improvement areas */
    char              weaknesses[8][MPC_OTS_DESCRIPTION_MAX]; /**< Improvement areas */
    double            response_time_avg;     /**< Mean response time */
    double            response_time_best;    /**< Best response time */
    double            response_time_worst;   /**< Worst response time */
    int32_t           alarms_missed;         /**< Count of missed alarms */
    double            economic_penalty_total; /**< Total economic penalty incurred */
    char              overall_assessment[MPC_OTS_GUIDANCE_TEXT_MAX]; /**< Narrative assessment */
    double            improvement_vs_last;   /**< Score delta from previous session */
    ScenarioDifficulty recommended_next;    /**< Recommended next difficulty */
    int32_t           recommended_focus_metric; /**< Metric needing most improvement */
} DebriefReport;

/** Training curriculum definition.
 *  Structured learning path for operator MPC competency development. */
typedef struct {
    int32_t           curriculum_id;         /**< Curriculum identifier */
    char              name[MPC_OTS_NAME_MAX]; /**< Curriculum name */
    char              description[MPC_OTS_DESCRIPTION_MAX]; /**< Description */
    int32_t           total_modules;         /**< Number of training modules */
    int32_t           modules_completed;     /**< Modules finished */
    ScenarioDifficulty starting_difficulty;  /**< Entry difficulty level */
    ScenarioDifficulty target_difficulty;    /**< Target proficiency level */
    int32_t           prerequisite_hours;    /**< Required prior training hours */
    double            passing_score;         /**< Minimum score to advance */
    bool              is_completed;          /**< Curriculum completion flag */
    double            completion_timestamp;  /**< Completion timestamp */
} TrainingCurriculum;

/* =========================================================================
 * L3: Engineering Structures — State Management Types
 * ========================================================================= */

/** OTS system configuration.
 *  Global OTS environment settings and resource limits. */
typedef struct {
    OTSFidelityLevel  default_fidelity;      /**< Default simulation fidelity */
    InterfaceMode     default_interface_mode; /**< Default assistance mode */
    double            max_session_duration;  /**< Maximum session duration (seconds) */
    double            guidance_base_delay;   /**< Base delay for guidance display */
    double            auto_fail_timeout;     /**< Timeout for auto-fail on no response */
    int32_t           max_concurrent_sessions; /**< Simultaneous session limit */
    double            score_decay_rate;      /**< Score decay for delayed response */
    double            elo_k_factor;          /**< Elo rating K-factor for adaptation */
    double            improvement_weight;    /**< Weight of improvement trend in scoring */
    bool              enable_what_if;        /**< Enable what-if analysis */
    bool              enable_auto_debrief;   /**< Auto-generate debrief reports */
    bool              log_all_interactions;  /**< Detailed logging toggle */
    char              log_directory[256];    /**< Log file path */
} OTSConfig;

/** Operator performance trend over time.
 *  Statistical analysis of operator improvement trajectory. */
typedef struct {
    int32_t           operator_id;           /**< Operator identifier */
    int32_t           num_sessions;          /**< Number of analyzed sessions */
    double            slope_response_time;   /**< Learning rate: response time */
    double            slope_stability;       /**< Learning rate: stability */
    double            slope_constraints;     /**< Learning rate: constraint compliance */
    double            slope_economic;        /**< Learning rate: economic optimality */
    double            slope_overall;         /**< Overall learning rate */
    double            r_squared;             /**< Regression fit quality */
    bool              is_improving;          /**< Positive improvement trend */
    int32_t           plateau_detected;      /**< Number of sessions since improvement */
    double            predicted_plateau_score; /**< Estimated skill ceiling */
} PerformanceTrend;

/* =========================================================================
 * L4: Status Codes
 * ========================================================================= */

/** OTS operation status codes.
 *  Comprehensive error/success reporting for all OTS API functions. */
typedef enum {
    MPC_OTS_OK                        = 0,   /**< Operation successful */
    MPC_OTS_ERR_NULL_POINTER          = -1,  /**< NULL pointer argument */
    MPC_OTS_ERR_INVALID_STATE         = -2,  /**< Invalid state transition */
    MPC_OTS_ERR_INVALID_PARAM         = -3,  /**< Out-of-range parameter */
    MPC_OTS_ERR_SESSION_NOT_FOUND     = -4,  /**< Session ID not found */
    MPC_OTS_ERR_OPERATOR_NOT_FOUND    = -5,  /**< Operator profile not found */
    MPC_OTS_ERR_EVENT_NOT_FOUND       = -6,  /**< Event ID not found */
    MPC_OTS_ERR_SESSION_FULL          = -7,  /**< Maximum concurrent sessions reached */
    MPC_OTS_ERR_SCENARIO_EMPTY        = -8,  /**< Scenario has no events */
    MPC_OTS_ERR_GUIDANCE_QUEUE_FULL   = -9,  /**< Guidance message queue overflow */
    MPC_OTS_ERR_INVALID_TRANSITION    = -10, /**< State machine transition denied */
    MPC_OTS_ERR_TIMEOUT               = -11, /**< Operation timed out */
    MPC_OTS_ERR_NOT_IMPLEMENTED       = -12, /**< Feature not available */
    MPC_OTS_ERR_SAFETY_VIOLATION      = -13, /**< Proposed action violates safety */
    MPC_OTS_ERR_IO_ERROR              = -14  /**< File/log I/O error */
} OTSStatus;

/* =========================================================================
 * L1+L2: Function Declarations — Session API
 * ========================================================================= */

/**
 * Initialize a training session with default values.
 * Complexity: O(1). Reference: Honeywell UniSim session lifecycle.
 */
OTSStatus ots_session_init(TrainingSession *session, int32_t session_id, const char *name);

/**
 * Start a training session (INIT/READY → RUNNING).
 * Complexity: O(1). Validates preconditions before starting.
 */
OTSStatus ots_session_start(TrainingSession *session);

/**
 * Pause a running training session.
 * Complexity: O(1). Freezes scenario clock, preserves state.
 */
OTSStatus ots_session_pause(TrainingSession *session);

/**
 * Resume a paused training session.
 * Complexity: O(1). Restores scenario state.
 */
OTSStatus ots_session_resume(TrainingSession *session);

/**
 * Complete a training session and trigger debrief generation.
 * Complexity: O(n) where n = events in session.
 */
OTSStatus ots_session_complete(TrainingSession *session, DebriefReport *report);

/**
 * Validate state transition logic.
 * Complexity: O(1). Ensures valid OTSState transitions.
 */
bool ots_state_transition_valid(OTSState current, OTSState next);

/* =========================================================================
 * L1+L2: Operator Profile API
 * ========================================================================= */

/**
 * Initialize operator profile with defaults.
 * Complexity: O(1).
 */
OTSStatus ots_profile_init(OperatorProfile *profile, int32_t id, const char *name, OperatorRole role);

/**
 * Update operator profile with session results.
 * Complexity: O(1). Rolling average update.
 */
OTSStatus ots_profile_update_after_session(OperatorProfile *profile, const DebriefReport *report);

/**
 * Calculate Elo rating adjustment after training session.
 * Complexity: O(1). Based on Arpad Elo's rating system adapted for training.
 */
double ots_elo_update(double current_elo, double session_score, double expected_score, double k_factor);

/**
 * Compute recommended difficulty based on performance trend.
 * Complexity: O(1). Uses Elo-derived threshold comparison.
 */
ScenarioDifficulty ots_recommend_difficulty(const OperatorProfile *profile);

/* =========================================================================
 * L1+L2: Scenario Event API
 * ========================================================================= */

/**
 * Initialize a scenario event.
 * Complexity: O(1).
 */
OTSStatus ots_event_init(ScenarioEvent *event, int32_t id, TrainingScenarioType type, double trigger_time);

/**
 * Evaluate operator response to a scenario event.
 * Complexity: O(1). Computes normalized score from response metrics.
 */
OTSStatus ots_event_evaluate_response(ScenarioEvent *event, double response_time, double peak_deviation, PerformanceRecord *record);

/* =========================================================================
 * L5: What-if Analysis API
 * ========================================================================= */

/**
 * Execute a what-if query against the current process model.
 * Complexity: O(m) where m = number of predicted CVs.
 *
 * Reference: Honeywell Profit Suite what-if capabilities;
 *            AspenTech DMC3 "what-if" mode.
 */
OTSStatus ots_whatif_execute(WhatIfQuery *query);

/**
 * Validate safety of a proposed what-if action.
 * Complexity: O(1). Checks against hard constraints only.
 */
bool ots_whatif_is_safe(const WhatIfQuery *query);

/* =========================================================================
 * L5+L6: Performance Analysis API
 * ========================================================================= */

/**
 * Compute overall session score from per-metric sub-scores.
 * Complexity: O(k) where k = number of metrics assessed.
 *
 * Uses weighted geometric mean to penalize uneven performance
 * (based on ASM Consortium multi-attribute utility framework).
 */
double ots_compute_overall_score(const double scores[], int32_t num_metrics, const double weights[]);

/**
 * Compute performance trend from history.
 * Complexity: O(n) where n = number of sessions.
 * Uses ordinary least squares regression on score trajectory.
 */
OTSStatus ots_compute_performance_trend(const PerformanceRecord history[], int32_t count, PerformanceTrend *trend);

/* =========================================================================
 * Utility Functions
 * ========================================================================= */

/** Convert OTSState enum to human-readable string. */
const char *ots_state_to_string(OTSState state);

/** Convert TrainingScenarioType enum to string. */
const char *ots_scenario_type_to_string(TrainingScenarioType type);

/** Convert ScenarioDifficulty enum to string. */
const char *ots_difficulty_to_string(ScenarioDifficulty difficulty);

/** Convert InterfaceMode enum to string. */
const char *ots_interface_mode_to_string(InterfaceMode mode);

/** Convert PerformanceMetric enum to string. */
const char *ots_metric_to_string(PerformanceMetric metric);

#ifdef __cplusplus
}
#endif

#endif /* MPC_OTS_DEFS_H */
