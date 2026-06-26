/**
 * @file mpc_ots_scenario.h
 * @brief Training scenario generation and management for MPC OTS.
 *
 * Handles scenario creation, event timeline scheduling, difficulty
 * progression, and scenario library management.
 *
 * @section L2_Core_Concepts
 * - Scenario-based training: each scenario represents a realistic process event
 * - Event timeline scheduling: events triggered by time or process conditions
 * - Difficulty-dependent parameter scaling: harder = faster, tighter constraints
 * - Scenario library: reusable, validated training scenarios
 * - Progressive disclosure: scenario details revealed based on difficulty
 *
 * @section L3_Engineering_Structures
 * - ScenarioTemplate: parameterized scenario definition with difficulty variants
 * - EventTimeline: ordered sequence of ScenarioEvents with trigger logic
 * - ScenarioLibrary: indexed collection of templates for reuse
 *
 * @section L4_Engineering_Laws
 * - Training scenario design follows EEMUA 201 competency assessment guidelines
 * - Scenario difficulty calibrated to ISA-101 operator workload standards
 * - Performance criteria based on ASM Consortium human factors research
 *
 * @section L5_Algorithms
 * - Scenario generation with configurable difficulty scaling
 * - Event sequencing with minimum spacing constraints
 * - Difficulty auto-calibration from operator profile
 * - Scenario validation against process safety constraints
 *
 * Reference:
 *   EEMUA 201 (2013), "Process plant control desks utilising HMI"
 *   ASM Consortium (2011), "Effective Operator Display Design"
 *   ISO 11064, "Ergonomic design of control centres"
 *   Honeywell (2020), "UniSim Operations — Scenario Builder Guide"
 *   AspenTech (2019), "Aspen OTS Framework — Scenario Authoring"
 */

#ifndef MPC_OTS_SCENARIO_H
#define MPC_OTS_SCENARIO_H

#include "mpc_ots_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Constants
 * ========================================================================= */

#define MPC_OTS_SCENARIO_MAX_EVENTS      64
#define MPC_OTS_SCENARIO_MAX_TEMPLATES  256
#define MPC_OTS_SCENARIO_MIN_EVENT_SPACING 30.0  /* seconds */
#define MPC_OTS_SCENARIO_MAX_TOTAL_DURATION 7200.0 /* seconds */
#define MPC_OTS_SCENARIO_DEFAULT_DURATION  1800.0

/* =========================================================================
 * L1+L3: Scenario Template Structures
 * ========================================================================= */

/** Parameterized scenario template.
 *  A configurable blueprint for generating training scenarios at different
 *  difficulty levels. Maps to industrial scenario builder concepts. */
typedef struct {
    int32_t           template_id;           /**< Unique template identifier */
    char              name[MPC_OTS_NAME_MAX]; /**< Template name */
    char              description[MPC_OTS_DESCRIPTION_MAX]; /**< Description */
    TrainingScenarioType type;               /**< Underlying scenario category */
    char              process_unit[MPC_OTS_NAME_MAX]; /**< Target process unit */
    char              process_flow[MPC_OTS_DESCRIPTION_MAX]; /**< Process description */

    /* Difficulty-scaled parameters */
    int32_t           num_events;            /**< Total events in scenario */
    double            event_spacing_base;    /**< Base inter-event spacing (seconds) */
    double            disturbance_base;      /**< Base disturbance magnitude */
    double            response_window_base;  /**< Base response time window (seconds) */
    double            max_deviation_base;    /**< Base max allowable deviation */
    double            economic_penalty_base; /**< Base economic penalty rate */

    /* Difficulty scaling factors (multipliers for each level) */
    double            spacing_factor[4];     /**< By difficulty: tighter spacing */
    double            disturbance_factor[4]; /**< By difficulty: larger disturbances */
    double            response_factor[4];    /**< By difficulty: less time */
    double            deviation_factor[4];   /**< By difficulty: tighter tolerance */
    double            penalty_factor[4];     /**< By difficulty: higher penalty */

    /* Guidance configuration */
    InterfaceMode     min_interface_mode;    /**< Minimum assistance mode */
    int32_t           initial_guidance_count; /**< Number of initial guidance messages */
    double            guidance_delay_base;   /**< Base delay before guidance appears */

    /* Metadata */
    char              author[MPC_OTS_NAME_MAX]; /**< Scenario author */
    double            creation_timestamp;    /**< Creation time */
    double            last_modified;         /**< Last modification time */
    int32_t           times_used;            /**< Usage count */
    double            average_score;         /**< Average operator score across uses */
    bool              is_validated;          /**< Safety validation flag */
    bool              is_active;             /**< Available for use */
} ScenarioTemplate;

/** Scenario event timeline with ordered events.
 *  Defines the complete sequence of events for a training session. */
typedef struct {
    int32_t           timeline_id;           /**< Unique timeline identifier */
    int32_t           template_id;           /**< Source template */
    ScenarioDifficulty difficulty;           /**< Difficulty level used */
    ScenarioEvent     events[MPC_OTS_SCENARIO_MAX_EVENTS]; /**< Ordered event sequence */
    int32_t           num_events;            /**< Number of populated events */
    double            total_duration;        /**< Total scenario duration */
    double            cumulative_difficulty; /**< Aggregate difficulty score */
    double            expected_overall_score; /**< Calibrated expected score */
    int32_t           min_operator_level;    /**< Minimum recommended operator level */
    double            last_event_time;       /**< Timestamp of final event */
    bool              has_critical_events;   /**< Contains safety-critical events */
    bool              is_valid;              /**< Timeline validation flag */
} EventTimeline;

/** Scenario library catalog.
 *  Manages collection of available training scenarios. */
typedef struct {
    ScenarioTemplate  templates[MPC_OTS_SCENARIO_MAX_TEMPLATES]; /**< Template storage */
    int32_t           num_templates;         /**< Number of loaded templates */
    int32_t           next_template_id;      /**< Auto-increment template ID */
    char              library_name[MPC_OTS_NAME_MAX]; /**< Library name */
    char              library_path[256];     /**< Filesystem path */
    int32_t           templates_by_type[6];  /**< Count per scenario type */
    int32_t           templates_by_difficulty[4]; /**< Count per difficulty */
    double            last_updated;          /**< Library update timestamp */
} ScenarioLibrary;

/* =========================================================================
 * L5: Difficulty Parameter Calculation
 * ========================================================================= */

/**
 * Compute difficulty-scaled parameter value.
 * Returns: base_value * factor[difficulty]
 *
 * Complexity: O(1).
 *
 * Reference: Adaptive difficulty scaling from intelligent tutoring systems
 * (VanLehn 2006, "The behavior of tutoring systems").
 * Factor calibration based on Kirkpatrick training evaluation levels.
 */
double ots_diff_scale(double base_value, ScenarioDifficulty difficulty, const double factors[4]);

/**
 * Compute optimal event spacing for given difficulty.
 * Harder difficulties have shorter inter-event spacing.
 * Formula: spacing = base_spacing * spacing_factor[difficulty]
 * Constrained to: MPC_OTS_SCENARIO_MIN_EVENT_SPACING ≤ spacing ≤ 600.0
 *
 * Complexity: O(1).
 */
double ots_diff_event_spacing(double base_spacing, ScenarioDifficulty difficulty, const double spacing_factors[4]);

/**
 * Compute disturbance magnitude for difficulty.
 * Formula: disturbance = base * disturbance_factor[difficulty]
 * Constrained to: 0.01 ≤ disturbance ≤ 100.0 * base
 *
 * Complexity: O(1).
 */
double ots_diff_disturbance_magnitude(double base, ScenarioDifficulty difficulty, const double factors[4]);

/**
 * Compute response time window for difficulty.
 * Harder: shorter response window.
 * Formula: window = base_response * response_factor[difficulty]
 * Constrained to: 5.0 ≤ window ≤ 3600.0 seconds
 *
 * Complexity: O(1).
 */
double ots_diff_response_window(double base_response, ScenarioDifficulty difficulty, const double factors[4]);

/* =========================================================================
 * L5: Scenario Generation
 * ========================================================================= */

/**
 * Generate an event timeline from a template at specified difficulty.
 *
 * Creates a complete, validated sequence of ScenarioEvents with
 * difficulty-scaled parameters. Events are spaced to ensure realistic
 * operator workload.
 *
 * Complexity: O(n * m) where n = num_events, m = guidance lookup.
 *
 * Reference: Honeywell UniSim scenario builder algorithm.
 */
OTSStatus ots_scenario_generate_timeline(const ScenarioTemplate *tmpl, ScenarioDifficulty difficulty, EventTimeline *timeline);

/**
 * Create a single scenario event with difficulty-scaled parameters.
 * Populates all required fields from template defaults and difficulty factors.
 *
 * Complexity: O(1).
 */
OTSStatus ots_scenario_create_event(const ScenarioTemplate *tmpl, ScenarioDifficulty difficulty, int32_t event_index, ScenarioEvent *event);

/**
 * Validate a generated scenario timeline.
 * Checks: event ordering, spacing, duration limits, critical event spacing.
 *
 * Complexity: O(n) where n = num_events.
 *
 * Returns true if all constraints satisfied.
 */
bool ots_scenario_validate_timeline(const EventTimeline *timeline);

/**
 * Compute the expected overall score for a scenario.
 * Used for Elo rating calibration: a well-designed scenario should yield
 * predictable scores at each difficulty level.
 *
 * Complexity: O(n) where n = events, iterates over all events.
 */
double ots_scenario_expected_score(const EventTimeline *timeline);

/* =========================================================================
 * L5: Scenario Library Management
 * ========================================================================= */

/**
 * Initialize scenario library with empty state.
 * Complexity: O(1).
 */
OTSStatus ots_library_init(ScenarioLibrary *lib, const char *name, const char *path);

/**
 * Add a template to the scenario library.
 * Validates template uniqueness and increments type/difficulty counters.
 *
 * Complexity: O(1). Indexed by next_template_id.
 */
OTSStatus ots_library_add_template(ScenarioLibrary *lib, const ScenarioTemplate *tmpl);

/**
 * Find templates matching scenario type and difficulty.
 * Returns number of matches, populates result_ids array.
 *
 * Complexity: O(t) where t = total templates in library.
 */
int32_t ots_library_find_templates(const ScenarioLibrary *lib, TrainingScenarioType type, ScenarioDifficulty difficulty, int32_t result_ids[], int32_t max_results);

/**
 * Select best template for an operator based on profile and history.
 * Uses: operator Elo rating, recommended difficulty, and scenario type.
 *
 * Complexity: O(t) where t = templates in library.
 * Reference: Content-based recommendation with Elo difficulty matching.
 */
OTSStatus ots_library_recommend_template(const ScenarioLibrary *lib, const OperatorProfile *profile, TrainingScenarioType preferred_type, ScenarioTemplate *recommended);

/* =========================================================================
 * L6: Canonical Scenario Presets
 * ========================================================================= */

/**
 * Create a CSTR reactor temperature control training scenario.
 * Includes: cooling failure, feed disturbance, exotherm runaway events.
 *
 * Complexity: O(1). Creates template with 5-8 events.
 * Reference: Fogler (2016), "Elements of Chemical Reaction Engineering", Ch.12.
 */
OTSStatus ots_scenario_preset_reactor(ScenarioTemplate *tmpl);

/**
 * Create a distillation column MPC operation training scenario.
 * Includes: feed composition upset, reboiler duty constraint, flooding approach.
 *
 * Complexity: O(1). Creates template with 6-10 events.
 * Reference: Luyben (2013), "Distillation Design and Control", Ch.15.
 */
OTSStatus ots_scenario_preset_distillation(ScenarioTemplate *tmpl);

/**
 * Create a boiler/turbine load following training scenario.
 * Includes: sudden load demand change, steam pressure constraint,
 * drum level shrink/swell, fuel supply upset.
 *
 * Complexity: O(1). Creates template with 5-8 events.
 * Reference: Steam Turbine Performance, GE Power (2020).
 */
OTSStatus ots_scenario_preset_boiler_turbine(ScenarioTemplate *tmpl);

/**
 * Create a grade transition scenario for polymerization MPC.
 * Includes: recipe change, catalyst activity variation, MI target control,
 * viscosity constraint management.
 *
 * Complexity: O(1). Creates template with 7-12 events.
 * Reference: Chatzidoukas et al. (2003), "Optimal grade transition in polymerization".
 */
OTSStatus ots_scenario_preset_grade_transition(ScenarioTemplate *tmpl);

/**
 * Create an equipment failure emergency response scenario.
 * Includes: compressor trip, pump failure, instrument fault,
 * and safe shutdown procedure steps.
 *
 * Complexity: O(1). Creates template with 4-6 events.
 * Reference: IEC 61511 functional safety lifecycle.
 */
OTSStatus ots_scenario_preset_emergency(ScenarioTemplate *tmpl);

/**
 * Create a constraint trade-off optimization scenario.
 * Multiple CVs competing for limited MV authority.
 *
 * Complexity: O(1). Creates template with 6-8 events.
 * Reference: Qin & Badgwell (2003), industrial MPC survey.
 */
OTSStatus ots_scenario_preset_constraint_tradeoff(ScenarioTemplate *tmpl);

#ifdef __cplusplus
}
#endif

#endif /* MPC_OTS_SCENARIO_H */
