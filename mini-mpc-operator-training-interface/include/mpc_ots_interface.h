/**
 * @file mpc_ots_interface.h
 * @brief MPC OTS Operator Interface Management — ISA-101 compliant HMI state.
 *
 * Manages the operator's visual interface state during training,
 * including HMI layout, alarm presentation, trend displays,
 * constraint visualization, and guidance overlay.
 *
 * @section L2_Core_Concepts
 * - ISA-101 hierarchical display structure (Level 1-4)
 * - Operator situation awareness via trend and radar displays
 * - Alarm rationalization and prioritization display
 * - Constraint boundary visualization (traffic light)
 * - Economic objective visualization (current vs optimal)
 * - Guidance overlay management (non-intrusive assistance)
 * - Interface mode transitions (MONITOR→GUIDE→ASSIST→AUTO)
 *
 * @section L3_Engineering_Structures
 * - HMI display state with level-based navigation
 * - Alarm banner with priority-based color coding
 * - Trend buffer for time-series visualization
 * - Constraint polygon for multi-variable limit display
 * - Radar chart for multi-metric performance visualization
 *
 * @section L4_Engineering_Laws
 * - ISA-101.01-2015: HMI hierarchy (4 levels), color palette, navigation rules
 * - EEMUA 201: Alarm display philosophy, operator workload limits
 * - ISO 11064-5: Control room displays and controls
 * - ASM Consortium: Abnormal situation management display design
 *
 * @section L5_Algorithms
 * - Display state machine with validated transitions
 * - Alarm flooding management (dynamic suppression)
 * - Trend data decimation for efficient rendering
 * - Constraint proximity warning calculation
 * - Performance radar chart axis scaling
 *
 * Reference:
 *   ISA-101.01-2015, "Human Machine Interfaces for Process Automation Systems"
 *   EEMUA 201 (2013), "Process plant control desks utilising HMI"
 *   ISO 11064-5:2008, "Displays and controls"
 *   ASM Consortium (2011), "Effective Operator Display Design"
 *   Hollifield & Habibi (2011), "Alarm Management", 2nd ed.
 */

#ifndef MPC_OTS_INTERFACE_H
#define MPC_OTS_INTERFACE_H

#include "mpc_ots_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Constants
 * ========================================================================= */

#define MPC_OTS_HMI_MAX_ALARM_BANNER    16     /**< Max simultaneous alarm entries */
#define MPC_OTS_HMI_TREND_BUFFER_SIZE  1024    /**< Trend data points per variable */
#define MPC_OTS_HMI_MAX_TRENDS           16     /**< Maximum trended variables */
#define MPC_OTS_HMI_RADAR_METRICS        7     /**< Radar chart metrics */
#define MPC_OTS_HMI_DISPLAY_LEVELS       4     /**< ISA-101 hierarchy levels */
#define MPC_OTS_HMI_COLOR_PALETTE_SIZE  12     /**< ISA-101 recommended colors */

/* =========================================================================
 * L1+L2: ISA-101 Display Hierarchy
 * ========================================================================= */

/** ISA-101 display level enumeration.
 *  Level 1: Process area overview (highest level, plant-wide)
 *  Level 2: Unit operations (single process unit)
 *  Level 3: Process detail (equipment-level detail)
 *  Level 4: Diagnostics/support (maintenance, loops, trends) */
typedef enum {
    MPC_HMI_LEVEL_1_AREA      = 0,  /**< Plant area overview */
    MPC_HMI_LEVEL_2_UNIT      = 1,  /**< Unit operation display */
    MPC_HMI_LEVEL_3_DETAIL    = 2,  /**< Equipment/loop detail */
    MPC_HMI_LEVEL_4_DIAGNOSTIC = 3  /**< Diagnostic/support display */
} HMIDisplayLevel;

/** ISA-101 alarm priority colors.
 *  Based on ISA-101 recommended alarm color scheme. */
typedef enum {
    MPC_ALARM_COLOR_EMERGENCY = 0,  /**< Red — immediate danger */
    MPC_ALARM_COLOR_HIGH      = 1,  /**< Orange — serious condition */
    MPC_ALARM_COLOR_MEDIUM    = 2,  /**< Yellow — warning */
    MPC_ALARM_COLOR_LOW       = 3,  /**< Blue — advisory */
    MPC_ALARM_COLOR_DIAGNOSTIC = 4  /**< White/gray — maintenance */
} AlarmDisplayColor;

/* =========================================================================
 * L3: Interface State Structures
 * ========================================================================= */

/** ISA-101 compliant HMI display state.
 *  Manages what the operator sees at any given moment. */
typedef struct {
    HMIDisplayLevel    current_level;         /**< Current display hierarchy level */
    HMIDisplayLevel    previous_level;        /**< Previous level (for back navigation) */
    char               display_title[MPC_OTS_NAME_MAX]; /**< Current display title */
    int32_t            active_alarm_count;    /**< Currently active alarms */
    int32_t            suppressed_alarm_count; /**< Temporarily suppressed alarms */
    int32_t            shelved_alarm_count;   /**< Shelved alarms */
    bool               alarm_banner_visible;  /**< Alarm banner display state */
    int32_t            visible_trend_count;   /**< Number of trend pens visible */
    bool               constraint_view_active; /**< Constraint polygon visible */
    bool               guidance_overlay_visible; /**< Guidance message display state */
    bool               radar_chart_visible;   /**< Performance radar visible */
    bool               whatif_panel_visible;  /**< What-if panel expanded */
    InterfaceMode      current_mode;          /**< Current assistance mode */
    double             last_operator_action;  /**< Timestamp of last interaction */
    double             display_heartbeat;     /**< Display refresh timestamp */
    bool               is_navigable;          /**< Navigation permitted flag */
    bool               requires_acknowledgment; /**< Pending alarm acknowledge */
    int32_t            pending_ack_alarm_id;  /**< Alarm requiring acknowledgment */
} HMIDisplayState;

/** Trend data buffer for time-series visualization.
 *  Ring buffer containing historical process data for trend display. */
typedef struct {
    char               variable_name[MPC_OTS_NAME_MAX]; /**< Process variable name */
    char               variable_unit[16];    /**< Engineering units */
    double             data[MPC_OTS_HMI_TREND_BUFFER_SIZE]; /**< Time-series data */
    double             timestamps[MPC_OTS_HMI_TREND_BUFFER_SIZE]; /**< Time stamps */
    int32_t            head;                 /**< Write position (ring buffer index) */
    int32_t            count;                /**< Number of valid data points */
    double             y_min;                /**< Trend scale minimum */
    double             y_max;                /**< Trend scale maximum */
    double             alarm_high;           /**< High alarm limit line */
    double             alarm_low;            /**< Low alarm limit line */
    double             target_value;         /**< Setpoint line */
    double             current_value;        /**< Current/live value */
    bool               is_auto_scaling;      /**< Auto Y-axis scaling flag */
    double             sample_interval;      /**< Sampling interval (seconds) */
    bool               is_active;            /**< Trend active flag */
} TrendBuffer;

/** Constraint polygon display data.
 *  Visualizes multi-variable operating envelope for operator awareness. */
typedef struct {
    int32_t            num_variables;         /**< Number of variables in polygon */
    char               variable_names[16][MPC_OTS_NAME_MAX]; /**< Variable labels */
    double             current_values[16];    /**< Current operating point */
    double             lower_limits[16];      /**< Hard lower limits */
    double             upper_limits[16];      /**< Hard upper limits */
    double             target_values[16];     /**< Optimal targets */
    double             soft_lower[16];        /**< Soft lower limits */
    double             soft_upper[16];        /**< Soft upper limits */
    double             distance_to_violation[16]; /**< Proximity to nearest limit */
    int32_t            most_critical_axis;    /**< Closest to constraint boundary */
    double             min_margin;            /**< Smallest constraint margin */
    bool               is_infeasible;         /**< Current point infeasible flag */
    double             economic_potential;    /**< Gap to economic optimum */
} ConstraintPolygon;

/** Performance radar chart data.
 *  Multi-dimensional visualization of operator competency profile. */
typedef struct {
    PerformanceMetric  metrics[MPC_OTS_HMI_RADAR_METRICS]; /**< Displayed metrics */
    double             scores[MPC_OTS_HMI_RADAR_METRICS];  /**< Current scores */
    double             previous_scores[MPC_OTS_HMI_RADAR_METRICS]; /**< Prior scores */
    double             benchmark_scores[MPC_OTS_HMI_RADAR_METRICS]; /**< Industry benchmark */
    double             max_scale;             /**< Radar axis maximum */
    double             min_scale;             /**< Radar axis minimum */
    bool               show_benchmark;        /**< Overlay benchmark flag */
    bool               show_progress;         /**< Comparison to previous flag */
    double             overall_area;          /**< Polygon area (comprehensiveness) */
    double             balance_index;         /**< 0=perfectly balanced, 1=fully skewed */
} PerformanceRadar;

/** Alarm banner entry for display.
 *  Single alarm row in the ISA-101 alarm banner. */
typedef struct {
    int32_t            alarm_id;              /**< Alarm identifier */
    char               tag[MPC_OTS_NAME_MAX]; /**< Alarm tag */
    char               description[MPC_OTS_DESCRIPTION_MAX]; /**< Alarm description */
    AlarmDisplayColor  color;                 /**< Display color code */
    int32_t            priority;              /**< Alarm priority (0=highest) */
    double             activation_time;       /**< Time of activation */
    double             time_in_alarm;         /**< Duration in alarm state */
    bool               is_acknowledged;       /**< Operator acknowledged */
    bool               is_suppressed;         /**< Suppression active */
    bool               is_shelved;            /**< Shelved by operator */
    int32_t            suppression_reason;    /**< Reason code for suppression */
    double             current_value;         /**< Current process value */
    double             alarm_limit;           /**< Limit being violated */
    bool               is_increasing;         /**< Trend direction (for rate alarms) */
} AlarmBannerEntry;

/* =========================================================================
 * L5: HMI State Management API
 * ========================================================================= */

/**
 * Initialize HMI display state with ISA-101 defaults.
 * Starts at Level 1 (area overview) with no active alarms.
 *
 * Complexity: O(1).
 */
OTSStatus ots_hmi_init(HMIDisplayState *hmi);

/**
 * Navigate to specified display level.
 * Validates ISA-101 navigation rules (cannot skip levels).
 *
 * Complexity: O(1).
 */
OTSStatus ots_hmi_navigate_to(HMIDisplayState *hmi, HMIDisplayLevel target_level);

/**
 * Navigate back to previous display level.
 * Restores previous_level state.
 *
 * Complexity: O(1).
 */
OTSStatus ots_hmi_navigate_back(HMIDisplayState *hmi);

/**
 * Update interface assistance mode.
 * Transitions between MONITOR→GUIDE→ASSIST→AUTO with escalation rules.
 *
 * Complexity: O(1).
 */
OTSStatus ots_hmi_set_interface_mode(HMIDisplayState *hmi, InterfaceMode mode);

/**
 * Add alarm to alarm banner display.
 * Manages banner overflow, priority ordering, and color assignment.
 *
 * Complexity: O(a) where a = active alarms (insertion sort by priority).
 */
OTSStatus ots_hmi_add_alarm(HMIDisplayState *hmi, const OTSTrainingAlarm *alarm, AlarmBannerEntry *banner);

/**
 * Acknowledge an alarm and update banner state.
 * Removes acknowledged alarm from active count.
 *
 * Complexity: O(a) where a = active alarms.
 */
OTSStatus ots_hmi_acknowledge_alarm(HMIDisplayState *hmi, int32_t alarm_id);

/**
 * Suppress alarm flooding by shelving low-priority alarms.
 * Implements EEMUA 201 dynamic alarm suppression.
 * When active alarm count exceeds threshold, lower-priority alarms are
 * aggregated into a single summary indicator.
 *
 * Complexity: O(a log a) where a = active alarms (sort by priority).
 */
OTSStatus ots_hmi_suppress_alarm_flood(HMIDisplayState *hmi, int32_t max_displayed);

/* =========================================================================
 * L5: Trend Management API
 * ========================================================================= */

/**
 * Initialize trend buffer for a process variable.
 * Complexity: O(1).
 */
OTSStatus ots_trend_init(TrendBuffer *trend, const char *name, const char *unit, double sample_interval);

/**
 * Push a new data point into the trend ring buffer.
 * Handles ring buffer wraparound and auto-scaling.
 *
 * Complexity: O(1).
 */
OTSStatus ots_trend_push(TrendBuffer *trend, double timestamp, double value);

/**
 * Get the most recent N data points from trend buffer.
 * Returns up to count points in reverse chronological order.
 *
 * Complexity: O(n) where n = count requested.
 */
int32_t ots_trend_get_recent(const TrendBuffer *trend, double values[], double timestamps[], int32_t count);

/**
 * Decimate trend data for efficient rendering.
 * Uses the Douglas-Peucker-inspired largest-triangle-three-buckets algorithm
 * for visual fidelity-preserving data reduction.
 *
 * Complexity: O(n) where n = buffer count.
 */
OTSStatus ots_trend_decimate(const TrendBuffer *trend, double output_values[], double output_timestamps[], int32_t output_size, int32_t *actual_count);

/* =========================================================================
 * L5: Constraint Visualization API
 * ========================================================================= */

/**
 * Initialize constraint polygon for multi-variable display.
 * Complexity: O(v) where v = num_variables.
 */
OTSStatus ots_constraint_polygon_init(ConstraintPolygon *poly, int32_t num_variables);

/**
 * Update constraint polygon with current process values.
 * Recalculates all distance-to-violation and min_margin metrics.
 *
 * Complexity: O(v) where v = num_variables.
 */
OTSStatus ots_constraint_polygon_update(ConstraintPolygon *poly, const double current_values[]);

/**
 * Compute the most critical constraint axis (closest to violation).
 * Returns index of variable with smallest distance_to_violation.
 *
 * Complexity: O(v).
 */
int32_t ots_constraint_most_critical(const ConstraintPolygon *poly);

/* =========================================================================
 * L5: Performance Radar API
 * ========================================================================= */

/**
 * Initialize performance radar chart.
 * Sets default metrics and scale ranges.
 *
 * Complexity: O(1).
 */
OTSStatus ots_radar_init(PerformanceRadar *radar);

/**
 * Update radar chart with new operator scores.
 * Recalculates area and balance index.
 *
 * Complexity: O(m) where m = 7 metrics.
 */
OTSStatus ots_radar_update(PerformanceRadar *radar, const double scores[7]);

/**
 * Compute radar chart polygon area for comprehensiveness measure.
 * Area of irregular heptagon from 7 metric scores (polar coords).
 * Larger area = broader competency.
 *
 * Formula: Area = (1/2) * Σ(r_i * r_{i+1} * sin(2π/7))
 * where r_i are normalized scores on [0, 100].
 *
 * Complexity: O(1).
 */
double ots_radar_area(const PerformanceRadar *radar);

/**
 * Compute balance index: measures how evenly skills are distributed.
 * 0 = all skills equal (fully balanced), 1 = maximum skill disparity.
 *
 * Uses coefficient of variation normalized to [0, 1]:
 *   CI = std_dev(scores) / mean(scores)
 *   balance_index = min(CI / max_CV, 1.0) where max_CV ≈ 2.0 for 7 metrics
 *
 * Complexity: O(m) where m = 7 metrics.
 */
double ots_radar_balance(const PerformanceRadar *radar);

#ifdef __cplusplus
}
#endif

#endif /* MPC_OTS_INTERFACE_H */
