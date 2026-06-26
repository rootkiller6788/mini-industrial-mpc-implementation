/**
 * @file mpc_ots_scenario.c
 * @brief Training scenario generation and management for MPC OTS.
 *
 * Implements scenario template creation, difficulty-scaled event generation,
 * timeline validation, scenario library management, and canonical scenario
 * presets for common industrial process units.
 *
 * Knowledge points:
 *   L1: ScenarioTemplate, EventTimeline, ScenarioLibrary struct lifecycles
 *   L2: Scenario-based training methodology, progressive difficulty
 *   L3: Event sequencing with minimum spacing constraints
 *   L4: EEMUA 201 competency assessment, ISO 11064 simulator fidelity
 *   L5: Difficulty parameter scaling, template recommendation algorithm
 *   L6: Preset scenarios: reactor, column, boiler, grade transition, emergency, tradeoff
 *   L7: Honeywell UniSim scenario builder, AspenTech OTS template framework
 *   L8: Adaptive difficulty calibration from operator Elo rating
 *
 * Reference:
 *   Honeywell (2020), "UniSim Operations — Scenario Builder Guide"
 *   AspenTech (2019), "Aspen OTS Framework — Scenario Authoring"
 *   EEMUA 201 (2013), "Process plant control desks utilising HMI"
 *   Fogler (2016), "Elements of Chemical Reaction Engineering", 6th ed.
 *   Luyben (2013), "Distillation Design and Control Using Aspen Simulation", 2nd ed.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include "../include/mpc_ots_scenario.h"

/* =========================================================================
 * L5: Difficulty Parameter Scaling
 * ========================================================================= */

double ots_diff_scale(double base_value, ScenarioDifficulty difficulty, const double factors[4])
{
    int32_t idx = (int32_t)difficulty;
    if (idx < 0) idx = 0;
    if (idx > 3) idx = 3;
    double factor = factors[idx];
    if (factor < 0.01) factor = 0.01;
    return base_value * factor;
}

double ots_diff_event_spacing(double base_spacing, ScenarioDifficulty difficulty, const double spacing_factors[4])
{
    double spacing = ots_diff_scale(base_spacing, difficulty, spacing_factors);
    if (spacing < MPC_OTS_SCENARIO_MIN_EVENT_SPACING)
        spacing = MPC_OTS_SCENARIO_MIN_EVENT_SPACING;
    if (spacing > 600.0)
        spacing = 600.0;
    return spacing;
}

double ots_diff_disturbance_magnitude(double base, ScenarioDifficulty difficulty, const double factors[4])
{
    double mag = ots_diff_scale(base, difficulty, factors);
    if (mag < 0.01) mag = 0.01;
    if (mag > 100.0 * base) mag = 100.0 * base;
    return mag;
}

double ots_diff_response_window(double base_response, ScenarioDifficulty difficulty, const double factors[4])
{
    double window = ots_diff_scale(base_response, difficulty, factors);
    if (window < 5.0) window = 5.0;
    if (window > 3600.0) window = 3600.0;
    return window;
}

/* =========================================================================
 * L5: Scenario Timeline Generation
 * ========================================================================= */

OTSStatus ots_scenario_generate_timeline(const ScenarioTemplate *tmpl, ScenarioDifficulty difficulty, EventTimeline *timeline)
{
    if (!tmpl || !timeline) return MPC_OTS_ERR_NULL_POINTER;
    if (tmpl->num_events <= 0 || tmpl->num_events > MPC_OTS_SCENARIO_MAX_EVENTS)
        return MPC_OTS_ERR_INVALID_PARAM;

    memset(timeline, 0, sizeof(EventTimeline));
    timeline->template_id = tmpl->template_id;
    timeline->difficulty = difficulty;
    timeline->num_events = tmpl->num_events;

    double event_time = 0.0;
    double spacing = ots_diff_event_spacing(tmpl->event_spacing_base, difficulty, tmpl->spacing_factor);
    double cum_diff = 0.0;

    /* Event type rotation for variety */
    static const TrainingScenarioType event_rotation[] = {
        MPC_SCENARIO_DISTURBANCE_REJECTION,
        MPC_SCENARIO_SETPOINT_CHANGE,
        MPC_SCENARIO_CONSTRAINT_VIOLATION,
        MPC_SCENARIO_OPTIMIZATION_TRADEOFF,
        MPC_SCENARIO_CONSTRAINT_VIOLATION,
        MPC_SCENARIO_SETPOINT_CHANGE
    };
    static const int32_t rotation_len = 6;

    for (int32_t i = 0; i < timeline->num_events; i++) {
        ScenarioEvent *evt = &timeline->events[i];
        TrainingScenarioType etype = (i < rotation_len) ? event_rotation[i] : event_rotation[i % rotation_len];

        /* Variable timing jitter for realism (±20%) */
        double jitter = ((double)((i * 7 + 13) % 41) - 20.0) / 100.0;
        double trigger = event_time + jitter * spacing;
        if (trigger < 0.0) trigger = 0.0;

        ots_scenario_create_event(tmpl, difficulty, i, evt);
        evt->event_type = etype;
        evt->trigger_time = trigger;

        event_time += spacing;
        cum_diff += (double)difficulty;
    }

    timeline->total_duration = event_time + spacing; /* include buffer after last event */
    timeline->cumulative_difficulty = cum_diff;
    timeline->last_event_time = event_time - spacing; /* last event trigger */
    timeline->min_operator_level = (int32_t)difficulty;
    timeline->is_valid = ots_scenario_validate_timeline(timeline);

    /* Compute expected score: harder scenarios have lower expected scores */
    timeline->expected_overall_score = ots_scenario_expected_score(timeline);

    /* Check for critical/emergency events */
    timeline->has_critical_events = false;
    for (int32_t i = 0; i < timeline->num_events; i++) {
        if (timeline->events[i].is_critical) {
            timeline->has_critical_events = true;
            break;
        }
    }

    return MPC_OTS_OK;
}

OTSStatus ots_scenario_create_event(const ScenarioTemplate *tmpl, ScenarioDifficulty difficulty, int32_t event_index, ScenarioEvent *event)
{
    if (!tmpl || !event) return MPC_OTS_ERR_NULL_POINTER;
    if (event_index < 0) return MPC_OTS_ERR_INVALID_PARAM;

    memset(event, 0, sizeof(ScenarioEvent));
    event->event_id = event_index;

    double time_scale = 60.0 + (double)event_index * 120.0;
    event->trigger_time = time_scale;
    event->ramp_duration = ots_diff_scale(10.0, difficulty, tmpl->response_factor);
    snprintf(event->description, MPC_OTS_DESCRIPTION_MAX, "Scenario event %d at difficulty %s", event_index, ots_difficulty_to_string(difficulty));
    snprintf(event->affected_variable, MPC_OTS_NAME_MAX, "CV%d", (event_index % 8) + 1);

    event->initial_value = 100.0 + (double)(event_index % 5) * 10.0;
    event->target_value = event->initial_value + 20.0;
    event->disturbance_magnitude = ots_diff_disturbance_magnitude(tmpl->disturbance_base, difficulty, tmpl->disturbance_factor);
    event->disturbance_decay_rate = 60.0;
    event->optimal_response_time = ots_diff_response_window(tmpl->response_window_base, difficulty, tmpl->response_factor);
    event->max_allowable_deviation = ots_diff_scale(tmpl->max_deviation_base, difficulty, tmpl->deviation_factor) * 2.0;
    event->economic_penalty_rate = ots_diff_scale(tmpl->economic_penalty_base, difficulty, tmpl->penalty_factor);
    event->is_critical = (difficulty >= MPC_DIFFICULTY_ADVANCED && (event_index % 4) == 0);
    event->requires_operator_action = true;
    event->guidance_hint_id = event_index;

    return MPC_OTS_OK;
}

bool ots_scenario_validate_timeline(const EventTimeline *timeline)
{
    if (!timeline) return false;
    if (timeline->num_events <= 0) return false;
    if (timeline->num_events > MPC_OTS_SCENARIO_MAX_EVENTS) return false;
    if (timeline->total_duration > MPC_OTS_SCENARIO_MAX_TOTAL_DURATION) return false;

    /* Check event ordering: each event triggers at or after previous */
    double last_trigger = -1.0;
    int32_t critical_gap = 0;

    for (int32_t i = 0; i < timeline->num_events; i++) {
        const ScenarioEvent *evt = &timeline->events[i];

        if (evt->trigger_time < last_trigger - MPC_OTS_EPS)
            return false;

        /* Minimum spacing between non-critical events */
        if (i > 0 && evt->trigger_time - last_trigger < MPC_OTS_SCENARIO_MIN_EVENT_SPACING)
            return false;

        /* Critical events must have at least 2x minimum spacing */
        if (evt->is_critical) {
            if (critical_gap > 0 && critical_gap < 2) return false;
            critical_gap = 5; /* reset spacing counter */
        } else if (critical_gap > 0) {
            critical_gap--;
        }

        last_trigger = evt->trigger_time;
    }

    /* Check total duration limits */
    if (timeline->total_duration <= 0.0) return false;

    return true;
}

double ots_scenario_expected_score(const EventTimeline *timeline)
{
    if (!timeline || timeline->num_events <= 0) return 50.0;

    /* Expected score decreases with difficulty and event count:
     * base = 95 (max achievable)
     * deduction per event = 3 * (difficulty+1)
     * minimum = 10
     */
    double base = 95.0;
    double difficulty_penalty = 3.0 * ((double)timeline->difficulty + 1.0);
    double event_penalty = difficulty_penalty * (double)timeline->num_events;
    double expected = base - event_penalty;

    if (expected < 10.0) expected = 10.0;
    if (expected > 95.0) expected = 95.0;

    return expected;
}

/* =========================================================================
 * L5: Scenario Library Management
 * ========================================================================= */

OTSStatus ots_library_init(ScenarioLibrary *lib, const char *name, const char *path)
{
    if (!lib) return MPC_OTS_ERR_NULL_POINTER;
    if (!name || !path) return MPC_OTS_ERR_NULL_POINTER;

    memset(lib, 0, sizeof(ScenarioLibrary));
    snprintf(lib->library_name, MPC_OTS_NAME_MAX, "%s", name);
    snprintf(lib->library_path, 256, "%s", path);
    lib->next_template_id = 1;
    lib->num_templates = 0;
    lib->last_updated = 0.0;

    for (int32_t i = 0; i < 6; i++) lib->templates_by_type[i] = 0;
    for (int32_t i = 0; i < 4; i++) lib->templates_by_difficulty[i] = 0;

    return MPC_OTS_OK;
}

OTSStatus ots_library_add_template(ScenarioLibrary *lib, const ScenarioTemplate *tmpl)
{
    if (!lib || !tmpl) return MPC_OTS_ERR_NULL_POINTER;
    if (lib->num_templates >= MPC_OTS_SCENARIO_MAX_TEMPLATES)
        return MPC_OTS_ERR_SESSION_FULL;

    /* Check for duplicate template ID */
    for (int32_t i = 0; i < lib->num_templates; i++) {
        if (lib->templates[i].template_id == tmpl->template_id)
            return MPC_OTS_ERR_INVALID_PARAM;
    }

    int32_t idx = lib->num_templates;
    lib->templates[idx] = *tmpl;
    if (lib->templates[idx].template_id <= 0) {
        lib->templates[idx].template_id = lib->next_template_id;
    }
    lib->num_templates++;
    lib->next_template_id++;

    if (tmpl->type >= 0 && tmpl->type < 6) {
        lib->templates_by_type[tmpl->type]++;
    }

    return MPC_OTS_OK;
}

int32_t ots_library_find_templates(const ScenarioLibrary *lib, TrainingScenarioType type, ScenarioDifficulty difficulty, int32_t result_ids[], int32_t max_results)
{
    (void)difficulty; /* reserved for future difficulty filtering */
    if (!lib || !result_ids || max_results <= 0) return 0;

    int32_t found = 0;
    for (int32_t i = 0; i < lib->num_templates && found < max_results; i++) {
        if (lib->templates[i].type == type) {
            result_ids[found++] = lib->templates[i].template_id;
        }
    }
    return found;
}

OTSStatus ots_library_recommend_template(const ScenarioLibrary *lib, const OperatorProfile *profile, TrainingScenarioType preferred_type, ScenarioTemplate *recommended)
{
    if (!lib || !profile || !recommended) return MPC_OTS_ERR_NULL_POINTER;
    if (lib->num_templates <= 0) return MPC_OTS_ERR_SCENARIO_EMPTY;

    ScenarioDifficulty target_difficulty = profile->recommended_difficulty;
    const ScenarioTemplate *best = NULL;
    double best_score = -1.0;

    for (int32_t i = 0; i < lib->num_templates; i++) {
        const ScenarioTemplate *candidate = &lib->templates[i];
        if (!candidate->is_active) continue;

        /* Scoring: type match (+50), difficulty alignment, usage balance */
        double score = 0.0;
        if (candidate->type == preferred_type) {
            score += 50.0;
        } else if (candidate->type == MPC_SCENARIO_DISTURBANCE_REJECTION) {
            score += 30.0; /* disturbance rejection is always relevant */
        }

        /* Prefer less-used templates for variety */
        score += 50.0 / (1.0 + (double)candidate->times_used);

        /* Slight preference for validated templates */
        if (candidate->is_validated) score += 10.0;

        /* Match to the target difficulty range */
        if (candidate->num_events >= 4 + (int32_t)target_difficulty * 2
            && candidate->num_events <= 6 + (int32_t)target_difficulty * 3) {
            score += 20.0;
        }

        if (score > best_score) {
            best_score = score;
            best = candidate;
        }
    }

    if (!best) {
        /* Fallback: return first active template */
        for (int32_t i = 0; i < lib->num_templates; i++) {
            if (lib->templates[i].is_active) {
                best = &lib->templates[i];
                break;
            }
        }
    }

    if (!best) return MPC_OTS_ERR_SCENARIO_EMPTY;
    *recommended = *best;
    return MPC_OTS_OK;
}

/* =========================================================================
 * L6: Canonical Scenario Presets — CSTR Reactor
 * ========================================================================= */

static void preset_init_common(ScenarioTemplate *tmpl, const char *name, const char *desc, TrainingScenarioType type, const char *unit, int32_t num_events)
{
    memset(tmpl, 0, sizeof(ScenarioTemplate));
    snprintf(tmpl->name, MPC_OTS_NAME_MAX, "%s", name);
    snprintf(tmpl->description, MPC_OTS_DESCRIPTION_MAX, "%s", desc);
    tmpl->type = type;
    snprintf(tmpl->process_unit, MPC_OTS_NAME_MAX, "%s", unit);
    tmpl->num_events = num_events;
    tmpl->event_spacing_base = 120.0;
    tmpl->disturbance_base = 10.0;
    tmpl->response_window_base = 30.0;
    tmpl->max_deviation_base = 5.0;
    tmpl->economic_penalty_base = 100.0;

    /* Difficulty scaling factors */
    tmpl->spacing_factor[0] = 2.0;     /* BEGINNER: 2x base spacing */
    tmpl->spacing_factor[1] = 1.5;     /* INTERMEDIATE */
    tmpl->spacing_factor[2] = 1.0;     /* ADVANCED */
    tmpl->spacing_factor[3] = 0.7;     /* EXPERT: 0.7x base (tighter) */

    tmpl->disturbance_factor[0] = 0.5;
    tmpl->disturbance_factor[1] = 1.0;
    tmpl->disturbance_factor[2] = 1.5;
    tmpl->disturbance_factor[3] = 2.5;

    tmpl->response_factor[0] = 2.0;    /* 2x response time for beginners */
    tmpl->response_factor[1] = 1.3;
    tmpl->response_factor[2] = 0.9;
    tmpl->response_factor[3] = 0.6;    /* expert: only 60% of base time */

    tmpl->deviation_factor[0] = 2.0;   /* generous tolerance for beginners */
    tmpl->deviation_factor[1] = 1.3;
    tmpl->deviation_factor[2] = 0.9;
    tmpl->deviation_factor[3] = 0.5;   /* expert: tight tolerance */

    tmpl->penalty_factor[0] = 0.5;
    tmpl->penalty_factor[1] = 1.0;
    tmpl->penalty_factor[2] = 2.0;
    tmpl->penalty_factor[3] = 4.0;

    tmpl->min_interface_mode = MPC_IF_MODE_GUIDE;
    tmpl->initial_guidance_count = (num_events + 1) / 2;
    tmpl->guidance_delay_base = 30.0;
    tmpl->is_validated = true;
    tmpl->is_active = true;
    tmpl->times_used = 0;
    tmpl->average_score = 50.0;
}

OTSStatus ots_scenario_preset_reactor(ScenarioTemplate *tmpl)
{
    if (!tmpl) return MPC_OTS_ERR_NULL_POINTER;
    preset_init_common(tmpl,
        "CSTR Reactor Temperature Control",
        "Training scenario for exothermic CSTR reactor with cooling jacket. "
        "Covers cooling failure, feed disturbance, and thermal runaway prevention. "
        "Reaction: A → B, exothermic, ∆H = -200 kJ/mol. "
        "Key MVs: coolant flow rate, feed temperature. "
        "Key CVs: reactor temperature, conversion, jacket temperature.",
        MPC_SCENARIO_DISTURBANCE_REJECTION,
        "CSTR_Reactor_R101",
        8);
    tmpl->disturbance_base = 15.0;   /* °C disturbance */
    tmpl->response_window_base = 25.0; /* seconds */
    tmpl->max_deviation_base = 3.0;    /* °C tolerance */
    tmpl->economic_penalty_base = 500.0; /* $/hr for off-spec */
    snprintf(tmpl->process_flow, MPC_OTS_DESCRIPTION_MAX,
        "Feed A → Reactor (T_control via cooling jacket) → Product B. "
        "Safety: prevent thermal runaway, max temp 250°C.");
    tmpl->min_interface_mode = MPC_IF_MODE_ASSIST; /* safety-critical process */
    return MPC_OTS_OK;
}

OTSStatus ots_scenario_preset_distillation(ScenarioTemplate *tmpl)
{
    if (!tmpl) return MPC_OTS_ERR_NULL_POINTER;
    preset_init_common(tmpl,
        "Distillation Column MPC Operation",
        "Training scenario for binary distillation column with MPC control. "
        "Covers feed composition upset, reboiler duty constraint, tray flooding, "
        "and product purity control trade-off. "
        "Key MVs: reflux ratio, reboiler steam, distillate rate. "
        "Key CVs: top/bottom composition, column ∆P, tray temperatures.",
        MPC_SCENARIO_OPTIMIZATION_TRADEOFF,
        "Distillation_C101",
        10);
    tmpl->disturbance_base = 10.0;
    tmpl->response_window_base = 40.0;
    tmpl->max_deviation_base = 4.0;
    tmpl->economic_penalty_base = 1000.0;
    snprintf(tmpl->process_flow, MPC_OTS_DESCRIPTION_MAX,
        "Feed → Distillation Column (30 trays) → Overhead/Bottoms products. "
        "MPC targets: 99.5%% top purity, 98%% bottoms recovery. "
        "Constraint: max column dP 150 mbar, max reboiler duty 5 MW.");
    return MPC_OTS_OK;
}

OTSStatus ots_scenario_preset_boiler_turbine(ScenarioTemplate *tmpl)
{
    if (!tmpl) return MPC_OTS_ERR_NULL_POINTER;
    preset_init_common(tmpl,
        "Boiler-Turbine Load Following",
        "Training scenario for industrial boiler-turbine coordination. "
        "Covers sudden steam demand change, drum level shrink/swell, "
        "fuel supply disturbance, and steam pressure constraint management. "
        "Key MVs: fuel flow, feedwater flow, turbine valve position. "
        "Key CVs: drum level, steam pressure, power output, O2 in flue gas.",
        MPC_SCENARIO_SETPOINT_CHANGE,
        "Boiler_Turbine_BLR01",
        7);
    tmpl->disturbance_base = 20.0;   /* MW load change */
    tmpl->response_window_base = 20.0;
    tmpl->max_deviation_base = 5.0;
    tmpl->economic_penalty_base = 2000.0; /* high cost of steam outage */
    snprintf(tmpl->process_flow, MPC_OTS_DESCRIPTION_MAX,
        "Fuel + Air → Boiler → HP Steam → Turbine → Generator → Grid. "
        "Capacity: 100 MW. Steam: 540°C, 160 bar. "
        "Critical constraint: drum level must stay within ±50mm.");
    return MPC_OTS_OK;
}

OTSStatus ots_scenario_preset_grade_transition(ScenarioTemplate *tmpl)
{
    if (!tmpl) return MPC_OTS_ERR_NULL_POINTER;
    preset_init_common(tmpl,
        "Polymerization Grade Transition",
        "Training scenario for polymer grade change in continuous process. "
        "Covers melt index (MI) transition, catalyst feed adjustment, "
        "hydrogen/co-monomer ratio control, and viscosity management. "
        "Key MVs: H2 flow, co-monomer flow, catalyst feed rate. "
        "Key CVs: melt index, density, production rate, reactor temperature.",
        MPC_SCENARIO_GRADE_TRANSITION,
        "Polymerization_R201",
        12);
    tmpl->disturbance_base = 5.0;
    tmpl->response_window_base = 60.0; /* grade transitions are slow */
    tmpl->max_deviation_base = 3.0;
    tmpl->economic_penalty_base = 5000.0; /* off-spec polymer is very costly */
    snprintf(tmpl->process_flow, MPC_OTS_DESCRIPTION_MAX,
        "Ethylene + Co-monomer + H2 → Polymerization → Pelletizing → Product. "
        "Grade transition: MI 2.0 → MI 0.8, density 0.918 → 0.925 g/cm³. "
        "Transition time target: < 4 hours to minimize off-spec product.");
    return MPC_OTS_OK;
}

OTSStatus ots_scenario_preset_emergency(ScenarioTemplate *tmpl)
{
    if (!tmpl) return MPC_OTS_ERR_NULL_POINTER;
    preset_init_common(tmpl,
        "Emergency Shutdown — Compressor Trip",
        "Training scenario for emergency response to critical equipment failure. "
        "Covers recycle compressor trip, reactor pressure rise, quench system "
        "activation, and safe unit shutdown sequence. "
        "Key actions: initiate shutdown interlock, isolate feed, depressurize, "
        "activate emergency cooling.",
        MPC_SCENARIO_EMERGENCY_SHUTDOWN,
        "Hydrocracker_Recycle",
        5);
    tmpl->disturbance_base = 100.0;  /* severe upset */
    tmpl->response_window_base = 15.0; /* very tight */
    tmpl->max_deviation_base = 10.0;
    tmpl->economic_penalty_base = 10000.0; /* emergency shutdown is expensive */
    tmpl->min_interface_mode = MPC_IF_MODE_ASSIST;
    snprintf(tmpl->process_flow, MPC_OTS_DESCRIPTION_MAX,
        "Recycle gas compressor K-201 failure scenario. "
        "Safety: prevent overpressure, activate ESD Level 2. "
        "Critical: reactor PSV setpoint 180 bar, must stay below.");
    return MPC_OTS_OK;
}

OTSStatus ots_scenario_preset_constraint_tradeoff(ScenarioTemplate *tmpl)
{
    if (!tmpl) return MPC_OTS_ERR_NULL_POINTER;
    preset_init_common(tmpl,
        "FCC Unit Constraint Trade-off",
        "Training scenario for fluid catalytic cracking unit with multiple "
        "competing CV constraints. Covers wet gas compressor limitation, "
        "regenerator temperature ceiling, and catalyst circulation optimization. "
        "Key MVs: feed rate, riser temperature, catalyst circulation. "
        "Key CVs: conversion, regenerator temp, WGC suction pressure, LPG yield.",
        MPC_SCENARIO_CONSTRAINT_VIOLATION,
        "FCC_Unit_F101",
        8);
    tmpl->disturbance_base = 8.0;
    tmpl->response_window_base = 35.0;
    tmpl->max_deviation_base = 4.0;
    tmpl->economic_penalty_base = 3000.0;
    snprintf(tmpl->process_flow, MPC_OTS_DESCRIPTION_MAX,
        "Feed + Catalyst → Riser → Reactor → Regenerator → Products. "
        "Active constraints: WGC suction P > 1.5 barg, Regenerator T < 750°C, "
        "Air blower max 120 kNm3/h, Wet gas compressor max 95%% speed.");
    return MPC_OTS_OK;
}
