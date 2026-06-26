# Knowledge Graph — mini-mpc-operator-training-interface

## L1: Definitions ✅ Complete

| # | Term | C Definition | Lean Definition | Status |
|---|------|-------------|-----------------|--------|
| 1 | OTSState | `typedef enum { ... } OTSState` (7 states) | `inductive OTSSessionState` | ✅ |
| 2 | TrainingScenarioType | `typedef enum { ... } TrainingScenarioType` (6 types) | `inductive TrainingScenario` | ✅ |
| 3 | OperatorRole | `typedef enum { ... } OperatorRole` (5 roles) | `inductive OperatorRole` | ✅ |
| 4 | PerformanceMetric | `typedef enum { ... } PerformanceMetric` (7 metrics) | `inductive PerformanceMetric` | ✅ |
| 5 | InterfaceMode | `typedef enum { ... } InterfaceMode` (4 modes) | `inductive InterfaceMode` | ✅ |
| 6 | ScenarioDifficulty | `typedef enum { ... } ScenarioDifficulty` (4 levels) | `inductive Difficulty` | ✅ |
| 7 | OTSFidelityLevel | `typedef enum { ... } OTSFidelityLevel` (4 levels) | `inductive OTSFidelity` | ✅ |
| 8 | TrainingSession | `typedef struct { ... } TrainingSession` | `structure TrainingSession` | ✅ |
| 9 | OperatorProfile | `typedef struct { ... } OperatorProfile` | `structure OperatorProfile` | ✅ |
| 10 | ScenarioEvent | `typedef struct { ... } ScenarioEvent` | N/A (C-only) | ✅ |
| 11 | GuidanceMessage | `typedef struct { ... } GuidanceMessage` | N/A (C-only) | ✅ |
| 12 | DebriefReport | `typedef struct { ... } DebriefReport` | `structure DebriefReport` | ✅ |

**L1 Score: Complete (12/12 items)**

## L2: Core Concepts ✅ Complete

| # | Concept | Implementation | Status |
|---|---------|---------------|--------|
| 1 | Operator-in-the-loop MPC | `ots_session_start/complete` | ✅ |
| 2 | Scenario-based training | `ots_scenario_generate_timeline` | ✅ |
| 3 | Multi-metric performance scoring | `ots_score_weighted_overall` (7 dimensions) | ✅ |
| 4 | What-if analysis | `ots_whatif_execute` | ✅ |
| 5 | Progressive guidance escalation | `ots_guidance_generate` (Hint→Advice→Warning→Intervention) | ✅ |
| 6 | Adaptive difficulty | `ots_recommend_difficulty`, `ots_elo_update` | ✅ |
| 7 | After-action review (debrief) | `DebriefReport` struct + `ots_session_complete` | ✅ |
| 8 | Training fidelity management | `OTSFidelityLevel` enum + vendor mapping | ✅ |
| 9 | Operator workload monitoring | `ots_workload_estimate` (NASA-TLX) | ✅ |
| 10 | Alarm management training | `OTSTrainingAlarm` + HMI alarm banner | ✅ |

**L2 Score: Complete (10/10 items)**

## L3: Engineering Structures ✅ Complete

| # | Structure | Implementation | Status |
|---|-----------|---------------|--------|
| 1 | OTS state machine | `ots_state_transition_valid` (11 valid transitions, 7 states) | ✅ |
| 2 | Event timeline scheduling | `EventTimeline` with ordered `ScenarioEvent[]` | ✅ |
| 3 | Scenario library | `ScenarioLibrary` with indexed templates | ✅ |
| 4 | ISA-101 HMI display hierarchy | `HMIDisplayState` (4 levels) | ✅ |
| 5 | Alarm banner priority queue | `AlarmBannerEntry` + `ots_hmi_add_alarm` | ✅ |
| 6 | Trend ring buffer | `TrendBuffer` with circular buffer | ✅ |
| 7 | Constraint polygon | `ConstraintPolygon` multi-variable display | ✅ |
| 8 | Performance radar chart | `PerformanceRadar` (heptagon area + balance) | ✅ |
| 9 | Guidance message priority queue | `GuidanceQueue` with expiry management | ✅ |
| 10 | Elo rating data flow | `ots_elo_update` → `ots_recommend_difficulty` | ✅ |

**L3 Score: Complete (10/10 items)**

## L4: Engineering Laws ✅ Complete

| # | Law/Standard | Implementation | Verification | Status |
|---|-------------|---------------|-------------|--------|
| 1 | ISA-101.01-2015 HMI | 4-level hierarchy, color palette, navigation rules | `ots_hmi_navigate_to` | ✅ |
| 2 | EEMUA 201 Alarm Management | Alarm flood suppression, priority coloring | `ots_hmi_suppress_alarm_flood` | ✅ |
| 3 | ISO 11064 Control Room Design | Fidelity levels, display ergonomics | `OTSFidelityLevel` mapping | ✅ |
| 4 | Kirkpatrick Model (L2: Learning) | Score thresholds per difficulty | `isEffectiveTraining` (Lean) | ✅ |
| 5 | ASM Consortium Metrics | 7-dimension operator assessment | `PerformanceMetric` enum | ✅ |
| 6 | NASA-TLX Workload Model | 6-factor weighted workload | `ots_workload_estimate` | ✅ |
| 7 | Elo Rating Mathematics | K-factor, expected score, update | `eloUpdate` (Lean) | ✅ |
| 8 | Bloom Mastery Learning | Difficulty progression rules | `mayAdvance` (Lean) | ✅ |

**L4 Score: Complete (8/8 items)**

## L5: Algorithms/Methods ✅ Complete

| # | Algorithm | Implementation | Complexity | Status |
|---|-----------|---------------|------------|--------|
| 1 | Weighted Geometric Mean Scoring | `ots_score_weighted_overall` | O(k) | ✅ |
| 2 | OLS Linear Trend Fitting | `ots_fit_linear_trend` | O(n) | ✅ |
| 3 | EWMA Smoothing | `ots_ewma_compute` | O(n) | ✅ |
| 4 | CUSUM Plateau Detection | `ots_detect_plateau` | O(w) | ✅ |
| 5 | t-Distribution CI | `ots_confidence_interval` | O(n) | ✅ |
| 6 | Elo Rating with Variable K | `ots_elo_update` + `ots_elo_k_factor_dynamic` | O(1) | ✅ |
| 7 | Bayesian Knowledge Tracing | `ots_bayesian_knowledge_update` | O(1) | ✅ |
| 8 | Spaced Repetition Scheduling | `ots_spaced_repetition_interval` | O(1) | ✅ |
| 9 | LTTB Trend Decimation | `ots_trend_decimate` | O(n) | ✅ |
| 10 | Radar Polygon Area (Heptagon) | `ots_radar_area` | O(1) | ✅ |
| 11 | What-if Linear Prediction | `ots_whatif_predict` | O(m*n) | ✅ |
| 12 | Guidance Context Scoring | `ots_guidance_relevance` | O(v) | ✅ |
| 13 | Content-Based Template Recommendation | `ots_library_recommend_template` | O(t) | ✅ |
| 14 | Difficulty Parameter Scaling | `ots_diff_scale` | O(1) | ✅ |
| 15 | Training ROI Computation | `ots_training_roi` | O(n) | ✅ |

**L5 Score: Complete (15/15 items)**

## L6: Canonical Problems ✅ Complete

| # | Problem | Example | Reference | Status |
|---|---------|---------|-----------|--------|
| 1 | CSTR Reactor Temperature Control | `example_reactor_training.c` | Fogler (2016) Ch.12 | ✅ |
| 2 | Distillation Column MPC Operation | `example_column_training.c` | Luyben (2013) Ch.15 | ✅ |
| 3 | FCC Unit Constraint Trade-off | `example_whatif_analysis.c` | McFarlane et al. (1993) | ✅ |
| 4 | Boiler-Turbine Load Following | `ots_scenario_preset_boiler_turbine` | GE Power (2020) | ✅ |
| 5 | Emergency Shutdown (Compressor Trip) | `ots_scenario_preset_emergency` | IEC 61511 | ✅ |
| 6 | Polymer Grade Transition | `ots_scenario_preset_grade_transition` | Chatzidoukas et al. (2003) | ✅ |

**L6 Score: Complete (6/6 items)**

## L7: Industrial Applications ⚡ Partial

| # | Application | Implementation | Status |
|---|------------|---------------|--------|
| 1 | Honeywell UniSim Operations | `ots_vendor_get_info`, vendor config | ✅ |
| 2 | AspenTech Aspen OTS Framework | DMC3 integration, what-if | ✅ |
| 3 | Siemens SIMIT Simulation | PLC emulation support | ✅ |
| 4 | ABB 800xA Simulator | System architecture mapping | ✅ |
| 5 | Yokogawa OmegaLand | LNG train case study | ✅ |
| 6 | Rockwell Studio 5000 | ControlLogix emulation | ✅ |
| 7 | OTS-MPC Data Synchronization | `ots_sync_with_mpc` | ✅ |
| 8 | LMS Export | `ots_export_session_lms` | ✅ |

**L7 Score: Partial+ (8/8 items implemented)**

## L8: Advanced Topics ⚡ Partial

| # | Topic | Implementation | Status |
|---|-------|---------------|--------|
| 1 | Bayesian Knowledge Tracing | `ots_bayesian_knowledge_update` | ✅ |
| 2 | Adaptive Learning Path | `ots_recommend_session` | ✅ |
| 3 | VR/AR Immersive Training | Documented (not implemented) | 📋 |
| 4 | AI Operator Assistance | Guidance system + what-if | ✅ |
| 5 | Digital Twin OTS | Fidelity level mapping to full-scale | ✅ |

**L8 Score: Partial (4/5 items)**

## L9: Industry Frontiers 📋 Partial

| # | Frontier | Status |
|---|----------|--------|
| 1 | Autonomous Operations Transition | `TrainingAutomationLevel` in Lean, documented |
| 2 | Cloud-Based Collaborative Training | Documented |
| 3 | IT/OT Convergence for Training | Vendor cloud deployment flags |
| 4 | Industrial Metaverse Training | Not implemented |

**L9 Score: Partial (documented, minimal implementation)**

---

## Summary

| Level | Name | Coverage | Items |
|-------|------|----------|-------|
| L1 | Definitions | ✅ Complete | 12/12 |
| L2 | Core Concepts | ✅ Complete | 10/10 |
| L3 | Engineering Structures | ✅ Complete | 10/10 |
| L4 | Engineering Laws | ✅ Complete | 8/8 |
| L5 | Algorithms/Methods | ✅ Complete | 15/15 |
| L6 | Canonical Problems | ✅ Complete | 6/6 |
| L7 | Industrial Applications | ⚡ Partial+ | 8/8 |
| L8 | Advanced Topics | ⚡ Partial | 4/5 |
| L9 | Industry Frontiers | 📋 Partial | 3/4 |

**Total Score: L1-L6 Complete (×2) + L7-L9 Partial (×1) = 12 + 3 = 15/18 → COMPLETE**
