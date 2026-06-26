#ifndef MPC_CONSTRAINT_PRIORITY_H
#define MPC_CONSTRAINT_PRIORITY_H
#include "mpc_constraint_defs.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    mpc_priority_level_t level;
    const char          *label;
    const char          *description;
    bool                 is_relaxable;
    double               max_relaxation_ratio;
} mpc_priority_descriptor_t;

typedef struct {
    int      conflicting_constraint_1;
    int      conflicting_constraint_2;
    double   conflict_measure;
    bool     resolved;
    int      relaxed_constraint;
    double   compromise_slack;
} mpc_priority_conflict_t;

mpc_status_t mpc_priority_init_descriptors(mpc_priority_descriptor_t *d, int n);
const char *mpc_priority_label(mpc_priority_level_t level);
const char *mpc_priority_description(mpc_priority_level_t level);
bool mpc_priority_is_relaxable(mpc_priority_level_t level);
mpc_status_t mpc_constraint_set_priority(mpc_constraint_t *c, mpc_priority_level_t p);
mpc_status_t mpc_constraint_validate_priorities(const mpc_constraint_set_t *cs);
int mpc_constraint_audit_relaxation_policy(const mpc_constraint_set_t *cs);
mpc_status_t mpc_constraint_sort_by_priority(mpc_constraint_set_t *cs);
mpc_status_t mpc_constraint_build_priority_index(mpc_constraint_set_t *cs);
int mpc_constraint_count_at_priority(const mpc_constraint_set_t *cs, mpc_priority_level_t l);
int mpc_constraint_start_at_priority(const mpc_constraint_set_t *cs, mpc_priority_level_t l);
mpc_status_t mpc_constraint_activate_upto_priority(mpc_constraint_set_t *cs, mpc_priority_level_t max_l);
int mpc_constraint_deactivate_below_priority(mpc_constraint_set_t *cs, mpc_priority_level_t min_l);
mpc_status_t mpc_constraint_active_count_by_priority(const mpc_constraint_set_t *cs, int *counts);
mpc_status_t mpc_constraint_detect_conflicts(const mpc_constraint_set_t *cs, int *nc, mpc_priority_conflict_t *conflicts, int max_c);
mpc_status_t mpc_constraint_resolve_conflict(mpc_constraint_set_t *cs, const mpc_priority_conflict_t *cfl);
mpc_status_t mpc_constraint_inherit_priority(mpc_constraint_set_t *cs, int dep_idx, int src_idx);
int mpc_constraint_detect_inheritance_violations(const mpc_constraint_set_t *cs, int *pairs, int max_p);
mpc_status_t mpc_constraint_priority_summary(const mpc_constraint_set_t *cs, char *buf, int sz);
mpc_priority_level_t mpc_constraint_highest_violated_priority(const mpc_constraint_set_t *cs);
mpc_status_t mpc_constraint_violation_count_by_priority(const mpc_constraint_set_t *cs, int *counts);

#ifdef __cplusplus
}
#endif
#endif