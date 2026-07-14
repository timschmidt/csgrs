//! Audits for prepared triangle-solid component arrangement reports.
//!
//! These checks deliberately sit outside the voxelization scheduler. The
//! scheduler produces retained evidence; this module replays the accounting
//! invariants that make that evidence consumable by downstream code. Exact
//! predicates are necessary, but the system must also preserve enough
//! structure to audit the combinatorial decision that consumed them.

use crate::triangle_prepared::PreparedTriangleSolidComponentConsensusVoxelizationReport;

/// Audit of component-level consensus arrangement evidence.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct PreparedTriangleSolidComponentConsensusAuditReport {
    /// Whether open-cell materialization paths account for every open cell.
    pub open_cell_accounting_matches: bool,
    /// Whether component paths account for every connected open component.
    pub component_accounting_matches: bool,
    /// Whether retry consensus is a subset of consensus acceptance.
    pub retry_subset_matches: bool,
    /// Whether deterministic retry directions, successful directions, failed
    /// directions, and per-cell retry ray outcomes replay exactly.
    pub retry_direction_accounting_matches: bool,
    /// Whether retry direction schedules prove full component-cell coverage
    /// for every attempted deterministic direction.
    pub retry_direction_schedule_matches: bool,
    /// Whether row candidate schedules match attempted row counts.
    pub row_candidate_schedule_matches: bool,
    /// Whether retained row-cache hits and misses account for attempted rows.
    pub row_cache_accounting_matches: bool,
    /// Whether retained row-cache hits are partitioned into certified and
    /// ambiguous exact row certificates.
    pub row_cache_replay_accounting_matches: bool,
    /// Whether component row-window scheduling is accounted for as a subset of
    /// exact candidate scheduling.
    pub row_window_accounting_matches: bool,
    /// Whether retained component-row plans account for attempted local rows
    /// without duplicate, missing, or invalid minimum-coordinate memberships.
    pub row_plan_accounting_matches: bool,
    /// Whether row candidate AABB rejections replay the row AABB rejection
    /// counter exactly.
    pub row_candidate_rejections_match: bool,
    /// Whether fallback repair cells are exactly explained by retained
    /// certified and ambiguous ray attempts.
    pub fallback_replay_accounting_matches: bool,
    /// Whether every attempted row ended in exactly one certified or ambiguous
    /// outcome.
    pub row_outcomes_match: bool,
    /// Whether no unrepaired readiness blocker remains in the accelerated
    /// component evidence.
    pub no_component_blockers: bool,
    /// Whether this report is acceptable as exact component-consensus
    /// arrangement evidence.
    pub exact_component_consensus_audit_ready: bool,
}

/// Replay component-consensus accounting before an accelerated report can be
/// treated as exact arrangement evidence.
///
/// The audit is intentionally arithmetic over retained counters, not geometry.
/// Geometry has already been decided by exact predicates in the producer; this
/// layer checks that no accelerated path silently dropped open cells,
/// double-counted component outcomes, or treated broad-phase candidate lists
/// as topology evidence.
pub fn audit_prepared_triangle_solid_component_consensus(
    report: &PreparedTriangleSolidComponentConsensusVoxelizationReport,
) -> PreparedTriangleSolidComponentConsensusAuditReport {
    let open_cell_accounting_matches = report.consensus_cells
        + report.retry_consensus_cells
        + report.exterior_cells
        + report.fallback_cells
        == report.open_cells;
    let component_accounting_matches =
        report.consensus_components + report.exterior_components + report.fallback_components
            == report.components;
    let retry_subset_matches = report.retry_consensus_components <= report.consensus_components
        && report.retry_consensus_components <= report.components
        && report.retry_consensus_cells <= report.open_cells
        && (report.retry_consensus_components > 0 || report.retry_consensus_cells == 0)
        && (report.retry_consensus_cells == 0 || report.retry_direction_attempts > 0)
        && (report.retry_consensus_cells == 0
            || report.retry_ray_attempts >= report.retry_consensus_cells);
    let retry_direction_schedule_matches = if report.retry_direction_attempts == 0 {
        report.retry_direction_component_cells == 0
            && report.retry_successful_direction_cells == 0
            && report.retry_failed_direction_cells == 0
    } else {
        report.retry_direction_component_cells == report.retry_ray_attempts
            && report.retry_direction_component_cells
                == report.retry_successful_direction_cells + report.retry_failed_direction_cells
            && report.retry_successful_direction_cells == report.retry_successful_cells
            && report.retry_successful_direction_cells == report.retry_consensus_cells
            && report.retry_certified_cells >= report.retry_successful_direction_cells
            && report.retry_failed_direction_cells
                == (report.retry_certified_cells - report.retry_successful_direction_cells)
                    + report.retry_unknown_cells
    };
    let retry_direction_accounting_matches = if report.retry_direction_attempts == 0 {
        report.retry_successful_direction_attempts == 0
            && report.retry_failed_direction_attempts == 0
            && report.retry_ray_attempts == 0
            && report.retry_certified_cells == 0
            && report.retry_successful_cells == 0
            && report.retry_unknown_cells == 0
            && report.retry_conflicting_cells == 0
            && retry_direction_schedule_matches
    } else {
        report.retry_direction_attempts
            == report.retry_successful_direction_attempts + report.retry_failed_direction_attempts
            && report.retry_successful_direction_attempts == report.retry_consensus_components
            && report.retry_successful_cells == report.retry_consensus_cells
            && report.retry_ray_attempts
                == report.retry_certified_cells + report.retry_unknown_cells
            && report.retry_conflicting_cells <= report.retry_certified_cells
            && retry_direction_schedule_matches
    };
    let attempted_rows = report.axis_sweep_rows.iter().sum::<usize>();
    let row_cache_replay_accounting_matches =
        report.row_cache_certified_hits + report.row_cache_ambiguous_hits == report.row_cache_hits;
    let row_cache_accounting_matches = if report.row_cache_lookups == 0
        && report.row_cache_hits == 0
        && report.row_cache_misses == 0
    {
        row_cache_replay_accounting_matches
    } else {
        report.row_cache_lookups == attempted_rows
            && report.row_cache_hits + report.row_cache_misses == report.row_cache_lookups
            && report.row_cache_misses == report.row_candidate_scheduled_rows
            && row_cache_replay_accounting_matches
    };
    let row_candidate_schedule_matches = report.row_candidate_scheduled_rows == 0
        || report.row_candidate_scheduled_rows + report.row_cache_hits == attempted_rows;
    let row_window_accounting_matches = report.row_window_scheduled_rows == 0
        || (report.row_window_scheduled_rows == report.row_candidate_scheduled_rows
            && report.row_window_aabb_rejections <= report.row_candidate_aabb_rejections
            && report.row_cache_broadened_misses <= report.row_cache_misses);
    let row_plan_accounting_matches = if report.row_plan_axes == 0
        && report.row_plan_rows == 0
        && report.row_plan_cell_memberships == 0
    {
        true
    } else {
        report.row_plan_rows == attempted_rows
            && report.row_plan_cell_memberships >= attempted_rows
            && report.row_plan_duplicate_memberships == 0
            && report.row_plan_missing_memberships == 0
            && report.row_plan_min_axis_violations == 0
    };
    let row_candidate_rejections_match = report.row_candidate_scheduled_rows == 0
        || report.row_candidate_aabb_rejections == report.row_ray_aabb_rejections;
    let fallback_replay_accounting_matches = if report.fallback_cells == 0 {
        report.fallback_ray_attempts == 0
            && report.certified_fallback_ray_attempts == 0
            && report.ambiguous_fallback_ray_attempts == 0
    } else {
        report.certified_fallback_ray_attempts + report.ambiguous_fallback_ray_attempts
            == report.fallback_ray_attempts
            && report.fallback_cells
                >= report.fallback_unknown_cells + report.fallback_boundary_regression_cells
            && report.certified_fallback_ray_attempts
                == report.fallback_cells
                    - report.fallback_unknown_cells
                    - report.fallback_boundary_regression_cells
    };
    let row_outcomes_match = attempted_rows
        == report.axis_certified_sweep_rows.iter().sum::<usize>()
            + report.axis_ambiguous_sweep_rows.iter().sum::<usize>();
    let no_component_blockers = report.boundary_unknown_cells == 0
        && report.fallback_unknown_cells == 0
        && report.fallback_boundary_regression_cells == 0
        && report.row_parameter_order_unknowns == 0;
    let exact_component_consensus_audit_ready = open_cell_accounting_matches
        && component_accounting_matches
        && retry_subset_matches
        && retry_direction_accounting_matches
        && retry_direction_schedule_matches
        && row_candidate_schedule_matches
        && row_cache_accounting_matches
        && row_cache_replay_accounting_matches
        && row_window_accounting_matches
        && row_plan_accounting_matches
        && row_candidate_rejections_match
        && fallback_replay_accounting_matches
        && row_outcomes_match
        && no_component_blockers
        && report.classified_cells > 0;

    PreparedTriangleSolidComponentConsensusAuditReport {
        open_cell_accounting_matches,
        component_accounting_matches,
        retry_subset_matches,
        retry_direction_accounting_matches,
        retry_direction_schedule_matches,
        row_candidate_schedule_matches,
        row_cache_accounting_matches,
        row_cache_replay_accounting_matches,
        row_window_accounting_matches,
        row_plan_accounting_matches,
        row_candidate_rejections_match,
        fallback_replay_accounting_matches,
        row_outcomes_match,
        no_component_blockers,
        exact_component_consensus_audit_ready,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn valid_report() -> PreparedTriangleSolidComponentConsensusVoxelizationReport {
        PreparedTriangleSolidComponentConsensusVoxelizationReport {
            classified_cells: 5,
            boundary_cells: 1,
            open_cells: 4,
            components: 3,
            consensus_components: 2,
            consensus_cells: 2,
            exterior_components: 1,
            exterior_cells: 1,
            retry_consensus_components: 1,
            retry_consensus_cells: 1,
            retry_direction_attempts: 1,
            retry_successful_direction_attempts: 1,
            retry_direction_component_cells: 1,
            retry_successful_direction_cells: 1,
            retry_ray_attempts: 1,
            retry_certified_cells: 1,
            retry_successful_cells: 1,
            fallback_components: 0,
            fallback_cells: 0,
            axis_sweep_rows: [1, 1, 0],
            axis_certified_sweep_rows: [1, 1, 0],
            row_plan_axes: 2,
            row_plan_rows: 2,
            row_plan_cell_memberships: 4,
            row_candidate_scheduled_rows: 2,
            row_window_scheduled_rows: 2,
            row_candidate_aabb_rejections: 7,
            row_window_aabb_rejections: 1,
            row_ray_aabb_rejections: 7,
            ..PreparedTriangleSolidComponentConsensusVoxelizationReport::default()
        }
    }

    #[test]
    fn audit_accepts_consistent_component_retry_evidence() {
        let audit = audit_prepared_triangle_solid_component_consensus(&valid_report());

        assert!(audit.open_cell_accounting_matches);
        assert!(audit.component_accounting_matches);
        assert!(audit.retry_subset_matches);
        assert!(audit.retry_direction_accounting_matches);
        assert!(audit.retry_direction_schedule_matches);
        assert!(audit.row_candidate_schedule_matches);
        assert!(audit.row_cache_accounting_matches);
        assert!(audit.row_cache_replay_accounting_matches);
        assert!(audit.row_window_accounting_matches);
        assert!(audit.row_plan_accounting_matches);
        assert!(audit.row_candidate_rejections_match);
        assert!(audit.fallback_replay_accounting_matches);
        assert!(audit.row_outcomes_match);
        assert!(audit.no_component_blockers);
        assert!(audit.exact_component_consensus_audit_ready);
    }

    #[test]
    fn audit_rejects_forged_open_cell_accounting() {
        let mut report = valid_report();
        report.open_cells += 1;

        let audit = audit_prepared_triangle_solid_component_consensus(&report);

        assert!(!audit.open_cell_accounting_matches);
        assert!(!audit.exact_component_consensus_audit_ready);
    }

    #[test]
    fn audit_rejects_retry_cells_without_retry_evidence() {
        let mut report = valid_report();
        report.retry_direction_attempts = 0;
        report.retry_ray_attempts = 0;

        let audit = audit_prepared_triangle_solid_component_consensus(&report);

        assert!(!audit.retry_subset_matches);
        assert!(!audit.exact_component_consensus_audit_ready);
    }

    #[test]
    fn audit_rejects_retry_direction_accounting_mismatch() {
        let mut report = valid_report();
        report.retry_certified_cells = 0;

        let audit = audit_prepared_triangle_solid_component_consensus(&report);

        assert!(!audit.retry_direction_accounting_matches);
        assert!(!audit.exact_component_consensus_audit_ready);
    }

    #[test]
    fn audit_rejects_retry_direction_without_component_cell_schedule() {
        let mut report = valid_report();
        report.retry_direction_component_cells = 0;
        report.retry_successful_direction_cells = 0;

        let audit = audit_prepared_triangle_solid_component_consensus(&report);

        assert!(!audit.retry_direction_schedule_matches);
        assert!(!audit.retry_direction_accounting_matches);
        assert!(!audit.exact_component_consensus_audit_ready);
    }

    #[test]
    fn audit_rejects_row_schedule_counter_mismatch() {
        let mut report = valid_report();
        report.row_candidate_scheduled_rows = 1;

        let audit = audit_prepared_triangle_solid_component_consensus(&report);

        assert!(!audit.row_candidate_schedule_matches);
        assert!(!audit.exact_component_consensus_audit_ready);
    }

    #[test]
    fn audit_accepts_retained_row_cache_hits_as_scheduled_evidence() {
        let mut report = valid_report();
        report.row_cache_lookups = 2;
        report.row_cache_hits = 1;
        report.row_cache_certified_hits = 1;
        report.row_cache_misses = 1;
        report.row_candidate_scheduled_rows = 1;
        report.row_window_scheduled_rows = 1;

        let audit = audit_prepared_triangle_solid_component_consensus(&report);

        assert!(audit.row_cache_accounting_matches);
        assert!(audit.row_cache_replay_accounting_matches);
        assert!(audit.row_candidate_schedule_matches);
        assert!(audit.exact_component_consensus_audit_ready);
    }

    #[test]
    fn audit_rejects_unpartitioned_row_cache_hit_evidence() {
        let mut report = valid_report();
        report.row_cache_lookups = 2;
        report.row_cache_hits = 1;
        report.row_cache_misses = 1;
        report.row_candidate_scheduled_rows = 1;
        report.row_window_scheduled_rows = 1;

        let audit = audit_prepared_triangle_solid_component_consensus(&report);

        assert!(!audit.row_cache_replay_accounting_matches);
        assert!(!audit.row_cache_accounting_matches);
        assert!(!audit.exact_component_consensus_audit_ready);
    }

    #[test]
    fn audit_rejects_unaccounted_row_cache_hits() {
        let mut report = valid_report();
        report.row_cache_lookups = 2;
        report.row_cache_hits = 1;
        report.row_cache_misses = 0;
        report.row_candidate_scheduled_rows = 1;

        let audit = audit_prepared_triangle_solid_component_consensus(&report);

        assert!(!audit.row_cache_accounting_matches);
        assert!(!audit.exact_component_consensus_audit_ready);
    }

    #[test]
    fn audit_rejects_window_rows_outside_candidate_schedule() {
        let mut report = valid_report();
        report.row_window_scheduled_rows = report.row_candidate_scheduled_rows + 1;

        let audit = audit_prepared_triangle_solid_component_consensus(&report);

        assert!(!audit.row_window_accounting_matches);
        assert!(!audit.exact_component_consensus_audit_ready);
    }

    #[test]
    fn audit_rejects_component_row_plan_membership_mismatch() {
        let mut report = valid_report();
        report.row_plan_duplicate_memberships = 1;

        let audit = audit_prepared_triangle_solid_component_consensus(&report);

        assert!(!audit.row_plan_accounting_matches);
        assert!(!audit.exact_component_consensus_audit_ready);
    }

    #[test]
    fn audit_accepts_exact_fallback_repair_for_unvoted_component_cells() {
        let mut report = valid_report();
        report.consensus_components = 1;
        report.consensus_cells = 1;
        report.fallback_components = 1;
        report.fallback_cells = 1;
        report.fallback_ray_attempts = 1;
        report.certified_fallback_ray_attempts = 1;
        report.unvoted_component_cells = 1;
        report.deferred_ambiguous_cells = 3;

        let audit = audit_prepared_triangle_solid_component_consensus(&report);

        assert!(audit.open_cell_accounting_matches);
        assert!(audit.component_accounting_matches);
        assert!(audit.fallback_replay_accounting_matches);
        assert!(audit.no_component_blockers);
        assert!(audit.exact_component_consensus_audit_ready);
    }

    #[test]
    fn audit_rejects_fallback_repair_without_ray_replay() {
        let mut report = valid_report();
        report.consensus_components = 1;
        report.consensus_cells = 1;
        report.fallback_components = 1;
        report.fallback_cells = 1;
        report.unvoted_component_cells = 1;

        let audit = audit_prepared_triangle_solid_component_consensus(&report);

        assert!(!audit.fallback_replay_accounting_matches);
        assert!(!audit.exact_component_consensus_audit_ready);
    }
}
