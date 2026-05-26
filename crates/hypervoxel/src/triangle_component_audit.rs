//! Audits for prepared triangle-solid component arrangement reports.
//!
//! These checks deliberately sit outside the voxelization scheduler. The
//! scheduler produces retained evidence; this module replays the accounting
//! invariants that make that evidence consumable by downstream code. This is
//! Yap's "geometric-system" discipline from "Towards Exact Geometric
//! Computation," *Computational Geometry* 7(1-2), 1997: exact predicates are
//! necessary, but the system must also preserve enough structure to audit the
//! combinatorial decision that consumed them.

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
    /// Whether row candidate schedules match attempted row counts.
    pub row_candidate_schedule_matches: bool,
    /// Whether retained row-cache hits and misses account for attempted rows.
    pub row_cache_accounting_matches: bool,
    /// Whether row candidate AABB rejections replay the row AABB rejection
    /// counter exactly.
    pub row_candidate_rejections_match: bool,
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
    let attempted_rows = report.axis_sweep_rows.iter().sum::<usize>();
    let row_cache_accounting_matches = if report.row_cache_lookups == 0
        && report.row_cache_hits == 0
        && report.row_cache_misses == 0
    {
        true
    } else {
        report.row_cache_lookups == attempted_rows
            && report.row_cache_hits + report.row_cache_misses == report.row_cache_lookups
            && report.row_cache_misses == report.row_candidate_scheduled_rows
    };
    let row_candidate_schedule_matches = report.row_candidate_scheduled_rows == 0
        || report.row_candidate_scheduled_rows + report.row_cache_hits == attempted_rows;
    let row_candidate_rejections_match = report.row_candidate_scheduled_rows == 0
        || report.row_candidate_aabb_rejections == report.row_ray_aabb_rejections;
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
        && row_candidate_schedule_matches
        && row_cache_accounting_matches
        && row_candidate_rejections_match
        && row_outcomes_match
        && no_component_blockers
        && report.classified_cells > 0;

    PreparedTriangleSolidComponentConsensusAuditReport {
        open_cell_accounting_matches,
        component_accounting_matches,
        retry_subset_matches,
        row_candidate_schedule_matches,
        row_cache_accounting_matches,
        row_candidate_rejections_match,
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
            retry_ray_attempts: 1,
            fallback_components: 0,
            fallback_cells: 0,
            axis_sweep_rows: [1, 1, 0],
            axis_certified_sweep_rows: [1, 1, 0],
            row_candidate_scheduled_rows: 2,
            row_candidate_aabb_rejections: 7,
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
        assert!(audit.row_candidate_schedule_matches);
        assert!(audit.row_cache_accounting_matches);
        assert!(audit.row_candidate_rejections_match);
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
        report.row_cache_misses = 1;
        report.row_candidate_scheduled_rows = 1;

        let audit = audit_prepared_triangle_solid_component_consensus(&report);

        assert!(audit.row_cache_accounting_matches);
        assert!(audit.row_candidate_schedule_matches);
        assert!(audit.exact_component_consensus_audit_ready);
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
    fn audit_accepts_exact_fallback_repair_for_unvoted_component_cells() {
        let mut report = valid_report();
        report.consensus_components = 1;
        report.consensus_cells = 1;
        report.fallback_components = 1;
        report.fallback_cells = 1;
        report.unvoted_component_cells = 1;
        report.deferred_ambiguous_cells = 3;

        let audit = audit_prepared_triangle_solid_component_consensus(&report);

        assert!(audit.open_cell_accounting_matches);
        assert!(audit.component_accounting_matches);
        assert!(audit.no_component_blockers);
        assert!(audit.exact_component_consensus_audit_ready);
    }
}
