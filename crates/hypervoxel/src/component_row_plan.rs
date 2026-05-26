//! Exact component-local row planning for triangle-solid schedules.
//!
//! Component-local winding consensus only has value if the row work it skips
//! and performs is itself retained evidence. This module turns a connected
//! component's integer cells into deterministic axis-row memberships before
//! any ray/AABB or ray/triangle predicate runs. The plan is not topology
//! evidence; it is schedule evidence that can be audited against later row
//! cache and candidate counters.
//!
//! The component model follows Rosenfeld and Pfaltz, "Sequential Operations in
//! Digital Picture Processing," *JACM* 13(4), 1966. The exactness rule follows
//! Yap, "Towards Exact Geometric Computation," *Computational Geometry*
//! 7(1-2), 1997: an accelerated arrangement may be consumed only when the
//! discrete object facts it relies on are retained and replayable.

use std::collections::{BTreeMap, BTreeSet};

/// One component-local row membership in an axis-parallel arrangement.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct ComponentAxisRowMembership {
    /// Two perpendicular row coordinates for the sweep axis.
    pub row: [u64; 2],
    /// Component-local cell indices on this row, in deterministic order.
    pub component_indices: Vec<usize>,
    /// Minimum sweep-axis coordinate among the row's component cells.
    pub min_axis_coord: u64,
}

/// Deterministic row plan for one component and one sweep axis.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct ComponentAxisRowPlan {
    /// Sweep axis, where `0`, `1`, and `2` are `+X`, `+Y`, and `+Z`.
    pub axis: usize,
    /// Planned row memberships.
    pub rows: Vec<ComponentAxisRowMembership>,
    /// Audit report for this component-axis plan.
    pub report: ComponentAxisRowPlanReport,
}

/// Audit report for one component-axis row plan.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct ComponentAxisRowPlanReport {
    /// Sweep axis planned by this report.
    pub axis: usize,
    /// Number of component cells supplied to the planner.
    pub component_cells: usize,
    /// Number of row memberships emitted.
    pub planned_rows: usize,
    /// Total cell-to-row memberships emitted.
    pub row_cell_memberships: usize,
    /// Component-local cell indices that appeared in more than one row.
    pub duplicate_memberships: Vec<usize>,
    /// Component-local cell indices that did not appear in any row.
    pub missing_memberships: Vec<usize>,
    /// Rows whose retained minimum coordinate did not equal the minimum over
    /// their member cells.
    pub min_axis_coord_violations: usize,
    /// Whether this plan is usable as exact component-local row schedule
    /// evidence.
    pub exact_component_row_plan_ready: bool,
}

/// Build a deterministic row plan for one component and sweep axis.
///
/// Each component cell must appear in exactly one row for the chosen axis, and
/// each row retains the minimum sweep-axis coordinate needed by the later
/// exact lower-window ray/AABB schedule. This function does not inspect
/// triangles and does not decide parity; it only preserves the integer
/// component-to-row structure that the arrangement scheduler will consume.
pub(crate) fn plan_component_axis_rows(
    axis: usize,
    component: &[[u64; 3]],
) -> ComponentAxisRowPlan {
    let [row_axis_a, row_axis_b] = perpendicular_axes(axis);
    let mut row_map = BTreeMap::<[u64; 2], Vec<usize>>::new();
    for (component_index, coords) in component.iter().enumerate() {
        row_map
            .entry([coords[row_axis_a], coords[row_axis_b]])
            .or_default()
            .push(component_index);
    }

    let mut rows = Vec::with_capacity(row_map.len());
    let mut seen = BTreeSet::new();
    let mut duplicate_memberships = Vec::new();
    let mut row_cell_memberships = 0_usize;
    let mut min_axis_coord_violations = 0_usize;
    for (row, component_indices) in row_map {
        row_cell_memberships += component_indices.len();
        for &component_index in &component_indices {
            if !seen.insert(component_index) {
                duplicate_memberships.push(component_index);
            }
        }
        let min_axis_coord = component_indices
            .iter()
            .map(|&component_index| component[component_index][axis])
            .min()
            .unwrap_or(0);
        if component_indices
            .iter()
            .any(|&component_index| component[component_index][axis] < min_axis_coord)
        {
            min_axis_coord_violations += 1;
        }
        rows.push(ComponentAxisRowMembership {
            row,
            component_indices,
            min_axis_coord,
        });
    }

    let missing_memberships = (0..component.len())
        .filter(|component_index| !seen.contains(component_index))
        .collect::<Vec<_>>();
    let exact_component_row_plan_ready = !component.is_empty()
        && row_cell_memberships == component.len()
        && duplicate_memberships.is_empty()
        && missing_memberships.is_empty()
        && min_axis_coord_violations == 0;
    let report = ComponentAxisRowPlanReport {
        axis,
        component_cells: component.len(),
        planned_rows: rows.len(),
        row_cell_memberships,
        duplicate_memberships,
        missing_memberships,
        min_axis_coord_violations,
        exact_component_row_plan_ready,
    };

    ComponentAxisRowPlan { axis, rows, report }
}

fn perpendicular_axes(axis: usize) -> [usize; 2] {
    match axis {
        0 => [1, 2],
        1 => [0, 2],
        2 => [0, 1],
        _ => unreachable!("sweep axis must be 0, 1, or 2"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn row_plan_groups_component_cells_by_exact_perpendicular_coordinates() {
        let component = vec![[3, 1, 2], [4, 1, 2], [2, 5, 2]];

        let plan = plan_component_axis_rows(0, &component);

        assert_eq!(plan.axis, 0);
        assert_eq!(plan.rows.len(), 2);
        assert_eq!(plan.rows[0].row, [1, 2]);
        assert_eq!(plan.rows[0].component_indices, vec![0, 1]);
        assert_eq!(plan.rows[0].min_axis_coord, 3);
        assert_eq!(plan.rows[1].row, [5, 2]);
        assert_eq!(plan.rows[1].component_indices, vec![2]);
        assert_eq!(plan.rows[1].min_axis_coord, 2);
        assert_eq!(plan.report.row_cell_memberships, component.len());
        assert!(plan.report.duplicate_memberships.is_empty());
        assert!(plan.report.missing_memberships.is_empty());
        assert!(plan.report.exact_component_row_plan_ready);
    }

    #[test]
    fn row_plan_refuses_empty_component_as_vacuous_schedule_evidence() {
        let plan = plan_component_axis_rows(1, &[]);

        assert_eq!(plan.report.planned_rows, 0);
        assert_eq!(plan.report.row_cell_memberships, 0);
        assert!(!plan.report.exact_component_row_plan_ready);
    }
}
