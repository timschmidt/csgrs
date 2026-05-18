//! Bounded exact search backends for small 3D packing instances.
//!
//! This module is deliberately small and explicit about limits. It uses exact
//! candidate points and exact replay, following Yap, "Towards Exact Geometric
//! Computation," *Computational Geometry* 7(1-2), 1997
//! (<https://doi.org/10.1016/0925-7721(95)00040-2>): the search may enumerate
//! combinatorial proposals, but accepted placements are certified by exact
//! predicates. The volume/dimension prefilter and decreasing-volume branching
//! are standard rectangular-packing ingredients related to Martello, Pisinger,
//! and Vigo, "The Three-Dimensional Bin Packing Problem," *Operations
//! Research* 48(2), 2000.

use hyperreal::{Real, RealSign};

use crate::{
    Bin3, CapacityBoundStatus, Item3, PackResult, PackingVerification3, Placement3,
    capacity_bounds_3d, verify_packing_3d,
};

/// Limits for bounded exact one-bin search.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ExactSearchLimit3 {
    /// Maximum item count accepted by this backend.
    pub max_items: usize,
    /// Maximum DFS nodes visited before returning [`ExactSearchStatus3::Unknown`].
    pub max_nodes: usize,
}

/// Status returned by bounded exact one-bin search.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactSearchStatus3 {
    /// A feasible placement set was found and replayed exactly.
    Feasible,
    /// The bounded exhaustive search proved no placement in this model.
    Infeasible,
    /// Search was not exhaustive because an explicit limit was reached.
    Unknown,
}

/// Report from bounded exact one-bin cuboid search.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactSearchReport3 {
    /// Search result.
    pub status: ExactSearchStatus3,
    /// DFS nodes visited.
    pub nodes: usize,
    /// Candidate placements tested by exact proposal predicates.
    pub candidate_points: usize,
    /// Exact capacity-bound status checked before branching.
    pub lower_bound_status: CapacityBoundStatus,
    /// Exact replay for the first feasible incumbent, when found.
    pub incumbent: Option<PackingVerification3>,
}

/// Searches for a fixed-orientation one-bin 3D packing with explicit limits.
///
/// This is a small-instance backend, not a production optimizer. Items are
/// branched in certified non-increasing volume order with source-order ties.
/// Candidate origins are exact face-induced corner points from already placed
/// cuboids. The function returns `Unknown` when `max_items` or `max_nodes`
/// prevents exhaustive search.
pub fn branch_and_bound_one_bin_3d(
    bin: &Bin3,
    items: &[Item3],
    limit: ExactSearchLimit3,
) -> PackResult<ExactSearchReport3> {
    let lower_bound = capacity_bounds_3d(bin, items);
    if lower_bound.status == CapacityBoundStatus::Violated {
        return Ok(ExactSearchReport3 {
            status: ExactSearchStatus3::Infeasible,
            nodes: 0,
            candidate_points: 0,
            lower_bound_status: lower_bound.status,
            incumbent: None,
        });
    }
    if items.len() > limit.max_items {
        return Ok(ExactSearchReport3 {
            status: ExactSearchStatus3::Unknown,
            nodes: 0,
            candidate_points: 0,
            lower_bound_status: lower_bound.status,
            incumbent: None,
        });
    }

    let mut ordered = items.iter().enumerate().collect::<Vec<_>>();
    ordered.sort_by(|(left_index, left), (right_index, right)| {
        compare_desc(&left.size.volume(), &right.size.volume())
            .unwrap_or_else(|| left_index.cmp(right_index))
            .then_with(|| left_index.cmp(right_index))
    });
    let ordered_items = ordered
        .iter()
        .map(|(_, item)| (*item).clone())
        .collect::<Vec<_>>();
    let mut state = SearchState3 {
        nodes: 0,
        candidate_points: 0,
        hit_limit: false,
        incumbent: None,
    };
    let mut placements = Vec::new();
    dfs(
        bin,
        &ordered_items,
        0,
        limit.max_nodes,
        &mut placements,
        &mut state,
    )?;

    let status = match (&state.incumbent, state.hit_limit) {
        (Some(_), _) => ExactSearchStatus3::Feasible,
        (None, true) => ExactSearchStatus3::Unknown,
        (None, false) => ExactSearchStatus3::Infeasible,
    };
    Ok(ExactSearchReport3 {
        status,
        nodes: state.nodes,
        candidate_points: state.candidate_points,
        lower_bound_status: lower_bound.status,
        incumbent: state.incumbent,
    })
}

struct SearchState3 {
    nodes: usize,
    candidate_points: usize,
    hit_limit: bool,
    incumbent: Option<PackingVerification3>,
}

fn dfs(
    bin: &Bin3,
    items: &[Item3],
    index: usize,
    max_nodes: usize,
    placements: &mut Vec<Placement3>,
    state: &mut SearchState3,
) -> PackResult<()> {
    if state.incumbent.is_some() || state.hit_limit {
        return Ok(());
    }
    if state.nodes >= max_nodes {
        state.hit_limit = true;
        return Ok(());
    }
    state.nodes += 1;
    if index == items.len() {
        let replay = verify_packing_3d(bin, items, placements)?;
        if replay.feasibility.status == crate::FeasibilityStatus::Feasible
            && replay.objective.unplaced_items == 0
            && replay.objective.duplicate_placements == 0
        {
            state.incumbent = Some(replay);
        }
        return Ok(());
    }

    let item = &items[index];
    for point in candidate_points(placements, items) {
        state.candidate_points += 1;
        if !candidate_fits(bin, item, placements, items, &point) {
            continue;
        }
        placements.push(Placement3 {
            item: item.id.clone(),
            x: point.x,
            y: point.y,
            z: point.z,
        });
        dfs(bin, items, index + 1, max_nodes, placements, state)?;
        placements.pop();
        if state.incumbent.is_some() || state.hit_limit {
            return Ok(());
        }
    }
    Ok(())
}

#[derive(Clone, Debug, PartialEq)]
struct Point3 {
    x: Real,
    y: Real,
    z: Real,
}

fn candidate_points(placements: &[Placement3], items: &[Item3]) -> Vec<Point3> {
    let mut points = vec![Point3 {
        x: Real::zero(),
        y: Real::zero(),
        z: Real::zero(),
    }];
    for placement in placements {
        let Some(item) = items.iter().find(|item| item.id == placement.item) else {
            continue;
        };
        push_unique(
            &mut points,
            Point3 {
                x: placement.x.clone() + item.size.x.clone(),
                y: placement.y.clone(),
                z: placement.z.clone(),
            },
        );
        push_unique(
            &mut points,
            Point3 {
                x: placement.x.clone(),
                y: placement.y.clone() + item.size.y.clone(),
                z: placement.z.clone(),
            },
        );
        push_unique(
            &mut points,
            Point3 {
                x: placement.x.clone(),
                y: placement.y.clone(),
                z: placement.z.clone() + item.size.z.clone(),
            },
        );
    }
    points
}

fn push_unique(points: &mut Vec<Point3>, point: Point3) {
    if !points
        .iter()
        .any(|candidate| points_equal(candidate, &point))
    {
        points.push(point);
    }
}

fn candidate_fits(
    bin: &Bin3,
    item: &Item3,
    placements: &[Placement3],
    items: &[Item3],
    point: &Point3,
) -> bool {
    if !nonnegative(&point.x).unwrap_or(false)
        || !nonnegative(&point.y).unwrap_or(false)
        || !nonnegative(&point.z).unwrap_or(false)
        || !leq(&(point.x.clone() + item.size.x.clone()), &bin.size.x).unwrap_or(false)
        || !leq(&(point.y.clone() + item.size.y.clone()), &bin.size.y).unwrap_or(false)
        || !leq(&(point.z.clone() + item.size.z.clone()), &bin.size.z).unwrap_or(false)
    {
        return false;
    }
    placements.iter().all(|placement| {
        let Some(placed_item) = items.iter().find(|placed| placed.id == placement.item) else {
            return false;
        };
        boxes_disjoint(item, point, placed_item, placement).unwrap_or(false)
    })
}

fn boxes_disjoint(
    item: &Item3,
    point: &Point3,
    placed_item: &Item3,
    placement: &Placement3,
) -> Option<bool> {
    Some(
        leq(&(point.x.clone() + item.size.x.clone()), &placement.x)?
            || leq(
                &(placement.x.clone() + placed_item.size.x.clone()),
                &point.x,
            )?
            || leq(&(point.y.clone() + item.size.y.clone()), &placement.y)?
            || leq(
                &(placement.y.clone() + placed_item.size.y.clone()),
                &point.y,
            )?
            || leq(&(point.z.clone() + item.size.z.clone()), &placement.z)?
            || leq(
                &(placement.z.clone() + placed_item.size.z.clone()),
                &point.z,
            )?,
    )
}

fn points_equal(left: &Point3, right: &Point3) -> bool {
    exact_eq(&left.x, &right.x) && exact_eq(&left.y, &right.y) && exact_eq(&left.z, &right.z)
}

fn compare_desc(left: &Real, right: &Real) -> Option<std::cmp::Ordering> {
    match (left - right).refine_sign_until(-64)? {
        RealSign::Positive => Some(std::cmp::Ordering::Less),
        RealSign::Zero => Some(std::cmp::Ordering::Equal),
        RealSign::Negative => Some(std::cmp::Ordering::Greater),
    }
}

fn exact_eq(left: &Real, right: &Real) -> bool {
    matches!((left - right).refine_sign_until(-64), Some(RealSign::Zero))
}

fn leq(left: &Real, right: &Real) -> Option<bool> {
    match (left - right).refine_sign_until(-64)? {
        RealSign::Negative | RealSign::Zero => Some(true),
        RealSign::Positive => Some(false),
    }
}

fn nonnegative(value: &Real) -> Option<bool> {
    match value.refine_sign_until(-64)? {
        RealSign::Negative => Some(false),
        RealSign::Zero | RealSign::Positive => Some(true),
    }
}
