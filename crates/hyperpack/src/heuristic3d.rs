//! Exact-aware 3D cuboid heuristic proposal reports.
//!
//! The first 3D proposal baselines here use deterministic corner points and
//! free boxes derived from already placed cuboids, common rectangular-packing
//! ideas in bottom-left/back/front, extreme-point, and maximal-space
//! heuristics. The proposal/replay split follows Yap, "Towards Exact Geometric
//! Computation," *Computational Geometry* 7(1-2), 1997
//! (<https://doi.org/10.1016/0925-7721(95)00040-2>): candidate enumeration may
//! be heuristic, but containment and no-overlap acceptance are exact. The
//! decreasing-volume order is a simple analogue of volume-oriented bin-packing
//! baselines discussed in Martello, Pisinger, and Vigo, "The Three-Dimensional
//! Bin Packing Problem," *Operations Research* 48(2), 2000. The free-box split
//! route is intentionally conservative and cites maximal-space/difference
//! process packing ideas near the implementation rather than pretending the
//! heuristic state is an optimality proof.

use hyperreal::{Real, RealSign};

use crate::{
    AxisBox3, Bin3, FeasibilityStatus, Item3, ItemId, PackResult, PackingVerification3, Placement3,
    verify_packing_3d,
};

/// 3D cuboid heuristic family implemented by this module.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CuboidHeuristic3 {
    /// First feasible exact corner point after decreasing-volume item sorting.
    FirstFitDecreasingVolume,
    /// Feasible exact corner point with least residual corner volume.
    BestFitDecreasingVolume,
    /// First feasible exact corner point after decreasing-max-side sorting.
    FirstFitDecreasingMaxSide,
    /// Best feasible exact corner point after decreasing-max-side sorting.
    BestFitDecreasingMaxSide,
    /// First feasible exact corner point after decreasing footprint-area sorting.
    FirstFitDecreasingFootprintArea,
    /// Best feasible exact corner point after decreasing footprint-area sorting.
    BestFitDecreasingFootprintArea,
    /// Exact extreme-point scan after decreasing-volume item sorting.
    ExtremePointDecreasingVolume,
    /// Conservative exact free-box scan after decreasing-volume item sorting.
    MaximalSpaceDecreasingVolume,
    /// Conservative exact 3D guillotine free-box scan after decreasing volume.
    GuillotineBestVolumeFit,
    /// Largest-area-fit-first style proposal after decreasing footprint area.
    LaffLargestAreaFitFirst,
}

/// Exact candidate point inspected by a 3D cuboid heuristic.
#[derive(Clone, Debug, PartialEq)]
pub struct CandidatePoint3 {
    /// Exact x origin.
    pub x: Real,
    /// Exact y origin.
    pub y: Real,
    /// Exact z origin.
    pub z: Real,
}

/// Exact origin-bearing free box retained by a 3D free-space heuristic.
#[derive(Clone, Debug, PartialEq)]
pub struct FreeBox3 {
    /// Exact x origin.
    pub x: Real,
    /// Exact y origin.
    pub y: Real,
    /// Exact z origin.
    pub z: Real,
    /// Exact available dimensions.
    pub size: AxisBox3,
}

/// Candidate placement emitted before exact replay acceptance.
#[derive(Clone, Debug, PartialEq)]
pub struct PlacementCandidate3 {
    /// Proposed placement.
    pub placement: Placement3,
    /// Exact item size used by this fixed-orientation candidate.
    pub size: AxisBox3,
    /// Candidate-point index that generated the placement.
    pub point_index: usize,
}

/// Trace counters for a 3D cuboid heuristic proposal.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CuboidHeuristicTrace3 {
    /// Items considered in deterministic heuristic order.
    pub considered_items: usize,
    /// Candidate placements emitted.
    pub emitted_candidates: usize,
    /// Items rejected before replay because no candidate point fit.
    pub rejected_items: usize,
    /// Exact comparisons performed by the proposal stage.
    pub exact_comparisons: usize,
    /// Candidate points inspected by the proposal stage.
    pub candidate_points: usize,
}

/// Full 3D proposal plus exact replay report.
#[derive(Clone, Debug, PartialEq)]
pub struct CuboidHeuristicReport3 {
    /// Heuristic family.
    pub heuristic: CuboidHeuristic3,
    /// Candidate placements proposed before replay.
    pub candidates: Vec<PlacementCandidate3>,
    /// Candidate points inspected or left as exact placement frontiers.
    pub points: Vec<CandidatePoint3>,
    /// Trace counters.
    pub trace: CuboidHeuristicTrace3,
    /// Exact replay of the emitted placements.
    pub replay: PackingVerification3,
    /// Item ids rejected by the proposal stage.
    pub rejected: Vec<ItemId>,
    /// Exact free boxes retained by free-space based heuristics.
    pub free_boxes: Vec<FreeBox3>,
}

/// Proposes a 3D cuboid layout with first-fit decreasing-volume corner points.
///
/// Items are sorted by non-increasing exact volume when the sign comparison can
/// be certified; uncertified ties retain source order. Each item takes the
/// first exact candidate point that passes proposal-time containment and
/// no-overlap checks. The resulting layout is still accepted only through
/// [`verify_packing_3d`].
pub fn cuboid_first_fit_decreasing_volume_3d(
    bin: &Bin3,
    items: &[Item3],
) -> PackResult<CuboidHeuristicReport3> {
    cuboid_ordered_corner_points_3d(bin, items, CuboidHeuristic3::FirstFitDecreasingVolume)
}

/// Proposes a 3D cuboid layout with best-fit decreasing-volume corner points.
///
/// This scans the same exact candidate points as
/// [`cuboid_first_fit_decreasing_volume_3d`], but chooses the feasible point
/// with least exact residual corner volume
/// `(bin_x - x - item_x) * (bin_y - y - item_y) * (bin_z - z - item_z)`, then
/// uses lowest `z`, `y`, and `x` as deterministic tie-breakers.
pub fn cuboid_best_fit_decreasing_volume_3d(
    bin: &Bin3,
    items: &[Item3],
) -> PackResult<CuboidHeuristicReport3> {
    cuboid_ordered_corner_points_3d(bin, items, CuboidHeuristic3::BestFitDecreasingVolume)
}

/// Proposes a 3D cuboid layout with first-fit decreasing-max-side corner points.
///
/// The ordering key is the exact maximum of each cuboid's three side lengths.
/// This antagonizes long rods and slabs before smaller pieces, while retaining
/// exact replay as the acceptance boundary.
pub fn cuboid_first_fit_decreasing_max_side_3d(
    bin: &Bin3,
    items: &[Item3],
) -> PackResult<CuboidHeuristicReport3> {
    cuboid_ordered_corner_points_3d(bin, items, CuboidHeuristic3::FirstFitDecreasingMaxSide)
}

/// Proposes a 3D cuboid layout with best-fit decreasing-max-side corner points.
///
/// It uses the same exact max-side ordering as
/// [`cuboid_first_fit_decreasing_max_side_3d`] and the same residual-volume
/// scoring as [`cuboid_best_fit_decreasing_volume_3d`].
pub fn cuboid_best_fit_decreasing_max_side_3d(
    bin: &Bin3,
    items: &[Item3],
) -> PackResult<CuboidHeuristicReport3> {
    cuboid_ordered_corner_points_3d(bin, items, CuboidHeuristic3::BestFitDecreasingMaxSide)
}

/// Proposes a 3D cuboid layout with first-fit decreasing-footprint-area points.
///
/// The ordering key is exact `x * y` footprint area. This is a simple layer-ish
/// baseline: broad bases are considered before narrow towers, then exact replay
/// decides whether the proposal is usable.
pub fn cuboid_first_fit_decreasing_footprint_area_3d(
    bin: &Bin3,
    items: &[Item3],
) -> PackResult<CuboidHeuristicReport3> {
    cuboid_ordered_corner_points_3d(
        bin,
        items,
        CuboidHeuristic3::FirstFitDecreasingFootprintArea,
    )
}

/// Proposes a 3D cuboid layout with best-fit decreasing-footprint-area points.
///
/// It uses the same exact footprint ordering as
/// [`cuboid_first_fit_decreasing_footprint_area_3d`] and exact residual-volume
/// scoring among feasible candidate points.
pub fn cuboid_best_fit_decreasing_footprint_area_3d(
    bin: &Bin3,
    items: &[Item3],
) -> PackResult<CuboidHeuristicReport3> {
    cuboid_ordered_corner_points_3d(bin, items, CuboidHeuristic3::BestFitDecreasingFootprintArea)
}

/// Proposes a 3D cuboid layout with exact extreme-point/DBLF-style scoring.
///
/// Candidate points are exact x/y/z face origins induced by already emitted
/// cuboids. Following the deepest-bottom-left-fill heuristic family, this
/// volume-ordered variant scans all feasible points and chooses lowest exact
/// `y`, then lowest `z`, then lowest `x`. Exact replay remains the acceptance
/// gate.
pub fn cuboid_extreme_point_decreasing_volume_3d(
    bin: &Bin3,
    items: &[Item3],
) -> PackResult<CuboidHeuristicReport3> {
    cuboid_ordered_corner_points_3d(bin, items, CuboidHeuristic3::ExtremePointDecreasingVolume)
}

/// Proposes a 3D layout with conservative exact free-box splitting.
///
/// This is a maximal-space/difference-process inspired proposal path in the
/// sense of Crainic, Perboli, and Tadei, "Extreme Point-based Heuristics for
/// Three-Dimensional Bin Packing," *INFORMS Journal on Computing* 20(3),
/// 2008, and the heuristic families surveyed by Lodi, Martello, and Vigo,
/// "Heuristic algorithms for the three-dimensional bin packing problem,"
/// *European Journal of Operational Research* 141(2), 2002. It keeps exact
/// origin-bearing free boxes, places each decreasing-volume item at the origin
/// of the best exact residual-volume box, and partitions that selected box into
/// right/front/top residual boxes. Those residual boxes are scheduling state;
/// the returned placements are trusted only after [`verify_packing_3d`] replay.
pub fn cuboid_maximal_space_decreasing_volume_3d(
    bin: &Bin3,
    items: &[Item3],
) -> PackResult<CuboidHeuristicReport3> {
    cuboid_free_boxes_3d(bin, items, CuboidHeuristic3::MaximalSpaceDecreasingVolume)
}

/// Proposes a 3D layout with exact guillotine-style residual splits.
///
/// Following the guillotine-cut restrictions discussed by Lodi, Martello, and
/// Vigo, "Heuristic algorithms for the three-dimensional bin packing problem,"
/// *European Journal of Operational Research* 141(2), 2002, this proposal
/// treats the retained free boxes as a cut-feasible scheduling state. It places
/// each decreasing-volume cuboid in the best exact residual-volume free box and
/// emits the right/front/top residual boxes induced by sequential orthogonal
/// cuts. As required by Yap's exact-computation discipline, this cut tree is
/// evidence for proposal generation only; exact 3D containment and no-overlap
/// replay still decides acceptance.
pub fn cuboid_guillotine_best_volume_fit_3d(
    bin: &Bin3,
    items: &[Item3],
) -> PackResult<CuboidHeuristicReport3> {
    cuboid_free_boxes_3d(bin, items, CuboidHeuristic3::GuillotineBestVolumeFit)
}

/// Proposes a 3D cuboid layout with a LAFF-style largest-area-first order.
///
/// LAFF/largest-area-fit-first container packers prioritize broad base
/// footprints before smaller pieces, then build from low layers. See Lodi,
/// Martello, and Vigo, "Heuristic algorithms for the three-dimensional bin
/// packing problem," *European Journal of Operational Research* 141(2), 2002,
/// for the layer/area-oriented heuristic family. This implementation keeps the
/// Hyper boundary conservative: it sorts by exact `x * y` footprint area, scans
/// exact face-induced candidate points, chooses the certified lowest `z`, then
/// `y`, then `x`, and accepts only through [`verify_packing_3d`].
pub fn cuboid_laff_largest_area_fit_first_3d(
    bin: &Bin3,
    items: &[Item3],
) -> PackResult<CuboidHeuristicReport3> {
    cuboid_ordered_corner_points_3d(bin, items, CuboidHeuristic3::LaffLargestAreaFitFirst)
}

fn cuboid_ordered_corner_points_3d(
    bin: &Bin3,
    items: &[Item3],
    heuristic: CuboidHeuristic3,
) -> PackResult<CuboidHeuristicReport3> {
    let mut trace = CuboidHeuristicTrace3 {
        considered_items: items.len(),
        emitted_candidates: 0,
        rejected_items: 0,
        exact_comparisons: 0,
        candidate_points: 0,
    };
    let mut ordered = items.iter().enumerate().collect::<Vec<_>>();
    ordered.sort_by(|(left_index, left), (right_index, right)| {
        trace.exact_comparisons += 1;
        match compare_desc(&sort_key(left, heuristic), &sort_key(right, heuristic)) {
            Some(ordering) => ordering.then_with(|| left_index.cmp(right_index)),
            None => left_index.cmp(right_index),
        }
    });

    let mut candidates = Vec::<PlacementCandidate3>::new();
    let mut rejected = Vec::new();
    let mut points = vec![CandidatePoint3 {
        x: Real::zero(),
        y: Real::zero(),
        z: Real::zero(),
    }];

    for (_, item) in ordered {
        let mut best = None::<CuboidChoice3>;
        for (point_index, point) in candidate_points_from(&candidates).into_iter().enumerate() {
            trace.candidate_points += 1;
            if !candidate_fits(bin, item, &candidates, &point, &mut trace) {
                continue;
            }
            let residual = residual_corner_volume(bin, item, &point);
            let choice = CuboidChoice3 {
                point_index,
                point: point.clone(),
                residual,
            };
            match (&best, heuristic) {
                (None, _) => best = Some(choice),
                (Some(_), CuboidHeuristic3::FirstFitDecreasingVolume)
                | (Some(_), CuboidHeuristic3::FirstFitDecreasingMaxSide)
                | (Some(_), CuboidHeuristic3::FirstFitDecreasingFootprintArea)
                | (Some(_), CuboidHeuristic3::MaximalSpaceDecreasingVolume)
                | (Some(_), CuboidHeuristic3::GuillotineBestVolumeFit) => break,
                (Some(current), CuboidHeuristic3::BestFitDecreasingVolume)
                | (Some(current), CuboidHeuristic3::BestFitDecreasingMaxSide)
                | (Some(current), CuboidHeuristic3::BestFitDecreasingFootprintArea) => {
                    if cuboid_choice_better(&choice, current, &mut trace) {
                        best = Some(choice);
                    }
                }
                (Some(current), CuboidHeuristic3::ExtremePointDecreasingVolume) => {
                    if cuboid_deep_bottom_left_better(&choice, current, &mut trace) {
                        best = Some(choice);
                    }
                }
                (Some(current), CuboidHeuristic3::LaffLargestAreaFitFirst) => {
                    if cuboid_low_layer_better(&choice, current, &mut trace) {
                        best = Some(choice);
                    }
                }
            }
        }

        match best {
            Some(choice) => {
                candidates.push(PlacementCandidate3 {
                    placement: Placement3 {
                        item: item.id.clone(),
                        x: choice.point.x.clone(),
                        y: choice.point.y.clone(),
                        z: choice.point.z.clone(),
                    },
                    size: item.size.clone(),
                    point_index: choice.point_index,
                });
                trace.emitted_candidates += 1;
                points = candidate_points_from(&candidates);
            }
            None => {
                rejected.push(item.id.clone());
                trace.rejected_items += 1;
            }
        }
    }

    let placements = candidates
        .iter()
        .map(|candidate| candidate.placement.clone())
        .collect::<Vec<_>>();
    let mut replay = verify_packing_3d(bin, items, &placements)?;
    if !rejected.is_empty() && replay.feasibility.status == FeasibilityStatus::Feasible {
        replay
            .feasibility
            .facts
            .push("proposal rejected at least one item before replay".into());
    }

    Ok(CuboidHeuristicReport3 {
        heuristic,
        candidates,
        points,
        trace,
        replay,
        rejected,
        free_boxes: Vec::new(),
    })
}

fn cuboid_free_boxes_3d(
    bin: &Bin3,
    items: &[Item3],
    heuristic: CuboidHeuristic3,
) -> PackResult<CuboidHeuristicReport3> {
    let mut trace = CuboidHeuristicTrace3 {
        considered_items: items.len(),
        emitted_candidates: 0,
        rejected_items: 0,
        exact_comparisons: 0,
        candidate_points: 0,
    };
    let mut ordered = items.iter().enumerate().collect::<Vec<_>>();
    ordered.sort_by(|(left_index, left), (right_index, right)| {
        trace.exact_comparisons += 1;
        match compare_desc(&left.size.volume(), &right.size.volume()) {
            Some(ordering) => ordering.then_with(|| left_index.cmp(right_index)),
            None => left_index.cmp(right_index),
        }
    });

    let mut candidates = Vec::<PlacementCandidate3>::new();
    let mut rejected = Vec::new();
    let mut free_boxes = vec![FreeBox3 {
        x: Real::zero(),
        y: Real::zero(),
        z: Real::zero(),
        size: bin.size.clone(),
    }];
    let mut points = Vec::<CandidatePoint3>::new();

    for (_, item) in ordered {
        let mut best = None::<FreeBoxChoice3>;
        for (free_box_index, free_box) in free_boxes.iter().enumerate() {
            trace.candidate_points += 1;
            points.push(CandidatePoint3 {
                x: free_box.x.clone(),
                y: free_box.y.clone(),
                z: free_box.z.clone(),
            });
            if !free_box_fits(free_box, item, &mut trace) {
                continue;
            }
            let residual = free_box.size.volume() - item.size.volume();
            let choice = FreeBoxChoice3 {
                free_box_index,
                point: CandidatePoint3 {
                    x: free_box.x.clone(),
                    y: free_box.y.clone(),
                    z: free_box.z.clone(),
                },
                residual,
            };
            match &best {
                None => best = Some(choice),
                Some(current) => {
                    if free_box_choice_better(&choice, current, &mut trace) {
                        best = Some(choice);
                    }
                }
            }
        }

        match best {
            Some(choice) => {
                let used_space = free_boxes.remove(choice.free_box_index);
                candidates.push(PlacementCandidate3 {
                    placement: Placement3 {
                        item: item.id.clone(),
                        x: choice.point.x.clone(),
                        y: choice.point.y.clone(),
                        z: choice.point.z.clone(),
                    },
                    size: item.size.clone(),
                    point_index: choice.free_box_index,
                });
                trace.emitted_candidates += 1;
                split_free_box_3d(&mut free_boxes, &used_space, item);
            }
            None => {
                rejected.push(item.id.clone());
                trace.rejected_items += 1;
            }
        }
    }

    let placements = candidates
        .iter()
        .map(|candidate| candidate.placement.clone())
        .collect::<Vec<_>>();
    let mut replay = verify_packing_3d(bin, items, &placements)?;
    if !rejected.is_empty() && replay.feasibility.status == FeasibilityStatus::Feasible {
        replay
            .feasibility
            .facts
            .push("proposal rejected at least one item before replay".into());
    }

    Ok(CuboidHeuristicReport3 {
        heuristic,
        candidates,
        points,
        trace,
        replay,
        rejected,
        free_boxes,
    })
}

fn sort_key(item: &Item3, heuristic: CuboidHeuristic3) -> Real {
    match heuristic {
        CuboidHeuristic3::FirstFitDecreasingVolume | CuboidHeuristic3::BestFitDecreasingVolume => {
            item.size.volume()
        }
        CuboidHeuristic3::FirstFitDecreasingMaxSide
        | CuboidHeuristic3::BestFitDecreasingMaxSide => {
            max_real(&max_real(&item.size.x, &item.size.y), &item.size.z)
        }
        CuboidHeuristic3::FirstFitDecreasingFootprintArea
        | CuboidHeuristic3::BestFitDecreasingFootprintArea => {
            item.size.x.clone() * item.size.y.clone()
        }
        CuboidHeuristic3::ExtremePointDecreasingVolume => item.size.volume(),
        CuboidHeuristic3::MaximalSpaceDecreasingVolume => item.size.volume(),
        CuboidHeuristic3::GuillotineBestVolumeFit => item.size.volume(),
        CuboidHeuristic3::LaffLargestAreaFitFirst => item.size.x.clone() * item.size.y.clone(),
    }
}

fn max_real(left: &Real, right: &Real) -> Real {
    if leq(left, right).unwrap_or(false) {
        right.clone()
    } else {
        left.clone()
    }
}

#[derive(Clone, Debug, PartialEq)]
struct CuboidChoice3 {
    point_index: usize,
    point: CandidatePoint3,
    residual: Real,
}

#[derive(Clone, Debug, PartialEq)]
struct FreeBoxChoice3 {
    free_box_index: usize,
    point: CandidatePoint3,
    residual: Real,
}

fn candidate_points_from(candidates: &[PlacementCandidate3]) -> Vec<CandidatePoint3> {
    let mut points = vec![CandidatePoint3 {
        x: Real::zero(),
        y: Real::zero(),
        z: Real::zero(),
    }];
    for candidate in candidates {
        points.push(CandidatePoint3 {
            x: candidate.placement.x.clone() + candidate.size.x.clone(),
            y: candidate.placement.y.clone(),
            z: candidate.placement.z.clone(),
        });
        points.push(CandidatePoint3 {
            x: candidate.placement.x.clone(),
            y: candidate.placement.y.clone() + candidate.size.y.clone(),
            z: candidate.placement.z.clone(),
        });
        points.push(CandidatePoint3 {
            x: candidate.placement.x.clone(),
            y: candidate.placement.y.clone(),
            z: candidate.placement.z.clone() + candidate.size.z.clone(),
        });
    }
    points
}

fn candidate_fits(
    bin: &Bin3,
    item: &Item3,
    candidates: &[PlacementCandidate3],
    point: &CandidatePoint3,
    trace: &mut CuboidHeuristicTrace3,
) -> bool {
    trace.exact_comparisons += 6;
    if !nonnegative(&point.x).unwrap_or(false)
        || !nonnegative(&point.y).unwrap_or(false)
        || !nonnegative(&point.z).unwrap_or(false)
        || !leq(&(point.x.clone() + item.size.x.clone()), &bin.size.x).unwrap_or(false)
        || !leq(&(point.y.clone() + item.size.y.clone()), &bin.size.y).unwrap_or(false)
        || !leq(&(point.z.clone() + item.size.z.clone()), &bin.size.z).unwrap_or(false)
    {
        return false;
    }

    for candidate in candidates {
        trace.exact_comparisons += 6;
        if !boxes_disjoint(point, &item.size, candidate).unwrap_or(false) {
            return false;
        }
    }
    true
}

fn boxes_disjoint(
    point: &CandidatePoint3,
    size: &AxisBox3,
    candidate: &PlacementCandidate3,
) -> Option<bool> {
    Some(
        leq(&(point.x.clone() + size.x.clone()), &candidate.placement.x)?
            || leq(
                &(candidate.placement.x.clone() + candidate.size.x.clone()),
                &point.x,
            )?
            || leq(&(point.y.clone() + size.y.clone()), &candidate.placement.y)?
            || leq(
                &(candidate.placement.y.clone() + candidate.size.y.clone()),
                &point.y,
            )?
            || leq(&(point.z.clone() + size.z.clone()), &candidate.placement.z)?
            || leq(
                &(candidate.placement.z.clone() + candidate.size.z.clone()),
                &point.z,
            )?,
    )
}

fn residual_corner_volume(bin: &Bin3, item: &Item3, point: &CandidatePoint3) -> Real {
    (bin.size.x.clone() - (point.x.clone() + item.size.x.clone()))
        * (bin.size.y.clone() - (point.y.clone() + item.size.y.clone()))
        * (bin.size.z.clone() - (point.z.clone() + item.size.z.clone()))
}

fn free_box_fits(free_box: &FreeBox3, item: &Item3, trace: &mut CuboidHeuristicTrace3) -> bool {
    trace.exact_comparisons += 3;
    leq(&item.size.x, &free_box.size.x).unwrap_or(false)
        && leq(&item.size.y, &free_box.size.y).unwrap_or(false)
        && leq(&item.size.z, &free_box.size.z).unwrap_or(false)
}

fn free_box_choice_better(
    choice: &FreeBoxChoice3,
    current: &FreeBoxChoice3,
    trace: &mut CuboidHeuristicTrace3,
) -> bool {
    trace.exact_comparisons += 1;
    if lt(&choice.residual, &current.residual).unwrap_or(false) {
        return true;
    }
    trace.exact_comparisons += 1;
    exact_eq(&choice.residual, &current.residual)
        && score_low_xyz(
            &choice.point.z,
            &current.point.z,
            &choice.point.y,
            &current.point.y,
            &choice.point.x,
            &current.point.x,
            trace,
        )
}

fn split_free_box_3d(free_boxes: &mut Vec<FreeBox3>, used_space: &FreeBox3, item: &Item3) {
    let remaining_x = used_space.size.x.clone() - item.size.x.clone();
    let remaining_y = used_space.size.y.clone() - item.size.y.clone();
    let remaining_z = used_space.size.z.clone() - item.size.z.clone();

    push_positive_free_box(
        free_boxes,
        used_space.x.clone() + item.size.x.clone(),
        used_space.y.clone(),
        used_space.z.clone(),
        remaining_x,
        used_space.size.y.clone(),
        used_space.size.z.clone(),
    );
    push_positive_free_box(
        free_boxes,
        used_space.x.clone(),
        used_space.y.clone() + item.size.y.clone(),
        used_space.z.clone(),
        item.size.x.clone(),
        remaining_y,
        used_space.size.z.clone(),
    );
    push_positive_free_box(
        free_boxes,
        used_space.x.clone(),
        used_space.y.clone(),
        used_space.z.clone() + item.size.z.clone(),
        item.size.x.clone(),
        item.size.y.clone(),
        remaining_z,
    );
}

fn push_positive_free_box(
    free_boxes: &mut Vec<FreeBox3>,
    x: Real,
    y: Real,
    z: Real,
    size_x: Real,
    size_y: Real,
    size_z: Real,
) {
    if positive(&size_x).unwrap_or(false)
        && positive(&size_y).unwrap_or(false)
        && positive(&size_z).unwrap_or(false)
    {
        free_boxes.push(FreeBox3 {
            x,
            y,
            z,
            size: AxisBox3 {
                x: size_x,
                y: size_y,
                z: size_z,
            },
        });
    }
}

fn cuboid_choice_better(
    choice: &CuboidChoice3,
    current: &CuboidChoice3,
    trace: &mut CuboidHeuristicTrace3,
) -> bool {
    trace.exact_comparisons += 1;
    let lower_residual = lt(&choice.residual, &current.residual).unwrap_or(false);
    trace.exact_comparisons += 1;
    let same_residual = exact_eq(&choice.residual, &current.residual);
    if lower_residual {
        return true;
    }
    same_residual
        && score_low_xyz(
            &choice.point.z,
            &current.point.z,
            &choice.point.y,
            &current.point.y,
            &choice.point.x,
            &current.point.x,
            trace,
        )
}

fn cuboid_deep_bottom_left_better(
    choice: &CuboidChoice3,
    current: &CuboidChoice3,
    trace: &mut CuboidHeuristicTrace3,
) -> bool {
    score_low_xyz(
        &choice.point.y,
        &current.point.y,
        &choice.point.z,
        &current.point.z,
        &choice.point.x,
        &current.point.x,
        trace,
    )
}

fn cuboid_low_layer_better(
    choice: &CuboidChoice3,
    current: &CuboidChoice3,
    trace: &mut CuboidHeuristicTrace3,
) -> bool {
    score_low_xyz(
        &choice.point.z,
        &current.point.z,
        &choice.point.y,
        &current.point.y,
        &choice.point.x,
        &current.point.x,
        trace,
    )
}

fn score_low_xyz(
    left_primary: &Real,
    right_primary: &Real,
    left_secondary: &Real,
    right_secondary: &Real,
    left_tertiary: &Real,
    right_tertiary: &Real,
    trace: &mut CuboidHeuristicTrace3,
) -> bool {
    trace.exact_comparisons += 1;
    if lt(left_primary, right_primary).unwrap_or(false) {
        return true;
    }
    trace.exact_comparisons += 1;
    if !exact_eq(left_primary, right_primary) {
        return false;
    }
    trace.exact_comparisons += 1;
    if lt(left_secondary, right_secondary).unwrap_or(false) {
        return true;
    }
    trace.exact_comparisons += 1;
    exact_eq(left_secondary, right_secondary) && lt(left_tertiary, right_tertiary).unwrap_or(false)
}

fn compare_desc(left: &Real, right: &Real) -> Option<std::cmp::Ordering> {
    match (left - right).refine_sign_until(-64)? {
        RealSign::Positive => Some(std::cmp::Ordering::Less),
        RealSign::Zero => Some(std::cmp::Ordering::Equal),
        RealSign::Negative => Some(std::cmp::Ordering::Greater),
    }
}

fn leq(left: &Real, right: &Real) -> Option<bool> {
    match (left - right).refine_sign_until(-64)? {
        RealSign::Negative | RealSign::Zero => Some(true),
        RealSign::Positive => Some(false),
    }
}

fn lt(left: &Real, right: &Real) -> Option<bool> {
    match (left - right).refine_sign_until(-64)? {
        RealSign::Negative => Some(true),
        RealSign::Zero | RealSign::Positive => Some(false),
    }
}

fn exact_eq(left: &Real, right: &Real) -> bool {
    matches!((left - right).refine_sign_until(-64), Some(RealSign::Zero))
}

fn nonnegative(value: &Real) -> Option<bool> {
    match value.refine_sign_until(-64)? {
        RealSign::Negative => Some(false),
        RealSign::Zero | RealSign::Positive => Some(true),
    }
}

fn positive(value: &Real) -> Option<bool> {
    match value.refine_sign_until(-64)? {
        RealSign::Positive => Some(true),
        RealSign::Negative | RealSign::Zero => Some(false),
    }
}
