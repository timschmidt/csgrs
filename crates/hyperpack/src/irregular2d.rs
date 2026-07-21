//! Exact-aware irregular 2D sheet packing.
//!
//! Convex line contours receive exact no-fit regions from `hypercurve`.
//! Prepared geometry is immutable and keyed by stable item ids, so it cannot
//! silently outlive the shapes from which it was built. Replay remains the
//! authority: unsupported curves, concave decomposition, and undecidable
//! predicates propagate as [`FeasibilityStatus::Unknown`].

use std::{cmp::Ordering, collections::BTreeMap, fmt};

use hypercurve::{
    Classification, Contour2, ContourPointLocation, CurveError, CurvePolicy, Point2, Segment2,
    TranslationObstacle2, TranslationObstacleBlocker2, translation_obstacle_convex,
};
use hyperreal::{Real, RealSign};

use crate::{FeasibilityStatus, ItemId, PackError, SheetBin2};

/// Result alias for irregular packing operations that can fail in either crate.
pub type IrregularPackResult2<T> = Result<T, IrregularPackError2>;

/// Error surfaced while preparing or replaying irregular geometry.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum IrregularPackError2 {
    /// Packing-model validation failed.
    Pack(PackError),
    /// Exact curve construction or evaluation failed.
    Curve(CurveError),
}

impl fmt::Display for IrregularPackError2 {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Pack(error) => write!(formatter, "packing validation failed: {error:?}"),
            Self::Curve(error) => write!(formatter, "curve operation failed: {error}"),
        }
    }
}

impl std::error::Error for IrregularPackError2 {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Pack(_) => None,
            Self::Curve(error) => Some(error),
        }
    }
}

impl From<PackError> for IrregularPackError2 {
    fn from(error: PackError) -> Self {
        Self::Pack(error)
    }
}

impl From<CurveError> for IrregularPackError2 {
    fn from(error: CurveError) -> Self {
        Self::Curve(error)
    }
}

/// One fixed-orientation irregular item represented by a closed contour.
#[derive(Clone, Debug, PartialEq)]
pub struct IrregularSheetItem2 {
    /// Stable item id.
    pub id: ItemId,
    /// Exact local-space boundary.
    pub shape: Contour2,
}

/// Translation-only placement of an irregular item.
#[derive(Clone, Debug, PartialEq)]
pub struct IrregularSheetPlacement2 {
    /// Placed item id.
    pub item: ItemId,
    /// Exact x translation.
    pub x: Real,
    /// Exact y translation.
    pub y: Real,
}

/// Cached no-fit result for one canonical unordered pair of item ids.
#[derive(Clone, Debug, PartialEq)]
pub struct PreparedNoFitPair2 {
    fixed_item: ItemId,
    moving_item: ItemId,
    obstacle: Option<TranslationObstacle2>,
    blocker: Option<TranslationObstacleBlocker2>,
}

impl PreparedNoFitPair2 {
    /// Canonically ordered stationary item id.
    pub const fn fixed_item(&self) -> &ItemId {
        &self.fixed_item
    }

    /// Canonically ordered moving item id.
    pub const fn moving_item(&self) -> &ItemId {
        &self.moving_item
    }

    /// Exact forbidden translation region, when construction succeeded.
    pub const fn obstacle(&self) -> Option<&TranslationObstacle2> {
        self.obstacle.as_ref()
    }

    /// Explicit reason the region was not built.
    pub const fn blocker(&self) -> Option<&TranslationObstacleBlocker2> {
        self.blocker.as_ref()
    }
}

/// Immutable item inventory and pairwise no-fit cache for repeated search.
#[derive(Clone, Debug, PartialEq)]
pub struct PreparedIrregularPacking2 {
    items: BTreeMap<ItemId, IrregularSheetItem2>,
    pairs: BTreeMap<(ItemId, ItemId), PreparedNoFitPair2>,
    ready_pair_count: usize,
    blocked_pair_count: usize,
}

impl PreparedIrregularPacking2 {
    /// Immutable item inventory retained with the cache.
    pub const fn items(&self) -> &BTreeMap<ItemId, IrregularSheetItem2> {
        &self.items
    }

    /// Canonically keyed unordered item-pair cache.
    pub const fn pairs(&self) -> &BTreeMap<(ItemId, ItemId), PreparedNoFitPair2> {
        &self.pairs
    }

    /// Number of pairs with an exact no-fit region.
    pub const fn ready_pair_count(&self) -> usize {
        self.ready_pair_count
    }

    /// Number of pairs carrying an explicit construction blocker.
    pub const fn blocked_pair_count(&self) -> usize {
        self.blocked_pair_count
    }
}

/// Exact objective/accounting summary for one irregular sheet.
#[derive(Clone, Debug, PartialEq)]
pub struct IrregularSheetObjective2 {
    /// Exact sheet area.
    pub bin_area: Real,
    /// Sum of placed contour areas, or `None` if an area was unavailable.
    pub used_area: Option<Real>,
    /// Exact `bin_area - used_area`, or `None` with unavailable area evidence.
    pub waste_area: Option<Real>,
    /// Number of item ids placed at least once.
    pub placed_items: usize,
    /// Number of item ids not placed.
    pub unplaced_items: usize,
    /// Number of placement records beyond the first for an item id.
    pub duplicate_placements: usize,
}

/// Authoritative replay report for an irregular sheet placement.
#[derive(Clone, Debug, PartialEq)]
pub struct IrregularSheetVerification2 {
    /// Overall exact feasibility status.
    pub status: FeasibilityStatus,
    /// Number of sheet-containment checks.
    pub containment_checks: usize,
    /// Number of cached no-overlap checks.
    pub no_overlap_checks: usize,
    /// Exact objective/accounting evidence.
    pub objective: IrregularSheetObjective2,
    /// Item ids not present in the proposal.
    pub unplaced: Vec<ItemId>,
    /// Item ids placed more than once.
    pub duplicates: Vec<ItemId>,
    /// Human-readable exact facts and explicit unknowns.
    pub facts: Vec<String>,
}

/// Deterministic bottom-left proposal report for prepared irregular items.
#[derive(Clone, Debug, PartialEq)]
pub struct IrregularBottomLeftReport2 {
    /// Proposed placements in stable item-id order.
    pub placements: Vec<IrregularSheetPlacement2>,
    /// Items for which no exactly feasible candidate was found.
    pub unplaced: Vec<ItemId>,
    /// Number of candidate translations replayed.
    pub candidates_tested: usize,
    /// Number of candidate translations derived from cached no-fit boundaries.
    pub cache_boundary_candidates: usize,
    /// Number of candidates rejected because replay returned `Unknown`.
    pub unknown_candidates: usize,
    /// Number of candidate order comparisons that were undecidable.
    pub uncertain_orderings: usize,
    /// Authoritative replay of the final proposal.
    pub replay: IrregularSheetVerification2,
}

/// Build one exact no-fit region for every unordered pair of declared items.
pub fn prepare_irregular_packing_2d(
    items: &[IrregularSheetItem2],
) -> IrregularPackResult2<PreparedIrregularPacking2> {
    let item_map = items
        .iter()
        .cloned()
        .map(|item| (item.id.clone(), item))
        .collect::<BTreeMap<_, _>>();
    if item_map.len() != items.len() {
        return Err(PackError::DuplicateItem.into());
    }

    let policy = CurvePolicy::certified();
    let ids = item_map.keys().cloned().collect::<Vec<_>>();
    let mut pairs = BTreeMap::new();
    let mut ready_pair_count = 0;
    let mut blocked_pair_count = 0;
    for fixed_index in 0..ids.len() {
        for moving_index in (fixed_index + 1)..ids.len() {
            let fixed_id = ids[fixed_index].clone();
            let moving_id = ids[moving_index].clone();
            let report = translation_obstacle_convex(
                &item_map[&fixed_id].shape,
                &item_map[&moving_id].shape,
                &policy,
            )?;
            let obstacle = report.obstacle().cloned();
            let blocker = report.blocker().cloned();
            if obstacle.is_some() {
                ready_pair_count += 1;
            } else {
                blocked_pair_count += 1;
            }
            pairs.insert(
                (fixed_id.clone(), moving_id.clone()),
                PreparedNoFitPair2 {
                    fixed_item: fixed_id,
                    moving_item: moving_id,
                    obstacle,
                    blocker,
                },
            );
        }
    }

    Ok(PreparedIrregularPacking2 {
        items: item_map,
        pairs,
        ready_pair_count,
        blocked_pair_count,
    })
}

/// Propose a translation-only layout from sheet corners and cached no-fit vertices.
///
/// Items are considered in stable id order. For each item, candidates include
/// its sheet lower-left translation and full/x/y projections of every relevant
/// cached no-fit boundary vertex. Exactly feasible candidates are ranked by y
/// then x. This is a deterministic proposal heuristic, not an optimality proof;
/// [`IrregularBottomLeftReport2::replay`] remains authoritative.
pub fn bottom_left_irregular_2d(
    bin: &SheetBin2,
    prepared: &PreparedIrregularPacking2,
) -> IrregularPackResult2<IrregularBottomLeftReport2> {
    let mut placements: Vec<IrregularSheetPlacement2> = Vec::new();
    let mut unplaced = Vec::new();
    let mut candidates_tested = 0;
    let mut cache_boundary_candidates = 0;
    let mut unknown_candidates = 0;
    let mut uncertain_orderings = 0;

    for item in prepared.items.values() {
        let Some((min_x, min_y, _, _)) = contour_bounds(&item.shape) else {
            unplaced.push(item.id.clone());
            continue;
        };
        let base_x = -min_x;
        let base_y = -min_y;
        let mut candidates = vec![(base_x.clone(), base_y.clone())];
        for placed in &placements {
            let pair = canonical_pair(prepared, &placed.item, &item.id);
            let Some(obstacle) = pair.obstacle() else {
                continue;
            };
            let candidate_is_moving = pair.moving_item() == &item.id;
            for segment in obstacle.boundary().segments() {
                let vertex = segment.start();
                let (x, y) = if candidate_is_moving {
                    (&placed.x + vertex.x(), &placed.y + vertex.y())
                } else {
                    (&placed.x - vertex.x(), &placed.y - vertex.y())
                };
                candidates.push((x.clone(), y.clone()));
                candidates.push((x, base_y.clone()));
                candidates.push((base_x.clone(), y));
                cache_boundary_candidates += 3;
            }
        }

        let mut best: Option<IrregularSheetPlacement2> = None;
        for (x, y) in candidates {
            candidates_tested += 1;
            let candidate = IrregularSheetPlacement2 {
                item: item.id.clone(),
                x,
                y,
            };
            let mut proposal = placements.clone();
            proposal.push(candidate.clone());
            match verify_irregular_packing_2d(bin, prepared, &proposal)?.status {
                FeasibilityStatus::Infeasible => {}
                FeasibilityStatus::Unknown => unknown_candidates += 1,
                FeasibilityStatus::Feasible => match &best {
                    None => best = Some(candidate),
                    Some(current) => match compare_bottom_left(&candidate, current) {
                        Some(Ordering::Less) => best = Some(candidate),
                        Some(Ordering::Equal | Ordering::Greater) => {}
                        None => uncertain_orderings += 1,
                    },
                },
            }
        }
        match best {
            Some(placement) => placements.push(placement),
            None => unplaced.push(item.id.clone()),
        }
    }

    let replay = verify_irregular_packing_2d(bin, prepared, &placements)?;
    Ok(IrregularBottomLeftReport2 {
        placements,
        unplaced,
        candidates_tested,
        cache_boundary_candidates,
        unknown_candidates,
        uncertain_orderings,
        replay,
    })
}

/// Replay translations against sheet containment and the prepared no-fit cache.
pub fn verify_irregular_packing_2d(
    bin: &SheetBin2,
    prepared: &PreparedIrregularPacking2,
    placements: &[IrregularSheetPlacement2],
) -> IrregularPackResult2<IrregularSheetVerification2> {
    let policy = CurvePolicy::certified();
    let mut status = FeasibilityStatus::Feasible;
    let mut containment_checks = 0;
    let mut no_overlap_checks = 0;
    let mut facts = Vec::new();

    for placement in placements {
        let item = prepared
            .items
            .get(&placement.item)
            .ok_or(PackError::MissingItem)?;
        containment_checks += 1;
        match contained_in_sheet(bin, item, placement) {
            Some(true) => {}
            Some(false) => {
                status = FeasibilityStatus::Infeasible;
                facts.push(format!("{} outside sheet", placement.item.as_str()));
            }
            None => {
                if status != FeasibilityStatus::Infeasible {
                    status = FeasibilityStatus::Unknown;
                }
                facts.push(format!(
                    "{} sheet containment was not exactly decidable",
                    placement.item.as_str()
                ));
            }
        }
    }

    if status != FeasibilityStatus::Infeasible {
        for left_index in 0..placements.len() {
            for right_index in (left_index + 1)..placements.len() {
                let left = &placements[left_index];
                let right = &placements[right_index];
                if left.item == right.item {
                    continue;
                }
                no_overlap_checks += 1;
                match classify_pair(prepared, left, right, &policy)? {
                    PairStatus::SeparatedOrTouching => {}
                    PairStatus::Overlapping => {
                        status = FeasibilityStatus::Infeasible;
                        facts.push(format!(
                            "{} overlaps {}",
                            left.item.as_str(),
                            right.item.as_str()
                        ));
                    }
                    PairStatus::Unknown => {
                        status = FeasibilityStatus::Unknown;
                        facts.push(format!(
                            "{} / {} no-fit classification is unavailable",
                            left.item.as_str(),
                            right.item.as_str()
                        ));
                    }
                }
            }
        }
    }

    let mut placement_counts = BTreeMap::<ItemId, usize>::new();
    for placement in placements {
        *placement_counts.entry(placement.item.clone()).or_default() += 1;
    }
    let mut unplaced = Vec::new();
    let mut duplicates = Vec::new();
    let mut used_area = Some(Real::zero());
    let mut placed_items = 0;
    for item in prepared.items.values() {
        let count = placement_counts.get(&item.id).copied().unwrap_or(0);
        match count {
            0 => unplaced.push(item.id.clone()),
            _ => {
                placed_items += 1;
                if count > 1 {
                    duplicates.push(item.id.clone());
                }
                used_area = match (used_area, absolute_contour_area(&item.shape)?) {
                    (Some(total), Some(area)) => Some(total + area * Real::from(count as i64)),
                    _ => None,
                };
            }
        }
    }
    if !duplicates.is_empty() {
        status = FeasibilityStatus::Infeasible;
        for duplicate in &duplicates {
            facts.push(format!("{} placed more than once", duplicate.as_str()));
        }
    }
    if used_area.is_none() && status != FeasibilityStatus::Infeasible {
        status = FeasibilityStatus::Unknown;
        facts.push("one or more contour areas were unavailable".to_string());
    }

    let bin_area = bin.size.area();
    let waste_area = used_area.as_ref().map(|used_area| &bin_area - used_area);
    Ok(IrregularSheetVerification2 {
        status,
        containment_checks,
        no_overlap_checks,
        objective: IrregularSheetObjective2 {
            bin_area,
            used_area,
            waste_area,
            placed_items,
            unplaced_items: unplaced.len(),
            duplicate_placements: placement_counts
                .values()
                .map(|count| count.saturating_sub(1))
                .sum(),
        },
        unplaced,
        duplicates,
        facts,
    })
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum PairStatus {
    SeparatedOrTouching,
    Overlapping,
    Unknown,
}

fn canonical_pair<'a>(
    prepared: &'a PreparedIrregularPacking2,
    left: &ItemId,
    right: &ItemId,
) -> &'a PreparedNoFitPair2 {
    let key = if left < right {
        (left.clone(), right.clone())
    } else {
        (right.clone(), left.clone())
    };
    prepared
        .pairs
        .get(&key)
        .expect("prepared cache contains every distinct item pair")
}

fn classify_pair(
    prepared: &PreparedIrregularPacking2,
    left: &IrregularSheetPlacement2,
    right: &IrregularSheetPlacement2,
    policy: &CurvePolicy,
) -> IrregularPackResult2<PairStatus> {
    let (fixed, moving) = if left.item < right.item {
        (left, right)
    } else {
        (right, left)
    };
    let pair = canonical_pair(prepared, &fixed.item, &moving.item);
    let Some(obstacle) = pair.obstacle() else {
        return Ok(PairStatus::Unknown);
    };
    let translation = Point2::new(&moving.x - &fixed.x, &moving.y - &fixed.y);
    Ok(match obstacle.classify_translation(&translation, policy)? {
        Classification::Decided(ContourPointLocation::Inside) => PairStatus::Overlapping,
        Classification::Decided(ContourPointLocation::Boundary | ContourPointLocation::Outside) => {
            PairStatus::SeparatedOrTouching
        }
        Classification::Uncertain(_) => PairStatus::Unknown,
    })
}

fn contour_bounds(contour: &Contour2) -> Option<(Real, Real, Real, Real)> {
    let first = contour.segments().first()?.start();
    let mut min_x = first.x().clone();
    let mut min_y = first.y().clone();
    let mut max_x = first.x().clone();
    let mut max_y = first.y().clone();
    for segment in contour.segments() {
        if !matches!(segment, Segment2::Line(_)) {
            return None;
        }
        let point = segment.start();
        update_min(&mut min_x, point.x())?;
        update_min(&mut min_y, point.y())?;
        update_max(&mut max_x, point.x())?;
        update_max(&mut max_y, point.y())?;
    }
    Some((min_x, min_y, max_x, max_y))
}

fn update_min(current: &mut Real, candidate: &Real) -> Option<()> {
    if real_cmp(candidate, current)? == Ordering::Less {
        *current = candidate.clone();
    }
    Some(())
}

fn update_max(current: &mut Real, candidate: &Real) -> Option<()> {
    if real_cmp(candidate, current)? == Ordering::Greater {
        *current = candidate.clone();
    }
    Some(())
}

fn compare_bottom_left(
    left: &IrregularSheetPlacement2,
    right: &IrregularSheetPlacement2,
) -> Option<Ordering> {
    match real_cmp(&left.y, &right.y)? {
        Ordering::Equal => real_cmp(&left.x, &right.x),
        ordering => Some(ordering),
    }
}

fn real_cmp(left: &Real, right: &Real) -> Option<Ordering> {
    match (left - right).refine_sign_until(-64)? {
        RealSign::Negative => Some(Ordering::Less),
        RealSign::Zero => Some(Ordering::Equal),
        RealSign::Positive => Some(Ordering::Greater),
    }
}

fn contained_in_sheet(
    bin: &SheetBin2,
    item: &IrregularSheetItem2,
    placement: &IrregularSheetPlacement2,
) -> Option<bool> {
    for segment in item.shape.segments() {
        let Segment2::Line(line) = segment else {
            return None;
        };
        let x = line.start().x() + &placement.x;
        let y = line.start().y() + &placement.y;
        let remaining_x = &bin.size.x - &x;
        let remaining_y = &bin.size.y - &y;
        for value in [&x, &y, &remaining_x, &remaining_y] {
            match value.refine_sign_until(-64)? {
                RealSign::Negative => return Some(false),
                RealSign::Zero | RealSign::Positive => {}
            }
        }
    }
    Some(true)
}

fn absolute_contour_area(contour: &Contour2) -> Result<Option<Real>, CurveError> {
    let Some(area) = contour.signed_area()? else {
        return Ok(None);
    };
    Ok(match area.refine_sign_until(-64) {
        Some(RealSign::Negative) => Some(-area),
        Some(RealSign::Zero | RealSign::Positive) => Some(area),
        None => None,
    })
}

#[cfg(test)]
mod tests {
    use hypercurve::{Contour2, LineSeg2, Point2, Segment2};

    use super::*;
    use crate::Rect2;

    fn r(value: i64) -> Real {
        Real::from(value)
    }

    fn id(value: &str) -> ItemId {
        ItemId::new(value).unwrap()
    }

    fn polygon(points: &[(i64, i64)]) -> Contour2 {
        let points = points
            .iter()
            .map(|&(x, y)| Point2::from_values(x, y))
            .collect::<Vec<_>>();
        let segments = (0..points.len())
            .map(|index| {
                Segment2::Line(
                    LineSeg2::try_new(
                        points[index].clone(),
                        points[(index + 1) % points.len()].clone(),
                    )
                    .unwrap(),
                )
            })
            .collect();
        Contour2::try_new(segments).unwrap()
    }

    fn rectangle(name: &str, width: i64, height: i64) -> IrregularSheetItem2 {
        IrregularSheetItem2 {
            id: id(name),
            shape: polygon(&[(0, 0), (width, 0), (width, height), (0, height)]),
        }
    }

    #[test]
    fn prepares_one_canonical_cache_entry_per_unordered_pair() {
        let prepared = prepare_irregular_packing_2d(&[
            rectangle("a", 1, 1),
            rectangle("b", 2, 1),
            rectangle("c", 1, 2),
        ])
        .unwrap();

        assert_eq!(prepared.pairs().len(), 3);
        assert_eq!(prepared.ready_pair_count(), 3);
        assert_eq!(prepared.blocked_pair_count(), 0);
    }

    #[test]
    fn replay_allows_boundary_contact_and_rejects_interior_overlap() {
        let bin = SheetBin2::new(Rect2::new(r(10), r(10)).unwrap());
        let prepared =
            prepare_irregular_packing_2d(&[rectangle("a", 2, 2), rectangle("b", 2, 2)]).unwrap();
        let touching = verify_irregular_packing_2d(
            &bin,
            &prepared,
            &[
                IrregularSheetPlacement2 {
                    item: id("a"),
                    x: r(0),
                    y: r(0),
                },
                IrregularSheetPlacement2 {
                    item: id("b"),
                    x: r(2),
                    y: r(0),
                },
            ],
        )
        .unwrap();
        assert_eq!(touching.status, FeasibilityStatus::Feasible);

        let overlapping = verify_irregular_packing_2d(
            &bin,
            &prepared,
            &[
                IrregularSheetPlacement2 {
                    item: id("a"),
                    x: r(0),
                    y: r(0),
                },
                IrregularSheetPlacement2 {
                    item: id("b"),
                    x: r(1),
                    y: r(0),
                },
            ],
        )
        .unwrap();
        assert_eq!(overlapping.status, FeasibilityStatus::Infeasible);
    }

    #[test]
    fn bottom_left_reuses_no_fit_vertices_and_replays_the_proposal() {
        let bin = SheetBin2::new(Rect2::new(r(4), r(2)).unwrap());
        let prepared =
            prepare_irregular_packing_2d(&[rectangle("a", 2, 2), rectangle("b", 2, 2)]).unwrap();

        let report = bottom_left_irregular_2d(&bin, &prepared).unwrap();

        assert_eq!(report.placements.len(), 2);
        assert!(report.unplaced.is_empty());
        assert!(report.cache_boundary_candidates > 0);
        assert_eq!(report.replay.status, FeasibilityStatus::Feasible);
        assert_eq!(report.placements[0].x, r(0));
        assert_eq!(report.placements[1].x, r(2));
    }

    #[test]
    fn concave_pairs_retain_a_blocker_and_replay_unknown() {
        let concave = IrregularSheetItem2 {
            id: id("concave"),
            shape: polygon(&[(0, 0), (2, 0), (1, 1), (2, 2), (0, 2)]),
        };
        let prepared = prepare_irregular_packing_2d(&[rectangle("box", 1, 1), concave]).unwrap();
        assert_eq!(prepared.ready_pair_count(), 0);
        assert_eq!(prepared.blocked_pair_count(), 1);
        assert!(
            prepared
                .pairs()
                .values()
                .next()
                .unwrap()
                .blocker()
                .is_some()
        );

        let bin = SheetBin2::new(Rect2::new(r(10), r(10)).unwrap());
        let replay = verify_irregular_packing_2d(
            &bin,
            &prepared,
            &[
                IrregularSheetPlacement2 {
                    item: id("box"),
                    x: r(0),
                    y: r(0),
                },
                IrregularSheetPlacement2 {
                    item: id("concave"),
                    x: r(5),
                    y: r(5),
                },
            ],
        )
        .unwrap();
        assert_eq!(replay.status, FeasibilityStatus::Unknown);
    }
}
