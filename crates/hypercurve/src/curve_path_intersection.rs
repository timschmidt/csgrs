//! Retained exact intersections between top-level curve paths.

use std::cell::OnceCell;
use std::rc::Rc;

use crate::curve_intersection::split_curve_spans;
use crate::{
    BezierArrangementFragment2, BezierArrangementGraph2, BezierArrangementTraversal2,
    BezierParameter2, BezierSplitFragment2, BezierSplitMaterialization2, BooleanOp,
    CircleCircleRelation, Classification, ContourPointLocation, Curve2, CurveFamily2,
    CurveGeometry2, CurveIntersectionContact2, CurveIntersectionOverlap2,
    CurveIntersectionPairBlocker2, CurveIntersectionPairBlockerKind2, CurveOperation2, CurvePath2,
    CurvePolicy, CurveRegion2, CurveRegionFragmentProvenance2, CurveResult, CurveSpanProvenance2,
    ExactCurveError, ExactCurveResult, PreparedCurveIntersection2,
    RationalBezierOverlapOrientation2, UncertaintyReason,
};

/// Filled side of an oriented closed curve path.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveBoundaryInteriorSide2 {
    /// Material lies to the left while traversing the path.
    Left,
    /// Material lies to the right while traversing the path.
    Right,
}

/// Operation-aware ownership action for one certified shared path span.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurvePathOverlapAction2 {
    /// The shared image is not part of the regularized result boundary.
    DiscardBoth,
    /// Emit the first path span in its authored direction.
    KeepFirst,
    /// Emit the first path span in reverse direction.
    KeepFirstReversed,
}

/// Provenance-bearing ownership decision for one certified shared path span.
#[derive(Clone, Debug, PartialEq)]
pub struct CurvePathOverlapResolution2 {
    overlap: CurvePathIntersectionOverlap2,
    operation: BooleanOp,
    first_interior_side: CurveBoundaryInteriorSide2,
    second_interior_side: CurveBoundaryInteriorSide2,
    action: CurvePathOverlapAction2,
}

/// Operand that owns one fragment in a top-level curved Boolean report.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurvePathBooleanOperand2 {
    /// Fragment originates in the first path.
    First,
    /// Fragment originates in the second path.
    Second,
}

/// Exact regularized-Boolean action for one retained path fragment.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurvePathBooleanFragmentAction2 {
    /// The source fragment is not part of the result boundary.
    Discard,
    /// Emit the source fragment in its authored direction.
    Keep,
    /// Emit the source fragment in reverse direction.
    KeepReversed,
}

/// One classified split fragment with full authored and promoted provenance.
#[derive(Clone, Debug, PartialEq)]
pub struct CurvePathBooleanFragment2 {
    operand: CurvePathBooleanOperand2,
    family: CurveFamily2,
    curve_index: usize,
    promoted_span_index: usize,
    split_fragment_index: usize,
    arrangement_source_index: usize,
    provenance: CurveSpanProvenance2,
    fragment: BezierSplitFragment2,
    start_topology_vertex: Option<usize>,
    end_topology_vertex: Option<usize>,
    location_in_other: ContourPointLocation,
    action: CurvePathBooleanFragmentAction2,
}

/// Clone-shared exact fragment selection, arrangement, traversal, and region.
#[derive(Clone, Debug)]
pub struct CurvePathBooleanSelection2 {
    data: Rc<CurvePathBooleanSelectionData>,
}

#[derive(Debug)]
struct CurvePathBooleanSelectionData {
    operation: BooleanOp,
    policy: CurvePolicy,
    first_interior_side: CurveBoundaryInteriorSide2,
    second_interior_side: CurveBoundaryInteriorSide2,
    fragments: Rc<[CurvePathBooleanFragment2]>,
    overlap_resolutions: Rc<[CurvePathOverlapResolution2]>,
    arrangement: OnceCell<ExactCurveResult<BezierArrangementGraph2>>,
    traversal: OnceCell<ExactCurveResult<BezierArrangementTraversal2>>,
    region: OnceCell<ExactCurveResult<CurveRegion2>>,
}

/// One path-pair contact with authored curve indices and exact span provenance.
#[derive(Clone, Debug, PartialEq)]
pub struct CurvePathIntersectionContact2 {
    first_curve_index: usize,
    second_curve_index: usize,
    contact: CurveIntersectionContact2,
}

/// One certified shared span between authored curves in two paths.
#[derive(Clone, Debug, PartialEq)]
pub struct CurvePathIntersectionOverlap2 {
    first_curve_index: usize,
    second_curve_index: usize,
    overlap: CurveIntersectionOverlap2,
}

/// One incomplete authored curve pair in a path-pair report.
#[derive(Clone, Debug, PartialEq)]
pub struct CurvePathIntersectionBlocker2 {
    first_curve_index: usize,
    second_curve_index: usize,
    blocker: CurveIntersectionPairBlocker2,
}

/// Clone-shared complete replay report for a prepared path pair.
#[derive(Clone, Debug)]
pub struct CurvePathIntersectionReport2 {
    data: Rc<CurvePathIntersectionReportData>,
}

#[derive(Debug)]
struct CurvePathIntersectionReportData {
    contacts: Rc<[CurvePathIntersectionContact2]>,
    overlaps: Rc<[CurvePathIntersectionOverlap2]>,
    blockers: Rc<[CurvePathIntersectionBlocker2]>,
}

/// Exact split materializations retained for one authored path curve.
#[derive(Clone, Debug)]
pub struct CurvePathSplit2 {
    curve_index: usize,
    materializations: Rc<[BezierSplitMaterialization2]>,
}

/// Clone-shared path-pair split topology and lazy arrangement.
#[derive(Clone, Debug)]
pub struct CurvePathIntersectionTopology2 {
    data: Rc<CurvePathIntersectionTopologyData>,
}

#[derive(Debug)]
struct CurvePathIntersectionTopologyData {
    report: CurvePathIntersectionReport2,
    first: Rc<[CurvePathSplit2]>,
    second: Rc<[CurvePathSplit2]>,
    arrangement: OnceCell<CurveResult<BezierArrangementGraph2>>,
}

/// Prepared path-pair intersection whose curve-pair resultants are computed once.
#[derive(Clone, Debug)]
pub struct PreparedCurvePathIntersection2 {
    data: Rc<PreparedCurvePathIntersectionData>,
}

#[derive(Debug)]
struct PreparedCurvePathIntersectionData {
    first: CurvePath2,
    second: CurvePath2,
    policy: CurvePolicy,
    authored_curve_pair_count: usize,
    pairs: Vec<PreparedCurvePathPair>,
    report: OnceCell<ExactCurveResult<CurvePathIntersectionReport2>>,
    topology: OnceCell<ExactCurveResult<CurvePathIntersectionTopology2>>,
    boolean_selections: [OnceCell<ExactCurveResult<CurvePathBooleanSelection2>>; 16],
}

#[derive(Debug)]
struct PreparedCurvePathPair {
    first_curve_index: usize,
    second_curve_index: usize,
    prepared: PreparedCurveIntersection2,
}

fn curve_pair_bounds_decided_disjoint(
    first: &Curve2,
    second: &Curve2,
    policy: &CurvePolicy,
) -> bool {
    let (Ok(first_bounds), Ok(second_bounds)) = (first.bounds(), second.bounds()) else {
        return false;
    };
    matches!(
        first_bounds.overlaps(second_bounds, policy),
        Classification::Decided(false)
    )
}

impl CurvePath2 {
    /// Prepares every authored curve pair once for exact path topology.
    pub fn try_prepare_intersection(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> ExactCurveResult<PreparedCurvePathIntersection2> {
        let authored_curve_pair_count = self.curves().len().saturating_mul(other.curves().len());
        let candidate_capacity = self
            .curves()
            .len()
            .saturating_add(other.curves().len())
            .min(authored_curve_pair_count);
        let mut pairs = Vec::with_capacity(candidate_capacity);
        for (first_curve_index, first) in self.curves().iter().enumerate() {
            for (second_curve_index, second) in other.curves().iter().enumerate() {
                if authored_curve_pair_count > 1
                    && curve_pair_bounds_decided_disjoint(first, second, policy)
                {
                    continue;
                }
                pairs.push(PreparedCurvePathPair {
                    first_curve_index,
                    second_curve_index,
                    prepared: first.try_prepare_intersection(second, policy)?,
                });
            }
        }
        Ok(PreparedCurvePathIntersection2 {
            data: Rc::new(PreparedCurvePathIntersectionData {
                first: self.clone(),
                second: other.clone(),
                policy: policy.clone(),
                authored_curve_pair_count,
                pairs,
                report: OnceCell::new(),
                topology: OnceCell::new(),
                boolean_selections: std::array::from_fn(|_| OnceCell::new()),
            }),
        })
    }

    /// Computes one exact regularized Boolean region.
    ///
    /// Use [`Self::try_prepare_intersection`] when several operations or side
    /// policies will be evaluated for the same path pair.
    pub fn boolean_region(
        &self,
        other: &Self,
        operation: BooleanOp,
        first_interior_side: CurveBoundaryInteriorSide2,
        second_interior_side: CurveBoundaryInteriorSide2,
        policy: &CurvePolicy,
    ) -> ExactCurveResult<CurveRegion2> {
        self.try_prepare_intersection(other, policy)?
            .boolean_region(operation, first_interior_side, second_interior_side)
    }
}

impl PreparedCurvePathIntersection2 {
    /// Returns the retained first path.
    pub fn first(&self) -> &CurvePath2 {
        &self.data.first
    }

    /// Returns the retained second path.
    pub fn second(&self) -> &CurvePath2 {
        &self.data.second
    }

    /// Returns the exact policy captured by preparation.
    pub fn policy(&self) -> &CurvePolicy {
        &self.data.policy
    }

    /// Returns the Cartesian authored curve-pair count before broad-phase filtering.
    pub fn authored_curve_pair_count(&self) -> usize {
        self.data.authored_curve_pair_count
    }

    /// Returns the curve-pair count retained after certified AABB filtering.
    pub fn candidate_curve_pair_count(&self) -> usize {
        self.data.pairs.len()
    }

    /// Returns whether the combined report has already been retained.
    pub fn is_report_cached(&self) -> bool {
        self.data.report.get().is_some()
    }

    /// Returns a clone-shared combined path-pair report.
    pub fn report(&self) -> ExactCurveResult<CurvePathIntersectionReport2> {
        self.report_view().cloned()
    }

    /// Borrows the combined path-pair report without copying its records.
    pub fn report_view(&self) -> ExactCurveResult<&CurvePathIntersectionReport2> {
        match self.data.report.get_or_init(|| self.build_report()) {
            Ok(report) => Ok(report),
            Err(error) => Err(error.clone()),
        }
    }

    /// Returns whether aggregate split topology has already been retained.
    pub fn is_topology_cached(&self) -> bool {
        self.data.topology.get().is_some()
    }

    /// Returns clone-shared aggregate split topology.
    pub fn topology(&self) -> ExactCurveResult<CurvePathIntersectionTopology2> {
        self.topology_view().cloned()
    }

    /// Borrows aggregate split topology without copying fragments.
    pub fn topology_view(&self) -> ExactCurveResult<&CurvePathIntersectionTopology2> {
        match self.data.topology.get_or_init(|| self.build_topology()) {
            Ok(topology) => Ok(topology),
            Err(error) => Err(error.clone()),
        }
    }

    /// Returns whether this operation and side-policy selection is already retained.
    pub fn is_boolean_selection_cached(
        &self,
        operation: BooleanOp,
        first_interior_side: CurveBoundaryInteriorSide2,
        second_interior_side: CurveBoundaryInteriorSide2,
    ) -> bool {
        self.data.boolean_selections
            [boolean_selection_index(operation, first_interior_side, second_interior_side)]
        .get()
        .is_some()
    }

    /// Returns a clone-shared operation-aware curved Boolean selection.
    pub fn boolean_selection(
        &self,
        operation: BooleanOp,
        first_interior_side: CurveBoundaryInteriorSide2,
        second_interior_side: CurveBoundaryInteriorSide2,
    ) -> ExactCurveResult<CurvePathBooleanSelection2> {
        self.boolean_selection_view(operation, first_interior_side, second_interior_side)
            .cloned()
    }

    /// Borrows a retained operation-aware curved Boolean selection.
    pub fn boolean_selection_view(
        &self,
        operation: BooleanOp,
        first_interior_side: CurveBoundaryInteriorSide2,
        second_interior_side: CurveBoundaryInteriorSide2,
    ) -> ExactCurveResult<&CurvePathBooleanSelection2> {
        let slot = &self.data.boolean_selections
            [boolean_selection_index(operation, first_interior_side, second_interior_side)];
        match slot.get_or_init(|| {
            self.build_boolean_selection(operation, first_interior_side, second_interior_side)
        }) {
            Ok(selection) => Ok(selection),
            Err(error) => Err(error.clone()),
        }
    }

    /// Returns a clone-shared exact regularized Boolean region.
    pub fn boolean_region(
        &self,
        operation: BooleanOp,
        first_interior_side: CurveBoundaryInteriorSide2,
        second_interior_side: CurveBoundaryInteriorSide2,
    ) -> ExactCurveResult<CurveRegion2> {
        self.boolean_region_view(operation, first_interior_side, second_interior_side)
            .cloned()
    }

    /// Borrows the cached exact regularized Boolean region.
    pub fn boolean_region_view(
        &self,
        operation: BooleanOp,
        first_interior_side: CurveBoundaryInteriorSide2,
        second_interior_side: CurveBoundaryInteriorSide2,
    ) -> ExactCurveResult<&CurveRegion2> {
        self.boolean_selection_view(operation, first_interior_side, second_interior_side)?
            .region_view()
    }

    fn build_report(&self) -> ExactCurveResult<CurvePathIntersectionReport2> {
        let pair_count = self.data.pairs.len();
        let mut contacts = Vec::with_capacity(pair_count);
        let mut overlaps = Vec::with_capacity(pair_count);
        let mut blockers = Vec::with_capacity(pair_count);
        for pair in &self.data.pairs {
            let report = pair.prepared.report_view()?;
            contacts.extend(report.contacts().iter().cloned().map(|contact| {
                CurvePathIntersectionContact2 {
                    first_curve_index: pair.first_curve_index,
                    second_curve_index: pair.second_curve_index,
                    contact,
                }
            }));
            overlaps.extend(report.overlaps().iter().cloned().map(|overlap| {
                CurvePathIntersectionOverlap2 {
                    first_curve_index: pair.first_curve_index,
                    second_curve_index: pair.second_curve_index,
                    overlap,
                }
            }));
            blockers.extend(report.blockers().iter().cloned().map(|blocker| {
                CurvePathIntersectionBlocker2 {
                    first_curve_index: pair.first_curve_index,
                    second_curve_index: pair.second_curve_index,
                    blocker,
                }
            }));
        }
        Ok(CurvePathIntersectionReport2 {
            data: Rc::new(CurvePathIntersectionReportData {
                contacts: contacts.into(),
                overlaps: overlaps.into(),
                blockers: blockers.into(),
            }),
        })
    }

    fn build_topology(&self) -> ExactCurveResult<CurvePathIntersectionTopology2> {
        let report = self.report_view()?.clone();
        if let Some(blocker) = report.blockers().first() {
            let reason = match blocker.blocker().kind() {
                CurveIntersectionPairBlockerKind2::Uncertain(reason) => *reason,
                CurveIntersectionPairBlockerKind2::IncompleteReplay { .. } => {
                    UncertaintyReason::Predicate
                }
                CurveIntersectionPairBlockerKind2::SharedComponent => UncertaintyReason::Boundary,
            };
            return Err(ExactCurveError::blocked(
                CurveOperation2::Arrangement,
                self.data.first.curves()[blocker.first_curve_index].family(),
                self.data.first.curves()[blocker.first_curve_index].source(),
                reason,
            ));
        }
        let first = split_path(
            &self.data.first,
            report
                .contacts()
                .iter()
                .map(|contact| {
                    (
                        contact.first_curve_index(),
                        contact.contact().first().promoted_span_index(),
                        contact.contact().first().local_parameter().clone(),
                    )
                })
                .chain(report.overlaps().iter().flat_map(|overlap| {
                    [
                        (
                            overlap.first_curve_index(),
                            overlap.overlap().first_span_index(),
                            overlap.overlap().first_range().start().clone(),
                        ),
                        (
                            overlap.first_curve_index(),
                            overlap.overlap().first_span_index(),
                            overlap.overlap().first_range().end().clone(),
                        ),
                    ]
                })),
            &self.data.policy,
        )?;
        let second = split_path(
            &self.data.second,
            report
                .contacts()
                .iter()
                .map(|contact| {
                    (
                        contact.second_curve_index(),
                        contact.contact().second().promoted_span_index(),
                        contact.contact().second().local_parameter().clone(),
                    )
                })
                .chain(report.overlaps().iter().flat_map(|overlap| {
                    [
                        (
                            overlap.second_curve_index(),
                            overlap.overlap().second_span_index(),
                            overlap.overlap().second_range().start().clone(),
                        ),
                        (
                            overlap.second_curve_index(),
                            overlap.overlap().second_span_index(),
                            overlap.overlap().second_range().end().clone(),
                        ),
                    ]
                })),
            &self.data.policy,
        )?;
        Ok(CurvePathIntersectionTopology2 {
            data: Rc::new(CurvePathIntersectionTopologyData {
                report,
                first: first.into(),
                second: second.into(),
                arrangement: OnceCell::new(),
            }),
        })
    }

    fn build_boolean_selection(
        &self,
        operation: BooleanOp,
        first_interior_side: CurveBoundaryInteriorSide2,
        second_interior_side: CurveBoundaryInteriorSide2,
    ) -> ExactCurveResult<CurvePathBooleanSelection2> {
        let topology = self.topology_view()?;
        let overlap_resolutions = topology.report().resolve_overlap_ownership(
            operation,
            first_interior_side,
            second_interior_side,
        );
        let path_bounds_disjoint = match (self.data.first.bounds(), self.data.second.bounds()) {
            (Ok(first), Ok(second)) => matches!(
                first.overlaps(second, &self.data.policy),
                Classification::Decided(false)
            ),
            _ => false,
        };
        let first_source_count = topology
            .first()
            .iter()
            .map(|split| split.materializations().len())
            .sum();
        let fragment_capacity = topology
            .first()
            .iter()
            .chain(topology.second())
            .flat_map(CurvePathSplit2::materializations)
            .map(|materialization| materialization.fragments().len())
            .sum();
        let mut fragments = Vec::with_capacity(fragment_capacity);
        append_boolean_fragments(
            &mut fragments,
            &self.data.first,
            topology.first(),
            &self.data.second,
            topology.report(),
            CurvePathBooleanOperand2::First,
            operation,
            first_interior_side,
            second_interior_side,
            &overlap_resolutions,
            path_bounds_disjoint,
            0,
            &self.data.policy,
        )?;
        let first_fragment_count = fragments.len();
        append_boolean_fragments(
            &mut fragments,
            &self.data.second,
            topology.second(),
            &self.data.first,
            topology.report(),
            CurvePathBooleanOperand2::Second,
            operation,
            first_interior_side,
            second_interior_side,
            &overlap_resolutions,
            path_bounds_disjoint,
            first_source_count,
            &self.data.policy,
        )?;
        propagate_path_junction_topology_vertices(
            &mut fragments,
            0..first_fragment_count,
            self.data.first.start() == self.data.first.end(),
        );
        let fragment_count = fragments.len();
        propagate_path_junction_topology_vertices(
            &mut fragments,
            first_fragment_count..fragment_count,
            self.data.second.start() == self.data.second.end(),
        );
        Ok(CurvePathBooleanSelection2 {
            data: Rc::new(CurvePathBooleanSelectionData {
                operation,
                policy: self.data.policy.clone(),
                first_interior_side,
                second_interior_side,
                fragments: fragments.into(),
                overlap_resolutions: overlap_resolutions.into(),
                arrangement: OnceCell::new(),
                traversal: OnceCell::new(),
                region: OnceCell::new(),
            }),
        })
    }
}

fn propagate_path_junction_topology_vertices(
    fragments: &mut [CurvePathBooleanFragment2],
    range: std::ops::Range<usize>,
    closed: bool,
) {
    if range.len() < 2 {
        return;
    }
    for left_index in range.start..range.end - 1 {
        merge_adjacent_topology_vertices(fragments, left_index, left_index + 1);
    }
    if closed {
        merge_adjacent_topology_vertices(fragments, range.end - 1, range.start);
    }
}

fn merge_adjacent_topology_vertices(
    fragments: &mut [CurvePathBooleanFragment2],
    left_index: usize,
    right_index: usize,
) {
    let left = fragments[left_index].end_topology_vertex;
    let right = fragments[right_index].start_topology_vertex;
    match (left, right) {
        (Some(vertex), None) => fragments[right_index].start_topology_vertex = Some(vertex),
        (None, Some(vertex)) => fragments[left_index].end_topology_vertex = Some(vertex),
        (Some(left), Some(right)) if left != right => {
            let retained = left.min(right);
            let replaced = left.max(right);
            for fragment in fragments {
                if fragment.start_topology_vertex == Some(replaced) {
                    fragment.start_topology_vertex = Some(retained);
                }
                if fragment.end_topology_vertex == Some(replaced) {
                    fragment.end_topology_vertex = Some(retained);
                }
            }
        }
        _ => {}
    }
}

impl CurvePathIntersectionContact2 {
    /// Returns the authored curve index in the first path.
    pub const fn first_curve_index(&self) -> usize {
        self.first_curve_index
    }

    /// Returns the authored curve index in the second path.
    pub const fn second_curve_index(&self) -> usize {
        self.second_curve_index
    }

    /// Returns the exact curve-pair contact and source-span provenance.
    pub const fn contact(&self) -> &CurveIntersectionContact2 {
        &self.contact
    }
}

impl CurvePathIntersectionOverlap2 {
    /// Returns the authored curve index in the first path.
    pub const fn first_curve_index(&self) -> usize {
        self.first_curve_index
    }

    /// Returns the authored curve index in the second path.
    pub const fn second_curve_index(&self) -> usize {
        self.second_curve_index
    }

    /// Returns the certified overlap and source-span provenance.
    pub const fn overlap(&self) -> &CurveIntersectionOverlap2 {
        &self.overlap
    }
}

impl CurvePathIntersectionBlocker2 {
    /// Returns the authored curve index in the first path.
    pub const fn first_curve_index(&self) -> usize {
        self.first_curve_index
    }

    /// Returns the authored curve index in the second path.
    pub const fn second_curve_index(&self) -> usize {
        self.second_curve_index
    }

    /// Returns the exact blocked span pair and its provenance.
    pub const fn blocker(&self) -> &CurveIntersectionPairBlocker2 {
        &self.blocker
    }
}

impl CurvePathIntersectionReport2 {
    /// Returns contacts in deterministic authored curve-pair order.
    pub fn contacts(&self) -> &[CurvePathIntersectionContact2] {
        &self.data.contacts
    }

    /// Returns certified positive-length overlaps.
    pub fn overlaps(&self) -> &[CurvePathIntersectionOverlap2] {
        &self.data.overlaps
    }

    /// Returns all promoted span pairs that remain incomplete.
    pub fn blockers(&self) -> &[CurvePathIntersectionBlocker2] {
        &self.data.blockers
    }

    /// Returns true when every authored curve pair has complete replay.
    pub fn is_complete(&self) -> bool {
        self.data.blockers.is_empty()
    }

    /// Returns true when complete replay found no contacts or overlaps.
    pub fn is_disjoint(&self) -> bool {
        self.is_complete() && self.data.contacts.is_empty() && self.data.overlaps.is_empty()
    }

    /// Resolves every certified shared span using exact regularized-Boolean side logic.
    ///
    /// The first operand wins deterministic provenance when a shared image remains on
    /// the output boundary. The action records whether its authored direction already
    /// places result material on the left or must be reversed.
    pub fn resolve_overlap_ownership(
        &self,
        operation: BooleanOp,
        first_interior_side: CurveBoundaryInteriorSide2,
        second_interior_side: CurveBoundaryInteriorSide2,
    ) -> Vec<CurvePathOverlapResolution2> {
        self.data
            .overlaps
            .iter()
            .cloned()
            .map(|overlap| {
                let second_side_in_first_direction = match overlap.overlap().orientation() {
                    RationalBezierOverlapOrientation2::Same => second_interior_side,
                    RationalBezierOverlapOrientation2::Reversed => second_interior_side.opposite(),
                };
                let first_left = first_interior_side == CurveBoundaryInteriorSide2::Left;
                let second_left =
                    second_side_in_first_direction == CurveBoundaryInteriorSide2::Left;
                let result_left = operation.apply(first_left, second_left);
                let result_right = operation.apply(!first_left, !second_left);
                let action = match (result_left, result_right) {
                    (true, false) => CurvePathOverlapAction2::KeepFirst,
                    (false, true) => CurvePathOverlapAction2::KeepFirstReversed,
                    (false, false) | (true, true) => CurvePathOverlapAction2::DiscardBoth,
                };
                CurvePathOverlapResolution2 {
                    overlap,
                    operation,
                    first_interior_side,
                    second_interior_side,
                    action,
                }
            })
            .collect()
    }
}

impl CurveBoundaryInteriorSide2 {
    const fn opposite(self) -> Self {
        match self {
            Self::Left => Self::Right,
            Self::Right => Self::Left,
        }
    }
}

impl CurvePathOverlapResolution2 {
    /// Returns the certified shared span and complete source provenance.
    pub const fn overlap(&self) -> &CurvePathIntersectionOverlap2 {
        &self.overlap
    }

    /// Returns the regularized Boolean operation used for ownership.
    pub fn operation(&self) -> BooleanOp {
        self.operation
    }

    /// Returns the declared filled side of the first path.
    pub fn first_interior_side(&self) -> CurveBoundaryInteriorSide2 {
        self.first_interior_side
    }

    /// Returns the declared filled side of the second path.
    pub fn second_interior_side(&self) -> CurveBoundaryInteriorSide2 {
        self.second_interior_side
    }

    /// Returns the deterministic boundary ownership action.
    pub const fn action(&self) -> CurvePathOverlapAction2 {
        self.action
    }
}

impl CurvePathBooleanFragment2 {
    /// Returns the source operand.
    pub const fn operand(&self) -> CurvePathBooleanOperand2 {
        self.operand
    }

    /// Returns the source curve family.
    pub const fn family(&self) -> CurveFamily2 {
        self.family
    }

    /// Returns the authored curve index within the source path.
    pub const fn curve_index(&self) -> usize {
        self.curve_index
    }

    /// Returns the promoted native span index within the authored curve.
    pub const fn promoted_span_index(&self) -> usize {
        self.promoted_span_index
    }

    /// Returns the fragment index within the promoted span's split materialization.
    pub const fn split_fragment_index(&self) -> usize {
        self.split_fragment_index
    }

    /// Returns the stable source index used by the selected arrangement.
    pub const fn arrangement_source_index(&self) -> usize {
        self.arrangement_source_index
    }

    /// Returns exact authored source and parameter-range provenance.
    pub const fn provenance(&self) -> &CurveSpanProvenance2 {
        &self.provenance
    }

    /// Returns the retained exact split fragment.
    pub const fn fragment(&self) -> &BezierSplitFragment2 {
        &self.fragment
    }

    /// Returns the exact representative-point location against the other boundary.
    pub const fn location_in_other(&self) -> ContourPointLocation {
        self.location_in_other
    }

    /// Returns the regularized-Boolean action.
    pub const fn action(&self) -> CurvePathBooleanFragmentAction2 {
        self.action
    }
}

impl CurvePathBooleanSelection2 {
    /// Returns the selected regularized Boolean operation.
    pub fn operation(&self) -> BooleanOp {
        self.data.operation
    }

    /// Returns the declared filled side of the first path.
    pub fn first_interior_side(&self) -> CurveBoundaryInteriorSide2 {
        self.data.first_interior_side
    }

    /// Returns the declared filled side of the second path.
    pub fn second_interior_side(&self) -> CurveBoundaryInteriorSide2 {
        self.data.second_interior_side
    }

    /// Returns every classified source fragment, including discarded fragments.
    pub fn fragments(&self) -> &[CurvePathBooleanFragment2] {
        &self.data.fragments
    }

    /// Returns the shared-span ownership decisions consumed by this selection.
    pub fn overlap_resolutions(&self) -> &[CurvePathOverlapResolution2] {
        &self.data.overlap_resolutions
    }

    /// Returns the number of emitted boundary fragments.
    pub fn kept_fragment_count(&self) -> usize {
        self.data
            .fragments
            .iter()
            .filter(|fragment| fragment.action != CurvePathBooleanFragmentAction2::Discard)
            .count()
    }

    /// Returns whether selected arrangement assembly has already been retained.
    pub fn is_arrangement_cached(&self) -> bool {
        self.data.arrangement.get().is_some()
    }

    /// Borrows the exact selected-fragment arrangement.
    pub fn arrangement_graph_view(&self) -> ExactCurveResult<&BezierArrangementGraph2> {
        match self.data.arrangement.get_or_init(|| {
            let mut fragments = Vec::with_capacity(self.kept_fragment_count());
            for source in self
                .data
                .fragments
                .iter()
                .filter(|fragment| fragment.action != CurvePathBooleanFragmentAction2::Discard)
            {
                let fragment = match source.action {
                    CurvePathBooleanFragmentAction2::Discard => unreachable!(),
                    CurvePathBooleanFragmentAction2::Keep => source.fragment.clone(),
                    CurvePathBooleanFragmentAction2::KeepReversed => source
                        .fragment
                        .reversed()
                        .map_err(|cause| selection_invalid_error(source, cause))?,
                };
                let (start_topology_vertex, end_topology_vertex) = match source.action {
                    CurvePathBooleanFragmentAction2::Keep => {
                        (source.start_topology_vertex, source.end_topology_vertex)
                    }
                    CurvePathBooleanFragmentAction2::KeepReversed => {
                        (source.end_topology_vertex, source.start_topology_vertex)
                    }
                    CurvePathBooleanFragmentAction2::Discard => unreachable!(),
                };
                fragments.push(
                    BezierArrangementFragment2::new(
                        source.arrangement_source_index,
                        source.split_fragment_index,
                        fragment,
                    )
                    .with_topology_vertices(start_topology_vertex, end_topology_vertex),
                );
            }
            BezierArrangementGraph2::new(fragments)
                .map_err(|cause| selection_invalid_error_from_first(&self.data.fragments, cause))
        }) {
            Ok(graph) => Ok(graph),
            Err(error) => Err(error.clone()),
        }
    }

    /// Returns whether exact selected traversal has already been retained.
    pub fn is_traversal_cached(&self) -> bool {
        self.data.traversal.get().is_some()
    }

    /// Borrows exact selected boundary traversal.
    pub fn traversal_view(&self) -> ExactCurveResult<&BezierArrangementTraversal2> {
        match self.data.traversal.get_or_init(|| {
            match self
                .arrangement_graph_view()?
                .traverse_retained_with_tangent_order(&self.data.policy)
            {
                Classification::Decided(traversal) => Ok(traversal),
                Classification::Uncertain(reason) => Err(selection_blocked_error_from_first(
                    &self.data.fragments,
                    reason,
                )),
            }
        }) {
            Ok(traversal) => Ok(traversal),
            Err(error) => Err(error.clone()),
        }
    }

    /// Returns whether retained region materialization has already been retained.
    pub fn is_region_cached(&self) -> bool {
        self.data.region.get().is_some()
    }

    /// Borrows the exact retained curved Boolean region.
    pub fn region_view(&self) -> ExactCurveResult<&CurveRegion2> {
        match self.data.region.get_or_init(|| {
            let graph = self.arrangement_graph_view()?;
            let traversal = self.traversal_view()?;
            match CurveRegion2::from_retained_arrangement_traversal(graph, traversal) {
                Classification::Decided(region) => {
                    let provenance = self
                        .data
                        .fragments
                        .iter()
                        .filter(|fragment| {
                            fragment.action != CurvePathBooleanFragmentAction2::Discard
                        })
                        .enumerate()
                        .map(|(arrangement_fragment_index, fragment)| {
                            CurveRegionFragmentProvenance2::new(
                                arrangement_fragment_index,
                                fragment.arrangement_source_index,
                                Some(fragment.operand),
                                match fragment.operand {
                                    CurvePathBooleanOperand2::First => 0,
                                    CurvePathBooleanOperand2::Second => 1,
                                },
                                fragment.family,
                                fragment.curve_index,
                                fragment.promoted_span_index,
                                fragment.split_fragment_index,
                                fragment.provenance.clone(),
                                fragment.action == CurvePathBooleanFragmentAction2::KeepReversed,
                            )
                        })
                        .collect();
                    region
                        .with_certified_filled_side_is_left(vec![true; traversal.chains().len()])
                        .map_err(|cause| {
                            selection_invalid_error_from_first(&self.data.fragments, cause)
                        })?
                        .with_fragment_provenance(provenance)
                        .map_err(|cause| {
                            selection_invalid_error_from_first(&self.data.fragments, cause)
                        })
                }
                Classification::Uncertain(reason) => Err(selection_blocked_error_from_first(
                    &self.data.fragments,
                    reason,
                )),
            }
        }) {
            Ok(region) => Ok(region),
            Err(error) => Err(error.clone()),
        }
    }
}

impl CurvePathSplit2 {
    /// Returns the authored path curve index.
    pub const fn curve_index(&self) -> usize {
        self.curve_index
    }

    /// Returns split materializations in promoted source-span order.
    pub fn materializations(&self) -> &[BezierSplitMaterialization2] {
        &self.materializations
    }
}

impl CurvePathIntersectionTopology2 {
    /// Returns the complete report that generated this topology.
    pub fn report(&self) -> &CurvePathIntersectionReport2 {
        &self.data.report
    }

    /// Returns split topology for authored curves in the first path.
    pub fn first(&self) -> &[CurvePathSplit2] {
        &self.data.first
    }

    /// Returns split topology for authored curves in the second path.
    pub fn second(&self) -> &[CurvePathSplit2] {
        &self.data.second
    }

    /// Returns whether the combined arrangement has already been retained.
    pub fn is_arrangement_cached(&self) -> bool {
        self.data.arrangement.get().is_some()
    }

    /// Borrows the lazily assembled aggregate arrangement graph.
    pub fn arrangement_graph_view(&self) -> CurveResult<&BezierArrangementGraph2> {
        match self.data.arrangement.get_or_init(|| {
            let materializations = self
                .data
                .first
                .iter()
                .chain(self.data.second.iter())
                .flat_map(CurvePathSplit2::materializations)
                .cloned()
                .collect::<Vec<_>>();
            BezierArrangementGraph2::from_split_materializations(&materializations)
        }) {
            Ok(graph) => Ok(graph),
            Err(error) => Err(error.clone()),
        }
    }

    /// Returns an owned aggregate arrangement graph.
    pub fn arrangement_graph(&self) -> CurveResult<BezierArrangementGraph2> {
        self.arrangement_graph_view().cloned()
    }
}

fn split_path(
    path: &CurvePath2,
    parameters: impl Iterator<Item = (usize, usize, BezierParameter2)>,
    policy: &CurvePolicy,
) -> ExactCurveResult<Vec<CurvePathSplit2>> {
    let mut by_curve = vec![Vec::new(); path.curves().len()];
    for (curve_index, span_index, parameter) in parameters {
        by_curve[curve_index].push((span_index, parameter));
    }
    path.curves()
        .iter()
        .zip(by_curve)
        .enumerate()
        .map(|(curve_index, (curve, parameters))| {
            Ok(CurvePathSplit2 {
                curve_index,
                materializations: split_curve_spans(curve, parameters.into_iter(), policy)?.into(),
            })
        })
        .collect()
}

#[allow(clippy::too_many_arguments)]
fn append_boolean_fragments(
    output: &mut Vec<CurvePathBooleanFragment2>,
    path: &CurvePath2,
    splits: &[CurvePathSplit2],
    other_path: &CurvePath2,
    report: &CurvePathIntersectionReport2,
    operand: CurvePathBooleanOperand2,
    operation: BooleanOp,
    first_interior_side: CurveBoundaryInteriorSide2,
    second_interior_side: CurveBoundaryInteriorSide2,
    overlaps: &[CurvePathOverlapResolution2],
    path_bounds_disjoint: bool,
    source_offset: usize,
    policy: &CurvePolicy,
) -> ExactCurveResult<()> {
    let mut local_source_index = 0_usize;
    for split in splits {
        let curve = &path.curves()[split.curve_index()];
        let native_fragments = curve.native_bezier_fragments()?;
        for (promoted_span_index, materialization) in split.materializations().iter().enumerate() {
            let provenance = native_fragments[promoted_span_index].provenance().clone();
            for (split_fragment_index, fragment) in materialization.fragments().iter().enumerate() {
                let (start, end) = split_fragment_parameter_range(fragment);
                let overlap_action = match overlap_action_for_fragment(
                    operand,
                    split.curve_index(),
                    &provenance,
                    start,
                    end,
                    overlaps,
                    policy,
                )
                .map_err(|cause| {
                    ExactCurveError::invalid(
                        CurveOperation2::Boolean,
                        curve.family(),
                        curve.source(),
                        cause,
                    )
                })? {
                    Classification::Decided(action) => action,
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Boolean,
                            curve.family(),
                            curve.source(),
                            reason,
                        ));
                    }
                };
                let (location_in_other, action) = if let Some(action) = overlap_action {
                    (ContourPointLocation::Boundary, action)
                } else if path_bounds_disjoint {
                    (
                        ContourPointLocation::Outside,
                        boolean_fragment_action(
                            operation,
                            operand,
                            first_interior_side,
                            second_interior_side,
                            false,
                        ),
                    )
                } else {
                    let representative_classification = match (start.as_exact(), end.as_exact()) {
                        (Some(start), Some(end)) => {
                            let midpoint =
                                ((start + end) / crate::Real::from(2_i8)).map_err(|cause| {
                                    ExactCurveError::invalid(
                                        CurveOperation2::Boolean,
                                        curve.family(),
                                        curve.source(),
                                        cause.into(),
                                    )
                                })?;
                            curve.rational_evaluators()?[promoted_span_index]
                                .point_at_classified(&midpoint, policy)
                        }
                        _ => fragment.representative_point(policy).map_err(|cause| {
                            ExactCurveError::invalid(
                                CurveOperation2::Boolean,
                                curve.family(),
                                curve.source(),
                                cause,
                            )
                        })?,
                    };
                    let representative = match representative_classification {
                        Classification::Decided(point) => point,
                        Classification::Uncertain(reason) => {
                            return Err(ExactCurveError::blocked(
                                CurveOperation2::Boolean,
                                curve.family(),
                                curve.source(),
                                reason,
                            ));
                        }
                    };
                    let location_classification = match classify_retained_same_circle_fragment(
                        curve,
                        other_path,
                        &representative,
                        policy,
                    )? {
                        Some(location) => location,
                        None => other_path.classify_point(&representative, policy)?,
                    };
                    let location = match location_classification {
                        Classification::Decided(location) => location,
                        Classification::Uncertain(reason) => {
                            return Err(ExactCurveError::blocked(
                                CurveOperation2::Boolean,
                                curve.family(),
                                curve.source(),
                                reason,
                            ));
                        }
                    };
                    if location == ContourPointLocation::Boundary {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Boolean,
                            curve.family(),
                            curve.source(),
                            UncertaintyReason::Boundary,
                        ));
                    }
                    let other_inside = location == ContourPointLocation::Inside;
                    (
                        location,
                        boolean_fragment_action(
                            operation,
                            operand,
                            first_interior_side,
                            second_interior_side,
                            other_inside,
                        ),
                    )
                };
                output.push(CurvePathBooleanFragment2 {
                    operand,
                    family: curve.family(),
                    curve_index: split.curve_index(),
                    promoted_span_index,
                    split_fragment_index,
                    arrangement_source_index: source_offset + local_source_index,
                    provenance: provenance.clone(),
                    fragment: fragment.clone(),
                    start_topology_vertex: topology_vertex_for_parameter(
                        report,
                        operand,
                        split.curve_index(),
                        promoted_span_index,
                        start,
                    ),
                    end_topology_vertex: topology_vertex_for_parameter(
                        report,
                        operand,
                        split.curve_index(),
                        promoted_span_index,
                        end,
                    ),
                    location_in_other,
                    action,
                });
            }
            local_source_index += 1;
        }
    }
    Ok(())
}

fn classify_retained_same_circle_fragment(
    curve: &crate::Curve2,
    other_path: &CurvePath2,
    representative: &crate::Point2,
    policy: &CurvePolicy,
) -> ExactCurveResult<Option<Classification<ContourPointLocation>>> {
    let CurveGeometry2::CircularArc(source_arc) = curve.geometry() else {
        return Ok(None);
    };
    if other_path.start() != other_path.end() {
        return Ok(None);
    }
    let target_arc = match other_path.curves() {
        [target] => match target.geometry() {
            CurveGeometry2::CircularArc(arc) => arc,
            _ => return Ok(None),
        },
        [first, second] => match (first.geometry(), second.geometry()) {
            (CurveGeometry2::CircularArc(arc), CurveGeometry2::Line(_))
            | (CurveGeometry2::Line(_), CurveGeometry2::CircularArc(arc)) => arc,
            _ => return Ok(None),
        },
        _ => return Ok(None),
    };
    let relation = source_arc
        .circle_relation(target_arc, policy)
        .map_err(|cause| {
            ExactCurveError::invalid(
                CurveOperation2::Boolean,
                curve.family(),
                curve.source(),
                cause,
            )
        })?;
    match relation {
        CircleCircleRelation::Coincident => Ok(Some(
            target_arc
                .contains_sweep_point(representative, policy)
                .map(|on_boundary_arc| {
                    if on_boundary_arc {
                        ContourPointLocation::Boundary
                    } else {
                        ContourPointLocation::Outside
                    }
                }),
        )),
        CircleCircleRelation::Uncertain { reason } => Ok(Some(Classification::Uncertain(reason))),
        CircleCircleRelation::Disjoint
        | CircleCircleRelation::Tangent { .. }
        | CircleCircleRelation::Secant { .. } => Ok(None),
    }
}

fn split_fragment_parameter_range(
    fragment: &BezierSplitFragment2,
) -> (&BezierParameter2, &BezierParameter2) {
    match fragment {
        BezierSplitFragment2::Materialized { start, end, .. }
        | BezierSplitFragment2::AlgebraicEndpointImages { start, end, .. }
        | BezierSplitFragment2::Unresolved { start, end } => (start, end),
    }
}

fn topology_vertex_for_parameter(
    report: &CurvePathIntersectionReport2,
    operand: CurvePathBooleanOperand2,
    curve_index: usize,
    promoted_span_index: usize,
    parameter: &BezierParameter2,
) -> Option<usize> {
    if let Some((_, matched)) = report.contacts().iter().enumerate().find(|(_, contact)| {
        let (contact_curve_index, contact_parameter) = match operand {
            CurvePathBooleanOperand2::First => {
                (contact.first_curve_index(), contact.contact().first())
            }
            CurvePathBooleanOperand2::Second => {
                (contact.second_curve_index(), contact.contact().second())
            }
        };
        contact_curve_index == curve_index
            && contact_parameter.promoted_span_index() == promoted_span_index
            && contact_parameter.local_parameter() == parameter
    }) {
        let point = matched.contact().point();
        return report
            .contacts()
            .iter()
            .position(|candidate| candidate.contact().point() == point);
    }

    for (overlap_index, overlap) in report.overlaps().iter().enumerate() {
        let (matches_source, range) = match operand {
            CurvePathBooleanOperand2::First => (
                overlap.first_curve_index() == curve_index
                    && overlap.overlap().first_span_index() == promoted_span_index,
                overlap.overlap().first_range(),
            ),
            CurvePathBooleanOperand2::Second => (
                overlap.second_curve_index() == curve_index
                    && overlap.overlap().second_span_index() == promoted_span_index,
                overlap.overlap().second_range(),
            ),
        };
        if !matches_source {
            continue;
        }
        let endpoint_index = if range.start() == parameter {
            Some(0)
        } else if range.end() == parameter {
            Some(1)
        } else {
            None
        };
        if let Some(endpoint_index) = endpoint_index {
            return Some(report.contacts().len() + overlap_index * 2 + endpoint_index);
        }
    }
    None
}

fn overlap_action_for_fragment(
    operand: CurvePathBooleanOperand2,
    curve_index: usize,
    provenance: &CurveSpanProvenance2,
    fragment_start: &BezierParameter2,
    fragment_end: &BezierParameter2,
    overlaps: &[CurvePathOverlapResolution2],
    policy: &CurvePolicy,
) -> CurveResult<Classification<Option<CurvePathBooleanFragmentAction2>>> {
    for resolution in overlaps {
        let overlap = resolution.overlap();
        let (matches_source, range) = match operand {
            CurvePathBooleanOperand2::First => (
                overlap.first_curve_index() == curve_index
                    && overlap.overlap().first() == provenance,
                overlap.overlap().first_range(),
            ),
            CurvePathBooleanOperand2::Second => (
                overlap.second_curve_index() == curve_index
                    && overlap.overlap().second() == provenance,
                overlap.overlap().second_range(),
            ),
        };
        if !matches_source {
            continue;
        }
        let (range_start, range_end) = match range.start().cmp_by_interval(range.end(), policy)? {
            Classification::Decided(std::cmp::Ordering::Less) => (range.start(), range.end()),
            Classification::Decided(std::cmp::Ordering::Greater) => (range.end(), range.start()),
            Classification::Decided(std::cmp::Ordering::Equal) => {
                return Err(crate::CurveError::DegenerateOverlapRange);
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let expected_start = range_start.clone();
        let expected_end = range_end.clone();
        let start_matches = match fragment_start.cmp_by_interval(&expected_start, policy)? {
            Classification::Decided(std::cmp::Ordering::Equal) => true,
            Classification::Decided(_) => false,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let end_matches = match fragment_end.cmp_by_interval(&expected_end, policy)? {
            Classification::Decided(std::cmp::Ordering::Equal) => true,
            Classification::Decided(_) => false,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        if !(start_matches && end_matches) {
            continue;
        }
        let action = match operand {
            CurvePathBooleanOperand2::Second => CurvePathBooleanFragmentAction2::Discard,
            CurvePathBooleanOperand2::First => match resolution.action() {
                CurvePathOverlapAction2::DiscardBoth => CurvePathBooleanFragmentAction2::Discard,
                CurvePathOverlapAction2::KeepFirst => CurvePathBooleanFragmentAction2::Keep,
                CurvePathOverlapAction2::KeepFirstReversed => {
                    CurvePathBooleanFragmentAction2::KeepReversed
                }
            },
        };
        return Ok(Classification::Decided(Some(action)));
    }
    Ok(Classification::Decided(None))
}

fn boolean_fragment_action(
    operation: BooleanOp,
    operand: CurvePathBooleanOperand2,
    first_interior_side: CurveBoundaryInteriorSide2,
    second_interior_side: CurveBoundaryInteriorSide2,
    other_inside: bool,
) -> CurvePathBooleanFragmentAction2 {
    let own_left = match operand {
        CurvePathBooleanOperand2::First => first_interior_side == CurveBoundaryInteriorSide2::Left,
        CurvePathBooleanOperand2::Second => {
            second_interior_side == CurveBoundaryInteriorSide2::Left
        }
    };
    let (result_left, result_right) = match operand {
        CurvePathBooleanOperand2::First => (
            operation.apply(own_left, other_inside),
            operation.apply(!own_left, other_inside),
        ),
        CurvePathBooleanOperand2::Second => (
            operation.apply(other_inside, own_left),
            operation.apply(other_inside, !own_left),
        ),
    };
    match (result_left, result_right) {
        (true, false) => CurvePathBooleanFragmentAction2::Keep,
        (false, true) => CurvePathBooleanFragmentAction2::KeepReversed,
        (false, false) | (true, true) => CurvePathBooleanFragmentAction2::Discard,
    }
}

fn selection_invalid_error(
    source: &CurvePathBooleanFragment2,
    cause: crate::CurveError,
) -> ExactCurveError {
    ExactCurveError::invalid(
        CurveOperation2::Boolean,
        source.family,
        source.provenance.source(),
        cause,
    )
}

fn selection_invalid_error_from_first(
    fragments: &[CurvePathBooleanFragment2],
    cause: crate::CurveError,
) -> ExactCurveError {
    if let Some(source) = fragments.first() {
        selection_invalid_error(source, cause)
    } else {
        ExactCurveError::invalid(CurveOperation2::Boolean, CurveFamily2::Line, None, cause)
    }
}

fn selection_blocked_error_from_first(
    fragments: &[CurvePathBooleanFragment2],
    reason: UncertaintyReason,
) -> ExactCurveError {
    fragments.first().map_or_else(
        || ExactCurveError::blocked(CurveOperation2::Boolean, CurveFamily2::Line, None, reason),
        |source| {
            ExactCurveError::blocked(
                CurveOperation2::Boolean,
                source.family,
                source.provenance.source(),
                reason,
            )
        },
    )
}

const fn boolean_selection_index(
    operation: BooleanOp,
    first_interior_side: CurveBoundaryInteriorSide2,
    second_interior_side: CurveBoundaryInteriorSide2,
) -> usize {
    let operation = match operation {
        BooleanOp::Union => 0,
        BooleanOp::Intersection => 1,
        BooleanOp::Difference => 2,
        BooleanOp::Xor => 3,
    };
    let first = match first_interior_side {
        CurveBoundaryInteriorSide2::Left => 0,
        CurveBoundaryInteriorSide2::Right => 1,
    };
    let second = match second_interior_side {
        CurveBoundaryInteriorSide2::Left => 0,
        CurveBoundaryInteriorSide2::Right => 1,
    };
    operation * 4 + first * 2 + second
}
