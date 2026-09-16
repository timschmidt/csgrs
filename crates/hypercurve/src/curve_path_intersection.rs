//! Immediate exact intersections and Booleans between top-level curve paths.

use std::sync::Arc;

use crate::curve_intersection::{
    CurveIntersectionContext, arrangement_from_curve_pieces, split_curve,
};
use crate::policy::resolve_certified_operation;
use crate::{
    BezierArrangementGraph2, Classification, Curve2, CurveContext, CurveIntersectionContact2,
    CurveIntersectionOverlap2, CurveIntersectionPairBlocker2, CurveIntersectionPairBlockerKind2,
    CurveIntersectionParameterComponent2, CurveOperation2, CurveOutcome, CurveParameter2,
    CurvePath2, ExactCurveError, ExactCurveResult, UncertaintyReason,
};

/// One path-pair contact with authored curve and span indices.
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

/// One point-image parameter component between authored curves in two paths.
#[derive(Clone, Debug, PartialEq)]
pub struct CurvePathIntersectionParameterComponent2 {
    first_curve_index: usize,
    second_curve_index: usize,
    component: CurveIntersectionParameterComponent2,
}

impl CurvePathIntersectionParameterComponent2 {
    /// Returns the authored curve index in the first path.
    pub const fn first_curve_index(&self) -> usize {
        self.first_curve_index
    }

    /// Returns the authored curve index in the second path.
    pub const fn second_curve_index(&self) -> usize {
        self.second_curve_index
    }

    /// Returns the complete curve-pair parameter component and its point image.
    pub const fn component(&self) -> &CurveIntersectionParameterComponent2 {
        &self.component
    }
}

/// One incomplete authored curve pair in a path-pair result.
#[derive(Clone, Debug, PartialEq)]
pub struct CurvePathIntersectionBlocker2 {
    first_curve_index: usize,
    second_curve_index: usize,
    blocker: CurveIntersectionPairBlocker2,
}

/// Clone-shared complete replay result for a retained path pair.
#[derive(Clone, Debug)]
pub struct CurvePathIntersectionResult2 {
    data: Arc<CurvePathIntersectionResultData>,
}

#[derive(Debug)]
struct CurvePathIntersectionResultData {
    authored_curve_pair_count: usize,
    candidate_curve_pair_count: usize,
    contacts: Arc<[CurvePathIntersectionContact2]>,
    overlaps: Arc<[CurvePathIntersectionOverlap2]>,
    blockers: Arc<[CurvePathIntersectionBlocker2]>,
    parameter_components: Option<Arc<[CurvePathIntersectionParameterComponent2]>>,
}

/// Exact pieces of one authored path curve, in traversal order.
#[derive(Clone, Debug)]
pub struct CurvePathSplit2 {
    curve_index: usize,
    curves: Arc<[Curve2]>,
}

/// Clone-shared exact path pieces and their certified arrangement.
#[derive(Clone, Debug)]
pub struct CurvePathIntersectionTopology2 {
    data: Arc<CurvePathIntersectionTopologyData>,
}

#[derive(Debug)]
struct CurvePathIntersectionTopologyData {
    result: CurvePathIntersectionResult2,
    first: Arc<[CurvePathSplit2]>,
    second: Arc<[CurvePathSplit2]>,
    arrangement: BezierArrangementGraph2,
}

#[derive(Debug)]
struct CurvePathIntersectionContext<'a> {
    first: &'a CurvePath2,
    second: &'a CurvePath2,
    policy: CurveContext,
    authored_curve_pair_count: usize,
    pairs: Vec<CurvePathPair>,
}

#[derive(Debug)]
struct CurvePathPair {
    first_curve_index: usize,
    second_curve_index: usize,
    context: CurveIntersectionContext,
}

fn curve_pair_bounds_decided_disjoint(
    first: &Curve2,
    second: &Curve2,
    policy: &CurveContext,
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
    /// Computes exact contacts, overlaps, point-image components, and blockers against another path
    /// immediately and reports any consumed terminal decision once.
    pub fn intersect_path(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<CurvePathIntersectionResult2>> {
        resolve_certified_operation(policy, |attempt| self.intersect_path_raw(other, attempt))
    }

    pub(crate) fn intersect_path_raw(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurvePathIntersectionResult2> {
        CurvePathIntersectionContext::try_new(self, other, policy)?.build_evidence()
    }

    /// Computes exact split topology against another path immediately and
    /// reports any consumed terminal decision once.
    pub fn intersection_topology(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<CurvePathIntersectionTopology2>> {
        resolve_certified_operation(policy, |attempt| {
            self.intersection_topology_raw(other, attempt)
        })
    }

    pub(crate) fn intersection_topology_raw(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurvePathIntersectionTopology2> {
        CurvePathIntersectionContext::try_new(self, other, policy)?.build_topology()
    }
}

impl<'a> CurvePathIntersectionContext<'a> {
    fn try_new(
        first_path: &'a CurvePath2,
        second_path: &'a CurvePath2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        let authored_curve_pair_count = first_path
            .curves()
            .len()
            .saturating_mul(second_path.curves().len());
        let candidate_capacity = first_path
            .curves()
            .len()
            .saturating_add(second_path.curves().len())
            .min(authored_curve_pair_count);
        let mut pairs = Vec::with_capacity(candidate_capacity);
        for (first_curve_index, first) in first_path.curves().iter().enumerate() {
            for (second_curve_index, second) in second_path.curves().iter().enumerate() {
                if authored_curve_pair_count > 1
                    && curve_pair_bounds_decided_disjoint(first, second, policy)
                {
                    continue;
                }
                pairs.push(CurvePathPair {
                    first_curve_index,
                    second_curve_index,
                    context: CurveIntersectionContext::try_new(first, second, policy)?,
                });
            }
        }
        Ok(Self {
            first: first_path,
            second: second_path,
            policy: *policy,
            authored_curve_pair_count,
            pairs,
        })
    }

    fn build_evidence(&self) -> ExactCurveResult<CurvePathIntersectionResult2> {
        let pair_count = self.pairs.len();
        let mut contacts = Vec::with_capacity(pair_count);
        let mut overlaps = Vec::with_capacity(pair_count);
        let mut blockers = Vec::with_capacity(pair_count);
        let mut parameter_components = Vec::new();
        for pair in &self.pairs {
            let result = pair.context.result_view()?;
            contacts.extend(result.contacts().iter().cloned().map(|contact| {
                CurvePathIntersectionContact2 {
                    first_curve_index: pair.first_curve_index,
                    second_curve_index: pair.second_curve_index,
                    contact,
                }
            }));
            overlaps.extend(result.overlaps().iter().cloned().map(|overlap| {
                CurvePathIntersectionOverlap2 {
                    first_curve_index: pair.first_curve_index,
                    second_curve_index: pair.second_curve_index,
                    overlap,
                }
            }));
            parameter_components.extend(result.parameter_components().iter().cloned().map(
                |component| CurvePathIntersectionParameterComponent2 {
                    first_curve_index: pair.first_curve_index,
                    second_curve_index: pair.second_curve_index,
                    component,
                },
            ));
            blockers.extend(result.blockers().iter().cloned().map(|blocker| {
                CurvePathIntersectionBlocker2 {
                    first_curve_index: pair.first_curve_index,
                    second_curve_index: pair.second_curve_index,
                    blocker,
                }
            }));
        }
        Ok(CurvePathIntersectionResult2 {
            data: Arc::new(CurvePathIntersectionResultData {
                authored_curve_pair_count: self.authored_curve_pair_count,
                candidate_curve_pair_count: pair_count,
                contacts: contacts.into(),
                overlaps: overlaps.into(),
                blockers: blockers.into(),
                parameter_components: (!parameter_components.is_empty())
                    .then(|| parameter_components.into()),
            }),
        })
    }

    fn build_topology(&self) -> ExactCurveResult<CurvePathIntersectionTopology2> {
        let result = self.build_evidence()?;
        if let Some(blocker) = result.blockers().first() {
            let reason = match blocker.blocker().kind() {
                CurveIntersectionPairBlockerKind2::Uncertain(reason) => *reason,
                CurveIntersectionPairBlockerKind2::IncompleteReplay { .. } => {
                    UncertaintyReason::Predicate
                }
                CurveIntersectionPairBlockerKind2::SharedComponent => UncertaintyReason::Boundary,
            };
            return Err(ExactCurveError::blocked(
                CurveOperation2::Arrangement,
                self.first.curves()[blocker.first_curve_index].family(),
                reason,
            ));
        }
        let first = split_path(
            self.first,
            result
                .contacts()
                .iter()
                .map(|contact| {
                    (
                        contact.first_curve_index(),
                        contact.contact().first().span_index(),
                        contact.contact().first().local_parameter().clone(),
                    )
                })
                .chain(result.overlaps().iter().flat_map(|overlap| {
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
                }))
                .chain(result.parameter_components().iter().flat_map(|component| {
                    component
                        .component()
                        .first_parameters()
                        .boundaries()
                        .map(|parameter| {
                            (
                                component.first_curve_index(),
                                component.component().first_span_index(),
                                parameter.clone(),
                            )
                        })
                })),
            &self.policy,
        )?;
        let second = split_path(
            self.second,
            result
                .contacts()
                .iter()
                .map(|contact| {
                    (
                        contact.second_curve_index(),
                        contact.contact().second().span_index(),
                        contact.contact().second().local_parameter().clone(),
                    )
                })
                .chain(result.overlaps().iter().flat_map(|overlap| {
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
                }))
                .chain(result.parameter_components().iter().flat_map(|component| {
                    component
                        .component()
                        .second_parameters()
                        .boundaries()
                        .map(|parameter| {
                            (
                                component.second_curve_index(),
                                component.component().second_span_index(),
                                parameter.clone(),
                            )
                        })
                })),
            &self.policy,
        )?;
        let arrangement = arrangement_from_curve_pieces(
            first.iter().chain(&second).map(CurvePathSplit2::curves),
            &self.policy,
        )?;
        Ok(CurvePathIntersectionTopology2 {
            data: Arc::new(CurvePathIntersectionTopologyData {
                result,
                first: first.into(),
                second: second.into(),
                arrangement,
            }),
        })
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

    /// Returns the exact curve-pair contact.
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

    /// Returns the certified overlap.
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

    /// Returns the exact blocked span pair.
    pub const fn blocker(&self) -> &CurveIntersectionPairBlocker2 {
        &self.blocker
    }
}

impl CurvePathIntersectionResult2 {
    /// Returns the Cartesian authored curve-pair count before broad-phase filtering.
    pub fn authored_curve_pair_count(&self) -> usize {
        self.data.authored_curve_pair_count
    }

    /// Returns the curve-pair count kept by certified broad-phase filtering.
    pub fn candidate_curve_pair_count(&self) -> usize {
        self.data.candidate_curve_pair_count
    }

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

    /// Returns complete parameter components with a single point image.
    pub fn parameter_components(&self) -> &[CurvePathIntersectionParameterComponent2] {
        self.data.parameter_components.as_deref().unwrap_or(&[])
    }

    /// Returns true when every authored curve pair has complete replay.
    pub fn is_complete(&self) -> bool {
        self.data.blockers.is_empty()
    }

    /// Returns true when complete replay found no shared point or span.
    pub fn is_disjoint(&self) -> bool {
        self.is_complete()
            && self.data.contacts.is_empty()
            && self.data.overlaps.is_empty()
            && self.parameter_components().is_empty()
    }
}

impl CurvePathSplit2 {
    /// Returns the authored path curve index.
    pub const fn curve_index(&self) -> usize {
        self.curve_index
    }

    /// Returns exact curve pieces in source traversal order.
    pub fn curves(&self) -> &[Curve2] {
        &self.curves
    }
}

impl CurvePathIntersectionTopology2 {
    /// Returns the complete result that generated this topology.
    pub fn result(&self) -> &CurvePathIntersectionResult2 {
        &self.data.result
    }

    /// Returns split topology for authored curves in the first path.
    pub fn first(&self) -> &[CurvePathSplit2] {
        &self.data.first
    }

    /// Returns split topology for authored curves in the second path.
    pub fn second(&self) -> &[CurvePathSplit2] {
        &self.data.second
    }

    /// Borrows the arrangement certified with this topology's curve pieces.
    /// Source indices enumerate the first path's authored curves followed by
    /// the second path's curves; fragment indices follow each source's traversal.
    pub fn arrangement_graph(&self) -> &BezierArrangementGraph2 {
        &self.data.arrangement
    }
}

fn split_path(
    path: &CurvePath2,
    parameters: impl Iterator<Item = (usize, usize, CurveParameter2)>,
    policy: &CurveContext,
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
                curves: split_curve(curve, parameters.into_iter(), policy)?.into(),
            })
        })
        .collect()
}
