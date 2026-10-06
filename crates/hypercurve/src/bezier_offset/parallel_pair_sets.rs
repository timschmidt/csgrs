//! Parallel-pair intersection sets, contacts, components and overlaps.

use super::*;

/// One exactly replayed contact between two analytic Bezier parallels.
///
/// The parameter pair is the lossless point construction: evaluating the two
/// supplied carriers at these parameters denotes the same exact affine point.
/// Keeping only the pair avoids embedding either clone-shared carrier—or a
/// second algebraic point expression—in every topology event.
/// Selected component boundaries retain their local field and source map;
/// they do not require reconstruction as standalone polynomial roots.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct BezierParallelPairIntersectionContact2 {
    pub(in crate::bezier_offset) first_parameter: CurveParameter2,
    pub(in crate::bezier_offset) second_parameter: CurveParameter2,
    pub(in crate::bezier_offset) certified_transverse: bool,
    pub(in crate::bezier_offset) tangent_cross_sign: Option<RealSign>,
    pub(in crate::bezier_offset) tangent_dot_sign: Option<RealSign>,
}

impl BezierParallelPairIntersectionContact2 {
    /// Returns the exact parameter on the first analytic parallel.
    pub const fn first_parameter(&self) -> &CurveParameter2 {
        &self.first_parameter
    }

    /// Returns the exact parameter on the second analytic parallel.
    pub const fn second_parameter(&self) -> &CurveParameter2 {
        &self.second_parameter
    }

    /// Returns whether exact tangent directions on the queried branches certify a crossing.
    pub const fn is_certified_transverse(&self) -> bool {
        self.certified_transverse
    }

    /// Returns the certified sign of the first parallel tangent crossed with
    /// the second parallel tangent, when exact replay decided it.
    pub const fn tangent_cross_sign(&self) -> Option<RealSign> {
        self.tangent_cross_sign
    }

    /// Returns the certified sign of the first parallel tangent dotted with
    /// the second parallel tangent, when exact replay decided it.
    ///
    /// Together with [`Self::tangent_cross_sign`], this distinguishes equal
    /// and opposite tangent directions at a tangent contact without sampling
    /// either selected parameter.
    pub const fn tangent_dot_sign(&self) -> Option<RealSign> {
        self.tangent_dot_sign
    }
}

/// One exact positive-dimensional parameter component with zero-dimensional image.
///
/// A missing parameter means that the complete queried domain of that
/// operand belongs to this component. At least one parameter is missing.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct BezierParallelPairIntersectionParameterComponent2 {
    pub(in crate::bezier_offset) first_parameter: Option<BezierParameter2>,
    pub(in crate::bezier_offset) second_parameter: Option<BezierParameter2>,
    pub(in crate::bezier_offset) point: crate::CurvePoint2,
}

impl BezierParallelPairIntersectionParameterComponent2 {
    /// Returns the fixed first parameter, or `None` for its complete domain.
    pub const fn first_parameter(&self) -> Option<&BezierParameter2> {
        self.first_parameter.as_ref()
    }

    /// Returns the fixed second parameter, or `None` for its complete domain.
    pub const fn second_parameter(&self) -> Option<&BezierParameter2> {
        self.second_parameter.as_ref()
    }

    /// Returns retained exact evidence for the component's single image point.
    pub const fn point(&self) -> &crate::CurvePoint2 {
        &self.point
    }
}

#[derive(Clone, Debug, PartialEq)]
pub(super) enum BezierParallelPairIntersectionSupplement2 {
    Complete {
        parameter_components: Arc<[BezierParallelPairIntersectionParameterComponent2]>,
        component_overlaps: Arc<[BezierParameterComponentOverlap2]>,
    },
    Incomplete(CurveIntersectionCandidates2),
}

/// Complete or explicitly incomplete intersection set for two analytic parallels.
///
/// The common complete result is two slice pointers plus one empty optional
/// pointer. Rare point-image components and incomplete elimination evidence
/// share that optional allocation.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct BezierParallelPairIntersectionSet2 {
    pub(in crate::bezier_offset) contacts: Arc<[BezierParallelPairIntersectionContact2]>,
    pub(in crate::bezier_offset) overlaps: Arc<[RationalBezierIntersectionOverlap2]>,
    pub(in crate::bezier_offset) supplement: Option<Arc<BezierParallelPairIntersectionSupplement2>>,
}

/// Exact incident-cell pair result, retaining every discovered correspondence
/// and point-image family beside the ordinary finite kernel evidence.
pub(crate) struct BezierParallelPairDomainIntersectionSet2 {
    pub(in crate::bezier_offset) intersections: BezierParallelPairIntersectionSet2,
    pub(in crate::bezier_offset) components: Vec<CurveParameterComponent2>,
}

impl BezierParallelPairDomainIntersectionSet2 {
    pub(super) fn enumerated(intersections: BezierParallelPairIntersectionSet2) -> Self {
        Self {
            intersections,
            components: Vec::new(),
        }
    }

    pub(super) fn from_components(components: Vec<CurveParameterComponent2>) -> Self {
        Self {
            intersections: BezierParallelPairIntersectionSet2::complete(
                Arc::from([]),
                Arc::from([]),
            ),
            components,
        }
    }

    pub(super) fn with_components(
        intersections: BezierParallelPairIntersectionSet2,
        components: Vec<CurveParameterComponent2>,
    ) -> Self {
        Self {
            intersections,
            components,
        }
    }

    pub(crate) fn into_parts(
        self,
    ) -> (
        BezierParallelPairIntersectionSet2,
        Vec<CurveParameterComponent2>,
    ) {
        (self.intersections, self.components)
    }
}

/// Complete rational incidence on retained finite source domains. Root
/// projection and replay use the original equations. Only shared-component
/// topology uses compact affine charts, retained by each correspondence.
pub(crate) fn rational_pair_intersections_on_ranges(
    first: &RationalBezier2,
    second: &RationalBezier2,
    ranges: [&CurveParameterRange2; 2],
    off_diagonal: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierParallelPairIntersectionSet2>> {
    let strict = policy.strict_counterpart();
    for (source, range) in [first, second].into_iter().zip(ranges) {
        match polynomial_is_nonzero_on_parameter_range(
            &source.homogeneous_power_basis()?.weight,
            range,
            &strict,
        )? {
            Classification::Decided(true) => {}
            Classification::Decided(false) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    }
    for (axis, source) in [first, second].into_iter().enumerate() {
        let Classification::Decided(Some(point)) =
            crate::BezierSubcurve2::Rational(source.clone()).point_image(&strict)
        else {
            continue;
        };
        let other_axis = 1 - axis;
        let other = [first, second][other_axis];
        let incidence = match other.point_incidence_on_range(&point, ranges[other_axis], &strict)? {
            Classification::Decided(incidence) => incidence,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let point = CurvePoint2::from(point);
        let components = match incidence {
            crate::RationalBezierPointIncidence2::EntireCurve => {
                vec![BezierParallelPairIntersectionParameterComponent2 {
                    first_parameter: None,
                    second_parameter: None,
                    point,
                }]
            }
            crate::RationalBezierPointIncidence2::Parameters(parameters) => parameters
                .into_iter()
                .map(|parameter| {
                    let mut parameters = [None, None];
                    parameters[other_axis] = Some(parameter);
                    let [first_parameter, second_parameter] = parameters;
                    BezierParallelPairIntersectionParameterComponent2 {
                        first_parameter,
                        second_parameter,
                        point: point.clone(),
                    }
                })
                .collect(),
        };
        return Ok(Classification::Decided(
            BezierParallelPairIntersectionSet2::complete_with_supplement(
                Arc::from([]),
                Arc::from([]),
                components.into(),
                Arc::from([]),
            ),
        ));
    }
    let first = first.parallel_left(Real::zero())?;
    let second = second.parallel_left(Real::zero())?;
    Ok(first
        .zero_distance_pair_intersections_in_domain(
            &second,
            ranges.map(|range| CurveParameterDomain2::new(range, None)),
            false,
            off_diagonal,
            ParameterComponentQuery2::RetainFinite,
            None,
            &strict,
        )?
        .map(|result| {
            debug_assert!(result.components.is_empty());
            result.intersections
        }))
}

impl BezierParallelPairIntersectionSet2 {
    #[cfg(test)]
    pub(crate) fn is_empty(&self) -> bool {
        self.is_complete()
            && self.contacts.is_empty()
            && self.overlaps.is_empty()
            && self.parameter_components().is_empty()
    }

    pub(super) fn complete(
        contacts: Arc<[BezierParallelPairIntersectionContact2]>,
        overlaps: Arc<[RationalBezierIntersectionOverlap2]>,
    ) -> Self {
        Self {
            contacts,
            overlaps,
            supplement: None,
        }
    }

    pub(super) fn complete_with_supplement(
        contacts: Arc<[BezierParallelPairIntersectionContact2]>,
        overlaps: Arc<[RationalBezierIntersectionOverlap2]>,
        parameter_components: Arc<[BezierParallelPairIntersectionParameterComponent2]>,
        component_overlaps: Arc<[BezierParameterComponentOverlap2]>,
    ) -> Self {
        if parameter_components.is_empty() && component_overlaps.is_empty() {
            return Self::complete(contacts, overlaps);
        }
        Self {
            contacts,
            overlaps,
            supplement: Some(Arc::new(
                BezierParallelPairIntersectionSupplement2::Complete {
                    parameter_components,
                    component_overlaps,
                },
            )),
        }
    }

    pub(super) fn incomplete(
        contacts: Arc<[BezierParallelPairIntersectionContact2]>,
        overlaps: Arc<[RationalBezierIntersectionOverlap2]>,
        candidates: CurveIntersectionCandidates2,
    ) -> Self {
        Self {
            contacts,
            overlaps,
            supplement: Some(Arc::new(
                BezierParallelPairIntersectionSupplement2::Incomplete(candidates),
            )),
        }
    }

    /// Returns every exactly replayed isolated selected-branch contact.
    pub fn contacts(&self) -> &[BezierParallelPairIntersectionContact2] {
        &self.contacts
    }

    /// Returns every exactly certified positive-length image overlap.
    ///
    /// Overlap ranges use the first and second parallel parameter domains in
    /// that order.
    pub fn overlaps(&self) -> &[RationalBezierIntersectionOverlap2] {
        &self.overlaps
    }

    /// Returns every positive-dimensional parameter component with point image.
    pub fn parameter_components(&self) -> &[BezierParallelPairIntersectionParameterComponent2] {
        match self.supplement.as_deref() {
            Some(BezierParallelPairIntersectionSupplement2::Complete {
                parameter_components,
                ..
            }) => parameter_components,
            Some(BezierParallelPairIntersectionSupplement2::Incomplete(_)) | None => &[],
        }
    }

    pub(crate) fn component_overlaps(&self) -> &[BezierParameterComponentOverlap2] {
        match self.supplement.as_deref() {
            Some(BezierParallelPairIntersectionSupplement2::Complete {
                component_overlaps,
                ..
            }) => component_overlaps,
            Some(BezierParallelPairIntersectionSupplement2::Incomplete(_)) | None => &[],
        }
    }

    /// Returns whether all possible finite contacts and components were decided.
    pub fn is_complete(&self) -> bool {
        !matches!(
            self.supplement.as_deref(),
            Some(BezierParallelPairIntersectionSupplement2::Incomplete(_))
        )
    }

    /// Returns complete unpaired projections retained after incomplete replay.
    pub fn incomplete_candidates(&self) -> Option<&CurveIntersectionCandidates2> {
        match self.supplement.as_deref() {
            Some(BezierParallelPairIntersectionSupplement2::Incomplete(candidates)) => {
                Some(candidates)
            }
            Some(BezierParallelPairIntersectionSupplement2::Complete { .. }) | None => None,
        }
    }
}

pub(super) struct BezierParallelIntersectionCandidateSystem2 {
    pub(in crate::bezier_offset) candidates: CurveIntersectionCandidates2,
    pub(in crate::bezier_offset) replay_equations: Option<[BivariatePolynomial; 2]>,
    pub(in crate::bezier_offset) overlaps: Arc<[RationalBezierIntersectionOverlap2]>,
    pub(in crate::bezier_offset) component_overlaps: Arc<[BezierParameterComponentOverlap2]>,
    pub(in crate::bezier_offset) component_pairs: Arc<[BezierParallelIntersectionParameterPair2]>,
    pub(in crate::bezier_offset) selected_component_pair_count: usize,
}

#[derive(Clone, Debug, PartialEq)]
pub(super) struct BezierParallelIntersectionParameterPair2 {
    pub(in crate::bezier_offset) parallel_parameter: BezierParameter2,
    pub(in crate::bezier_offset) other_parameter: BezierParameter2,
}

impl BezierParallelIntersectionCandidateSystem2 {
    pub(super) fn projected(
        candidates: CurveIntersectionCandidates2,
        replay_equations: Option<[BivariatePolynomial; 2]>,
    ) -> Self {
        Self {
            candidates,
            replay_equations,
            overlaps: Arc::from([]),
            component_overlaps: Arc::from([]),
            component_pairs: Arc::from([]),
            selected_component_pair_count: 0,
        }
    }

    pub(super) fn overlaps(overlaps: Arc<[RationalBezierIntersectionOverlap2]>) -> Self {
        Self {
            candidates: CurveIntersectionCandidates2::NoIntersection,
            replay_equations: None,
            overlaps,
            component_overlaps: Arc::from([]),
            component_pairs: Arc::from([]),
            selected_component_pair_count: 0,
        }
    }
}

/// One exactly replayed contact between an analytic parallel and a rational Bezier.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct BezierParallelIntersectionContact2 {
    pub(in crate::bezier_offset) parallel_parameter: BezierParameter2,
    pub(in crate::bezier_offset) other_parameter: BezierParameter2,
    pub(in crate::bezier_offset) point: crate::CurvePoint2,
    pub(in crate::bezier_offset) certified_transverse: bool,
    pub(in crate::bezier_offset) tangent_cross_sign: Option<RealSign>,
    pub(in crate::bezier_offset) tangent_dot_sign: Option<RealSign>,
}

impl BezierParallelIntersectionContact2 {
    #[cfg(test)]
    pub(crate) const fn tangent_dot_sign(&self) -> Option<RealSign> {
        self.tangent_dot_sign
    }

    /// Returns the exact parameter on the analytic parallel.
    pub const fn parallel_parameter(&self) -> &BezierParameter2 {
        &self.parallel_parameter
    }

    /// Returns the exact parameter on the rational Bezier.
    pub const fn other_parameter(&self) -> &BezierParameter2 {
        &self.other_parameter
    }

    /// Returns retained affine point evidence evaluated on the rational Bezier.
    pub const fn point(&self) -> &crate::CurvePoint2 {
        &self.point
    }

    /// Returns whether exact first derivatives certify a transverse contact.
    pub const fn is_certified_transverse(&self) -> bool {
        self.certified_transverse
    }

    /// Returns the certified sign of the analytic-parallel tangent crossed
    /// with the rational-curve tangent, when exact replay decided it.
    pub const fn tangent_cross_sign(&self) -> Option<RealSign> {
        self.tangent_cross_sign
    }
}

/// One exact positive-dimensional parameter component with zero-dimensional image.
///
/// A missing parameter means that every parameter in that operand's authored
/// domain maps to `point`; at least one parameter is always missing. This
/// keeps collapsed or constant curves out of positive-length overlap evidence
/// while retaining their complete parameter solution set.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct BezierParallelIntersectionParameterComponent2 {
    pub(in crate::bezier_offset) parallel_parameter: Option<BezierParameter2>,
    pub(in crate::bezier_offset) other_parameter: Option<BezierParameter2>,
    pub(in crate::bezier_offset) point: crate::CurvePoint2,
}

impl BezierParallelIntersectionParameterComponent2 {
    #[cfg(test)]
    pub(crate) const fn is_entire_parameter_square(&self) -> bool {
        self.parallel_parameter.is_none() && self.other_parameter.is_none()
    }

    pub(super) fn fixed_parallel_parameter(
        parallel_parameter: BezierParameter2,
        point: Point2,
    ) -> Self {
        Self {
            parallel_parameter: Some(parallel_parameter),
            other_parameter: None,
            point: crate::CurvePoint2::from(point),
        }
    }

    pub(super) fn fixed_other_parameter(other_parameter: BezierParameter2, point: Point2) -> Self {
        Self {
            parallel_parameter: None,
            other_parameter: Some(other_parameter),
            point: crate::CurvePoint2::from(point),
        }
    }

    pub(super) fn entire_parameter_square(point: Point2) -> Self {
        Self {
            parallel_parameter: None,
            other_parameter: None,
            point: crate::CurvePoint2::from(point),
        }
    }

    /// Returns the fixed analytic-parallel parameter, or `None` when every
    /// parallel parameter belongs to this component.
    pub const fn parallel_parameter(&self) -> Option<&BezierParameter2> {
        self.parallel_parameter.as_ref()
    }

    /// Returns the fixed rational-curve parameter, or `None` when every
    /// rational-curve parameter belongs to this component.
    pub const fn other_parameter(&self) -> Option<&BezierParameter2> {
        self.other_parameter.as_ref()
    }

    /// Returns retained exact evidence for the component's single image point.
    pub const fn point(&self) -> &crate::CurvePoint2 {
        &self.point
    }
}

#[derive(Clone, Debug, PartialEq)]
pub(super) enum BezierParallelIntersectionSupplement2 {
    Complete {
        parameter_components: Arc<[BezierParallelIntersectionParameterComponent2]>,
        component_overlaps: Arc<[BezierParameterComponentOverlap2]>,
    },
    Incomplete(CurveIntersectionCandidates2),
}

/// Complete or explicitly incomplete analytic-parallel/rational-Bezier intersection set.
///
/// Isolated contacts, positive-length image overlaps, and positive-dimensional
/// parameter components with point image are independent slices. The rare
/// parameter-component or incomplete-replay payload shares the existing
/// optional pointer, so the common result representation does not grow.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct BezierParallelIntersectionSet2 {
    pub(in crate::bezier_offset) contacts: Arc<[BezierParallelIntersectionContact2]>,
    pub(in crate::bezier_offset) overlaps: Arc<[RationalBezierIntersectionOverlap2]>,
    pub(in crate::bezier_offset) supplement: Option<Arc<BezierParallelIntersectionSupplement2>>,
}

impl BezierParallelIntersectionSet2 {
    #[cfg(test)]
    pub(crate) fn is_empty(&self) -> bool {
        self.is_complete()
            && self.contacts.is_empty()
            && self.overlaps.is_empty()
            && self.parameter_components().is_empty()
    }

    pub(super) fn complete(
        contacts: Arc<[BezierParallelIntersectionContact2]>,
        overlaps: Arc<[RationalBezierIntersectionOverlap2]>,
    ) -> Self {
        Self {
            contacts,
            overlaps,
            supplement: None,
        }
    }

    pub(super) fn complete_parameter_components(
        components: Arc<[BezierParallelIntersectionParameterComponent2]>,
    ) -> Self {
        Self::complete_with_supplement(Arc::from([]), Arc::from([]), components, Arc::from([]))
    }

    pub(super) fn complete_with_supplement(
        contacts: Arc<[BezierParallelIntersectionContact2]>,
        overlaps: Arc<[RationalBezierIntersectionOverlap2]>,
        parameter_components: Arc<[BezierParallelIntersectionParameterComponent2]>,
        component_overlaps: Arc<[BezierParameterComponentOverlap2]>,
    ) -> Self {
        if parameter_components.is_empty() && component_overlaps.is_empty() {
            return Self::complete(contacts, overlaps);
        }
        Self {
            contacts,
            overlaps,
            supplement: Some(Arc::new(BezierParallelIntersectionSupplement2::Complete {
                parameter_components,
                component_overlaps,
            })),
        }
    }

    pub(super) fn incomplete(
        contacts: Arc<[BezierParallelIntersectionContact2]>,
        overlaps: Arc<[RationalBezierIntersectionOverlap2]>,
        candidates: CurveIntersectionCandidates2,
    ) -> Self {
        Self {
            contacts,
            overlaps,
            supplement: Some(Arc::new(BezierParallelIntersectionSupplement2::Incomplete(
                candidates,
            ))),
        }
    }

    /// Returns every exactly replayed isolated selected-branch contact.
    pub fn contacts(&self) -> &[BezierParallelIntersectionContact2] {
        &self.contacts
    }

    /// Returns every exactly certified positive-length overlap.
    ///
    /// Each overlap uses its first range for the analytic parallel and its
    /// second range for the rational operand.
    pub fn overlaps(&self) -> &[RationalBezierIntersectionOverlap2] {
        &self.overlaps
    }

    /// Returns every exact positive-dimensional parameter component whose
    /// geometric image is one point.
    pub fn parameter_components(&self) -> &[BezierParallelIntersectionParameterComponent2] {
        match self.supplement.as_deref() {
            Some(BezierParallelIntersectionSupplement2::Complete {
                parameter_components,
                ..
            }) => parameter_components,
            Some(BezierParallelIntersectionSupplement2::Incomplete(_)) | None => &[],
        }
    }

    pub(crate) fn component_overlaps(&self) -> &[BezierParameterComponentOverlap2] {
        match self.supplement.as_deref() {
            Some(BezierParallelIntersectionSupplement2::Complete {
                component_overlaps, ..
            }) => component_overlaps,
            Some(BezierParallelIntersectionSupplement2::Incomplete(_)) | None => &[],
        }
    }

    /// Returns whether all possible finite contacts and components were decided.
    pub fn is_complete(&self) -> bool {
        !matches!(
            self.supplement.as_deref(),
            Some(BezierParallelIntersectionSupplement2::Incomplete(_))
        )
    }

    /// Returns complete unpaired projections retained after incomplete replay.
    pub fn incomplete_candidates(&self) -> Option<&CurveIntersectionCandidates2> {
        match self.supplement.as_deref() {
            Some(BezierParallelIntersectionSupplement2::Incomplete(candidates)) => Some(candidates),
            Some(BezierParallelIntersectionSupplement2::Complete { .. }) | None => None,
        }
    }
}

pub(super) fn parallel_contact_pair_is_retained(
    contacts: &[BezierParallelIntersectionContact2],
    pair: &BezierParallelIntersectionParameterPair2,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    let mut uncertain = None;
    for contact in contacts {
        let parallel_equal = contact
            .parallel_parameter
            .same_value(&pair.parallel_parameter, policy)?;
        let other_equal = contact
            .other_parameter
            .same_value(&pair.other_parameter, policy)?;
        match (parallel_equal, other_equal) {
            (Classification::Decided(true), Classification::Decided(true)) => {
                return Ok(Classification::Decided(true));
            }
            (Classification::Decided(false), _) | (_, Classification::Decided(false)) => {}
            (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => {
                uncertain = Some(reason)
            }
        }
    }
    Ok(uncertain.map_or(Classification::Decided(false), Classification::Uncertain))
}

pub(super) fn parallel_pair_contact_parameters_are_retained(
    contacts: &[BezierParallelPairIntersectionContact2],
    first_parameter: &CurveParameter2,
    second_parameter: &CurveParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    let mut uncertain = None;
    for contact in contacts {
        match (
            contact
                .first_parameter
                .same_value(first_parameter, policy)?,
            contact
                .second_parameter
                .same_value(second_parameter, policy)?,
        ) {
            (Classification::Decided(true), Classification::Decided(true)) => {
                return Ok(Classification::Decided(true));
            }
            (Classification::Decided(false), _) | (_, Classification::Decided(false)) => {}
            (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => {
                uncertain = Some(reason);
            }
        }
    }
    Ok(uncertain.map_or(Classification::Decided(false), Classification::Uncertain))
}

/// Combines retained component events with replayed isolated contacts through
/// the same exact deduplication and completeness authority on every domain.
pub(super) fn extend_parallel_pair_contacts(
    result: Classification<BezierParallelPairIntersectionSet2>,
    retained_contacts: Vec<BezierParallelPairIntersectionContact2>,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierParallelPairIntersectionSet2>> {
    match result {
        Classification::Decided(intersections) if !retained_contacts.is_empty() => {
            merge_parallel_pair_intersection_sets(
                intersections,
                BezierParallelPairIntersectionSet2::complete(
                    retained_contacts.into(),
                    Arc::from([]),
                ),
                policy,
            )
        }
        result => Ok(result),
    }
}

pub(super) fn merge_parallel_pair_intersection_sets(
    first: BezierParallelPairIntersectionSet2,
    second: BezierParallelPairIntersectionSet2,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierParallelPairIntersectionSet2>> {
    let first_complete = first.is_complete();
    let second_complete = second.is_complete();
    let incomplete_candidates = match (
        first.incomplete_candidates(),
        second.incomplete_candidates(),
    ) {
        (Some(candidates), None) | (None, Some(candidates)) => candidates.clone(),
        (Some(_), Some(_)) => CurveIntersectionCandidates2::DegenerateResultant,
        (None, None) => CurveIntersectionCandidates2::NoIntersection,
    };

    let mut contacts = first.contacts().to_vec();
    let mut merge_incomplete = false;
    for contact in second.contacts() {
        match parallel_pair_contact_parameters_are_retained(
            &contacts,
            contact.first_parameter(),
            contact.second_parameter(),
            policy,
        )? {
            Classification::Decided(true) => {}
            Classification::Decided(false) => contacts.push(contact.clone()),
            Classification::Uncertain(_) => {
                contacts.push(contact.clone());
                merge_incomplete = true;
            }
        }
    }
    let mut overlaps = first.overlaps().to_vec();
    for overlap in second.overlaps() {
        if !overlaps.contains(overlap) {
            overlaps.push(overlap.clone());
        }
    }
    let mut components = first.parameter_components().to_vec();
    for component in second.parameter_components() {
        if !components.contains(component) {
            components.push(component.clone());
        }
    }
    let mut component_overlap_evidence = first.component_overlaps().to_vec();
    for overlap in second.component_overlaps() {
        if !component_overlap_evidence.contains(overlap) {
            component_overlap_evidence.push(overlap.clone());
        }
    }

    if !first_complete || !second_complete || merge_incomplete {
        // The compact supplement deliberately stores either complete
        // point-image components or incomplete projection evidence. No
        // saturation replay currently produces both; keep that invariant
        // explicit rather than dropping already certified components.
        if !components.is_empty() || !component_overlap_evidence.is_empty() {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }
        return Ok(Classification::Decided(
            BezierParallelPairIntersectionSet2::incomplete(
                contacts.into(),
                overlaps.into(),
                incomplete_candidates,
            ),
        ));
    }
    Ok(Classification::Decided(
        BezierParallelPairIntersectionSet2::complete_with_supplement(
            contacts.into(),
            overlaps.into(),
            components.into(),
            component_overlap_evidence.into(),
        ),
    ))
}

pub(super) fn parallel_parameter_pair_is_on_overlap_correspondence(
    overlap: &RationalBezierIntersectionOverlap2,
    correspondence: &RationalBezierOverlapParameterCorrespondence2,
    first_parameter: &BezierParameter2,
    second_parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    for (parameter, range) in [
        (first_parameter, overlap.first_range()),
        (second_parameter, overlap.second_range()),
    ] {
        match overlap_parameter_is_in_range(&parameter.clone().into(), range, true, policy)? {
            Classification::Decided(true) => {}
            Classification::Decided(false) => return Ok(Classification::Decided(false)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    let mapped = match correspondence.map_first_to_second(
        first_parameter,
        overlap.first_range(),
        overlap.second_range(),
        policy,
    )? {
        Classification::Decided(Some(mapped)) => mapped,
        Classification::Decided(None) => return Ok(Classification::Decided(false)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    match mapped.same_value(second_parameter, policy)? {
        Classification::Decided(false) => Ok(Classification::Decided(false)),
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        Classification::Decided(true) => {
            for (boundary, included) in [
                (overlap.first_range().start(), overlap.includes_start()),
                (overlap.first_range().end(), overlap.includes_end()),
            ] {
                match first_parameter.same_value(boundary, policy)? {
                    Classification::Decided(true) => {
                        return Ok(Classification::Decided(included));
                    }
                    Classification::Decided(false) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            Ok(Classification::Decided(true))
        }
    }
}

pub(super) fn parallel_parameter_pair_is_overlap_boundary(
    overlaps: &[RationalBezierIntersectionOverlap2],
    parallel_parameter: &BezierParameter2,
    other_parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    let mut uncertain = None;
    for overlap in overlaps {
        for (parallel_boundary, other_boundary) in [
            (
                overlap.first_range().start(),
                overlap.second_range().start(),
            ),
            (overlap.first_range().end(), overlap.second_range().end()),
        ] {
            match (
                parallel_parameter.same_value(parallel_boundary, policy)?,
                other_parameter.same_value(other_boundary, policy)?,
            ) {
                (Classification::Decided(true), Classification::Decided(true)) => {
                    return Ok(Classification::Decided(true));
                }
                (Classification::Decided(false), _) | (_, Classification::Decided(false)) => {}
                (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => {
                    uncertain = Some(reason)
                }
            }
        }
    }
    Ok(uncertain.map_or(Classification::Decided(false), Classification::Uncertain))
}

pub(super) fn parallel_parameter_pair_is_excluded(
    excluded: &[BezierParallelIntersectionParameterPair2],
    parallel_parameter: &BezierParameter2,
    other_parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    let mut uncertain = None;
    for pair in excluded {
        match (
            parallel_parameter.same_value(&pair.parallel_parameter, policy)?,
            other_parameter.same_value(&pair.other_parameter, policy)?,
        ) {
            (Classification::Decided(true), Classification::Decided(true)) => {
                return Ok(Classification::Decided(true));
            }
            (Classification::Decided(false), _) | (_, Classification::Decided(false)) => {}
            (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => {
                uncertain = Some(reason);
            }
        }
    }
    Ok(uncertain.map_or(Classification::Decided(false), Classification::Uncertain))
}

pub(super) fn swapped_parallel_overlap(
    overlap: &RationalBezierIntersectionOverlap2,
) -> RationalBezierIntersectionOverlap2 {
    RationalBezierIntersectionOverlap2::from_certified_parameters(
        overlap.second_range().start().clone(),
        overlap.second_range().end().clone(),
        overlap.first_range().start().clone(),
        overlap.first_range().end().clone(),
        overlap.orientation(),
        [overlap.includes_start(), overlap.includes_end()],
    )
}

pub(super) fn swapped_parameter_component_overlap(
    overlap: &BezierParameterComponentOverlap2,
) -> BezierParameterComponentOverlap2 {
    BezierParameterComponentOverlap2 {
        overlap: swapped_parallel_overlap(&overlap.overlap),
        parameter_charts: overlap.parameter_charts.as_ref().map(|charts| {
            Arc::new(ParameterComponentAffineCharts2 {
                to_source: [charts.to_source[1].clone(), charts.to_source[0].clone()],
                to_local: [charts.to_local[1].clone(), charts.to_local[0].clone()],
                overlap: swapped_parallel_overlap(&charts.overlap),
            })
        }),
        support: Arc::new(bivariate_swap_parameters(&overlap.support)),
        fiber_root_ranks: [overlap.fiber_root_ranks[1], overlap.fiber_root_ranks[0]],
        witness: BezierParallelIntersectionParameterPair2 {
            parallel_parameter: overlap.witness.other_parameter.clone(),
            other_parameter: overlap.witness.parallel_parameter.clone(),
        },
    }
}

pub(super) fn structural_parallel_overlap(
    first: &BezierParallel2,
    second: &BezierParallel2,
    policy: &CurveContext,
) -> CurveResult<Option<RationalBezierIntersectionOverlap2>> {
    let unit_overlap = |orientation, second_start, second_end| {
        RationalBezierIntersectionOverlap2::from_certified_parameters(
            BezierParameter2::Exact(Real::zero()),
            BezierParameter2::Exact(Real::one()),
            BezierParameter2::Exact(second_start),
            BezierParameter2::Exact(second_end),
            orientation,
            [true, true],
        )
    };
    if first.source() == second.source()
        && matches!(
            compare_reals(first.distance(), second.distance(), policy),
            Some(std::cmp::Ordering::Equal)
        )
    {
        return Ok(Some(unit_overlap(
            CurveOverlapOrientation2::Same,
            Real::zero(),
            Real::one(),
        )));
    }
    if first.source() == &second.source().reversed()
        && matches!(
            compare_reals(first.distance(), &-second.distance().clone(), policy),
            Some(std::cmp::Ordering::Equal)
        )
    {
        return Ok(Some(unit_overlap(
            CurveOverlapOrientation2::Reversed,
            Real::one(),
            Real::zero(),
        )));
    }
    Ok(None)
}

pub(super) enum CertifiedParallelSourceOverlapKind2 {
    None,
    Selected(RationalBezierIntersectionOverlap2),
    Excluded,
}

pub(super) struct CertifiedParallelSourceOverlap2 {
    pub(in crate::bezier_offset) kind: CertifiedParallelSourceOverlapKind2,
    pub(in crate::bezier_offset) contacts: Arc<[BezierParallelIntersectionParameterPair2]>,
}

impl CertifiedParallelSourceOverlap2 {
    pub(super) fn without_contacts(kind: CertifiedParallelSourceOverlapKind2) -> Self {
        Self {
            kind,
            contacts: Arc::from([]),
        }
    }

    pub(super) fn selected_overlap(&self) -> Option<&RationalBezierIntersectionOverlap2> {
        match &self.kind {
            CertifiedParallelSourceOverlapKind2::Selected(overlap) => Some(overlap),
            CertifiedParallelSourceOverlapKind2::None
            | CertifiedParallelSourceOverlapKind2::Excluded => None,
        }
    }
}

pub(super) fn certified_parallel_source_overlap(
    first: &BezierParallel2,
    second: &BezierParallel2,
    policy: &CurveContext,
) -> CurveResult<Classification<CertifiedParallelSourceOverlap2>> {
    let first_source = first.source().to_rational_bezier()?;
    let second_source = second.source().to_rational_bezier()?;
    let contacts = match first_source.intersection_contacts_classified(&second_source, policy)? {
        Classification::Decided(contacts) => contacts,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let isolated = contacts
        .isolated_contacts()
        .iter()
        .map(|contact| BezierParallelIntersectionParameterPair2 {
            parallel_parameter: contact.first_parameter().clone(),
            other_parameter: contact.second_parameter().clone(),
        })
        .collect::<Arc<[_]>>();
    let overlap = match contacts {
        RationalBezierIntersectionContacts2::Overlap(overlap)
        | RationalBezierIntersectionContacts2::ContactsAndOverlap { overlap, .. } => Some(overlap),
        RationalBezierIntersectionContacts2::NoIntersection
        | RationalBezierIntersectionContacts2::Contacts(_) => None,
        RationalBezierIntersectionContacts2::Incomplete { .. }
        | RationalBezierIntersectionContacts2::DegenerateResultant => {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }
    };
    let Some(overlap) = overlap else {
        return Ok(Classification::Decided(CertifiedParallelSourceOverlap2 {
            kind: CertifiedParallelSourceOverlapKind2::None,
            contacts: isolated,
        }));
    };
    let second_distance = match overlap.orientation() {
        CurveOverlapOrientation2::Same => second.distance().clone(),
        CurveOverlapOrientation2::Reversed => -second.distance().clone(),
    };
    Ok(
        match compare_reals(first.distance(), &second_distance, policy) {
            Some(std::cmp::Ordering::Equal) => {
                Classification::Decided(CertifiedParallelSourceOverlap2 {
                    kind: CertifiedParallelSourceOverlapKind2::Selected(overlap),
                    contacts: isolated,
                })
            }
            Some(std::cmp::Ordering::Less | std::cmp::Ordering::Greater) => {
                Classification::Decided(CertifiedParallelSourceOverlap2 {
                    kind: CertifiedParallelSourceOverlapKind2::Excluded,
                    contacts: isolated,
                })
            }
            None => Classification::Uncertain(UncertaintyReason::RealSign),
        },
    )
}

pub(super) fn parallel_pair_set_from_parallel_rational(
    result: BezierParallelIntersectionSet2,
    swapped: bool,
) -> BezierParallelPairIntersectionSet2 {
    let contacts = result
        .contacts
        .iter()
        .map(|contact| {
            let (first_parameter, second_parameter) = if swapped {
                (
                    contact.other_parameter.clone(),
                    contact.parallel_parameter.clone(),
                )
            } else {
                (
                    contact.parallel_parameter.clone(),
                    contact.other_parameter.clone(),
                )
            };
            BezierParallelPairIntersectionContact2 {
                first_parameter: first_parameter.into(),
                second_parameter: second_parameter.into(),
                certified_transverse: contact.certified_transverse,
                tangent_cross_sign: contact.tangent_cross_sign.map(|sign| {
                    if swapped {
                        match sign {
                            RealSign::Positive => RealSign::Negative,
                            RealSign::Negative => RealSign::Positive,
                            RealSign::Zero => RealSign::Zero,
                        }
                    } else {
                        sign
                    }
                }),
                tangent_dot_sign: contact.tangent_dot_sign,
            }
        })
        .collect::<Arc<[_]>>();
    let overlaps = if swapped {
        result
            .overlaps
            .iter()
            .map(swapped_parallel_overlap)
            .collect::<Arc<[_]>>()
    } else {
        result.overlaps.clone()
    };
    match result.supplement.as_deref() {
        None => BezierParallelPairIntersectionSet2::complete(contacts, overlaps),
        Some(BezierParallelIntersectionSupplement2::Incomplete(candidates)) => {
            BezierParallelPairIntersectionSet2::incomplete(
                contacts,
                overlaps,
                if swapped {
                    candidates.clone().swapped()
                } else {
                    candidates.clone()
                },
            )
        }
        Some(BezierParallelIntersectionSupplement2::Complete {
            parameter_components,
            component_overlaps,
        }) => {
            let components = parameter_components
                .iter()
                .map(|component| {
                    let (first_parameter, second_parameter) = if swapped {
                        (
                            component.other_parameter.clone(),
                            component.parallel_parameter.clone(),
                        )
                    } else {
                        (
                            component.parallel_parameter.clone(),
                            component.other_parameter.clone(),
                        )
                    };
                    BezierParallelPairIntersectionParameterComponent2 {
                        first_parameter,
                        second_parameter,
                        point: component.point.clone(),
                    }
                })
                .collect();
            let component_overlaps = if swapped {
                component_overlaps
                    .iter()
                    .map(swapped_parameter_component_overlap)
                    .collect()
            } else {
                component_overlaps.clone()
            };
            BezierParallelPairIntersectionSet2::complete_with_supplement(
                contacts,
                overlaps,
                components,
                component_overlaps,
            )
        }
    }
}

pub(super) fn rational_self_contact_on_parallel(
    parallel: &BezierParallel2,
    mut contact: BezierParallelIntersectionContact2,
    policy: &CurveContext,
) -> CurveResult<BezierParallelIntersectionContact2> {
    let tangent_relation = match (
        parallel
            .parallel_derivative_scale_sign(&contact.parallel_parameter.clone().into(), policy)?,
        parallel.parallel_derivative_scale_sign(&contact.other_parameter.clone().into(), policy)?,
        parallel.source_tangent_pair_cross_and_dot_signs(
            &contact.parallel_parameter,
            parallel,
            &contact.other_parameter,
            policy,
        )?,
    ) {
        (
            Classification::Decided(first @ (RealSign::Positive | RealSign::Negative)),
            Classification::Decided(second @ (RealSign::Positive | RealSign::Negative)),
            Classification::Decided((cross, dot)),
        ) => {
            let scale = product_sign(first, second);
            Some((product_sign(cross, scale), product_sign(dot, scale)))
        }
        _ => None,
    };
    if let (Some(rational_cross), Some((parallel_cross, _))) =
        (contact.tangent_cross_sign, tangent_relation)
        && rational_cross != parallel_cross
    {
        return Err(CurveError::Topology(
            "an exact rational parallel self-contact changed tangent orientation".to_owned(),
        ));
    }
    contact.tangent_cross_sign = contact
        .tangent_cross_sign
        .or(tangent_relation.map(|relation| relation.0));
    contact.tangent_dot_sign = tangent_relation.map(|relation| relation.1);
    Ok(contact)
}

pub(super) fn parallel_pair_set_from_rational_self_contacts(
    parallel: &BezierParallel2,
    result: RationalBezierIntersectionContacts2,
    policy: &CurveContext,
) -> CurveResult<BezierParallelPairIntersectionSet2> {
    let mut result = parallel_set_from_rational_contacts(result);
    result.contacts = result
        .contacts
        .iter()
        .cloned()
        .map(|contact| rational_self_contact_on_parallel(parallel, contact, policy))
        .collect::<CurveResult<Arc<[_]>>>()?;
    Ok(parallel_pair_set_from_parallel_rational(result, false))
}

pub(super) fn parallel_set_from_rational_contacts(
    result: RationalBezierIntersectionContacts2,
) -> BezierParallelIntersectionSet2 {
    let map_contacts = |contacts: &[crate::RationalBezierIntersectionContact2]| {
        contacts
            .iter()
            .map(|contact| BezierParallelIntersectionContact2 {
                parallel_parameter: contact.first_parameter().clone(),
                other_parameter: contact.second_parameter().clone(),
                point: contact.point().clone(),
                certified_transverse: contact.is_certified_transverse(),
                tangent_cross_sign: contact.tangent_cross_sign(),
                tangent_dot_sign: None,
            })
            .collect::<Arc<[_]>>()
    };
    match result {
        RationalBezierIntersectionContacts2::NoIntersection => {
            BezierParallelIntersectionSet2::complete(Arc::from([]), Arc::from([]))
        }
        RationalBezierIntersectionContacts2::Contacts(contacts) => {
            BezierParallelIntersectionSet2::complete(map_contacts(&contacts), Arc::from([]))
        }
        RationalBezierIntersectionContacts2::Overlap(overlap) => {
            BezierParallelIntersectionSet2::complete(Arc::from([]), Arc::from([overlap]))
        }
        RationalBezierIntersectionContacts2::ContactsAndOverlap { contacts, overlap } => {
            BezierParallelIntersectionSet2::complete(map_contacts(&contacts), Arc::from([overlap]))
        }
        RationalBezierIntersectionContacts2::Incomplete {
            contacts,
            candidates,
        } => BezierParallelIntersectionSet2::incomplete(
            map_contacts(&contacts),
            Arc::from([]),
            candidates,
        ),
        RationalBezierIntersectionContacts2::DegenerateResultant => {
            BezierParallelIntersectionSet2::incomplete(
                Arc::from([]),
                Arc::from([]),
                CurveIntersectionCandidates2::DegenerateResultant,
            )
        }
    }
}

impl std::fmt::Debug for BezierParallel2 {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("BezierParallel2")
            .field("source", self.source())
            .field("distance", &self.data.distance)
            .finish()
    }
}
