//! Normalized topology events for contour-level algorithms.
//!
//! Small event collections use a candidate-filtered pair scan. Medium contour
//! pairs scan boxes ordered by exact minimum x; dense certified pairs also use
//! retained exact coordinate ranks, while large sampled-sparse pairs augment
//! the x order with one exact subtree-maximum witness per segment. Bounding
//! boxes only remove pairs whose disjointness is decided; every remaining
//! candidate goes through the exact segment kernels.

use std::cell::OnceCell;
use std::cmp::Ordering;
use std::fmt;
use std::num::NonZeroU64;
use std::rc::Rc;

use hyperreal::{
    ExactDyadicLine2, ExactDyadicLineParameters2, ExactDyadicWideLineParameters2, Real,
};

use crate::bbox::{Aabb2, aabbs_decided_disjoint, decided_contour_aabb, decided_segment_aabb};
use crate::classify::{
    at_unit_interval_endpoint, compare_reals, compare_reals_for_split_ordering,
    in_closed_unit_interval, is_zero, min_real,
};
use crate::intersect::{
    CertifiedLineSegmentSupportRelation,
    certified_line_segment_support_relation_with_prepared_exact_dyadic_f64,
};
use crate::{
    ArcArcIntersection, Classification, Contour2, CurveError, CurvePolicy, CurveResult,
    IntersectionKind, LineArcIntersection, LineArcOrder, LineLineIntersection, ParamRange, Point2,
    Segment2, SegmentIntersection, SegmentKind, UncertaintyReason,
};

/// Which side of a contour-pair event to inspect.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ContourOperand {
    /// First contour passed to the intersection query.
    First,
    /// Second contour passed to the intersection query.
    Second,
}

/// A normalized set of contour-pair topology events.
#[derive(Clone, Default)]
pub struct ContourIntersectionSet {
    storage: ContourIntersectionStorage,
    // Bits 0..62 retain positive winding deltas. Bit 63 is always set so the
    // `Option<NonZeroU64>` occupies one word; larger event sets use the exact
    // fallback instead of allocating a sidecar.
    certified_positive_line_crossings: Option<NonZeroU64>,
}

#[derive(Clone)]
enum ContourIntersectionStorage {
    Materialized(Vec<ContourIntersection>),
    CertifiedLineCrossings {
        crossings: Rc<Vec<CertifiedLineCrossingEvent>>,
        materialized: OnceCell<Vec<ContourIntersection>>,
    },
}

impl Default for ContourIntersectionStorage {
    fn default() -> Self {
        Self::Materialized(Vec::new())
    }
}

#[derive(Clone, Debug)]
pub(crate) struct CertifiedLineCrossingEvent {
    pub(crate) a_segment_index: u16,
    pub(crate) b_segment_index: u16,
    pub(crate) point: Point2,
    parameters: CertifiedLineCrossingParameters,
}

#[derive(Clone, Debug)]
enum CertifiedLineCrossingParameters {
    ExactDyadic(ExactDyadicLineParameters2),
    ExactDyadicWide(Box<ExactDyadicWideLineParameters2>),
    Materialized(Box<[Real; 2]>),
}

impl CertifiedLineCrossingEvent {
    fn new_exact_dyadic(
        a_segment_index: u16,
        b_segment_index: u16,
        point: Point2,
        parameters: ExactDyadicLineParameters2,
    ) -> Self {
        Self {
            a_segment_index,
            b_segment_index,
            point,
            parameters: CertifiedLineCrossingParameters::ExactDyadic(parameters),
        }
    }

    fn new_materialized(
        a_segment_index: u16,
        b_segment_index: u16,
        point: Point2,
        a_param: Real,
        b_param: Real,
    ) -> Self {
        Self {
            a_segment_index,
            b_segment_index,
            point,
            parameters: CertifiedLineCrossingParameters::Materialized(Box::new([a_param, b_param])),
        }
    }

    fn new_exact_dyadic_wide(
        a_segment_index: u16,
        b_segment_index: u16,
        point: Point2,
        parameters: ExactDyadicWideLineParameters2,
    ) -> Self {
        Self {
            a_segment_index,
            b_segment_index,
            point,
            parameters: CertifiedLineCrossingParameters::ExactDyadicWide(Box::new(parameters)),
        }
    }

    fn parameter_index(operand: ContourOperand) -> usize {
        match operand {
            ContourOperand::First => 0,
            ContourOperand::Second => 1,
        }
    }

    fn compare_parameter_impl<const NORMALIZED_COMPACT_PRODUCT: bool>(
        &self,
        other: &Self,
        operand: ContourOperand,
        policy: &CurvePolicy,
    ) -> Option<Ordering> {
        match (&self.parameters, &other.parameters) {
            (
                CertifiedLineCrossingParameters::ExactDyadic(left),
                CertifiedLineCrossingParameters::ExactDyadic(right),
            ) => Some(match operand {
                ContourOperand::First if NORMALIZED_COMPACT_PRODUCT => {
                    left.compare_first_parameter_normalized(right)
                }
                ContourOperand::Second if NORMALIZED_COMPACT_PRODUCT => {
                    left.compare_second_parameter_normalized(right)
                }
                ContourOperand::First => left.compare_first_parameter(right),
                ContourOperand::Second => left.compare_second_parameter(right),
            }),
            (
                CertifiedLineCrossingParameters::ExactDyadicWide(left),
                CertifiedLineCrossingParameters::ExactDyadicWide(right),
            ) => Some(match operand {
                ContourOperand::First => left.compare_first_parameter(right),
                ContourOperand::Second => left.compare_second_parameter(right),
            }),
            (
                CertifiedLineCrossingParameters::ExactDyadicWide(left),
                CertifiedLineCrossingParameters::ExactDyadic(right),
            ) => Some(match operand {
                ContourOperand::First => left.compare_first_parameter_to_compact(right),
                ContourOperand::Second => left.compare_second_parameter_to_compact(right),
            }),
            (
                CertifiedLineCrossingParameters::ExactDyadic(left),
                CertifiedLineCrossingParameters::ExactDyadicWide(right),
            ) => Some(
                match operand {
                    ContourOperand::First => right.compare_first_parameter_to_compact(left),
                    ContourOperand::Second => right.compare_second_parameter_to_compact(left),
                }
                .reverse(),
            ),
            (
                CertifiedLineCrossingParameters::Materialized(left),
                CertifiedLineCrossingParameters::Materialized(right),
            ) => compare_reals(
                &left[Self::parameter_index(operand)],
                &right[Self::parameter_index(operand)],
                policy,
            ),
            _ => {
                let left = self.materialize_parameter(operand);
                let right = other.materialize_parameter(operand);
                compare_reals(&left, &right, policy)
            }
        }
    }

    pub(crate) fn compare_parameter(
        &self,
        other: &Self,
        operand: ContourOperand,
        policy: &CurvePolicy,
    ) -> Option<Ordering> {
        self.compare_parameter_impl::<false>(other, operand, policy)
    }

    #[cold]
    #[inline(never)]
    pub(crate) fn compare_parameter_normalized(
        &self,
        other: &Self,
        operand: ContourOperand,
        policy: &CurvePolicy,
    ) -> Option<Ordering> {
        self.compare_parameter_impl::<true>(other, operand, policy)
    }

    fn materialize_parameter(&self, operand: ContourOperand) -> Real {
        match &self.parameters {
            CertifiedLineCrossingParameters::ExactDyadic(parameters) => match operand {
                ContourOperand::First => parameters.materialize_first_parameter(),
                ContourOperand::Second => parameters.materialize_second_parameter(),
            },
            CertifiedLineCrossingParameters::ExactDyadicWide(parameters) => match operand {
                ContourOperand::First => parameters.materialize_first_parameter(),
                ContourOperand::Second => parameters.materialize_second_parameter(),
            },
            CertifiedLineCrossingParameters::Materialized(parameters) => {
                parameters[Self::parameter_index(operand)].clone()
            }
        }
    }

    pub(crate) fn materialized_parameter(&self, operand: ContourOperand) -> Option<&Real> {
        match &self.parameters {
            CertifiedLineCrossingParameters::ExactDyadic(_)
            | CertifiedLineCrossingParameters::ExactDyadicWide(_) => None,
            CertifiedLineCrossingParameters::Materialized(parameters) => {
                Some(&parameters[Self::parameter_index(operand)])
            }
        }
    }

    fn materialize(&self) -> ContourIntersection {
        ContourIntersection::Point(ContourPointIntersection {
            a_segment_index: usize::from(self.a_segment_index),
            b_segment_index: usize::from(self.b_segment_index),
            a_segment_kind: SegmentKind::Line,
            b_segment_kind: SegmentKind::Line,
            point: self.point.clone(),
            a_param: self.materialize_parameter(ContourOperand::First),
            b_param: self.materialize_parameter(ContourOperand::Second),
            kind: IntersectionKind::Crossing,
        })
    }
}

impl fmt::Debug for ContourIntersectionSet {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter
            .debug_struct("ContourIntersectionSet")
            .field("events", &self.events())
            .finish()
    }
}

impl PartialEq for ContourIntersectionSet {
    fn eq(&self, other: &Self) -> bool {
        self.events() == other.events()
    }
}

impl ContourIntersectionSet {
    /// Constructs an event set from already-normalized events.
    pub fn new(events: Vec<ContourIntersection>) -> CurveResult<Self> {
        validate_contour_intersection_events(&events, &CurvePolicy::certified())?;
        Ok(Self {
            storage: ContourIntersectionStorage::Materialized(events),
            certified_positive_line_crossings: None,
        })
    }

    fn from_normalized_events(events: Vec<ContourIntersection>) -> Self {
        // Module-private collectors normalize each relation as it is appended;
        // public or forged event vectors still go through `new` above.
        Self {
            storage: ContourIntersectionStorage::Materialized(events),
            certified_positive_line_crossings: None,
        }
    }

    fn from_certified_line_crossings(
        events: Vec<ContourIntersection>,
        positive_crossings: Option<NonZeroU64>,
    ) -> Self {
        Self {
            storage: ContourIntersectionStorage::Materialized(events),
            certified_positive_line_crossings: positive_crossings,
        }
    }

    fn from_certified_line_crossing_points(
        crossings: Vec<CertifiedLineCrossingEvent>,
        positive_crossings: Option<NonZeroU64>,
    ) -> Self {
        if crossings.is_empty() {
            return Self::default();
        }
        Self {
            storage: ContourIntersectionStorage::CertifiedLineCrossings {
                crossings: Rc::new(crossings),
                materialized: OnceCell::new(),
            },
            certified_positive_line_crossings: positive_crossings,
        }
    }

    pub(crate) fn retained_certified_line_crossings(
        &self,
    ) -> Option<&Rc<Vec<CertifiedLineCrossingEvent>>> {
        match &self.storage {
            ContourIntersectionStorage::CertifiedLineCrossings { crossings, .. } => Some(crossings),
            ContourIntersectionStorage::Materialized(_) => None,
        }
    }

    pub(crate) fn certified_line_crossing_delta(&self, event_index: usize) -> Option<i32> {
        let crossings = self.certified_positive_line_crossings?.get();
        (event_index < 63).then_some(if crossings & (1 << event_index) == 0 {
            -1
        } else {
            1
        })
    }

    /// Returns all events in segment-pair scan order.
    pub fn events(&self) -> &[ContourIntersection] {
        match &self.storage {
            ContourIntersectionStorage::Materialized(events) => events,
            ContourIntersectionStorage::CertifiedLineCrossings {
                crossings,
                materialized,
            } => materialized.get_or_init(|| {
                crossings
                    .iter()
                    .map(CertifiedLineCrossingEvent::materialize)
                    .collect()
            }),
        }
    }

    /// Consumes the set and returns its events.
    pub fn into_events(self) -> Vec<ContourIntersection> {
        match self.storage {
            ContourIntersectionStorage::Materialized(events) => events,
            ContourIntersectionStorage::CertifiedLineCrossings {
                crossings,
                materialized,
            } => materialized.into_inner().unwrap_or_else(|| {
                crossings
                    .iter()
                    .map(CertifiedLineCrossingEvent::materialize)
                    .collect()
            }),
        }
    }

    /// Returns true when no events were collected.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Returns the number of collected events.
    pub fn len(&self) -> usize {
        match &self.storage {
            ContourIntersectionStorage::Materialized(events) => events.len(),
            ContourIntersectionStorage::CertifiedLineCrossings { crossings, .. } => crossings.len(),
        }
    }

    /// Returns retained point intersection events.
    pub fn point_event_count(&self) -> usize {
        match &self.storage {
            ContourIntersectionStorage::Materialized(events) => events
                .iter()
                .filter(|event| matches!(event, ContourIntersection::Point(_)))
                .count(),
            ContourIntersectionStorage::CertifiedLineCrossings { crossings, .. } => crossings.len(),
        }
    }

    /// Returns retained overlap intersection events.
    pub fn overlap_event_count(&self) -> usize {
        match &self.storage {
            ContourIntersectionStorage::Materialized(events) => events
                .iter()
                .filter(|event| matches!(event, ContourIntersection::Overlap(_)))
                .count(),
            ContourIntersectionStorage::CertifiedLineCrossings { .. } => 0,
        }
    }

    /// Returns retained unresolved intersection events.
    pub fn uncertain_event_count(&self) -> usize {
        match &self.storage {
            ContourIntersectionStorage::Materialized(events) => events
                .iter()
                .filter(|event| matches!(event, ContourIntersection::Uncertain(_)))
                .count(),
            ContourIntersectionStorage::CertifiedLineCrossings { .. } => 0,
        }
    }

    /// Returns events for one segment sorted by that segment's local parameter.
    pub fn sorted_events_for_segment<'a>(
        &'a self,
        operand: ContourOperand,
        segment_index: usize,
        policy: &CurvePolicy,
    ) -> Classification<Vec<&'a ContourIntersection>> {
        let mut sorted: Vec<(&ContourIntersection, Real)> = Vec::new();

        for event in self.events() {
            if event.segment_index(operand) != Some(segment_index) {
                continue;
            }

            let order_param = match event.order_param(operand, policy) {
                Ok(order_param) => order_param,
                Err(reason) => return Classification::Uncertain(reason),
            };

            let Some(insert_at) = insertion_index(&sorted, &order_param, policy) else {
                return Classification::Uncertain(UncertaintyReason::Ordering);
            };
            // Sorted local events are the contour-level analogue of the event
            // ordering used by sweep-line clipping algorithms. We keep the
            // order proof explicit because a wrong tie-breaker here can create
            // branch vertices in the downstream boundary graph.
            sorted.insert(insert_at, (event, order_param));
        }

        Classification::Decided(sorted.into_iter().map(|(event, _)| event).collect())
    }
}

fn validate_contour_intersection_events(
    events: &[ContourIntersection],
    policy: &CurvePolicy,
) -> CurveResult<()> {
    for (left_index, left) in events.iter().enumerate() {
        validate_contour_intersection_event(left, policy)?;
        if events[left_index + 1..].iter().any(|right| right == left) {
            return Err(CurveError::Topology(
                "contour intersection set must not contain duplicate events".into(),
            ));
        }
    }
    Ok(())
}

fn validate_contour_intersection_event(
    event: &ContourIntersection,
    policy: &CurvePolicy,
) -> CurveResult<()> {
    match event {
        ContourIntersection::Point(point) => {
            validate_event_unit_parameter(&point.a_param, policy, "first point")?;
            validate_event_unit_parameter(&point.b_param, policy, "second point")?;
            validate_point_event_kind(point, policy)
        }
        ContourIntersection::Overlap(overlap) => {
            validate_overlap_event_geometry(&overlap.segment, policy)?;
            validate_event_unit_range(&overlap.a_range, policy, "first overlap")?;
            validate_event_unit_range(&overlap.b_range, policy, "second overlap")
        }
        ContourIntersection::Uncertain(_) => Ok(()),
    }
}

fn validate_point_event_kind(
    point: &ContourPointIntersection,
    policy: &CurvePolicy,
) -> CurveResult<()> {
    let Some(a_endpoint) = at_unit_interval_endpoint(&point.a_param, policy) else {
        return Err(CurveError::Topology(
            "contour point event first endpoint status must be certified".into(),
        ));
    };
    let Some(b_endpoint) = at_unit_interval_endpoint(&point.b_param, policy) else {
        return Err(CurveError::Topology(
            "contour point event second endpoint status must be certified".into(),
        ));
    };
    let endpoint_contact = a_endpoint || b_endpoint;
    match (point.kind, endpoint_contact) {
        (IntersectionKind::Endpoint, true)
        | (IntersectionKind::Crossing | IntersectionKind::Tangent, false) => Ok(()),
        (IntersectionKind::Overlap, _) => Err(CurveError::Topology(
            "contour point event must not carry overlap contact kind".into(),
        )),
        (IntersectionKind::Endpoint, false) => Err(CurveError::Topology(
            "contour endpoint event kind must carry endpoint parameter evidence".into(),
        )),
        (IntersectionKind::Crossing | IntersectionKind::Tangent, true) => {
            Err(CurveError::Topology(
                "contour interior point event kind must not carry endpoint parameter evidence"
                    .into(),
            ))
        }
    }
}

fn validate_event_unit_parameter(
    parameter: &Real,
    policy: &CurvePolicy,
    name: &str,
) -> CurveResult<()> {
    if in_closed_unit_interval(parameter, policy) != Some(true) {
        return Err(CurveError::Topology(format!(
            "contour intersection {name} parameter must be certified inside the unit interval"
        )));
    }
    Ok(())
}

fn validate_event_unit_range(
    range: &ParamRange,
    policy: &CurvePolicy,
    name: &str,
) -> CurveResult<()> {
    validate_event_unit_parameter(range.start(), policy, name)?;
    validate_event_unit_parameter(range.end(), policy, name)?;
    match compare_reals_for_split_ordering(range.start(), range.end(), policy) {
        Some(std::cmp::Ordering::Less | std::cmp::Ordering::Greater) => Ok(()),
        Some(std::cmp::Ordering::Equal) => Err(CurveError::Topology(format!(
            "contour intersection {name} range must be positive-dimensional"
        ))),
        None => Err(CurveError::Topology(format!(
            "contour intersection {name} range ordering must be certified"
        ))),
    }
}

fn validate_overlap_event_geometry(segment: &Segment2, policy: &CurvePolicy) -> CurveResult<()> {
    let value = match segment {
        Segment2::Line(line) => line.length_squared(),
        Segment2::Arc(arc) => arc.radius_squared(),
    };
    match is_zero(&value, policy) {
        Some(false) => Ok(()),
        Some(true) => Err(CurveError::Topology(
            "contour overlap event must carry nondegenerate overlap geometry".into(),
        )),
        None => Err(CurveError::Topology(
            "contour overlap event geometry must be certified nondegenerate".into(),
        )),
    }
}

/// One normalized contour-pair topology event.
#[derive(Clone, Debug, PartialEq)]
#[allow(clippy::large_enum_variant)]
pub enum ContourIntersection {
    /// A single point event.
    Point(ContourPointIntersection),
    /// A finite overlap event, boxed to keep point-event arrays compact.
    Overlap(Box<ContourOverlapIntersection>),
    /// Segment-pair classification could not be completed.
    Uncertain(ContourUncertainIntersection),
}

impl ContourIntersection {
    /// Returns the segment index on one side of the event.
    pub const fn segment_index(&self, operand: ContourOperand) -> Option<usize> {
        match self {
            Self::Point(event) => Some(match operand {
                ContourOperand::First => event.a_segment_index,
                ContourOperand::Second => event.b_segment_index,
            }),
            Self::Overlap(event) => Some(match operand {
                ContourOperand::First => event.a_segment_index,
                ContourOperand::Second => event.b_segment_index,
            }),
            Self::Uncertain(event) => Some(match operand {
                ContourOperand::First => event.a_segment_index,
                ContourOperand::Second => event.b_segment_index,
            }),
        }
    }

    /// Returns the retained primitive family on one side of the event.
    pub const fn segment_kind(&self, operand: ContourOperand) -> SegmentKind {
        match self {
            Self::Point(event) => match operand {
                ContourOperand::First => event.a_segment_kind,
                ContourOperand::Second => event.b_segment_kind,
            },
            Self::Overlap(event) => match operand {
                ContourOperand::First => event.a_segment_kind,
                ContourOperand::Second => event.b_segment_kind,
            },
            Self::Uncertain(event) => match operand {
                ContourOperand::First => event.a_segment_kind,
                ContourOperand::Second => event.b_segment_kind,
            },
        }
    }

    fn order_param(
        &self,
        operand: ContourOperand,
        policy: &CurvePolicy,
    ) -> Result<Real, UncertaintyReason> {
        match self {
            Self::Point(event) => Ok(match operand {
                ContourOperand::First => event.a_param.clone(),
                ContourOperand::Second => event.b_param.clone(),
            }),
            Self::Overlap(event) => {
                let range = match operand {
                    ContourOperand::First => &event.a_range,
                    ContourOperand::Second => &event.b_range,
                };
                min_real(range.start().clone(), range.end().clone(), policy)
                    .ok_or(UncertaintyReason::Ordering)
            }
            Self::Uncertain(event) => Err(event.reason),
        }
    }
}

/// A point event between two contours.
#[derive(Clone, Debug, PartialEq)]
pub struct ContourPointIntersection {
    /// Segment index in the first contour.
    pub a_segment_index: usize,
    /// Segment index in the second contour.
    pub b_segment_index: usize,
    /// Primitive family of the first contour segment.
    pub a_segment_kind: SegmentKind,
    /// Primitive family of the second contour segment.
    pub b_segment_kind: SegmentKind,
    /// Intersection point.
    pub point: Point2,
    /// Local parameter on the first contour segment.
    pub a_param: Real,
    /// Local parameter on the second contour segment.
    pub b_param: Real,
    /// Local contact kind.
    pub kind: IntersectionKind,
}

/// A finite overlap event between two contours.
#[derive(Clone, Debug, PartialEq)]
pub struct ContourOverlapIntersection {
    /// Segment index in the first contour.
    pub a_segment_index: usize,
    /// Segment index in the second contour.
    pub b_segment_index: usize,
    /// Primitive family of the first contour segment.
    pub a_segment_kind: SegmentKind,
    /// Primitive family of the second contour segment.
    pub b_segment_kind: SegmentKind,
    /// Overlap geometry.
    pub segment: Segment2,
    /// Parameter range on the first contour segment.
    pub a_range: ParamRange,
    /// Parameter range on the second contour segment.
    pub b_range: ParamRange,
}

/// An uncertain segment-pair relation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ContourUncertainIntersection {
    /// Segment index in the first contour.
    pub a_segment_index: usize,
    /// Segment index in the second contour.
    pub b_segment_index: usize,
    /// Primitive family of the first contour segment.
    pub a_segment_kind: SegmentKind,
    /// Primitive family of the second contour segment.
    pub b_segment_kind: SegmentKind,
    /// Why classification stopped.
    pub reason: UncertaintyReason,
}

pub(crate) fn intersect_contours(
    a: &Contour2,
    b: &Contour2,
    policy: &CurvePolicy,
) -> CurveResult<ContourIntersectionSet> {
    if let (Some(a_boxes), Some(b_boxes)) = (
        a.exact_dyadic_line_aabbs(policy),
        b.exact_dyadic_line_aabbs(policy),
    ) {
        return intersect_contours_with_exact_dyadic_line_aabbs(a, b, &a_boxes, &b_boxes, policy);
    }
    // The bounding-box broad phase is only a candidate filter. This crate keeps
    // the simple pair scan but skips pairs whose boxes are decidably disjoint.
    let a_box = decided_contour_aabb(a, policy);
    let b_box = decided_contour_aabb(b, policy);
    let a_boxes: Vec<_> = a
        .segments()
        .iter()
        .map(|segment| decided_segment_aabb(segment, policy))
        .collect();
    let b_boxes: Vec<_> = b
        .segments()
        .iter()
        .map(|segment| decided_segment_aabb(segment, policy))
        .collect();

    intersect_contours_with_cached_aabbs(
        a,
        b,
        a_box.as_ref(),
        b_box.as_ref(),
        &a_boxes,
        &b_boxes,
        None,
        policy,
    )
}

#[inline(always)]
pub(crate) fn intersect_contours_with_exact_dyadic_line_aabbs(
    a: &Contour2,
    b: &Contour2,
    a_boxes: &crate::contour::ExactDyadicLineAabbs,
    b_boxes: &crate::contour::ExactDyadicLineAabbs,
    policy: &CurvePolicy,
) -> CurveResult<ContourIntersectionSet> {
    const MIN_RETAINED_CERTIFICATE_PAIR_COUNT: usize = 256;
    const MAX_RETAINED_CERTIFICATE_PAIR_COUNT: usize = 4_194_304;
    let pair_count = a.len().saturating_mul(b.len());
    if (MIN_RETAINED_CERTIFICATE_PAIR_COUNT..=MAX_RETAINED_CERTIFICATE_PAIR_COUNT)
        .contains(&pair_count)
        && a.len() <= usize::from(u16::MAX) + 1
        && b.len() <= usize::from(u16::MAX) + 1
    {
        intersect_contours_with_retained_line_candidates::<false>(a, b, a_boxes, b_boxes, policy)
    } else {
        intersect_contours_with_unreserved_exact_dyadic_line_aabbs(a, b, a_boxes, b_boxes, policy)
    }
}

pub(crate) fn intersect_contours_with_exact_dyadic_line_aabbs_point_only(
    a: &Contour2,
    b: &Contour2,
    a_boxes: &crate::contour::ExactDyadicLineAabbs,
    b_boxes: &crate::contour::ExactDyadicLineAabbs,
    policy: &CurvePolicy,
) -> CurveResult<ContourIntersectionSet> {
    const MIN_RETAINED_CERTIFICATE_PAIR_COUNT: usize = 256;
    const MAX_RETAINED_CERTIFICATE_PAIR_COUNT: usize = 4_194_304;
    let pair_count = a.len().saturating_mul(b.len());
    if (MIN_RETAINED_CERTIFICATE_PAIR_COUNT..=MAX_RETAINED_CERTIFICATE_PAIR_COUNT)
        .contains(&pair_count)
        && a.len() <= usize::from(u16::MAX) + 1
        && b.len() <= usize::from(u16::MAX) + 1
    {
        intersect_contours_with_retained_line_candidates::<true>(a, b, a_boxes, b_boxes, policy)
    } else {
        intersect_contours_with_unreserved_exact_dyadic_line_aabbs(a, b, a_boxes, b_boxes, policy)
    }
}

fn intersect_contours_with_unreserved_exact_dyadic_line_aabbs(
    a: &Contour2,
    b: &Contour2,
    a_boxes: &crate::contour::ExactDyadicLineAabbs,
    b_boxes: &crate::contour::ExactDyadicLineAabbs,
    policy: &CurvePolicy,
) -> CurveResult<ContourIntersectionSet> {
    if matches!(
        a.retained_offset_relation(b, policy),
        Some(
            crate::contour::RetainedContourOffsetRelation2::FirstContainsSecond
                | crate::contour::RetainedContourOffsetRelation2::SecondContainsFirst
        )
    ) || a_boxes.contour.is_disjoint(b_boxes.contour)
    {
        return Ok(ContourIntersectionSet::default());
    }
    debug_assert_eq!(a_boxes.segments.len(), a.len());
    debug_assert_eq!(b_boxes.segments.len(), b.len());

    let mut b_order = (0..b.len()).collect::<Vec<_>>();
    b_order.sort_unstable_by(|left, right| {
        b_boxes.segments[*left]
            .min_x
            .total_cmp(&b_boxes.segments[*right].min_x)
            .then_with(|| left.cmp(right))
    });
    let mut events = Vec::new();
    for (a_segment_index, a_box) in a_boxes.segments.iter().copied().enumerate() {
        for &b_segment_index in &b_order {
            let b_box = b_boxes.segments[b_segment_index];
            if b_box.min_x > a_box.max_x {
                break;
            }
            if a_box.is_disjoint(b_box) {
                continue;
            }
            let a_segment = &a.segments()[a_segment_index];
            let b_segment = &b.segments()[b_segment_index];
            let relation =
                a_segment.intersect_segment_with_certified_aabb_overlap(b_segment, policy)?;
            append_segment_relation_events(
                &mut events,
                a_segment_index,
                b_segment_index,
                a_segment,
                b_segment,
                relation,
                policy,
            )?;
        }
    }
    Ok(ContourIntersectionSet::from_normalized_events(events))
}

fn intersect_contours_with_retained_line_candidates<const POINT_ONLY: bool>(
    a: &Contour2,
    b: &Contour2,
    a_boxes: &crate::contour::ExactDyadicLineAabbs,
    b_boxes: &crate::contour::ExactDyadicLineAabbs,
    policy: &CurvePolicy,
) -> CurveResult<ContourIntersectionSet> {
    if matches!(
        a.retained_offset_relation(b, policy),
        Some(
            crate::contour::RetainedContourOffsetRelation2::FirstContainsSecond
                | crate::contour::RetainedContourOffsetRelation2::SecondContainsFirst
        )
    ) || a_boxes.contour.is_disjoint(b_boxes.contour)
    {
        return Ok(ContourIntersectionSet::default());
    }
    debug_assert_eq!(a_boxes.segments.len(), a.len());
    debug_assert_eq!(b_boxes.segments.len(), b.len());
    let Some(b_order) = &b_boxes.min_x_order_with_prefix_max else {
        return intersect_contours_with_unreserved_exact_dyadic_line_aabbs(
            a, b, a_boxes, b_boxes, policy,
        );
    };

    #[derive(Clone, Copy)]
    struct Candidate(u32);

    impl Candidate {
        fn new(a_segment_index: u16, b_segment_index: u16) -> Self {
            Self(u32::from(a_segment_index) | (u32::from(b_segment_index) << 16))
        }

        fn a_segment_index(self) -> u16 {
            self.0 as u16
        }

        fn b_segment_index(self) -> u16 {
            (self.0 >> 16) as u16
        }
    }
    const _: () = assert!(std::mem::size_of::<Candidate>() == 4);

    let mut candidates = Vec::new();
    let mut positive_crossings = 1_u64 << 63;
    for (a_segment_index, a_box) in a_boxes.segments.iter().copied().enumerate() {
        let a_endpoints = a_boxes.segment_endpoints(a_segment_index);
        let a_filter =
            Real::prepare_affine_det2_filter_from_exact_dyadic_f64(a_endpoints[0], a_endpoints[1]);
        let Segment2::Line(a_line) = &a.segments()[a_segment_index] else {
            unreachable!("exact dyadic line bounds contain only line segments");
        };
        let first_possible_overlap = b_boxes.first_possible_x_overlap(a_box.min_x);
        for &packed_b_index in &b_order[first_possible_overlap..] {
            let b_segment_index =
                crate::contour::ExactDyadicLineAabbs::ordered_segment_index(packed_b_index);
            let b_box = b_boxes.segments[b_segment_index];
            if b_box.min_x > a_box.max_x {
                break;
            }
            if a_box.is_disjoint(b_box) {
                continue;
            }
            let b_segment = &b.segments()[b_segment_index];
            let Segment2::Line(b_line) = b_segment else {
                unreachable!("exact dyadic line bounds contain only line segments");
            };
            let b_endpoints = b_boxes.segment_endpoints(b_segment_index);
            let relation = certified_line_segment_support_relation_with_prepared_exact_dyadic_f64(
                a_line,
                b_line,
                a_filter,
                a_endpoints,
                b_endpoints,
            );
            match relation {
                CertifiedLineSegmentSupportRelation::ProperCrossing(sign) => {
                    if candidates.is_empty() {
                        candidates.reserve_exact(a.len().min(b.len()).min(64));
                    }
                    let event_index = candidates.len();
                    // The dispatcher admits this path only when both contour
                    // lengths fit the packed candidate indices.
                    candidates.push(Candidate::new(
                        a_segment_index as u16,
                        b_segment_index as u16,
                    ));
                    if event_index < 63
                        && match sign {
                            hyperreal::RealSign::Positive => true,
                            hyperreal::RealSign::Negative => false,
                            hyperreal::RealSign::Zero => {
                                unreachable!("a certified proper crossing has nonzero orientation")
                            }
                        }
                    {
                        positive_crossings |= 1 << event_index;
                    }
                }
                CertifiedLineSegmentSupportRelation::Unknown => {
                    return intersect_contours_with_unreserved_exact_dyadic_line_aabbs(
                        a, b, a_boxes, b_boxes, policy,
                    );
                }
                CertifiedLineSegmentSupportRelation::Separated => {}
            }
        }
    }
    let mut events = Vec::with_capacity(if POINT_ONLY { 0 } else { candidates.len() });
    let mut crossings = Vec::with_capacity(if POINT_ONLY { candidates.len() } else { 0 });
    let retain_crossing_signs = candidates.len() < 64;
    let mut retained_a_segment_index = None;
    let mut retained_a_line = None;
    for candidate in candidates {
        let a_segment_index_u16 = candidate.a_segment_index();
        let b_segment_index_u16 = candidate.b_segment_index();
        let a_segment_index = usize::from(a_segment_index_u16);
        let b_segment_index = usize::from(b_segment_index_u16);
        let a_segment = &a.segments()[a_segment_index];
        let b_segment = &b.segments()[b_segment_index];
        let (Segment2::Line(a_line), Segment2::Line(b_line)) = (a_segment, b_segment) else {
            unreachable!("exact dyadic line bounds contain only line segments");
        };
        if POINT_ONLY {
            if retained_a_segment_index != Some(a_segment_index_u16) {
                let a_endpoints = a_boxes.segment_endpoints(a_segment_index);
                retained_a_line = ExactDyadicLine2::from_f64(a_endpoints[0], a_endpoints[1]);
                retained_a_segment_index = Some(a_segment_index_u16);
            }
            let Some(retained_a_line) = retained_a_line.as_ref() else {
                return intersect_contours_with_unreserved_exact_dyadic_line_aabbs(
                    a, b, a_boxes, b_boxes, policy,
                );
            };
            let b_endpoints = b_boxes.segment_endpoints(b_segment_index);
            let crossing = match retained_a_line
                .retained_intersection_point_f64(b_endpoints[0], b_endpoints[1])
            {
                Some((parameters, point)) => CertifiedLineCrossingEvent::new_exact_dyadic(
                    a_segment_index_u16,
                    b_segment_index_u16,
                    Point2::from_exact_dyadic_line_point(point),
                    parameters,
                ),
                None => {
                    match retained_a_line
                        .wide_retained_intersection_point_f64(b_endpoints[0], b_endpoints[1])
                    {
                        Some((parameters, point)) => {
                            CertifiedLineCrossingEvent::new_exact_dyadic_wide(
                                a_segment_index_u16,
                                b_segment_index_u16,
                                Point2::from_exact_dyadic_wide_line_point(point),
                                parameters,
                            )
                        }
                        None => {
                            let LineLineIntersection::Point {
                                point,
                                a_param,
                                b_param,
                                ..
                            } = a_line.intersect_line_with_certified_exact_dyadic_proper_crossing(
                                b_line, policy,
                            )?
                            else {
                                return intersect_contours_with_unreserved_exact_dyadic_line_aabbs(
                                    a, b, a_boxes, b_boxes, policy,
                                );
                            };
                            CertifiedLineCrossingEvent::new_materialized(
                                a_segment_index_u16,
                                b_segment_index_u16,
                                point,
                                a_param,
                                b_param,
                            )
                        }
                    }
                }
            };
            crossings.push(crossing);
        } else {
            let LineLineIntersection::Point {
                point,
                a_param,
                b_param,
                kind,
            } = a_line
                .intersect_line_with_certified_exact_dyadic_proper_crossing(b_line, policy)?
            else {
                return intersect_contours_with_unreserved_exact_dyadic_line_aabbs(
                    a, b, a_boxes, b_boxes, policy,
                );
            };
            events.push(ContourIntersection::Point(ContourPointIntersection {
                a_segment_index,
                b_segment_index,
                a_segment_kind: SegmentKind::Line,
                b_segment_kind: SegmentKind::Line,
                point,
                a_param,
                b_param,
                kind,
            }));
        }
    }
    let signs = retain_crossing_signs
        .then(|| NonZeroU64::new(positive_crossings).expect("the retained marker bit is nonzero"));
    Ok(if POINT_ONLY {
        ContourIntersectionSet::from_certified_line_crossing_points(crossings, signs)
    } else {
        ContourIntersectionSet::from_certified_line_crossings(events, signs)
    })
}

pub(crate) fn intersect_contour_self(
    contour: &Contour2,
    policy: &CurvePolicy,
) -> CurveResult<ContourIntersectionSet> {
    let segment_boxes: Vec<_> = contour
        .segments()
        .iter()
        .map(|segment| decided_segment_aabb(segment, policy))
        .collect();

    intersect_contour_self_with_cached_aabbs(contour, &segment_boxes, policy)
}

pub(crate) fn intersect_contours_with_cached_aabbs(
    a: &Contour2,
    b: &Contour2,
    a_box: Option<&Aabb2>,
    b_box: Option<&Aabb2>,
    a_segment_boxes: &[Option<Aabb2>],
    b_segment_boxes: &[Option<Aabb2>],
    b_x_index: Option<&SegmentAabbXIndex>,
    policy: &CurvePolicy,
) -> CurveResult<ContourIntersectionSet> {
    if matches!(
        a.retained_offset_relation(b, policy),
        Some(
            crate::contour::RetainedContourOffsetRelation2::FirstContainsSecond
                | crate::contour::RetainedContourOffsetRelation2::SecondContainsFirst
        )
    ) {
        return Ok(ContourIntersectionSet::default());
    }

    if let (Some(a_box), Some(b_box)) = (a_box, b_box)
        && aabbs_decided_disjoint(a_box, b_box, policy)
    {
        return Ok(ContourIntersectionSet::default());
    }

    let mut events = Vec::new();
    if let Some(result) = visit_swept_segment_pair_candidates(
        a_segment_boxes,
        b_segment_boxes,
        a.segments().len(),
        b.segments().len(),
        b_x_index,
        policy,
        |a_segment_index, b_segment_index, aabb_overlap_certified| {
            let a_segment = &a.segments()[a_segment_index];
            let b_segment = &b.segments()[b_segment_index];
            let relation = if aabb_overlap_certified {
                a_segment.intersect_segment_with_certified_aabb_overlap(b_segment, policy)?
            } else {
                a_segment.intersect_segment(b_segment, policy)?
            };
            append_segment_relation_events(
                &mut events,
                a_segment_index,
                b_segment_index,
                a_segment,
                b_segment,
                relation,
                policy,
            )
        },
    ) {
        result?;
    } else {
        for (a_segment_index, a_segment) in a.segments().iter().enumerate() {
            for (b_segment_index, b_segment) in b.segments().iter().enumerate() {
                if let (Some(Some(a_box)), Some(Some(b_box))) = (
                    a_segment_boxes.get(a_segment_index),
                    b_segment_boxes.get(b_segment_index),
                ) && aabbs_decided_disjoint(a_box, b_box, policy)
                {
                    continue;
                }

                let relation = a_segment.intersect_segment(b_segment, policy)?;
                append_segment_relation_events(
                    &mut events,
                    a_segment_index,
                    b_segment_index,
                    a_segment,
                    b_segment,
                    relation,
                    policy,
                )?;
            }
        }
    }

    Ok(ContourIntersectionSet::from_normalized_events(events))
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct SegmentAabbXIndex {
    ordered: Vec<usize>,
    unknown: Vec<usize>,
    subtree_maxima: Vec<usize>,
}

impl SegmentAabbXIndex {
    pub(crate) fn try_new(
        boxes: &[Option<Aabb2>],
        segment_count: usize,
        policy: &CurvePolicy,
    ) -> Option<Self> {
        let mut ordered: Vec<_> = (0..segment_count)
            .filter(|index| boxes.get(*index).and_then(Option::as_ref).is_some())
            .collect();
        if !sort_segment_indices_by_certified_box_coordinate(
            &mut ordered,
            boxes,
            segment_count,
            policy,
            Aabb2::min_x,
        ) {
            return None;
        }

        let unknown = (0..segment_count)
            .filter(|index| boxes.get(*index).and_then(Option::as_ref).is_none())
            .collect();
        Some(Self {
            ordered,
            unknown,
            subtree_maxima: Vec::new(),
        })
    }

    pub(crate) fn prepare_interval_queries(
        &mut self,
        boxes: &[Option<Aabb2>],
        policy: &CurvePolicy,
    ) -> bool {
        self.subtree_maxima.resize(self.ordered.len(), 0);
        if !self.ordered.is_empty()
            && build_x_interval_index(
                boxes,
                &self.ordered,
                &mut self.subtree_maxima,
                0,
                self.ordered.len(),
                policy,
            )
            .is_none()
        {
            self.subtree_maxima.clear();
        }
        self.supports_interval_queries()
    }

    fn segment_count(&self) -> usize {
        self.ordered.len() + self.unknown.len()
    }

    fn supports_interval_queries(&self) -> bool {
        self.ordered.is_empty() || self.subtree_maxima.len() == self.ordered.len()
    }

    fn collect(
        &self,
        boxes: &[Option<Aabb2>],
        query: &Aabb2,
        policy: &CurvePolicy,
        candidates: &mut Vec<usize>,
    ) {
        self.collect_range(boxes, 0, self.ordered.len(), query, policy, candidates);
    }

    pub(crate) fn collect_overlapping(
        &self,
        boxes: &[Option<Aabb2>],
        query: &Aabb2,
        policy: &CurvePolicy,
        candidates: &mut Vec<usize>,
    ) {
        candidates.extend(self.unknown.iter().copied());
        if self.supports_interval_queries() {
            self.collect(boxes, query, policy, candidates);
        } else {
            candidates.extend(self.ordered.iter().copied().filter(|&index| {
                !aabbs_decided_disjoint(query, boxes[index].as_ref().unwrap(), policy)
            }));
        }
    }

    fn collect_range(
        &self,
        boxes: &[Option<Aabb2>],
        start: usize,
        end: usize,
        query: &Aabb2,
        policy: &CurvePolicy,
        candidates: &mut Vec<usize>,
    ) {
        if start == end
            || matches!(
                compare_box(
                    boxes[self.ordered[start]].as_ref().unwrap().min_x(),
                    query.max_x(),
                    policy,
                ),
                Some(Ordering::Greater)
            )
        {
            return;
        }
        let middle = start + (end - start) / 2;
        if matches!(
            compare_box(
                boxes[self.ordered[self.subtree_maxima[middle]]]
                    .as_ref()
                    .unwrap()
                    .max_x(),
                query.min_x(),
                policy,
            ),
            Some(Ordering::Less)
        ) {
            return;
        }
        self.collect_range(boxes, start, middle, query, policy, candidates);
        let index = self.ordered[middle];
        if !aabbs_decided_disjoint(query, boxes[index].as_ref().unwrap(), policy) {
            candidates.push(index);
        }
        self.collect_range(boxes, middle + 1, end, query, policy, candidates);
    }
}

pub(crate) type BoxCoordinate = for<'a> fn(&'a Aabb2) -> &'a Real;

fn compare_box(left: &Real, right: &Real, policy: &CurvePolicy) -> Option<Ordering> {
    if std::ptr::eq(left, right) {
        return Some(Ordering::Equal);
    }
    if let (Some(left_rational), Some(right_rational)) =
        (left.exact_rational_ref(), right.exact_rational_ref())
    {
        // Lossless binary64 views preserve exact order. Use them only after
        // native-word comparison stops being the cheaper broad-phase filter.
        let wide = left_rational.numerator().bits() > 32
            || left_rational.denominator().bits() > 32
            || right_rational.numerator().bits() > 32
            || right_rational.denominator().bits() > 32;
        if wide
            && let (Some(left), Some(right)) =
                (left.to_f64_exact_dyadic(), right.to_f64_exact_dyadic())
        {
            return left.partial_cmp(&right);
        }
        return left_rational.partial_cmp(right_rational);
    }
    compare_reals(left, right, policy)
}

pub(crate) fn sort_segment_indices_by_certified_box_coordinate(
    ordered: &mut [usize],
    boxes: &[Option<Aabb2>],
    segment_count: usize,
    policy: &CurvePolicy,
    coordinate: BoxCoordinate,
) -> bool {
    let mut preview = vec![0.0; segment_count];
    let used_preview = ordered.iter().all(|&index| {
        let Some(value) = coordinate(boxes[index].as_ref().unwrap())
            .to_f64_lossy()
            .filter(|value| value.is_finite())
        else {
            return false;
        };
        preview[index] = value;
        true
    });
    if used_preview {
        ordered.sort_unstable_by(|left, right| {
            preview[*left]
                .total_cmp(&preview[*right])
                .then_with(|| left.cmp(right))
        });
    } else {
        sort_segment_indices_by_exact_box_coordinate(ordered, boxes, policy, coordinate);
    }
    drop(preview);

    let mut certified =
        segment_box_coordinate_order_is_certified(ordered, boxes, policy, coordinate);
    if !certified && used_preview {
        sort_segment_indices_by_exact_box_coordinate(ordered, boxes, policy, coordinate);
        certified = segment_box_coordinate_order_is_certified(ordered, boxes, policy, coordinate);
    }
    certified
}

fn sort_segment_indices_by_exact_box_coordinate(
    ordered: &mut [usize],
    boxes: &[Option<Aabb2>],
    policy: &CurvePolicy,
    coordinate: BoxCoordinate,
) {
    ordered.sort_unstable_by(|left, right| {
        compare_box(
            coordinate(boxes[*left].as_ref().unwrap()),
            coordinate(boxes[*right].as_ref().unwrap()),
            policy,
        )
        .unwrap_or(Ordering::Equal)
        .then_with(|| left.cmp(right))
    });
}

fn segment_box_coordinate_order_is_certified(
    ordered: &[usize],
    boxes: &[Option<Aabb2>],
    policy: &CurvePolicy,
    coordinate: BoxCoordinate,
) -> bool {
    !ordered.windows(2).any(|window| {
        !matches!(
            compare_box(
                coordinate(boxes[window[0]].as_ref().unwrap()),
                coordinate(boxes[window[1]].as_ref().unwrap()),
                policy,
            ),
            Some(Ordering::Less | Ordering::Equal)
        )
    })
}

struct DenseAabbRankSchedule {
    cuts: Vec<[u32; 4]>,
    ranks: Vec<[u32; 3]>,
}

impl DenseAabbRankSchedule {
    fn try_new(
        first_boxes: &[Option<Aabb2>],
        second_boxes: &[Option<Aabb2>],
        first_segment_count: usize,
        second_segment_count: usize,
        second_index: &SegmentAabbXIndex,
        policy: &CurvePolicy,
    ) -> Option<Self> {
        if matches!(policy.mode, crate::policy::NumericMode::EdgePreview) {
            return None;
        }
        u32::try_from(second_segment_count).ok()?;
        let first = first_boxes.get(..first_segment_count)?;
        if first.iter().any(Option::is_none) {
            return None;
        }
        let mut cuts = vec![[0; 4]; first_segment_count];
        for (cut, bbox) in cuts.iter_mut().zip(first) {
            cut[0] = u32::try_from(sorted_box_coordinate_partition(
                &second_index.ordered,
                second_boxes,
                bbox.as_ref().unwrap().max_x(),
                policy,
                Aabb2::min_x,
                true,
            )?)
            .ok()?;
        }

        let dimensions: [(BoxCoordinate, BoxCoordinate, bool); 3] = [
            (Aabb2::max_x, Aabb2::min_x, false),
            (Aabb2::min_y, Aabb2::max_y, true),
            (Aabb2::max_y, Aabb2::min_y, false),
        ];
        let mut ranks = vec![[u32::MAX; 3]; second_segment_count];
        for (rank_slot, (second_coordinate, first_query, include_equal)) in
            dimensions.into_iter().enumerate()
        {
            let mut ordered = second_index.ordered.clone();
            sort_segment_indices_by_certified_box_coordinate(
                &mut ordered,
                second_boxes,
                second_segment_count,
                policy,
                second_coordinate,
            )
            .then_some(())?;
            for (rank, index) in ordered.iter().copied().enumerate() {
                ranks[index][rank_slot] = u32::try_from(rank).ok()?;
            }
            for (cut, bbox) in cuts.iter_mut().zip(first) {
                cut[rank_slot + 1] = u32::try_from(sorted_box_coordinate_partition(
                    &ordered,
                    second_boxes,
                    first_query(bbox.as_ref().unwrap()),
                    policy,
                    second_coordinate,
                    include_equal,
                )?)
                .ok()?;
            }
        }
        Some(Self { cuts, ranks })
    }
}

fn sorted_box_coordinate_partition(
    ordered: &[usize],
    boxes: &[Option<Aabb2>],
    query: &Real,
    policy: &CurvePolicy,
    coordinate: BoxCoordinate,
    include_equal_in_lower_partition: bool,
) -> Option<usize> {
    let mut start = 0;
    let mut end = ordered.len();
    while start < end {
        let middle = start + (end - start) / 2;
        let bbox = boxes[ordered[middle]].as_ref()?;
        match compare_box(coordinate(bbox), query, policy)? {
            Ordering::Less => start = middle + 1,
            Ordering::Equal if include_equal_in_lower_partition => start = middle + 1,
            Ordering::Equal | Ordering::Greater => end = middle,
        }
    }
    Some(start)
}

fn visit_swept_segment_pair_candidates<F>(
    first_boxes: &[Option<Aabb2>],
    second_boxes: &[Option<Aabb2>],
    first_segment_count: usize,
    second_segment_count: usize,
    prepared_second_index: Option<&SegmentAabbXIndex>,
    policy: &CurvePolicy,
    mut visit: F,
) -> Option<CurveResult<()>>
where
    F: FnMut(usize, usize, bool) -> CurveResult<()>,
{
    const MIN_CARTESIAN_PAIR_COUNT: usize = 256;
    const MIN_RANKED_PAIR_COUNT: usize = 4_096;
    const MIN_INDEXED_PAIR_COUNT: usize = 16_384;

    let cartesian_pair_count = first_segment_count.checked_mul(second_segment_count)?;
    if cartesian_pair_count < MIN_CARTESIAN_PAIR_COUNT {
        return None;
    }

    let prepared_index =
        prepared_second_index.filter(|index| index.segment_count() == second_segment_count);
    let mut local_index = match prepared_index {
        Some(_) => None,
        None => Some(SegmentAabbXIndex::try_new(
            second_boxes,
            second_segment_count,
            policy,
        )?),
    };
    let sparse = cartesian_pair_count >= MIN_INDEXED_PAIR_COUNT
        && !sampled_x_overlap_is_dense(
            first_boxes,
            second_boxes,
            prepared_index.unwrap_or_else(|| local_index.as_ref().unwrap()),
            policy,
        );
    if sparse && prepared_index.is_none() {
        local_index
            .as_mut()
            .unwrap()
            .prepare_interval_queries(second_boxes, policy);
    }
    let second_index = prepared_index.unwrap_or_else(|| local_index.as_ref().unwrap());
    let indexed = sparse && second_index.supports_interval_queries();
    let rank_schedule = (cartesian_pair_count >= MIN_RANKED_PAIR_COUNT && !indexed)
        .then(|| {
            DenseAabbRankSchedule::try_new(
                first_boxes,
                second_boxes,
                first_segment_count,
                second_segment_count,
                second_index,
                policy,
            )
        })
        .flatten();
    let mut candidates = Vec::with_capacity(second_segment_count);
    for first_index in 0..first_segment_count {
        candidates.clear();
        if let Some(Some(first_box)) = first_boxes.get(first_index) {
            candidates.extend(second_index.unknown.iter().copied());
            if indexed {
                second_index.collect(second_boxes, first_box, policy, &mut candidates);
            } else if let Some(rank_schedule) = &rank_schedule {
                let [min_x_end, max_x_start, min_y_end, max_y_start] =
                    rank_schedule.cuts[first_index];
                for &candidate_index in &second_index.ordered[..min_x_end as usize] {
                    let [max_x_rank, min_y_rank, max_y_rank] = rank_schedule.ranks[candidate_index];
                    if max_x_rank < max_x_start
                        || min_y_rank >= min_y_end
                        || max_y_rank < max_y_start
                    {
                        continue;
                    }
                    candidates.push(candidate_index);
                }
            } else {
                for &candidate_index in &second_index.ordered {
                    let second_box = second_boxes[candidate_index].as_ref().unwrap();
                    match compare_box(second_box.min_x(), first_box.max_x(), policy) {
                        Some(Ordering::Greater) => break,
                        Some(Ordering::Less | Ordering::Equal) | None => {}
                    }
                    if matches!(
                        compare_box(second_box.max_x(), first_box.min_x(), policy),
                        Some(Ordering::Less)
                    ) || matches!(
                        compare_box(first_box.max_y(), second_box.min_y(), policy),
                        Some(Ordering::Less)
                    ) || matches!(
                        compare_box(second_box.max_y(), first_box.min_y(), policy),
                        Some(Ordering::Less)
                    ) {
                        continue;
                    }
                    candidates.push(candidate_index);
                }
            }
        } else {
            candidates.extend(0..second_segment_count);
        }
        candidates.sort_unstable();
        for second_index in candidates.iter().copied() {
            let aabb_overlap_certified = rank_schedule.is_some()
                && first_boxes
                    .get(first_index)
                    .and_then(Option::as_ref)
                    .is_some()
                && second_boxes
                    .get(second_index)
                    .and_then(Option::as_ref)
                    .is_some();
            if let Err(error) = visit(first_index, second_index, aabb_overlap_certified) {
                return Some(Err(error));
            }
        }
    }
    Some(Ok(()))
}

fn sampled_x_overlap_is_dense(
    first: &[Option<Aabb2>],
    second_boxes: &[Option<Aabb2>],
    second_index: &SegmentAabbXIndex,
    policy: &CurvePolicy,
) -> bool {
    const SAMPLE_COUNT: usize = 8;

    let first_count = first.len().min(SAMPLE_COUNT);
    let second_count = second_index.ordered.len().min(SAMPLE_COUNT);
    let sample_count = first_count * second_count;
    if sample_count == 0 {
        return false;
    }
    let mut overlaps = 0;
    for first_sample in 0..first_count {
        let first_index = first_sample * (first.len() - 1) / first_count.saturating_sub(1).max(1);
        let Some(first) = &first[first_index] else {
            return true;
        };
        for second_sample in 0..second_count {
            let second_position = second_sample * (second_index.ordered.len() - 1)
                / second_count.saturating_sub(1).max(1);
            let second = second_boxes[second_index.ordered[second_position]]
                .as_ref()
                .unwrap();
            overlaps += usize::from(
                !matches!(
                    compare_box(first.max_x(), second.min_x(), policy),
                    Some(Ordering::Less)
                ) && !matches!(
                    compare_box(second.max_x(), first.min_x(), policy),
                    Some(Ordering::Less)
                ),
            );
        }
    }
    overlaps * 8 > sample_count
}

fn build_x_interval_index(
    boxes: &[Option<Aabb2>],
    ordered: &[usize],
    maxima: &mut [usize],
    start: usize,
    end: usize,
    policy: &CurvePolicy,
) -> Option<usize> {
    let middle = start + (end - start) / 2;
    let mut maximum = middle;
    for candidate in [
        (start < middle)
            .then(|| build_x_interval_index(boxes, ordered, maxima, start, middle, policy)),
        (middle + 1 < end)
            .then(|| build_x_interval_index(boxes, ordered, maxima, middle + 1, end, policy)),
    ]
    .into_iter()
    .flatten()
    {
        let candidate = candidate?;
        if matches!(
            compare_box(
                boxes[ordered[maximum]].as_ref().unwrap().max_x(),
                boxes[ordered[candidate]].as_ref().unwrap().max_x(),
                policy,
            )?,
            Ordering::Less
        ) {
            maximum = candidate;
        }
    }
    maxima[middle] = maximum;
    Some(maximum)
}

pub(crate) fn intersect_contour_self_with_cached_aabbs(
    contour: &Contour2,
    segment_boxes: &[Option<Aabb2>],
    policy: &CurvePolicy,
) -> CurveResult<ContourIntersectionSet> {
    let segments = contour.segments();
    let mut events = Vec::new();

    for first_index in 0..segments.len() {
        for second_index in (first_index + 1)..segments.len() {
            if let (Some(Some(first_box)), Some(Some(second_box))) = (
                segment_boxes.get(first_index),
                segment_boxes.get(second_index),
            ) && aabbs_decided_disjoint(first_box, second_box, policy)
            {
                continue;
            }

            let relation =
                segments[first_index].intersect_segment(&segments[second_index], policy)?;
            let mut pair_events = Vec::new();
            append_segment_relation_events(
                &mut pair_events,
                first_index,
                second_index,
                &segments[first_index],
                &segments[second_index],
                relation,
                policy,
            )?;

            for event in pair_events {
                if is_contour_connectivity_event(
                    &event,
                    segments,
                    first_index,
                    second_index,
                    policy,
                ) {
                    continue;
                }
                events.push(event);
            }
        }
    }

    Ok(ContourIntersectionSet::from_normalized_events(events))
}

fn append_segment_relation_events(
    events: &mut Vec<ContourIntersection>,
    a_segment_index: usize,
    b_segment_index: usize,
    a_segment: &Segment2,
    b_segment: &Segment2,
    relation: SegmentIntersection,
    policy: &CurvePolicy,
) -> CurveResult<()> {
    match relation {
        SegmentIntersection::LineLine(LineLineIntersection::None) => {}
        SegmentIntersection::LineLine(LineLineIntersection::Point {
            point,
            a_param,
            b_param,
            kind,
        }) => events.push(ContourIntersection::Point(ContourPointIntersection {
            a_segment_index,
            b_segment_index,
            a_segment_kind: a_segment.kind(),
            b_segment_kind: b_segment.kind(),
            point,
            a_param,
            b_param,
            kind,
        })),
        SegmentIntersection::LineLine(LineLineIntersection::Overlap {
            segment,
            a_range,
            b_range,
        }) => events.push(ContourIntersection::Overlap(Box::new(
            ContourOverlapIntersection {
                a_segment_index,
                b_segment_index,
                a_segment_kind: a_segment.kind(),
                b_segment_kind: b_segment.kind(),
                segment: Segment2::Line(segment),
                a_range,
                b_range,
            },
        ))),
        SegmentIntersection::LineLine(LineLineIntersection::Uncertain { reason }) => {
            append_uncertain(
                events,
                a_segment_index,
                b_segment_index,
                a_segment,
                b_segment,
                reason,
            );
        }
        SegmentIntersection::LineArc { order, result } => {
            append_line_arc_events(
                events,
                a_segment_index,
                b_segment_index,
                a_segment,
                b_segment,
                order,
                result,
                policy,
            )?;
        }
        SegmentIntersection::ArcArc(ArcArcIntersection::None) => {}
        SegmentIntersection::ArcArc(ArcArcIntersection::Point(hit)) => {
            append_certified_point_event(
                events,
                a_segment_index,
                b_segment_index,
                a_segment,
                b_segment,
                hit.point,
                hit.a_param,
                hit.b_param,
                hit.kind,
                policy,
            );
        }
        SegmentIntersection::ArcArc(ArcArcIntersection::TwoPoints { first, second }) => {
            append_certified_point_event(
                events,
                a_segment_index,
                b_segment_index,
                a_segment,
                b_segment,
                first.point,
                first.a_param,
                first.b_param,
                first.kind,
                policy,
            );
            append_certified_point_event(
                events,
                a_segment_index,
                b_segment_index,
                a_segment,
                b_segment,
                second.point,
                second.a_param,
                second.b_param,
                second.kind,
                policy,
            );
        }
        SegmentIntersection::ArcArc(ArcArcIntersection::Overlap {
            segment,
            a_range,
            b_range,
        }) => events.push(ContourIntersection::Overlap(Box::new(
            ContourOverlapIntersection {
                a_segment_index,
                b_segment_index,
                a_segment_kind: a_segment.kind(),
                b_segment_kind: b_segment.kind(),
                segment: Segment2::Arc(segment),
                a_range,
                b_range,
            },
        ))),
        SegmentIntersection::ArcArc(ArcArcIntersection::Uncertain { reason }) => {
            append_uncertain(
                events,
                a_segment_index,
                b_segment_index,
                a_segment,
                b_segment,
                reason,
            );
        }
    }

    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn append_line_arc_events(
    events: &mut Vec<ContourIntersection>,
    a_segment_index: usize,
    b_segment_index: usize,
    a_segment: &Segment2,
    b_segment: &Segment2,
    order: LineArcOrder,
    result: LineArcIntersection,
    policy: &CurvePolicy,
) -> CurveResult<()> {
    match result {
        LineArcIntersection::None => {}
        LineArcIntersection::Point(hit) => {
            append_line_arc_hit(
                events,
                a_segment_index,
                b_segment_index,
                a_segment,
                b_segment,
                order,
                hit,
                policy,
            )?;
        }
        LineArcIntersection::TwoPoints { first, second } => {
            append_line_arc_hit(
                events,
                a_segment_index,
                b_segment_index,
                a_segment,
                b_segment,
                order,
                first,
                policy,
            )?;
            append_line_arc_hit(
                events,
                a_segment_index,
                b_segment_index,
                a_segment,
                b_segment,
                order,
                second,
                policy,
            )?;
        }
        LineArcIntersection::Uncertain { reason } => {
            append_uncertain(
                events,
                a_segment_index,
                b_segment_index,
                a_segment,
                b_segment,
                reason,
            );
        }
    }

    Ok(())
}

fn append_line_arc_hit(
    events: &mut Vec<ContourIntersection>,
    a_segment_index: usize,
    b_segment_index: usize,
    a_segment: &Segment2,
    b_segment: &Segment2,
    order: LineArcOrder,
    hit: crate::LineArcIntersectionPoint,
    policy: &CurvePolicy,
) -> CurveResult<()> {
    let (a_param, b_param) = match order {
        LineArcOrder::LineThenArc => (hit.line_param, hit.arc_param),
        LineArcOrder::ArcThenLine => (hit.arc_param, hit.line_param),
    };

    append_certified_point_event(
        events,
        a_segment_index,
        b_segment_index,
        a_segment,
        b_segment,
        hit.point,
        a_param,
        b_param,
        hit.kind,
        policy,
    );

    Ok(())
}

fn append_certified_point_event(
    events: &mut Vec<ContourIntersection>,
    a_segment_index: usize,
    b_segment_index: usize,
    a_segment: &Segment2,
    b_segment: &Segment2,
    point: Point2,
    a_param: Real,
    b_param: Real,
    kind: IntersectionKind,
    policy: &CurvePolicy,
) {
    if in_closed_unit_interval(&a_param, policy) == Some(true)
        && in_closed_unit_interval(&b_param, policy) == Some(true)
    {
        events.push(ContourIntersection::Point(ContourPointIntersection {
            a_segment_index,
            b_segment_index,
            a_segment_kind: a_segment.kind(),
            b_segment_kind: b_segment.kind(),
            point,
            a_param,
            b_param,
            kind,
        }));
    } else {
        append_uncertain(
            events,
            a_segment_index,
            b_segment_index,
            a_segment,
            b_segment,
            UncertaintyReason::Ordering,
        );
    }
}

fn append_uncertain(
    events: &mut Vec<ContourIntersection>,
    a_segment_index: usize,
    b_segment_index: usize,
    a_segment: &Segment2,
    b_segment: &Segment2,
    reason: UncertaintyReason,
) {
    events.push(ContourIntersection::Uncertain(
        ContourUncertainIntersection {
            a_segment_index,
            b_segment_index,
            a_segment_kind: a_segment.kind(),
            b_segment_kind: b_segment.kind(),
            reason,
        },
    ));
}

fn is_contour_connectivity_event(
    event: &ContourIntersection,
    segments: &[Segment2],
    first_index: usize,
    second_index: usize,
    policy: &CurvePolicy,
) -> bool {
    let Some(shared_point) = connected_contour_vertex(segments, first_index, second_index) else {
        return false;
    };

    match event {
        ContourIntersection::Point(point) => {
            points_match_for_connectivity(&point.point, shared_point, policy)
        }
        ContourIntersection::Overlap(_) | ContourIntersection::Uncertain(_) => false,
    }
}

fn connected_contour_vertex(
    segments: &[Segment2],
    first_index: usize,
    second_index: usize,
) -> Option<&Point2> {
    if first_index + 1 == second_index {
        return Some(segments[first_index].end());
    }

    if first_index == 0 && second_index + 1 == segments.len() {
        return Some(segments[first_index].start());
    }

    None
}

fn points_match_for_connectivity(point: &Point2, expected: &Point2, policy: &CurvePolicy) -> bool {
    let distance = point.distance_squared(expected);
    if is_zero(&distance, policy) == Some(true) {
        return true;
    }

    if matches!(policy.mode, crate::policy::NumericMode::EdgePreview)
        && let (Some(distance), Some(tolerance)) =
            (distance.to_f64_lossy(), policy.preview_tolerance)
    {
        let tolerance = tolerance.absolute.max(tolerance.relative);
        return distance.is_finite() && distance <= tolerance * tolerance;
    }

    false
}

fn insertion_index(
    sorted: &[(&ContourIntersection, Real)],
    order_param: &Real,
    policy: &CurvePolicy,
) -> Option<usize> {
    for (index, (_, existing_param)) in sorted.iter().enumerate() {
        match compare_reals(order_param, existing_param, policy)? {
            std::cmp::Ordering::Less => return Some(index),
            std::cmp::Ordering::Equal | std::cmp::Ordering::Greater => {}
        }
    }
    Some(sorted.len())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bbox(min_x: i32, min_y: i32, max_x: i32, max_y: i32) -> Aabb2 {
        Aabb2::new_unchecked(
            Point2::new(Real::from(min_x), Real::from(min_y)),
            Point2::new(Real::from(max_x), Real::from(max_y)),
        )
    }

    fn rectangle(min_x: i32, min_y: i32, max_x: i32, max_y: i32) -> Contour2 {
        let points = [
            Point2::new(min_x.into(), min_y.into()),
            Point2::new(max_x.into(), min_y.into()),
            Point2::new(max_x.into(), max_y.into()),
            Point2::new(min_x.into(), max_y.into()),
        ];
        Contour2::try_new(
            (0..points.len())
                .map(|index| {
                    crate::LineSeg2::try_new(
                        points[index].clone(),
                        points[(index + 1) % points.len()].clone(),
                    )
                    .map(Segment2::Line)
                })
                .collect::<CurveResult<Vec<_>>>()
                .unwrap(),
        )
        .unwrap()
    }

    fn star(vertex_count: usize, center: (f64, f64), radii: (f64, f64), rotation: f64) -> Contour2 {
        let points = (0..vertex_count)
            .map(|index| {
                let angle = rotation + std::f64::consts::TAU * index as f64 / vertex_count as f64;
                let radius = if index % 2 == 0 { radii.0 } else { radii.1 };
                Point2::new(
                    Real::try_from(center.0 + radius * angle.cos()).unwrap(),
                    Real::try_from(center.1 + radius * angle.sin()).unwrap(),
                )
            })
            .collect::<Vec<_>>();
        Contour2::try_new(
            (0..points.len())
                .map(|index| {
                    crate::LineSeg2::try_new(
                        points[index].clone(),
                        points[(index + 1) % points.len()].clone(),
                    )
                    .map(Segment2::Line)
                })
                .collect::<CurveResult<Vec<_>>>()
                .unwrap(),
        )
        .unwrap()
    }

    #[test]
    fn exact_dyadic_line_sweep_matches_exact_box_sweep() {
        let policy = CurvePolicy::certified();
        let first = rectangle(0, 0, 8, 6);
        let second = rectangle(3, -2, 11, 4);
        let first_box = decided_contour_aabb(&first, &policy);
        let second_box = decided_contour_aabb(&second, &policy);
        let first_boxes = first
            .segments()
            .iter()
            .map(|segment| decided_segment_aabb(segment, &policy))
            .collect::<Vec<_>>();
        let second_boxes = second
            .segments()
            .iter()
            .map(|segment| decided_segment_aabb(segment, &policy))
            .collect::<Vec<_>>();
        let exact_box_events = intersect_contours_with_cached_aabbs(
            &first,
            &second,
            first_box.as_ref(),
            second_box.as_ref(),
            &first_boxes,
            &second_boxes,
            None,
            &policy,
        )
        .unwrap();
        let dyadic_events = intersect_contours(&first, &second, &policy).unwrap();

        assert_eq!(dyadic_events, exact_box_events);
    }

    #[test]
    fn retained_line_candidates_match_unreserved_dense_sweep() {
        let policy = CurvePolicy::certified();
        let first = star(64, (0.0, 0.0), (100.0, 72.0), 0.0);
        let second = star(64, (18.0, 7.0), (96.0, 68.0), std::f64::consts::PI / 64.0);
        let first_boxes = first.exact_dyadic_line_aabbs(&policy).unwrap();
        let second_boxes = second.exact_dyadic_line_aabbs(&policy).unwrap();
        assert!(
            std::mem::size_of::<CertifiedLineCrossingEvent>()
                <= std::mem::size_of::<ContourPointIntersection>(),
            "the lazy exact crossing carrier must not exceed an eager point event"
        );

        let retained = intersect_contours_with_retained_line_candidates::<false>(
            &first,
            &second,
            &first_boxes,
            &second_boxes,
            &policy,
        )
        .unwrap();
        let point_only = intersect_contours_with_retained_line_candidates::<true>(
            &first,
            &second,
            &first_boxes,
            &second_boxes,
            &policy,
        )
        .unwrap();
        let unreserved = intersect_contours_with_unreserved_exact_dyadic_line_aabbs(
            &first,
            &second,
            &first_boxes,
            &second_boxes,
            &policy,
        )
        .unwrap();

        assert!(retained.len() < 64);
        assert!(point_only.retained_certified_line_crossings().is_some());
        for (event_index, event) in retained.events().iter().enumerate() {
            let ContourIntersection::Point(point) = event else {
                panic!("the certified candidate path emits only point crossings");
            };
            let Segment2::Line(first_line) = &first.segments()[point.a_segment_index] else {
                unreachable!();
            };
            let Segment2::Line(second_line) = &second.segments()[point.b_segment_index] else {
                unreachable!();
            };
            assert_eq!(
                retained.certified_line_crossing_delta(event_index),
                crate::intersect::certified_line_crossing_winding_delta(first_line, second_line),
            );
        }
        assert_eq!(point_only, retained);
        assert_eq!(retained, unreserved);
    }

    fn flat_candidates(
        first: &[Option<Aabb2>],
        second: &[Option<Aabb2>],
        policy: &CurvePolicy,
    ) -> Vec<(usize, usize)> {
        let mut candidates = Vec::new();
        for (first_index, first_box) in first.iter().enumerate() {
            for (second_index, second_box) in second.iter().enumerate() {
                if !matches!((first_box, second_box), (Some(first_box), Some(second_box))
                    if aabbs_decided_disjoint(first_box, second_box, policy))
                {
                    candidates.push((first_index, second_index));
                }
            }
        }
        candidates
    }

    #[test]
    fn indexed_sweep_candidates_match_flat_decided_aabb_filter() {
        let policy = CurvePolicy::certified();
        let first: Vec<_> = (0..128)
            .map(|index| Some(bbox(index * 3, -1, index * 3 + 2, 1)))
            .collect();
        let mut second: Vec<_> = (0..128)
            .map(|index| Some(bbox(index * 3 + 1, 0, index * 3 + 3, 2)))
            .collect();
        second[7] = None;

        let mut swept = Vec::new();
        visit_swept_segment_pair_candidates(
            &first,
            &second,
            first.len(),
            second.len(),
            None,
            &policy,
            |first_index, second_index, _| {
                swept.push((first_index, second_index));
                Ok(())
            },
        )
        .expect("large exact boxes support the retained x sweep")
        .expect("candidate visitor succeeds");
        let mut index = SegmentAabbXIndex::try_new(&second, second.len(), &policy).unwrap();
        assert!(index.prepare_interval_queries(&second, &policy));
        let mut prepared = Vec::new();
        visit_swept_segment_pair_candidates(
            &first,
            &second,
            first.len(),
            second.len(),
            Some(&index),
            &policy,
            |first_index, second_index, _| {
                prepared.push((first_index, second_index));
                Ok(())
            },
        )
        .unwrap()
        .unwrap();
        let flat = flat_candidates(&first, &second, &policy);

        assert_eq!(swept, flat);
        assert_eq!(prepared, flat);
    }

    #[test]
    fn dense_ranked_candidates_match_flat_decided_aabb_filter() {
        let policy = CurvePolicy::certified();
        let first: Vec<_> = (0..64)
            .map(|index| Some(bbox(0, index * 3, 10, index * 3 + 2)))
            .collect();
        let mut second: Vec<_> = (0..64)
            .map(|index| Some(bbox(10, index * 3 + 2, 15, index * 3 + 4)))
            .collect();
        second[7] = None;

        let mut ranked = Vec::new();
        visit_swept_segment_pair_candidates(
            &first,
            &second,
            first.len(),
            second.len(),
            None,
            &policy,
            |first_index, second_index, aabb_overlap_certified| {
                ranked.push((first_index, second_index, aabb_overlap_certified));
                Ok(())
            },
        )
        .expect("dense exact boxes support the retained rank schedule")
        .expect("candidate visitor succeeds");
        assert_eq!(
            ranked
                .iter()
                .map(|&(first, second, _)| (first, second))
                .collect::<Vec<_>>(),
            flat_candidates(&first, &second, &policy),
        );
        assert!(
            ranked
                .iter()
                .all(|&(_, second, certified)| certified == (second != 7))
        );
    }

    #[test]
    fn preview_min_x_sort_recovers_from_rounded_ties() {
        let policy = CurvePolicy::certified();
        let one = Real::one();
        let epsilon = (Real::one() / Real::from(1_u128 << 100)).unwrap();
        let larger = &one + epsilon;
        assert_eq!(one.to_f64_lossy(), larger.to_f64_lossy());
        let boxes = [
            Some(Aabb2::new_unchecked(
                Point2::new(larger.clone(), Real::zero()),
                Point2::new(larger, Real::zero()),
            )),
            Some(Aabb2::new_unchecked(
                Point2::new(one.clone(), Real::zero()),
                Point2::new(one, Real::zero()),
            )),
        ];

        let index = SegmentAabbXIndex::try_new(&boxes, boxes.len(), &policy).unwrap();

        assert_eq!(index.ordered, [1, 0]);
    }
}
