//! Retained curve identity, domain, trim, and cache evidence.
//!
//! CAD curve imports need a way to carry source identity and parameter-domain
//! facts before a topology kernel is allowed to consume the curve.  The types
//! in this module are intentionally small evidence records: they do not sample
//! curves, and they do not imply native topology.  That follows Yap's exact
//! geometric computation model, where exact objects and predicates remain
//! replayable until a certified operation consumes them; see Yap, "Towards
//! Exact Geometric Computation," *Computational Geometry* 7(1-2), 3-23 (1997).

use hyperreal::Real;

use crate::classify::compare_reals;
use crate::{
    Classification, CurveError, CurvePolicy, CurveResult, Point2, RetainedTopologyStatus,
    UncertaintyReason,
};

/// Curve family carried by retained curve metadata.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RetainedCurveFamily2 {
    /// Polynomial B-spline carrier.
    PolynomialBSpline,
    /// Degree-two rational B-spline/NURBS carrier.
    RationalQuadraticBSpline,
    /// Degree-two-or-higher rational B-spline/NURBS carrier.
    RationalBSpline,
}

/// Stable source identity for a retained curve.
///
/// `source_index` is deliberately opaque to `hypercurve`: an importer can map
/// it to a STEP entity id, DXF handle table, or application-local source row
/// without changing the exact curve evidence.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RetainedCurveIdentity2 {
    family: RetainedCurveFamily2,
    source_index: u64,
}

/// Exact one-dimensional parameter domain.
#[derive(Clone, Debug, PartialEq)]
pub struct RetainedParameterDomain1 {
    start: Real,
    end: Real,
}

/// Direction of a trim interval relative to its parameter domain.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RetainedTrimDirection {
    /// Trim runs from low parameter to high parameter.
    Forward,
    /// Trim runs from high parameter to low parameter.
    Reversed,
}

/// Exact trim interval on a retained curve domain.
#[derive(Clone, Debug, PartialEq)]
pub struct RetainedTrimInterval1 {
    start: Real,
    end: Real,
    direction: RetainedTrimDirection,
}

/// Periodicity evidence for a retained curve.
#[derive(Clone, Debug, PartialEq)]
pub enum RetainedCurvePeriodicity1 {
    /// The carrier is non-periodic in the retained parameter domain.
    NonPeriodic,
    /// The carrier has an exact positive parameter period.
    Periodic { period: Box<Real> },
}

/// Exact endpoint evidence at the active retained domain boundaries.
#[derive(Clone, Debug, PartialEq)]
pub struct RetainedEndpointEvidence2 {
    start_parameter: Real,
    end_parameter: Real,
    start_point: Point2,
    end_point: Point2,
}

/// Prepared-cache shape summary for a retained curve.
///
/// This summary is not topology by itself. It is an audit trail for how much
/// exact construction evidence is available locally: controls, knots, Bezier
/// spans, and how many spans are native versus retained evidence.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RetainedCurveCacheSummary2 {
    control_count: usize,
    knot_count: usize,
    span_count: usize,
    native_span_count: usize,
    retained_span_count: usize,
}

/// Retained curve profile combining identity, domain, trim, endpoints, and cache facts.
#[derive(Clone, Debug, PartialEq)]
pub struct RetainedCurveProfile2 {
    identity: RetainedCurveIdentity2,
    domain: RetainedParameterDomain1,
    trim: RetainedTrimInterval1,
    periodicity: RetainedCurvePeriodicity1,
    topology_status: RetainedTopologyStatus,
    endpoints: RetainedEndpointEvidence2,
    cache_summary: RetainedCurveCacheSummary2,
}

impl RetainedCurveIdentity2 {
    /// Constructs a retained curve identity.
    pub const fn new(family: RetainedCurveFamily2, source_index: u64) -> Self {
        Self {
            family,
            source_index,
        }
    }

    /// Returns the retained curve family.
    pub const fn family(self) -> RetainedCurveFamily2 {
        self.family
    }

    /// Returns the opaque source index.
    pub const fn source_index(self) -> u64 {
        self.source_index
    }
}

impl RetainedParameterDomain1 {
    /// Constructs an exact ordered parameter domain.
    pub fn try_new(
        start: Real,
        end: Real,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        match compare_reals(&start, &end, policy) {
            Some(std::cmp::Ordering::Less) => Ok(Classification::Decided(Self { start, end })),
            Some(std::cmp::Ordering::Equal | std::cmp::Ordering::Greater) => {
                Err(CurveError::InvalidBezierRange)
            }
            None => Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }
    }

    /// Returns the low end of the domain.
    pub const fn start(&self) -> &Real {
        &self.start
    }

    /// Returns the high end of the domain.
    pub const fn end(&self) -> &Real {
        &self.end
    }

    /// Certifies whether a parameter lies inside the closed domain.
    pub fn contains(&self, parameter: &Real, policy: &CurvePolicy) -> Classification<bool> {
        let lower = compare_reals(&self.start, parameter, policy);
        let upper = compare_reals(parameter, &self.end, policy);
        match (lower, upper) {
            (
                Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal),
                Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal),
            ) => Classification::Decided(true),
            (Some(_), Some(_)) => Classification::Decided(false),
            _ => Classification::Uncertain(UncertaintyReason::Ordering),
        }
    }
}

impl RetainedTrimInterval1 {
    /// Constructs a nondegenerate trim interval whose endpoints lie in `domain`.
    pub fn try_new(
        start: Real,
        end: Real,
        domain: &RetainedParameterDomain1,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        for parameter in [&start, &end] {
            match domain.contains(parameter, policy) {
                Classification::Decided(true) => {}
                Classification::Decided(false) => return Err(CurveError::InvalidBezierRange),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        let direction = match compare_reals(&start, &end, policy) {
            Some(std::cmp::Ordering::Less) => RetainedTrimDirection::Forward,
            Some(std::cmp::Ordering::Greater) => RetainedTrimDirection::Reversed,
            Some(std::cmp::Ordering::Equal) => return Err(CurveError::InvalidBezierRange),
            None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        };
        Ok(Classification::Decided(Self {
            start,
            end,
            direction,
        }))
    }

    /// Returns the authored trim start.
    pub const fn start(&self) -> &Real {
        &self.start
    }

    /// Returns the authored trim end.
    pub const fn end(&self) -> &Real {
        &self.end
    }

    /// Returns the exact trim direction.
    pub const fn direction(&self) -> RetainedTrimDirection {
        self.direction
    }
}

impl RetainedCurvePeriodicity1 {
    /// Constructs exact positive-period evidence.
    pub fn periodic(period: Real, policy: &CurvePolicy) -> CurveResult<Classification<Self>> {
        match compare_reals(&Real::zero(), &period, policy) {
            Some(std::cmp::Ordering::Less) => Ok(Classification::Decided(Self::Periodic {
                period: Box::new(period),
            })),
            Some(_) => Err(CurveError::InvalidBezierRange),
            None => Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }
    }
}

impl RetainedEndpointEvidence2 {
    /// Constructs exact endpoint evidence.
    pub const fn new(
        start_parameter: Real,
        end_parameter: Real,
        start_point: Point2,
        end_point: Point2,
    ) -> Self {
        Self {
            start_parameter,
            end_parameter,
            start_point,
            end_point,
        }
    }

    /// Returns the start-domain parameter.
    pub const fn start_parameter(&self) -> &Real {
        &self.start_parameter
    }

    /// Returns the end-domain parameter.
    pub const fn end_parameter(&self) -> &Real {
        &self.end_parameter
    }

    /// Returns the retained start point.
    pub const fn start_point(&self) -> &Point2 {
        &self.start_point
    }

    /// Returns the retained end point.
    pub const fn end_point(&self) -> &Point2 {
        &self.end_point
    }
}

impl RetainedCurveCacheSummary2 {
    /// Constructs a retained cache summary.
    pub const fn new(
        control_count: usize,
        knot_count: usize,
        span_count: usize,
        native_span_count: usize,
        retained_span_count: usize,
    ) -> Self {
        Self {
            control_count,
            knot_count,
            span_count,
            native_span_count,
            retained_span_count,
        }
    }

    /// Returns the number of retained controls.
    pub const fn control_count(&self) -> usize {
        self.control_count
    }

    /// Returns the number of retained knots.
    pub const fn knot_count(&self) -> usize {
        self.knot_count
    }

    /// Returns the number of extracted Bezier spans.
    pub const fn span_count(&self) -> usize {
        self.span_count
    }

    /// Returns the number of spans with exact native topology.
    pub const fn native_span_count(&self) -> usize {
        self.native_span_count
    }

    /// Returns the number of spans retained without native topology.
    pub const fn retained_span_count(&self) -> usize {
        self.retained_span_count
    }
}

impl RetainedCurveProfile2 {
    /// Constructs a retained curve profile.
    pub const fn new(
        identity: RetainedCurveIdentity2,
        domain: RetainedParameterDomain1,
        trim: RetainedTrimInterval1,
        periodicity: RetainedCurvePeriodicity1,
        topology_status: RetainedTopologyStatus,
        endpoints: RetainedEndpointEvidence2,
        cache_summary: RetainedCurveCacheSummary2,
    ) -> Self {
        Self {
            identity,
            domain,
            trim,
            periodicity,
            topology_status,
            endpoints,
            cache_summary,
        }
    }

    /// Returns retained source identity.
    pub const fn identity(&self) -> RetainedCurveIdentity2 {
        self.identity
    }

    /// Returns the active parameter domain.
    pub const fn domain(&self) -> &RetainedParameterDomain1 {
        &self.domain
    }

    /// Returns the active trim interval.
    pub const fn trim(&self) -> &RetainedTrimInterval1 {
        &self.trim
    }

    /// Returns periodicity evidence.
    pub const fn periodicity(&self) -> &RetainedCurvePeriodicity1 {
        &self.periodicity
    }

    /// Returns the topology-readiness status for the whole retained curve.
    pub const fn topology_status(&self) -> RetainedTopologyStatus {
        self.topology_status
    }

    /// Returns endpoint evidence at the active domain boundaries.
    pub const fn endpoints(&self) -> &RetainedEndpointEvidence2 {
        &self.endpoints
    }

    /// Returns prepared-cache shape evidence.
    pub const fn cache_summary(&self) -> &RetainedCurveCacheSummary2 {
        &self.cache_summary
    }
}
