//! Retained curve identity, domain, trim, and cache evidence.
//!
//! CAD curve imports need a way to carry source identity and parameter-domain
//! facts before a topology kernel is allowed to consume the curve.  The types
//! in this module are intentionally small evidence records: they do not sample
//! curves, and they do not imply native topology.  That follows the exactness model's exact
//! geometric computation model, where exact objects and predicates remain
//! replayable until a certified operation consumes them.

use hyperreal::Real;

use crate::classify::compare_reals;
use crate::{
    Classification, CurveError, CurvePolicy, CurveResult, Point2, RetainedTopologyStatus,
    SplinePeriodicity2, UncertaintyReason,
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
    source_version: u64,
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
    source_version: u64,
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
    periodicity: SplinePeriodicity2,
    topology_status: RetainedTopologyStatus,
    endpoints: RetainedEndpointEvidence2,
    cache_summary: RetainedCurveCacheSummary2,
}

impl RetainedCurveIdentity2 {
    /// Constructs a retained curve identity.
    pub const fn new(family: RetainedCurveFamily2, source_index: u64) -> Self {
        Self::new_with_source_version(family, source_index, 0)
    }

    /// Constructs a retained curve identity with source version/revision evidence.
    pub const fn new_with_source_version(
        family: RetainedCurveFamily2,
        source_index: u64,
        source_version: u64,
    ) -> Self {
        Self {
            family,
            source_index,
            source_version,
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

    /// Returns the retained source version/revision.
    pub const fn source_version(self) -> u64 {
        self.source_version
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

impl RetainedEndpointEvidence2 {
    /// Constructs exact endpoint evidence for a certified retained domain.
    pub fn new(domain: &RetainedParameterDomain1, start_point: Point2, end_point: Point2) -> Self {
        Self {
            start_parameter: domain.start().clone(),
            end_parameter: domain.end().clone(),
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
    pub fn new(
        control_count: usize,
        knot_count: usize,
        span_count: usize,
        native_span_count: usize,
        retained_span_count: usize,
    ) -> CurveResult<Self> {
        Self::new_with_source_version(
            0,
            control_count,
            knot_count,
            span_count,
            native_span_count,
            retained_span_count,
        )
    }

    /// Constructs a retained cache summary stamped with the source version it replayed.
    pub fn new_with_source_version(
        source_version: u64,
        control_count: usize,
        knot_count: usize,
        span_count: usize,
        native_span_count: usize,
        retained_span_count: usize,
    ) -> CurveResult<Self> {
        validate_cache_summary_counts(
            control_count,
            knot_count,
            span_count,
            native_span_count,
            retained_span_count,
        )?;
        Ok(Self {
            source_version,
            control_count,
            knot_count,
            span_count,
            native_span_count,
            retained_span_count,
        })
    }

    /// Returns the source version used to build this cache evidence.
    pub const fn source_version(&self) -> u64 {
        self.source_version
    }

    /// Returns true when the cache was replayed against the retained identity version.
    pub const fn is_fresh_for(&self, identity: RetainedCurveIdentity2) -> bool {
        self.source_version == identity.source_version()
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
    pub fn new(
        identity: RetainedCurveIdentity2,
        domain: RetainedParameterDomain1,
        trim: RetainedTrimInterval1,
        periodicity: SplinePeriodicity2,
        topology_status: RetainedTopologyStatus,
        endpoints: RetainedEndpointEvidence2,
        cache_summary: RetainedCurveCacheSummary2,
    ) -> CurveResult<Self> {
        validate_curve_profile_evidence(
            identity,
            &domain,
            &trim,
            &periodicity,
            topology_status,
            &endpoints,
            &cache_summary,
        )?;
        Ok(Self {
            identity,
            domain,
            trim,
            periodicity,
            topology_status,
            endpoints,
            cache_summary,
        })
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
    pub const fn periodicity(&self) -> &SplinePeriodicity2 {
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

fn validate_cache_summary_counts(
    control_count: usize,
    knot_count: usize,
    span_count: usize,
    native_span_count: usize,
    retained_span_count: usize,
) -> CurveResult<()> {
    if control_count == 0 || knot_count == 0 || span_count == 0 {
        return Err(CurveError::Topology(
            "retained curve cache summary must carry nonempty controls, knots, and spans".into(),
        ));
    }
    if knot_count <= control_count {
        return Err(CurveError::Topology(
            "retained B-spline cache summary must carry more knots than controls".into(),
        ));
    }
    if span_count
        .checked_add(2)
        .is_none_or(|minimum_control_count| control_count < minimum_control_count)
    {
        return Err(CurveError::Topology(
            "retained B-spline cache summary must carry at least two more controls than spans"
                .into(),
        ));
    }
    let Some(order) = knot_count.checked_sub(control_count) else {
        return Err(CurveError::Topology(
            "retained B-spline cache summary knot/control counts are inconsistent".into(),
        ));
    };
    if order < 3 || control_count < order {
        return Err(CurveError::Topology(
            "retained B-spline cache summary must carry a supported degree shape".into(),
        ));
    }
    let degree = order - 1;
    if span_count > control_count - degree {
        return Err(CurveError::Topology(
            "retained B-spline cache summary span count exceeds the degree-implied maximum".into(),
        ));
    }
    if native_span_count
        .checked_add(retained_span_count)
        .is_none_or(|count| count != span_count)
    {
        return Err(CurveError::Topology(
            "retained curve cache summary span decomposition does not match span count".into(),
        ));
    }
    Ok(())
}

fn validate_curve_profile_evidence(
    identity: RetainedCurveIdentity2,
    domain: &RetainedParameterDomain1,
    trim: &RetainedTrimInterval1,
    periodicity: &SplinePeriodicity2,
    topology_status: RetainedTopologyStatus,
    endpoints: &RetainedEndpointEvidence2,
    cache_summary: &RetainedCurveCacheSummary2,
) -> CurveResult<()> {
    validate_profile_family_shape(identity, cache_summary)?;
    if !cache_summary.is_fresh_for(identity) {
        return Err(CurveError::Topology(
            "retained curve profile cache summary source version is stale".into(),
        ));
    }
    let policy = CurvePolicy::certified();
    for parameter in [trim.start(), trim.end()] {
        if domain.contains(parameter, &policy) != Classification::Decided(true) {
            return Err(CurveError::Topology(
                "retained curve trim evidence must lie inside the active parameter domain".into(),
            ));
        }
    }
    if endpoints.start_parameter() != domain.start() || endpoints.end_parameter() != domain.end() {
        return Err(CurveError::Topology(
            "retained curve endpoint evidence must match the active parameter domain".into(),
        ));
    }
    validate_profile_periodicity(periodicity, domain, endpoints)?;
    if topology_status.is_native_exact() && cache_summary.retained_span_count() != 0 {
        return Err(CurveError::Topology(
            "native retained curve profile must not report retained unsupported spans".into(),
        ));
    }
    if !topology_status.is_native_exact() && cache_summary.retained_span_count() == 0 {
        return Err(CurveError::Topology(
            "non-native retained curve profile must report retained evidence spans".into(),
        ));
    }
    if !topology_status.is_native_exact() && !topology_status.is_retained_evidence() {
        return Err(CurveError::Topology(
            "retained curve profile must carry exact native or retained evidence status".into(),
        ));
    }
    Ok(())
}

fn validate_profile_periodicity(
    periodicity: &SplinePeriodicity2,
    domain: &RetainedParameterDomain1,
    endpoints: &RetainedEndpointEvidence2,
) -> CurveResult<()> {
    let SplinePeriodicity2::Periodic { period } = periodicity else {
        return Ok(());
    };
    let policy = CurvePolicy::certified();
    if compare_reals(&Real::zero(), period, &policy) != Some(std::cmp::Ordering::Less) {
        return Err(CurveError::Topology(
            "retained periodic curve must carry an exact positive period".into(),
        ));
    }
    let domain_width = domain.end() - domain.start();
    if compare_reals(&domain_width, period, &policy) != Some(std::cmp::Ordering::Equal) {
        return Err(CurveError::Topology(
            "retained periodic curve period must equal its active parameter-domain width".into(),
        ));
    }
    if compare_reals(
        endpoints.start_point().x(),
        endpoints.end_point().x(),
        &policy,
    ) != Some(std::cmp::Ordering::Equal)
        || compare_reals(
            endpoints.start_point().y(),
            endpoints.end_point().y(),
            &policy,
        ) != Some(std::cmp::Ordering::Equal)
    {
        return Err(CurveError::Topology(
            "retained periodic curve endpoint evidence must close at the canonical seam".into(),
        ));
    }
    Ok(())
}

fn validate_profile_family_shape(
    identity: RetainedCurveIdentity2,
    cache_summary: &RetainedCurveCacheSummary2,
) -> CurveResult<()> {
    let degree = cache_summary
        .knot_count()
        .checked_sub(cache_summary.control_count())
        .and_then(|order| order.checked_sub(1))
        .ok_or_else(|| {
            CurveError::Topology(
                "retained curve profile cache summary has no certified B-spline degree".into(),
            )
        })?;
    match identity.family() {
        RetainedCurveFamily2::PolynomialBSpline if !(1..=3).contains(&degree) => {
            Err(CurveError::Topology(
                "polynomial B-spline profile must carry linear, quadratic, or cubic cache evidence"
                    .into(),
            ))
        }
        RetainedCurveFamily2::RationalQuadraticBSpline if degree != 2 => Err(CurveError::Topology(
            "rational quadratic B-spline profile must carry quadratic cache evidence".into(),
        )),
        _ => Ok(()),
    }
}
