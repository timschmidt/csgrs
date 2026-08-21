//! Finite polyline projection adapters for native hyper curves.
//!
//! Projection is an IO/rendering boundary, not a topology kernel. The methods
//! in this module preserve line segments exactly, approximate curved carriers
//! by a chord-error budget, and return primitive `f64` coordinates only after
//! the source [`Real`](hyperreal::Real) coordinates can be exported finitely.
//! This follows exact-computation discipline:
//! exact objects own CAD/topology; finite samples are boundary products.
//! Boundary and containment decisions should continue to use the exact
//! contour/region APIs surveyed by boundary-first winding classification.

use std::f64::consts::PI;

use crate::bezier_offset::algebraic_chord_endpoint_bounds_refined;
use crate::bezier_parameter::BezierParameterRefinement2;
use crate::bezier_split::BezierSelectedFiberSource2;
use crate::{
    BezierParallel2, BezierParallelSource2, BezierParameter2, BezierSplitFragment2,
    BezierSubcurve2, CircularArc2, Classification, Contour2, Curve2, CurveContext, CurveError,
    CurveOutcome, CurvePath2, CurveRegion2, CurveRegionBoundaryLoop2, CurveRegionLoopRole,
    CurveRegionParameter2, CurveResult, CurveString2, Point2,
    RationalBezierIntersectionPointEvidence2, Segment2,
};
use hyperreal::{Real, RealSign};

/// Options for projecting native curves to finite `f64` polylines.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FiniteProjectionOptions {
    chord_error: f64,
}

/// Finite `f64` polyline emitted from a native curve object.
#[derive(Clone, Debug, PartialEq)]
pub struct FinitePolyline2 {
    points: Vec<[f64; 2]>,
    chord_error: f64,
    closed: bool,
}

/// A finite material ring and the finite hole rings owned by it.
///
/// Ownership is decided in authoritative [`CurveRegion2`] topology before any finite ring is
/// emitted; this type only carries the boundary result.
#[derive(Clone, Debug, PartialEq)]
pub struct FiniteRegionProfile2 {
    material: FinitePolyline2,
    holes: Vec<FinitePolyline2>,
}

impl FiniteProjectionOptions {
    /// Constructs projection options with a positive finite chord-error budget.
    pub fn try_new(chord_error: f64) -> CurveResult<Self> {
        if chord_error.is_finite() && chord_error > 0.0 {
            Ok(Self { chord_error })
        } else {
            Err(CurveError::InvalidFiniteProjectionOptions)
        }
    }

    /// Returns the maximum requested chord error.
    pub const fn chord_error(&self) -> f64 {
        self.chord_error
    }
}

impl FinitePolyline2 {
    fn new(points: Vec<[f64; 2]>, chord_error: f64, closed: bool) -> Self {
        Self {
            points,
            chord_error,
            closed,
        }
    }

    /// Returns the finite projected vertices.
    pub fn points(&self) -> &[[f64; 2]] {
        &self.points
    }

    /// Consumes the projection and returns finite vertices.
    pub fn into_points(self) -> Vec<[f64; 2]> {
        self.points
    }

    /// Returns the chord-error budget requested for this projection.
    pub const fn chord_error(&self) -> f64 {
        self.chord_error
    }

    /// Returns true when this polyline was explicitly closed for a contour.
    pub const fn is_closed(&self) -> bool {
        self.closed
    }

    /// Returns the finite signed shoelace area when this polyline is treated as
    /// a ring.
    ///
    /// This is only a boundary/product measurement of projected vertices. Exact
    /// contour area stays on [`Contour2::signed_area`] and
    /// [`CurveRegion2::filled_area`].
    pub fn signed_ring_area(&self) -> f64 {
        finite_ring_signed_area(&self.points)
    }

    /// Returns the checked finite signed shoelace area of this projected ring.
    pub fn try_signed_ring_area(&self) -> CurveResult<f64> {
        try_finite_ring_signed_area(&self.points)
    }

    /// Returns the arithmetic centroid of this finite projected polyline.
    ///
    /// This is a boundary-product measurement over emitted finite vertices, not
    /// an exact centroid of the native curve or filled area. A repeated closing
    /// vertex is ignored. Keeping this helper on the projected polyline type
    /// prevents downstream crates from reimplementing small finite adapters
    /// around hypercurve output. The exact-object/boundary split follows exact-computation discipline.
    pub fn vertex_centroid(&self) -> Option<[f64; 2]> {
        finite_polyline_vertex_centroid(&self.points)
    }

    /// Returns the checked arithmetic centroid of this projected polyline.
    pub fn try_vertex_centroid(&self) -> CurveResult<Option<[f64; 2]>> {
        try_finite_polyline_vertex_centroid(&self.points)
    }
}

impl FiniteRegionProfile2 {
    fn new(material: FinitePolyline2, holes: Vec<FinitePolyline2>) -> Self {
        Self { material, holes }
    }

    /// Returns the projected material ring.
    pub const fn material(&self) -> &FinitePolyline2 {
        &self.material
    }

    /// Returns the projected hole rings owned by the material ring.
    pub fn holes(&self) -> &[FinitePolyline2] {
        &self.holes
    }

    /// Returns the finite projected material-minus-hole area.
    ///
    /// Hole ownership has already been decided by native region topology before
    /// this projected profile exists, so this method does not infer roles from
    /// winding. It only measures the finite output rings with the shoelace
    /// formula. Exact CAD area should use [`CurveRegion2::filled_area`]; this helper
    /// exists for IO, diagnostics, and tests at the projection boundary.
    pub fn projected_filled_area(&self) -> f64 {
        let material = self.material.signed_ring_area().abs();
        let holes = self
            .holes
            .iter()
            .map(|hole| hole.signed_ring_area().abs())
            .sum::<f64>();
        material - holes
    }

    /// Returns the checked finite projected material-minus-hole area.
    pub fn try_projected_filled_area(&self) -> CurveResult<f64> {
        let material = self.material.try_signed_ring_area()?.abs();
        let holes = self.holes.iter().try_fold(0.0, |sum, hole| {
            let next = sum + hole.try_signed_ring_area()?.abs();
            next.is_finite()
                .then_some(next)
                .ok_or(CurveError::NonFiniteProjectionPoint)
        })?;
        let area = material - holes;
        area.is_finite()
            .then_some(area)
            .ok_or(CurveError::NonFiniteProjectionPoint)
    }
}

/// Returns the finite signed shoelace area of projected ring vertices.
///
/// The closing edge is included even when the caller did not repeat the first
/// vertex. This is the familiar Green's-theorem polygon formula applied only
/// to finite boundary data; exact CAD area should use native contour/region
/// area APIs instead. The boundary split follows exact-computation discipline.
pub fn finite_ring_signed_area(ring: &[[f64; 2]]) -> f64 {
    if ring.len() < 3 {
        return 0.0;
    }
    let mut area = 0.0;
    for edge in ring.windows(2) {
        area += edge[0][0] * edge[1][1] - edge[1][0] * edge[0][1];
    }
    if let (Some(first), Some(last)) = (ring.first(), ring.last()) {
        area += last[0] * first[1] - first[0] * last[1];
    }
    0.5 * area
}

/// Returns the checked finite signed shoelace area of projected ring vertices.
///
/// Non-finite coordinates or arithmetic overflow are reported explicitly
/// instead of leaking `NaN`/`inf` through a finite-boundary measurement.
pub fn try_finite_ring_signed_area(ring: &[[f64; 2]]) -> CurveResult<f64> {
    let ring = normalize_finite_ring_vertices(ring)?;
    if ring.len() < 3 {
        return Ok(0.0);
    }

    let mut area = 0.0;
    for edge in ring.windows(2) {
        area = checked_shoelace_sum(area, edge[0], edge[1])?;
    }
    if let (Some(first), Some(last)) = (ring.first(), ring.last()) {
        area = checked_shoelace_sum(area, *last, *first)?;
    }
    let area = 0.5 * area;
    area.is_finite()
        .then_some(area)
        .ok_or(CurveError::NonFiniteProjectionPoint)
}

/// Returns the arithmetic centroid of finite polyline vertices.
///
/// A repeated final closing point is ignored so closed-ring projections do not
/// overweight the first vertex. This is a finite boundary statistic only; exact
/// geometric centroids belong on native curve/region facts.
pub fn finite_polyline_vertex_centroid(points: &[[f64; 2]]) -> Option<[f64; 2]> {
    let unique = points
        .iter()
        .copied()
        .enumerate()
        .filter_map(|(index, point)| {
            (index + 1 != points.len() || Some(&point) != points.first()).then_some(point)
        })
        .collect::<Vec<_>>();
    if unique.is_empty() {
        return None;
    }
    let count = unique.len() as f64;
    let (sum_x, sum_y) = unique
        .iter()
        .fold((0.0, 0.0), |(x, y), point| (x + point[0], y + point[1]));
    Some([sum_x / count, sum_y / count])
}

/// Returns the checked arithmetic centroid of finite polyline vertices.
///
/// A repeated final closing point is ignored. Non-finite coordinates or
/// arithmetic overflow are reported explicitly.
pub fn try_finite_polyline_vertex_centroid(points: &[[f64; 2]]) -> CurveResult<Option<[f64; 2]>> {
    let unique = finite_unique_polyline_vertices(points)?;
    if unique.is_empty() {
        return Ok(None);
    }
    let count = unique.len() as f64;
    let (sum_x, sum_y) = unique.iter().try_fold((0.0, 0.0), |(x, y), point| {
        let next_x = x + point[0];
        let next_y = y + point[1];
        (next_x.is_finite() && next_y.is_finite())
            .then_some((next_x, next_y))
            .ok_or(CurveError::NonFiniteProjectionPoint)
    })?;
    let centroid = [sum_x / count, sum_y / count];
    centroid
        .iter()
        .all(|value| value.is_finite())
        .then_some(Some(centroid))
        .ok_or(CurveError::NonFiniteProjectionPoint)
}

pub(crate) fn normalize_finite_ring_vertices(ring: &[[f64; 2]]) -> CurveResult<Vec<[f64; 2]>> {
    let mut normalized = Vec::with_capacity(ring.len());
    for point in finite_unique_polyline_vertices(ring)? {
        if normalized.last() == Some(&point) {
            continue;
        }
        normalized.push(point);
    }
    if normalized.len() > 1 && normalized.first() == normalized.last() {
        normalized.pop();
    }
    Ok(normalized)
}

fn finite_unique_polyline_vertices(points: &[[f64; 2]]) -> CurveResult<Vec<[f64; 2]>> {
    let mut unique = Vec::with_capacity(points.len());
    for (index, &[x, y]) in points.iter().enumerate() {
        if !x.is_finite() || !y.is_finite() {
            return Err(CurveError::NonFiniteProjectionPoint);
        }
        let point = [x, y];
        if index + 1 != points.len() || Some(&point) != points.first() {
            unique.push(point);
        }
    }
    Ok(unique)
}

fn checked_shoelace_sum(sum: f64, start: [f64; 2], end: [f64; 2]) -> CurveResult<f64> {
    let term = start[0] * end[1] - end[0] * start[1];
    let next = sum + term;
    next.is_finite()
        .then_some(next)
        .ok_or(CurveError::NonFiniteProjectionPoint)
}

impl CurveString2 {
    /// Projects this curve string to a finite polyline for IO and display.
    ///
    /// This is a lossy boundary view: circular arcs are sampled by chord error,
    /// and the returned `f64` vertices must not be used as the source of exact
    /// topology decisions.
    pub fn project_to_finite_polyline(
        &self,
        options: &FiniteProjectionOptions,
    ) -> CurveResult<FinitePolyline2> {
        project_curve_string(self, options, false)
    }
}

impl CurvePath2 {
    /// Projects a full higher-order path to a finite polyline.
    ///
    /// Authored curve families remain authoritative. Polynomial and rational
    /// Bezier spans are subdivided in their native representation and only the
    /// resulting boundary product is converted to `f64`.
    #[inline]
    pub fn project_to_finite_polyline(
        &self,
        options: &FiniteProjectionOptions,
        policy: &CurveContext,
    ) -> CurveResult<CurveOutcome<FinitePolyline2>> {
        crate::policy::resolve_certified_operation(policy, |attempt| {
            self.project_to_finite_polyline_raw(options, attempt)
        })
    }

    fn project_to_finite_polyline_raw(
        &self,
        options: &FiniteProjectionOptions,
        policy: &CurveContext,
    ) -> CurveResult<FinitePolyline2> {
        let closed = if self.start() == self.end() {
            true
        } else {
            crate::classify::is_zero(&self.start().distance_squared(self.end()), policy)
                .ok_or_else(|| {
                    CurveError::Topology(
                        "finite path projection could not decide endpoint closure".into(),
                    )
                })?
        };
        let fragments = match self
            .native_bezier_fragments_with_policy(policy)
            .map_err(|error| CurveError::Topology(error.to_string()))?
        {
            Classification::Decided(fragments) => fragments,
            Classification::Uncertain(reason) => {
                return Err(CurveError::Topology(format!(
                    "finite path projection was blocked by {reason:?}"
                )));
            }
        };
        let mut points = Vec::with_capacity(fragments.len() + 1);
        for fragment in fragments {
            append_bezier_subcurve_samples(&mut points, fragment.curve(), options, policy, 0)?;
        }
        if closed {
            close_ring(&mut points);
        }
        Ok(FinitePolyline2::new(points, options.chord_error, closed))
    }
}

impl CurveRegion2 {
    /// Projects retained algebraic split parameters to finite rational
    /// representatives while preserving each boundary curve family.
    ///
    /// This is a display/export boundary: Boolean topology remains owned by
    /// this exact region, while the returned paths retain lines and native
    /// Bezier curves instead of segmenting them into chords.
    #[inline]
    pub fn project_to_finite_curve_paths(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<CurveOutcome<Classification<Vec<CurvePath2>>>> {
        crate::policy::resolve_certified_operation(policy, |attempt| {
            self.project_to_finite_curve_paths_raw(attempt)
        })
    }

    #[inline]
    fn project_to_finite_curve_paths_raw(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<CurvePath2>>> {
        let mut paths = Vec::with_capacity(self.boundary_loops().len());
        for boundary in self.boundary_loops() {
            let Some(path) = project_curve_region_loop_to_curve_path(boundary, policy)? else {
                return Ok(Classification::Uncertain(
                    crate::UncertaintyReason::Unsupported,
                ));
            };
            paths.push(path);
        }
        Ok(Classification::Decided(paths))
    }

    /// Projects retained higher-order region loops into material profiles.
    ///
    /// Representable loop roles and hole ownership are decided by exact curved
    /// topology before any coordinate crosses the finite boundary. Retained
    /// algebraic endpoints that cannot be represented as [`Point2`] keep their
    /// certified filled sides, then derive export-only roles and ownership from
    /// projected ring orientation and containment. That fallback never feeds
    /// exact predicates.
    #[inline]
    pub fn project_to_finite_profiles(
        &self,
        options: &FiniteProjectionOptions,
        policy: &CurveContext,
    ) -> CurveResult<CurveOutcome<Classification<Vec<FiniteRegionProfile2>>>> {
        crate::policy::resolve_certified_operation(policy, |attempt| {
            self.project_to_finite_profiles_raw(options, attempt)
        })
    }

    pub(crate) fn project_to_finite_profiles_raw(
        &self,
        options: &FiniteProjectionOptions,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<FiniteRegionProfile2>>> {
        if self.is_empty() {
            return Ok(Classification::Decided(Vec::new()));
        }
        let rings = self
            .boundary_loops()
            .iter()
            .map(|boundary| project_curve_region_loop(boundary, options, policy))
            .collect::<CurveResult<Vec<_>>>()?;
        if let Classification::Decided(exact_profiles) = self.boundary_profiles_raw(policy)? {
            return Ok(Classification::Decided(
                exact_profiles
                    .into_iter()
                    .map(|profile| {
                        let material = rings[profile.material_loop_index()].clone();
                        let holes = profile
                            .hole_loop_indices()
                            .iter()
                            .map(|index| rings[*index].clone())
                            .collect();
                        FiniteRegionProfile2::new(material, holes)
                    })
                    .collect(),
            ));
        }

        // Retained algebraic loops may have a certified filled side while
        // exact role/ownership sampling is not representable as Point2. Keep
        // that limitation at this finite boundary rather than blocking display
        // and meshing of otherwise decided Boolean output.
        let filled_sides = match self.filled_side_is_left_raw(policy)? {
            Classification::Decided(sides) => sides,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let roles = rings
            .iter()
            .zip(filled_sides)
            .map(|(ring, filled_side_is_left)| {
                let interior_is_left = finite_ring_signed_area(ring.points()) > 0.0;
                if interior_is_left == *filled_side_is_left {
                    CurveRegionLoopRole::Material
                } else {
                    CurveRegionLoopRole::Hole
                }
            })
            .collect::<Vec<_>>();
        let material_indices = roles
            .iter()
            .enumerate()
            .filter_map(|(index, role)| (*role == CurveRegionLoopRole::Material).then_some(index))
            .collect::<Vec<_>>();
        let mut profiles = material_indices
            .iter()
            .map(|index| FiniteRegionProfile2::new(rings[*index].clone(), Vec::new()))
            .collect::<Vec<_>>();
        for (hole_index, role) in roles.iter().enumerate() {
            if *role != CurveRegionLoopRole::Hole {
                continue;
            }
            let Some(sample) = finite_polyline_vertex_centroid(rings[hole_index].points()) else {
                return Err(CurveError::NonFiniteProjectionPoint);
            };
            let owner = material_indices
                .iter()
                .enumerate()
                .filter(|(_, material_index)| {
                    finite_ring_contains_point(rings[**material_index].points(), sample)
                })
                .min_by(|(_, left), (_, right)| {
                    finite_ring_signed_area(rings[**left].points())
                        .abs()
                        .total_cmp(&finite_ring_signed_area(rings[**right].points()).abs())
                })
                .map(|(profile_index, _)| profile_index)
                .ok_or_else(|| {
                    CurveError::Topology(
                        "projected curved-region hole has no material owner".into(),
                    )
                })?;
            profiles[owner].holes.push(rings[hole_index].clone());
        }
        Ok(Classification::Decided(profiles))
    }

    /// Projects this region only when exact loop roles and ownership are
    /// decided before the finite boundary is crossed.
    ///
    /// Unlike [`Self::project_to_finite_profiles`], this mesh/topology-facing
    /// variant does not infer export-only roles from finite winding or
    /// containment when algebraic endpoints cannot inhabit [`Point2`].
    #[inline]
    pub fn project_to_finite_profiles_exact(
        &self,
        options: &FiniteProjectionOptions,
        policy: &CurveContext,
    ) -> CurveResult<CurveOutcome<Classification<Vec<FiniteRegionProfile2>>>> {
        crate::policy::resolve_certified_operation(policy, |attempt| {
            self.project_to_finite_profiles_exact_raw(options, attempt)
        })
    }

    pub(crate) fn project_to_finite_profiles_exact_raw(
        &self,
        options: &FiniteProjectionOptions,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<FiniteRegionProfile2>>> {
        if self.is_empty() {
            return Ok(Classification::Decided(Vec::new()));
        }
        let rings = self
            .boundary_loops()
            .iter()
            .map(|boundary| project_curve_region_loop(boundary, options, policy))
            .collect::<CurveResult<Vec<_>>>()?;
        match self.boundary_profiles_raw(policy)? {
            Classification::Decided(exact_profiles) => Ok(Classification::Decided(
                exact_profiles
                    .into_iter()
                    .map(|profile| {
                        let material = rings[profile.material_loop_index()].clone();
                        let holes = profile
                            .hole_loop_indices()
                            .iter()
                            .map(|index| rings[*index].clone())
                            .collect();
                        FiniteRegionProfile2::new(material, holes)
                    })
                    .collect(),
            )),
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }
}

fn project_curve_region_loop_to_curve_path(
    boundary: &CurveRegionBoundaryLoop2,
    policy: &CurveContext,
) -> CurveResult<Option<CurvePath2>> {
    let finish = |curves| {
        CurvePath2::try_new(curves)
            .map(Some)
            .map_err(|error| match error {
                crate::ExactCurveError::Invalid { cause, .. } => cause,
                crate::ExactCurveError::Blocked(blocker) => CurveError::Topology(format!(
                    "finite curve projection was blocked by {:?}",
                    blocker.reason()
                )),
            })
    };
    let fragments = boundary.fragments();
    let fully_materialized = !fragments.is_empty()
        && fragments
            .iter()
            .all(|fragment| matches!(fragment, BezierSplitFragment2::Materialized { .. }));
    let structurally_connected = fully_materialized
        && fragments
            .iter()
            .zip(fragments.iter().cycle().skip(1))
            .all(|(left, right)| match (left, right) {
                (
                    BezierSplitFragment2::Materialized {
                        curve: left_curve, ..
                    },
                    BezierSplitFragment2::Materialized {
                        curve: right_curve, ..
                    },
                ) => {
                    projection_points_are_structurally_equal(left_curve.end(), right_curve.start())
                }
                _ => unreachable!("the materialized fast path checked every fragment"),
            });
    if structurally_connected {
        return Ok(Some(CurvePath2::from_structurally_closed_curves(
            fragments
                .iter()
                .map(|fragment| match fragment {
                    BezierSplitFragment2::Materialized { curve, .. } => Curve2::from(curve.clone()),
                    _ => unreachable!("the materialized fast path checked every fragment"),
                })
                .collect(),
        )));
    }
    if fully_materialized {
        for (left, right) in fragments.iter().zip(fragments.iter().cycle().skip(1)) {
            let (
                BezierSplitFragment2::Materialized {
                    curve: left_curve, ..
                },
                BezierSplitFragment2::Materialized {
                    curve: right_curve, ..
                },
            ) = (left, right)
            else {
                unreachable!("the materialized connectivity path checked every fragment");
            };
            match crate::classify::is_zero(
                &left_curve.end().distance_squared(right_curve.start()),
                policy,
            ) {
                Some(true) => {}
                Some(false) => {
                    return Err(CurveError::Topology(
                        "materialized finite curve projection contains a disconnected join".into(),
                    ));
                }
                None => return Ok(None),
            }
        }
    }

    let mut subcurves = Vec::with_capacity(fragments.len());
    for fragment in fragments {
        let curve = match fragment {
            BezierSplitFragment2::Materialized { curve, .. } => curve.clone(),
            BezierSplitFragment2::AlgebraicEndpointImages {
                reversed,
                start,
                end,
                source_curve: source,
                ..
            } => {
                let (start, end) = finite_parameter_pair(start, end, policy)?;
                let curve = match source.subcurve_between_exact(&start, &end, policy)? {
                    Classification::Decided(curve) => curve,
                    Classification::Uncertain(_) => return Ok(None),
                };
                if *reversed { curve.reversed() } else { curve }
            }
            BezierSplitFragment2::AnalyticParallel(_)
            | BezierSplitFragment2::AlgebraicChord(_)
            | BezierSplitFragment2::AlgebraicCuspSemicircle(_) => return Ok(None),
            BezierSplitFragment2::SelectedFiber(_) => return Ok(None),
        };
        subcurves.push(curve);
    }
    if subcurves.is_empty() {
        return Ok(None);
    }
    let endpoints = subcurves
        .iter()
        .map(|curve| curve.end().clone())
        .collect::<Vec<_>>();
    let curve_count = subcurves.len();
    let curves = subcurves
        .into_iter()
        .enumerate()
        .map(|(index, curve)| {
            projected_subcurve_with_endpoints(
                curve,
                endpoints[(index + curve_count - 1) % curve_count].clone(),
                endpoints[index].clone(),
            )
            .map(Curve2::from)
        })
        .collect::<CurveResult<Vec<_>>>()?;
    finish(curves)
}

fn projection_points_are_structurally_equal(left: &Point2, right: &Point2) -> bool {
    if left.shares_storage(right) {
        return true;
    }
    if let (Some(left_x), Some(left_y), Some(right_x), Some(right_y)) = (
        left.x().exact_rational_ref(),
        left.y().exact_rational_ref(),
        right.x().exact_rational_ref(),
        right.y().exact_rational_ref(),
    ) {
        return left_x == right_x && left_y == right_y;
    }
    left == right
}

fn finite_parameter_pair(
    start: &BezierParameter2,
    end: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<(Real, Real)> {
    let mut start_refinement = BezierParameterRefinement2::new(start, policy);
    let mut end_refinement = BezierParameterRefinement2::new(end, policy);
    for refinement_steps in [0, 2, 4, 8, 16, 32, 64] {
        let start = start_refinement.refine_to(refinement_steps);
        let end = end_refinement.refine_to(refinement_steps);
        let start = finite_parameter_representative(start, policy)?;
        let end = finite_parameter_representative(end, policy)?;
        if matches!(
            crate::classify::compare_reals(&start, &end, policy),
            Some(std::cmp::Ordering::Less)
        ) {
            return Ok((start, end));
        }
    }
    Err(CurveError::InvalidBezierRange)
}

fn projected_subcurve_with_endpoints(
    curve: BezierSubcurve2,
    start: Point2,
    end: Point2,
) -> CurveResult<BezierSubcurve2> {
    Ok(match curve {
        BezierSubcurve2::Quadratic(curve) => BezierSubcurve2::Quadratic(
            crate::QuadraticBezier2::new(start, curve.control().clone(), end),
        ),
        BezierSubcurve2::Cubic(curve) => BezierSubcurve2::Cubic(crate::CubicBezier2::new(
            start,
            curve.control1().clone(),
            curve.control2().clone(),
            end,
        )),
        BezierSubcurve2::RationalQuadratic(curve) => {
            BezierSubcurve2::RationalQuadratic(crate::RationalQuadraticBezier2::try_new(
                start,
                curve.control().clone(),
                end,
                curve.start_weight().clone(),
                curve.control_weight().clone(),
                curve.end_weight().clone(),
            )?)
        }
        BezierSubcurve2::Rational(curve) => {
            let mut controls = curve.control_points().to_vec();
            controls[0] = start;
            *controls
                .last_mut()
                .expect("rational Bezier controls are nonempty") = end;
            BezierSubcurve2::Rational(crate::RationalBezier2::try_new(
                controls,
                curve.weights().to_vec(),
            )?)
        }
    })
}

impl Contour2 {
    /// Projects this closed contour to a finite closed ring for IO and display.
    ///
    /// The contour itself remains authoritative for area, containment, and
    /// winding. This method only emits a finite boundary ring after all points
    /// can cross the API boundary as `f64`.
    pub fn project_to_finite_ring(
        &self,
        options: &FiniteProjectionOptions,
    ) -> CurveResult<FinitePolyline2> {
        project_curve_string(self.curve_string(), options, true)
    }
}

fn project_curve_string(
    curve: &CurveString2,
    options: &FiniteProjectionOptions,
    close: bool,
) -> CurveResult<FinitePolyline2> {
    let first = curve.start().ok_or(CurveError::EmptyCurveString)?;
    let mut points = Vec::with_capacity(curve.len() + 1);
    push_if_new(&mut points, finite_point(first)?);

    for segment in curve.segments() {
        match segment {
            Segment2::Line(line) => {
                push_if_new(&mut points, finite_point(line.end())?);
            }
            Segment2::Arc(arc) => {
                append_arc_samples(&mut points, arc, options.chord_error)?;
            }
        }
    }

    if close {
        close_ring(&mut points);
    }

    Ok(FinitePolyline2::new(points, options.chord_error, close))
}

fn project_curve_region_loop(
    boundary: &CurveRegionBoundaryLoop2,
    options: &FiniteProjectionOptions,
    policy: &CurveContext,
) -> CurveResult<FinitePolyline2> {
    let mut points = Vec::with_capacity(boundary.fragments().len() + 1);
    for fragment in boundary.fragments() {
        match fragment {
            BezierSplitFragment2::Materialized { curve, .. } => {
                append_bezier_subcurve_samples(&mut points, curve, options, policy, 0)?;
            }
            BezierSplitFragment2::AlgebraicEndpointImages {
                reversed,
                start,
                end,
                source_curve: source,
                ..
            } => {
                let start = finite_parameter_representative(start, policy)?;
                let end = finite_parameter_representative(end, policy)?;
                let subcurve = match source.subcurve_between_exact(&start, &end, policy)? {
                    Classification::Decided(curve) => curve,
                    Classification::Uncertain(reason) => {
                        return Err(CurveError::Topology(format!(
                            "finite projection could not materialize retained algebraic fragment: {reason:?}"
                        )));
                    }
                };
                let subcurve = if *reversed {
                    subcurve.reversed()
                } else {
                    subcurve
                };
                append_bezier_subcurve_samples(&mut points, &subcurve, options, policy, 0)?;
            }
            BezierSplitFragment2::AnalyticParallel(fragment) => {
                append_analytic_parallel_samples(&mut points, fragment, options, policy)?;
            }
            BezierSplitFragment2::AlgebraicChord(chord) => {
                push_if_new(
                    &mut points,
                    finite_retained_point(chord.start(), options.chord_error, policy)?,
                );
                push_if_new(
                    &mut points,
                    finite_retained_point(chord.end(), options.chord_error, policy)?,
                );
            }
            BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) => {
                append_algebraic_cusp_semicircle_samples(&mut points, fragment, options, policy)?;
            }
            BezierSplitFragment2::SelectedFiber(fragment) => {
                append_selected_fiber_samples(&mut points, fragment, options, policy)?;
            }
        }
    }
    close_ring(&mut points);
    Ok(FinitePolyline2::new(points, options.chord_error, true))
}

struct FiniteParameterProjection2 {
    lower: Real,
    value: Real,
    upper: Real,
}

fn finite_region_parameter_projection(
    parameter: &CurveRegionParameter2,
    refinement_steps: usize,
    policy: &CurveContext,
) -> CurveResult<FiniteParameterProjection2> {
    if let Some(parameter) = parameter.as_bezier_parameter() {
        return Ok(
            match parameter
                .clone()
                .refined_isolating_interval(refinement_steps, policy)
            {
                BezierParameter2::Exact(value) => FiniteParameterProjection2 {
                    lower: value.clone(),
                    value: value.clone(),
                    upper: value,
                },
                BezierParameter2::Algebraic(parameter) => {
                    let lower = parameter.interval().start().clone();
                    let upper = parameter.interval().end().clone();
                    FiniteParameterProjection2 {
                        value: ((&lower + &upper) / Real::from(2_u8))?,
                        lower,
                        upper,
                    }
                }
            },
        );
    }
    let selected = parameter.as_selected_fiber().ok_or_else(|| {
        CurveError::Topology(
            "finite projection encountered a non-source selected parameter domain".into(),
        )
    })?;
    match selected.finite_projection_interval(refinement_steps, policy)? {
        Classification::Decided((lower, value, upper)) => Ok(FiniteParameterProjection2 {
            lower,
            value,
            upper,
        }),
        Classification::Uncertain(reason) => Err(CurveError::Topology(format!(
            "finite projection could not refine a selected-fiber parameter: {reason:?}"
        ))),
    }
}

fn finite_retained_point(
    point: &RationalBezierIntersectionPointEvidence2,
    chord_error: f64,
    policy: &CurveContext,
) -> CurveResult<[f64; 2]> {
    if let Some(point) = point.as_exact() {
        return finite_point(point);
    }
    for refinement_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
        let Classification::Decided(bounds) =
            algebraic_chord_endpoint_bounds_refined(point, refinement_steps, policy)
        else {
            continue;
        };
        let minimum = finite_point(bounds.min())?;
        let maximum = finite_point(bounds.max())?;
        let projected = [
            (minimum[0] + maximum[0]) * 0.5,
            (minimum[1] + maximum[1]) * 0.5,
        ];
        let finite_resolution =
            f64::EPSILON * projected[0].abs().max(projected[1].abs()).max(1.0) * 8.0;
        let allowed = (chord_error * 0.125).max(finite_resolution);
        if maximum[0] - minimum[0] <= allowed && maximum[1] - minimum[1] <= allowed {
            return Ok(projected);
        }
    }
    Err(CurveError::Topology(
        "finite projection could not resolve a retained endpoint within its chord-error budget"
            .into(),
    ))
}

fn append_analytic_parallel_samples(
    points: &mut Vec<[f64; 2]>,
    fragment: &crate::BezierParallelFragment2,
    options: &FiniteProjectionOptions,
    policy: &CurveContext,
) -> CurveResult<()> {
    let endpoint = |parameter: &BezierParameter2| -> CurveResult<_> {
        if let Some(parameter) = parameter.as_exact() {
            return match fragment.parallel().point_at(parameter, policy)? {
                Classification::Decided(point) => {
                    Ok(RationalBezierIntersectionPointEvidence2::Exact(point))
                }
                Classification::Uncertain(reason) => Err(CurveError::Topology(format!(
                    "finite analytic-parallel endpoint evaluation remained uncertain: {reason:?}"
                ))),
            };
        }
        Ok(RationalBezierIntersectionPointEvidence2::AnalyticParallel(
            crate::BezierAnalyticParallelPoint2::new(
                fragment.parallel().clone(),
                parameter.clone(),
                policy,
            ),
        ))
    };
    let source_start = endpoint(fragment.range().start())?;
    let source_end = endpoint(fragment.range().end())?;
    let (start, end) = if fragment.is_reversed() {
        (&source_end, &source_start)
    } else {
        (&source_start, &source_end)
    };
    let start = finite_retained_point(start, options.chord_error, policy)?;
    let end = finite_retained_point(end, options.chord_error, policy)?;
    let range_start = CurveRegionParameter2::from_bezier(fragment.range().start().clone());
    let range_end = CurveRegionParameter2::from_bezier(fragment.range().end().clone());
    append_retained_parallel_range_samples(
        points,
        fragment.parallel(),
        &range_start,
        &range_end,
        fragment.is_reversed(),
        start,
        end,
        options.chord_error,
        policy,
    )
}

fn finite_cusp_parameter_projection(
    parameter: &crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2,
    refinement_steps: usize,
    policy: &CurveContext,
) -> CurveResult<FiniteParameterProjection2> {
    match parameter.finite_projection_interval(refinement_steps, policy)? {
        Classification::Decided((lower, value, upper)) => Ok(FiniteParameterProjection2 {
            lower,
            value,
            upper,
        }),
        Classification::Uncertain(reason) => Err(CurveError::Topology(format!(
            "finite projection could not refine a selected-circle parameter: {reason:?}"
        ))),
    }
}

fn append_algebraic_cusp_semicircle_samples(
    points: &mut Vec<[f64; 2]>,
    fragment: &crate::BezierAlgebraicCuspSemicircleFragment2,
    options: &FiniteProjectionOptions,
    policy: &CurveContext,
) -> CurveResult<()> {
    let finite_endpoint = |start_endpoint| -> CurveResult<Option<[f64; 2]>> {
        Ok(
            match fragment.endpoint_point_evidence(start_endpoint, policy)? {
                Classification::Decided(Some(point)) => {
                    Some(finite_retained_point(&point, options.chord_error, policy)?)
                }
                Classification::Decided(None) | Classification::Uncertain(_) => None,
            },
        )
    };
    let retained_start = finite_endpoint(true)?;
    let retained_end = finite_endpoint(false)?;
    let radius = finite_real(&fragment.semicircle().radial_distance().abs())?;
    let mut last_error = None;
    for refinement_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
        let range_start =
            finite_cusp_parameter_projection(fragment.start_parameter(), refinement_steps, policy)?;
        let range_end =
            finite_cusp_parameter_projection(fragment.end_parameter(), refinement_steps, policy)?;
        if crate::classify::compare_reals(
            &range_start.upper,
            &range_end.lower,
            &CurveContext::STRICT,
        ) != Some(std::cmp::Ordering::Less)
        {
            continue;
        }
        let (start_range, end_range) = if fragment.is_reversed() {
            (&range_end, &range_start)
        } else {
            (&range_start, &range_end)
        };
        let projected_endpoint = |range: &FiniteParameterProjection2| -> CurveResult<Option<_>> {
            let angle = |parameter: &Real| {
                let parameter = finite_real(parameter)?;
                Ok::<_, CurveError>(
                    (2.0 * parameter * (1.0 - parameter)).atan2(1.0 - 2.0 * parameter),
                )
            };
            let angular_width = angle(&range.upper)? - angle(&range.lower)?;
            if 2.0 * radius * (angular_width * 0.5).sin().abs() > options.chord_error * 0.125 {
                return Ok(None);
            }
            let point = match fragment
                .semicircle()
                .point_evidence_at(&range.value, policy)?
            {
                Classification::Decided(point) => point,
                Classification::Uncertain(_) => return Ok(None),
            };
            finite_retained_point(&point, options.chord_error, policy).map(Some)
        };
        let start = match retained_start {
            Some(start) => start,
            None => match projected_endpoint(start_range)? {
                Some(start) => start,
                None => continue,
            },
        };
        let end = match retained_end {
            Some(end) => end,
            None => match projected_endpoint(end_range)? {
                Some(end) => end,
                None => continue,
            },
        };
        let mut projected = vec![start];
        match append_cusp_parameter_samples(
            &mut projected,
            fragment,
            &start_range.value,
            &end_range.value,
            &range_start.lower,
            &range_end.upper,
            end,
            radius,
            options.chord_error,
            policy,
            0,
        ) {
            Ok(()) => {
                for point in projected {
                    push_if_new(points, point);
                }
                return Ok(());
            }
            Err(error @ CurveError::Topology(_)) => last_error = Some(error),
            Err(error) => return Err(error),
        }
    }
    Err(last_error.unwrap_or_else(|| {
        CurveError::Topology(
            "finite selected-circle projection could not separate its exact parameter endpoints"
                .into(),
        )
    }))
}

#[allow(clippy::too_many_arguments)]
fn append_cusp_parameter_samples(
    points: &mut Vec<[f64; 2]>,
    fragment: &crate::BezierAlgebraicCuspSemicircleFragment2,
    start_parameter: &Real,
    end_parameter: &Real,
    bounds_lower: &Real,
    bounds_upper: &Real,
    end_point: [f64; 2],
    radius: f64,
    chord_error: f64,
    policy: &CurveContext,
    depth: usize,
) -> CurveResult<()> {
    const MAX_DEPTH: usize = 32;
    let half_circle_angle = |parameter: f64| {
        let radial = 1.0 - 2.0 * parameter;
        let tangent = 2.0 * parameter * (1.0 - parameter);
        tangent.atan2(radial)
    };
    let angular_span = half_circle_angle(finite_real(bounds_upper)?)
        - half_circle_angle(finite_real(bounds_lower)?);
    let sagitta = radius * (1.0 - (angular_span * 0.5).cos());
    if sagitta <= chord_error * 0.75 {
        push_if_new(points, end_point);
        return Ok(());
    }
    if depth >= MAX_DEPTH {
        return Err(CurveError::Topology(
            "finite selected-circle projection exceeded its subdivision depth".into(),
        ));
    }
    let midpoint = ((start_parameter + end_parameter) / Real::from(2_u8))?;
    let midpoint_evidence = match fragment.semicircle().point_evidence_at(&midpoint, policy)? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => {
            return Err(CurveError::Topology(format!(
                "finite selected-circle midpoint remained uncertain: {reason:?}"
            )));
        }
    };
    let midpoint_point = finite_retained_point(&midpoint_evidence, chord_error, policy)?;
    let increasing =
        match crate::classify::compare_reals(start_parameter, end_parameter, &CurveContext::STRICT)
        {
            Some(std::cmp::Ordering::Less) => true,
            Some(std::cmp::Ordering::Greater) => false,
            Some(std::cmp::Ordering::Equal) | None => {
                return Err(CurveError::Topology(
                    "finite selected-circle projection lost parameter ordering".into(),
                ));
            }
        };
    let (first_lower, first_upper, second_lower, second_upper) = if increasing {
        (bounds_lower, &midpoint, &midpoint, bounds_upper)
    } else {
        (&midpoint, bounds_upper, bounds_lower, &midpoint)
    };
    append_cusp_parameter_samples(
        points,
        fragment,
        start_parameter,
        &midpoint,
        first_lower,
        first_upper,
        midpoint_point,
        radius,
        chord_error,
        policy,
        depth + 1,
    )?;
    append_cusp_parameter_samples(
        points,
        fragment,
        &midpoint,
        end_parameter,
        second_lower,
        second_upper,
        end_point,
        radius,
        chord_error,
        policy,
        depth + 1,
    )
}

fn append_selected_fiber_samples(
    points: &mut Vec<[f64; 2]>,
    fragment: &crate::bezier_split::BezierSelectedFiberFragment2,
    options: &FiniteProjectionOptions,
    policy: &CurveContext,
) -> CurveResult<()> {
    let parallel = match fragment.source() {
        BezierSelectedFiberSource2::Rational(curve) => BezierParallel2::from_source(
            BezierParallelSource2::Rational(curve.clone()),
            Real::zero(),
        ),
        BezierSelectedFiberSource2::AnalyticParallel(parallel) => parallel.clone(),
    };
    let start_point = finite_retained_point(fragment.start_point(), options.chord_error, policy)?;
    let end_point = finite_retained_point(fragment.end_point(), options.chord_error, policy)?;
    append_retained_parallel_range_samples(
        points,
        &parallel,
        fragment.range().start(),
        fragment.range().end(),
        fragment.is_reversed(),
        start_point,
        end_point,
        options.chord_error,
        policy,
    )
}

#[allow(clippy::too_many_arguments)]
fn append_retained_parallel_range_samples(
    points: &mut Vec<[f64; 2]>,
    parallel: &BezierParallel2,
    range_start_parameter: &CurveRegionParameter2,
    range_end_parameter: &CurveRegionParameter2,
    reversed: bool,
    start_point: [f64; 2],
    end_point: [f64; 2],
    chord_error: f64,
    policy: &CurveContext,
) -> CurveResult<()> {
    let mut last_error = None;
    for refinement_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
        let range_start =
            finite_region_parameter_projection(range_start_parameter, refinement_steps, policy)?;
        let range_end =
            finite_region_parameter_projection(range_end_parameter, refinement_steps, policy)?;
        if crate::classify::compare_reals(
            &range_start.upper,
            &range_end.lower,
            &CurveContext::STRICT,
        ) != Some(std::cmp::Ordering::Less)
        {
            continue;
        }
        let (start_parameter, end_parameter) = if reversed {
            (&range_end.value, &range_start.value)
        } else {
            (&range_start.value, &range_end.value)
        };
        let mut projected = vec![start_point];
        match append_parallel_parameter_samples(
            &mut projected,
            parallel,
            start_parameter,
            end_parameter,
            &range_start.lower,
            &range_end.upper,
            start_point,
            end_point,
            chord_error,
            policy,
            0,
        ) {
            Ok(()) => {
                for point in projected {
                    push_if_new(points, point);
                }
                return Ok(());
            }
            Err(error @ CurveError::Topology(_)) => last_error = Some(error),
            Err(error) => return Err(error),
        }
    }
    Err(last_error.unwrap_or_else(|| {
        CurveError::Topology(
            "finite retained-parallel projection could not separate its exact parameter endpoints"
                .into(),
        )
    }))
}

#[allow(clippy::too_many_arguments)]
fn append_parallel_parameter_samples(
    points: &mut Vec<[f64; 2]>,
    parallel: &BezierParallel2,
    start_parameter: &Real,
    end_parameter: &Real,
    bounds_lower: &Real,
    bounds_upper: &Real,
    start_point: [f64; 2],
    end_point: [f64; 2],
    chord_error: f64,
    policy: &CurveContext,
    depth: usize,
) -> CurveResult<()> {
    const MAX_DEPTH: usize = 32;
    let bounds = match parallel.finite_projection_bounds_over_parameter_interval(
        bounds_lower,
        bounds_upper,
        policy,
    ) {
        Classification::Decided(bounds) => bounds,
        Classification::Uncertain(reason) => {
            return Err(CurveError::Topology(format!(
                "finite retained-parallel projection could not bound its analytic carrier: {reason:?}"
            )));
        }
    };
    let corners = [
        [
            finite_real(bounds.min().x())?,
            finite_real(bounds.min().y())?,
        ],
        [
            finite_real(bounds.min().x())?,
            finite_real(bounds.max().y())?,
        ],
        [
            finite_real(bounds.max().x())?,
            finite_real(bounds.min().y())?,
        ],
        [
            finite_real(bounds.max().x())?,
            finite_real(bounds.max().y())?,
        ],
    ];
    if corners
        .into_iter()
        .map(|point| point_segment_distance(point, start_point, end_point))
        .fold(0.0, f64::max)
        <= chord_error
    {
        push_if_new(points, end_point);
        return Ok(());
    }
    if depth >= MAX_DEPTH {
        return Err(CurveError::Topology(
            "finite retained-parallel projection exceeded its subdivision depth".into(),
        ));
    }

    let midpoint = ((start_parameter + end_parameter) / Real::from(2_u8))?;
    let midpoint_point = match parallel.point_at(&midpoint, policy)? {
        Classification::Decided(point) => finite_point(&point)?,
        Classification::Uncertain(reason) => {
            return Err(CurveError::Topology(format!(
                "finite retained-parallel projection could not evaluate its analytic carrier: {reason:?}"
            )));
        }
    };
    let increasing =
        match crate::classify::compare_reals(start_parameter, end_parameter, &CurveContext::STRICT)
        {
            Some(std::cmp::Ordering::Less) => true,
            Some(std::cmp::Ordering::Greater) => false,
            Some(std::cmp::Ordering::Equal) | None => {
                return Err(CurveError::Topology(
                    "finite retained-parallel projection lost parameter ordering".into(),
                ));
            }
        };
    let (first_lower, first_upper, second_lower, second_upper) = if increasing {
        (bounds_lower, &midpoint, &midpoint, bounds_upper)
    } else {
        (&midpoint, bounds_upper, bounds_lower, &midpoint)
    };
    append_parallel_parameter_samples(
        points,
        parallel,
        start_parameter,
        &midpoint,
        first_lower,
        first_upper,
        start_point,
        midpoint_point,
        chord_error,
        policy,
        depth + 1,
    )?;
    append_parallel_parameter_samples(
        points,
        parallel,
        &midpoint,
        end_parameter,
        second_lower,
        second_upper,
        midpoint_point,
        end_point,
        chord_error,
        policy,
        depth + 1,
    )
}

fn finite_parameter_representative(
    parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Real> {
    if let Some(exact) = parameter.as_exact() {
        return Ok(exact.clone());
    }
    let interval = match parameter.known_interval(policy)? {
        Classification::Decided(interval) => interval,
        Classification::Uncertain(reason) => {
            return Err(CurveError::Topology(format!(
                "finite projection could not isolate algebraic parameter: {reason:?}"
            )));
        }
    };
    Ok(((&interval.start().clone() + interval.end()) / Real::from(2_u8))?)
}

fn append_bezier_subcurve_samples(
    points: &mut Vec<[f64; 2]>,
    curve: &BezierSubcurve2,
    options: &FiniteProjectionOptions,
    policy: &CurveContext,
    depth: usize,
) -> CurveResult<()> {
    const MAX_DEPTH: usize = 32;
    let controls = finite_subcurve_controls(curve)?;
    let common_weight_sign = subcurve_has_common_weight_sign(curve, policy);
    let flat = common_weight_sign && control_polygon_chord_error(&controls) <= options.chord_error;
    if flat {
        push_if_new(points, controls[0]);
        push_if_new(
            points,
            *controls.last().expect("Bezier controls are nonempty"),
        );
        return Ok(());
    }
    if depth >= MAX_DEPTH {
        return Err(CurveError::Topology(
            "finite higher-order projection exceeded its subdivision depth".into(),
        ));
    }

    let half = (Real::one() / Real::from(2_u8))?;
    let left = match curve.subcurve_between_exact(&Real::zero(), &half, policy)? {
        Classification::Decided(curve) => curve,
        Classification::Uncertain(reason) => {
            return Err(CurveError::Topology(format!(
                "finite projection could not split higher-order curve: {reason:?}"
            )));
        }
    };
    let right = match curve.subcurve_between_exact(&half, &Real::one(), policy)? {
        Classification::Decided(curve) => curve,
        Classification::Uncertain(reason) => {
            return Err(CurveError::Topology(format!(
                "finite projection could not split higher-order curve: {reason:?}"
            )));
        }
    };
    append_bezier_subcurve_samples(points, &left, options, policy, depth + 1)?;
    append_bezier_subcurve_samples(points, &right, options, policy, depth + 1)
}

fn finite_subcurve_controls(curve: &BezierSubcurve2) -> CurveResult<Vec<[f64; 2]>> {
    match curve {
        BezierSubcurve2::Quadratic(curve) => curve
            .control_points()
            .into_iter()
            .map(finite_point)
            .collect(),
        BezierSubcurve2::Cubic(curve) => curve
            .control_points()
            .into_iter()
            .map(finite_point)
            .collect(),
        BezierSubcurve2::RationalQuadratic(curve) => curve
            .control_points()
            .into_iter()
            .map(finite_point)
            .collect(),
        BezierSubcurve2::Rational(curve) => {
            curve.control_points().iter().map(finite_point).collect()
        }
    }
}

fn subcurve_has_common_weight_sign(curve: &BezierSubcurve2, policy: &CurveContext) -> bool {
    let mut sign = None;
    let mut accepts = |weight: &Real| {
        let current =
            crate::classify::real_sign(weight, policy).filter(|value| *value != RealSign::Zero);
        match (sign, current) {
            (None, Some(current)) => {
                sign = Some(current);
                true
            }
            (Some(expected), Some(current)) => expected == current,
            _ => false,
        }
    };
    let common = match curve {
        BezierSubcurve2::Quadratic(_) | BezierSubcurve2::Cubic(_) => return true,
        BezierSubcurve2::RationalQuadratic(curve) => curve.weights().into_iter().all(&mut accepts),
        BezierSubcurve2::Rational(curve) => curve.weights().iter().all(&mut accepts),
    };
    common && sign.is_some()
}

fn control_polygon_chord_error(controls: &[[f64; 2]]) -> f64 {
    let start = controls[0];
    let end = *controls.last().expect("Bezier controls are nonempty");
    controls
        .iter()
        .copied()
        .map(|point| point_segment_distance(point, start, end))
        .fold(0.0, f64::max)
}

fn point_segment_distance(point: [f64; 2], start: [f64; 2], end: [f64; 2]) -> f64 {
    let dx = end[0] - start[0];
    let dy = end[1] - start[1];
    let length_squared = dx * dx + dy * dy;
    if length_squared <= f64::EPSILON {
        return ((point[0] - start[0]).powi(2) + (point[1] - start[1]).powi(2)).sqrt();
    }
    let t = (((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_squared)
        .clamp(0.0, 1.0);
    let projection = [start[0] + t * dx, start[1] + t * dy];
    ((point[0] - projection[0]).powi(2) + (point[1] - projection[1]).powi(2)).sqrt()
}

fn finite_ring_contains_point(ring: &[[f64; 2]], point: [f64; 2]) -> bool {
    if ring.len() < 3 {
        return false;
    }
    let mut inside = false;
    for edge in ring.windows(2) {
        let [first, second] = [edge[0], edge[1]];
        if (first[1] > point[1]) != (second[1] > point[1])
            && point[0]
                < (second[0] - first[0]) * (point[1] - first[1]) / (second[1] - first[1]) + first[0]
        {
            inside = !inside;
        }
    }
    inside
}

fn finite_point(point: &Point2) -> CurveResult<[f64; 2]> {
    Ok([finite_real(point.x())?, finite_real(point.y())?])
}

fn finite_real(value: &Real) -> CurveResult<f64> {
    value
        .to_f64_lossy()
        .filter(|value| value.is_finite())
        .ok_or(CurveError::NonFiniteProjectionPoint)
}

fn append_arc_samples(
    points: &mut Vec<[f64; 2]>,
    arc: &CircularArc2,
    chord_error: f64,
) -> CurveResult<usize> {
    let start = finite_point(arc.start())?;
    let end = finite_point(arc.end())?;
    let center = finite_point(arc.center())?;

    let radius = ((start[0] - center[0]).powi(2) + (start[1] - center[1]).powi(2)).sqrt();
    if !radius.is_finite() || radius <= f64::EPSILON {
        return Err(CurveError::NonFiniteProjectionPoint);
    }

    let a0 = (start[1] - center[1]).atan2(start[0] - center[0]);
    let a1 = (end[1] - center[1]).atan2(end[0] - center[0]);
    let mut sweep = a1 - a0;
    if arc.is_clockwise() {
        if sweep > 0.0 {
            sweep -= 2.0 * PI;
        }
    } else if sweep < 0.0 {
        sweep += 2.0 * PI;
    }

    let max_angle = (1.0 - (chord_error / radius).min(1.0)).acos().max(1e-3) * 2.0;
    let steps = ((sweep.abs() / max_angle).ceil() as usize).max(1);
    let before = points.len();
    for step in 1..=steps {
        let t = step as f64 / steps as f64;
        let angle = a0 + sweep * t;
        let point = [
            center[0] + radius * angle.cos(),
            center[1] + radius * angle.sin(),
        ];
        if !point[0].is_finite() || !point[1].is_finite() {
            return Err(CurveError::NonFiniteProjectionPoint);
        }
        push_if_new(points, point);
    }
    Ok(points.len() - before)
}

fn close_ring(points: &mut Vec<[f64; 2]>) {
    if points.len() >= 2 && points.first() != points.last() {
        points.push(points[0]);
    }
}

fn push_if_new(points: &mut Vec<[f64; 2]>, point: [f64; 2]) {
    if points.last().is_none_or(|last| *last != point) {
        points.push(point);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::QuadraticBezier2;
    use crate::{CubicBezier2, Curve2, LineSeg2};

    fn point(x: i64, y: i64) -> Point2 {
        Point2::new(Real::from(x), Real::from(y))
    }

    fn cubic_cap() -> CurvePath2 {
        CurvePath2::try_new(vec![
            Curve2::from(LineSeg2::try_new(point(0, 0), point(4, 0)).unwrap()),
            Curve2::from(LineSeg2::try_new(point(4, 0), point(4, 4)).unwrap()),
            Curve2::from(CubicBezier2::new(
                point(4, 4),
                point(3, 5),
                point(1, 5),
                point(0, 4),
            )),
            Curve2::from(LineSeg2::try_new(point(0, 4), point(0, 0)).unwrap()),
        ])
        .unwrap()
    }

    #[test]
    fn projects_higher_order_path_without_demoting_source() {
        let path = cubic_cap();
        let projection = path
            .project_to_finite_polyline(
                &FiniteProjectionOptions::try_new(1.0e-3).unwrap(),
                &CurveContext::STRICT,
            )
            .unwrap()
            .into_value();

        assert!(projection.is_closed());
        assert!(projection.points().len() > path.curves().len() + 1);
        assert!(matches!(
            path.curves()[2].geometry(),
            crate::CurveGeometry2::CubicBezier(_)
        ));
    }

    #[test]
    fn path_projection_obeys_terminal_policy_and_reports_consumption() {
        let start = Point2::new(Real::pi() + Real::e(), Real::zero());
        let end = Point2::new(Real::e() + Real::pi(), Real::zero());
        let path = CurvePath2::try_new(vec![Curve2::from(QuadraticBezier2::new(
            start,
            point(0, 1),
            end,
        ))])
        .unwrap();
        let options = FiniteProjectionOptions::try_new(10.0).unwrap();

        assert!(matches!(
            path.project_to_finite_polyline(&options, &CurveContext::STRICT),
            Err(CurveError::Topology(message))
                if message == "finite path projection could not decide endpoint closure"
        ));
        let approximate = path
            .project_to_finite_polyline(&options, &CurveContext::APPROXIMATE_512)
            .unwrap();
        assert_eq!(
            approximate.certainty,
            crate::CurveCertainty::Approximate512Consumed
        );
        assert!(approximate.value.is_closed());
    }

    #[test]
    fn region_curve_path_projection_rechecks_symbolic_materialized_joins() {
        let start = Point2::new(Real::pi() + Real::e(), Real::zero());
        let end = Point2::new(Real::e() + Real::pi(), Real::zero());
        let path = CurvePath2::try_new(vec![Curve2::from(QuadraticBezier2::new(
            start,
            point(0, 1),
            end,
        ))])
        .unwrap();
        let region = CurveRegion2::try_from_boundary_paths(&[path], &CurveContext::APPROXIMATE_512)
            .unwrap()
            .into_value();

        let strict = region
            .project_to_finite_curve_paths(&CurveContext::STRICT)
            .unwrap();
        assert_eq!(strict.certainty, crate::CurveCertainty::Certified);
        assert_eq!(
            strict.value,
            Classification::Uncertain(crate::UncertaintyReason::Unsupported)
        );
        let approximate = region
            .project_to_finite_curve_paths(&CurveContext::APPROXIMATE_512)
            .unwrap();
        assert_eq!(
            approximate.certainty,
            crate::CurveCertainty::Approximate512Consumed
        );
        let Classification::Decided(paths) = approximate.value else {
            panic!("the authorized terminal must project the symbolic loop");
        };
        assert_eq!(paths.len(), 1);
        assert_eq!(paths[0].start(), paths[0].end());
    }

    #[test]
    fn projects_higher_order_region_after_exact_role_assignment() {
        let region = CurveRegion2::try_from_boundary_paths(&[cubic_cap()], &CurveContext::STRICT)
            .unwrap()
            .into_value();
        let options = FiniteProjectionOptions::try_new(1.0e-3).unwrap();
        let policy = CurveContext::STRICT;
        let profiles = region
            .project_to_finite_profiles(&options, &policy)
            .unwrap()
            .into_value();
        let exact_profiles = region
            .project_to_finite_profiles_exact(&options, &policy)
            .unwrap()
            .into_value();
        let Classification::Decided(profiles) = profiles else {
            panic!("cubic region roles should be decided");
        };
        let Classification::Decided(exact_profiles) = exact_profiles else {
            panic!("exact cubic region roles should be decided");
        };

        assert_eq!(profiles, exact_profiles);
        assert_eq!(profiles.len(), 1);
        assert!(profiles[0].material().points().len() > 5);
        assert!(profiles[0].holes().is_empty());
    }

    #[test]
    fn materialized_curve_path_projection_preserves_conic_provenance() {
        let circle = CurvePath2::try_new(vec![
            Curve2::from(
                CircularArc2::try_from_center(point(1, 0), point(-1, 0), point(0, 0), false)
                    .unwrap(),
            ),
            Curve2::from(
                CircularArc2::try_from_center(point(-1, 0), point(1, 0), point(0, 0), false)
                    .unwrap(),
            ),
        ])
        .unwrap();
        let region = CurveRegion2::try_from_boundary_paths(&[circle], &CurveContext::STRICT)
            .unwrap()
            .into_value();
        let Classification::Decided(paths) = region
            .project_to_finite_curve_paths(&CurveContext::STRICT)
            .unwrap()
            .into_value()
        else {
            panic!("the materialized circle must project to native curve paths");
        };

        assert!(paths[0].curves().iter().all(|curve| {
            matches!(
                curve.geometry(),
                crate::CurveGeometry2::RationalQuadraticBezier(conic)
                    if conic.retained_circular_conic().is_some()
            )
        }));
    }
}
