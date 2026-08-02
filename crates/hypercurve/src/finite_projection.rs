//! Finite polyline projection adapters for native hyper curves.
//!
//! Projection is an IO/rendering boundary, not a topology kernel. The methods
//! in this module preserve line segments exactly, approximate circular arcs by
//! a chord-error budget, and return primitive `f64` coordinates only after the
//! source [`Real`](hyperreal::Real) coordinates can be exported finitely. This
//! follows exact-computation discipline:
//! exact objects own CAD/topology; finite samples are boundary products.
//! Boundary and containment decisions should continue to use the exact
//! contour/region APIs surveyed by boundary-first winding classification.

use std::f64::consts::PI;

use crate::bezier_parameter::BezierParameterRefinement2;
use crate::{
    BezierParameter2, BezierSplitFragment2, BezierSubcurve2, CircularArc2, Classification,
    Contour2, Curve2, CurveContext, CurveError, CurveOutcome, CurvePath2, CurveRegion2,
    CurveRegionBoundaryLoop2, CurveRegionLoopRole, CurveResult, CurveString2, LineArcRegion2,
    Point2, RegionContourProfile, RegionView2, Segment2,
};
use hyperreal::{Real, RealSign};

/// Options for projecting native curves to finite `f64` polylines.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FiniteProjectionOptions {
    arc_chord_error: f64,
}

/// Finite `f64` polyline emitted from a native curve object.
#[derive(Clone, Debug, PartialEq)]
pub struct FinitePolyline2 {
    points: Vec<[f64; 2]>,
    arc_chord_error: f64,
    closed: bool,
}

/// Finite `f64` projection of a region with material and hole roles retained.
///
/// This is an IO/display object. Exact containment, area, and boolean topology
/// remain on [`LineArcRegion2`] / [`RegionView2`].
#[derive(Clone, Debug, PartialEq)]
pub struct FiniteRegionProjection2 {
    material_rings: Vec<FinitePolyline2>,
    hole_rings: Vec<FinitePolyline2>,
}

/// A finite material ring and the finite hole rings owned by it.
///
/// This is the projected counterpart to [`RegionContourProfile`]. Ownership is
/// still decided in exact hypercurve topology before any finite ring is
/// emitted; this type only carries the boundary result.
#[derive(Clone, Debug, PartialEq)]
pub struct FiniteRegionProfile2 {
    material: FinitePolyline2,
    holes: Vec<FinitePolyline2>,
}

impl FiniteProjectionOptions {
    /// Constructs projection options with a positive finite arc chord-error budget.
    pub fn try_new(arc_chord_error: f64) -> CurveResult<Self> {
        if arc_chord_error.is_finite() && arc_chord_error > 0.0 {
            Ok(Self { arc_chord_error })
        } else {
            Err(CurveError::InvalidFiniteProjectionOptions)
        }
    }

    /// Returns the maximum requested circular-arc chord error.
    pub const fn arc_chord_error(&self) -> f64 {
        self.arc_chord_error
    }
}

impl FinitePolyline2 {
    fn new(points: Vec<[f64; 2]>, arc_chord_error: f64, closed: bool) -> Self {
        Self {
            points,
            arc_chord_error,
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

    /// Returns the arc chord-error budget requested for this projection.
    pub const fn arc_chord_error(&self) -> f64 {
        self.arc_chord_error
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
    /// [`crate::LineArcRegion2::filled_area`].
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

impl FiniteRegionProjection2 {
    fn new(material_rings: Vec<FinitePolyline2>, hole_rings: Vec<FinitePolyline2>) -> Self {
        Self {
            material_rings,
            hole_rings,
        }
    }

    /// Returns projected material rings.
    pub fn material_rings(&self) -> &[FinitePolyline2] {
        &self.material_rings
    }

    /// Returns projected hole rings.
    pub fn hole_rings(&self) -> &[FinitePolyline2] {
        &self.hole_rings
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
    /// formula. Exact CAD area should use [`LineArcRegion2::filled_area`]; this helper
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
        Ok(FinitePolyline2::new(
            points,
            options.arc_chord_error,
            closed,
        ))
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
                source_curve: Some(source),
                ..
            } => {
                let (start, end) = finite_parameter_pair(start, end, policy)?;
                let curve = match source.subcurve_between_exact(&start, &end, policy)? {
                    Classification::Decided(curve) => curve,
                    Classification::Uncertain(_) => return Ok(None),
                };
                if *reversed { curve.reversed() } else { curve }
            }
            BezierSplitFragment2::AlgebraicEndpointImages {
                source_curve: None, ..
            }
            | BezierSplitFragment2::AnalyticParallel(_)
            | BezierSplitFragment2::Unresolved { .. } => return Ok(None),
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

impl LineArcRegion2 {
    /// Projects this region to finite material/hole rings for IO and display.
    ///
    /// Region roles are preserved, but the returned rings are boundary
    /// products only. Exact point classification and area should continue to
    /// use [`LineArcRegion2::classify_point`] and [`LineArcRegion2::filled_area`].
    pub fn project_to_finite_region(
        &self,
        options: &FiniteProjectionOptions,
    ) -> CurveResult<FiniteRegionProjection2> {
        self.as_view().project_to_finite_region(options)
    }

    /// Projects exact material/hole ownership profiles to finite rings.
    ///
    /// Ownership is classified before projection with
    /// [`LineArcRegion2::contour_profiles`], so this method does not recover holes
    /// from sampled centroids or winding heuristics. The returned rings are
    /// still finite API-boundary products; exact topology remains in the
    /// region. This follows the exact-object/API-boundary split and the
    /// boundary-first point-in-polygon structure used by
    /// [`LineArcRegion2::contour_profiles`].
    #[inline]
    pub fn project_to_finite_profiles(
        &self,
        options: &FiniteProjectionOptions,
        policy: &CurveContext,
    ) -> CurveResult<CurveOutcome<Classification<Vec<FiniteRegionProfile2>>>> {
        crate::policy::resolve_certified_operation(policy, |attempt| {
            self.as_view()
                .project_to_finite_profiles_raw(options, attempt)
        })
    }
}

impl<'a> RegionView2<'a> {
    /// Projects this borrowed region view to finite material/hole rings.
    ///
    /// This method exists for export adapters that already work with borrowed
    /// topology. It clones only finite output vertices, not exact contours.
    pub fn project_to_finite_region(
        &self,
        options: &FiniteProjectionOptions,
    ) -> CurveResult<FiniteRegionProjection2> {
        let material_rings = project_contour_slice(self.material_contours(), options)?;
        let hole_rings = project_contour_slice(self.hole_contours(), options)?;
        Ok(FiniteRegionProjection2::new(material_rings, hole_rings))
    }

    /// Projects exact material/hole ownership profiles to finite rings.
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
        match self.contour_profiles(policy) {
            Classification::Decided(profiles) => profiles
                .iter()
                .map(|profile| project_region_profile(profile, options))
                .collect::<CurveResult<Vec<_>>>()
                .map(Classification::Decided),
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }
}

fn project_region_profile(
    profile: &RegionContourProfile<'_>,
    options: &FiniteProjectionOptions,
) -> CurveResult<FiniteRegionProfile2> {
    let material = profile.material.project_to_finite_ring(options)?;
    let holes = profile
        .holes
        .iter()
        .map(|hole| hole.project_to_finite_ring(options))
        .collect::<CurveResult<Vec<_>>>()?;
    Ok(FiniteRegionProfile2::new(material, holes))
}

fn project_contour_slice(
    contours: &[&Contour2],
    options: &FiniteProjectionOptions,
) -> CurveResult<Vec<FinitePolyline2>> {
    contours
        .iter()
        .map(|contour| contour.project_to_finite_ring(options))
        .collect()
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
                append_arc_samples(&mut points, arc, options.arc_chord_error)?;
            }
        }
    }

    if close {
        close_ring(&mut points);
    }

    Ok(FinitePolyline2::new(points, options.arc_chord_error, close))
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
                source_curve: Some(source),
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
            BezierSplitFragment2::AlgebraicEndpointImages {
                source_curve: None, ..
            }
            | BezierSplitFragment2::Unresolved { .. } => {
                return Err(CurveError::Topology(
                    "finite projection requires a retained source curve for algebraic fragments"
                        .into(),
                ));
            }
            BezierSplitFragment2::AnalyticParallel(_) => {
                return Err(CurveError::Topology(
                    "finite projection of exact analytic parallel fragments is not implemented"
                        .into(),
                ));
            }
        }
    }
    close_ring(&mut points);
    Ok(FinitePolyline2::new(points, options.arc_chord_error, true))
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
    let flat =
        common_weight_sign && control_polygon_chord_error(&controls) <= options.arc_chord_error;
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
    let x = point
        .x()
        .to_f64_lossy()
        .filter(|value| value.is_finite())
        .ok_or(CurveError::NonFiniteProjectionPoint)?;
    let y = point
        .y()
        .to_f64_lossy()
        .filter(|value| value.is_finite())
        .ok_or(CurveError::NonFiniteProjectionPoint)?;
    Ok([x, y])
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
    #[cfg(feature = "predicates")]
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

    #[cfg(feature = "predicates")]
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

    #[cfg(feature = "predicates")]
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
