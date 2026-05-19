//! Finite polyline projection adapters for native hyper curves.
//!
//! Projection is an IO/rendering boundary, not a topology kernel. The methods
//! in this module preserve line segments exactly, approximate circular arcs by
//! a chord-error budget, and return primitive `f64` coordinates only after the
//! source [`Real`](hyperreal::Real) coordinates can be exported finitely. This
//! follows Yap, "Towards Exact Geometric Computation," *Computational
//! Geometry* 7(1-2), 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>):
//! exact objects own CAD/topology; finite samples are boundary products.
//! Boundary and containment decisions should continue to use the exact
//! contour/region APIs surveyed by Hormann and Agathos, "The Point in Polygon
//! Problem for Arbitrary Polygons," *Computational Geometry* 20(3), 2001
//! (<https://doi.org/10.1016/S0925-7721(01)00012-8>).

use std::f64::consts::PI;

use crate::{
    CircularArc2, Classification, Contour2, CurveError, CurvePolicy, CurveResult, CurveString2,
    Point2, Region2, RegionContourProfile, RegionView2, Segment2,
};

/// Options for projecting native curves to finite `f64` polylines.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FiniteProjectionOptions {
    arc_chord_error: f64,
}

/// Certificate for a finite polyline projection.
///
/// This report records how exact native line/arc topology crossed the finite
/// output boundary. It deliberately certifies only the projection process:
/// source segment counts, emitted finite vertices, closure, and the requested
/// arc chord-error budget. It is not a topology certificate for the projected
/// `f64` polyline. That boundary follows Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7(1-2), 1997
/// (<https://doi.org/10.1016/0925-7721(95)00040-2>): exact curve objects own
/// predicates, while finite output carries explicit approximation metadata.
#[derive(Clone, Debug, PartialEq)]
pub struct FiniteProjectionCertificate {
    source_segment_count: usize,
    line_segment_count: usize,
    arc_segment_count: usize,
    emitted_point_count: usize,
    emitted_arc_sample_count: usize,
    arc_chord_error: f64,
    closed: bool,
}

/// Finite `f64` polyline emitted from a native curve object.
#[derive(Clone, Debug, PartialEq)]
pub struct FinitePolyline2 {
    points: Vec<[f64; 2]>,
    certificate: FiniteProjectionCertificate,
}

/// Aggregate certificate for projecting a region to finite material/hole rings.
///
/// The certificate preserves region roles at the export boundary and aggregates
/// the per-contour projection facts. It certifies only that every exact contour
/// crossed to finite `f64` vertices under the requested chord-error budget; it
/// does not license downstream code to infer topology from those finite rings.
/// This is the same exact-object/lossy-adapter split advocated by Yap,
/// "Towards Exact Geometric Computation," *Computational Geometry* 7(1-2),
/// 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>).
#[derive(Clone, Debug, PartialEq)]
pub struct FiniteRegionProjectionCertificate {
    material_ring_count: usize,
    hole_ring_count: usize,
    source_segment_count: usize,
    line_segment_count: usize,
    arc_segment_count: usize,
    emitted_point_count: usize,
    emitted_arc_sample_count: usize,
    arc_chord_error: f64,
}

/// Finite `f64` projection of a region with material and hole roles retained.
///
/// This is an IO/display object. Exact containment, area, and boolean topology
/// remain on [`Region2`] / [`RegionView2`].
#[derive(Clone, Debug, PartialEq)]
pub struct FiniteRegionProjection2 {
    material_rings: Vec<FinitePolyline2>,
    hole_rings: Vec<FinitePolyline2>,
    certificate: FiniteRegionProjectionCertificate,
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

impl FiniteProjectionCertificate {
    /// Returns the number of exact source segments traversed.
    pub const fn source_segment_count(&self) -> usize {
        self.source_segment_count
    }

    /// Returns the number of exact source line segments traversed.
    pub const fn line_segment_count(&self) -> usize {
        self.line_segment_count
    }

    /// Returns the number of exact source circular arcs traversed.
    pub const fn arc_segment_count(&self) -> usize {
        self.arc_segment_count
    }

    /// Returns the number of finite vertices emitted.
    pub const fn emitted_point_count(&self) -> usize {
        self.emitted_point_count
    }

    /// Returns the number of finite vertices emitted while sampling arcs.
    ///
    /// This count includes each sampled arc endpoint as emitted by the
    /// projection adapter after duplicate suppression.
    pub const fn emitted_arc_sample_count(&self) -> usize {
        self.emitted_arc_sample_count
    }

    /// Returns the maximum requested circular-arc chord error.
    pub const fn arc_chord_error(&self) -> f64 {
        self.arc_chord_error
    }

    /// Returns true when the projection was explicitly closed as a contour ring.
    pub const fn is_closed(&self) -> bool {
        self.closed
    }
}

impl FinitePolyline2 {
    fn new(points: Vec<[f64; 2]>, certificate: FiniteProjectionCertificate) -> Self {
        Self {
            points,
            certificate,
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

    /// Returns the projection certificate for this finite boundary product.
    pub const fn certificate(&self) -> &FiniteProjectionCertificate {
        &self.certificate
    }

    /// Returns the arc chord-error budget requested for this projection.
    pub const fn arc_chord_error(&self) -> f64 {
        self.certificate.arc_chord_error()
    }

    /// Returns true when this polyline was explicitly closed for a contour.
    pub const fn is_closed(&self) -> bool {
        self.certificate.is_closed()
    }

    /// Returns the finite signed shoelace area when this polyline is treated as
    /// a ring.
    ///
    /// This is only a boundary/product measurement of projected vertices. Exact
    /// contour area stays on [`Contour2::signed_area`] and
    /// [`crate::Region2::filled_area`].
    pub fn signed_ring_area(&self) -> f64 {
        finite_ring_signed_area(&self.points)
    }

    /// Returns the arithmetic centroid of this finite projected polyline.
    ///
    /// This is a boundary-product measurement over emitted finite vertices, not
    /// an exact centroid of the native curve or filled area. A repeated closing
    /// vertex is ignored. Keeping this helper on the projected polyline type
    /// prevents downstream crates from reimplementing small finite adapters
    /// around hypercurve output. The exact-object/boundary split follows Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry* 7(1-2),
    /// 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn vertex_centroid(&self) -> Option<[f64; 2]> {
        finite_polyline_vertex_centroid(&self.points)
    }
}

impl FiniteRegionProjectionCertificate {
    /// Returns the number of material rings projected.
    pub const fn material_ring_count(&self) -> usize {
        self.material_ring_count
    }

    /// Returns the number of hole rings projected.
    pub const fn hole_ring_count(&self) -> usize {
        self.hole_ring_count
    }

    /// Returns the total number of exact source segments traversed.
    pub const fn source_segment_count(&self) -> usize {
        self.source_segment_count
    }

    /// Returns the total number of exact source line segments traversed.
    pub const fn line_segment_count(&self) -> usize {
        self.line_segment_count
    }

    /// Returns the total number of exact source circular arcs traversed.
    pub const fn arc_segment_count(&self) -> usize {
        self.arc_segment_count
    }

    /// Returns the total number of finite vertices emitted across all rings.
    pub const fn emitted_point_count(&self) -> usize {
        self.emitted_point_count
    }

    /// Returns the total number of finite vertices emitted while sampling arcs.
    pub const fn emitted_arc_sample_count(&self) -> usize {
        self.emitted_arc_sample_count
    }

    /// Returns the requested circular-arc chord-error budget.
    pub const fn arc_chord_error(&self) -> f64 {
        self.arc_chord_error
    }
}

impl FiniteRegionProjection2 {
    fn new(
        material_rings: Vec<FinitePolyline2>,
        hole_rings: Vec<FinitePolyline2>,
        certificate: FiniteRegionProjectionCertificate,
    ) -> Self {
        Self {
            material_rings,
            hole_rings,
            certificate,
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

    /// Returns the aggregate projection certificate.
    pub const fn certificate(&self) -> &FiniteRegionProjectionCertificate {
        &self.certificate
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
    /// formula. Exact CAD area should use [`Region2::filled_area`]; this helper
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
}

/// Returns the finite signed shoelace area of projected ring vertices.
///
/// The closing edge is included even when the caller did not repeat the first
/// vertex. This is the familiar Green's-theorem polygon formula applied only
/// to finite boundary data; exact CAD area should use native contour/region
/// area APIs instead. The boundary split follows Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7(1-2), 1997
/// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
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

impl Region2 {
    /// Projects this region to finite material/hole rings for IO and display.
    ///
    /// Region roles are preserved, but the returned rings are boundary
    /// products only. Exact point classification and area should continue to
    /// use [`Region2::classify_point`] and [`Region2::filled_area_report`].
    pub fn project_to_finite_region(
        &self,
        options: &FiniteProjectionOptions,
    ) -> CurveResult<FiniteRegionProjection2> {
        self.as_view().project_to_finite_region(options)
    }

    /// Projects exact material/hole ownership profiles to finite rings.
    ///
    /// Ownership is classified before projection with
    /// [`Region2::contour_profiles`], so this method does not recover holes
    /// from sampled centroids or winding heuristics. The returned rings are
    /// still finite API-boundary products; exact topology remains in the
    /// region. This follows Yap's exact-object/API-boundary split and the
    /// boundary-first point-in-polygon structure surveyed by Hormann and
    /// Agathos, both cited on [`Region2::contour_profiles`].
    pub fn project_to_finite_profiles(
        &self,
        options: &FiniteProjectionOptions,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Vec<FiniteRegionProfile2>>> {
        self.as_view().project_to_finite_profiles(options, policy)
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
        let certificate = finite_region_certificate(&material_rings, &hole_rings, options);
        Ok(FiniteRegionProjection2::new(
            material_rings,
            hole_rings,
            certificate,
        ))
    }

    /// Projects exact material/hole ownership profiles to finite rings.
    pub fn project_to_finite_profiles(
        &self,
        options: &FiniteProjectionOptions,
        policy: &CurvePolicy,
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

fn finite_region_certificate(
    material_rings: &[FinitePolyline2],
    hole_rings: &[FinitePolyline2],
    options: &FiniteProjectionOptions,
) -> FiniteRegionProjectionCertificate {
    let mut certificate = FiniteRegionProjectionCertificate {
        material_ring_count: material_rings.len(),
        hole_ring_count: hole_rings.len(),
        source_segment_count: 0,
        line_segment_count: 0,
        arc_segment_count: 0,
        emitted_point_count: 0,
        emitted_arc_sample_count: 0,
        arc_chord_error: options.arc_chord_error,
    };
    for ring in material_rings.iter().chain(hole_rings) {
        let ring_certificate = ring.certificate();
        certificate.source_segment_count += ring_certificate.source_segment_count();
        certificate.line_segment_count += ring_certificate.line_segment_count();
        certificate.arc_segment_count += ring_certificate.arc_segment_count();
        certificate.emitted_point_count += ring_certificate.emitted_point_count();
        certificate.emitted_arc_sample_count += ring_certificate.emitted_arc_sample_count();
    }
    certificate
}

fn project_curve_string(
    curve: &CurveString2,
    options: &FiniteProjectionOptions,
    close: bool,
) -> CurveResult<FinitePolyline2> {
    let first = curve.start().ok_or(CurveError::EmptyCurveString)?;
    let mut points = Vec::with_capacity(curve.len() + 1);
    push_if_new(&mut points, finite_point(first)?);
    let mut line_segment_count = 0;
    let mut arc_segment_count = 0;
    let mut emitted_arc_sample_count = 0;

    for segment in curve.segments() {
        match segment {
            Segment2::Line(line) => {
                line_segment_count += 1;
                push_if_new(&mut points, finite_point(line.end())?);
            }
            Segment2::Arc(arc) => {
                arc_segment_count += 1;
                emitted_arc_sample_count +=
                    append_arc_samples(&mut points, arc, options.arc_chord_error)?;
            }
        }
    }

    if close {
        close_ring(&mut points);
    }

    let emitted_point_count = points.len();
    let certificate = FiniteProjectionCertificate {
        source_segment_count: curve.len(),
        line_segment_count,
        arc_segment_count,
        emitted_point_count,
        emitted_arc_sample_count,
        arc_chord_error: options.arc_chord_error,
        closed: close,
    };

    Ok(FinitePolyline2::new(points, certificate))
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
        push_if_new(
            points,
            [
                center[0] + radius * angle.cos(),
                center[1] + radius * angle.sin(),
            ],
        );
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
