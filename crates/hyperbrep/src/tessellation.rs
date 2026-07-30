//! Optional exact planar-face triangulation and explicit chordal output.
//!
//! This module derives an index mesh from a validated BREP face without
//! changing or replacing the source model. Line-bounded planar faces can be
//! triangulated exactly. Any explicitly trimmed face has a separate,
//! unmistakably lossy chordal API: its policy controls parameter-space
//! subdivision, every emitted vertex is an exact [`Real`] evaluation of the
//! source surface, and triangle interiors carry no invented error guarantee.

use std::collections::HashMap;
use std::fmt;
use std::num::NonZeroUsize;

use hyperlimit::{PredicateOutcome, compare_reals};

use crate::{FaceId, GeometryError, Model, Orientation, Point2, Point3, Real, SurfaceKind};

/// Exact derived triangulation of one line-bounded planar face.
#[derive(Clone, Debug)]
pub struct ExactPlanarFaceTriangulation {
    parameters: Vec<Point2>,
    points: Vec<Point3>,
    triangles: Vec<[usize; 3]>,
}

impl ExactPlanarFaceTriangulation {
    /// Returns face-local exact parameter vertices.
    pub fn parameters(&self) -> &[Point2] {
        &self.parameters
    }

    /// Returns exact model-space images of [`Self::parameters`].
    pub fn points(&self) -> &[Point3] {
        &self.points
    }

    /// Returns oriented triangle indices into both vertex arrays.
    pub fn triangles(&self) -> &[[usize; 3]] {
        &self.triangles
    }
}

/// Explicit parameter-space sampling policy for lossy chordal output.
///
/// Each oriented boundary pcurve is divided into
/// [`Self::boundary_segments`] equal exact parameter intervals. HyperTRI then
/// triangulates that sampled parameter polygon, after which every triangle is
/// split into four by exact parameter midpoints for each
/// [`Self::interior_refinement_levels`] level.
///
/// A seam-free complete sphere has no boundary pcurve. For that family,
/// `boundary_segments` instead selects that many cells per angular quadrant
/// and twice as many latitude bands; each refinement level doubles both
/// counts. The resulting seam exists only in the independent artifact.
///
/// This policy deliberately contains no geometric tolerance: until a
/// surface-specific enclosure proves a chord-error bound, such a tolerance
/// would be a request rather than a certificate.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub struct ChordalApproximationPolicy {
    boundary_segments: NonZeroUsize,
    interior_refinement_levels: u8,
}

impl ChordalApproximationPolicy {
    /// Constructs an explicit uniform parameter-space subdivision policy.
    pub const fn uniform(boundary_segments: NonZeroUsize, interior_refinement_levels: u8) -> Self {
        Self {
            boundary_segments,
            interior_refinement_levels,
        }
    }

    /// Returns boundary intervals per use, or cells per spherical quadrant.
    pub const fn boundary_segments(self) -> NonZeroUsize {
        self.boundary_segments
    }

    /// Returns the number of four-way exact-midpoint triangle refinements.
    pub const fn interior_refinement_levels(self) -> u8 {
        self.interior_refinement_levels
    }
}

/// Certified relationship between a chordal artifact and its source face.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum ChordalSourceRelation {
    /// Every vertex is an exact surface image of its retained exact parameter.
    ///
    /// Triangle edges and interiors are lossy model-space chords. No Hausdorff,
    /// normal, or curvature error bound is claimed.
    ExactAtVerticesOnly,
}

/// Lossy chordal approximation derived from one explicitly trimmed face.
///
/// The source BREP remains authoritative and shares no mutable storage with
/// this artifact. `parameters[index]` is retained for every `points[index]`,
/// making the exact-at-vertices relation replayable.
#[derive(Clone, Debug)]
pub struct ChordalFaceApproximation {
    source_face: FaceId,
    source_surface_kind: SurfaceKind,
    policy: ChordalApproximationPolicy,
    parameters: Vec<Point2>,
    points: Vec<Point3>,
    triangles: Vec<[usize; 3]>,
}

impl ChordalFaceApproximation {
    /// Returns the face from which this independent artifact was derived.
    pub const fn source_face(&self) -> FaceId {
        self.source_face
    }

    /// Returns the exact source-surface family.
    pub const fn source_surface_kind(&self) -> SurfaceKind {
        self.source_surface_kind
    }

    /// Returns the explicit policy used to derive this artifact.
    pub const fn policy(&self) -> ChordalApproximationPolicy {
        self.policy
    }

    /// Returns the exact relation certified by construction.
    pub const fn source_relation(&self) -> ChordalSourceRelation {
        ChordalSourceRelation::ExactAtVerticesOnly
    }

    /// Returns the retained exact face parameters.
    pub fn parameters(&self) -> &[Point2] {
        &self.parameters
    }

    /// Returns exact source-surface evaluations of [`Self::parameters`].
    pub fn points(&self) -> &[Point3] {
        &self.points
    }

    /// Returns oriented chordal triangle indices into both vertex arrays.
    pub fn triangles(&self) -> &[[usize; 3]] {
        &self.triangles
    }
}

/// Failure to derive an exact or explicitly chordal face artifact.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum TessellationError {
    /// The model does not contain the requested face.
    InvalidFace(FaceId),
    /// The face is not supported by a planar exact tessellation.
    UnsupportedSurface(SurfaceKind),
    /// A boundary is not composed exclusively of exact line pcurves.
    CurvedBoundary,
    /// A boundaryless surface lacks a supported artifact-local parameter mesh.
    MissingFiniteBoundary,
    /// The requested uniform refinement exceeds addressable index storage.
    RefinementOverflow,
    /// HyperTRI removed a sampled parameter-boundary vertex that could not be
    /// reinserted into the derived boundary topology.
    BoundarySampleNotRetained,
    /// Exact surface evaluation or a topology predicate failed.
    Geometry(GeometryError),
    /// HyperTRI rejected the exact polygon input or could not certify an ear.
    Triangulation(hypertri::Error),
}

impl fmt::Display for TessellationError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidFace(face) => write!(formatter, "invalid face ID {}", face.index()),
            Self::UnsupportedSurface(kind) => {
                write!(
                    formatter,
                    "exact face tessellation does not support {kind:?}"
                )
            }
            Self::CurvedBoundary => {
                formatter.write_str("exact planar tessellation requires line pcurves")
            }
            Self::MissingFiniteBoundary => formatter.write_str(
                "boundaryless face lacks a supported artifact-local parameter tessellation",
            ),
            Self::RefinementOverflow => {
                formatter.write_str("chordal refinement exceeds addressable index storage")
            }
            Self::BoundarySampleNotRetained => {
                formatter.write_str("sampled chordal boundary vertex was not retained")
            }
            Self::Geometry(error) => error.fmt(formatter),
            Self::Triangulation(error) => error.fmt(formatter),
        }
    }
}

impl std::error::Error for TessellationError {}

impl From<GeometryError> for TessellationError {
    fn from(value: GeometryError) -> Self {
        Self::Geometry(value)
    }
}

impl From<hypertri::Error> for TessellationError {
    fn from(value: hypertri::Error) -> Self {
        Self::Triangulation(value)
    }
}

/// Triangulates one validated line-bounded planar face exactly.
///
/// The returned mesh is a derived value. It shares no mutable storage with the
/// source model, preserves every exact parameter/model-space vertex, and
/// orients triangles consistently with the BREP face orientation.
pub fn triangulate_planar_face(
    model: &Model,
    face_id: FaceId,
) -> Result<ExactPlanarFaceTriangulation, TessellationError> {
    let face = model
        .face(face_id)
        .ok_or(TessellationError::InvalidFace(face_id))?;
    let surface = model
        .surface(face.surface())
        .expect("validated face surface ID");
    if surface.kind() != SurfaceKind::Plane {
        return Err(TessellationError::UnsupportedSurface(surface.kind()));
    }

    let (parameters, hole_indices) = sampled_boundaries(model, face_id, 1, true)?;

    let hypertri_points = parameters
        .iter()
        .map(|point| hypertri::Point2::new(point.x.clone(), point.y.clone()))
        .collect::<Vec<_>>();
    let flat_triangles = hypertri::earcut(&hypertri_points, &hole_indices)?;
    let mut triangles = flat_triangles
        .chunks_exact(3)
        .map(|indices| [indices[0], indices[1], indices[2]])
        .collect::<Vec<_>>();
    retain_all_boundary_samples(&parameters, &mut triangles)?;
    for triangle in &mut triangles {
        orient_triangle(&parameters, triangle, face.orientation())?;
    }
    let points = parameters
        .iter()
        .map(|parameter| surface.point_at(parameter))
        .collect::<Result<Vec<_>, _>>()?;
    Ok(ExactPlanarFaceTriangulation {
        parameters,
        points,
        triangles,
    })
}

/// Derives an explicitly lossy chordal approximation of a trimmed face.
///
/// Every supported surface family is accepted when the face has an explicit
/// finite outer boundary. A complete seam-free sphere is also accepted through
/// an artifact-local periodic mesh. Boundary pcurves of every evaluable family
/// are sampled at exact uniform parameters. All boundary and refinement
/// parameters are computed with [`Real`]. Every output point is then evaluated
/// exactly from the immutable source surface.
///
/// Only vertices are certified to lie on the source. Triangle edges and
/// interiors are model-space chords with no geometric error bound.
pub fn approximate_face_chordally(
    model: &Model,
    face_id: FaceId,
    policy: ChordalApproximationPolicy,
) -> Result<ChordalFaceApproximation, TessellationError> {
    let face = model
        .face(face_id)
        .ok_or(TessellationError::InvalidFace(face_id))?;
    let surface = model
        .surface(face.surface())
        .expect("validated face surface ID");
    if face.outer().is_none() && surface.kind() == SurfaceKind::Sphere {
        return approximate_complete_sphere_chordally(face_id, surface, face.orientation(), policy);
    }
    let (mut parameters, hole_indices) =
        sampled_boundaries(model, face_id, policy.boundary_segments.get(), false)?;
    let hypertri_points = parameters
        .iter()
        .map(|point| hypertri::Point2::new(point.x.clone(), point.y.clone()))
        .collect::<Vec<_>>();
    let flat_triangles = hypertri::earcut(&hypertri_points, &hole_indices)?;
    let mut triangles = flat_triangles
        .chunks_exact(3)
        .map(|indices| [indices[0], indices[1], indices[2]])
        .collect::<Vec<_>>();
    retain_all_boundary_samples(&parameters, &mut triangles)?;
    for triangle in &mut triangles {
        orient_triangle(&parameters, triangle, face.orientation())?;
    }

    for _ in 0..policy.interior_refinement_levels {
        refine_triangles_at_exact_midpoints(&mut parameters, &mut triangles)?;
    }
    let points = parameters
        .iter()
        .map(|parameter| surface.point_at(parameter))
        .collect::<Result<Vec<_>, _>>()?;

    Ok(ChordalFaceApproximation {
        source_face: face_id,
        source_surface_kind: surface.kind(),
        policy,
        parameters,
        points,
        triangles,
    })
}

fn approximate_complete_sphere_chordally(
    face_id: FaceId,
    surface: &crate::Surface,
    orientation: Orientation,
    policy: ChordalApproximationPolicy,
) -> Result<ChordalFaceApproximation, TessellationError> {
    let mut segments_per_quadrant = policy.boundary_segments.get();
    for _ in 0..policy.interior_refinement_levels {
        segments_per_quadrant = segments_per_quadrant
            .checked_mul(2)
            .ok_or(TessellationError::RefinementOverflow)?;
    }
    let longitude_count = segments_per_quadrant
        .checked_mul(4)
        .ok_or(TessellationError::RefinementOverflow)?;
    let latitude_band_count = segments_per_quadrant
        .checked_mul(2)
        .ok_or(TessellationError::RefinementOverflow)?;
    let ring_count = latitude_band_count - 1;
    let parameter_count = ring_count
        .checked_mul(longitude_count)
        .and_then(|count| count.checked_add(2))
        .ok_or(TessellationError::RefinementOverflow)?;
    let triangle_count = ring_count
        .checked_mul(longitude_count)
        .and_then(|count| count.checked_mul(2))
        .ok_or(TessellationError::RefinementOverflow)?;

    let pi = Real::pi();
    let two_pi = &pi * Real::from(2);
    let half_pi = (pi.clone() / Real::from(2)).map_err(|_| GeometryError::ProjectiveDivision)?;
    let longitude_denominator = Real::from(
        u128::try_from(longitude_count).map_err(|_| TessellationError::RefinementOverflow)?,
    );
    let latitude_denominator = Real::from(
        u128::try_from(latitude_band_count).map_err(|_| TessellationError::RefinementOverflow)?,
    );
    let mut parameters = Vec::with_capacity(parameter_count);
    parameters.push(Point2::new(Real::zero(), -half_pi.clone()));
    for latitude_index in 1..latitude_band_count {
        let latitude_fraction = (Real::from(
            u128::try_from(latitude_index).map_err(|_| TessellationError::RefinementOverflow)?,
        ) / &latitude_denominator)
            .map_err(|_| GeometryError::ProjectiveDivision)?;
        let latitude = -half_pi.clone() + &pi * latitude_fraction;
        for longitude_index in 0..longitude_count {
            let longitude_fraction = (Real::from(
                u128::try_from(longitude_index)
                    .map_err(|_| TessellationError::RefinementOverflow)?,
            ) / &longitude_denominator)
                .map_err(|_| GeometryError::ProjectiveDivision)?;
            parameters.push(Point2::new(&two_pi * longitude_fraction, latitude.clone()));
        }
    }
    parameters.push(Point2::new(Real::zero(), half_pi));

    let south = 0;
    let north = parameters.len() - 1;
    let ring_start = |ring: usize| 1 + ring * longitude_count;
    let mut triangles = Vec::with_capacity(triangle_count);
    for longitude in 0..longitude_count {
        let next = (longitude + 1) % longitude_count;
        triangles.push([south, ring_start(0) + next, ring_start(0) + longitude]);
    }
    for ring in 0..ring_count - 1 {
        let lower = ring_start(ring);
        let upper = ring_start(ring + 1);
        for longitude in 0..longitude_count {
            let next = (longitude + 1) % longitude_count;
            triangles.extend_from_slice(&[
                [lower + longitude, lower + next, upper + next],
                [lower + longitude, upper + next, upper + longitude],
            ]);
        }
    }
    let last_ring = ring_start(ring_count - 1);
    for longitude in 0..longitude_count {
        let next = (longitude + 1) % longitude_count;
        triangles.push([last_ring + longitude, last_ring + next, north]);
    }
    if orientation == Orientation::Reversed {
        for triangle in &mut triangles {
            triangle.swap(1, 2);
        }
    }
    debug_assert_eq!(parameters.len(), parameter_count);
    debug_assert_eq!(triangles.len(), triangle_count);
    let points = parameters
        .iter()
        .map(|parameter| surface.point_at(parameter))
        .collect::<Result<Vec<_>, _>>()?;
    Ok(ChordalFaceApproximation {
        source_face: face_id,
        source_surface_kind: SurfaceKind::Sphere,
        policy,
        parameters,
        points,
        triangles,
    })
}

fn sampled_boundaries(
    model: &Model,
    face_id: FaceId,
    segments_per_use: usize,
    lines_only: bool,
) -> Result<(Vec<Point2>, Vec<usize>), TessellationError> {
    debug_assert!(segments_per_use > 0);
    let face = model
        .face(face_id)
        .ok_or(TessellationError::InvalidFace(face_id))?;
    let mut parameters = Vec::new();
    let mut hole_indices = Vec::with_capacity(face.inner().len());
    let outer = face
        .outer()
        .ok_or(TessellationError::MissingFiniteBoundary)?;
    let denominator = Real::from(
        u128::try_from(segments_per_use).map_err(|_| TessellationError::RefinementOverflow)?,
    );
    for (wire_index, wire_id) in std::iter::once(&outer)
        .chain(face.inner().iter())
        .enumerate()
    {
        if wire_index > 0 {
            hole_indices.push(parameters.len());
        }
        let wire = model.wire(*wire_id).expect("validated face wire ID");
        for edge_use_id in wire.edge_uses() {
            let edge_use = model
                .edge_use(*edge_use_id)
                .expect("validated wire edge-use ID");
            let pcurve = model
                .pcurve(edge_use.pcurve())
                .expect("validated edge-use pcurve ID");
            if lines_only && pcurve.line_segment().is_none() {
                return Err(TessellationError::CurvedBoundary);
            }
            let span = pcurve.domain_end() - pcurve.domain_start();
            for segment in 0..segments_per_use {
                let numerator = Real::from(
                    u128::try_from(segment).map_err(|_| TessellationError::RefinementOverflow)?,
                );
                let fraction =
                    (numerator / &denominator).map_err(|_| GeometryError::ProjectiveDivision)?;
                let parameter = pcurve.domain_start() + &span * fraction;
                parameters.push(pcurve.point_at(&parameter)?);
            }
        }
    }
    Ok((parameters, hole_indices))
}

fn retain_all_boundary_samples(
    parameters: &[Point2],
    triangles: &mut Vec<[usize; 3]>,
) -> Result<(), TessellationError> {
    for sample in 0..parameters.len() {
        if triangles.iter().flatten().any(|index| *index == sample) {
            continue;
        }
        let mut edge_counts = HashMap::<(usize, usize), usize>::new();
        for triangle in triangles.iter() {
            for (first, second) in [
                (triangle[0], triangle[1]),
                (triangle[1], triangle[2]),
                (triangle[2], triangle[0]),
            ] {
                let key = if first < second {
                    (first, second)
                } else {
                    (second, first)
                };
                *edge_counts.entry(key).or_default() += 1;
            }
        }

        let mut insertion = None;
        'triangles: for (triangle_index, triangle) in triangles.iter().enumerate() {
            for edge_index in 0..3 {
                let (first, second, opposite) = match edge_index {
                    0 => (triangle[0], triangle[1], triangle[2]),
                    1 => (triangle[1], triangle[2], triangle[0]),
                    _ => (triangle[2], triangle[0], triangle[1]),
                };
                let key = if first < second {
                    (first, second)
                } else {
                    (second, first)
                };
                if edge_counts.get(&key) == Some(&1)
                    && point_strictly_inside_segment(
                        &parameters[sample],
                        &parameters[first],
                        &parameters[second],
                    )?
                {
                    insertion = Some((triangle_index, first, second, opposite));
                    break 'triangles;
                }
            }
        }
        let Some((triangle_index, first, second, opposite)) = insertion else {
            return Err(TessellationError::BoundarySampleNotRetained);
        };
        triangles[triangle_index] = [first, sample, opposite];
        triangles.push([sample, second, opposite]);
    }
    Ok(())
}

fn point_strictly_inside_segment(
    point: &Point2,
    start: &Point2,
    end: &Point2,
) -> Result<bool, GeometryError> {
    let cross =
        (&point.x - &start.x) * (&end.y - &start.y) - (&point.y - &start.y) * (&end.x - &start.x);
    if decided_comparison(&cross, &Real::zero())? != std::cmp::Ordering::Equal {
        return Ok(false);
    }
    let point_is_start = decided_comparison(&point.x, &start.x)? == std::cmp::Ordering::Equal
        && decided_comparison(&point.y, &start.y)? == std::cmp::Ordering::Equal;
    let point_is_end = decided_comparison(&point.x, &end.x)? == std::cmp::Ordering::Equal
        && decided_comparison(&point.y, &end.y)? == std::cmp::Ordering::Equal;
    if point_is_start || point_is_end {
        return Ok(false);
    }
    Ok(coordinate_between(&point.x, &start.x, &end.x)?
        && coordinate_between(&point.y, &start.y, &end.y)?)
}

fn coordinate_between(value: &Real, first: &Real, second: &Real) -> Result<bool, GeometryError> {
    let order = decided_comparison(first, second)?;
    let (minimum, maximum) = if order == std::cmp::Ordering::Greater {
        (second, first)
    } else {
        (first, second)
    };
    Ok(
        decided_comparison(value, minimum)? != std::cmp::Ordering::Less
            && decided_comparison(value, maximum)? != std::cmp::Ordering::Greater,
    )
}

fn decided_comparison(first: &Real, second: &Real) -> Result<std::cmp::Ordering, GeometryError> {
    match compare_reals(first, second) {
        PredicateOutcome::Decided { value, .. } => Ok(value),
        PredicateOutcome::Unknown { needed, stage } => {
            Err(GeometryError::PredicateUnresolved { needed, stage })
        }
    }
}

fn refine_triangles_at_exact_midpoints(
    parameters: &mut Vec<Point2>,
    triangles: &mut Vec<[usize; 3]>,
) -> Result<(), TessellationError> {
    let refined_capacity = triangles
        .len()
        .checked_mul(4)
        .ok_or(TessellationError::RefinementOverflow)?;
    let mut edge_midpoints = HashMap::new();
    let mut refined = Vec::with_capacity(refined_capacity);
    for &[a, b, c] in triangles.iter() {
        let ab = exact_midpoint_index(parameters, &mut edge_midpoints, a, b)?;
        let bc = exact_midpoint_index(parameters, &mut edge_midpoints, b, c)?;
        let ca = exact_midpoint_index(parameters, &mut edge_midpoints, c, a)?;
        refined.extend_from_slice(&[[a, ab, ca], [ab, b, bc], [ca, bc, c], [ab, bc, ca]]);
    }
    *triangles = refined;
    Ok(())
}

fn exact_midpoint_index(
    parameters: &mut Vec<Point2>,
    edge_midpoints: &mut HashMap<(usize, usize), usize>,
    first: usize,
    second: usize,
) -> Result<usize, TessellationError> {
    let key = if first < second {
        (first, second)
    } else {
        (second, first)
    };
    if let Some(index) = edge_midpoints.get(&key) {
        return Ok(*index);
    }
    let midpoint = Point2::new(
        ((&parameters[first].x + &parameters[second].x) / Real::from(2))
            .map_err(|_| GeometryError::ProjectiveDivision)?,
        ((&parameters[first].y + &parameters[second].y) / Real::from(2))
            .map_err(|_| GeometryError::ProjectiveDivision)?,
    );
    let index = parameters.len();
    parameters
        .len()
        .checked_add(1)
        .ok_or(TessellationError::RefinementOverflow)?;
    parameters.push(midpoint);
    edge_midpoints.insert(key, index);
    Ok(index)
}

fn orient_triangle(
    points: &[Point2],
    triangle: &mut [usize; 3],
    orientation: Orientation,
) -> Result<(), GeometryError> {
    let [first, second, third] = triangle.map(|index| &points[index]);
    let signed_double_area = (&second.x - &first.x) * (&third.y - &first.y)
        - (&second.y - &first.y) * (&third.x - &first.x);
    let order = match compare_reals(&signed_double_area, &Real::zero()) {
        PredicateOutcome::Decided { value, .. } => value,
        PredicateOutcome::Unknown { needed, stage } => {
            return Err(GeometryError::PredicateUnresolved { needed, stage });
        }
    };
    let should_be_positive = orientation == Orientation::Forward;
    if (order == std::cmp::Ordering::Greater) != should_be_positive {
        triangle.swap(1, 2);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use hyperlimit::point3_equal;

    fn r(value: i32) -> Real {
        Real::from(value)
    }

    #[test]
    fn exact_planar_face_tessellation_preserves_holes_and_orientation() {
        let outer = vec![
            Point2::new(r(0), r(0)),
            Point2::new(r(4), r(0)),
            Point2::new(r(4), r(4)),
            Point2::new(r(0), r(4)),
        ];
        let hole = vec![
            Point2::new(r(1), r(1)),
            Point2::new(r(1), r(3)),
            Point2::new(r(3), r(3)),
            Point2::new(r(3), r(1)),
        ];
        let (model, _) = crate::builder::extrude_region(&outer, &[hole], r(0), r(2)).unwrap();
        let top = FaceId::from_index(1).unwrap();
        let mesh = triangulate_planar_face(&model, top).unwrap();
        assert_eq!(mesh.parameters().len(), 8);
        assert_eq!(mesh.triangles().len(), 8);
        for (parameter, point) in mesh.parameters().iter().zip(mesh.points()) {
            assert_eq!(
                point3_equal(
                    point,
                    &Point3::new(parameter.x.clone(), parameter.y.clone(), r(2)),
                )
                .value(),
                Some(true)
            );
        }
        for triangle in mesh.triangles() {
            let [a, b, c] = triangle.map(|index| &mesh.parameters()[index]);
            let signed = (&b.x - &a.x) * (&c.y - &a.y) - (&b.y - &a.y) * (&c.x - &a.x);
            assert_eq!(
                compare_reals(&signed, &Real::zero()).value(),
                Some(std::cmp::Ordering::Greater)
            );
        }

        let bottom = FaceId::from_index(0).unwrap();
        let bottom_mesh = triangulate_planar_face(&model, bottom).unwrap();
        for triangle in bottom_mesh.triangles() {
            let [a, b, c] = triangle.map(|index| &bottom_mesh.parameters()[index]);
            let signed = (&b.x - &a.x) * (&c.y - &a.y) - (&b.y - &a.y) * (&c.x - &a.x);
            assert_eq!(
                compare_reals(&signed, &Real::zero()).value(),
                Some(std::cmp::Ordering::Less)
            );
        }
    }

    #[test]
    fn exact_planar_path_rejects_curved_and_nonplanar_faces() {
        let (model, _) = crate::builder::cylinder(r(2), r(3)).unwrap();
        assert_eq!(
            triangulate_planar_face(&model, FaceId::from_index(0).unwrap()).unwrap_err(),
            TessellationError::CurvedBoundary
        );
        assert_eq!(
            triangulate_planar_face(&model, FaceId::from_index(2).unwrap()).unwrap_err(),
            TessellationError::UnsupportedSurface(SurfaceKind::Cylinder)
        );
    }

    #[test]
    fn chordal_output_is_explicit_and_exact_at_every_vertex() {
        let control_points = vec![
            vec![
                Point3::new(r(0), r(0), r(0)),
                Point3::new(r(1), r(0), r(2)),
                Point3::new(r(2), r(0), r(0)),
            ],
            vec![
                Point3::new(r(0), r(1), r(1)),
                Point3::new(r(1), r(1), r(3)),
                Point3::new(r(2), r(1), r(1)),
            ],
            vec![
                Point3::new(r(0), r(2), r(0)),
                Point3::new(r(1), r(2), r(2)),
                Point3::new(r(2), r(2), r(0)),
            ],
        ];
        let patch = crate::TensorPatch::RationalBezier {
            control_points,
            weights: vec![vec![r(1); 3]; 3],
        };
        let (model, faces) = crate::builder::tensor_patch_shell(vec![patch]).unwrap();
        let source_json = model.to_json().unwrap();
        let policy = ChordalApproximationPolicy::uniform(NonZeroUsize::new(1).unwrap(), 2);
        let artifact = approximate_face_chordally(&model, faces[0], policy).unwrap();

        assert_eq!(artifact.source_face(), faces[0]);
        assert_eq!(artifact.source_surface_kind(), SurfaceKind::RationalBezier);
        assert_eq!(artifact.policy(), policy);
        assert_eq!(
            artifact.source_relation(),
            ChordalSourceRelation::ExactAtVerticesOnly
        );
        assert_eq!(artifact.triangles().len(), 32);
        let face = model.face(faces[0]).unwrap();
        let surface = model.surface(face.surface()).unwrap();
        for (parameter, point) in artifact.parameters().iter().zip(artifact.points()) {
            assert_eq!(
                point3_equal(point, &surface.point_at(parameter).unwrap()).value(),
                Some(true)
            );
        }
        for index in 0..artifact.parameters().len() {
            assert!(
                artifact
                    .triangles()
                    .iter()
                    .flatten()
                    .any(|retained| *retained == index)
            );
        }
        assert_eq!(model.to_json().unwrap(), source_json);
    }

    #[test]
    fn chordal_policy_retains_nurbs_domain_samples() {
        let patch = crate::TensorPatch::Nurbs {
            u_degree: 1,
            v_degree: 1,
            control_points: vec![
                vec![Point3::new(r(0), r(0), r(0)), Point3::new(r(2), r(0), r(1))],
                vec![Point3::new(r(0), r(2), r(1)), Point3::new(r(2), r(2), r(0))],
            ],
            weights: vec![vec![r(1), r(2)], vec![r(1), r(2)]],
            u_knots: vec![r(2), r(2), r(5), r(5)],
            v_knots: vec![r(-3), r(-3), r(1), r(1)],
        };
        let (model, faces) = crate::builder::tensor_patch_shell(vec![patch]).unwrap();
        let policy = ChordalApproximationPolicy::uniform(NonZeroUsize::new(2).unwrap(), 1);
        let artifact = approximate_face_chordally(&model, faces[0], policy).unwrap();
        assert_eq!(artifact.source_surface_kind(), SurfaceKind::Nurbs);
        assert_eq!(artifact.triangles().len(), 24);
        assert!(
            artifact
                .parameters()
                .iter()
                .any(|parameter| parameter.x == r(2) && parameter.y == r(-1))
        );
    }

    #[test]
    fn chordal_output_accepts_curved_trims_and_analytic_surfaces() {
        let policy = ChordalApproximationPolicy::uniform(NonZeroUsize::new(3).unwrap(), 1);
        let (cylinder, _) = crate::builder::cylinder(r(2), r(3)).unwrap();
        for face_index in 0..6 {
            let face_id = FaceId::from_index(face_index).unwrap();
            let artifact = approximate_face_chordally(&cylinder, face_id, policy).unwrap();
            let face = cylinder.face(face_id).unwrap();
            let surface = cylinder.surface(face.surface()).unwrap();
            assert_eq!(artifact.source_surface_kind(), surface.kind());
            assert!(!artifact.triangles().is_empty());
            for (parameter, point) in artifact.parameters().iter().zip(artifact.points()) {
                assert_eq!(
                    point3_equal(point, &surface.point_at(parameter).unwrap()).value(),
                    Some(true)
                );
            }
            for index in 0..artifact.parameters().len() {
                assert!(
                    artifact
                        .triangles()
                        .iter()
                        .flatten()
                        .any(|retained| *retained == index)
                );
            }
        }

        let (torus, _) = crate::builder::torus(r(3), r(1)).unwrap();
        let torus_face = FaceId::from_index(0).unwrap();
        let artifact = approximate_face_chordally(&torus, torus_face, policy).unwrap();
        assert_eq!(artifact.source_surface_kind(), SurfaceKind::Torus);
        assert!(!artifact.triangles().is_empty());
    }

    #[test]
    fn chordal_output_derives_a_seam_without_mutating_a_whole_sphere() {
        let policy = ChordalApproximationPolicy::uniform(NonZeroUsize::new(1).unwrap(), 1);
        let (sphere, _) = crate::builder::sphere(r(2)).unwrap();
        let source_json = sphere.to_json().unwrap();
        let face_id = FaceId::from_index(0).unwrap();
        let artifact = approximate_face_chordally(&sphere, face_id, policy).unwrap();
        assert_eq!(artifact.source_surface_kind(), SurfaceKind::Sphere);
        assert_eq!(artifact.parameters().len(), 26);
        assert_eq!(artifact.triangles().len(), 48);
        let face = sphere.face(face_id).unwrap();
        let surface = sphere.surface(face.surface()).unwrap();
        for (parameter, point) in artifact.parameters().iter().zip(artifact.points()) {
            assert_eq!(
                point3_equal(point, &surface.point_at(parameter).unwrap()).value(),
                Some(true)
            );
        }
        for index in 0..artifact.parameters().len() {
            assert!(
                artifact
                    .triangles()
                    .iter()
                    .flatten()
                    .any(|retained| *retained == index)
            );
        }
        for triangle in artifact.triangles() {
            let [a, b, c] = triangle.map(|index| &artifact.points()[index]);
            let normal = (b - a).cross(&(c - a));
            let radial =
                crate::Vector3::new([&a.x + &b.x + &c.x, &a.y + &b.y + &c.y, &a.z + &b.z + &c.z]);
            assert_eq!(
                compare_reals(&normal.dot(&radial), &Real::zero()).value(),
                Some(std::cmp::Ordering::Greater)
            );
        }
        assert_eq!(sphere.to_json().unwrap(), source_json);
    }

    #[test]
    fn chordal_sphere_refinement_overflow_is_explicit() {
        let policy = ChordalApproximationPolicy::uniform(NonZeroUsize::new(1).unwrap(), u8::MAX);
        let (sphere, _) = crate::builder::sphere(r(2)).unwrap();
        assert_eq!(
            approximate_face_chordally(&sphere, FaceId::from_index(0).unwrap(), policy)
                .unwrap_err(),
            TessellationError::RefinementOverflow
        );
    }
}
