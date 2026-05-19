//! Exact closed-convex boolean fragments.
//!
//! This module handles certified object-level closed-convex boolean fragments.
//! Each output face is produced by clipping a source face polygon against the
//! other solid's exact oriented halfspaces, then the resulting triangle mesh is
//! revalidated through [`ExactMesh`]. That follows Yap, "Towards Exact
//! Geometric Computation," *Computational Geometry* 7.1-2 (1997): boolean
//! topology is emitted only from retained object facts and proof-producing
//! predicate routes.
//!
//! The clipping pass is the convex-polyhedron specialization of Sutherland and
//! Hodgman, "Reentrant Polygon Clipping," *Communications of the ACM* 17.1
//! (1974). Hypermesh replaces screen-space floating tests with
//! `hyperlimit::orient3d_report` and exact determinant-ratio interpolation.

use std::cmp::Ordering;

use hyperlimit::{PlaneSide, Point3, compare_reals, orient3d_report};

use super::error::{DiagnosticKind, MeshDiagnostic, MeshError, Severity};
use super::mesh::{ExactMesh, ExactPoint3, Triangle};
use super::provenance::SourceProvenance;
use super::scalar::ExactReal;
use super::solid::{
    ClosedMeshOrientation, ConvexSolidFacts, ConvexSolidReportError, certify_convex_solid,
};
use super::validation::ValidationPolicy;

/// Certified intersection of two closed convex solids.
#[derive(Clone, Debug, PartialEq)]
pub struct ConvexSolidIntersection {
    /// Convexity and orientation facts for the left operand.
    pub left_facts: ConvexSolidFacts,
    /// Convexity and orientation facts for the right operand.
    pub right_facts: ConvexSolidFacts,
    /// Exact closed mesh materialized from clipped source-face polygons.
    pub mesh: ExactMesh,
}

impl ConvexSolidIntersection {
    /// Validate retained facts and the materialized mesh.
    pub fn validate(&self) -> Result<(), MeshError> {
        self.left_facts.validate().map_err(report_error)?;
        self.right_facts.validate().map_err(report_error)?;
        if !self.left_facts.is_certified_convex() || !self.right_facts.is_certified_convex() {
            return Err(MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::UnsupportedExactOperation,
                "convex intersection retained non-certified solid facts",
            )));
        }
        self.mesh.validate_retained_state().map_err(|error| {
            MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::UnsupportedExactOperation,
                format!("convex intersection output failed retained-state replay: {error:?}"),
            ))
        })
    }

    /// Recompute this intersection from the supplied source meshes.
    ///
    /// Yap's exact-computation boundary treats retained artifacts as certified
    /// computation history, not detached meshes. This replay check rejects an
    /// otherwise coherent intersection if its source solids or clipped output
    /// no longer match the operands that produced it.
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), MeshError> {
        self.validate()?;
        let replay = intersect_closed_convex_solids(left, right).ok_or_else(|| {
            MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::UnsupportedExactOperation,
                "convex intersection source replay did not reproduce an output",
            ))
        })?;
        if self == &replay {
            Ok(())
        } else {
            Err(MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::UnsupportedExactOperation,
                "convex intersection retained output no longer matches source replay",
            )))
        }
    }
}

/// Certified difference of closed convex solids for a single cap.
///
/// This is intentionally not a general convex difference implementation.
/// Difference of two convex solids can be nonconvex or holed, so emitting a
/// mesh without winding/cell decomposition would violate Yap's exact geometric
/// computation boundary. The supported case is narrower and auditable: every
/// left vertex is inside all but one right halfspace, the remaining right
/// halfspace cuts the left solid in exactly one cap loop, and the retained
/// output is the closed convex solid on the outside of that one certified
/// plane. This is the 3D analogue of a one-corner clipped polygon fragment,
/// with the clipping idea following Sutherland-Hodgman while all signs and
/// interpolation ratios remain exact.
#[derive(Clone, Debug, PartialEq)]
pub struct ConvexSolidSingleCapDifference {
    /// Convexity and orientation facts for the left operand.
    pub left_facts: ConvexSolidFacts,
    /// Convexity and orientation facts for the right operand.
    pub right_facts: ConvexSolidFacts,
    /// Index of the right face whose oriented plane produced the cap.
    pub cutting_face: usize,
    /// Exact closed mesh materialized from the left solid and triangular cap.
    pub mesh: ExactMesh,
}

impl ConvexSolidSingleCapDifference {
    /// Validate retained facts and the materialized mesh.
    pub fn validate(&self) -> Result<(), MeshError> {
        self.left_facts.validate().map_err(report_error)?;
        self.right_facts.validate().map_err(report_error)?;
        if !self.left_facts.is_certified_convex() || !self.right_facts.is_certified_convex() {
            return Err(MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::UnsupportedExactOperation,
                "convex difference retained non-certified solid facts",
            )));
        }
        self.mesh.validate_retained_state().map_err(|error| {
            MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::UnsupportedExactOperation,
                format!("convex difference output failed retained-state replay: {error:?}"),
            ))
        })
    }

    /// Recompute this single-cap difference from the supplied source meshes.
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), MeshError> {
        self.validate()?;
        let replay = subtract_closed_convex_solids_single_cap(left, right).ok_or_else(|| {
            MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::UnsupportedExactOperation,
                "convex single-cap difference source replay did not reproduce an output",
            ))
        })?;
        if self == &replay {
            Ok(())
        } else {
            Err(MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::UnsupportedExactOperation,
                "convex single-cap difference retained output no longer matches source replay",
            )))
        }
    }
}

/// Certify and materialize the intersection of two closed convex solids.
///
/// Returns `None` unless both operands certify as closed convex solids and the
/// clipped output revalidates as a closed exact triangle mesh. It does not
/// approximate winding, and it does not claim union/difference support for
/// partial overlaps.
pub fn intersect_closed_convex_solids(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<ConvexSolidIntersection> {
    let left_facts = certify_convex_solid(left);
    let right_facts = certify_convex_solid(right);
    if !left_facts.is_certified_convex() || !right_facts.is_certified_convex() {
        return None;
    }

    let mut polygons = Vec::new();
    clipped_source_faces(left, right, &right_facts, ClipKeep::Inside, &mut polygons)?;
    clipped_source_faces(right, left, &left_facts, ClipKeep::Inside, &mut polygons)?;
    if polygons.is_empty() {
        return None;
    }

    let mesh = polygons_to_closed_mesh(
        &polygons,
        "exact closed-convex solid intersection",
        ValidationPolicy::CLOSED,
    )?;
    let intersection = ConvexSolidIntersection {
        left_facts,
        right_facts,
        mesh,
    };
    intersection.validate().ok()?;
    Some(intersection)
}

/// Certify and materialize `left - right` for one convex cap.
///
/// Returns `None` unless both operands are closed convex solids and the right
/// operand removes exactly one cap from the left operand. The cap boundary is
/// replayed from clipped source-face edges on the cutting plane rather than
/// inferred from approximate coordinates, preserving Yap's retained
/// computation-history discipline. General convex difference remains a
/// planar-cell/winding problem because the output can be nonconvex or contain
/// holes.
pub fn subtract_closed_convex_solids_single_cap(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<ConvexSolidSingleCapDifference> {
    let left_facts = certify_convex_solid(left);
    let right_facts = certify_convex_solid(right);
    if !left_facts.is_certified_convex() || !right_facts.is_certified_convex() {
        return None;
    }

    let cutting_face = single_active_cutting_face(left, right, &right_facts)?;
    let face = right.triangles().get(cutting_face)?.0;
    let mut polygons = Vec::new();
    clipped_source_faces_by_face(
        left,
        right,
        face,
        &right_facts,
        ClipKeep::Outside,
        &mut polygons,
    )?;
    let cap = cap_polygon_from_clipped_faces(&polygons, right, face)?;
    let cap_start = polygons.len();
    polygons.extend(cap_polygons(&cap)?);
    let mesh = polygons_to_closed_mesh(
        &polygons,
        "exact closed-convex single-cap difference",
        ValidationPolicy::CLOSED,
    )
    .or_else(|| {
        let mut reversed = polygons;
        for polygon in &mut reversed[cap_start..] {
            polygon.reverse();
        }
        polygons_to_closed_mesh(
            &reversed,
            "exact closed-convex single-cap difference",
            ValidationPolicy::CLOSED,
        )
    })?;

    let difference = ConvexSolidSingleCapDifference {
        left_facts,
        right_facts,
        cutting_face,
        mesh,
    };
    difference.validate().ok()?;
    Some(difference)
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum ClipKeep {
    Inside,
    Outside,
}

fn clipped_source_faces(
    source: &ExactMesh,
    clip: &ExactMesh,
    clip_facts: &ConvexSolidFacts,
    keep: ClipKeep,
    polygons: &mut Vec<Vec<Point3>>,
) -> Option<()> {
    for triangle in source.triangles() {
        let mut polygon = triangle
            .0
            .iter()
            .map(|&index| source.vertices()[index].to_hyperlimit_point())
            .collect::<Vec<_>>();
        for clip_triangle in clip.triangles() {
            polygon = clip_polygon_by_face(&polygon, clip, clip_triangle.0, clip_facts, keep)?;
            if polygon.len() < 3 {
                break;
            }
        }
        simplify_polygon(&mut polygon);
        if polygon.len() >= 3 && !polygon_is_degenerate(&polygon) {
            polygons.push(polygon);
        }
    }
    Some(())
}

fn clipped_source_faces_by_face(
    source: &ExactMesh,
    clip: &ExactMesh,
    face: [usize; 3],
    clip_facts: &ConvexSolidFacts,
    keep: ClipKeep,
    polygons: &mut Vec<Vec<Point3>>,
) -> Option<()> {
    for triangle in source.triangles() {
        let mut polygon = triangle
            .0
            .iter()
            .map(|&index| source.vertices()[index].to_hyperlimit_point())
            .collect::<Vec<_>>();
        polygon = clip_polygon_by_face(&polygon, clip, face, clip_facts, keep)?;
        simplify_polygon(&mut polygon);
        if polygon.len() >= 3 && !polygon_is_degenerate(&polygon) {
            polygons.push(polygon);
        }
    }
    Some(())
}

fn clip_polygon_by_face(
    polygon: &[Point3],
    clip: &ExactMesh,
    face: [usize; 3],
    clip_facts: &ConvexSolidFacts,
    keep: ClipKeep,
) -> Option<Vec<Point3>> {
    if polygon.is_empty() {
        return Some(Vec::new());
    }
    let a = clip.vertices()[face[0]].to_hyperlimit_point();
    let b = clip.vertices()[face[1]].to_hyperlimit_point();
    let c = clip.vertices()[face[2]].to_hyperlimit_point();

    let mut output = Vec::new();
    let mut previous = polygon.last()?.clone();
    let mut previous_inside = keep_side(
        clip_facts.orientation,
        point_side(&a, &b, &c, &previous)?,
        keep,
    );
    for current in polygon {
        let current_inside = keep_side(
            clip_facts.orientation,
            point_side(&a, &b, &c, current)?,
            keep,
        );
        match (previous_inside, current_inside) {
            (true, true) => output.push(current.clone()),
            (true, false) => {
                output.push(intersect_segment_with_plane(
                    &previous, current, &a, &b, &c,
                )?);
            }
            (false, true) => {
                output.push(intersect_segment_with_plane(
                    &previous, current, &a, &b, &c,
                )?);
                output.push(current.clone());
            }
            (false, false) => {}
        }
        previous = current.clone();
        previous_inside = current_inside;
    }
    simplify_polygon(&mut output);
    Some(output)
}

fn keep_side(orientation: ClosedMeshOrientation, side: PlaneSide, keep: ClipKeep) -> bool {
    side == PlaneSide::On
        || match keep {
            ClipKeep::Inside => !side_is_outside(orientation, side),
            ClipKeep::Outside => side_is_outside(orientation, side),
        }
}

fn single_active_cutting_face(
    left: &ExactMesh,
    right: &ExactMesh,
    right_facts: &ConvexSolidFacts,
) -> Option<usize> {
    let mut active = None;
    for (face_index, triangle) in right.triangles().iter().enumerate() {
        let a = right.vertices()[triangle.0[0]].to_hyperlimit_point();
        let b = right.vertices()[triangle.0[1]].to_hyperlimit_point();
        let c = right.vertices()[triangle.0[2]].to_hyperlimit_point();
        let mut outside = 0;
        let mut inside = 0;
        for vertex in left.vertices() {
            let side = point_side(&a, &b, &c, &vertex.to_hyperlimit_point())?;
            if side_is_outside(right_facts.orientation, side) {
                outside += 1;
            } else {
                inside += 1;
            }
        }
        match (outside, inside) {
            (0, _) => {}
            (_, 0) => return None,
            (_, _) if active.replace(face_index).is_some() => return None,
            (_, _) => {}
        }
    }
    active
}

fn cap_polygon_from_clipped_faces(
    polygons: &[Vec<Point3>],
    clip: &ExactMesh,
    face: [usize; 3],
) -> Option<Vec<Point3>> {
    let a = clip.vertices()[face[0]].to_hyperlimit_point();
    let b = clip.vertices()[face[1]].to_hyperlimit_point();
    let c = clip.vertices()[face[2]].to_hyperlimit_point();
    let mut segments = Vec::new();
    for polygon in polygons {
        if polygon.len() < 2 {
            continue;
        }
        for index in 0..polygon.len() {
            let start = &polygon[index];
            let end = &polygon[(index + 1) % polygon.len()];
            if points_equal(start, end) {
                continue;
            }
            if point_side(&a, &b, &c, start)? == PlaneSide::On
                && point_side(&a, &b, &c, end)? == PlaneSide::On
            {
                push_unique_segment(&mut segments, [start.clone(), end.clone()]);
            }
        }
    }
    chain_segments_to_polygon(segments)
}

/// Triangulate one exact cap loop while preserving its boundary subdivision.
///
/// Clipped source faces may split a geometric cap edge at source-triangle
/// diagonals. Collapsing those collinear points creates T-junctions, so the
/// cap is triangulated through an exact rational centroid and every retained
/// boundary segment is emitted as a triangle edge. This follows Yap's retained
/// numerical structural information principle from "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997): the cap topology is
/// built from replayable exact construction facts, not a simplified float view.
fn cap_polygons(cap: &[Point3]) -> Option<Vec<Vec<Point3>>> {
    if cap.len() < 3 {
        return None;
    }
    if cap.len() == 3 {
        return Some(vec![cap.to_vec()]);
    }

    let center = polygon_centroid(cap)?;
    let mut triangles = Vec::with_capacity(cap.len());
    for index in 0..cap.len() {
        let triangle = vec![
            center.clone(),
            cap[index].clone(),
            cap[(index + 1) % cap.len()].clone(),
        ];
        if polygon_is_degenerate(&triangle) {
            return None;
        }
        triangles.push(triangle);
    }
    Some(triangles)
}

fn polygon_centroid(points: &[Point3]) -> Option<Point3> {
    let count = ExactReal::from(i64::try_from(points.len()).ok()?);
    let mut x = ExactReal::from(0);
    let mut y = ExactReal::from(0);
    let mut z = ExactReal::from(0);
    for point in points {
        x = add(&x, &point.x);
        y = add(&y, &point.y);
        z = add(&z, &point.z);
    }
    Some(Point3::new(
        (x / &count).ok()?,
        (y / &count).ok()?,
        (z / &count).ok()?,
    ))
}

fn push_unique_segment(segments: &mut Vec<[Point3; 2]>, segment: [Point3; 2]) {
    if segments.iter().any(|existing| {
        (points_equal(&existing[0], &segment[0]) && points_equal(&existing[1], &segment[1]))
            || (points_equal(&existing[0], &segment[1]) && points_equal(&existing[1], &segment[0]))
    }) {
        return;
    }
    segments.push(segment);
}

fn chain_segments_to_polygon(mut segments: Vec<[Point3; 2]>) -> Option<Vec<Point3>> {
    let first = segments.pop()?;
    let mut polygon = vec![first[0].clone(), first[1].clone()];
    while !segments.is_empty() {
        let last = polygon.last()?.clone();
        if points_equal(&last, &polygon[0]) {
            break;
        }
        let (index, reverse) = segments.iter().enumerate().find_map(|(index, segment)| {
            if points_equal(&segment[0], &last) {
                Some((index, false))
            } else if points_equal(&segment[1], &last) {
                Some((index, true))
            } else {
                None
            }
        })?;
        let segment = segments.swap_remove(index);
        polygon.push(if reverse {
            segment[0].clone()
        } else {
            segment[1].clone()
        });
    }
    if !segments.is_empty() || !points_equal(polygon.first()?, polygon.last()?) {
        return None;
    }
    polygon.pop();
    simplify_polygon(&mut polygon);
    if polygon.len() >= 3 && !polygon_is_degenerate(&polygon) {
        Some(polygon)
    } else {
        None
    }
}

fn point_side(a: &Point3, b: &Point3, c: &Point3, point: &Point3) -> Option<PlaneSide> {
    orient3d_report(a, b, c, point).value().map(PlaneSide::from)
}

fn intersect_segment_with_plane(
    p0: &Point3,
    p1: &Point3,
    a: &Point3,
    b: &Point3,
    c: &Point3,
) -> Option<Point3> {
    let d0 = orient3d_value(a, b, c, p0);
    let d1 = orient3d_value(a, b, c, p1);
    let denominator = sub(&d0, &d1);
    if compare_reals(&denominator, &ExactReal::from(0)).value() == Some(Ordering::Equal) {
        return None;
    }
    let t = (d0 / &denominator).ok()?;
    Some(interpolate3(p0, p1, &t))
}

fn orient3d_value(a: &Point3, b: &Point3, c: &Point3, d: &Point3) -> ExactReal {
    let adx = sub(&a.x, &d.x);
    let ady = sub(&a.y, &d.y);
    let adz = sub(&a.z, &d.z);
    let bdx = sub(&b.x, &d.x);
    let bdy = sub(&b.y, &d.y);
    let bdz = sub(&b.z, &d.z);
    let cdx = sub(&c.x, &d.x);
    let cdy = sub(&c.y, &d.y);
    let cdz = sub(&c.z, &d.z);

    let minor_z = sub(&mul(&bdx, &cdy), &mul(&cdx, &bdy));
    let minor_y = sub(&mul(&cdx, &ady), &mul(&adx, &cdy));
    let minor_x = sub(&mul(&adx, &bdy), &mul(&bdx, &ady));

    add(
        &add(&mul(&adz, &minor_z), &mul(&bdz, &minor_y)),
        &mul(&cdz, &minor_x),
    )
}

fn polygons_to_closed_mesh(
    polygons: &[Vec<Point3>],
    label: &str,
    validation: ValidationPolicy,
) -> Option<ExactMesh> {
    let mut vertices: Vec<ExactPoint3> = Vec::new();
    let mut triangles = Vec::new();
    for polygon in polygons {
        let base = intern_point(&mut vertices, &polygon[0]);
        for index in 1..polygon.len() - 1 {
            let b = intern_point(&mut vertices, &polygon[index]);
            let c = intern_point(&mut vertices, &polygon[index + 1]);
            if base != b && b != c && c != base {
                triangles.push(Triangle([base, b, c]));
            }
        }
    }
    if triangles.is_empty() {
        return None;
    }
    ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact(label),
        validation,
    )
    .ok()
}

fn intern_point(vertices: &mut Vec<ExactPoint3>, point: &Point3) -> usize {
    if let Some(index) = vertices
        .iter()
        .position(|existing| point_equals_exact(existing, point))
    {
        index
    } else {
        vertices.push(ExactPoint3::new(
            point.x.clone(),
            point.y.clone(),
            point.z.clone(),
        ));
        vertices.len() - 1
    }
}

fn point_equals_exact(left: &ExactPoint3, right: &Point3) -> bool {
    let coordinates = left.coordinates();
    compare_reals(&coordinates.0[0], &right.x).value() == Some(Ordering::Equal)
        && compare_reals(&coordinates.0[1], &right.y).value() == Some(Ordering::Equal)
        && compare_reals(&coordinates.0[2], &right.z).value() == Some(Ordering::Equal)
}

fn simplify_polygon(points: &mut Vec<Point3>) {
    points.dedup_by(|right, left| points_equal(left, right));
    if points.len() > 1 && points_equal(points.first().unwrap(), points.last().unwrap()) {
        points.pop();
    }
}

fn polygon_is_degenerate(points: &[Point3]) -> bool {
    if points.len() < 3 {
        return true;
    }
    let anchor = &points[0];
    for i in 1..points.len() - 1 {
        if !points_are_collinear(anchor, &points[i], &points[i + 1]) {
            return false;
        }
    }
    true
}

fn points_are_collinear(a: &Point3, b: &Point3, c: &Point3) -> bool {
    let abx = sub(&b.x, &a.x);
    let aby = sub(&b.y, &a.y);
    let abz = sub(&b.z, &a.z);
    let acx = sub(&c.x, &a.x);
    let acy = sub(&c.y, &a.y);
    let acz = sub(&c.z, &a.z);
    let cross_x = sub(&mul(&aby, &acz), &mul(&abz, &acy));
    let cross_y = sub(&mul(&abz, &acx), &mul(&abx, &acz));
    let cross_z = sub(&mul(&abx, &acy), &mul(&aby, &acx));
    compare_reals(&cross_x, &ExactReal::from(0)).value() == Some(Ordering::Equal)
        && compare_reals(&cross_y, &ExactReal::from(0)).value() == Some(Ordering::Equal)
        && compare_reals(&cross_z, &ExactReal::from(0)).value() == Some(Ordering::Equal)
}

fn side_is_outside(orientation: ClosedMeshOrientation, side: PlaneSide) -> bool {
    matches!(
        (orientation, side),
        (ClosedMeshOrientation::Positive, PlaneSide::Below)
            | (ClosedMeshOrientation::Negative, PlaneSide::Above)
    )
}

fn interpolate3(p0: &Point3, p1: &Point3, t: &ExactReal) -> Point3 {
    Point3::new(
        add(&p0.x, &mul(t, &sub(&p1.x, &p0.x))),
        add(&p0.y, &mul(t, &sub(&p1.y, &p0.y))),
        add(&p0.z, &mul(t, &sub(&p1.z, &p0.z))),
    )
}

fn points_equal(left: &Point3, right: &Point3) -> bool {
    compare_reals(&left.x, &right.x).value() == Some(Ordering::Equal)
        && compare_reals(&left.y, &right.y).value() == Some(Ordering::Equal)
        && compare_reals(&left.z, &right.z).value() == Some(Ordering::Equal)
}

fn add(left: &ExactReal, right: &ExactReal) -> ExactReal {
    left.clone() + right
}

fn sub(left: &ExactReal, right: &ExactReal) -> ExactReal {
    left.clone() - right
}

fn mul(left: &ExactReal, right: &ExactReal) -> ExactReal {
    left.clone() * right
}

fn report_error(error: ConvexSolidReportError) -> MeshError {
    MeshError::one(MeshDiagnostic::new(
        Severity::Error,
        DiagnosticKind::UnsupportedExactOperation,
        format!("invalid convex-solid facts retained by intersection: {error:?}"),
    ))
}
