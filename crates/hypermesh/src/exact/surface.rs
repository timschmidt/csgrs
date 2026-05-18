//! Exact certification for lower-dimensional surface special cases.
//!
//! This module keeps sheet/surface shortcuts separate from volumetric convex
//! shortcuts. The certified cases are intentionally narrow: single coplanar
//! triangle containment, positive-area intersection, convex union, and the
//! convex one-corner difference shapes that can be represented as an open
//! triangle mesh. The predicates are the same projected orientation and
//! point-in-triangle facts used by the coplanar overlap classifier, following
//! Yap, "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
//! (1997): topology claims are emitted only when the combinatorial relation is
//! certified, and missing output models such as holed sheets remain explicit.
//!
//! The underlying coplanar test follows the orientation-predicate style of
//! Guigue and Devillers, "Fast and Robust Triangle-Triangle Overlap Test Using
//! Orientation Predicates," *Journal of Graphics Tools* 8.1 (2003), routed
//! through `hyperlimit` by [`crate::exact::coplanar`].

use core::cmp::Ordering;

use hyperlimit::{
    Point2, Point3, Sign, TriangleLocation, compare_reals, orient2d_report, point_on_segment,
};

use super::coplanar::CoplanarTriangleClassification;
use super::coplanar::{CoplanarProjection, CoplanarTriangleRelation, classify_coplanar_triangles};
use super::error::{DiagnosticKind, MeshDiagnostic, MeshError, Severity};
use super::mesh::{ExactMesh, ExactPoint3, Triangle};
use super::narrow::{
    TriangleTriangleClassification, TriangleTriangleRelation, classify_triangle_triangle,
};
use super::provenance::SourceProvenance;
use super::scalar::ExactReal;
use super::validation::ValidationPolicy;

/// Certified containment relation between two single-triangle coplanar sheets.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CoplanarSurfaceContainment {
    /// Every left triangle vertex lies in the closed right triangle.
    LeftInsideRight,
    /// Every right triangle vertex lies in the closed left triangle.
    RightInsideLeft,
}

/// Certification status for single-triangle coplanar containment.
#[derive(Clone, Debug, PartialEq)]
pub enum CoplanarSurfaceContainmentStatus {
    /// At least one input was not exactly one triangle.
    NotSingleTriangle,
    /// The 3D triangle/triangle classifier did not certify coplanar contact.
    NotCoplanar,
    /// The projected coplanar classifier was disjoint or undecided.
    DisjointOrUnknown,
    /// Both triangles contain each other, neither contains the other, or the
    /// case belongs to a stronger same-surface/planar-arrangement path.
    AmbiguousOrIdentical,
    /// Exactly one triangle is certified inside the other.
    Certified(CoplanarSurfaceContainment),
}

impl CoplanarSurfaceContainmentStatus {
    /// Return the certified containment relation, if one was established.
    pub const fn certified(&self) -> Option<CoplanarSurfaceContainment> {
        match self {
            Self::Certified(containment) => Some(*containment),
            _ => None,
        }
    }
}

/// Auditable single-triangle coplanar containment certificate.
#[derive(Clone, Debug, PartialEq)]
pub struct CoplanarSurfaceContainmentReport {
    /// Coarse certification status.
    pub status: CoplanarSurfaceContainmentStatus,
    /// Exact 3D triangle/triangle classification, when the input shape allows
    /// that query.
    pub triangle: Option<TriangleTriangleClassification>,
    /// Projected coplanar classification, when the 3D relation reaches the
    /// coplanar stage.
    pub coplanar: Option<CoplanarTriangleClassification>,
}

impl CoplanarSurfaceContainmentReport {
    /// Return whether every retained predicate route was proof-producing.
    pub fn all_proof_producing(&self) -> bool {
        self.triangle
            .as_ref()
            .is_none_or(TriangleTriangleClassification::all_proof_producing)
            && self
                .coplanar
                .as_ref()
                .is_none_or(CoplanarTriangleClassification::projection_proof_producing)
    }
}

/// Exact positive-area intersection of two single-triangle coplanar sheets.
///
/// The returned mesh is an open triangle mesh representing the polygonal
/// intersection surface. Lower-dimensional contacts are intentionally reported
/// as `None` because triangle meshes cannot encode a pure point or segment
/// result without a separate output channel.
#[derive(Clone, Debug, PartialEq)]
pub struct CoplanarTriangleIntersection {
    /// Projection used by the certified 2D clipping predicates.
    pub projection: CoplanarProjection,
    /// Exact 3D polygon boundary after clipping and simplification.
    pub polygon: Vec<Point3>,
    /// Exact triangulated surface mesh for the polygon.
    pub mesh: ExactMesh,
}

impl CoplanarTriangleIntersection {
    /// Validate the materialized intersection polygon and mesh.
    ///
    /// The constructor already builds this shape through exact clipping, but
    /// the fields are public so callers can inspect, serialize, or transform
    /// the artifact. This method replays the output-side invariants before a
    /// downstream consumer trusts it as topology: polygon vertices must be
    /// exact-distinct, have certified nonzero projected area, and match the
    /// fan-triangulated [`ExactMesh`]. This follows Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7.1-2 (1997): a
    /// constructed geometric object should remain auditable at API handoffs.
    pub fn validate(&self) -> Result<(), MeshError> {
        validate_coplanar_surface_output(
            self.projection,
            &self.polygon,
            &self.mesh,
            "coplanar intersection",
        )
    }
}

/// Exact convex union of two single-triangle coplanar sheets.
///
/// This is deliberately narrower than a full planar arrangement. It is emitted
/// only when the union of the two closed triangles is certified to equal the
/// convex hull of their vertices; nonconvex unions and holed/difference cases
/// remain explicit unsupported topology.
#[derive(Clone, Debug, PartialEq)]
pub struct CoplanarTriangleUnion {
    /// Projection used by the certified 2D hull and coverage predicates.
    pub projection: CoplanarProjection,
    /// Exact 3D convex hull boundary.
    pub polygon: Vec<Point3>,
    /// Exact triangulated surface mesh for the convex union.
    pub mesh: ExactMesh,
}

impl CoplanarTriangleUnion {
    /// Validate the materialized convex-union polygon and mesh.
    ///
    /// The union shortcut is accepted only after exact hull coverage checks,
    /// following Andrew's monotone-chain hull construction and Yap's exact
    /// computation boundary. This method validates the persisted output
    /// artifact itself: exact point distinctness, positive projected area, and
    /// fan mesh consistency.
    pub fn validate(&self) -> Result<(), MeshError> {
        validate_coplanar_surface_output(
            self.projection,
            &self.polygon,
            &self.mesh,
            "coplanar convex union",
        )
    }
}

/// Exact convex difference of two single-triangle coplanar sheets.
///
/// This is emitted only for a strict one-corner cut from the left triangle,
/// where the result is one convex polygon representable as a fan-triangulated
/// open triangle mesh. Cuts that split the surface, create holes, or require a
/// nonconvex boundary remain explicit planar-arrangement work.
#[derive(Clone, Debug, PartialEq)]
pub struct CoplanarTriangleDifference {
    /// Projection used by the certified 2D predicates.
    pub projection: CoplanarProjection,
    /// Exact 3D boundary of `left - right`.
    pub polygon: Vec<Point3>,
    /// Exact triangulated surface mesh for the difference.
    pub mesh: ExactMesh,
}

impl CoplanarTriangleDifference {
    /// Validate the materialized one-corner difference polygon and mesh.
    ///
    /// One-corner difference is a narrowly certified planar-arrangement
    /// fragment: the accepted polygon is justified by exact area conservation.
    /// This output validation keeps that fragment auditable after construction
    /// by checking projected area, exact point distinctness, and mesh fan
    /// consistency before callers reuse the artifact.
    pub fn validate(&self) -> Result<(), MeshError> {
        validate_coplanar_surface_output(
            self.projection,
            &self.polygon,
            &self.mesh,
            "coplanar one-corner difference",
        )
    }
}

/// Certify containment for two single-triangle coplanar sheets.
///
/// This is not a general planar arrangement solver. It only returns a
/// certificate when both meshes contain one triangle, the triangles are
/// certified coplanar, and all vertices of exactly one triangle are certified
/// inside or on the boundary of the other closed triangle. Identical surfaces
/// are left to the stronger same-surface certificate in the boolean layer.
pub fn certify_single_triangle_coplanar_containment(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarSurfaceContainment> {
    certify_single_triangle_coplanar_containment_report(left, right)
        .status
        .certified()
}

/// Certify single-triangle coplanar containment and retain predicate artifacts.
///
/// This report is the auditable form of
/// [`certify_single_triangle_coplanar_containment`]. It keeps the 3D
/// `hyperlimit::orient3d_report`-backed triangle classifier and the projected
/// coplanar classifier beside the collapsed containment status. That matches
/// Yap, "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997): a topology shortcut should expose the certified predicate facts
/// that justified it, and unsupported or ambiguous cases stay explicit.
pub fn certify_single_triangle_coplanar_containment_report(
    left: &ExactMesh,
    right: &ExactMesh,
) -> CoplanarSurfaceContainmentReport {
    if left.triangles().len() != 1 || right.triangles().len() != 1 {
        return CoplanarSurfaceContainmentReport {
            status: CoplanarSurfaceContainmentStatus::NotSingleTriangle,
            triangle: None,
            coplanar: None,
        };
    }

    let points = left
        .vertices()
        .iter()
        .chain(right.vertices())
        .map(|point| point.to_hyperlimit_point())
        .collect::<Vec<_>>();
    let left_tri = left.triangles()[0].0;
    let right_offset = left.vertices().len();
    let right_tri = right.triangles()[0].0.map(|index| index + right_offset);

    let classification = classify_triangle_triangle(&points, left_tri, right_tri);
    if !matches!(
        classification.relation,
        TriangleTriangleRelation::CoplanarTouching | TriangleTriangleRelation::CoplanarOverlapping
    ) {
        return CoplanarSurfaceContainmentReport {
            status: CoplanarSurfaceContainmentStatus::NotCoplanar,
            triangle: Some(classification),
            coplanar: None,
        };
    }

    let coplanar = classify_coplanar_triangles(&points, left_tri, right_tri);
    if coplanar.relation == CoplanarTriangleRelation::Unknown
        || coplanar.relation == CoplanarTriangleRelation::Disjoint
    {
        return CoplanarSurfaceContainmentReport {
            status: CoplanarSurfaceContainmentStatus::DisjointOrUnknown,
            triangle: Some(classification),
            coplanar: Some(coplanar),
        };
    }

    let left_inside_right = all_in_closed_triangle(&coplanar.left_vertices_in_right);
    let right_inside_left = all_in_closed_triangle(&coplanar.right_vertices_in_left);
    let status = match (left_inside_right, right_inside_left) {
        (true, false) => {
            CoplanarSurfaceContainmentStatus::Certified(CoplanarSurfaceContainment::LeftInsideRight)
        }
        (false, true) => {
            CoplanarSurfaceContainmentStatus::Certified(CoplanarSurfaceContainment::RightInsideLeft)
        }
        _ => CoplanarSurfaceContainmentStatus::AmbiguousOrIdentical,
    };
    CoplanarSurfaceContainmentReport {
        status,
        triangle: Some(classification),
        coplanar: Some(coplanar),
    }
}

/// Certify and materialize the positive-area intersection of two coplanar
/// single-triangle sheets.
///
/// This is the smallest exact replacement for a legacy partial-overlap case:
/// Sutherland-Hodgman style half-plane clipping is performed with
/// `hyperlimit::orient2d_report`, and edge/clip-line crossings are constructed
/// as exact `Real` ratios. The algorithmic shape follows Sutherland and
/// Hodgman, "Reentrant Polygon Clipping," *Communications of the ACM* 17.1
/// (1974), but every combinatorial decision is certified as required by Yap,
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997).
pub fn intersect_single_triangle_coplanar_surfaces(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarTriangleIntersection> {
    if left.triangles().len() != 1 || right.triangles().len() != 1 {
        return None;
    }

    let points = combined_points(left, right);
    let left_tri = left.triangles()[0].0;
    let right_offset = left.vertices().len();
    let right_tri = right.triangles()[0].0.map(|index| index + right_offset);

    let classification = classify_triangle_triangle(&points, left_tri, right_tri);
    if classification.relation != TriangleTriangleRelation::CoplanarOverlapping {
        return None;
    }

    let coplanar = classify_coplanar_triangles(&points, left_tri, right_tri);
    if coplanar.relation != CoplanarTriangleRelation::Overlapping {
        return None;
    }
    let projection = coplanar.projection?;

    let left_polygon = left_tri
        .iter()
        .map(|&index| points[index].clone())
        .collect::<Vec<_>>();
    let clip_polygon = right_tri
        .iter()
        .map(|&index| points[index].clone())
        .collect::<Vec<_>>();
    let clipped = clip_convex_polygon(&left_polygon, &clip_polygon, projection)?;
    let polygon = simplify_projected_polygon(clipped, projection);
    if polygon.len() < 3 {
        return None;
    }

    let mesh = polygon_to_open_mesh(&polygon)?;
    let intersection = CoplanarTriangleIntersection {
        projection,
        polygon,
        mesh,
    };
    intersection.validate().ok()?;
    Some(intersection)
}

/// Certify and materialize a convex union of two coplanar single-triangle
/// sheets.
///
/// The candidate output is the exact convex hull of all triangle vertices.
/// Hypermesh certifies that this hull is not overclaiming the union by clipping
/// each fan triangle against both inputs and checking exact area coverage:
/// `area(left clip) + area(right clip) - area(overlap clip) == area(fan)`.
/// This preserves Yap's distinction between a constructed object and the
/// certified predicates that justify its topology. The convex-hull
/// construction is the standard monotone chain algorithm from Andrew, "Another
/// Efficient Algorithm for Convex Hulls in Two Dimensions," *Information
/// Processing Letters* 9.5 (1979), with exact comparisons and orientations.
pub fn union_single_triangle_coplanar_surfaces(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarTriangleUnion> {
    if left.triangles().len() != 1 || right.triangles().len() != 1 {
        return None;
    }

    let points = combined_points(left, right);
    let left_tri = left.triangles()[0].0;
    let right_offset = left.vertices().len();
    let right_tri = right.triangles()[0].0.map(|index| index + right_offset);
    let classification = classify_triangle_triangle(&points, left_tri, right_tri);
    if !matches!(
        classification.relation,
        TriangleTriangleRelation::CoplanarTouching | TriangleTriangleRelation::CoplanarOverlapping
    ) {
        return None;
    }
    let coplanar = classify_coplanar_triangles(&points, left_tri, right_tri);
    if !matches!(
        coplanar.relation,
        CoplanarTriangleRelation::Touching | CoplanarTriangleRelation::Overlapping
    ) {
        return None;
    }
    let projection = coplanar.projection?;

    let hull = convex_hull_3d(points.clone(), projection)?;
    if hull.len() < 3
        || !fan_triangles_covered_by_inputs(&hull, &points, left_tri, right_tri, projection)?
    {
        return None;
    }
    let mesh = polygon_to_open_mesh_with_label(&hull, "exact convex coplanar triangle union")?;
    let union = CoplanarTriangleUnion {
        projection,
        polygon: hull,
        mesh,
    };
    union.validate().ok()?;
    Some(union)
}

/// Certify and materialize a one-corner coplanar triangle difference.
///
/// This is a small planar-arrangement output case rather than a winding
/// shortcut. Hypermesh currently accepts the two convex one-corner shapes:
/// one strict left corner removed by the right triangle, or one strict left
/// corner remaining outside the right triangle. Both variants reuse the exact
/// clipped intersection polygon to find replacement vertices on the adjacent
/// left edges. The candidate output is accepted only when exact projected area
/// proves `area(output) + area(intersection) == area(left)`, following Yap's
/// requirement that constructed topology be justified by certified facts.
pub fn difference_single_triangle_coplanar_surfaces(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarTriangleDifference> {
    if left.triangles().len() != 1 || right.triangles().len() != 1 {
        return None;
    }

    let points = combined_points(left, right);
    let left_tri = left.triangles()[0].0;
    let right_offset = left.vertices().len();
    let right_tri = right.triangles()[0].0.map(|index| index + right_offset);
    let classification = classify_triangle_triangle(&points, left_tri, right_tri);
    if classification.relation != TriangleTriangleRelation::CoplanarOverlapping {
        return None;
    }
    let coplanar = classify_coplanar_triangles(&points, left_tri, right_tri);
    if coplanar.relation != CoplanarTriangleRelation::Overlapping {
        return None;
    }
    let projection = coplanar.projection?;
    let intersection = intersect_single_triangle_coplanar_surfaces(left, right)?;

    let left_points = left_tri
        .iter()
        .map(|&index| points[index].clone())
        .collect::<Vec<_>>();
    let polygon = if let Some(inside_index) =
        one_strict_left_vertex_inside_right(&coplanar.left_vertices_in_right)
    {
        difference_one_corner_removed(&left_points, inside_index, &intersection, projection)?
    } else if let Some(outside_index) =
        one_strict_left_vertex_outside_right(&coplanar.left_vertices_in_right)
    {
        difference_one_corner_remaining(&left_points, outside_index, &intersection, projection)?
    } else {
        return None;
    };

    let mesh =
        polygon_to_open_mesh_with_label(&polygon, "exact one-corner coplanar triangle difference")?;
    let difference = CoplanarTriangleDifference {
        projection,
        polygon,
        mesh,
    };
    difference.validate().ok()?;
    Some(difference)
}

fn difference_one_corner_removed(
    left_points: &[Point3],
    inside_index: usize,
    intersection: &CoplanarTriangleIntersection,
    projection: CoplanarProjection,
) -> Option<Vec<Point3>> {
    if intersection.polygon.len() != 3 {
        return None;
    }
    let inside = &left_points[inside_index];
    let next_index = (inside_index + 1) % 3;
    let prev_index = (inside_index + 2) % 3;
    let next = &left_points[next_index];
    let prev = &left_points[prev_index];
    let cut_next = intersection
        .polygon
        .iter()
        .find(|point| {
            !points_equal(point, inside)
                && point_on_projected_segment(inside, next, point, projection)
        })
        .cloned()?;
    let cut_prev = intersection
        .polygon
        .iter()
        .find(|point| {
            !points_equal(point, inside)
                && point_on_projected_segment(prev, inside, point, projection)
        })
        .cloned()?;

    let polygon = simplify_projected_polygon(
        vec![cut_next, next.clone(), prev.clone(), cut_prev],
        projection,
    );
    certify_difference_area(left_points, &intersection.polygon, polygon, projection)
}

fn difference_one_corner_remaining(
    left_points: &[Point3],
    outside_index: usize,
    intersection: &CoplanarTriangleIntersection,
    projection: CoplanarProjection,
) -> Option<Vec<Point3>> {
    let outside = &left_points[outside_index];
    let next_index = (outside_index + 1) % 3;
    let prev_index = (outside_index + 2) % 3;
    let next = &left_points[next_index];
    let prev = &left_points[prev_index];
    let cut_next = intersection
        .polygon
        .iter()
        .find(|point| point_on_projected_segment(outside, next, point, projection))
        .cloned()?;
    let cut_prev = intersection
        .polygon
        .iter()
        .find(|point| point_on_projected_segment(prev, outside, point, projection))
        .cloned()?;

    let polygon = simplify_projected_polygon(vec![outside.clone(), cut_next, cut_prev], projection);
    certify_difference_area(left_points, &intersection.polygon, polygon, projection)
}

fn certify_difference_area(
    left_points: &[Point3],
    intersection: &[Point3],
    polygon: Vec<Point3>,
    projection: CoplanarProjection,
) -> Option<Vec<Point3>> {
    if polygon.len() < 3 {
        return None;
    }
    let left_area = projected_area2_abs(left_points, projection)?;
    let intersection_area = projected_area2_abs(intersection, projection)?;
    let output_area = projected_area2_abs(&polygon, projection)?;
    if compare_reals(&add(&output_area, &intersection_area), &left_area).value()
        == Some(Ordering::Equal)
    {
        Some(polygon)
    } else {
        None
    }
}

fn one_strict_left_vertex_inside_right(locations: &[Option<TriangleLocation>; 3]) -> Option<usize> {
    let mut inside = None;
    for (index, location) in locations.iter().enumerate() {
        match location {
            Some(TriangleLocation::Inside) if inside.is_none() => inside = Some(index),
            Some(TriangleLocation::Inside) => return None,
            Some(TriangleLocation::Outside) => {}
            _ => return None,
        }
    }
    inside
}

fn one_strict_left_vertex_outside_right(
    locations: &[Option<TriangleLocation>; 3],
) -> Option<usize> {
    let mut outside = None;
    for (index, location) in locations.iter().enumerate() {
        match location {
            Some(TriangleLocation::Outside) if outside.is_none() => outside = Some(index),
            Some(TriangleLocation::Outside) => return None,
            Some(TriangleLocation::Inside) => {}
            _ => return None,
        }
    }
    outside
}

fn combined_points(left: &ExactMesh, right: &ExactMesh) -> Vec<Point3> {
    left.vertices()
        .iter()
        .chain(right.vertices())
        .map(|point| point.to_hyperlimit_point())
        .collect()
}

fn all_in_closed_triangle(locations: &[Option<TriangleLocation>; 3]) -> bool {
    locations.iter().all(|location| {
        matches!(
            location,
            Some(TriangleLocation::Inside | TriangleLocation::OnEdge | TriangleLocation::OnVertex)
        )
    })
}

fn clip_convex_polygon(
    subject: &[Point3],
    clip: &[Point3],
    projection: CoplanarProjection,
) -> Option<Vec<Point3>> {
    let clip2 = clip
        .iter()
        .map(|point| project_point(point, projection))
        .collect::<Vec<_>>();
    let orientation = orient2d_report(&clip2[0], &clip2[1], &clip2[2]).value()?;
    if orientation == Sign::Zero {
        return None;
    }

    let mut output = subject.to_vec();
    for edge in 0..3 {
        if output.is_empty() {
            break;
        }
        let a = &clip[edge];
        let b = &clip[(edge + 1) % 3];
        let a2 = &clip2[edge];
        let b2 = &clip2[(edge + 1) % 3];
        let input = output;
        output = Vec::new();
        let mut previous = input.last()?.clone();
        let mut previous_inside =
            point_inside_or_on_edge(&previous, a2, b2, orientation, projection)?;
        for current in input {
            let current_inside =
                point_inside_or_on_edge(&current, a2, b2, orientation, projection)?;
            match (previous_inside, current_inside) {
                (true, true) => output.push(current.clone()),
                (true, false) => {
                    output.push(intersect_segment_with_projected_line(
                        &previous, &current, a, b, projection,
                    )?);
                }
                (false, true) => {
                    output.push(intersect_segment_with_projected_line(
                        &previous, &current, a, b, projection,
                    )?);
                    output.push(current.clone());
                }
                (false, false) => {}
            }
            previous = current;
            previous_inside = current_inside;
        }
    }
    Some(output)
}

fn point_inside_or_on_edge(
    point: &Point3,
    edge_start: &Point2,
    edge_end: &Point2,
    clip_orientation: Sign,
    projection: CoplanarProjection,
) -> Option<bool> {
    let projected = project_point(point, projection);
    let side = orient2d_report(edge_start, edge_end, &projected).value()?;
    Some(side == Sign::Zero || side == clip_orientation)
}

fn intersect_segment_with_projected_line(
    p0: &Point3,
    p1: &Point3,
    line_a: &Point3,
    line_b: &Point3,
    projection: CoplanarProjection,
) -> Option<Point3> {
    let a = project_point(line_a, projection);
    let b = project_point(line_b, projection);
    let q0 = project_point(p0, projection);
    let q1 = project_point(p1, projection);
    let d0 = orient2d_value(&a, &b, &q0);
    let d1 = orient2d_value(&a, &b, &q1);
    let denominator = sub(&d0, &d1);
    if compare_reals(&denominator, &ExactReal::from(0)).value() == Some(Ordering::Equal) {
        return None;
    }
    let t = (d0 / &denominator).ok()?;
    Some(interpolate3(p0, p1, &t))
}

fn simplify_projected_polygon(
    mut polygon: Vec<Point3>,
    projection: CoplanarProjection,
) -> Vec<Point3> {
    remove_duplicate_neighbors(&mut polygon);
    loop {
        let original_len = polygon.len();
        if original_len < 3 {
            return polygon;
        }
        let mut simplified = Vec::with_capacity(original_len);
        for index in 0..original_len {
            let previous = &polygon[(index + original_len - 1) % original_len];
            let current = &polygon[index];
            let next = &polygon[(index + 1) % original_len];
            let pa = project_point(previous, projection);
            let pb = project_point(current, projection);
            let pc = project_point(next, projection);
            if orient2d_report(&pa, &pb, &pc).value() != Some(Sign::Zero) {
                simplified.push(current.clone());
            }
        }
        remove_duplicate_neighbors(&mut simplified);
        if simplified.len() == original_len {
            return simplified;
        }
        polygon = simplified;
    }
}

fn remove_duplicate_neighbors(points: &mut Vec<Point3>) {
    points.dedup_by(|right, left| points_equal(left, right));
    if points.len() > 1 && points_equal(points.first().unwrap(), points.last().unwrap()) {
        points.pop();
    }
}

fn polygon_to_open_mesh(polygon: &[Point3]) -> Option<ExactMesh> {
    polygon_to_open_mesh_with_label(polygon, "exact coplanar triangle intersection")
}

fn polygon_to_open_mesh_with_label(polygon: &[Point3], label: &'static str) -> Option<ExactMesh> {
    if polygon.len() < 3 {
        return None;
    }
    let vertices = polygon
        .iter()
        .map(|point| ExactPoint3::new(point.x.clone(), point.y.clone(), point.z.clone()))
        .collect::<Vec<_>>();
    let triangles = (1..polygon.len() - 1)
        .map(|index| Triangle([0, index, index + 1]))
        .collect::<Vec<_>>();
    ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact(label),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .ok()
}

fn validate_coplanar_surface_output(
    projection: CoplanarProjection,
    polygon: &[Point3],
    mesh: &ExactMesh,
    label: &'static str,
) -> Result<(), MeshError> {
    if polygon.len() < 3 {
        return Err(surface_validation_error(
            label,
            "surface polygon has fewer than three vertices",
        ));
    }
    if mesh.vertices().len() != polygon.len() {
        return Err(surface_validation_error(
            label,
            "surface mesh vertex count does not match polygon vertex count",
        ));
    }
    if mesh.triangles().len() != polygon.len() - 2 {
        return Err(surface_validation_error(
            label,
            "surface mesh triangle count does not match fan triangulation",
        ));
    }

    for (index, point) in polygon.iter().enumerate() {
        if !points_equal(point, &mesh.vertices()[index].to_hyperlimit_point()) {
            return Err(surface_validation_error(
                label,
                "surface mesh vertex does not match polygon point",
            ));
        }
    }
    for (index, triangle) in mesh.triangles().iter().enumerate() {
        if triangle.0 != [0, index + 1, index + 2] {
            return Err(surface_validation_error(
                label,
                "surface mesh is not the expected fan triangulation",
            ));
        }
    }
    for left in 0..polygon.len() {
        for right in left + 1..polygon.len() {
            if points_equal(&polygon[left], &polygon[right]) {
                return Err(surface_validation_error(
                    label,
                    "surface polygon repeats an exact point",
                ));
            }
        }
    }
    let Some(area) = projected_area2_abs(polygon, projection) else {
        return Err(surface_validation_error(
            label,
            "surface polygon projected area was undecided",
        ));
    };
    if compare_reals(&area, &ExactReal::from(0)).value() != Some(Ordering::Greater) {
        return Err(surface_validation_error(
            label,
            "surface polygon has zero projected area",
        ));
    }

    Ok(())
}

fn surface_validation_error(label: &'static str, reason: &'static str) -> MeshError {
    MeshError::one(MeshDiagnostic::new(
        Severity::Error,
        DiagnosticKind::DegenerateTriangle,
        format!("{label} validation failed: {reason}"),
    ))
}

fn convex_hull_3d(points: Vec<Point3>, projection: CoplanarProjection) -> Option<Vec<Point3>> {
    let mut projected = points
        .into_iter()
        .map(|point| {
            let projected = project_point(&point, projection);
            (projected, point)
        })
        .collect::<Vec<_>>();
    projected.sort_by(|left, right| compare_point2(&left.0, &right.0).unwrap_or(Ordering::Equal));
    projected.dedup_by(|right, left| point2_equal(&left.0, &right.0));
    if projected.len() < 3 {
        return None;
    }

    let mut lower = Vec::<(Point2, Point3)>::new();
    for point in &projected {
        while lower.len() >= 2 {
            let sign = orient2d_report(
                &lower[lower.len() - 2].0,
                &lower[lower.len() - 1].0,
                &point.0,
            )
            .value()?;
            if sign == Sign::Positive {
                break;
            }
            lower.pop();
        }
        lower.push(point.clone());
    }

    let mut upper = Vec::<(Point2, Point3)>::new();
    for point in projected.iter().rev() {
        while upper.len() >= 2 {
            let sign = orient2d_report(
                &upper[upper.len() - 2].0,
                &upper[upper.len() - 1].0,
                &point.0,
            )
            .value()?;
            if sign == Sign::Positive {
                break;
            }
            upper.pop();
        }
        upper.push(point.clone());
    }

    lower.pop();
    upper.pop();
    lower.extend(upper);
    let hull = lower
        .into_iter()
        .map(|(_, point)| point)
        .collect::<Vec<_>>();
    if hull.len() < 3 { None } else { Some(hull) }
}

fn fan_triangles_covered_by_inputs(
    hull: &[Point3],
    points: &[Point3],
    left_tri: [usize; 3],
    right_tri: [usize; 3],
    projection: CoplanarProjection,
) -> Option<bool> {
    let left = left_tri
        .iter()
        .map(|&index| points[index].clone())
        .collect::<Vec<_>>();
    let right = right_tri
        .iter()
        .map(|&index| points[index].clone())
        .collect::<Vec<_>>();
    for index in 1..hull.len() - 1 {
        let fan = vec![
            hull[0].clone(),
            hull[index].clone(),
            hull[index + 1].clone(),
        ];
        if !fan_triangle_covered_by_inputs(&fan, &left, &right, projection)? {
            return Some(false);
        }
    }
    Some(true)
}

fn fan_triangle_covered_by_inputs(
    fan: &[Point3],
    left: &[Point3],
    right: &[Point3],
    projection: CoplanarProjection,
) -> Option<bool> {
    let fan_area = projected_area2_abs(fan, projection)?;
    let left_clip = simplify_projected_polygon(
        clip_convex_polygon(fan, left, projection).unwrap_or_default(),
        projection,
    );
    let right_clip = simplify_projected_polygon(
        clip_convex_polygon(fan, right, projection).unwrap_or_default(),
        projection,
    );
    let both_clip = if left_clip.len() >= 3 {
        simplify_projected_polygon(
            clip_convex_polygon(&left_clip, right, projection).unwrap_or_default(),
            projection,
        )
    } else {
        Vec::new()
    };
    let covered = sub(
        &add(
            &projected_area2_abs(&left_clip, projection).unwrap_or_else(|| ExactReal::from(0)),
            &projected_area2_abs(&right_clip, projection).unwrap_or_else(|| ExactReal::from(0)),
        ),
        &projected_area2_abs(&both_clip, projection).unwrap_or_else(|| ExactReal::from(0)),
    );
    Some(compare_reals(&covered, &fan_area).value() == Some(Ordering::Equal))
}

fn project_point(point: &Point3, projection: CoplanarProjection) -> Point2 {
    match projection {
        CoplanarProjection::Xy => Point2::new(point.x.clone(), point.y.clone()),
        CoplanarProjection::Xz => Point2::new(point.x.clone(), point.z.clone()),
        CoplanarProjection::Yz => Point2::new(point.y.clone(), point.z.clone()),
    }
}

fn point_on_projected_segment(
    start: &Point3,
    end: &Point3,
    point: &Point3,
    projection: CoplanarProjection,
) -> bool {
    point_on_segment(
        &project_point(start, projection),
        &project_point(end, projection),
        &project_point(point, projection),
    )
    .value()
        == Some(true)
}

fn projected_area2_abs(points: &[Point3], projection: CoplanarProjection) -> Option<ExactReal> {
    if points.len() < 3 {
        return Some(ExactReal::from(0));
    }
    let mut sum = ExactReal::from(0);
    for index in 0..points.len() {
        let current = project_point(&points[index], projection);
        let next = project_point(&points[(index + 1) % points.len()], projection);
        sum = add(
            &sum,
            &sub(&mul(&current.x, &next.y), &mul(&current.y, &next.x)),
        );
    }
    match compare_reals(&sum, &ExactReal::from(0)).value()? {
        Ordering::Less => Some(sub(&ExactReal::from(0), &sum)),
        Ordering::Equal | Ordering::Greater => Some(sum),
    }
}

fn compare_point2(left: &Point2, right: &Point2) -> Option<Ordering> {
    match compare_reals(&left.x, &right.x).value()? {
        Ordering::Equal => compare_reals(&left.y, &right.y).value(),
        ordering => Some(ordering),
    }
}

fn point2_equal(left: &Point2, right: &Point2) -> bool {
    compare_reals(&left.x, &right.x).value() == Some(Ordering::Equal)
        && compare_reals(&left.y, &right.y).value() == Some(Ordering::Equal)
}

fn orient2d_value(a: &Point2, b: &Point2, c: &Point2) -> ExactReal {
    let bax = sub(&b.x, &a.x);
    let bay = sub(&b.y, &a.y);
    let cax = sub(&c.x, &a.x);
    let cay = sub(&c.y, &a.y);
    sub(&mul(&bax, &cay), &mul(&bay, &cax))
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
