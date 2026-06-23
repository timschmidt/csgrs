//! Exact closed-convex boolean fragments.
//!
//! This module handles certified object-level closed-convex boolean fragments.
//! Each output face is produced by clipping a source face polygon against the
//! other solid's exact oriented halfspaces, then the resulting triangle mesh is
//! revalidated through [`ExactMesh`]. Boolean topology is emitted only from
//! retained object facts and proof-producing predicate routes.
//!
//! The clipping is the convex halfspace specialization of Sutherland and
//! Hodgman, "Reentrant Polygon Clipping," *Communications of the ACM* 17.1
//! (1974), using `hyperlimit::orient3d_report` and exact determinant-ratio
//! interpolation.

use std::{
    cmp::Ordering,
    collections::{BTreeMap, BTreeSet},
};

use hyperlimit::{
    PlaneSide, Point2, Point3, compare_reals, interpolate_point3 as interpolate3, orient3d_report,
    point_on_segment,
};

use super::arrangement2d::{
    ExactArrangement2dBoundaryPolicy, ExactArrangement2dOutputLoop, ExactArrangement2dOverlay,
    ExactArrangement2dRegion, ExactArrangement2dRegionRing, ExactArrangement2dSetOperation,
    build_exact_arrangement2d_overlay_with_boundary_policy,
};
use super::error::{ExactMeshBlocker, ExactMeshBlockerKind, ExactMeshError};
use super::mesh::{ExactMesh, Triangle};
use super::solid::{
    ClosedMeshOrientation, ConvexSolidFacts, ConvexSolidReportError, certify_convex_solid,
};
use super::topology::triangle_edges;
use super::validation::ExactMeshValidationPolicy;
use hyperlimit::SourceProvenance;
use hyperlimit::{CoplanarProjection, project_point3, projected_polygon_area2_value};
use hyperreal::Real;

/// Certified intersection of two closed convex solids.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ConvexSolidIntersection {
    /// Convexity and orientation facts for the left operand.
    pub left_facts: ConvexSolidFacts,
    /// Convexity and orientation facts for the right operand.
    pub right_facts: ConvexSolidFacts,
    /// Exact closed mesh materialized from clipped source-face polygons.
    pub mesh: ExactMesh,
}

/// Certified union of two closed convex solids.
///
/// The output is not assumed to be convex. Each source face is projected to an
/// exact 2D carrier plane, the portion covered by the opposite convex solid is
/// subtracted with the exact planar arrangement layer, and retained cells are
/// triangulated back on the original source plane. Exact edge refinement is run
/// before closed validation so retained whole-face fragments are split at
/// neighboring intersection vertices instead of forming T-junctions.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ConvexSolidUnion {
    /// Convexity and orientation facts for the left operand.
    pub left_facts: ConvexSolidFacts,
    /// Convexity and orientation facts for the right operand.
    pub right_facts: ConvexSolidFacts,
    /// Exact closed mesh materialized from outside source-face cells.
    pub mesh: ExactMesh,
}

/// Certified difference of two closed convex solids.
///
/// The result may be nonconvex. It is materialized as exact source-face cells:
/// left boundary portions outside the right solid keep their source
/// orientation, while right boundary portions inside the left solid are
/// retained with reversed orientation as cut/cavity faces.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ConvexSolidDifference {
    /// Convexity and orientation facts for the left operand.
    pub left_facts: ConvexSolidFacts,
    /// Convexity and orientation facts for the right operand.
    pub right_facts: ConvexSolidFacts,
    /// Exact closed mesh materialized from retained source-face cells.
    pub mesh: ExactMesh,
}

impl ConvexSolidUnion {
    /// Validate retained facts and the materialized mesh.
    pub fn validate(&self) -> Result<(), ExactMeshError> {
        self.left_facts.validate().map_err(report_error)?;
        self.right_facts.validate().map_err(report_error)?;
        if !self.left_facts.is_certified_convex() || !self.right_facts.is_certified_convex() {
            return Err(ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::ExactConstructionFailure,
                "convex union retained non-certified solid facts",
            )));
        }
        self.mesh.validate_retained_state().map_err(|error| {
            ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::ExactConstructionFailure,
                format!("convex union output failed retained-state replay: {error:?}"),
            ))
        })
    }
}

impl ConvexSolidDifference {
    /// Validate retained facts and the materialized mesh.
    pub fn validate(&self) -> Result<(), ExactMeshError> {
        self.left_facts.validate().map_err(report_error)?;
        self.right_facts.validate().map_err(report_error)?;
        if !self.left_facts.is_certified_convex() || !self.right_facts.is_certified_convex() {
            return Err(ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::ExactConstructionFailure,
                "convex difference retained non-certified solid facts",
            )));
        }
        self.mesh.validate_retained_state().map_err(|error| {
            ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::ExactConstructionFailure,
                format!("convex difference output failed retained-state replay: {error:?}"),
            ))
        })
    }
}

impl ConvexSolidIntersection {
    /// Validate retained facts and the materialized mesh.
    pub fn validate(&self) -> Result<(), ExactMeshError> {
        self.left_facts.validate().map_err(report_error)?;
        self.right_facts.validate().map_err(report_error)?;
        if !self.left_facts.is_certified_convex() || !self.right_facts.is_certified_convex() {
            return Err(ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::ExactConstructionFailure,
                "convex intersection retained non-certified solid facts",
            )));
        }
        self.mesh.validate_retained_state().map_err(|error| {
            ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::ExactConstructionFailure,
                format!("convex intersection output failed retained-state replay: {error:?}"),
            ))
        })
    }
}

/// Certify and materialize the union of two closed convex solids.
///
/// Unlike intersection, convex union can be nonconvex. This routine still
/// stays inside exact arithmetic: it keeps the parts of each source face that
/// are outside the opposite convex solid and revalidates the resulting shell as
/// a closed exact mesh.
pub(crate) fn union_closed_convex_solids(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<ConvexSolidUnion> {
    let left_facts = certify_convex_solid(left);
    let right_facts = certify_convex_solid(right);
    if !left_facts.is_certified_convex() || !right_facts.is_certified_convex() {
        return None;
    }

    let mut vertices = Vec::new();
    let mut triangles = Vec::new();
    append_convex_union_source_faces(left, right, &right_facts, &mut vertices, &mut triangles)?;
    append_convex_union_source_faces(right, left, &left_facts, &mut vertices, &mut triangles)?;
    if triangles.is_empty() {
        return None;
    }
    refine_triangles_at_existing_edge_vertices(&vertices, &mut triangles)?;
    remove_duplicate_triangle_vertex_sets(&mut triangles);
    orient_paired_triangle_edges(&mut triangles)?;
    let mesh = ExactMesh::new_with_policy(
        vertices.clone(),
        triangles.clone(),
        SourceProvenance::exact("exact closed-convex solid union"),
        ExactMeshValidationPolicy::CLOSED,
    )
    .ok()
    .or_else(|| close_planar_boundary_loops(vertices.clone(), triangles.clone()))
    .or_else(|| union_from_difference_and_operand(left, right))
    .or_else(|| union_from_difference_and_operand(right, left))?;
    if !mesh_has_nonzero_signed_volume(&mesh)? {
        return None;
    }
    let union = ConvexSolidUnion {
        left_facts,
        right_facts,
        mesh,
    };
    union.validate().ok()?;
    Some(union)
}

/// Certify and materialize `left - right` for two closed convex solids.
///
/// This is the exact source-face-cell construction for general convex
/// difference. It returns `None` for empty/lower-dimensional results or when
/// exact face-cell triangulation cannot be certified.
pub(crate) fn subtract_closed_convex_solids(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<ConvexSolidDifference> {
    let left_facts = certify_convex_solid(left);
    let right_facts = certify_convex_solid(right);
    if !left_facts.is_certified_convex() || !right_facts.is_certified_convex() {
        return None;
    }

    let mut vertices = Vec::new();
    let mut triangles = Vec::new();
    append_convex_difference_left_faces(left, right, &right_facts, &mut vertices, &mut triangles)?;
    append_convex_difference_right_faces(right, left, &left_facts, &mut vertices, &mut triangles)?;
    if triangles.is_empty() {
        return None;
    }
    refine_triangles_at_existing_edge_vertices(&vertices, &mut triangles)?;
    remove_duplicate_triangle_vertex_sets(&mut triangles);
    orient_paired_triangle_edges(&mut triangles)?;
    let mesh = ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact("exact closed-convex solid difference"),
        ExactMeshValidationPolicy::CLOSED,
    )
    .ok()?;
    if !mesh_has_nonzero_signed_volume(&mesh)? {
        return None;
    }
    let difference = ConvexSolidDifference {
        left_facts,
        right_facts,
        mesh,
    };
    difference.validate().ok()?;
    Some(difference)
}

fn union_from_difference_and_operand(left: &ExactMesh, right: &ExactMesh) -> Option<ExactMesh> {
    let difference = subtract_closed_convex_solids(left, right)?;
    let mut vertices = difference.mesh.vertices().to_vec();
    let mut triangles = difference.mesh.triangles().to_vec();
    let right_vertex_map = right
        .vertices()
        .iter()
        .map(|point| intern_point(&mut vertices, point))
        .collect::<Vec<_>>();
    triangles.extend(right.triangles().iter().map(|triangle| {
        Triangle([
            right_vertex_map[triangle.0[0]],
            right_vertex_map[triangle.0[1]],
            right_vertex_map[triangle.0[2]],
        ])
    }));
    refine_triangles_at_existing_edge_vertices(&vertices, &mut triangles)?;
    remove_duplicate_triangle_vertex_sets(&mut triangles);
    orient_paired_triangle_edges(&mut triangles)?;
    ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact("exact closed-convex solid union from exact difference"),
        ExactMeshValidationPolicy::CLOSED,
    )
    .ok()
}

fn close_planar_boundary_loops(
    vertices: Vec<Point3>,
    mut triangles: Vec<Triangle>,
) -> Option<ExactMesh> {
    let loops = directed_boundary_loops(&triangles)?;
    if loops.is_empty() {
        return None;
    }
    for loop_ in loops {
        if loop_.len() < 3 || !loop_is_planar(&vertices, &loop_)? {
            return None;
        }
        for index in 1..loop_.len() - 1 {
            triangles.push(Triangle([loop_[0], loop_[index], loop_[index + 1]]));
        }
    }
    remove_duplicate_triangle_vertex_sets(&mut triangles);
    orient_paired_triangle_edges(&mut triangles)?;
    ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact("exact closed-convex solid union with exact planar caps"),
        ExactMeshValidationPolicy::CLOSED,
    )
    .ok()
}

fn directed_boundary_loops(triangles: &[Triangle]) -> Option<Vec<Vec<usize>>> {
    let mut edge_counts = BTreeMap::<[usize; 2], usize>::new();
    for triangle in triangles {
        for edge in triangle_edges(triangle.0) {
            let mut key = edge;
            key.sort_unstable();
            *edge_counts.entry(key).or_insert(0) += 1;
        }
    }
    let mut next = BTreeMap::<usize, usize>::new();
    for triangle in triangles {
        for edge in triangle_edges(triangle.0) {
            let mut key = edge;
            key.sort_unstable();
            if edge_counts.get(&key).copied() == Some(1) && next.insert(edge[0], edge[1]).is_some()
            {
                return None;
            }
        }
    }
    let mut loops = Vec::new();
    while let Some((&start, _)) = next.iter().next() {
        let mut loop_ = vec![start];
        let mut current = start;
        loop {
            let end = next.remove(&current)?;
            if end == start {
                break;
            }
            if loop_.contains(&end) {
                return None;
            }
            loop_.push(end);
            current = end;
        }
        loops.push(loop_);
    }
    Some(loops)
}

fn loop_is_planar(vertices: &[Point3], loop_: &[usize]) -> Option<bool> {
    let a = vertices.get(loop_[0])?;
    let b = vertices.get(loop_[1])?;
    let c = vertices.get(loop_[2])?;
    if points_are_collinear(a, b, c) {
        return Some(false);
    }
    for &vertex in loop_ {
        if point_side(a, b, c, vertices.get(vertex)?)? != PlaneSide::On {
            return Some(false);
        }
    }
    Some(true)
}

/// Certify and materialize the intersection of two closed convex solids.
///
/// Returns `None` unless both operands certify as closed convex solids and the
/// clipped output revalidates as a closed exact triangle mesh. It does not
/// approximate winding, and it does not claim union/difference support for
/// partial overlaps.
pub(crate) fn intersect_closed_convex_solids(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<ConvexSolidIntersection> {
    let left_facts = certify_convex_solid(left);
    let right_facts = certify_convex_solid(right);
    if !left_facts.is_certified_convex() || !right_facts.is_certified_convex() {
        return None;
    }

    let mut polygons = Vec::new();
    clipped_source_faces(left, right, &right_facts, &mut polygons)?;
    clipped_source_faces(right, left, &left_facts, &mut polygons)?;
    if polygons.is_empty() {
        return None;
    }

    let hull_polygons = convex_hull_polygons_from_clipped_faces(&polygons)?;
    let mesh = polygons_to_closed_mesh(
        &hull_polygons,
        "exact closed-convex solid intersection",
        ExactMeshValidationPolicy::CLOSED,
    )?;
    if !mesh_has_nonzero_signed_volume(&mesh)? {
        return None;
    }
    let intersection = ConvexSolidIntersection {
        left_facts,
        right_facts,
        mesh,
    };
    intersection.validate().ok()?;
    Some(intersection)
}

/// Return whether the retained triangle shell encloses nonzero exact volume.
///
/// This guard is what keeps closed-convex intersection from promoting
/// only after an exact predicate, here the signed tetrahedral volume sum,
/// proves the result is a solid instead of a lower-dimensional boundary.
fn mesh_has_nonzero_signed_volume(mesh: &ExactMesh) -> Option<bool> {
    let signed_volume = mesh
        .triangles()
        .iter()
        .map(|triangle| {
            let tri = triangle.0;
            determinant_from_origin(
                &mesh.vertices()[tri[0]].clone(),
                &mesh.vertices()[tri[1]].clone(),
                &mesh.vertices()[tri[2]].clone(),
            )
        })
        .fold(Real::from(0), |sum, det| &sum + &det);

    Some(compare_reals(&signed_volume, &Real::from(0)).value()? != Ordering::Equal)
}

fn determinant_from_origin(a: &Point3, b: &Point3, c: &Point3) -> Real {
    let by_cz = &b.y * &c.z;
    let bz_cy = &b.z * &c.y;
    let bx_cz = &b.x * &c.z;
    let bz_cx = &b.z * &c.x;
    let bx_cy = &b.x * &c.y;
    let by_cx = &b.y * &c.x;

    let x_minor = &by_cz - &bz_cy;
    let y_minor = &bx_cz - &bz_cx;
    let z_minor = &bx_cy - &by_cx;

    let x_term = &a.x * &x_minor;
    let y_term = &a.y * &y_minor;
    let z_term = &a.z * &z_minor;

    &(&x_term - &y_term) + &z_term
}

fn clipped_source_faces(
    source: &ExactMesh,
    clip: &ExactMesh,
    clip_facts: &ConvexSolidFacts,
    polygons: &mut Vec<Vec<Point3>>,
) -> Option<()> {
    for triangle in source.triangles() {
        let mut polygon = triangle
            .0
            .iter()
            .map(|&index| source.vertices()[index].clone())
            .collect::<Vec<_>>();
        for clip_triangle in clip.triangles() {
            polygon = clip_polygon_by_face(&polygon, clip, clip_triangle.0, clip_facts)?;
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

fn clip_polygon_by_face(
    polygon: &[Point3],
    clip: &ExactMesh,
    face: [usize; 3],
    clip_facts: &ConvexSolidFacts,
) -> Option<Vec<Point3>> {
    if polygon.is_empty() {
        return Some(Vec::new());
    }
    let a = clip.vertices()[face[0]].clone();
    let b = clip.vertices()[face[1]].clone();
    let c = clip.vertices()[face[2]].clone();

    let mut output = Vec::new();
    let mut previous = polygon.last()?.clone();
    let mut previous_inside =
        keep_inside_side(clip_facts.orientation, point_side(&a, &b, &c, &previous)?);
    for current in polygon {
        let current_inside =
            keep_inside_side(clip_facts.orientation, point_side(&a, &b, &c, current)?);
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

fn keep_inside_side(orientation: ClosedMeshOrientation, side: PlaneSide) -> bool {
    side == PlaneSide::On || !side_is_outside(orientation, side)
}

fn polygon_centroid(points: &[Point3]) -> Option<Point3> {
    let count = Real::from(i64::try_from(points.len()).ok()?);
    let mut x = Real::from(0);
    let mut y = Real::from(0);
    let mut z = Real::from(0);
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

fn append_convex_union_source_faces(
    source: &ExactMesh,
    clip: &ExactMesh,
    clip_facts: &ConvexSolidFacts,
    vertices: &mut Vec<Point3>,
    triangles: &mut Vec<Triangle>,
) -> Option<()> {
    for source_triangle in source.triangles() {
        let source_points = source_triangle
            .0
            .iter()
            .map(|&index| source.vertices()[index].clone())
            .collect::<Vec<_>>();
        append_source_face_minus_convex_inside(
            &source_points,
            clip,
            clip_facts,
            vertices,
            triangles,
        )?;
    }
    Some(())
}

fn append_convex_difference_left_faces(
    left: &ExactMesh,
    right: &ExactMesh,
    right_facts: &ConvexSolidFacts,
    vertices: &mut Vec<Point3>,
    triangles: &mut Vec<Triangle>,
) -> Option<()> {
    append_convex_union_source_faces(left, right, right_facts, vertices, triangles)
}

fn append_convex_difference_right_faces(
    right: &ExactMesh,
    left: &ExactMesh,
    left_facts: &ConvexSolidFacts,
    vertices: &mut Vec<Point3>,
    triangles: &mut Vec<Triangle>,
) -> Option<()> {
    for right_triangle in right.triangles() {
        let source_points = right_triangle
            .0
            .iter()
            .map(|&index| right.vertices()[index].clone())
            .collect::<Vec<_>>();
        append_source_face_convex_inside_reversed(
            &source_points,
            left,
            left_facts,
            vertices,
            triangles,
        )?;
    }
    Some(())
}

fn append_source_face_minus_convex_inside(
    source_points: &[Point3],
    clip: &ExactMesh,
    clip_facts: &ConvexSolidFacts,
    vertices: &mut Vec<Point3>,
    triangles: &mut Vec<Triangle>,
) -> Option<()> {
    if source_points.len() < 3 || polygon_is_degenerate(source_points) {
        return Some(());
    }
    let projection = choose_polygon_projection(source_points)?;
    let source_projected = source_points
        .iter()
        .map(|point| project_point3(point, projection))
        .collect::<Vec<_>>();
    let source_sign = compare_reals(
        &projected_polygon_area2_value(source_points, projection),
        &Real::from(0),
    )
    .value()?;
    if source_sign == Ordering::Equal {
        return None;
    }

    let mut inside = source_points.to_vec();
    for clip_triangle in clip.triangles() {
        inside = clip_polygon_by_face(&inside, clip, clip_triangle.0, clip_facts)?;
        if inside.len() < 3 {
            break;
        }
    }
    simplify_polygon(&mut inside);
    if inside.len() < 3 || polygon_is_degenerate(&inside) {
        append_projected_polygon_triangles(
            &source_projected,
            source_points,
            projection,
            source_sign,
            vertices,
            triangles,
        )?;
        return Some(());
    }

    let inside_projected = inside
        .iter()
        .map(|point| project_point3(point, projection))
        .collect::<Vec<_>>();
    let overlay = build_exact_arrangement2d_overlay_with_boundary_policy(
        &[
            ExactArrangement2dRegionRing::new(ExactArrangement2dRegion::Left, source_projected),
            ExactArrangement2dRegionRing::new(ExactArrangement2dRegion::Right, inside_projected),
        ],
        ExactArrangement2dSetOperation::Difference,
        ExactArrangement2dBoundaryPolicy::PreserveCollinear,
    );
    if !overlay.is_complete() {
        return None;
    }
    append_projected_overlay_triangles(
        &overlay,
        source_points,
        projection,
        source_sign,
        vertices,
        triangles,
    )
}

fn append_source_face_convex_inside_reversed(
    source_points: &[Point3],
    clip: &ExactMesh,
    clip_facts: &ConvexSolidFacts,
    vertices: &mut Vec<Point3>,
    triangles: &mut Vec<Triangle>,
) -> Option<()> {
    if source_points.len() < 3 || polygon_is_degenerate(source_points) {
        return Some(());
    }

    let mut inside = source_points.to_vec();
    for clip_triangle in clip.triangles() {
        inside = clip_polygon_by_face(&inside, clip, clip_triangle.0, clip_facts)?;
        if inside.len() < 3 {
            break;
        }
    }
    simplify_polygon(&mut inside);
    if inside.len() < 3 || polygon_is_degenerate(&inside) {
        return Some(());
    }
    if polygon_lies_on_any_clip_boundary_face(&inside, clip) {
        return Some(());
    }

    let projection = choose_polygon_projection(&inside)?;
    let source_sign = compare_reals(
        &projected_polygon_area2_value(&inside, projection),
        &Real::from(0),
    )
    .value()?;
    if source_sign == Ordering::Equal {
        return None;
    }
    let reversed_sign = reverse_orientation_sign(source_sign);
    let projected = inside
        .iter()
        .map(|point| project_point3(point, projection))
        .collect::<Vec<_>>();
    append_projected_polygon_triangles(
        &projected,
        &inside,
        projection,
        reversed_sign,
        vertices,
        triangles,
    )
}

fn polygon_lies_on_any_clip_boundary_face(polygon: &[Point3], clip: &ExactMesh) -> bool {
    clip.triangles().iter().any(|triangle| {
        let [a, b, c] = triangle.0;
        polygon.iter().all(|point| {
            point_side(
                &clip.vertices()[a],
                &clip.vertices()[b],
                &clip.vertices()[c],
                point,
            ) == Some(PlaneSide::On)
        })
    })
}

fn reverse_orientation_sign(sign: Ordering) -> Ordering {
    match sign {
        Ordering::Less => Ordering::Greater,
        Ordering::Equal => Ordering::Equal,
        Ordering::Greater => Ordering::Less,
    }
}

fn append_projected_polygon_triangles(
    projected_points: &[Point2],
    carrier_points: &[Point3],
    projection: CoplanarProjection,
    source_sign: Ordering,
    vertices: &mut Vec<Point3>,
    triangles: &mut Vec<Triangle>,
) -> Option<()> {
    let projected = projected_points
        .iter()
        .map(point2_for_hypertri)
        .collect::<Vec<_>>();
    let indices = match hypertri::earcut(&projected, &[]) {
        Ok(indices) if !indices.is_empty() && indices.len() % 3 == 0 => indices,
        _ => return None,
    };
    let lifted = projected_points
        .iter()
        .map(|point| lift_projected_point_to_carrier(point, carrier_points, projection))
        .collect::<Option<Vec<_>>>()?;
    let local_to_global = lifted
        .iter()
        .map(|point| Some(intern_point(vertices, point)))
        .collect::<Option<Vec<_>>>()?;
    append_oriented_earcut_triangles(&indices, &local_to_global, source_sign, triangles);
    Some(())
}

fn append_projected_overlay_triangles(
    overlay: &ExactArrangement2dOverlay,
    carrier_points: &[Point3],
    projection: CoplanarProjection,
    source_sign: Ordering,
    vertices: &mut Vec<Point3>,
    triangles: &mut Vec<Triangle>,
) -> Option<()> {
    for component in &overlay.output_components {
        let outer = overlay.output_loops.get(component.outer_loop)?;
        let mut projected = outer
            .points
            .iter()
            .map(point2_for_hypertri)
            .collect::<Vec<_>>();
        let mut lifted = lifted_output_loop_points(outer, carrier_points, projection)?;
        let mut hole_indices = Vec::with_capacity(component.hole_loops.len());
        for &hole_loop in &component.hole_loops {
            let hole = overlay.output_loops.get(hole_loop)?;
            hole_indices.push(projected.len());
            projected.extend(hole.points.iter().map(point2_for_hypertri));
            lifted.extend(lifted_output_loop_points(hole, carrier_points, projection)?);
        }
        let local_to_global = lifted
            .iter()
            .map(|point| Some(intern_point(vertices, point)))
            .collect::<Option<Vec<_>>>()?;
        let indices = match hypertri::earcut(&projected, &hole_indices) {
            Ok(indices) if !indices.is_empty() && indices.len() % 3 == 0 => indices,
            _ => return None,
        };
        append_oriented_earcut_triangles(&indices, &local_to_global, source_sign, triangles);
    }
    Some(())
}

fn lifted_output_loop_points(
    loop_: &ExactArrangement2dOutputLoop,
    carrier_points: &[Point3],
    projection: CoplanarProjection,
) -> Option<Vec<Point3>> {
    loop_
        .points
        .iter()
        .map(|point| lift_projected_point_to_carrier(point, carrier_points, projection))
        .collect()
}

fn append_oriented_earcut_triangles(
    indices: &[usize],
    local_to_global: &[usize],
    source_sign: Ordering,
    triangles: &mut Vec<Triangle>,
) {
    for triangle in indices.chunks_exact(3) {
        let a = local_to_global[triangle[0]];
        let b = local_to_global[triangle[1]];
        let c = local_to_global[triangle[2]];
        if a == b || b == c || c == a {
            continue;
        }
        if source_sign == Ordering::Greater {
            triangles.push(Triangle([a, b, c]));
        } else {
            triangles.push(Triangle([a, c, b]));
        }
    }
}

fn point2_for_hypertri(point: &Point2) -> hypertri::ExactPoint {
    hypertri::ExactPoint::new(point.x.clone(), point.y.clone())
}

fn lift_projected_point_to_carrier(
    point: &Point2,
    carrier: &[Point3],
    projection: CoplanarProjection,
) -> Option<Point3> {
    let projected = [
        project_point3(&carrier[0], projection),
        project_point3(&carrier[1], projection),
        project_point3(&carrier[2], projection),
    ];
    let ux = projected[1].x.clone() - &projected[0].x;
    let uy = projected[1].y.clone() - &projected[0].y;
    let vx = projected[2].x.clone() - &projected[0].x;
    let vy = projected[2].y.clone() - &projected[0].y;
    let wx = point.x.clone() - &projected[0].x;
    let wy = point.y.clone() - &projected[0].y;
    let det = ux.clone() * &vy - &(uy.clone() * &vx);
    let a = ((wx.clone() * &vy - &(wy.clone() * &vx)) / &det).ok()?;
    let b = ((ux * &wy - &(uy * &wx)) / &det).ok()?;
    let p1 = vector_between(&carrier[0], &carrier[1]);
    let p2 = vector_between(&carrier[0], &carrier[2]);
    Some(Point3::new(
        carrier[0].x.clone() + &(p1.x * &a) + &(p2.x * &b),
        carrier[0].y.clone() + &(p1.y * &a) + &(p2.y * &b),
        carrier[0].z.clone() + &(p1.z * &a) + &(p2.z * &b),
    ))
}

fn vector_between(from: &Point3, to: &Point3) -> Point3 {
    Point3::new(
        to.x.clone() - &from.x,
        to.y.clone() - &from.y,
        to.z.clone() - &from.z,
    )
}

fn choose_polygon_projection(points: &[Point3]) -> Option<CoplanarProjection> {
    for &projection in &[
        CoplanarProjection::Xy,
        CoplanarProjection::Xz,
        CoplanarProjection::Yz,
    ] {
        let area = projected_polygon_area2_value(points, projection);
        if compare_reals(&area, &Real::from(0)).value()? != Ordering::Equal {
            return Some(projection);
        }
    }
    None
}

fn convex_hull_polygons_from_clipped_faces(polygons: &[Vec<Point3>]) -> Option<Vec<Vec<Point3>>> {
    let mut points = Vec::new();
    for polygon in polygons {
        for point in polygon {
            intern_point(&mut points, point);
        }
    }
    if points.len() < 4 {
        return None;
    }
    let interior = polygon_centroid(&points)?;
    let mut seen_faces = BTreeSet::new();
    let mut hull_faces = Vec::new();
    for a in 0..points.len() {
        for b in a + 1..points.len() {
            for c in b + 1..points.len() {
                if points_are_collinear(&points[a], &points[b], &points[c]) {
                    continue;
                }
                let mut saw_above = false;
                let mut saw_below = false;
                let mut coplanar = Vec::new();
                for (index, point) in points.iter().enumerate() {
                    match point_side(&points[a], &points[b], &points[c], point)? {
                        PlaneSide::On => coplanar.push(index),
                        PlaneSide::Above => saw_above = true,
                        PlaneSide::Below => saw_below = true,
                    }
                    if saw_above && saw_below {
                        break;
                    }
                }
                if saw_above && saw_below {
                    continue;
                }
                coplanar.sort_unstable();
                if coplanar.len() < 3 || !seen_faces.insert(coplanar.clone()) {
                    continue;
                }
                let mut face = convex_face_polygon_from_indices(&points, &coplanar)?;
                orient_face_polygon_outward(&mut face, &interior)?;
                hull_faces.push(face);
            }
        }
    }
    if hull_faces.len() >= 4 {
        Some(hull_faces)
    } else {
        None
    }
}

fn convex_face_polygon_from_indices(points: &[Point3], indices: &[usize]) -> Option<Vec<Point3>> {
    if indices.len() == 3 {
        return Some(indices.iter().map(|&index| points[index].clone()).collect());
    }
    let projection = choose_face_projection(points, indices)?;
    let mut segments = Vec::new();
    for (left_offset, &left) in indices.iter().enumerate() {
        for &right in &indices[left_offset + 1..] {
            if convex_face_pair_is_boundary_edge(points, indices, left, right, projection)? {
                push_unique_segment(&mut segments, [points[left].clone(), points[right].clone()]);
            }
        }
    }
    let mut polygon = chain_segments_to_polygon(segments)?;
    remove_collinear_polygon_vertices(&mut polygon, projection);
    if polygon.len() >= 3 && !polygon_is_degenerate(&polygon) {
        Some(polygon)
    } else {
        None
    }
}

fn choose_face_projection(points: &[Point3], indices: &[usize]) -> Option<CoplanarProjection> {
    for &projection in &[
        CoplanarProjection::Xy,
        CoplanarProjection::Xz,
        CoplanarProjection::Yz,
    ] {
        for a in 0..indices.len() {
            for b in a + 1..indices.len() {
                for c in b + 1..indices.len() {
                    let area = projected_area2_signed(
                        &points[indices[a]],
                        &points[indices[b]],
                        &points[indices[c]],
                        projection,
                    );
                    if compare_reals(&area, &Real::from(0)).value() != Some(Ordering::Equal) {
                        return Some(projection);
                    }
                }
            }
        }
    }
    None
}

fn convex_face_pair_is_boundary_edge(
    points: &[Point3],
    indices: &[usize],
    left: usize,
    right: usize,
    projection: CoplanarProjection,
) -> Option<bool> {
    let mut saw_positive = false;
    let mut saw_negative = false;
    for &candidate in indices {
        if candidate == left || candidate == right {
            continue;
        }
        if points_are_collinear(&points[left], &points[right], &points[candidate])
            && point_lies_between_segment_endpoints(
                &points[left],
                &points[right],
                &points[candidate],
            )?
        {
            return Some(false);
        }
        match compare_reals(
            &projected_area2_signed(
                &points[left],
                &points[right],
                &points[candidate],
                projection,
            ),
            &Real::from(0),
        )
        .value()?
        {
            Ordering::Greater => saw_positive = true,
            Ordering::Less => saw_negative = true,
            Ordering::Equal => {}
        }
        if saw_positive && saw_negative {
            return Some(false);
        }
    }
    Some(true)
}

fn point_lies_between_segment_endpoints(
    start: &Point3,
    end: &Point3,
    point: &Point3,
) -> Option<bool> {
    Some(
        coordinate_between(&point.x, &start.x, &end.x)?
            && coordinate_between(&point.y, &start.y, &end.y)?
            && coordinate_between(&point.z, &start.z, &end.z)?,
    )
}

fn coordinate_between(value: &Real, start: &Real, end: &Real) -> Option<bool> {
    let start_cmp = compare_reals(value, start).value()?;
    let end_cmp = compare_reals(value, end).value()?;
    Some(matches!(
        (start_cmp, end_cmp),
        (
            Ordering::Greater | Ordering::Equal,
            Ordering::Less | Ordering::Equal
        ) | (
            Ordering::Less | Ordering::Equal,
            Ordering::Greater | Ordering::Equal
        )
    ))
}

fn projected_area2_signed(
    a: &Point3,
    b: &Point3,
    c: &Point3,
    projection: CoplanarProjection,
) -> Real {
    let (ax, ay, bx, by, cx, cy) = match projection {
        CoplanarProjection::Xy => (&a.x, &a.y, &b.x, &b.y, &c.x, &c.y),
        CoplanarProjection::Xz => (&a.x, &a.z, &b.x, &b.z, &c.x, &c.z),
        CoplanarProjection::Yz => (&a.y, &a.z, &b.y, &b.z, &c.y, &c.z),
    };
    let abx = sub(bx, ax);
    let aby = sub(by, ay);
    let acx = sub(cx, ax);
    let acy = sub(cy, ay);
    sub(&mul(&abx, &acy), &mul(&aby, &acx))
}

fn orient_face_polygon_outward(face: &mut [Point3], interior: &Point3) -> Option<()> {
    if face.len() < 3 {
        return None;
    }
    match point_side(&face[0], &face[1], &face[2], interior)? {
        PlaneSide::Above => {}
        PlaneSide::Below => face.reverse(),
        PlaneSide::On => return None,
    }
    Some(())
}

fn remove_collinear_polygon_vertices(points: &mut Vec<Point3>, projection: CoplanarProjection) {
    loop {
        let len = points.len();
        if len < 3 {
            return;
        }
        let Some(index) = (0..len).find(|&index| {
            let area = projected_area2_signed(
                &points[(index + len - 1) % len],
                &points[index],
                &points[(index + 1) % len],
                projection,
            );
            compare_reals(&area, &Real::from(0)).value() == Some(Ordering::Equal)
        }) else {
            return;
        };
        points.remove(index);
    }
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
    if compare_reals(&denominator, &Real::from(0)).value() == Some(Ordering::Equal) {
        return None;
    }
    let t = (d0 / &denominator).ok()?;
    Some(interpolate3(p0, p1, &t))
}

fn orient3d_value(a: &Point3, b: &Point3, c: &Point3, d: &Point3) -> Real {
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
    validation: ExactMeshValidationPolicy,
) -> Option<ExactMesh> {
    let mut vertices: Vec<Point3> = Vec::new();
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
    remove_duplicate_triangle_vertex_sets(&mut triangles);
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

fn refine_triangles_at_existing_edge_vertices(
    vertices: &[Point3],
    triangles: &mut Vec<Triangle>,
) -> Option<()> {
    let mut guard = 0usize;
    loop {
        guard += 1;
        if guard
            > vertices
                .len()
                .saturating_mul(triangles.len().saturating_add(1))
        {
            return None;
        }
        let Some((triangle_index, edge, vertex)) =
            find_triangle_edge_split(vertices, triangles.as_slice())?
        else {
            return Some(());
        };
        let original = triangles[triangle_index];
        let a = original.0[edge];
        let b = original.0[(edge + 1) % 3];
        let c = original.0[(edge + 2) % 3];
        triangles.splice(
            triangle_index..triangle_index + 1,
            [Triangle([a, vertex, c]), Triangle([vertex, b, c])],
        );
    }
}

fn find_triangle_edge_split(
    vertices: &[Point3],
    triangles: &[Triangle],
) -> Option<Option<(usize, usize, usize)>> {
    for (triangle_index, triangle) in triangles.iter().enumerate() {
        let projection = choose_triangle_projection(vertices, triangle.0)?;
        let [ta, tb, tc] = triangle.0;
        for edge in 0..3 {
            let start = triangle.0[edge];
            let end = triangle.0[(edge + 1) % 3];
            for candidate in 0..vertices.len() {
                if candidate == ta || candidate == tb || candidate == tc {
                    continue;
                }
                if PlaneSide::from(
                    orient3d_report(
                        &vertices[ta],
                        &vertices[tb],
                        &vertices[tc],
                        &vertices[candidate],
                    )
                    .value()?,
                ) != PlaneSide::On
                {
                    continue;
                }
                if point_equals_exact(&vertices[candidate], &vertices[start])
                    || point_equals_exact(&vertices[candidate], &vertices[end])
                {
                    continue;
                }
                if point_on_segment(
                    &project_point3(&vertices[start], projection),
                    &project_point3(&vertices[end], projection),
                    &project_point3(&vertices[candidate], projection),
                )
                .value()?
                {
                    return Some(Some((triangle_index, edge, candidate)));
                }
            }
        }
    }
    Some(None)
}

fn choose_triangle_projection(
    vertices: &[Point3],
    triangle: [usize; 3],
) -> Option<CoplanarProjection> {
    choose_polygon_projection(&[
        vertices[triangle[0]].clone(),
        vertices[triangle[1]].clone(),
        vertices[triangle[2]].clone(),
    ])
}

fn remove_duplicate_triangle_vertex_sets(triangles: &mut Vec<Triangle>) {
    let mut seen = BTreeSet::new();
    triangles.retain(|triangle| {
        let mut key = triangle.0;
        key.sort_unstable();
        seen.insert(key)
    });
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct TriangleOrientationConstraint {
    triangle: usize,
    flip_relative_to_current: bool,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct TriangleEdgeUse {
    triangle: usize,
    forward_with_key: bool,
}

fn orient_paired_triangle_edges(triangles: &mut [Triangle]) -> Option<usize> {
    let mut edge_uses = BTreeMap::<[usize; 2], Vec<TriangleEdgeUse>>::new();
    for (triangle_index, triangle) in triangles.iter().enumerate() {
        for edge in [
            [triangle.0[0], triangle.0[1]],
            [triangle.0[1], triangle.0[2]],
            [triangle.0[2], triangle.0[0]],
        ] {
            let mut key = edge;
            key.sort_unstable();
            edge_uses.entry(key).or_default().push(TriangleEdgeUse {
                triangle: triangle_index,
                forward_with_key: edge == key,
            });
        }
    }

    let mut adjacency = vec![Vec::<TriangleOrientationConstraint>::new(); triangles.len()];
    for uses in edge_uses.values() {
        let [left, right] = uses.as_slice() else {
            continue;
        };
        let same_direction = left.forward_with_key == right.forward_with_key;
        adjacency[left.triangle].push(TriangleOrientationConstraint {
            triangle: right.triangle,
            flip_relative_to_current: same_direction,
        });
        adjacency[right.triangle].push(TriangleOrientationConstraint {
            triangle: left.triangle,
            flip_relative_to_current: same_direction,
        });
    }

    let mut flips = vec![None; triangles.len()];
    for start in 0..triangles.len() {
        if flips[start].is_some() {
            continue;
        }
        flips[start] = Some(false);
        let mut stack = vec![start];
        while let Some(triangle) = stack.pop() {
            let current_flip = flips[triangle]?;
            for constraint in &adjacency[triangle] {
                let required = current_flip ^ constraint.flip_relative_to_current;
                match flips[constraint.triangle] {
                    Some(existing) if existing != required => return None,
                    Some(_) => {}
                    None => {
                        flips[constraint.triangle] = Some(required);
                        stack.push(constraint.triangle);
                    }
                }
            }
        }
    }

    let mut flipped = 0;
    for (triangle, flip) in triangles.iter_mut().zip(flips) {
        if flip == Some(true) {
            triangle.0.swap(1, 2);
            flipped += 1;
        }
    }
    Some(flipped)
}

fn intern_point(vertices: &mut Vec<Point3>, point: &Point3) -> usize {
    if let Some(index) = vertices
        .iter()
        .position(|existing| point_equals_exact(existing, point))
    {
        index
    } else {
        vertices.push(Point3::new(
            point.x.clone(),
            point.y.clone(),
            point.z.clone(),
        ));
        vertices.len() - 1
    }
}

fn point_equals_exact(left: &Point3, right: &Point3) -> bool {
    compare_reals(&left.x, &right.x).value() == Some(Ordering::Equal)
        && compare_reals(&left.y, &right.y).value() == Some(Ordering::Equal)
        && compare_reals(&left.z, &right.z).value() == Some(Ordering::Equal)
}

fn simplify_polygon(points: &mut Vec<Point3>) {
    points.dedup_by(|right, left| points_equal(left, right));
    if points.len() > 1 && points_equal(&points[0], &points[points.len() - 1]) {
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
    compare_reals(&cross_x, &Real::from(0)).value() == Some(Ordering::Equal)
        && compare_reals(&cross_y, &Real::from(0)).value() == Some(Ordering::Equal)
        && compare_reals(&cross_z, &Real::from(0)).value() == Some(Ordering::Equal)
}

fn side_is_outside(orientation: ClosedMeshOrientation, side: PlaneSide) -> bool {
    matches!(
        (orientation, side),
        (ClosedMeshOrientation::Positive, PlaneSide::Below)
            | (ClosedMeshOrientation::Negative, PlaneSide::Above)
    )
}

fn points_equal(left: &Point3, right: &Point3) -> bool {
    compare_reals(&left.x, &right.x).value() == Some(Ordering::Equal)
        && compare_reals(&left.y, &right.y).value() == Some(Ordering::Equal)
        && compare_reals(&left.z, &right.z).value() == Some(Ordering::Equal)
}

fn add(left: &Real, right: &Real) -> Real {
    left.clone() + right
}

fn sub(left: &Real, right: &Real) -> Real {
    left.clone() - right
}

fn mul(left: &Real, right: &Real) -> Real {
    left.clone() * right
}

fn report_error(error: ConvexSolidReportError) -> ExactMeshError {
    ExactMeshError::one(ExactMeshBlocker::new(
        ExactMeshBlockerKind::ExactConstructionFailure,
        format!("invalid convex-solid facts retained by intersection: {error:?}"),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::boolean::{
        ExactBooleanOperation, ExactBooleanRequest, exact_boolean_evaluation_for_replay,
    };

    fn with_materialized_evaluation_for_test<R>(
        left: &ExactMesh,
        right: &ExactMesh,
        request: ExactBooleanRequest,
        f: impl FnOnce(&crate::boolean::ExactBooleanEvaluation) -> R,
    ) -> R {
        let evaluation = exact_boolean_evaluation_for_replay(left, right, request).unwrap();
        f(&evaluation)
    }

    fn tetrahedron_i64(a: [i64; 3], b: [i64; 3], c: [i64; 3], d: [i64; 3]) -> ExactMesh {
        ExactMesh::from_i64_triangles(
            &[
                a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], d[0], d[1], d[2],
            ],
            &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
        )
        .unwrap()
    }

    #[test]
    fn straddling_coplanar_convex_union_and_difference_materialize_closed() {
        let left = tetrahedron_i64([0, 0, 0], [6, 0, 0], [0, 6, 0], [0, 0, 6]);
        let right = tetrahedron_i64([2, 2, 2], [8, -1, -1], [-1, 8, -1], [3, 2, 0]);

        let union = union_closed_convex_solids(&left, &right)
            .expect("certified convex union should close exact planar boundary loop");
        union.validate().unwrap();
        assert!(union.mesh.facts().mesh.closed_manifold);
        assert_eq!(union.mesh.facts().mesh.boundary_edges, 0);
        with_materialized_evaluation_for_test(
            &left,
            &right,
            ExactBooleanRequest::new(
                ExactBooleanOperation::Union,
                ExactMeshValidationPolicy::CLOSED,
            ),
            |evaluation| {
                evaluation.validate_against_sources(&left, &right).unwrap();
            },
        );

        let difference = subtract_closed_convex_solids(&left, &right)
            .expect("certified convex difference should orient paired cut faces");
        difference.validate().unwrap();
        assert!(difference.mesh.facts().mesh.closed_manifold);
        assert_eq!(difference.mesh.facts().mesh.boundary_edges, 0);
        with_materialized_evaluation_for_test(
            &left,
            &right,
            ExactBooleanRequest::new(
                ExactBooleanOperation::Difference,
                ExactMeshValidationPolicy::CLOSED,
            ),
            |evaluation| {
                evaluation.validate_against_sources(&left, &right).unwrap();
            },
        );
    }
}
