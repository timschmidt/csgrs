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

use super::super::arrangement3d::arrangement2d::{
    ExactArrangement2dBoundaryPolicy, ExactArrangement2dOutputLoop, ExactArrangement2dOverlay,
    ExactArrangement2dRegion, ExactArrangement2dRegionRing, ExactArrangement2dSetOperation,
    build_exact_arrangement2d_overlay_with_boundary_policy,
};
use super::super::error::{ExactMeshBlocker, ExactMeshBlockerKind, ExactMeshError};
use super::super::triangle_edges;
use super::super::validation::ExactMeshValidationPolicy;
use super::super::{ExactMesh, Triangle};
use super::solid::{ClosedMeshOrientation, ConvexSolidFacts, certify_convex_solid};
use super::{choose_nonzero_projected_polygon_area, point3_exact_equal};
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
        validate_convex_boolean_output("union", &self.left_facts, &self.right_facts, &self.mesh)
    }
}

impl ConvexSolidDifference {
    /// Validate retained facts and the materialized mesh.
    pub fn validate(&self) -> Result<(), ExactMeshError> {
        validate_convex_boolean_output(
            "difference",
            &self.left_facts,
            &self.right_facts,
            &self.mesh,
        )
    }
}

impl ConvexSolidIntersection {
    /// Validate retained facts and the materialized mesh.
    pub fn validate(&self) -> Result<(), ExactMeshError> {
        validate_convex_boolean_output(
            "intersection",
            &self.left_facts,
            &self.right_facts,
            &self.mesh,
        )
    }
}

fn validate_convex_boolean_output(
    operation: &'static str,
    left_facts: &ConvexSolidFacts,
    right_facts: &ConvexSolidFacts,
    mesh: &ExactMesh,
) -> Result<(), ExactMeshError> {
    left_facts.validate().map_err(|error| {
        ExactMeshError::one(ExactMeshBlocker::new(
            ExactMeshBlockerKind::ExactConstructionFailure,
            format!("invalid convex-solid facts retained by convex boolean: {error:?}"),
        ))
    })?;
    right_facts.validate().map_err(|error| {
        ExactMeshError::one(ExactMeshBlocker::new(
            ExactMeshBlockerKind::ExactConstructionFailure,
            format!("invalid convex-solid facts retained by convex boolean: {error:?}"),
        ))
    })?;
    if !left_facts.is_certified_convex() || !right_facts.is_certified_convex() {
        return Err(ExactMeshError::one(ExactMeshBlocker::new(
            ExactMeshBlockerKind::ExactConstructionFailure,
            format!("convex {operation} retained non-certified solid facts"),
        )));
    }
    mesh.validate_retained_state().map_err(|error| {
        ExactMeshError::one(ExactMeshBlocker::new(
            ExactMeshBlockerKind::ExactConstructionFailure,
            format!("convex {operation} output failed retained-state replay: {error:?}"),
        ))
    })
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
) -> Result<Option<ConvexSolidUnion>, ExactMeshError> {
    let left_facts = certify_convex_solid(left);
    let right_facts = certify_convex_solid(right);
    if !left_facts.is_certified_convex() || !right_facts.is_certified_convex() {
        return Ok(None);
    }

    let mut vertices = Vec::new();
    let mut triangles = Vec::new();
    if append_convex_union_source_faces(left, right, &right_facts, &mut vertices, &mut triangles)
        .is_none()
        || append_convex_union_source_faces(right, left, &left_facts, &mut vertices, &mut triangles)
            .is_none()
    {
        return Ok(None);
    }
    if triangles.is_empty() {
        return Ok(None);
    }
    if refine_triangles_at_existing_edge_vertices(&vertices, &mut triangles).is_none() {
        return Ok(None);
    }
    remove_duplicate_triangle_vertex_sets(&mut triangles);
    if orient_paired_triangle_edges(&mut triangles).is_none() {
        return Ok(None);
    }
    let direct_mesh = ExactMesh::new_with_policy(
        vertices.clone(),
        triangles.clone(),
        SourceProvenance::exact("exact closed-convex solid union"),
        ExactMeshValidationPolicy::CLOSED,
    );
    let mesh = match direct_mesh {
        Ok(mesh) => mesh,
        Err(error) => {
            if let Some(mesh) = close_planar_boundary_loops(vertices.clone(), triangles.clone()) {
                mesh
            } else if let Some(mesh) = union_from_difference_and_operand(left, right)? {
                mesh
            } else if let Some(mesh) = union_from_difference_and_operand(right, left)? {
                mesh
            } else {
                return Err(error);
            }
        }
    };
    if !mesh_has_nonzero_signed_volume(&mesh)? {
        return Ok(None);
    }
    let union = ConvexSolidUnion {
        left_facts,
        right_facts,
        mesh,
    };
    union.validate()?;
    Ok(Some(union))
}

/// Certify and materialize `left - right` for two closed convex solids.
///
/// This is the exact source-face-cell construction for general convex
/// difference. It returns `None` for empty/lower-dimensional results or when
/// exact face-cell triangulation cannot be certified.
pub(crate) fn subtract_closed_convex_solids(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<Option<ConvexSolidDifference>, ExactMeshError> {
    let left_facts = certify_convex_solid(left);
    let right_facts = certify_convex_solid(right);
    if !left_facts.is_certified_convex() || !right_facts.is_certified_convex() {
        return Ok(None);
    }

    let mut vertices = Vec::new();
    let mut triangles = Vec::new();
    if append_convex_union_source_faces(left, right, &right_facts, &mut vertices, &mut triangles)
        .is_none()
        || append_convex_difference_right_faces(
            right,
            left,
            &left_facts,
            &mut vertices,
            &mut triangles,
        )
        .is_none()
    {
        return Ok(None);
    }
    if triangles.is_empty() {
        return Ok(None);
    }
    if refine_triangles_at_existing_edge_vertices(&vertices, &mut triangles).is_none() {
        return Ok(None);
    }
    remove_duplicate_triangle_vertex_sets(&mut triangles);
    if orient_paired_triangle_edges(&mut triangles).is_none() {
        return Ok(None);
    }
    let mesh = ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact("exact closed-convex solid difference"),
        ExactMeshValidationPolicy::CLOSED,
    )?;
    if !mesh_has_nonzero_signed_volume(&mesh)? {
        return Ok(None);
    }
    let difference = ConvexSolidDifference {
        left_facts,
        right_facts,
        mesh,
    };
    difference.validate()?;
    Ok(Some(difference))
}

fn union_from_difference_and_operand(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<Option<ExactMesh>, ExactMeshError> {
    let Some(difference) = subtract_closed_convex_solids(left, right)? else {
        return Ok(None);
    };
    let mut vertices = difference.mesh.vertices().to_vec();
    let mut triangles = difference
        .mesh
        .facts()
        .faces
        .iter()
        .map(|face| Triangle(face.triangle.vertices))
        .collect::<Vec<_>>();
    let right_vertex_map = right
        .vertices()
        .iter()
        .map(|point| intern_point(&mut vertices, point))
        .collect::<Vec<_>>();
    triangles.extend(right.facts().faces.iter().map(|face| {
        let vertices = face.triangle.vertices;
        Triangle([
            right_vertex_map[vertices[0]],
            right_vertex_map[vertices[1]],
            right_vertex_map[vertices[2]],
        ])
    }));
    if refine_triangles_at_existing_edge_vertices(&vertices, &mut triangles).is_none() {
        return Ok(None);
    }
    remove_duplicate_triangle_vertex_sets(&mut triangles);
    if orient_paired_triangle_edges(&mut triangles).is_none() {
        return Ok(None);
    }
    let mesh = ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact("exact closed-convex solid union from exact difference"),
        ExactMeshValidationPolicy::CLOSED,
    )?;
    Ok(Some(mesh))
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
) -> Result<Option<ConvexSolidIntersection>, ExactMeshError> {
    let left_facts = certify_convex_solid(left);
    let right_facts = certify_convex_solid(right);
    if !left_facts.is_certified_convex() || !right_facts.is_certified_convex() {
        return Ok(None);
    }

    let mut polygons = Vec::new();
    if clipped_source_faces(left, right, &right_facts, &mut polygons).is_none()
        || clipped_source_faces(right, left, &left_facts, &mut polygons).is_none()
    {
        return Ok(None);
    }
    if polygons.is_empty() {
        return Ok(None);
    }

    let Some(hull_polygons) = convex_hull_polygons_from_clipped_faces(&polygons) else {
        return Ok(None);
    };
    let Some(mesh) = polygons_to_closed_mesh(
        &hull_polygons,
        "exact closed-convex solid intersection",
        ExactMeshValidationPolicy::CLOSED,
    ) else {
        return Ok(None);
    };
    if !mesh_has_nonzero_signed_volume(&mesh)? {
        return Ok(None);
    }
    let intersection = ConvexSolidIntersection {
        left_facts,
        right_facts,
        mesh,
    };
    intersection.validate()?;
    Ok(Some(intersection))
}

/// Return whether the retained triangle shell encloses nonzero exact volume.
///
/// This guard is what keeps closed-convex intersection from promoting
/// only after an exact predicate, here the signed tetrahedral volume sum,
/// proves the result is a solid instead of a lower-dimensional boundary.
fn mesh_has_nonzero_signed_volume(mesh: &ExactMesh) -> Result<bool, ExactMeshError> {
    let signed_volume = mesh
        .facts()
        .faces
        .iter()
        .map(|face| {
            let tri = face.triangle.vertices;
            determinant_from_origin(
                &mesh.vertices()[tri[0]],
                &mesh.vertices()[tri[1]],
                &mesh.vertices()[tri[2]],
            )
        })
        .fold(Real::from(0), |sum, det| &sum + &det);

    let Some(ordering) = compare_reals(&signed_volume, &Real::from(0)).value() else {
        return Err(ExactMeshError::one(ExactMeshBlocker::new(
            ExactMeshBlockerKind::ExactConstructionFailure,
            "convex output signed-volume comparison was undecidable",
        )));
    };
    Ok(ordering != Ordering::Equal)
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
    for source_triangle in source.view().triangles() {
        let [a, b, c] = source_triangle.vertices().ok()?;
        let mut polygon = vec![a.clone(), b.clone(), c.clone()];
        for clip_face in &clip.facts().faces {
            polygon =
                clip_polygon_by_face(&polygon, clip, clip_face.triangle.vertices, clip_facts)?;
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
    let a = &clip.vertices()[face[0]];
    let b = &clip.vertices()[face[1]];
    let c = &clip.vertices()[face[2]];

    let mut output = Vec::new();
    let orientation = clip_facts.orientation;
    let mut previous = polygon.last()?.clone();
    let mut previous_inside = !matches!(
        (orientation, point_side(a, b, c, &previous)?),
        (ClosedMeshOrientation::Positive, PlaneSide::Below)
            | (ClosedMeshOrientation::Negative, PlaneSide::Above)
    );
    for current in polygon {
        let current_inside = !matches!(
            (orientation, point_side(a, b, c, current)?),
            (ClosedMeshOrientation::Positive, PlaneSide::Below)
                | (ClosedMeshOrientation::Negative, PlaneSide::Above)
        );
        match (previous_inside, current_inside) {
            (true, true) => output.push(current.clone()),
            (true, false) => {
                output.push(intersect_segment_with_plane(&previous, current, a, b, c)?);
            }
            (false, true) => {
                output.push(intersect_segment_with_plane(&previous, current, a, b, c)?);
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

fn polygon_centroid(points: &[Point3]) -> Option<Point3> {
    let count = Real::from(i64::try_from(points.len()).ok()?);
    let mut x = Real::from(0);
    let mut y = Real::from(0);
    let mut z = Real::from(0);
    for point in points {
        x = &x + &point.x;
        y = &y + &point.y;
        z = &z + &point.z;
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
    for source_triangle in source.view().triangles() {
        let [a, b, c] = source_triangle.vertices().ok()?;
        let source_points = [a.clone(), b.clone(), c.clone()];
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

fn append_convex_difference_right_faces(
    right: &ExactMesh,
    left: &ExactMesh,
    left_facts: &ConvexSolidFacts,
    vertices: &mut Vec<Point3>,
    triangles: &mut Vec<Triangle>,
) -> Option<()> {
    for right_triangle in right.view().triangles() {
        let [a, b, c] = right_triangle.vertices().ok()?;
        let source_points = [a.clone(), b.clone(), c.clone()];
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
    let projection = choose_nonzero_projected_polygon_area(source_points)?;
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
    for clip_face in &clip.facts().faces {
        inside = clip_polygon_by_face(&inside, clip, clip_face.triangle.vertices, clip_facts)?;
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
            ExactArrangement2dRegionRing {
                region: ExactArrangement2dRegion::Left,
                vertices: source_projected,
            },
            ExactArrangement2dRegionRing {
                region: ExactArrangement2dRegion::Right,
                vertices: inside_projected,
            },
        ],
        ExactArrangement2dSetOperation::Difference,
        ExactArrangement2dBoundaryPolicy::PreserveCollinear,
    );
    if !overlay.blockers.is_empty() {
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
    for clip_face in &clip.facts().faces {
        inside = clip_polygon_by_face(&inside, clip, clip_face.triangle.vertices, clip_facts)?;
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

    let projection = choose_nonzero_projected_polygon_area(&inside)?;
    let source_sign = compare_reals(
        &projected_polygon_area2_value(&inside, projection),
        &Real::from(0),
    )
    .value()?;
    if source_sign == Ordering::Equal {
        return None;
    }
    let reversed_sign = match source_sign {
        Ordering::Less => Ordering::Greater,
        Ordering::Equal => Ordering::Equal,
        Ordering::Greater => Ordering::Less,
    };
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
    clip.facts().faces.iter().any(|face| {
        let [a, b, c] = face.triangle.vertices;
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
    let p1x = carrier[1].x.clone() - &carrier[0].x;
    let p1y = carrier[1].y.clone() - &carrier[0].y;
    let p1z = carrier[1].z.clone() - &carrier[0].z;
    let p2x = carrier[2].x.clone() - &carrier[0].x;
    let p2y = carrier[2].y.clone() - &carrier[0].y;
    let p2z = carrier[2].z.clone() - &carrier[0].z;
    Some(Point3::new(
        carrier[0].x.clone() + &(p1x * &a) + &(p2x * &b),
        carrier[0].y.clone() + &(p1y * &a) + &(p2y * &b),
        carrier[0].z.clone() + &(p1z * &a) + &(p2z * &b),
    ))
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
    let x_start = compare_reals(&point.x, &start.x).value()?;
    let x_end = compare_reals(&point.x, &end.x).value()?;
    let y_start = compare_reals(&point.y, &start.y).value()?;
    let y_end = compare_reals(&point.y, &end.y).value()?;
    let z_start = compare_reals(&point.z, &start.z).value()?;
    let z_end = compare_reals(&point.z, &end.z).value()?;

    Some(
        matches!(
            (x_start, x_end),
            (
                Ordering::Greater | Ordering::Equal,
                Ordering::Less | Ordering::Equal
            ) | (
                Ordering::Less | Ordering::Equal,
                Ordering::Greater | Ordering::Equal
            )
        ) && matches!(
            (y_start, y_end),
            (
                Ordering::Greater | Ordering::Equal,
                Ordering::Less | Ordering::Equal
            ) | (
                Ordering::Less | Ordering::Equal,
                Ordering::Greater | Ordering::Equal
            )
        ) && matches!(
            (z_start, z_end),
            (
                Ordering::Greater | Ordering::Equal,
                Ordering::Less | Ordering::Equal
            ) | (
                Ordering::Less | Ordering::Equal,
                Ordering::Greater | Ordering::Equal
            )
        ),
    )
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
    let abx = bx - ax;
    let aby = by - ay;
    let acx = cx - ax;
    let acy = cy - ay;
    &(&abx * &acy) - &(&aby * &acx)
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
        (point3_exact_equal(&existing[0], &segment[0]) == Some(true)
            && point3_exact_equal(&existing[1], &segment[1]) == Some(true))
            || (point3_exact_equal(&existing[0], &segment[1]) == Some(true)
                && point3_exact_equal(&existing[1], &segment[0]) == Some(true))
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
        if point3_exact_equal(&last, &polygon[0]) == Some(true) {
            break;
        }
        let (index, reverse) = segments.iter().enumerate().find_map(|(index, segment)| {
            if point3_exact_equal(&segment[0], &last) == Some(true) {
                Some((index, false))
            } else if point3_exact_equal(&segment[1], &last) == Some(true) {
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
    if !segments.is_empty() || point3_exact_equal(polygon.first()?, polygon.last()?) != Some(true) {
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
    let denominator = &d0 - &d1;
    if compare_reals(&denominator, &Real::from(0)).value() == Some(Ordering::Equal) {
        return None;
    }
    let t = (d0 / &denominator).ok()?;
    Some(interpolate3(p0, p1, &t))
}

fn orient3d_value(a: &Point3, b: &Point3, c: &Point3, d: &Point3) -> Real {
    let adx = &a.x - &d.x;
    let ady = &a.y - &d.y;
    let adz = &a.z - &d.z;
    let bdx = &b.x - &d.x;
    let bdy = &b.y - &d.y;
    let bdz = &b.z - &d.z;
    let cdx = &c.x - &d.x;
    let cdy = &c.y - &d.y;
    let cdz = &c.z - &d.z;

    let minor_z = &(&bdx * &cdy) - &(&cdx * &bdy);
    let minor_y = &(&cdx * &ady) - &(&adx * &cdy);
    let minor_x = &(&adx * &bdy) - &(&bdx * &ady);

    let z_term = &adz * &minor_z;
    let y_term = &bdz * &minor_y;
    let x_term = &cdz * &minor_x;
    &(&z_term + &y_term) + &x_term
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
        let projection = choose_nonzero_projected_polygon_area(&[
            vertices[triangle.0[0]].clone(),
            vertices[triangle.0[1]].clone(),
            vertices[triangle.0[2]].clone(),
        ])?;
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
                if point3_exact_equal(&vertices[candidate], &vertices[start]) == Some(true)
                    || point3_exact_equal(&vertices[candidate], &vertices[end]) == Some(true)
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
        .position(|existing| point3_exact_equal(existing, point) == Some(true))
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

fn simplify_polygon(points: &mut Vec<Point3>) {
    points.dedup_by(|right, left| point3_exact_equal(left, right) == Some(true));
    if points.len() > 1 && point3_exact_equal(&points[0], &points[points.len() - 1]) == Some(true) {
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
    let abx = &b.x - &a.x;
    let aby = &b.y - &a.y;
    let abz = &b.z - &a.z;
    let acx = &c.x - &a.x;
    let acy = &c.y - &a.y;
    let acz = &c.z - &a.z;
    let cross_x = &(&aby * &acz) - &(&abz * &acy);
    let cross_y = &(&abz * &acx) - &(&abx * &acz);
    let cross_z = &(&abx * &acy) - &(&aby * &acx);
    compare_reals(&cross_x, &Real::from(0)).value() == Some(Ordering::Equal)
        && compare_reals(&cross_y, &Real::from(0)).value() == Some(Ordering::Equal)
        && compare_reals(&cross_z, &Real::from(0)).value() == Some(Ordering::Equal)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::boolean::{
        ExactBooleanOperation, ExactBooleanRequest,
        exact_boolean_evaluation_for_replay_result_with_materialization,
    };

    fn with_materialized_evaluation_for_test<R>(
        left: &ExactMesh,
        right: &ExactMesh,
        request: ExactBooleanRequest,
        f: impl FnOnce(&crate::mesh::boolean::ExactBooleanEvaluation) -> R,
    ) -> R {
        let evaluation = exact_boolean_evaluation_for_replay_result_with_materialization(
            left, right, request, true,
        )
        .unwrap();
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
            .expect("certified convex union should not hit exact construction blockers")
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
            .expect("certified convex difference should not hit exact construction blockers")
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
