//! Exact closed-convex boolean fragments.
//!
//! This module handles certified object-level closed-convex boolean fragments.
//! Each output face is produced by clipping a source face polygon against the
//! other solid's exact oriented halfspaces, then the resulting triangle mesh is
//! revalidated through [`Mesh`]. Boolean topology is emitted only from
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
use super::super::error::{MeshBlocker, MeshBlockerKind, MeshError};
use super::super::triangle_edges;
use super::super::validation::MeshValidationPolicy;
use super::super::{
    Mesh, Triangle, exact_points_are_collinear, orient_paired_triangle_edges,
    remove_duplicate_triangle_vertex_sets,
};
use super::solid::{
    ClosedMeshOrientation, ConvexSolidFacts, certify_convex_solid, determinant_from_origin,
};
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
    pub mesh: Mesh,
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
    pub mesh: Mesh,
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
    pub mesh: Mesh,
}

fn validate_convex_boolean_output(
    operation: &'static str,
    left_facts: &ConvexSolidFacts,
    right_facts: &ConvexSolidFacts,
    mesh: &Mesh,
) -> Result<(), MeshError> {
    left_facts.validate().map_err(|error| {
        MeshError::one(MeshBlocker::new(
            MeshBlockerKind::ExactConstructionFailure,
            format!("invalid convex-solid facts retained by convex boolean: {error:?}"),
        ))
    })?;
    right_facts.validate().map_err(|error| {
        MeshError::one(MeshBlocker::new(
            MeshBlockerKind::ExactConstructionFailure,
            format!("invalid convex-solid facts retained by convex boolean: {error:?}"),
        ))
    })?;
    if !left_facts.is_certified_convex() || !right_facts.is_certified_convex() {
        return Err(MeshError::one(MeshBlocker::new(
            MeshBlockerKind::ExactConstructionFailure,
            format!("convex {operation} retained non-certified solid facts"),
        )));
    }
    mesh.validate_retained_state().map_err(|error| {
        MeshError::one(MeshBlocker::new(
            MeshBlockerKind::ExactConstructionFailure,
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
    left: &Mesh,
    right: &Mesh,
) -> Result<Option<ConvexSolidUnion>, MeshError> {
    let left_facts = certify_convex_solid(left);
    let right_facts = certify_convex_solid(right);
    if !left_facts.is_certified_convex() || !right_facts.is_certified_convex() {
        return Ok(None);
    }

    let mut vertices = Vec::new();
    let mut triangles = Vec::new();
    if !append_convex_union_source_faces(left, right, &right_facts, &mut vertices, &mut triangles)?
        || !append_convex_union_source_faces(
            right,
            left,
            &left_facts,
            &mut vertices,
            &mut triangles,
        )?
    {
        return Ok(None);
    }
    if triangles.is_empty() {
        return Ok(None);
    }
    if !refine_triangles_at_existing_edge_vertices(&vertices, &mut triangles)? {
        return Ok(None);
    }
    remove_duplicate_triangle_vertex_sets(&mut triangles);
    if orient_paired_triangle_edges(&mut triangles).is_none() {
        return Ok(None);
    }
    let direct_mesh = Mesh::new_with_policy_and_version(
        vertices.clone(),
        triangles.clone(),
        SourceProvenance::exact("exact closed-convex solid union"),
        MeshValidationPolicy::CLOSED,
        1,
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
    validate_convex_boolean_output("union", &union.left_facts, &union.right_facts, &union.mesh)?;
    Ok(Some(union))
}

/// Certify and materialize `left - right` for two closed convex solids.
///
/// This is the exact source-face-cell construction for general convex
/// difference. It returns `None` for empty/lower-dimensional results or when
/// exact face-cell triangulation cannot be certified.
pub(crate) fn subtract_closed_convex_solids(
    left: &Mesh,
    right: &Mesh,
) -> Result<Option<ConvexSolidDifference>, MeshError> {
    let left_facts = certify_convex_solid(left);
    let right_facts = certify_convex_solid(right);
    if !left_facts.is_certified_convex() || !right_facts.is_certified_convex() {
        return Ok(None);
    }

    let mut vertices = Vec::new();
    let mut triangles = Vec::new();
    if !append_convex_union_source_faces(left, right, &right_facts, &mut vertices, &mut triangles)?
        || !append_convex_difference_right_faces(
            right,
            left,
            &left_facts,
            &mut vertices,
            &mut triangles,
        )?
    {
        return Ok(None);
    }
    if triangles.is_empty() {
        return Ok(None);
    }
    if !refine_triangles_at_existing_edge_vertices(&vertices, &mut triangles)? {
        return Ok(None);
    }
    remove_duplicate_triangle_vertex_sets(&mut triangles);
    if orient_paired_triangle_edges(&mut triangles).is_none() {
        return Ok(None);
    }
    let mesh = Mesh::new_with_policy_and_version(
        vertices,
        triangles,
        SourceProvenance::exact("exact closed-convex solid difference"),
        MeshValidationPolicy::CLOSED,
        1,
    )?;
    if !mesh_has_nonzero_signed_volume(&mesh)? {
        return Ok(None);
    }
    let difference = ConvexSolidDifference {
        left_facts,
        right_facts,
        mesh,
    };
    validate_convex_boolean_output(
        "difference",
        &difference.left_facts,
        &difference.right_facts,
        &difference.mesh,
    )?;
    Ok(Some(difference))
}

fn union_from_difference_and_operand(left: &Mesh, right: &Mesh) -> Result<Option<Mesh>, MeshError> {
    let Some(difference) = subtract_closed_convex_solids(left, right)? else {
        return Ok(None);
    };
    let mut vertices = difference.mesh.view().vertices().to_vec();
    let mut triangles = difference
        .mesh
        .view()
        .faces()
        .map(|face| Triangle(face.vertex_indices()))
        .collect::<Vec<_>>();
    let right_vertex_map = right
        .view()
        .vertices()
        .iter()
        .map(|point| intern_point(&mut vertices, point))
        .collect::<Result<Vec<_>, _>>()?;
    triangles.extend(right.view().faces().map(|face| {
        let vertices = face.vertex_indices();
        Triangle([
            right_vertex_map[vertices[0]],
            right_vertex_map[vertices[1]],
            right_vertex_map[vertices[2]],
        ])
    }));
    if !refine_triangles_at_existing_edge_vertices(&vertices, &mut triangles)? {
        return Ok(None);
    }
    remove_duplicate_triangle_vertex_sets(&mut triangles);
    if orient_paired_triangle_edges(&mut triangles).is_none() {
        return Ok(None);
    }
    let mesh = Mesh::new_with_policy_and_version(
        vertices,
        triangles,
        SourceProvenance::exact("exact closed-convex solid union from exact difference"),
        MeshValidationPolicy::CLOSED,
        1,
    )?;
    Ok(Some(mesh))
}

fn close_planar_boundary_loops(
    vertices: Vec<Point3>,
    mut triangles: Vec<Triangle>,
) -> Option<Mesh> {
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
    Mesh::new_with_policy_and_version(
        vertices,
        triangles,
        SourceProvenance::exact("exact closed-convex solid union with exact planar caps"),
        MeshValidationPolicy::CLOSED,
        1,
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
    if exact_points_are_collinear(a, b, c) == Some(true) {
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
    left: &Mesh,
    right: &Mesh,
) -> Result<Option<ConvexSolidIntersection>, MeshError> {
    let left_facts = certify_convex_solid(left);
    let right_facts = certify_convex_solid(right);
    if !left_facts.is_certified_convex() || !right_facts.is_certified_convex() {
        return Ok(None);
    }

    let mut polygons = Vec::new();
    if !clipped_source_faces(left, right, &right_facts, &mut polygons)?
        || !clipped_source_faces(right, left, &left_facts, &mut polygons)?
    {
        return Ok(None);
    }
    if polygons.is_empty() {
        return Ok(None);
    }

    let Some(hull_polygons) = convex_hull_polygons_from_clipped_faces(&polygons)? else {
        return Ok(None);
    };
    let Some(mesh) = polygons_to_closed_mesh(
        &hull_polygons,
        "exact closed-convex solid intersection",
        MeshValidationPolicy::CLOSED,
    )?
    else {
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
    validate_convex_boolean_output(
        "intersection",
        &intersection.left_facts,
        &intersection.right_facts,
        &intersection.mesh,
    )?;
    Ok(Some(intersection))
}

/// Return whether the retained triangle shell encloses nonzero exact volume.
///
/// This guard is what keeps closed-convex intersection from promoting
/// only after an exact predicate, here the signed tetrahedral volume sum,
/// proves the result is a solid instead of a lower-dimensional boundary.
fn mesh_has_nonzero_signed_volume(mesh: &Mesh) -> Result<bool, MeshError> {
    let mut signed_volume = Real::from(0);
    for face in mesh.view().faces() {
        let [a, b, c] = face.vertices()?;
        signed_volume = &signed_volume + &determinant_from_origin(a, b, c);
    }

    let Some(ordering) = compare_reals(&signed_volume, &Real::from(0)).value() else {
        return Err(MeshError::one(MeshBlocker::new(
            MeshBlockerKind::ExactConstructionFailure,
            "convex output signed-volume comparison was undecidable",
        )));
    };
    Ok(ordering != Ordering::Equal)
}

fn clipped_source_faces(
    source: &Mesh,
    clip: &Mesh,
    clip_facts: &ConvexSolidFacts,
    polygons: &mut Vec<Vec<Point3>>,
) -> Result<bool, MeshError> {
    for source_triangle in source.view().triangles() {
        let Ok([a, b, c]) = source_triangle.vertices() else {
            return Ok(false);
        };
        let mut polygon = vec![a.clone(), b.clone(), c.clone()];
        for clip_face in clip.view().faces() {
            let Ok(vertices) = clip_face.vertices() else {
                return Ok(false);
            };
            let Some(clipped) = clip_polygon_by_face(&polygon, vertices, clip_facts)? else {
                return Ok(false);
            };
            polygon = clipped;
            if polygon.len() < 3 {
                break;
            }
        }
        simplify_polygon(&mut polygon)?;
        if polygon.len() >= 3 && !polygon_is_degenerate(&polygon) {
            polygons.push(polygon);
        }
    }
    Ok(true)
}

fn clip_polygon_by_face(
    polygon: &[Point3],
    face: [&Point3; 3],
    clip_facts: &ConvexSolidFacts,
) -> Result<Option<Vec<Point3>>, MeshError> {
    if polygon.is_empty() {
        return Ok(Some(Vec::new()));
    }
    let [a, b, c] = face;

    let mut output = Vec::new();
    let orientation = clip_facts.orientation;
    let Some(mut previous) = polygon.last().cloned() else {
        return Ok(None);
    };
    let Some(previous_side) = point_side(a, b, c, &previous) else {
        return Err(MeshError::one(MeshBlocker::new(
            MeshBlockerKind::ExactConstructionFailure,
            "convex clipping plane-side predicate is undecidable",
        )));
    };
    let mut previous_inside = !matches!(
        (orientation, previous_side),
        (ClosedMeshOrientation::Positive, PlaneSide::Below)
            | (ClosedMeshOrientation::Negative, PlaneSide::Above)
    );
    for current in polygon {
        let Some(current_side) = point_side(a, b, c, current) else {
            return Err(MeshError::one(MeshBlocker::new(
                MeshBlockerKind::ExactConstructionFailure,
                "convex clipping plane-side predicate is undecidable",
            )));
        };
        let current_inside = !matches!(
            (orientation, current_side),
            (ClosedMeshOrientation::Positive, PlaneSide::Below)
                | (ClosedMeshOrientation::Negative, PlaneSide::Above)
        );
        match (previous_inside, current_inside) {
            (true, true) => output.push(current.clone()),
            (true, false) => {
                let Some(intersection) = intersect_segment_with_plane(&previous, current, a, b, c)
                else {
                    return Ok(None);
                };
                output.push(intersection);
            }
            (false, true) => {
                let Some(intersection) = intersect_segment_with_plane(&previous, current, a, b, c)
                else {
                    return Ok(None);
                };
                output.push(intersection);
                output.push(current.clone());
            }
            (false, false) => {}
        }
        previous = current.clone();
        previous_inside = current_inside;
    }
    simplify_polygon(&mut output)?;
    Ok(Some(output))
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
    source: &Mesh,
    clip: &Mesh,
    clip_facts: &ConvexSolidFacts,
    vertices: &mut Vec<Point3>,
    triangles: &mut Vec<Triangle>,
) -> Result<bool, MeshError> {
    for source_triangle in source.view().triangles() {
        let Ok([a, b, c]) = source_triangle.vertices() else {
            return Ok(false);
        };
        let source_points = [a.clone(), b.clone(), c.clone()];
        if !append_source_face_minus_convex_inside(
            &source_points,
            clip,
            clip_facts,
            vertices,
            triangles,
        )? {
            return Ok(false);
        }
    }
    Ok(true)
}

fn append_convex_difference_right_faces(
    right: &Mesh,
    left: &Mesh,
    left_facts: &ConvexSolidFacts,
    vertices: &mut Vec<Point3>,
    triangles: &mut Vec<Triangle>,
) -> Result<bool, MeshError> {
    for right_triangle in right.view().triangles() {
        let Ok([a, b, c]) = right_triangle.vertices() else {
            return Ok(false);
        };
        let source_points = [a.clone(), b.clone(), c.clone()];
        if !append_source_face_convex_inside_reversed(
            &source_points,
            left,
            left_facts,
            vertices,
            triangles,
        )? {
            return Ok(false);
        }
    }
    Ok(true)
}

fn append_source_face_minus_convex_inside(
    source_points: &[Point3],
    clip: &Mesh,
    clip_facts: &ConvexSolidFacts,
    vertices: &mut Vec<Point3>,
    triangles: &mut Vec<Triangle>,
) -> Result<bool, MeshError> {
    if source_points.len() < 3 || polygon_is_degenerate(source_points) {
        return Ok(true);
    }
    let Some(projection) = choose_nonzero_projected_polygon_area(source_points) else {
        return Ok(false);
    };
    let source_projected = source_points
        .iter()
        .map(|point| project_point3(point, projection))
        .collect::<Vec<_>>();
    let Some(source_sign) = compare_reals(
        &projected_polygon_area2_value(source_points, projection),
        &Real::from(0),
    )
    .value() else {
        return Ok(false);
    };
    if source_sign == Ordering::Equal {
        return Ok(false);
    }

    let mut inside = source_points.to_vec();
    for clip_face in clip.view().faces() {
        let Ok(vertices) = clip_face.vertices() else {
            return Ok(false);
        };
        let Some(clipped) = clip_polygon_by_face(&inside, vertices, clip_facts)? else {
            return Ok(false);
        };
        inside = clipped;
        if inside.len() < 3 {
            break;
        }
    }
    simplify_polygon(&mut inside)?;
    if inside.len() < 3 || polygon_is_degenerate(&inside) {
        return append_projected_polygon_triangles(
            &source_projected,
            source_points,
            projection,
            source_sign,
            vertices,
            triangles,
        );
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
        return Ok(false);
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
    clip: &Mesh,
    clip_facts: &ConvexSolidFacts,
    vertices: &mut Vec<Point3>,
    triangles: &mut Vec<Triangle>,
) -> Result<bool, MeshError> {
    if source_points.len() < 3 || polygon_is_degenerate(source_points) {
        return Ok(true);
    }

    let mut inside = source_points.to_vec();
    for clip_face in clip.view().faces() {
        let Ok(vertices) = clip_face.vertices() else {
            return Ok(false);
        };
        let Some(clipped) = clip_polygon_by_face(&inside, vertices, clip_facts)? else {
            return Ok(false);
        };
        inside = clipped;
        if inside.len() < 3 {
            break;
        }
    }
    simplify_polygon(&mut inside)?;
    if inside.len() < 3 || polygon_is_degenerate(&inside) {
        return Ok(true);
    }
    let Some(lies_on_boundary) = polygon_lies_on_any_clip_boundary_face(&inside, clip) else {
        return Ok(false);
    };
    if lies_on_boundary {
        return Ok(true);
    }

    let Some(projection) = choose_nonzero_projected_polygon_area(&inside) else {
        return Ok(false);
    };
    let Some(source_sign) = compare_reals(
        &projected_polygon_area2_value(&inside, projection),
        &Real::from(0),
    )
    .value() else {
        return Ok(false);
    };
    if source_sign == Ordering::Equal {
        return Ok(false);
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

fn polygon_lies_on_any_clip_boundary_face(polygon: &[Point3], clip: &Mesh) -> Option<bool> {
    for face in clip.view().faces() {
        let [a, b, c] = face.vertices().ok()?;
        if polygon
            .iter()
            .all(|point| point_side(a, b, c, point) == Some(PlaneSide::On))
        {
            return Some(true);
        }
    }
    Some(false)
}

fn append_projected_polygon_triangles(
    projected_points: &[Point2],
    carrier_points: &[Point3],
    projection: CoplanarProjection,
    source_sign: Ordering,
    vertices: &mut Vec<Point3>,
    triangles: &mut Vec<Triangle>,
) -> Result<bool, MeshError> {
    let projected = projected_points
        .iter()
        .map(point2_for_hypertri)
        .collect::<Vec<_>>();
    let indices = match hypertri::earcut(&projected, &[]) {
        Ok(indices) if !indices.is_empty() && indices.len() % 3 == 0 => indices,
        _ => return Ok(false),
    };
    let Some(lifted) =
        lift_projected_points_to_carrier(projected_points.iter(), carrier_points, projection)
    else {
        return Ok(false);
    };
    let local_to_global = intern_points(vertices, &lifted)?;
    append_oriented_earcut_triangles(&indices, &local_to_global, source_sign, triangles);
    Ok(true)
}

fn append_projected_overlay_triangles(
    overlay: &ExactArrangement2dOverlay,
    carrier_points: &[Point3],
    projection: CoplanarProjection,
    source_sign: Ordering,
    vertices: &mut Vec<Point3>,
    triangles: &mut Vec<Triangle>,
) -> Result<bool, MeshError> {
    for component in &overlay.output_components {
        let Some(outer) = overlay.output_loops.get(component.outer_loop) else {
            return Ok(false);
        };
        let mut projected = outer
            .points
            .iter()
            .map(point2_for_hypertri)
            .collect::<Vec<_>>();
        let Some(mut lifted) = lifted_output_loop_points(outer, carrier_points, projection) else {
            return Ok(false);
        };
        let mut hole_indices = Vec::with_capacity(component.hole_loops.len());
        for &hole_loop in &component.hole_loops {
            let Some(hole) = overlay.output_loops.get(hole_loop) else {
                return Ok(false);
            };
            hole_indices.push(projected.len());
            projected.extend(hole.points.iter().map(point2_for_hypertri));
            let Some(hole_lifted) = lifted_output_loop_points(hole, carrier_points, projection)
            else {
                return Ok(false);
            };
            lifted.extend(hole_lifted);
        }
        let local_to_global = intern_points(vertices, &lifted)?;
        let indices = match hypertri::earcut(&projected, &hole_indices) {
            Ok(indices) if !indices.is_empty() && indices.len() % 3 == 0 => indices,
            _ => return Ok(false),
        };
        append_oriented_earcut_triangles(&indices, &local_to_global, source_sign, triangles);
    }
    Ok(true)
}

fn lifted_output_loop_points(
    loop_: &ExactArrangement2dOutputLoop,
    carrier_points: &[Point3],
    projection: CoplanarProjection,
) -> Option<Vec<Point3>> {
    lift_projected_points_to_carrier(loop_.points.iter(), carrier_points, projection)
}

fn lift_projected_points_to_carrier<'a>(
    points: impl IntoIterator<Item = &'a Point2>,
    carrier_points: &[Point3],
    projection: CoplanarProjection,
) -> Option<Vec<Point3>> {
    points
        .into_iter()
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

fn convex_hull_polygons_from_clipped_faces(
    polygons: &[Vec<Point3>],
) -> Result<Option<Vec<Vec<Point3>>>, MeshError> {
    let mut points = Vec::new();
    for polygon in polygons {
        for point in polygon {
            intern_point(&mut points, point)?;
        }
    }
    if points.len() < 4 {
        return Ok(None);
    }
    let Some(interior) = polygon_centroid(&points) else {
        return Ok(None);
    };
    let mut seen_faces = BTreeSet::new();
    let mut hull_faces = Vec::new();
    for a in 0..points.len() {
        for b in a + 1..points.len() {
            for c in b + 1..points.len() {
                if exact_points_are_collinear(&points[a], &points[b], &points[c]) == Some(true) {
                    continue;
                }
                let mut saw_above = false;
                let mut saw_below = false;
                let mut coplanar = Vec::new();
                for (index, point) in points.iter().enumerate() {
                    let Some(side) = point_side(&points[a], &points[b], &points[c], point) else {
                        return Ok(None);
                    };
                    match side {
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
                let Some(mut face) = convex_face_polygon_from_indices(&points, &coplanar)? else {
                    return Ok(None);
                };
                if orient_face_polygon_outward(&mut face, &interior).is_none() {
                    return Ok(None);
                }
                hull_faces.push(face);
            }
        }
    }
    if hull_faces.len() >= 4 {
        Ok(Some(hull_faces))
    } else {
        Ok(None)
    }
}

fn convex_face_polygon_from_indices(
    points: &[Point3],
    indices: &[usize],
) -> Result<Option<Vec<Point3>>, MeshError> {
    if indices.len() == 3 {
        return Ok(Some(
            indices.iter().map(|&index| points[index].clone()).collect(),
        ));
    }
    let Some(projection) = choose_face_projection(points, indices) else {
        return Ok(None);
    };
    let mut segments = Vec::new();
    for (left_offset, &left) in indices.iter().enumerate() {
        for &right in &indices[left_offset + 1..] {
            let Some(is_boundary) =
                convex_face_pair_is_boundary_edge(points, indices, left, right, projection)
            else {
                return Ok(None);
            };
            if is_boundary {
                push_unique_segment(&mut segments, [points[left].clone(), points[right].clone()])?;
            }
        }
    }
    let Some(mut polygon) = chain_segments_to_polygon(segments)? else {
        return Ok(None);
    };
    remove_collinear_polygon_vertices(&mut polygon, projection);
    if polygon.len() >= 3 && !polygon_is_degenerate(&polygon) {
        Ok(Some(polygon))
    } else {
        Ok(None)
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
        if exact_points_are_collinear(&points[left], &points[right], &points[candidate])
            == Some(true)
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

fn convex_points_equal(
    left: &Point3,
    right: &Point3,
    context: &'static str,
) -> Result<bool, MeshError> {
    point3_exact_equal(left, right).ok_or_else(|| {
        MeshError::one(MeshBlocker::new(
            MeshBlockerKind::ExactConstructionFailure,
            context,
        ))
    })
}

fn ordered_segments_equal(left: &[Point3; 2], right: &[Point3; 2]) -> Option<bool> {
    let start = point3_exact_equal(&left[0], &right[0]);
    let end = point3_exact_equal(&left[1], &right[1]);
    if start == Some(false) || end == Some(false) {
        return Some(false);
    }
    if start == Some(true) && end == Some(true) {
        return Some(true);
    }
    None
}

fn unordered_segments_equal(left: &[Point3; 2], right: &[Point3; 2]) -> Result<bool, MeshError> {
    let forward = ordered_segments_equal(left, right);
    if forward == Some(true) {
        return Ok(true);
    }
    let reversed = [right[1].clone(), right[0].clone()];
    let reverse = ordered_segments_equal(left, &reversed);
    if reverse == Some(true) {
        return Ok(true);
    }
    if forward == Some(false) && reverse == Some(false) {
        return Ok(false);
    }
    Err(MeshError::one(MeshBlocker::new(
        MeshBlockerKind::ExactConstructionFailure,
        "convex hull duplicate segment equality is undecidable",
    )))
}

fn push_unique_segment(
    segments: &mut Vec<[Point3; 2]>,
    segment: [Point3; 2],
) -> Result<(), MeshError> {
    for existing in segments.iter() {
        if unordered_segments_equal(existing, &segment)? {
            return Ok(());
        }
    }
    segments.push(segment);
    Ok(())
}

fn connecting_segment(
    segments: &[[Point3; 2]],
    point: &Point3,
) -> Result<Option<(usize, bool)>, MeshError> {
    let mut saw_undecidable = false;
    for (index, segment) in segments.iter().enumerate() {
        match point3_exact_equal(&segment[0], point) {
            Some(true) => return Ok(Some((index, false))),
            Some(false) => {}
            None => saw_undecidable = true,
        }
        match point3_exact_equal(&segment[1], point) {
            Some(true) => return Ok(Some((index, true))),
            Some(false) => {}
            None => saw_undecidable = true,
        }
    }
    if saw_undecidable {
        Err(MeshError::one(MeshBlocker::new(
            MeshBlockerKind::ExactConstructionFailure,
            "convex hull segment chain endpoint equality is undecidable",
        )))
    } else {
        Ok(None)
    }
}

fn chain_segments_to_polygon(
    mut segments: Vec<[Point3; 2]>,
) -> Result<Option<Vec<Point3>>, MeshError> {
    let Some(first) = segments.pop() else {
        return Ok(None);
    };
    let mut polygon = vec![first[0].clone(), first[1].clone()];
    while !segments.is_empty() {
        let Some(last) = polygon.last().cloned() else {
            return Ok(None);
        };
        let closure = point3_exact_equal(&last, &polygon[0]);
        if closure == Some(true) {
            break;
        }
        let Some((index, reverse)) = connecting_segment(&segments, &last)? else {
            if closure.is_none() {
                return Err(MeshError::one(MeshBlocker::new(
                    MeshBlockerKind::ExactConstructionFailure,
                    "convex hull polygon closure equality is undecidable",
                )));
            }
            return Ok(None);
        };
        let segment = segments.swap_remove(index);
        polygon.push(if reverse {
            segment[0].clone()
        } else {
            segment[1].clone()
        });
    }
    let Some(first) = polygon.first() else {
        return Ok(None);
    };
    let Some(last) = polygon.last() else {
        return Ok(None);
    };
    if !segments.is_empty()
        || !convex_points_equal(first, last, "convex hull polygon final closure equality")?
    {
        return Ok(None);
    }
    polygon.pop();
    simplify_polygon(&mut polygon)?;
    if polygon.len() >= 3 && !polygon_is_degenerate(&polygon) {
        Ok(Some(polygon))
    } else {
        Ok(None)
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
    validation: MeshValidationPolicy,
) -> Result<Option<Mesh>, MeshError> {
    let mut vertices: Vec<Point3> = Vec::new();
    let mut triangles = Vec::new();
    for polygon in polygons {
        let base = intern_point(&mut vertices, &polygon[0])?;
        for index in 1..polygon.len() - 1 {
            let b = intern_point(&mut vertices, &polygon[index])?;
            let c = intern_point(&mut vertices, &polygon[index + 1])?;
            if base != b && b != c && c != base {
                triangles.push(Triangle([base, b, c]));
            }
        }
    }
    remove_duplicate_triangle_vertex_sets(&mut triangles);
    if triangles.is_empty() {
        return Ok(None);
    }
    Mesh::new_with_policy_and_version(
        vertices,
        triangles,
        SourceProvenance::exact(label),
        validation,
        1,
    )
    .map(Some)
}

fn refine_triangles_at_existing_edge_vertices(
    vertices: &[Point3],
    triangles: &mut Vec<Triangle>,
) -> Result<bool, MeshError> {
    let mut guard = 0usize;
    loop {
        guard += 1;
        if guard
            > vertices
                .len()
                .saturating_mul(triangles.len().saturating_add(1))
        {
            return Ok(false);
        }
        let (triangle_index, edge, vertex) =
            match find_triangle_edge_split(vertices, triangles.as_slice())? {
                TriangleEdgeSplitSearch::Split {
                    triangle_index,
                    edge,
                    vertex,
                } => (triangle_index, edge, vertex),
                TriangleEdgeSplitSearch::Complete => return Ok(true),
                TriangleEdgeSplitSearch::Unsupported => return Ok(false),
            };
        let Some(original) = triangles.get(triangle_index).copied() else {
            return Ok(false);
        };
        let a = original.0[edge];
        let b = original.0[(edge + 1) % 3];
        let c = original.0[(edge + 2) % 3];
        triangles.splice(
            triangle_index..triangle_index + 1,
            [Triangle([a, vertex, c]), Triangle([vertex, b, c])],
        );
    }
}

enum TriangleEdgeSplitSearch {
    Split {
        triangle_index: usize,
        edge: usize,
        vertex: usize,
    },
    Complete,
    Unsupported,
}

fn find_triangle_edge_split(
    vertices: &[Point3],
    triangles: &[Triangle],
) -> Result<TriangleEdgeSplitSearch, MeshError> {
    for (triangle_index, triangle) in triangles.iter().enumerate() {
        let Some(projection) = choose_nonzero_projected_polygon_area(&[
            vertices[triangle.0[0]].clone(),
            vertices[triangle.0[1]].clone(),
            vertices[triangle.0[2]].clone(),
        ]) else {
            return Ok(TriangleEdgeSplitSearch::Unsupported);
        };
        let [ta, tb, tc] = triangle.0;
        for edge in 0..3 {
            let start = triangle.0[edge];
            let end = triangle.0[(edge + 1) % 3];
            for candidate in 0..vertices.len() {
                if candidate == ta || candidate == tb || candidate == tc {
                    continue;
                }
                let Some(side) = orient3d_report(
                    &vertices[ta],
                    &vertices[tb],
                    &vertices[tc],
                    &vertices[candidate],
                )
                .value() else {
                    return Err(MeshError::one(MeshBlocker::new(
                        MeshBlockerKind::ExactConstructionFailure,
                        "convex triangle refinement plane-side predicate is undecidable",
                    )));
                };
                if PlaneSide::from(side) != PlaneSide::On {
                    continue;
                }
                if point_equals_either_endpoint(
                    &vertices[candidate],
                    &vertices[start],
                    &vertices[end],
                )? {
                    continue;
                }
                let Some(on_segment) = point_on_segment(
                    &project_point3(&vertices[start], projection),
                    &project_point3(&vertices[end], projection),
                    &project_point3(&vertices[candidate], projection),
                )
                .value() else {
                    return Err(MeshError::one(MeshBlocker::new(
                        MeshBlockerKind::ExactConstructionFailure,
                        "convex triangle refinement segment predicate is undecidable",
                    )));
                };
                if on_segment {
                    return Ok(TriangleEdgeSplitSearch::Split {
                        triangle_index,
                        edge,
                        vertex: candidate,
                    });
                }
            }
        }
    }
    Ok(TriangleEdgeSplitSearch::Complete)
}

fn point_equals_either_endpoint(
    point: &Point3,
    start: &Point3,
    end: &Point3,
) -> Result<bool, MeshError> {
    let start_equal = point3_exact_equal(point, start);
    let end_equal = point3_exact_equal(point, end);
    if start_equal == Some(true) || end_equal == Some(true) {
        return Ok(true);
    }
    if start_equal == Some(false) && end_equal == Some(false) {
        return Ok(false);
    }
    Err(MeshError::one(MeshBlocker::new(
        MeshBlockerKind::ExactConstructionFailure,
        "convex triangle refinement endpoint equality is undecidable",
    )))
}

fn intern_point(vertices: &mut Vec<Point3>, point: &Point3) -> Result<usize, MeshError> {
    for (index, existing) in vertices.iter().enumerate() {
        if convex_points_equal(existing, point, "convex point interning equality")? {
            return Ok(index);
        }
    }
    vertices.push(Point3::new(
        point.x.clone(),
        point.y.clone(),
        point.z.clone(),
    ));
    Ok(vertices.len() - 1)
}

fn intern_points(vertices: &mut Vec<Point3>, points: &[Point3]) -> Result<Vec<usize>, MeshError> {
    points
        .iter()
        .map(|point| intern_point(vertices, point))
        .collect()
}

fn simplify_polygon(points: &mut Vec<Point3>) -> Result<(), MeshError> {
    let mut index = 0;
    while index + 1 < points.len() {
        if convex_points_equal(
            &points[index],
            &points[index + 1],
            "convex polygon duplicate vertex equality",
        )? {
            points.remove(index + 1);
        } else {
            index += 1;
        }
    }
    if points.len() > 1
        && convex_points_equal(
            &points[0],
            &points[points.len() - 1],
            "convex polygon duplicate endpoint equality",
        )?
    {
        points.pop();
    }
    Ok(())
}

fn polygon_is_degenerate(points: &[Point3]) -> bool {
    if points.len() < 3 {
        return true;
    }
    let anchor = &points[0];
    for i in 1..points.len() - 1 {
        if exact_points_are_collinear(anchor, &points[i], &points[i + 1]) != Some(true) {
            return false;
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::boolean::{
        ExactBooleanOperation, ExactBooleanRequest,
        replay::{
            ExactBooleanEvaluation, exact_boolean_evaluation_for_replay_result_with_materialization,
        },
    };

    fn with_materialized_evaluation_for_test<R>(
        left: &Mesh,
        right: &Mesh,
        request: ExactBooleanRequest,
        f: impl FnOnce(&ExactBooleanEvaluation) -> R,
    ) -> R {
        let evaluation = exact_boolean_evaluation_for_replay_result_with_materialization(
            left, right, request, true,
        )
        .unwrap();
        f(&evaluation)
    }

    fn tetrahedron_i64(a: [i64; 3], b: [i64; 3], c: [i64; 3], d: [i64; 3]) -> Mesh {
        Mesh::from_i64_triangles(
            &[
                a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], d[0], d[1], d[2],
            ],
            &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
        )
        .unwrap()
    }

    fn p3(x: i64, y: i64, z: i64) -> Point3 {
        Point3::new(Real::from(x), Real::from(y), Real::from(z))
    }

    #[test]
    fn convex_hull_segment_equality_accepts_reversed_duplicate() {
        let a = p3(0, 0, 0);
        let b = p3(1, 0, 0);
        let c = p3(0, 1, 0);

        assert!(
            unordered_segments_equal(&[a.clone(), b.clone()], &[b.clone(), a.clone()]).unwrap()
        );
        assert!(
            !unordered_segments_equal(&[a.clone(), b.clone()], &[a.clone(), c.clone()]).unwrap()
        );

        let mut segments = vec![[a.clone(), b.clone()]];
        push_unique_segment(&mut segments, [b, a]).unwrap();
        assert_eq!(segments.len(), 1);
        push_unique_segment(&mut segments, [p3(0, 0, 0), c]).unwrap();
        assert_eq!(segments.len(), 2);
    }

    #[test]
    fn convex_triangle_refinement_splits_existing_edge_vertex() {
        let vertices = vec![p3(0, 0, 0), p3(2, 0, 0), p3(0, 2, 0), p3(1, 0, 0)];
        let mut triangles = vec![Triangle([0, 1, 2])];

        assert!(refine_triangles_at_existing_edge_vertices(&vertices, &mut triangles).unwrap());
        assert_eq!(triangles, vec![Triangle([0, 3, 2]), Triangle([3, 1, 2])]);
    }

    #[test]
    fn convex_materialization_interning_and_simplification_merge_exact_duplicates() {
        let a = p3(0, 0, 0);
        let b = p3(1, 0, 0);
        let c = p3(0, 1, 0);
        let mut vertices = Vec::new();

        assert_eq!(intern_point(&mut vertices, &a).unwrap(), 0);
        assert_eq!(intern_point(&mut vertices, &b).unwrap(), 1);
        assert_eq!(intern_point(&mut vertices, &a).unwrap(), 0);
        assert_eq!(vertices.len(), 2);

        let mut polygon = vec![a.clone(), a.clone(), b, c, a.clone()];
        simplify_polygon(&mut polygon).unwrap();
        assert_eq!(polygon, vec![a, p3(1, 0, 0), p3(0, 1, 0)]);
    }

    #[test]
    fn straddling_coplanar_convex_union_and_difference_materialize_closed() {
        let left = tetrahedron_i64([0, 0, 0], [6, 0, 0], [0, 6, 0], [0, 0, 6]);
        let right = tetrahedron_i64([2, 2, 2], [8, -1, -1], [-1, 8, -1], [3, 2, 0]);

        let union = union_closed_convex_solids(&left, &right)
            .expect("certified convex union should not hit exact construction blockers")
            .expect("certified convex union should close exact planar boundary loop");
        validate_convex_boolean_output("union", &union.left_facts, &union.right_facts, &union.mesh)
            .unwrap();
        assert!(union.mesh.facts().mesh.closed_manifold);
        assert_eq!(union.mesh.facts().mesh.boundary_edges, 0);
        with_materialized_evaluation_for_test(
            &left,
            &right,
            ExactBooleanRequest {
                operation: ExactBooleanOperation::Union,
                validation: MeshValidationPolicy::CLOSED,
            },
            |evaluation| {
                evaluation.validate_against_sources(&left, &right).unwrap();
            },
        );

        let difference = subtract_closed_convex_solids(&left, &right)
            .expect("certified convex difference should not hit exact construction blockers")
            .expect("certified convex difference should orient paired cut faces");
        validate_convex_boolean_output(
            "difference",
            &difference.left_facts,
            &difference.right_facts,
            &difference.mesh,
        )
        .unwrap();
        assert!(difference.mesh.facts().mesh.closed_manifold);
        assert_eq!(difference.mesh.facts().mesh.boundary_edges, 0);
        with_materialized_evaluation_for_test(
            &left,
            &right,
            ExactBooleanRequest {
                operation: ExactBooleanOperation::Difference,
                validation: MeshValidationPolicy::CLOSED,
            },
            |evaluation| {
                evaluation.validate_against_sources(&left, &right).unwrap();
            },
        );
    }
}
