//! Shared exact loop triangulation for arrangement replay and simplification.
//!
//! This module keeps the `hypertri` handoff in one place. Inputs are exact
//! coordinates; undecidable predicates and invalid topology return arrangement
//! blockers instead of falling back to tolerance repair.

use std::cmp::Ordering;
use std::collections::BTreeMap;

use hyperlimit::{
    Point2, Point3, RingPointLocation, SegmentIntersection, Sign, TriangleLocation,
    classify_point_ring_even_odd, classify_point_triangle, classify_segment_intersection,
    compare_reals, orient2d_report, orient3d_report, point3_equal, project_point3,
    projected_polygon_area2_value,
};
use hyperreal::Real;

use super::super::arrangement2d::{
    ExactArrangement2dBlocker, ExactArrangement2dBoundaryPolicy, ExactArrangement2dRegion,
    ExactArrangement2dRegionRing, ExactArrangement2dSetOperation,
    build_exact_arrangement2d_overlay_with_boundary_policy,
    build_exact_arrangement2d_ring_union_overlay_with_boundary_policy,
};
use super::super::graph::key::{ExactPoint3Key, exact_point3_key};
use super::super::mesh::Triangle;
use super::regularization::ExactArrangementBlocker;
use hyperlimit::CoplanarProjection;

#[derive(Clone)]
struct ProjectedFaceLoop {
    boundary: Vec<Point3>,
    projection: CoplanarProjection,
    projected: Vec<Point2>,
    witness: Point2,
    depth: usize,
}

struct ExactCoplanarLoopGroup {
    carrier: [Point3; 3],
    loops: Vec<Vec<Point3>>,
}

pub(crate) fn group_exact_coplanar_loops(
    boundaries: Vec<Vec<Point3>>,
) -> Result<Vec<Vec<Vec<Point3>>>, ExactArrangementBlocker> {
    let mut groups = Vec::<ExactCoplanarLoopGroup>::new();
    for boundary in boundaries {
        if boundary.len() < 3 {
            return Err(ExactArrangementBlocker::NonManifoldCellComplex);
        }
        let carrier = exact_non_collinear_point_loop_carrier(&boundary)
            .ok_or(ExactArrangementBlocker::NonManifoldCellComplex)?;
        let mut group_index = None;
        for (index, group) in groups.iter().enumerate() {
            if point_loop_is_exactly_coplanar(&boundary, group.carrier_refs())? {
                group_index = Some(index);
                break;
            }
        }
        if let Some(index) = group_index {
            groups[index].loops.push(boundary);
            continue;
        }
        if !point_loop_is_exactly_coplanar(&boundary, (&carrier[0], &carrier[1], &carrier[2]))? {
            return Err(ExactArrangementBlocker::NonManifoldCellComplex);
        }
        groups.push(ExactCoplanarLoopGroup {
            carrier,
            loops: vec![boundary],
        });
    }
    Ok(groups.into_iter().map(|group| group.loops).collect())
}

impl ExactCoplanarLoopGroup {
    fn carrier_refs(&self) -> (&Point3, &Point3, &Point3) {
        (&self.carrier[0], &self.carrier[1], &self.carrier[2])
    }
}

fn exact_non_collinear_point_loop_carrier(points: &[Point3]) -> Option<[Point3; 3]> {
    let anchor = points.first()?;
    for first_index in 1..points.len() - 1 {
        for second_index in first_index + 1..points.len() {
            let first = points.get(first_index)?;
            let second = points.get(second_index)?;
            if !exact_points_are_collinear(anchor, first, second)? {
                return Some([anchor.clone(), first.clone(), second.clone()]);
            }
        }
    }
    None
}

fn point_loop_is_exactly_coplanar(
    points: &[Point3],
    carrier: (&Point3, &Point3, &Point3),
) -> Result<bool, ExactArrangementBlocker> {
    let (a, b, c) = carrier;
    for point in points {
        match orient3d_report(a, b, c, point).value() {
            Some(Sign::Zero) => {}
            Some(Sign::Positive | Sign::Negative) => return Ok(false),
            None => return Err(ExactArrangementBlocker::UndecidableOrdering),
        }
    }
    Ok(true)
}

fn exact_points_are_collinear(a: &Point3, b: &Point3, c: &Point3) -> Option<bool> {
    let abx = b.x.clone() - &a.x;
    let aby = b.y.clone() - &a.y;
    let abz = b.z.clone() - &a.z;
    let acx = c.x.clone() - &a.x;
    let acy = c.y.clone() - &a.y;
    let acz = c.z.clone() - &a.z;
    let cross_x = aby.clone() * &acz - &(abz.clone() * &acy);
    let cross_y = abz * &acx - &(abx.clone() * &acz);
    let cross_z = abx * &acy - &(aby * &acx);
    Some(
        compare_reals(&cross_x, &Real::from(0)).value()? == Ordering::Equal
            && compare_reals(&cross_y, &Real::from(0)).value()? == Ordering::Equal
            && compare_reals(&cross_z, &Real::from(0)).value()? == Ordering::Equal,
    )
}

pub(crate) fn triangulate_exact_loop_group(
    boundaries: &[Vec<Point3>],
    vertices: &mut Vec<Point3>,
    triangles: &mut Vec<Triangle>,
) -> Result<(), ExactArrangementBlocker> {
    let mut loops = Vec::new();
    for boundary in boundaries {
        if boundary.len() < 3 {
            return Err(ExactArrangementBlocker::NonManifoldCellComplex);
        }
        let projection = choose_polygon_projection(boundary)?;
        let projected = boundary
            .iter()
            .map(|point| project_point3(point, projection))
            .collect::<Vec<_>>();
        let witness = projected_loop_interior_witness(&projected)?;
        loops.push(ProjectedFaceLoop {
            boundary: boundary.clone(),
            projection,
            projected,
            witness,
            depth: 0,
        });
    }
    let mut vertex_index = ExactVertexInsertIndex::from_vertices(vertices);
    if let Err(error) = compute_loop_depths(&mut loops) {
        return triangulate_loop_group_union_via_arrangement_or_error(
            &loops,
            vertices,
            &mut vertex_index,
            triangles,
            error,
        );
    }
    let isolate_component_vertices = match same_depth_endpoint_touch_flags(&loops) {
        Ok(flags) => flags,
        Err(error) => {
            return triangulate_loop_group_union_via_arrangement_or_error(
                &loops,
                vertices,
                &mut vertex_index,
                triangles,
                error,
            );
        }
    };
    if let Err(error) = validate_loop_topology(&loops) {
        return triangulate_loop_group_union_via_arrangement_or_error(
            &loops,
            vertices,
            &mut vertex_index,
            triangles,
            error,
        );
    }
    let mut used_as_hole = vec![false; loops.len()];
    for outer_index in 0..loops.len() {
        if loops[outer_index].depth % 2 != 0 {
            continue;
        }
        let mut hole_indices = Vec::new();
        for hole_index in 0..loops.len() {
            if loops[hole_index].depth == loops[outer_index].depth + 1
                && loop_contains_loop(&loops[outer_index], &loops[hole_index])?
            {
                if used_as_hole[hole_index] {
                    return Err(ExactArrangementBlocker::NonManifoldCellComplex);
                }
                hole_indices.push(hole_index);
                used_as_hole[hole_index] = true;
            }
        }
        triangulate_loop_with_holes(
            &loops,
            outer_index,
            &hole_indices,
            vertices,
            &mut vertex_index,
            triangles,
            isolate_component_vertices[outer_index],
        )?;
    }
    for (index, loop_) in loops.iter().enumerate() {
        if loop_.depth % 2 != 0 && !used_as_hole[index] {
            return triangulate_loop_group_union_via_arrangement_or_error(
                &loops,
                vertices,
                &mut vertex_index,
                triangles,
                ExactArrangementBlocker::NonManifoldCellComplex,
            );
        }
    }
    Ok(())
}

fn same_depth_endpoint_touch_flags(
    loops: &[ProjectedFaceLoop],
) -> Result<Vec<bool>, ExactArrangementBlocker> {
    let mut touches = vec![false; loops.len()];
    for left_index in 0..loops.len() {
        for right_index in (left_index + 1)..loops.len() {
            let endpoint_touching = validate_loop_boundaries_do_not_cross_or_overlap(
                &loops[left_index].projected,
                &loops[right_index].projected,
            )?;
            if endpoint_touching && loops[left_index].depth == loops[right_index].depth {
                touches[left_index] = true;
                touches[right_index] = true;
            }
        }
    }
    Ok(touches)
}

fn validate_loop_topology(loops: &[ProjectedFaceLoop]) -> Result<(), ExactArrangementBlocker> {
    for left_index in 0..loops.len() {
        for right_index in (left_index + 1)..loops.len() {
            let endpoint_touching = validate_loop_boundaries_do_not_cross_or_overlap(
                &loops[left_index].projected,
                &loops[right_index].projected,
            )?;
            if endpoint_touching && loops[left_index].depth != loops[right_index].depth {
                return Err(ExactArrangementBlocker::NonManifoldCellComplex);
            }
            if loops[left_index].depth == loops[right_index].depth {
                validate_same_depth_loops_are_area_disjoint(
                    &loops[left_index],
                    &loops[right_index],
                )?;
            }
        }
    }
    Ok(())
}

fn validate_loop_boundaries_do_not_cross_or_overlap(
    left: &[Point2],
    right: &[Point2],
) -> Result<bool, ExactArrangementBlocker> {
    let mut endpoint_touching = false;
    for left_index in 0..left.len() {
        let left_next = (left_index + 1) % left.len();
        for right_index in 0..right.len() {
            let right_next = (right_index + 1) % right.len();
            match classify_segment_intersection(
                &left[left_index],
                &left[left_next],
                &right[right_index],
                &right[right_next],
            )
            .value()
            {
                Some(SegmentIntersection::Disjoint) => {}
                Some(SegmentIntersection::EndpointTouch) => endpoint_touching = true,
                Some(
                    SegmentIntersection::Proper
                    | SegmentIntersection::CollinearOverlap
                    | SegmentIntersection::Identical,
                ) => return Err(ExactArrangementBlocker::NonManifoldCellComplex),
                None => return Err(ExactArrangementBlocker::UndecidableOrdering),
            }
        }
    }
    Ok(endpoint_touching)
}

fn validate_same_depth_loops_are_area_disjoint(
    left: &ProjectedFaceLoop,
    right: &ProjectedFaceLoop,
) -> Result<(), ExactArrangementBlocker> {
    validate_same_depth_loop_witness_outside(left, right)?;
    validate_same_depth_loop_witness_outside(right, left)
}

fn validate_same_depth_loop_witness_outside(
    container: &ProjectedFaceLoop,
    candidate: &ProjectedFaceLoop,
) -> Result<(), ExactArrangementBlocker> {
    match classify_point_ring_even_odd(&container.projected, &candidate.witness).value() {
        Some(RingPointLocation::Outside) => Ok(()),
        Some(RingPointLocation::Inside | RingPointLocation::Boundary) => {
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        }
        None => Err(ExactArrangementBlocker::UndecidableOrdering),
    }
}

fn compute_loop_depths(loops: &mut [ProjectedFaceLoop]) -> Result<(), ExactArrangementBlocker> {
    for loop_index in 0..loops.len() {
        let mut depth = 0;
        for container_index in 0..loops.len() {
            if loop_index == container_index {
                continue;
            }
            if loop_contains_loop(&loops[container_index], &loops[loop_index])? {
                depth += 1;
            }
        }
        loops[loop_index].depth = depth;
    }
    Ok(())
}

fn loop_contains_loop(
    container: &ProjectedFaceLoop,
    child: &ProjectedFaceLoop,
) -> Result<bool, ExactArrangementBlocker> {
    let mut boundary_touch = false;
    for point in &child.projected {
        match classify_point_ring_even_odd(&container.projected, point).value() {
            Some(RingPointLocation::Inside) => {}
            Some(RingPointLocation::Outside) => return Ok(false),
            Some(RingPointLocation::Boundary) => boundary_touch = true,
            None => return Err(ExactArrangementBlocker::UndecidableOrdering),
        }
    }
    match classify_point_ring_even_odd(&container.projected, &child.witness).value() {
        Some(RingPointLocation::Inside) => {
            if boundary_touch {
                return Err(ExactArrangementBlocker::NonManifoldCellComplex);
            }
        }
        Some(RingPointLocation::Outside) => return Ok(false),
        Some(RingPointLocation::Boundary) => {
            return Err(ExactArrangementBlocker::NonManifoldCellComplex);
        }
        None => return Err(ExactArrangementBlocker::UndecidableOrdering),
    }
    Ok(true)
}

pub(crate) fn projected_loop_interior_witness(
    points: &[Point2],
) -> Result<Point2, ExactArrangementBlocker> {
    if points.len() < 3 {
        return Err(ExactArrangementBlocker::NonManifoldCellComplex);
    }
    let signed_area_twice = signed_area_twice_points(points);
    let orientation = match compare_reals(&signed_area_twice, &Real::from(0)).value() {
        Some(Ordering::Greater) => Sign::Positive,
        Some(Ordering::Less) => Sign::Negative,
        Some(Ordering::Equal) => return Err(ExactArrangementBlocker::NonManifoldCellComplex),
        None => return Err(ExactArrangementBlocker::UndecidableOrdering),
    };

    for index in 0..points.len() {
        let previous = &points[(index + points.len() - 1) % points.len()];
        let current = &points[index];
        let next = &points[(index + 1) % points.len()];
        match orient2d_report(previous, current, next).value() {
            Some(sign) if sign == orientation => {}
            Some(Sign::Zero | Sign::Positive | Sign::Negative) => continue,
            None => return Err(ExactArrangementBlocker::UndecidableOrdering),
        }

        let mut contains_vertex = false;
        for (candidate_index, candidate) in points.iter().enumerate() {
            if candidate_index == index
                || candidate_index == (index + points.len() - 1) % points.len()
                || candidate_index == (index + 1) % points.len()
            {
                continue;
            }
            match classify_point_triangle(previous, current, next, candidate).value() {
                Some(TriangleLocation::Inside | TriangleLocation::Degenerate) => {
                    contains_vertex = true;
                    break;
                }
                Some(
                    TriangleLocation::Outside
                    | TriangleLocation::OnEdge
                    | TriangleLocation::OnVertex,
                ) => {}
                None => return Err(ExactArrangementBlocker::UndecidableOrdering),
            }
        }
        if contains_vertex {
            continue;
        }

        let witness = triangle_centroid_2d(previous, current, next)?;
        match classify_point_ring_even_odd(points, &witness).value() {
            Some(RingPointLocation::Inside) => return Ok(witness),
            Some(RingPointLocation::Outside | RingPointLocation::Boundary) => {}
            None => return Err(ExactArrangementBlocker::UndecidableOrdering),
        }
    }

    Err(ExactArrangementBlocker::NonManifoldCellComplex)
}

fn signed_area_twice_points(points: &[Point2]) -> Real {
    let mut area = Real::from(0);
    for index in 0..points.len() {
        let current = &points[index];
        let next = &points[(index + 1) % points.len()];
        area += &(current.x.clone() * &next.y) - &(current.y.clone() * &next.x);
    }
    area
}

fn triangle_centroid_2d(
    a: &Point2,
    b: &Point2,
    c: &Point2,
) -> Result<Point2, ExactArrangementBlocker> {
    let third = (Real::from(1) / &Real::from(3))
        .ok()
        .ok_or(ExactArrangementBlocker::UndecidableOrdering)?;
    Ok(Point2::new(
        (a.x.clone() + &b.x + &c.x) * &third,
        (a.y.clone() + &b.y + &c.y) * &third,
    ))
}

fn triangulate_loop_with_holes(
    loops: &[ProjectedFaceLoop],
    outer_index: usize,
    hole_loop_indices: &[usize],
    vertices: &mut Vec<Point3>,
    vertex_index: &mut ExactVertexInsertIndex,
    triangles: &mut Vec<Triangle>,
    isolate_component_vertices: bool,
) -> Result<(), ExactArrangementBlocker> {
    let projection = loops[outer_index].projection;
    let output_orientation = projected_loop_orientation(&loops[outer_index].boundary, projection)?;
    let mut polygon_points = if hole_loop_indices.is_empty() {
        loops[outer_index].boundary.clone()
    } else {
        oriented_loop_points_for_triangulation(
            &loops[outer_index].boundary,
            projection,
            Ordering::Greater,
        )?
    };
    let mut projected = polygon_points
        .iter()
        .map(|point| project_for_hypertri(point, projection))
        .collect::<Vec<_>>();
    let mut hole_indices = Vec::new();
    for &hole_index in hole_loop_indices {
        if loops[hole_index].projection != projection {
            return Err(ExactArrangementBlocker::NonManifoldCellComplex);
        }
        hole_indices.push(projected.len());
        let hole_points = oriented_loop_points_for_triangulation(
            &loops[hole_index].boundary,
            projection,
            Ordering::Less,
        )?;
        polygon_points.extend(hole_points.iter().cloned());
        projected.extend(
            hole_points
                .iter()
                .map(|point| project_for_hypertri(point, projection)),
        );
    }
    let local_to_global = if isolate_component_vertices {
        append_component_local_vertices(vertices, vertex_index, &polygon_points)?
    } else {
        polygon_points
            .iter()
            .map(|point| vertex_index.find_or_insert(vertices, point.clone()))
            .collect::<Result<Vec<_>, _>>()?
    };
    if polygon_points.len() == 3 && hole_indices.is_empty() {
        let triangle = oriented_output_triangle(
            &polygon_points,
            projection,
            &[0, 1, 2],
            &local_to_global,
            output_orientation,
        )?;
        triangles.push(triangle);
        return Ok(());
    }
    let indices = match hypertri::earcut(&projected, &hole_indices) {
        Ok(indices) if !indices.is_empty() => indices,
        Ok(_) | Err(_)
            if !hole_loop_indices.is_empty() && hole_loops_touch(loops, hole_loop_indices)? =>
        {
            return triangulate_touching_hole_loop_group_via_arrangement(
                loops,
                outer_index,
                hole_loop_indices,
                vertices,
                vertex_index,
                triangles,
                output_orientation,
            );
        }
        Ok(_) | Err(_) => return Err(ExactArrangementBlocker::NonManifoldCellComplex),
    };
    let mut emitted_triangles = indices.chunks_exact(3);
    for triangle in &mut emitted_triangles {
        triangles.push(oriented_output_triangle(
            &polygon_points,
            projection,
            triangle,
            &local_to_global,
            output_orientation,
        )?);
    }
    if !emitted_triangles.remainder().is_empty() {
        return Err(ExactArrangementBlocker::NonManifoldCellComplex);
    }
    Ok(())
}

fn hole_loops_touch(
    loops: &[ProjectedFaceLoop],
    hole_loop_indices: &[usize],
) -> Result<bool, ExactArrangementBlocker> {
    for left in 0..hole_loop_indices.len() {
        for right in (left + 1)..hole_loop_indices.len() {
            if validate_loop_boundaries_do_not_cross_or_overlap(
                &loops[hole_loop_indices[left]].projected,
                &loops[hole_loop_indices[right]].projected,
            )? {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

fn triangulate_touching_hole_loop_group_via_arrangement(
    loops: &[ProjectedFaceLoop],
    outer_index: usize,
    hole_loop_indices: &[usize],
    vertices: &mut Vec<Point3>,
    vertex_index: &mut ExactVertexInsertIndex,
    triangles: &mut Vec<Triangle>,
    output_orientation: Ordering,
) -> Result<(), ExactArrangementBlocker> {
    let mut loop_indices = Vec::with_capacity(hole_loop_indices.len() + 1);
    loop_indices.push(outer_index);
    loop_indices.extend(hole_loop_indices.iter().copied());
    triangulate_projected_loop_indices_via_arrangement(
        loops,
        &loop_indices,
        vertices,
        vertex_index,
        triangles,
        output_orientation,
    )
}

fn triangulate_loop_group_union_via_arrangement_or_error(
    loops: &[ProjectedFaceLoop],
    vertices: &mut Vec<Point3>,
    vertex_index: &mut ExactVertexInsertIndex,
    triangles: &mut Vec<Triangle>,
    error: ExactArrangementBlocker,
) -> Result<(), ExactArrangementBlocker> {
    triangulate_loop_group_union_via_arrangement(loops, vertices, vertex_index, triangles)
        .or(Err(error))
}

fn triangulate_loop_group_union_via_arrangement(
    loops: &[ProjectedFaceLoop],
    vertices: &mut Vec<Point3>,
    vertex_index: &mut ExactVertexInsertIndex,
    triangles: &mut Vec<Triangle>,
) -> Result<(), ExactArrangementBlocker> {
    if loops.is_empty() {
        return Ok(());
    }
    let projection = loops[0].projection;
    let output_orientation = projected_loop_orientation(&loops[0].boundary, projection)?;
    let loop_indices = (0..loops.len()).collect::<Vec<_>>();
    triangulate_projected_loop_indices_via_ring_union_arrangement(
        loops,
        &loop_indices,
        vertices,
        vertex_index,
        triangles,
        output_orientation,
    )
}

fn triangulate_projected_loop_indices_via_ring_union_arrangement(
    loops: &[ProjectedFaceLoop],
    loop_indices: &[usize],
    vertices: &mut Vec<Point3>,
    vertex_index: &mut ExactVertexInsertIndex,
    triangles: &mut Vec<Triangle>,
    output_orientation: Ordering,
) -> Result<(), ExactArrangementBlocker> {
    let first = *loop_indices
        .first()
        .ok_or(ExactArrangementBlocker::NonManifoldCellComplex)?;
    let projection = loops[first].projection;
    let carrier = carrier_triangle_for_projection(&loops[first].boundary, projection)?;
    let mut rings = Vec::with_capacity(loop_indices.len());
    for &loop_index in loop_indices {
        if loops[loop_index].projection != projection {
            return Err(ExactArrangementBlocker::NonManifoldCellComplex);
        }
        rings.push(loops[loop_index].projected.clone());
    }

    let overlay = build_exact_arrangement2d_ring_union_overlay_with_boundary_policy(
        &rings,
        ExactArrangement2dBoundaryPolicy::PreserveCollinear,
    );
    triangulate_selected_overlay_faces(
        &overlay,
        &carrier,
        projection,
        output_orientation,
        vertices,
        vertex_index,
        triangles,
    )
}

fn triangulate_projected_loop_indices_via_arrangement(
    loops: &[ProjectedFaceLoop],
    loop_indices: &[usize],
    vertices: &mut Vec<Point3>,
    vertex_index: &mut ExactVertexInsertIndex,
    triangles: &mut Vec<Triangle>,
    output_orientation: Ordering,
) -> Result<(), ExactArrangementBlocker> {
    let first = *loop_indices
        .first()
        .ok_or(ExactArrangementBlocker::NonManifoldCellComplex)?;
    let projection = loops[first].projection;
    let carrier = carrier_triangle_for_projection(&loops[first].boundary, projection)?;
    let mut rings = Vec::with_capacity(loop_indices.len());
    for &loop_index in loop_indices {
        if loops[loop_index].projection != projection {
            return Err(ExactArrangementBlocker::NonManifoldCellComplex);
        }
        rings.push(ExactArrangement2dRegionRing::new(
            ExactArrangement2dRegion::Left,
            loops[loop_index].projected.clone(),
        ));
    }

    let overlay = build_exact_arrangement2d_overlay_with_boundary_policy(
        &rings,
        ExactArrangement2dSetOperation::Union,
        ExactArrangement2dBoundaryPolicy::PreserveCollinear,
    );
    triangulate_selected_overlay_faces(
        &overlay,
        &carrier,
        projection,
        output_orientation,
        vertices,
        vertex_index,
        triangles,
    )
}

fn triangulate_selected_overlay_faces(
    overlay: &super::super::arrangement2d::ExactArrangement2dOverlay,
    carrier: &[Point3; 3],
    projection: CoplanarProjection,
    output_orientation: Ordering,
    vertices: &mut Vec<Point3>,
    vertex_index: &mut ExactVertexInsertIndex,
    triangles: &mut Vec<Triangle>,
) -> Result<(), ExactArrangementBlocker> {
    if !overlay.is_complete() {
        return Err(map_arrangement2d_blocker(
            overlay
                .blockers
                .first()
                .ok_or(ExactArrangementBlocker::NonManifoldCellComplex)?,
        ));
    }

    for overlay_face in overlay.faces.iter().filter(|face| face.selected) {
        let face = overlay
            .arrangement
            .faces
            .get(overlay_face.face)
            .ok_or(ExactArrangementBlocker::NonManifoldCellComplex)?;
        let boundary = face
            .vertices
            .iter()
            .map(|&vertex| {
                let point = &overlay
                    .arrangement
                    .vertices
                    .get(vertex)
                    .ok_or(ExactArrangementBlocker::NonManifoldCellComplex)?
                    .point;
                lift_projected_point_to_carrier(point, carrier, projection)
            })
            .collect::<Result<Vec<_>, _>>()?;
        triangulate_simple_arrangement_face(
            &boundary,
            projection,
            output_orientation,
            vertices,
            vertex_index,
            triangles,
        )?;
    }
    Ok(())
}

fn map_arrangement2d_blocker(blocker: &ExactArrangement2dBlocker) -> ExactArrangementBlocker {
    match blocker {
        ExactArrangement2dBlocker::UnresolvedPointEquality { .. }
        | ExactArrangement2dBlocker::UnresolvedSegmentRelation { .. }
        | ExactArrangement2dBlocker::UnresolvedProperIntersectionConstruction { .. }
        | ExactArrangement2dBlocker::UnresolvedPointOnSegment { .. } => {
            ExactArrangementBlocker::UnresolvedIntersection
        }
        ExactArrangement2dBlocker::UnresolvedSegmentOrdering { .. }
        | ExactArrangement2dBlocker::UnresolvedAngleOrdering { .. }
        | ExactArrangement2dBlocker::UnresolvedFaceArea { .. }
        | ExactArrangement2dBlocker::UnresolvedRingNormalization { .. }
        | ExactArrangement2dBlocker::UnresolvedOutputLoopContainment { .. }
        | ExactArrangement2dBlocker::UnresolvedParentSelection { .. }
        | ExactArrangement2dBlocker::UnresolvedSelectedBoundaryOrdering { .. } => {
            ExactArrangementBlocker::UndecidableOrdering
        }
        ExactArrangement2dBlocker::DegenerateSegment { .. }
        | ExactArrangement2dBlocker::IncompleteFaceWalk { .. }
        | ExactArrangement2dBlocker::InvalidRegionRing { .. }
        | ExactArrangement2dBlocker::UnresolvedFaceWitness { .. }
        | ExactArrangement2dBlocker::UnresolvedRingClassification { .. }
        | ExactArrangement2dBlocker::FaceWitnessOnBoundary { .. }
        | ExactArrangement2dBlocker::NonManifoldSelectedBoundary { .. }
        | ExactArrangement2dBlocker::DegenerateOutputLoop { .. }
        | ExactArrangement2dBlocker::OutputHoleWithoutOuter { .. }
        | ExactArrangement2dBlocker::OutputLoopBoundaryContainment { .. } => {
            ExactArrangementBlocker::NonManifoldCellComplex
        }
    }
}

fn carrier_triangle_for_projection(
    boundary: &[Point3],
    projection: CoplanarProjection,
) -> Result<[Point3; 3], ExactArrangementBlocker> {
    for first in 0..boundary.len() {
        for second in (first + 1)..boundary.len() {
            for third in (second + 1)..boundary.len() {
                let points = [
                    project_point3(&boundary[first], projection),
                    project_point3(&boundary[second], projection),
                    project_point3(&boundary[third], projection),
                ];
                match orient2d_report(&points[0], &points[1], &points[2]).value() {
                    Some(Sign::Positive | Sign::Negative) => {
                        return Ok([
                            boundary[first].clone(),
                            boundary[second].clone(),
                            boundary[third].clone(),
                        ]);
                    }
                    Some(Sign::Zero) => {}
                    None => return Err(ExactArrangementBlocker::UndecidableOrdering),
                }
            }
        }
    }
    Err(ExactArrangementBlocker::NonManifoldCellComplex)
}

fn lift_projected_point_to_carrier(
    point: &Point2,
    carrier: &[Point3; 3],
    projection: CoplanarProjection,
) -> Result<Point3, ExactArrangementBlocker> {
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
    let a = ((wx.clone() * &vy - &(wy.clone() * &vx)) / &det)
        .ok()
        .ok_or(ExactArrangementBlocker::UndecidableOrdering)?;
    let b = ((ux * &wy - &(uy * &wx)) / &det)
        .ok()
        .ok_or(ExactArrangementBlocker::UndecidableOrdering)?;
    let p1 = vector_between(&carrier[0], &carrier[1]);
    let p2 = vector_between(&carrier[0], &carrier[2]);
    Ok(Point3::new(
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

fn triangulate_simple_arrangement_face(
    boundary: &[Point3],
    projection: CoplanarProjection,
    output_orientation: Ordering,
    vertices: &mut Vec<Point3>,
    vertex_index: &mut ExactVertexInsertIndex,
    triangles: &mut Vec<Triangle>,
) -> Result<(), ExactArrangementBlocker> {
    let projected = boundary
        .iter()
        .map(|point| project_for_hypertri(point, projection))
        .collect::<Vec<_>>();
    let local_to_global = boundary
        .iter()
        .map(|point| vertex_index.find_or_insert(vertices, point.clone()))
        .collect::<Result<Vec<_>, _>>()?;
    if boundary.len() == 3 {
        triangles.push(oriented_output_triangle(
            boundary,
            projection,
            &[0, 1, 2],
            &local_to_global,
            output_orientation,
        )?);
        return Ok(());
    }
    let indices = hypertri::earcut(&projected, &[])
        .map_err(|_| ExactArrangementBlocker::NonManifoldCellComplex)?;
    if indices.is_empty() {
        return Err(ExactArrangementBlocker::NonManifoldCellComplex);
    }
    let mut emitted_triangles = indices.chunks_exact(3);
    for triangle in &mut emitted_triangles {
        triangles.push(oriented_output_triangle(
            boundary,
            projection,
            triangle,
            &local_to_global,
            output_orientation,
        )?);
    }
    if !emitted_triangles.remainder().is_empty() {
        return Err(ExactArrangementBlocker::NonManifoldCellComplex);
    }
    Ok(())
}

fn append_component_local_vertices(
    vertices: &mut Vec<Point3>,
    vertex_index: &mut ExactVertexInsertIndex,
    polygon_points: &[Point3],
) -> Result<Vec<usize>, ExactArrangementBlocker> {
    let offset = vertices.len();
    let mut component_vertices = Vec::<Point3>::new();
    let mut component_index = ExactVertexInsertIndex::default();
    let mut local_to_global = Vec::with_capacity(polygon_points.len());
    for point in polygon_points {
        let local = component_index.find_or_insert(&mut component_vertices, point.clone())?;
        local_to_global.push(offset + local);
    }
    for point in component_vertices {
        let index = vertices.len();
        vertex_index.insert_known(index, &point);
        vertices.push(point);
    }
    Ok(local_to_global)
}

fn oriented_output_triangle(
    polygon_points: &[Point3],
    projection: CoplanarProjection,
    triangle: &[usize],
    local_to_global: &[usize],
    output_orientation: Ordering,
) -> Result<Triangle, ExactArrangementBlocker> {
    let emitted_orientation = emitted_triangle_orientation(polygon_points, projection, triangle)?;
    let [a, b, c] = triangle else {
        return Err(ExactArrangementBlocker::NonManifoldCellComplex);
    };
    let a = *local_to_global
        .get(*a)
        .ok_or(ExactArrangementBlocker::NonManifoldCellComplex)?;
    let b = *local_to_global
        .get(*b)
        .ok_or(ExactArrangementBlocker::NonManifoldCellComplex)?;
    let c = *local_to_global
        .get(*c)
        .ok_or(ExactArrangementBlocker::NonManifoldCellComplex)?;
    if emitted_orientation == output_orientation {
        Ok(Triangle([a, b, c]))
    } else {
        Ok(Triangle([a, c, b]))
    }
}

pub(crate) fn projected_loop_orientation(
    points: &[Point3],
    projection: CoplanarProjection,
) -> Result<Ordering, ExactArrangementBlocker> {
    let area = projected_polygon_area2_value(points, projection);
    match compare_reals(&area, &Real::from(0)).value() {
        Some(ordering @ (Ordering::Less | Ordering::Greater)) => Ok(ordering),
        Some(Ordering::Equal) => Err(ExactArrangementBlocker::NonManifoldCellComplex),
        None => Err(ExactArrangementBlocker::UndecidableOrdering),
    }
}

pub(crate) fn emitted_triangle_orientation(
    polygon_points: &[Point3],
    projection: CoplanarProjection,
    triangle: &[usize],
) -> Result<Ordering, ExactArrangementBlocker> {
    let [a, b, c] = triangle else {
        return Err(ExactArrangementBlocker::NonManifoldCellComplex);
    };
    let points = [
        polygon_points
            .get(*a)
            .ok_or(ExactArrangementBlocker::NonManifoldCellComplex)?
            .clone(),
        polygon_points
            .get(*b)
            .ok_or(ExactArrangementBlocker::NonManifoldCellComplex)?
            .clone(),
        polygon_points
            .get(*c)
            .ok_or(ExactArrangementBlocker::NonManifoldCellComplex)?
            .clone(),
    ];
    match compare_reals(
        &projected_polygon_area2_value(&points, projection),
        &Real::from(0),
    )
    .value()
    {
        Some(ordering @ (Ordering::Less | Ordering::Greater)) => Ok(ordering),
        Some(Ordering::Equal) => Err(ExactArrangementBlocker::NonManifoldCellComplex),
        None => Err(ExactArrangementBlocker::UndecidableOrdering),
    }
}

fn oriented_loop_points_for_triangulation(
    points: &[Point3],
    projection: CoplanarProjection,
    expected: Ordering,
) -> Result<Vec<Point3>, ExactArrangementBlocker> {
    let area = projected_polygon_area2_value(points, projection);
    match compare_reals(&area, &Real::from(0)).value() {
        Some(Ordering::Equal) => Err(ExactArrangementBlocker::NonManifoldCellComplex),
        Some(ordering) if ordering == expected => Ok(points.to_vec()),
        Some(Ordering::Less | Ordering::Greater) => {
            let mut reversed = points.to_vec();
            reversed.reverse();
            Ok(reversed)
        }
        None => Err(ExactArrangementBlocker::UndecidableOrdering),
    }
}

#[derive(Default)]
struct ExactVertexInsertIndex {
    point_key_buckets: BTreeMap<ExactPoint3Key, Vec<usize>>,
    unkeyed_vertices: Vec<usize>,
}

impl ExactVertexInsertIndex {
    fn from_vertices(vertices: &[Point3]) -> Self {
        let mut index = Self::default();
        for (vertex, point) in vertices.iter().enumerate() {
            index.insert_known(vertex, point);
        }
        index
    }

    fn find_or_insert(
        &mut self,
        vertices: &mut Vec<Point3>,
        point: Point3,
    ) -> Result<usize, ExactArrangementBlocker> {
        let point_key = exact_point3_key(&point);
        if let Some(index) = self.find_matching(&point, point_key.as_ref(), vertices)? {
            return Ok(index);
        }
        let vertex = vertices.len();
        self.insert_with_key(vertex, point_key);
        vertices.push(point);
        Ok(vertex)
    }

    fn insert_known(&mut self, vertex: usize, point: &Point3) {
        self.insert_with_key(vertex, exact_point3_key(point));
    }

    fn insert_with_key(&mut self, vertex: usize, point_key: Option<ExactPoint3Key>) {
        if let Some(key) = point_key {
            self.point_key_buckets.entry(key).or_default().push(vertex);
        } else {
            self.unkeyed_vertices.push(vertex);
        }
    }

    fn find_matching(
        &self,
        point: &Point3,
        point_key: Option<&ExactPoint3Key>,
        vertices: &[Point3],
    ) -> Result<Option<usize>, ExactArrangementBlocker> {
        if let Some(key) = point_key {
            if let Some(bucket) = self.point_key_buckets.get(key)
                && let Some(index) = find_matching_vertex_in_indices(point, vertices, bucket)?
            {
                return Ok(Some(index));
            }
            return find_matching_vertex_in_indices(point, vertices, &self.unkeyed_vertices);
        }

        for bucket in self.point_key_buckets.values() {
            if let Some(index) = find_matching_vertex_in_indices(point, vertices, bucket)? {
                return Ok(Some(index));
            }
        }
        find_matching_vertex_in_indices(point, vertices, &self.unkeyed_vertices)
    }
}

fn find_matching_vertex_in_indices(
    point: &Point3,
    vertices: &[Point3],
    candidates: &[usize],
) -> Result<Option<usize>, ExactArrangementBlocker> {
    for &index in candidates {
        match point3_equal(&vertices[index], point).value() {
            Some(true) => return Ok(Some(index)),
            Some(false) => {}
            None => return Err(ExactArrangementBlocker::UndecidableOrdering),
        }
    }
    Ok(None)
}

pub(crate) fn choose_polygon_projection(
    points: &[Point3],
) -> Result<CoplanarProjection, ExactArrangementBlocker> {
    for projection in [
        CoplanarProjection::Xy,
        CoplanarProjection::Xz,
        CoplanarProjection::Yz,
    ] {
        let area = projected_polygon_area2_value(points, projection);
        match compare_reals(&area, &Real::from(0)).value() {
            Some(Ordering::Less | Ordering::Greater) => return Ok(projection),
            Some(Ordering::Equal) => {}
            None => return Err(ExactArrangementBlocker::UndecidableOrdering),
        }
    }
    Err(ExactArrangementBlocker::NonManifoldCellComplex)
}

fn project_for_hypertri(point: &Point3, projection: CoplanarProjection) -> hypertri::ExactPoint {
    match projection {
        CoplanarProjection::Xy => hypertri::ExactPoint::new(point.x.clone(), point.y.clone()),
        CoplanarProjection::Xz => hypertri::ExactPoint::new(point.x.clone(), point.z.clone()),
        CoplanarProjection::Yz => hypertri::ExactPoint::new(point.y.clone(), point.z.clone()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(x: i64, y: i64, z: i64) -> Point3 {
        Point3::new(Real::from(x), Real::from(y), Real::from(z))
    }

    fn q(numerator: i64, denominator: i64) -> Real {
        (Real::from(numerator) / &Real::from(denominator)).expect("nonzero denominator")
    }

    fn rational_p(x: [i64; 2], y: [i64; 2], z: [i64; 2]) -> Point3 {
        Point3::new(q(x[0], x[1]), q(y[0], y[1]), q(z[0], z[1]))
    }

    fn triangle_area2(
        vertices: &[Point3],
        triangles: &[Triangle],
        projection: CoplanarProjection,
    ) -> Real {
        let mut area = Real::from(0);
        for triangle in triangles {
            let points = [
                vertices[triangle.0[0]].clone(),
                vertices[triangle.0[1]].clone(),
                vertices[triangle.0[2]].clone(),
            ];
            area += &projected_polygon_area2_value(&points, projection);
        }
        area
    }

    fn area_magnitude_eq(area: &Real, expected: i64) -> bool {
        compare_reals(area, &Real::from(expected)).value() == Some(Ordering::Equal)
            || compare_reals(area, &Real::from(-expected)).value() == Some(Ordering::Equal)
    }

    #[test]
    fn vertex_insert_index_buckets_exact_rational_points() {
        let mut vertices = Vec::new();
        let mut index = ExactVertexInsertIndex::default();
        let point = rational_p([1, 2], [-3, 4], [5, 6]);

        assert_eq!(
            index.find_or_insert(&mut vertices, point.clone()).unwrap(),
            0
        );
        assert_eq!(index.find_or_insert(&mut vertices, point).unwrap(), 0);
        assert_eq!(
            index
                .find_or_insert(&mut vertices, rational_p([2, 3], [-3, 4], [5, 6]))
                .unwrap(),
            1
        );

        assert_eq!(vertices.len(), 2);
        assert_eq!(index.point_key_buckets.len(), 2);
        assert!(index.unkeyed_vertices.is_empty());
    }

    #[test]
    fn triangulates_endpoint_touching_holes_via_exact_arrangement_cells() {
        let loops = vec![
            vec![p(0, 0, 0), p(8, 0, 0), p(8, 8, 0), p(0, 8, 0)],
            vec![p(1, 1, 0), p(3, 1, 0), p(3, 3, 0), p(1, 3, 0)],
            vec![p(3, 3, 0), p(5, 3, 0), p(5, 5, 0), p(3, 5, 0)],
        ];

        let mut vertices = Vec::new();
        let mut triangles = Vec::new();
        triangulate_exact_loop_group(&loops, &mut vertices, &mut triangles).unwrap();

        assert!(!triangles.is_empty());
        assert!(
            vertices
                .iter()
                .any(|vertex| { point3_equal(vertex, &p(3, 3, 0)).value() == Some(true) })
        );
    }

    #[test]
    fn triangulates_proper_crossing_same_depth_loops_via_arrangement_union() {
        let loops = vec![
            vec![p(0, 0, 0), p(4, 0, 0), p(4, 4, 0), p(0, 4, 0)],
            vec![p(2, 1, 0), p(6, 1, 0), p(6, 3, 0), p(2, 3, 0)],
        ];

        let mut vertices = Vec::new();
        let mut triangles = Vec::new();
        triangulate_exact_loop_group(&loops, &mut vertices, &mut triangles).unwrap();

        assert!(!triangles.is_empty());
        assert!(area_magnitude_eq(
            &triangle_area2(&vertices, &triangles, CoplanarProjection::Xy),
            40
        ));
        assert!(
            vertices
                .iter()
                .any(|vertex| { point3_equal(vertex, &p(4, 1, 0)).value() == Some(true) })
        );
        assert!(
            vertices
                .iter()
                .any(|vertex| { point3_equal(vertex, &p(4, 3, 0)).value() == Some(true) })
        );
    }

    #[test]
    fn triangulates_collinear_overlapping_same_depth_loops_via_arrangement() {
        let loops = vec![
            vec![p(0, 0, 0), p(2, 0, 0), p(2, 2, 0), p(0, 2, 0)],
            vec![p(2, 0, 0), p(4, 0, 0), p(4, 2, 0), p(2, 2, 0)],
        ];

        let mut vertices = Vec::new();
        let mut triangles = Vec::new();
        triangulate_exact_loop_group(&loops, &mut vertices, &mut triangles).unwrap();

        assert!(!triangles.is_empty());
        assert!(
            vertices
                .iter()
                .any(|vertex| { point3_equal(vertex, &p(2, 0, 0)).value() == Some(true) })
        );
        assert!(
            vertices
                .iter()
                .any(|vertex| { point3_equal(vertex, &p(2, 2, 0)).value() == Some(true) })
        );
    }

    #[test]
    fn coplanar_loop_grouping_rejects_exact_non_planar_loop_as_topology() {
        let loops = vec![vec![p(0, 0, 0), p(4, 0, 0), p(4, 4, 0), p(0, 4, 1)]];

        assert_eq!(
            group_exact_coplanar_loops(loops),
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        );
    }
}
