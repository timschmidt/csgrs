//! Shared exact loop triangulation for arrangement replay and simplification.
//!
//! This module keeps the `hypertri` handoff in one place. Inputs are exact
//! coordinates; undecidable predicates and invalid topology return arrangement
//! blockers instead of falling back to tolerance repair.

use std::cmp::Ordering;

use hyperlimit::{
    Point2, Point3, RingPointLocation, SegmentIntersection, Sign, TriangleLocation,
    classify_point_ring_even_odd, classify_point_triangle, classify_segment_intersection,
    compare_reals, orient2d_report, point3_equal, project_point3, projected_polygon_area2_value,
};
use hyperreal::Real;

use super::mesh::Triangle;
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
    compute_loop_depths(&mut loops)?;
    let isolate_component_vertices = same_depth_endpoint_touch_flags(&loops)?;
    validate_loop_topology(&loops)?;
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
            triangles,
            isolate_component_vertices[outer_index],
        )?;
    }
    for (index, loop_) in loops.iter().enumerate() {
        if loop_.depth % 2 != 0 && !used_as_hole[index] {
            return Err(ExactArrangementBlocker::NonManifoldCellComplex);
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

pub(crate) fn triangulate_exact_loop(
    boundary: &[Point3],
    vertices: &mut Vec<Point3>,
    triangles: &mut Vec<Triangle>,
) -> Result<(), ExactArrangementBlocker> {
    triangulate_exact_loop_group(&[boundary.to_vec()], vertices, triangles)
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
        area = area + &(current.x.clone() * &next.y) - &(current.y.clone() * &next.x);
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
        append_component_local_vertices(vertices, &polygon_points)?
    } else {
        polygon_points
            .iter()
            .map(|point| find_or_insert_vertex(vertices, point.clone()))
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
    let indices = hypertri::earcut(&projected, &hole_indices)
        .map_err(|_| ExactArrangementBlocker::NonManifoldCellComplex)?;
    if indices.is_empty() {
        return Err(ExactArrangementBlocker::NonManifoldCellComplex);
    }
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

fn append_component_local_vertices(
    vertices: &mut Vec<Point3>,
    polygon_points: &[Point3],
) -> Result<Vec<usize>, ExactArrangementBlocker> {
    let offset = vertices.len();
    let mut component_vertices = Vec::<Point3>::new();
    let mut local_to_global = Vec::with_capacity(polygon_points.len());
    for point in polygon_points {
        let local = find_or_insert_vertex(&mut component_vertices, point.clone())?;
        local_to_global.push(offset + local);
    }
    vertices.extend(component_vertices);
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

fn find_or_insert_vertex(
    vertices: &mut Vec<Point3>,
    point: Point3,
) -> Result<usize, ExactArrangementBlocker> {
    for (index, existing) in vertices.iter().enumerate() {
        match point3_equal(existing, &point).value() {
            Some(true) => return Ok(index),
            Some(false) => {}
            None => return Err(ExactArrangementBlocker::UndecidableOrdering),
        }
    }
    let index = vertices.len();
    vertices.push(point);
    Ok(index)
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
