//! Exact coplanar surface certificates used by the boolean pipeline.
//!
//! Legacy coplanar materializers live in the test/fuzz source tree now.  The
//! library keeps only production certifiers here: convex surface equivalence,
//! boundary-loop recovery, and boundary-touch detection.

use core::cmp::Ordering;

use hyperlimit::{
    Point2, Point3, SegmentIntersection, Sign, classify_segment_intersection, compare_reals,
    orient2d_report, project_point3, proper_segment_intersection_point,
};
use hyperreal::Real;

use super::coplanar::{CoplanarProjection, CoplanarTriangleRelation, classify_coplanar_triangles};
use super::mesh::ExactMesh;
use super::narrow::{
    TrianglePlaneRelation, TriangleTriangleRelation,
    classify_mesh_triangle_against_retained_face_plane, classify_triangle_triangle,
};

/// Certified equivalence of two convex coplanar surface meshes.
#[derive(Clone, Debug, PartialEq)]
pub struct CoplanarConvexSurfaceEquivalence {
    /// Projection used for hull and area certificates.
    pub projection: CoplanarProjection,
    /// Exact shared convex hull boundary.
    pub polygon: Vec<Point3>,
    /// Twice the projected area covered by the left mesh.
    pub left_area2: Real,
    /// Twice the projected area covered by the right mesh.
    pub right_area2: Real,
}

type ConvexSurfaceHullsAndAreas = (CoplanarProjection, Vec<Point3>, Vec<Point3>, Real, Real);

/// Certify that two coplanar open meshes cover the same convex surface.
pub fn certify_coplanar_convex_surface_equivalence(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarConvexSurfaceEquivalence> {
    let (projection, left_hull, right_hull, left_area, right_area) =
        convex_surface_hulls_and_areas(left, right)?;
    if !polygons_equal(&left_hull, &right_hull) {
        return None;
    }
    let hull_area = projected_area2_abs(&left_hull, projection)?;
    if compare_reals(&left_area, &hull_area).value() != Some(Ordering::Equal)
        || compare_reals(&right_area, &hull_area).value() != Some(Ordering::Equal)
    {
        return None;
    }
    Some(CoplanarConvexSurfaceEquivalence {
        projection,
        polygon: left_hull,
        left_area2: left_area,
        right_area2: right_area,
    })
}

/// Recover all topological boundary loops from a triangulated surface mesh.
pub fn order_mesh_boundary_loops(mesh: &ExactMesh) -> Option<Vec<Vec<usize>>> {
    let mut edge_counts: Vec<((usize, usize), usize)> = Vec::new();
    for triangle in mesh.triangles() {
        for (a, b) in triangle_edges(triangle.0) {
            let edge = canonical_edge(a, b);
            if let Some((_, count)) = edge_counts
                .iter_mut()
                .find(|(candidate, _)| *candidate == edge)
            {
                *count += 1;
            } else {
                edge_counts.push((edge, 1));
            }
        }
    }
    if edge_counts
        .iter()
        .any(|(_, count)| *count == 0 || *count > 2)
    {
        return None;
    }
    let boundary_edges = edge_counts
        .into_iter()
        .filter_map(|(edge, count)| (count == 1).then_some(edge))
        .collect::<Vec<_>>();
    if boundary_edges.len() < 3 {
        return None;
    }

    let mut boundary_vertices = Vec::new();
    for &(a, b) in &boundary_edges {
        if !boundary_vertices.contains(&a) {
            boundary_vertices.push(a);
        }
        if !boundary_vertices.contains(&b) {
            boundary_vertices.push(b);
        }
    }
    for &vertex in &boundary_vertices {
        let degree = boundary_edges
            .iter()
            .filter(|(a, b)| *a == vertex || *b == vertex)
            .count();
        if degree != 2 {
            return None;
        }
    }

    let mut used = vec![false; boundary_edges.len()];
    let mut loops = Vec::new();
    while let Some(seed) = used.iter().position(|used| !*used) {
        let (a, b) = boundary_edges[seed];
        let start = a.min(b);
        let mut previous = None;
        let mut current = start;
        let mut loop_vertices = Vec::new();
        loop {
            loop_vertices.push(current);
            let mut candidates = boundary_edges
                .iter()
                .enumerate()
                .filter_map(|(index, (edge_a, edge_b))| {
                    if used[index] {
                        return None;
                    }
                    if *edge_a == current {
                        Some((index, *edge_b))
                    } else if *edge_b == current {
                        Some((index, *edge_a))
                    } else {
                        None
                    }
                })
                .collect::<Vec<_>>();
            candidates.sort_by_key(|(_, next)| *next);
            let (edge_index, next) = match previous {
                Some(previous) => candidates
                    .into_iter()
                    .find(|(_, candidate)| *candidate != previous)?,
                None => candidates.into_iter().next()?,
            };
            used[edge_index] = true;
            if next == start {
                break;
            }
            if loop_vertices.contains(&next) {
                return None;
            }
            previous = Some(current);
            current = next;
            if loop_vertices.len() > boundary_edges.len() {
                return None;
            }
        }
        if loop_vertices.len() < 3 {
            return None;
        }
        loops.push(loop_vertices);
    }
    if loops.is_empty() || used.iter().any(|used| !*used) {
        None
    } else {
        Some(loops)
    }
}

/// Certify positive-length coplanar boundary contact without positive-area overlap.
pub fn certify_coplanar_surface_boundary_touch(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarProjection> {
    if left.triangles().is_empty() || right.triangles().is_empty() {
        return None;
    }
    if !single_retained_plane(left, right)? {
        return None;
    }
    let projection = choose_mesh_projection(left).or_else(|| choose_mesh_projection(right))?;
    let left_boundary = boundary_edges(left)?;
    let right_boundary = boundary_edges(right)?;
    let mut saw_positive_length_boundary_touch = false;

    for left_face in 0..left.triangles().len() {
        let left_triangle = triangle_points(left, left_face);
        for right_face in 0..right.triangles().len() {
            let right_triangle = triangle_points(right, right_face);
            if let Some(clip) = pairwise_coplanar_triangle_intersection_polygon_points(
                &left_triangle,
                &right_triangle,
            ) {
                let area = projected_area2_abs(&clip, projection)?;
                if compare_reals(&area, &Real::from(0)).value() == Some(Ordering::Greater) {
                    return None;
                }
            }
        }
    }

    for (left_a, left_b) in left_boundary {
        for (right_a, right_b) in &right_boundary {
            let a = project_point3(&left.vertices()[left_a], projection);
            let b = project_point3(&left.vertices()[left_b], projection);
            let c = project_point3(&right.vertices()[*right_a], projection);
            let d = project_point3(&right.vertices()[*right_b], projection);
            match classify_segment_intersection(&a, &b, &c, &d).value()? {
                SegmentIntersection::CollinearOverlap | SegmentIntersection::Identical => {
                    saw_positive_length_boundary_touch = true;
                }
                SegmentIntersection::Disjoint
                | SegmentIntersection::Proper
                | SegmentIntersection::EndpointTouch => {}
            }
        }
    }

    saw_positive_length_boundary_touch.then_some(projection)
}

fn single_retained_plane(left: &ExactMesh, right: &ExactMesh) -> Option<bool> {
    for face in 0..left.triangles().len() {
        let classification =
            classify_mesh_triangle_against_retained_face_plane(left, 0, left, face).ok()?;
        if classification.relation != TrianglePlaneRelation::Coplanar {
            return Some(false);
        }
    }
    for face in 0..right.triangles().len() {
        let classification =
            classify_mesh_triangle_against_retained_face_plane(left, 0, right, face).ok()?;
        if classification.relation != TrianglePlaneRelation::Coplanar {
            return Some(false);
        }
    }
    Some(true)
}

fn convex_surface_hulls_and_areas(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<ConvexSurfaceHullsAndAreas> {
    if left.triangles().is_empty() || right.triangles().is_empty() {
        return None;
    }
    if left.triangles().len() == 1 && right.triangles().len() == 1 {
        return None;
    }
    if !single_retained_plane(left, right)? {
        return None;
    }
    let projection = choose_mesh_projection(left).or_else(|| choose_mesh_projection(right))?;
    let left_hull = convex_hull_3d(mesh_points(left), projection)?;
    let right_hull = convex_hull_3d(mesh_points(right), projection)?;
    let left_hull_area = projected_area2_abs(&left_hull, projection)?;
    let right_hull_area = projected_area2_abs(&right_hull, projection)?;
    let left_area = mesh_projected_area2(left, projection)?;
    let right_area = mesh_projected_area2(right, projection)?;
    if compare_reals(&left_area, &left_hull_area).value() != Some(Ordering::Equal)
        || compare_reals(&right_area, &right_hull_area).value() != Some(Ordering::Equal)
    {
        return None;
    }
    Some((projection, left_hull, right_hull, left_area, right_area))
}

fn choose_mesh_projection(mesh: &ExactMesh) -> Option<CoplanarProjection> {
    let triangle = mesh.triangles().first()?.0;
    for projection in [
        CoplanarProjection::Xy,
        CoplanarProjection::Xz,
        CoplanarProjection::Yz,
    ] {
        let a = project_point3(&mesh.vertices()[triangle[0]], projection);
        let b = project_point3(&mesh.vertices()[triangle[1]], projection);
        let c = project_point3(&mesh.vertices()[triangle[2]], projection);
        if orient2d_report(&a, &b, &c).value()? != Sign::Zero {
            return Some(projection);
        }
    }
    None
}

fn mesh_points(mesh: &ExactMesh) -> Vec<Point3> {
    mesh.vertices().to_vec()
}

fn mesh_projected_area2(mesh: &ExactMesh, projection: CoplanarProjection) -> Option<Real> {
    let mut area = Real::from(0);
    for face in 0..mesh.triangles().len() {
        area = area + projected_area2_abs(&triangle_points(mesh, face), projection)?;
    }
    Some(area)
}

fn pairwise_coplanar_triangle_intersection_polygon_points(
    left: &[Point3],
    right: &[Point3],
) -> Option<Vec<Point3>> {
    if left.len() != 3 || right.len() != 3 {
        return None;
    }
    let points = left.iter().chain(right).cloned().collect::<Vec<_>>();
    let classification = classify_triangle_triangle(&points, [0, 1, 2], [3, 4, 5]);
    if classification.relation != TriangleTriangleRelation::CoplanarOverlapping {
        return None;
    }
    let coplanar = classify_coplanar_triangles(&points, [0, 1, 2], [3, 4, 5]);
    if coplanar.relation != CoplanarTriangleRelation::Overlapping {
        return None;
    }
    let projection = coplanar.projection?;
    convex_polygon_intersection(left, right, projection)
}

fn convex_polygon_intersection(
    left: &[Point3],
    right: &[Point3],
    projection: CoplanarProjection,
) -> Option<Vec<Point3>> {
    let mut points = Vec::new();
    for point in left {
        if point_in_closed_convex_polygon(point, right, projection)? {
            push_unique_point(&mut points, point.clone());
        }
    }
    for point in right {
        if point_in_closed_convex_polygon(point, left, projection)? {
            push_unique_point(&mut points, point.clone());
        }
    }
    for left_edge in 0..left.len() {
        let left_start = &left[left_edge];
        let left_end = &left[(left_edge + 1) % left.len()];
        for right_edge in 0..right.len() {
            let right_start = &right[right_edge];
            let right_end = &right[(right_edge + 1) % right.len()];
            collect_segment_intersection_points(
                &mut points,
                left_start,
                left_end,
                right_start,
                right_end,
                projection,
            )?;
        }
    }
    if points.len() < 3 {
        return None;
    }
    let mut polygon = convex_hull_3d(points, projection)?;
    orient_polygon_ccw(&mut polygon, projection)?;
    (polygon.len() >= 3).then_some(polygon)
}

fn collect_segment_intersection_points(
    points: &mut Vec<Point3>,
    a: &Point3,
    b: &Point3,
    c: &Point3,
    d: &Point3,
    projection: CoplanarProjection,
) -> Option<()> {
    let pa = project_point3(a, projection);
    let pb = project_point3(b, projection);
    let pc = project_point3(c, projection);
    let pd = project_point3(d, projection);
    match classify_segment_intersection(&pa, &pb, &pc, &pd).value()? {
        SegmentIntersection::Disjoint => {}
        SegmentIntersection::EndpointTouch => {
            for point in [a, b, c, d] {
                let projected = project_point3(point, projection);
                if point_on_segment_2d(&pa, &pb, &projected)?
                    && point_on_segment_2d(&pc, &pd, &projected)?
                {
                    push_unique_point(points, point.clone());
                }
            }
        }
        SegmentIntersection::Proper => {
            let projected = proper_segment_intersection_point(&pa, &pb, &pc, &pd).value()??;
            push_unique_point(
                points,
                lift_projected_point_on_segment(a, b, &projected, projection)?,
            );
        }
        SegmentIntersection::CollinearOverlap | SegmentIntersection::Identical => {
            for point in [a, b, c, d] {
                let projected = project_point3(point, projection);
                if point_on_segment_2d(&pa, &pb, &projected)?
                    && point_on_segment_2d(&pc, &pd, &projected)?
                {
                    push_unique_point(points, point.clone());
                }
            }
        }
    }
    Some(())
}

fn point_on_segment_2d(a: &Point2, b: &Point2, point: &Point2) -> Option<bool> {
    hyperlimit::point_on_segment(a, b, point).value()
}

fn lift_projected_point_on_segment(
    start: &Point3,
    end: &Point3,
    point: &Point2,
    projection: CoplanarProjection,
) -> Option<Point3> {
    let start2 = project_point3(start, projection);
    let end2 = project_point3(end, projection);
    let dx = end2.x.clone() - &start2.x;
    let dy = end2.y.clone() - &start2.y;
    let zero = Real::from(0);
    let t = if compare_reals(&dx, &zero).value()? != Ordering::Equal {
        ((point.x.clone() - &start2.x) / dx).ok()?
    } else if compare_reals(&dy, &zero).value()? != Ordering::Equal {
        ((point.y.clone() - &start2.y) / dy).ok()?
    } else {
        return None;
    };
    Some(Point3::new(
        start.x.clone() + (end.x.clone() - &start.x) * &t,
        start.y.clone() + (end.y.clone() - &start.y) * &t,
        start.z.clone() + (end.z.clone() - &start.z) * &t,
    ))
}

fn triangle_points(mesh: &ExactMesh, face: usize) -> Vec<Point3> {
    mesh.triangles()[face]
        .0
        .iter()
        .map(|&index| mesh.vertices()[index].clone())
        .collect()
}

fn triangle_edges(triangle: [usize; 3]) -> [(usize, usize); 3] {
    [
        (triangle[0], triangle[1]),
        (triangle[1], triangle[2]),
        (triangle[2], triangle[0]),
    ]
}

fn boundary_edges(mesh: &ExactMesh) -> Option<Vec<(usize, usize)>> {
    let mut edge_counts: Vec<((usize, usize), usize)> = Vec::new();
    for triangle in mesh.triangles() {
        for (a, b) in triangle_edges(triangle.0) {
            let edge = canonical_edge(a, b);
            if let Some((_, count)) = edge_counts
                .iter_mut()
                .find(|(candidate, _)| *candidate == edge)
            {
                *count += 1;
            } else {
                edge_counts.push((edge, 1));
            }
        }
    }
    if edge_counts.iter().any(|(_, count)| *count > 2) {
        return None;
    }
    Some(
        edge_counts
            .into_iter()
            .filter_map(|(edge, count)| (count == 1).then_some(edge))
            .collect(),
    )
}

fn canonical_edge(a: usize, b: usize) -> (usize, usize) {
    if a <= b { (a, b) } else { (b, a) }
}

fn polygons_equal(left: &[Point3], right: &[Point3]) -> bool {
    if left.len() != right.len() {
        return false;
    }
    if left.is_empty() {
        return true;
    }
    (0..right.len()).any(|offset| {
        left.iter()
            .zip(right.iter().cycle().skip(offset))
            .take(left.len())
            .all(|(left, right)| points_equal(left, right))
    }) || (0..right.len()).any(|offset| {
        left.iter()
            .zip(right.iter().rev().cycle().skip(offset))
            .take(left.len())
            .all(|(left, right)| points_equal(left, right))
    })
}

fn point_in_closed_convex_polygon(
    point: &Point3,
    polygon: &[Point3],
    projection: CoplanarProjection,
) -> Option<bool> {
    let mut ring = polygon.to_vec();
    orient_polygon_ccw(&mut ring, projection)?;
    let query = project_point3(point, projection);
    for edge in 0..ring.len() {
        let a = project_point3(&ring[edge], projection);
        let b = project_point3(&ring[(edge + 1) % ring.len()], projection);
        if orient2d_report(&a, &b, &query).value()? == Sign::Negative {
            return Some(false);
        }
    }
    Some(true)
}

fn convex_hull_3d(points: Vec<Point3>, projection: CoplanarProjection) -> Option<Vec<Point3>> {
    let mut points = unique_points(points);
    if points.len() < 3 {
        return None;
    }
    points.sort_by(|left, right| {
        compare_point2(
            &project_point3(left, projection),
            &project_point3(right, projection),
        )
        .unwrap_or(Ordering::Equal)
    });

    let mut lower: Vec<Point3> = Vec::new();
    for point in &points {
        while lower.len() >= 2
            && orient_projected(
                &lower[lower.len() - 2],
                &lower[lower.len() - 1],
                point,
                projection,
            )? != Sign::Positive
        {
            lower.pop();
        }
        lower.push(point.clone());
    }

    let mut upper: Vec<Point3> = Vec::new();
    for point in points.iter().rev() {
        while upper.len() >= 2
            && orient_projected(
                &upper[upper.len() - 2],
                &upper[upper.len() - 1],
                point,
                projection,
            )? != Sign::Positive
        {
            upper.pop();
        }
        upper.push(point.clone());
    }

    lower.pop();
    upper.pop();
    lower.extend(upper);
    (lower.len() >= 3).then_some(lower)
}

fn orient_polygon_ccw(points: &mut [Point3], projection: CoplanarProjection) -> Option<()> {
    let area = hyperlimit::projected_polygon_area2_value(points, projection);
    if compare_reals(&area, &Real::from(0)).value()? == Ordering::Less {
        points.reverse();
    }
    Some(())
}

fn orient_projected(
    a: &Point3,
    b: &Point3,
    c: &Point3,
    projection: CoplanarProjection,
) -> Option<Sign> {
    orient2d_report(
        &project_point3(a, projection),
        &project_point3(b, projection),
        &project_point3(c, projection),
    )
    .value()
}

fn projected_area2_abs(points: &[Point3], projection: CoplanarProjection) -> Option<Real> {
    hyperlimit::projected_polygon_area2_abs_value(points, projection)
}

fn compare_point2(left: &Point2, right: &Point2) -> Option<Ordering> {
    match compare_reals(&left.x, &right.x).value()? {
        Ordering::Equal => compare_reals(&left.y, &right.y).value(),
        ordering => Some(ordering),
    }
}

fn points_equal(left: &Point3, right: &Point3) -> bool {
    compare_reals(&left.x, &right.x).value() == Some(Ordering::Equal)
        && compare_reals(&left.y, &right.y).value() == Some(Ordering::Equal)
        && compare_reals(&left.z, &right.z).value() == Some(Ordering::Equal)
}

fn unique_points(points: Vec<Point3>) -> Vec<Point3> {
    let mut unique = Vec::new();
    for point in points {
        push_unique_point(&mut unique, point);
    }
    unique
}

fn push_unique_point(points: &mut Vec<Point3>, point: Point3) {
    if !points
        .iter()
        .any(|candidate| points_equal(candidate, &point))
    {
        points.push(point);
    }
}
