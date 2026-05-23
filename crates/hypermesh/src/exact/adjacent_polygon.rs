//! Source-owned triangulated-disk full-face adjacency certificates.
//!
//! This module is the branch-face companion to [`crate::exact::adjacent`].
//! It accepts source-owned, coplanar face disks when both solids replay the same
//! simple projected boundary with opposite signed area. The certificate keeps a
//! strict separation in Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7.1-2 (1997): source topology is replayed as face
//! lists and edge incidences, while exact predicates certify that replayed
//! topology is valid in both source and projected spaces.
//!
//! The strict point-in-ring check uses the even-odd crossing classifier of
//! Hormann and Agathos, "The point in polygon problem for arbitrary polygons,"
//! *Computational Geometry* 20.3 (2001). Boundary loop ordering uses a degree-two
//! cycle reconstruction over the candidate boundary edge graph. Broader non-rectilinear
//! coplanar-cell materialization remains intentionally separate from this full-face
//! shortcut.

use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet};

use hyperlimit::{
    CoplanarProjection, Point2, Point3, RingPointLocation, SegmentIntersection, Sign,
    classify_point_ring_even_odd, classify_segment_intersection, compare_reals, orient3d_report,
    project_point3, projected_polygon_area2_value,
};

use super::mesh::ExactMesh;
use super::scalar::ExactReal;

const MAX_POLYGON_PATCH_FACES: usize = 9;
const MAX_POLYGON_PATCH_BOUNDARY: usize = 9;

#[derive(Clone, Debug, PartialEq)]
struct PolygonPatchCandidate {
    faces: Vec<usize>,
    boundary_points: Vec<Point3>,
    signed_area2: ExactReal,
    area_abs: ExactReal,
}

/// Discover source-owned simple-polygon adjacency patch pairs.
///
/// The input triangles are split into edge-connected components, and each component is
/// exhaustively searched for triangulated-disk candidates within practical bounds.
///
/// Algorithmically this follows Yap, "Towards Exact Geometric Computation"'s
/// object/predicate split: source topology is replayed from combinatorial adjacency,
/// while exact predicates certify coplanarity, interior inclusion, and signed-area
/// compatibility.
pub(crate) fn polygon_patch_pairs(
    left: &ExactMesh,
    consumed_left_faces: &BTreeSet<usize>,
    right: &ExactMesh,
    consumed_right_faces: &BTreeSet<usize>,
) -> Option<Vec<(Vec<usize>, Vec<usize>)>> {
    let left_candidates = polygon_patch_candidates(left, consumed_left_faces)?;
    let right_candidates = polygon_patch_candidates(right, consumed_right_faces)?;
    let mut pairs = Vec::new();
    let mut used_left = BTreeSet::new();
    let mut used_right = BTreeSet::new();

    for left_candidate in &left_candidates {
        if left_candidate
            .faces
            .iter()
            .any(|face| used_left.contains(face))
        {
            continue;
        }
        let Some((right_index, right_candidate)) =
            right_candidates
                .iter()
                .enumerate()
                .find(|(right_index, candidate)| {
                    !used_right.contains(right_index)
                        && polygon_patch_candidates_match(left_candidate, candidate)
                })
        else {
            continue;
        };
        used_left.extend(left_candidate.faces.iter().copied());
        used_right.insert(right_index);
        pairs.push((left_candidate.faces.clone(), right_candidate.faces.clone()));
    }

    Some(pairs)
}

fn polygon_patch_candidates(
    mesh: &ExactMesh,
    consumed_faces: &BTreeSet<usize>,
) -> Option<Vec<PolygonPatchCandidate>> {
    let mut candidates = Vec::new();
    let available = (0..mesh.triangles().len())
        .filter(|face| !consumed_faces.contains(face))
        .collect::<Vec<_>>();
    if available.is_empty() {
        return None;
    }
    let max_faces = available.len().min(MAX_POLYGON_PATCH_FACES);
    let max_boundary = available.len().min(MAX_POLYGON_PATCH_BOUNDARY);
    let neighbors = edge_connected_face_neighbors(mesh, &available)?;
    let mut unassigned = available.iter().copied().collect::<BTreeSet<_>>();
    while let Some(start_face) = unassigned.iter().next().copied() {
        let mut component =
            extract_polygon_patch_component(start_face, &neighbors, &mut unassigned)?;
        component.sort_unstable();
        if component.len() < 2 {
            continue;
        }
        let mut seen = BTreeSet::<Vec<usize>>::new();
        collect_polygon_patch_candidates(
            mesh,
            &neighbors,
            component[0],
            max_faces,
            &mut vec![component[0]],
            &mut seen,
            &mut candidates,
            max_boundary,
        )?;
    }

    candidates.sort_by(|left, right| {
        right
            .faces
            .len()
            .cmp(&left.faces.len())
            .then_with(|| right.boundary_points.len().cmp(&left.boundary_points.len()))
            .then_with(|| left.faces.cmp(&right.faces))
    });
    Some(candidates)
}

fn collect_polygon_patch_candidates(
    mesh: &ExactMesh,
    neighbors: &BTreeMap<usize, BTreeSet<usize>>,
    start_face: usize,
    max_faces: usize,
    selected: &mut Vec<usize>,
    seen: &mut BTreeSet<Vec<usize>>,
    candidates: &mut Vec<PolygonPatchCandidate>,
    max_boundary: usize,
) -> Option<()> {
    if (2..=max_faces).contains(&selected.len()) {
        let mut key = selected.clone();
        key.sort_unstable();
        if seen.insert(key.clone())
            && let Some(candidate) = polygon_patch_candidate(mesh, &key, max_boundary)?
        {
            candidates.push(candidate);
        }
    }
    if selected.len() == max_faces {
        return Some(());
    }

    let selected_set = selected.iter().copied().collect::<BTreeSet<_>>();
    let mut extensions = BTreeSet::new();
    for face in selected.iter().copied() {
        for &neighbor in neighbors.get(&face)? {
            if neighbor >= start_face && !selected_set.contains(&neighbor) {
                extensions.insert(neighbor);
            }
        }
    }
    for extension in extensions {
        selected.push(extension);
        collect_polygon_patch_candidates(
            mesh,
            neighbors,
            start_face,
            max_faces,
            selected,
            seen,
            candidates,
            max_boundary,
        )?;
        selected.pop();
    }
    Some(())
}

fn polygon_patch_candidate(
    mesh: &ExactMesh,
    faces: &[usize],
    max_boundary: usize,
) -> Option<Option<PolygonPatchCandidate>> {
    if faces.len() < 2 {
        return Some(None);
    }
    // Boundary topology is reconstructed from source-owned edge incidences. Candidate
    // patches must be a triangulated disk: every edge appears at most twice and at
    // least one boundary cycle remains after interior-edge cancellation.
    let mut edge_counts = BTreeMap::<(usize, usize), usize>::new();
    for &face in faces {
        let triangle = mesh.triangles().get(face)?.0;
        for edge in triangle_edges(triangle) {
            let count = edge_counts.entry(edge).or_default();
            *count += 1;
            if *count > 2 {
                return Some(None);
            }
        }
    }
    if edge_counts.values().any(|&count| count == 0 || count > 2) {
        return Some(None);
    }
    let boundary_edges = edge_counts
        .iter()
        .filter_map(|(&edge, &count)| (count == 1).then_some(edge))
        .collect::<Vec<_>>();
    if boundary_edges.len() < 3 || boundary_edges.len() > max_boundary {
        return Some(None);
    }
    let Some(boundary_vertices) = order_boundary_vertices(&boundary_edges) else {
        return Some(None);
    };
    if boundary_vertices.len() != boundary_edges.len() {
        return Some(None);
    }
    let boundary_points = boundary_vertices
        .iter()
        .map(|&vertex| {
            mesh.vertices()
                .get(vertex)
                .map(|point| point.to_hyperlimit_point())
        })
        .collect::<Option<Vec<_>>>()?;
    let Some(projection) = choose_polygon_projection(&boundary_points) else {
        return Some(None);
    };
    if !loop_is_simple(&boundary_points, projection)? {
        return Some(None);
    }
    let boundary_ring = boundary_points
        .iter()
        .map(|point| project_point3(point, projection))
        .collect::<Vec<_>>();

    let mut area_sign = None;
    let mut signed_area2 = ExactReal::from(0);
    for &face in faces {
        let triangle = mesh.triangles().get(face)?.0;
        let points = triangle_points(mesh, triangle)?;
        if !points
            .iter()
            .all(|point| point_on_triangle_plane_vec(&boundary_points, point) == Some(true))
        {
            return Some(None);
        }
        for point in &points {
            if !boundary_points
                .iter()
                .any(|boundary_point| points_equal(boundary_point, point) == Some(true))
                && !point_strictly_inside_simple_loop(point, &boundary_ring, projection)?
            {
                return Some(None);
            }
        }
        let area = projected_polygon_area2_value(&points, projection);
        let sign = real_sign(&area)?;
        if sign == Sign::Zero {
            return Some(None);
        }
        match area_sign {
            Some(existing) if existing != sign => return Some(None),
            Some(_) => {}
            None => area_sign = Some(sign),
        }
        signed_area2 = signed_area2 + area;
    }

    let area_abs = real_abs(&signed_area2)?;
    let boundary_area_abs = real_abs(&projected_polygon_area2_value(&boundary_points, projection))?;
    if compare_reals(&area_abs, &boundary_area_abs).value() != Some(Ordering::Equal) {
        return Some(None);
    }

    Some(Some(PolygonPatchCandidate {
        faces: faces.to_vec(),
        boundary_points,
        signed_area2,
        area_abs,
    }))
}

fn edge_connected_face_neighbors(
    mesh: &ExactMesh,
    faces: &[usize],
) -> Option<BTreeMap<usize, BTreeSet<usize>>> {
    let mut edge_faces = BTreeMap::<(usize, usize), Vec<usize>>::new();
    for &face in faces {
        for edge in triangle_edges(mesh.triangles().get(face)?.0) {
            edge_faces.entry(edge).or_default().push(face);
        }
    }

    let mut neighbors = BTreeMap::<usize, BTreeSet<usize>>::new();
    for &face in faces {
        neighbors.entry(face).or_default();
    }
    for edge_faces in edge_faces.values() {
        for &face in edge_faces {
            for &neighbor in edge_faces {
                if neighbor != face {
                    neighbors.entry(face).or_default().insert(neighbor);
                }
            }
        }
    }
    Some(neighbors)
}

fn extract_polygon_patch_component(
    start_face: usize,
    neighbors: &BTreeMap<usize, BTreeSet<usize>>,
    available: &mut BTreeSet<usize>,
) -> Option<Vec<usize>> {
    // Traverse a source-owned connected component to avoid enumerating disconnected
    // unions; each component is still searched exhaustively inside bounded limits.
    let mut stack = vec![start_face];
    let mut component = Vec::new();
    while let Some(face) = stack.pop() {
        if !available.remove(&face) {
            continue;
        }
        component.push(face);
        for neighbor in neighbors.get(&face)? {
            if available.contains(neighbor) {
                stack.push(*neighbor);
            }
        }
    }
    if component.len() < 2 {
        return Some(component);
    }
    component.sort_unstable();
    Some(component)
}

fn order_boundary_vertices(edges: &[(usize, usize)]) -> Option<Vec<usize>> {
    // Reconstruct a single boundary cycle from degree-2 adjacency.
    let mut adjacency = BTreeMap::<usize, Vec<usize>>::new();
    for &(a, b) in edges {
        adjacency.entry(a).or_default().push(b);
        adjacency.entry(b).or_default().push(a);
    }
    if adjacency.len() < 3 || adjacency.values().any(|neighbors| neighbors.len() != 2) {
        return None;
    }
    let start = *adjacency.keys().next()?;
    let mut ordered = vec![start];
    let mut previous = usize::MAX;
    let mut current = start;
    loop {
        let neighbors = adjacency.get(&current)?;
        let next = neighbors
            .iter()
            .copied()
            .find(|&neighbor| neighbor != previous)?;
        if next == start {
            break;
        }
        if ordered.contains(&next) {
            return None;
        }
        ordered.push(next);
        previous = current;
        current = next;
        if ordered.len() > adjacency.len() {
            return None;
        }
    }
    (ordered.len() == adjacency.len()).then_some(ordered)
}

fn loop_is_simple(points: &[Point3], projection: CoplanarProjection) -> Option<bool> {
    // Exact boundary simplification is judged by Hormann and Agathos' even-odd loop
    // interpretation after exact projection. We keep strict edge/vertex separation here;
    // touching at endpoints of adjacent edges is only accepted when expected.
    if points.len() < 3 {
        return Some(false);
    }
    if real_sign(&projected_polygon_area2_value(points, projection))? == Sign::Zero {
        return Some(false);
    }

    let projected = points
        .iter()
        .map(|point| project_point3(point, projection))
        .collect::<Vec<_>>();
    for left in 0..projected.len() {
        for right in left + 1..projected.len() {
            if points2_equal(&projected[left], &projected[right])? {
                return Some(false);
            }
        }
    }

    for left_edge in 0..projected.len() {
        let left_next = (left_edge + 1) % projected.len();
        for right_edge in left_edge + 1..projected.len() {
            let right_next = (right_edge + 1) % projected.len();
            let adjacent = left_next == right_edge
                || right_next == left_edge
                || (left_edge == 0 && right_next == 0);
            let relation = classify_segment_intersection(
                &projected[left_edge],
                &projected[left_next],
                &projected[right_edge],
                &projected[right_next],
            )
            .value()?;
            match (adjacent, relation) {
                (true, SegmentIntersection::EndpointTouch) | (_, SegmentIntersection::Disjoint) => {
                }
                _ => return Some(false),
            }
        }
    }
    Some(true)
}

fn point_strictly_inside_simple_loop(
    point: &Point3,
    projected_boundary: &[Point2],
    projection: CoplanarProjection,
) -> Option<bool> {
    // Strict interior is required by the patch certificate; boundary-touching is
    // rejected to prevent zero-area seam reuse and to preserve exact-source replay.
    if projected_boundary.len() < 3 {
        return Some(false);
    }
    let projected = project_point3(point, projection);
    match classify_point_ring_even_odd(projected_boundary, &projected).value()? {
        RingPointLocation::Inside => Some(true),
        RingPointLocation::Boundary | RingPointLocation::Outside => Some(false),
    }
}

fn points2_equal(left: &Point2, right: &Point2) -> Option<bool> {
    Some(
        compare_reals(&left.x, &right.x).value()? == Ordering::Equal
            && compare_reals(&left.y, &right.y).value()? == Ordering::Equal,
    )
}

fn polygon_patch_candidates_match(
    left: &PolygonPatchCandidate,
    right: &PolygonPatchCandidate,
) -> bool {
    boundary_point_sets_equal_slice(&left.boundary_points, &right.boundary_points) == Some(true)
        && compare_reals(&left.area_abs, &right.area_abs).value() == Some(Ordering::Equal)
        && compare_reals(
            &(left.signed_area2.clone() + right.signed_area2.clone()),
            &ExactReal::from(0),
        )
        .value()
            == Some(Ordering::Equal)
}

fn boundary_point_sets_equal_slice(left: &[Point3], right: &[Point3]) -> Option<bool> {
    if left.len() != right.len() {
        return Some(false);
    }
    for right_point in right {
        if !left
            .iter()
            .any(|left_point| points_equal(left_point, right_point) == Some(true))
        {
            return Some(false);
        }
    }
    Some(true)
}

fn point_on_triangle_plane_vec(points: &[Point3], point: &Point3) -> Option<bool> {
    let [a, b, c, ..] = points else {
        return Some(false);
    };
    Some(orient3d_report(a, b, c, point).value()? == Sign::Zero)
}

fn choose_polygon_projection(points: &[Point3]) -> Option<CoplanarProjection> {
    [
        CoplanarProjection::Xy,
        CoplanarProjection::Xz,
        CoplanarProjection::Yz,
    ]
    .into_iter()
    .find(|&projection| {
        let area = projected_polygon_area2_value(points, projection);
        !matches!(real_sign(&area), Some(Sign::Zero) | None)
    })
}

fn triangle_points(mesh: &ExactMesh, triangle: [usize; 3]) -> Option<[Point3; 3]> {
    Some([
        mesh.vertices().get(triangle[0])?.to_hyperlimit_point(),
        mesh.vertices().get(triangle[1])?.to_hyperlimit_point(),
        mesh.vertices().get(triangle[2])?.to_hyperlimit_point(),
    ])
}

fn triangle_edges(triangle: [usize; 3]) -> [(usize, usize); 3] {
    [
        canonical_edge(triangle[0], triangle[1]),
        canonical_edge(triangle[1], triangle[2]),
        canonical_edge(triangle[2], triangle[0]),
    ]
}

const fn canonical_edge(left: usize, right: usize) -> (usize, usize) {
    if left <= right {
        (left, right)
    } else {
        (right, left)
    }
}

fn real_abs(value: &ExactReal) -> Option<ExactReal> {
    match real_sign(value)? {
        Sign::Negative => Some(-value.clone()),
        Sign::Zero | Sign::Positive => Some(value.clone()),
    }
}

fn real_sign(value: &ExactReal) -> Option<Sign> {
    match compare_reals(value, &ExactReal::from(0)).value()? {
        Ordering::Less => Some(Sign::Negative),
        Ordering::Equal => Some(Sign::Zero),
        Ordering::Greater => Some(Sign::Positive),
    }
}

fn points_equal(left: &Point3, right: &Point3) -> Option<bool> {
    Some(
        compare_reals(&left.x, &right.x).value()? == Ordering::Equal
            && compare_reals(&left.y, &right.y).value()? == Ordering::Equal
            && compare_reals(&left.z, &right.z).value()? == Ordering::Equal,
    )
}
