//! Source-owned triangulated-disk full-face adjacency certificates.
//!
//! This module is the branch-face companion to [`crate::adjacent`].
//! It accepts source-owned, coplanar face disks when both solids replay the same
//! simple projected boundary with opposite signed area. The certificate keeps a
//! lists and edge incidences, while exact predicates certify that replayed
//! topology is valid in both source and projected spaces.
//!
//! The strict point-in-ring check uses the even-odd crossing classifier of
//! Hormann and Agathos, "The point in polygon problem for arbitrary polygons,"
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
use super::topology::triangle_edges_tuple;
use hyperreal::Real;

const MAX_POLYGON_PATCH_ENUMERATION_FACES: usize = 9;
const MAX_POLYGON_PATCH_ENUMERATION_BOUNDARY: usize = 9;

#[derive(Clone, Debug, PartialEq)]
struct PolygonPatchCandidate {
    faces: Vec<usize>,
    boundary_points: Vec<Point3>,
    signed_area2: Real,
    area_abs: Real,
}

/// Discover source-owned simple-polygon adjacency patch pairs.
///
/// The input triangles are split into edge-connected components, and each component is
/// exhaustively searched for triangulated-disk candidates within practical bounds.
///
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
    let max_faces = available.len().min(MAX_POLYGON_PATCH_ENUMERATION_FACES);
    let max_boundary = MAX_POLYGON_PATCH_ENUMERATION_BOUNDARY;
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
        // A whole connected component is a concrete source-owned object, not a
        // guessed subset. Certify it directly regardless of size; only the
        // combinatorial subpatch search below remains bounded.
        if let Some(candidate) = polygon_patch_candidate(mesh, &component, usize::MAX)? {
            seen.insert(component.clone());
            candidates.push(candidate);
        }
        // Enumerate from every face in the component. The recursive collector keeps
        // only extensions whose id is at least `start_face`, so each connected
        // subset has a unique minimum-face root. Starting only at component[0]
        // misses valid source disks nested inside a larger coplanar source
        // component, which is exactly the kind of retained evidence/topology split
        for &start_face in &component {
            collect_polygon_patch_candidates(
                mesh,
                &neighbors,
                start_face,
                max_faces,
                &mut vec![start_face],
                &mut seen,
                &mut candidates,
                max_boundary,
            )?;
        }
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
    for face in selected.iter() {
        for &neighbor in neighbors.get(face)? {
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
        for edge in triangle_edges_tuple(triangle) {
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
        .map(|&vertex| mesh.vertices().get(vertex).cloned())
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
    let mut signed_area2 = Real::from(0);
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
        signed_area2 += area;
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
        for edge in triangle_edges_tuple(mesh.triangles().get(face)?.0) {
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
                if neighbor != face && faces_are_coplanar(mesh, face, neighbor)? {
                    neighbors.entry(face).or_default().insert(neighbor);
                }
            }
        }
    }
    Some(neighbors)
}

fn faces_are_coplanar(mesh: &ExactMesh, left_face: usize, right_face: usize) -> Option<bool> {
    // Source-disk discovery is a planar certificate, not a shell-connectivity
    // topology by exact retained planes before promoting a connected component
    // to a planar disk candidate.
    let left_triangle = mesh.triangles().get(left_face)?.0;
    let right_triangle = mesh.triangles().get(right_face)?.0;
    let left_points = triangle_points(mesh, left_triangle)?;
    let right_points = triangle_points(mesh, right_triangle)?;
    Some(
        right_points
            .iter()
            .all(|point| point_on_triangle_plane_vec(&left_points, point) == Some(true))
            && left_points
                .iter()
                .all(|point| point_on_triangle_plane_vec(&right_points, point) == Some(true)),
    )
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
            &Real::from(0),
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
        mesh.vertices().get(triangle[0])?.clone(),
        mesh.vertices().get(triangle[1])?.clone(),
        mesh.vertices().get(triangle[2])?.clone(),
    ])
}

fn real_abs(value: &Real) -> Option<Real> {
    match real_sign(value)? {
        Sign::Negative => Some(-value.clone()),
        Sign::Zero | Sign::Positive => Some(value.clone()),
    }
}

fn real_sign(value: &Real) -> Option<Sign> {
    match compare_reals(value, &Real::from(0)).value()? {
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

#[cfg(test)]
mod tests {
    use std::collections::BTreeSet;

    use super::*;
    use crate::ValidationPolicy;
    use proptest::prelude::*;

    const OVERSIZED_COMPONENT_FACES: usize = 33;

    fn open_mesh(points: &[[i64; 3]], triangles: &[usize]) -> ExactMesh {
        let mut coordinates = Vec::with_capacity(points.len() * 3);
        for point in points {
            coordinates.extend_from_slice(point);
        }
        ExactMesh::from_i64_triangles_with_policy(
            &coordinates,
            triangles,
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap()
    }

    fn shifted_square_subpatch_pair(
        prefix: i64,
        width: i64,
        height: i64,
    ) -> (ExactMesh, ExactMesh) {
        let left = open_mesh(
            &[
                [0, 0, 0],
                [prefix, 0, 0],
                [prefix + width, 0, 0],
                [prefix + width, height, 0],
                [prefix, height, 0],
            ],
            &[
                0, 4, 1, //
                1, 4, 2, 2, 4, 3,
            ],
        );
        let right = open_mesh(
            &[
                [prefix, 0, 0],
                [prefix + width, 0, 0],
                [prefix + width, height, 0],
                [prefix, height, 0],
            ],
            &[0, 1, 2, 0, 2, 3],
        );
        (left, right)
    }

    fn oversized_component_fan(face_count: usize, reversed: bool) -> ExactMesh {
        let mut points = Vec::new();
        points.push([0, 0, 0]);
        for index in 0..=face_count {
            points.push([index as i64, 1, 0]);
        }
        let mut triangles = Vec::new();
        for index in 1..points.len() - 1 {
            if reversed {
                triangles.extend([0, index + 1, index]);
            } else {
                triangles.extend([0, index, index + 1]);
            }
        }
        open_mesh(&points, &triangles)
    }

    #[test]
    fn polygon_patch_pairs_find_subpatch_not_rooted_at_component_minimum() {
        let (left, right) = shifted_square_subpatch_pair(1, 4, 4);

        let pairs = polygon_patch_pairs(&left, &BTreeSet::new(), &right, &BTreeSet::new())
            .expect("bounded source-disk candidates should be available");

        assert_eq!(pairs, vec![(vec![1, 2], vec![0, 1])]);
    }

    #[test]
    fn polygon_patch_candidates_emit_oversized_whole_components() {
        let mesh = oversized_component_fan(OVERSIZED_COMPONENT_FACES, false);
        let candidates = polygon_patch_candidates(&mesh, &BTreeSet::new())
            .expect("source-disk candidates should be available");

        assert!(
            candidates
                .iter()
                .any(|candidate| candidate.faces.len() == OVERSIZED_COMPONENT_FACES)
        );
    }

    #[test]
    fn polygon_patch_pairs_match_oversized_whole_components() {
        let left = oversized_component_fan(OVERSIZED_COMPONENT_FACES, false);
        let right = oversized_component_fan(OVERSIZED_COMPONENT_FACES, true);

        let pairs = polygon_patch_pairs(&left, &BTreeSet::new(), &right, &BTreeSet::new())
            .expect("oversized source-disk pair should be available");

        assert_eq!(
            pairs,
            vec![(
                (0..OVERSIZED_COMPONENT_FACES).collect(),
                (0..OVERSIZED_COMPONENT_FACES).collect()
            )]
        );
    }

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(32))]

        #[test]
        fn generated_subpatch_excluding_component_minimum_is_still_found(
            prefix in 1_i64..24,
            width in 2_i64..24,
            height in 2_i64..24,
        ) {
            let (left, right) = shifted_square_subpatch_pair(prefix, width, height);
            let pairs = polygon_patch_pairs(&left, &BTreeSet::new(), &right, &BTreeSet::new())
                .expect("bounded source-disk candidates should be available");

            prop_assert_eq!(pairs, vec![(vec![1, 2], vec![0, 1])]);
        }

        #[test]
        fn generated_oversized_whole_components_are_retained(
            extra_faces in 1_usize..12,
        ) {
            let face_count = OVERSIZED_COMPONENT_FACES + extra_faces;
            let mesh = oversized_component_fan(face_count, false);
            let candidates = polygon_patch_candidates(&mesh, &BTreeSet::new())
                .expect("source-disk candidates should be available");

            prop_assert!(
                candidates
                    .iter()
                    .any(|candidate| candidate.faces.len() == face_count)
            );
        }
    }
}
