//! Source-owned triangulated-disk full-face adjacency certificates.
//!
//! This module is the branch-face companion to the full-face adjacency shortcut.
//! It accepts source-owned, coplanar face disks when both solids replay the same
//! simple projected boundary with opposite signed area. The certificate retains
//! source face lists, boundary points, and edge incidences while exact
//! predicates certify replayed topology in both source and projected spaces.
//!
//! The strict point-in-ring check uses the even-odd crossing classifier of
//! Hormann and Agathos, "The point in polygon problem for arbitrary polygons."
//! Boundary cycles are reconstructed from candidate edge graphs; broader
//! non-rectilinear coplanar-cell materialization remains separate from this
//! full-face shortcut.

use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet};

use hyperlimit::{
    CoplanarProjection, Point2, Point3, RingPointLocation, SegmentIntersection, Sign,
    classify_point_ring_even_odd, classify_segment_intersection, compare_reals, orient3d_report,
    point_on_segment3, point2_equal, project_point3, projected_polygon_area2_value,
};

use super::super::super::{ExactMesh, sorted_edge};
use super::super::{cloned_indexed_points, point3_exact_equal};
use super::{point_on_triangle_plane, real_sign, triangle_point_refs};
use hyperreal::Real;

#[derive(Clone, Debug, PartialEq)]
struct PolygonPatchCandidate {
    faces: Vec<usize>,
    boundary_points: Vec<Point3>,
    signed_area2: Real,
    area_abs: Real,
}

/// Discover source-owned simple-polygon adjacency patch pairs.
///
/// The input triangles are split into edge-connected components. Each component
/// is certified directly, while connected subpatches are exhaustively searched
/// inside that finite component.
///
/// Source topology is replayed from combinatorial adjacency, while exact
/// predicates certify coplanarity, interior inclusion, and signed-area
/// agreement.
pub(crate) fn polygon_patch_pairs(
    left: &ExactMesh,
    consumed_left_faces: &BTreeSet<usize>,
    right: &ExactMesh,
    consumed_right_faces: &BTreeSet<usize>,
) -> Option<Vec<(Vec<usize>, Vec<usize>)>> {
    let left_candidates = polygon_patch_candidates(left, consumed_left_faces)?;
    let right_candidates = polygon_patch_candidates(right, consumed_right_faces)?;
    Some(pair_polygon_patch_candidates(
        &left_candidates,
        &right_candidates,
    ))
}

fn pair_polygon_patch_candidates(
    left_candidates: &[PolygonPatchCandidate],
    right_candidates: &[PolygonPatchCandidate],
) -> Vec<(Vec<usize>, Vec<usize>)> {
    let mut pairs = Vec::new();
    let mut used_left = BTreeSet::new();
    let mut used_right = BTreeSet::new();

    for left_candidate in left_candidates {
        if left_candidate
            .faces
            .iter()
            .any(|face| used_left.contains(face))
        {
            continue;
        }
        let Some(right_candidate) = right_candidates.iter().find(|candidate| {
            candidate
                .faces
                .iter()
                .all(|face| !used_right.contains(face))
                && polygon_patch_candidates_match(left_candidate, candidate)
        }) else {
            continue;
        };
        used_left.extend(left_candidate.faces.iter().copied());
        used_right.extend(right_candidate.faces.iter().copied());
        pairs.push((left_candidate.faces.clone(), right_candidate.faces.clone()));
    }

    pairs
}

fn polygon_patch_candidates(
    mesh: &ExactMesh,
    consumed_faces: &BTreeSet<usize>,
) -> Option<Vec<PolygonPatchCandidate>> {
    let mut candidates = Vec::new();
    let available = mesh
        .view()
        .faces()
        .map(|face| face.index())
        .filter(|face| !consumed_faces.contains(face))
        .collect::<Vec<_>>();
    if available.is_empty() {
        return None;
    }
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
        // guessed subset. Certify it directly before enumerating proper connected
        // subpatches.
        if let Some(candidate) = polygon_patch_candidate(mesh, &component)? {
            seen.insert(component.clone());
            candidates.push(candidate);
        }
        // Enumerate from every face in the component. The recursive collector keeps
        // only extensions whose id is at least `start_face`, so each connected
        // subset has a unique minimum-face root. Starting only at component[0]
        // misses valid source disks nested inside a larger coplanar source
        // component, which is exactly the kind of retained evidence/topology split
        if let Some(path) = ordered_dual_path_component(&neighbors, &component) {
            collect_path_polygon_patch_candidates(mesh, &path, &mut seen, &mut candidates)?;
        } else {
            for &start_face in &component {
                collect_polygon_patch_candidates(
                    mesh,
                    &neighbors,
                    start_face,
                    &mut vec![start_face],
                    &mut seen,
                    &mut candidates,
                )?;
            }
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

fn collect_path_polygon_patch_candidates(
    mesh: &ExactMesh,
    path: &[usize],
    seen: &mut BTreeSet<Vec<usize>>,
    candidates: &mut Vec<PolygonPatchCandidate>,
) -> Option<()> {
    for start in 0..path.len() {
        for end in start + 2..=path.len() {
            let mut key = path[start..end].to_vec();
            key.sort_unstable();
            if !seen.contains(&key) {
                let candidate = polygon_patch_candidate(mesh, &key)?;
                seen.insert(key);
                if let Some(candidate) = candidate {
                    candidates.push(candidate);
                }
            }
        }
    }
    Some(())
}

fn collect_polygon_patch_candidates(
    mesh: &ExactMesh,
    neighbors: &BTreeMap<usize, BTreeSet<usize>>,
    start_face: usize,
    selected: &mut Vec<usize>,
    seen: &mut BTreeSet<Vec<usize>>,
    candidates: &mut Vec<PolygonPatchCandidate>,
) -> Option<()> {
    if selected.len() >= 2 {
        let mut key = selected.clone();
        key.sort_unstable();
        if !seen.contains(&key) {
            let candidate = polygon_patch_candidate(mesh, &key)?;
            seen.insert(key);
            if let Some(candidate) = candidate {
                candidates.push(candidate);
            }
        }
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
        collect_polygon_patch_candidates(mesh, neighbors, start_face, selected, seen, candidates)?;
        selected.pop();
    }
    Some(())
}

fn polygon_patch_candidate(
    mesh: &ExactMesh,
    faces: &[usize],
) -> Option<Option<PolygonPatchCandidate>> {
    if faces.len() < 2 {
        return Some(None);
    }
    // Boundary topology is reconstructed from source-owned edge incidences. Candidate
    // patches must be a triangulated disk: every edge appears at most twice and at
    // least one boundary cycle remains after interior-edge cancellation.
    let mut edge_counts = BTreeMap::<[usize; 2], usize>::new();
    for &face in faces {
        for edge in mesh
            .facts()
            .faces
            .get(face)?
            .oriented
            .directed_edges
            .map(sorted_edge)
        {
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
    if boundary_edges.len() < 3 {
        return Some(None);
    }
    let Some(boundary_vertices) = order_boundary_vertices(&boundary_edges) else {
        return Some(None);
    };
    if boundary_vertices.len() != boundary_edges.len() {
        return Some(None);
    }
    let boundary_points =
        cloned_indexed_points(mesh.view().vertices(), boundary_vertices.iter().copied())?;
    let Some(projection) = [
        CoplanarProjection::Xy,
        CoplanarProjection::Xz,
        CoplanarProjection::Yz,
    ]
    .into_iter()
    .find(|&projection| {
        let area = projected_polygon_area2_value(&boundary_points, projection);
        !matches!(real_sign(&area), Some(Sign::Zero) | None)
    }) else {
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
        let triangle = mesh.view().face(face)?.vertex_indices();
        let points = triangle_point_refs(mesh, triangle)?;
        if boundary_points.len() < 3 {
            return Some(None);
        }
        let (a, b, c) = (
            &boundary_points[0],
            &boundary_points[1],
            &boundary_points[2],
        );
        if !points
            .iter()
            .all(|point| point_on_triangle_plane(a, b, c, point) == Some(true))
        {
            return Some(None);
        }
        for point in &points {
            if !boundary_points
                .iter()
                .any(|boundary_point| point3_exact_equal(boundary_point, point) == Some(true))
                && !point_strictly_inside_simple_loop(point, &boundary_ring, projection)?
            {
                return Some(None);
            }
        }
        let projected_triangle = [
            (*points[0]).clone(),
            (*points[1]).clone(),
            (*points[2]).clone(),
        ];
        let area = projected_polygon_area2_value(&projected_triangle, projection);
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

    let area_abs = match real_sign(&signed_area2)? {
        Sign::Negative => -signed_area2.clone(),
        Sign::Zero | Sign::Positive => signed_area2.clone(),
    };
    let boundary_area = projected_polygon_area2_value(&boundary_points, projection);
    let boundary_area_abs = match real_sign(&boundary_area)? {
        Sign::Negative => -boundary_area,
        Sign::Zero | Sign::Positive => boundary_area,
    };
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
    let mut edge_faces = BTreeMap::<[usize; 2], Vec<usize>>::new();
    for &face in faces {
        for edge in mesh
            .facts()
            .faces
            .get(face)?
            .oriented
            .directed_edges
            .map(sorted_edge)
        {
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

fn ordered_dual_path_component(
    neighbors: &BTreeMap<usize, BTreeSet<usize>>,
    component: &[usize],
) -> Option<Vec<usize>> {
    if component.len() < 2 {
        return Some(component.to_vec());
    }
    let component_set = component.iter().copied().collect::<BTreeSet<_>>();
    let mut endpoints = Vec::new();
    for &face in component {
        let degree = neighbors
            .get(&face)?
            .iter()
            .filter(|neighbor| component_set.contains(neighbor))
            .count();
        match degree {
            0 => return None,
            1 => endpoints.push(face),
            2 => {}
            _ => return None,
        }
    }
    if endpoints.len() != 2 {
        return None;
    }

    let mut ordered = Vec::with_capacity(component.len());
    let mut previous = usize::MAX;
    let mut current = endpoints[0].min(endpoints[1]);
    loop {
        ordered.push(current);
        let next = neighbors
            .get(&current)?
            .iter()
            .copied()
            .filter(|neighbor| component_set.contains(neighbor) && *neighbor != previous)
            .min();
        let Some(next) = next else {
            break;
        };
        previous = current;
        current = next;
        if ordered.contains(&current) {
            return None;
        }
    }
    (ordered.len() == component.len()).then_some(ordered)
}

fn faces_are_coplanar(mesh: &ExactMesh, left_face: usize, right_face: usize) -> Option<bool> {
    // Source-disk discovery is a planar certificate, not a shell-connectivity
    // topology by exact retained planes before promoting a connected component
    // to a planar disk candidate.
    let left_triangle = mesh.view().face(left_face)?.vertex_indices();
    let right_triangle = mesh.view().face(right_face)?.vertex_indices();
    let left_points = triangle_point_refs(mesh, left_triangle)?;
    let right_points = triangle_point_refs(mesh, right_triangle)?;
    for point in right_points {
        if orient3d_report(left_points[0], left_points[1], left_points[2], point).value()?
            != Sign::Zero
        {
            return Some(false);
        }
    }
    for point in left_points {
        if orient3d_report(right_points[0], right_points[1], right_points[2], point).value()?
            != Sign::Zero
        {
            return Some(false);
        }
    }
    Some(true)
}

fn extract_polygon_patch_component(
    start_face: usize,
    neighbors: &BTreeMap<usize, BTreeSet<usize>>,
    available: &mut BTreeSet<usize>,
) -> Option<Vec<usize>> {
    // Traverse a source-owned connected component to avoid enumerating disconnected
    // unions; connected subpatches are searched from the component faces above.
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

fn order_boundary_vertices(edges: &[[usize; 2]]) -> Option<Vec<usize>> {
    // Reconstruct a single boundary cycle from degree-2 adjacency.
    let mut adjacency = BTreeMap::<usize, Vec<usize>>::new();
    for &[a, b] in edges {
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
            if point2_equal(&projected[left], &projected[right]).value()? {
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

fn polygon_patch_candidates_match(
    left: &PolygonPatchCandidate,
    right: &PolygonPatchCandidate,
) -> bool {
    boundary_loops_equivalent(&left.boundary_points, &right.boundary_points) == Some(true)
        && compare_reals(&left.area_abs, &right.area_abs).value() == Some(Ordering::Equal)
        && compare_reals(
            &(left.signed_area2.clone() + right.signed_area2.clone()),
            &Real::from(0),
        )
        .value()
            == Some(Ordering::Equal)
}

fn boundary_loops_equivalent(left: &[Point3], right: &[Point3]) -> Option<bool> {
    if left.len() < 3 || right.len() < 3 {
        return Some(false);
    }
    for point in left {
        if !point_on_boundary_loop(point, right)? {
            return Some(false);
        }
    }
    for point in right {
        if !point_on_boundary_loop(point, left)? {
            return Some(false);
        }
    }
    Some(true)
}

fn point_on_boundary_loop(point: &Point3, boundary: &[Point3]) -> Option<bool> {
    for index in 0..boundary.len() {
        if point_on_segment3(
            &boundary[index],
            &boundary[(index + 1) % boundary.len()],
            point,
        )
        .value()?
        {
            return Some(true);
        }
    }
    Some(false)
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeSet;

    use super::*;
    use crate::mesh::validation::ExactMeshValidationPolicy;
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
            ExactMeshValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap()
    }

    fn point(x: i64, y: i64, z: i64) -> Point3 {
        Point3::new(Real::from(x), Real::from(y), Real::from(z))
    }

    fn patch_candidate(faces: Vec<usize>, signed_area2: i64) -> PolygonPatchCandidate {
        PolygonPatchCandidate {
            faces,
            boundary_points: vec![point(0, 0, 0), point(1, 0, 0), point(0, 1, 0)],
            signed_area2: Real::from(signed_area2),
            area_abs: Real::from(signed_area2.abs()),
        }
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

    fn wide_boundary_subpatch_pair() -> (ExactMesh, ExactMesh) {
        let mut left_points = vec![[0, 0, 0]];
        for index in 0..=9 {
            left_points.push([index, 1, 0]);
        }
        let mut left_triangles = Vec::new();
        for index in 1..10 {
            left_triangles.extend([0, index, index + 1]);
        }
        let left = open_mesh(&left_points, &left_triangles);

        let mut right_points = vec![[0, 0, 0]];
        for index in 1..=9 {
            right_points.push([index, 1, 0]);
        }
        let mut right_triangles = Vec::new();
        for index in 1..9 {
            right_triangles.extend([0, index + 1, index]);
        }
        let right = open_mesh(&right_points, &right_triangles);

        (left, right)
    }

    fn wide_face_subpatch_pair() -> (ExactMesh, ExactMesh) {
        let mut left_points = vec![[0, 0, 0]];
        for index in 0..=11 {
            left_points.push([index, 1, 0]);
        }
        let mut left_triangles = Vec::new();
        for index in 1..12 {
            left_triangles.extend([0, index, index + 1]);
        }
        let left = open_mesh(&left_points, &left_triangles);

        let mut right_points = vec![[0, 0, 0]];
        for index in 1..=11 {
            right_points.push([index, 1, 0]);
        }
        let mut right_triangles = Vec::new();
        for index in 1..11 {
            right_triangles.extend([0, index + 1, index]);
        }
        let right = open_mesh(&right_points, &right_triangles);

        (left, right)
    }

    #[test]
    fn polygon_patch_pairs_find_subpatch_not_rooted_at_component_minimum() {
        let (left, right) = shifted_square_subpatch_pair(1, 4, 4);

        let pairs = polygon_patch_pairs(&left, &BTreeSet::new(), &right, &BTreeSet::new())
            .expect("source-disk candidates should be available");

        assert_eq!(pairs, vec![(vec![1, 2], vec![0, 1])]);
    }

    #[test]
    fn polygon_patch_pairing_reserves_right_source_faces() {
        let left_candidates = vec![
            patch_candidate(vec![0, 1], 1),
            patch_candidate(vec![2, 3], 1),
        ];
        let right_candidates = vec![
            patch_candidate(vec![10, 11], -1),
            patch_candidate(vec![11, 12], -1),
            patch_candidate(vec![12, 13], -1),
        ];

        let pairs = pair_polygon_patch_candidates(&left_candidates, &right_candidates);

        assert_eq!(
            pairs,
            vec![(vec![0, 1], vec![10, 11]), (vec![2, 3], vec![12, 13])]
        );
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

    #[test]
    fn polygon_patch_pairs_accept_subpatch_with_large_boundary() {
        let (left, right) = wide_boundary_subpatch_pair();
        let left_candidates = polygon_patch_candidates(&left, &BTreeSet::new())
            .expect("left source-disk candidates should be available");

        assert!(
            left_candidates
                .iter()
                .any(|candidate| candidate.faces == (1..9).collect::<Vec<_>>()
                    && candidate.boundary_points.len() == 10),
            "{left_candidates:?}"
        );

        let pairs = polygon_patch_pairs(&left, &BTreeSet::new(), &right, &BTreeSet::new())
            .expect("large-boundary source-disk pair should be available");

        assert_eq!(pairs, vec![((1..9).collect(), (0..8).collect())]);
    }

    #[test]
    fn polygon_patch_pairs_accept_subpatch_with_many_faces() {
        let (left, right) = wide_face_subpatch_pair();
        let left_candidates = polygon_patch_candidates(&left, &BTreeSet::new())
            .expect("left source-disk candidates should be available");

        assert!(
            left_candidates
                .iter()
                .any(|candidate| candidate.faces == (1..11).collect::<Vec<_>>()),
            "{left_candidates:?}"
        );

        let pairs = polygon_patch_pairs(&left, &BTreeSet::new(), &right, &BTreeSet::new())
            .expect("many-face source-disk pair should be available");

        assert_eq!(pairs, vec![((1..11).collect(), (0..10).collect())]);
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
                .expect("source-disk candidates should be available");

            prop_assert_eq!(pairs, vec![(vec![1, 2], vec![0, 1])]);
        }

        #[test]
        fn generated_oversized_whole_components_are_retained(
            extra_faces in 1_usize..12,
        ) {
            let face_count = OVERSIZED_COMPONENT_FACES + extra_faces;
            let mesh = oversized_component_fan(face_count, false);
            let faces = (0..face_count).collect::<Vec<_>>();
            let candidate = polygon_patch_candidate(&mesh, &faces)
                .expect("whole source-disk candidate should be decidable");

            prop_assert!(candidate.is_some());
        }
    }
}
