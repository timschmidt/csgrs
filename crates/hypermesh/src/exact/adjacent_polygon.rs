//! Bounded convex-polygon full-face adjacency certificates.
//!
//! This module is the branch-face companion to [`crate::exact::adjacent`].
//! It accepts a small source-owned triangulated disk on each closed solid when
//! both disks replay to the same strictly convex projected boundary with
//! opposite signed area. The certificate deliberately stops at a bounded
//! branch grammar: diagonal, ear, and fan-spoke edges can be deleted only when
//! exact source topology and exact projected predicates prove that they are
//! internal to one retained boundary object.
//!
//! The separation follows Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7.1-2 (1997): we keep the source triangles as the
//! combinatorial object and use exact predicates only to certify that this
//! object owns the topology change. General nonconvex or larger coplanar cell
//! extraction remains outside this bounded certificate.

use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet};

use hyperlimit::{
    CoplanarProjection, Point3, Sign, compare_reals, orient3d_report, projected_polygon_area2_value,
};

use super::mesh::ExactMesh;
use super::scalar::ExactReal;

const MAX_POLYGON_PATCH_FACES: usize = 8;
const MAX_POLYGON_PATCH_BOUNDARY: usize = 8;

#[derive(Clone, Debug, PartialEq)]
struct PolygonPatchCandidate {
    faces: Vec<usize>,
    boundary_points: Vec<Point3>,
    signed_area2: ExactReal,
    area_abs: ExactReal,
}

/// Discover bounded convex-polygon adjacency patch pairs.
///
/// The accepted grammar covers small triangulated disks whose exposed
/// boundary has at most eight vertices. This includes quadrilateral
/// cross-diagonals, one-sided quad fans, and pentagonal through octagonal
/// fan/boundary-ear triangulations. This is still not an arbitrary coplanar
/// arrangement materializer: every subset must be edge-connected, replay as a
/// source-owned disk, place all non-boundary vertices strictly inside the
/// retained convex loop, and cancel signed projected area against the opposite
/// operand.
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
    let neighbors = edge_connected_face_neighbors(mesh, &available)?;
    let mut seen = BTreeSet::<Vec<usize>>::new();
    for &start_face in &available {
        collect_polygon_patch_candidates(
            mesh,
            &neighbors,
            start_face,
            &mut vec![start_face],
            &mut seen,
            &mut candidates,
        )?;
    }
    Some(candidates)
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

fn collect_polygon_patch_candidates(
    mesh: &ExactMesh,
    neighbors: &BTreeMap<usize, BTreeSet<usize>>,
    start_face: usize,
    selected: &mut Vec<usize>,
    seen: &mut BTreeSet<Vec<usize>>,
    candidates: &mut Vec<PolygonPatchCandidate>,
) -> Option<()> {
    if (2..=MAX_POLYGON_PATCH_FACES).contains(&selected.len()) {
        let mut key = selected.clone();
        key.sort_unstable();
        if seen.insert(key.clone())
            && let Some(candidate) = polygon_patch_candidate(mesh, &key)?
        {
            candidates.push(candidate);
        }
    }
    if selected.len() == MAX_POLYGON_PATCH_FACES {
        return Some(());
    }

    let selected_set = selected.iter().copied().collect::<BTreeSet<_>>();
    let mut extensions = BTreeSet::new();
    for face in selected.iter().copied() {
        for &neighbor in neighbors.get(&face)? {
            if neighbor > start_face && !selected_set.contains(&neighbor) {
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
    if !(3..=MAX_POLYGON_PATCH_BOUNDARY).contains(&boundary_edges.len()) {
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
    if !loop_is_strictly_convex(&boundary_points, projection)? {
        return Some(None);
    }

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
                && !point_strictly_inside_convex_loop(point, &boundary_points, projection)?
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

fn order_boundary_vertices(edges: &[(usize, usize)]) -> Option<Vec<usize>> {
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

fn loop_is_strictly_convex(points: &[Point3], projection: CoplanarProjection) -> Option<bool> {
    if points.len() < 3 {
        return Some(false);
    }
    let area_sign = real_sign(&projected_polygon_area2_value(points, projection))?;
    if area_sign == Sign::Zero {
        return Some(false);
    }
    for index in 0..points.len() {
        let a = &points[index];
        let b = &points[(index + 1) % points.len()];
        let c = &points[(index + 2) % points.len()];
        let turn = projected_polygon_area2_value(&[a.clone(), b.clone(), c.clone()], projection);
        if real_sign(&turn)? != area_sign {
            return Some(false);
        }
    }
    Some(true)
}

fn point_strictly_inside_convex_loop(
    point: &Point3,
    loop_points: &[Point3],
    projection: CoplanarProjection,
) -> Option<bool> {
    if loop_points.len() < 3 {
        return Some(false);
    }
    let area_sign = real_sign(&projected_polygon_area2_value(loop_points, projection))?;
    if area_sign == Sign::Zero {
        return Some(false);
    }

    for index in 0..loop_points.len() {
        let a = &loop_points[index];
        let b = &loop_points[(index + 1) % loop_points.len()];
        let turn =
            projected_polygon_area2_value(&[a.clone(), b.clone(), point.clone()], projection);
        if real_sign(&turn)? != area_sign {
            return Some(false);
        }
    }
    Some(true)
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
