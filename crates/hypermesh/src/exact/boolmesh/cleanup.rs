//! Exact final cleanup for boolmesh `boolean45` export candidates.
//!
//! Legacy boolmesh runs `simplify_topology` and `cleanup_unused_verts` between
//! `triangulate` and `Manifold::new_impl`.  This module ports the part needed at
//! the exact object boundary: coincident output slots are merged only by exact
//! coordinate equality, degenerate triangles created by that merge are dropped,
//! the remaining triangle soup is oriented as a halfedge surface, and vertices
//! no longer referenced by any surviving triangle are removed.  Yap, "Towards
//! Exact Geometric Computation," *Computational Geometry* 7.1-2 (1997), is the
//! governing constraint here: cleanup may change topology only when exact
//! predicates decide the equality or incidence being used.

use std::collections::{BTreeMap, BTreeSet, VecDeque};

use hyperlimit::{Point3, Sign, compare_reals, orient3d_report, point_on_segment3};

use crate::exact::mesh::{ExactPoint3, Triangle};
use crate::exact::predicates::{TriangleDegeneracy, classify_triangle_degeneracy};
#[cfg(feature = "internal-fuzzing")]
use crate::exact::scalar::ExactReal;

/// Collapse exactly equal export coordinates and orient the resulting surface.
///
/// The `boolean45` stages intentionally preserve boolmesh output vertex slots:
/// multiple signed `Boolean03` rows may name the same exact point until the
/// final mesh boundary.  This function performs the boolmesh cleanup handoff
/// without introducing tolerances:
///
/// - vertices are welded only when all three coordinates compare exactly equal;
/// - triangles that become index-degenerate after welding are removed;
/// - exact duplicate opposite-oriented triangle pairs are canceled as
///   zero-thickness interface debris;
/// - each connected triangle component is flipped, when necessary, so every
///   two-face edge is traversed in opposite directions by its incident faces.
/// - vertices made unused by those exact triangle deletions are compacted away.
///
/// The orientation pass is the halfedge consistency condition used by the
/// boolmesh boolean kernel after topology simplification; see Komikado's
/// boolmesh-derived `simplify_topology`/`Manifold::new_impl` handoff in this
/// crate.  The final compaction ports boolmesh `cleanup_unused_verts` without
/// the legacy f64 Morton ordering: the ordering was a storage-locality detail,
/// while the exact kernel's topological contract is the retained old-to-new
/// vertex map over surviving triangle incidence.  The exact port keeps that
/// algorithmic boundary but replaces the primitive-float equality shortcut
/// with `hyperlimit::compare_reals`.
pub(super) fn cleanup_exact_export_vertices(
    raw_vertices: Vec<ExactPoint3>,
    raw_triangles: &[Triangle],
) -> (Vec<ExactPoint3>, Vec<Triangle>) {
    let raw_points = raw_vertices
        .iter()
        .map(ExactPoint3::to_hyperlimit_point)
        .collect::<Vec<_>>();
    let mut unique_vertices = Vec::<ExactPoint3>::new();
    let mut unique_points = Vec::<Point3>::new();
    let mut remap = Vec::with_capacity(raw_vertices.len());

    for (index, point) in raw_points.iter().enumerate() {
        let existing = unique_points
            .iter()
            .position(|candidate| exact_points_equal(candidate, point));
        if let Some(existing) = existing {
            remap.push(existing);
        } else {
            remap.push(unique_vertices.len());
            unique_vertices.push(raw_vertices[index].clone());
            unique_points.push(point.clone());
        }
    }

    let triangles = raw_triangles
        .iter()
        .filter_map(|triangle| {
            let vertices = [
                *remap.get(triangle.0[0])?,
                *remap.get(triangle.0[1])?,
                *remap.get(triangle.0[2])?,
            ];
            (vertices[0] != vertices[1] && vertices[1] != vertices[2] && vertices[2] != vertices[0])
                .then_some(Triangle(vertices))
        })
        .collect::<Vec<_>>();

    let triangles = remove_isolated_opposite_duplicate_pairs(triangles);
    let triangles = split_edges_at_existing_vertices(&unique_points, triangles);
    let triangles = orient_triangle_components(triangles);
    let triangles = close_coplanar_boundary_loops(&unique_points, triangles);
    let triangles = remove_internal_nonmanifold_triangles(triangles);

    compact_unused_vertices(unique_vertices, triangles)
}

/// Drop welded vertices that no surviving triangle references.
///
/// Legacy boolmesh `cleanup_unused_verts` runs immediately before
/// `Manifold::new_impl`: it rewrites halfedge vertex ids through a dense map
/// and truncates positions whose Morton code marked them unused.  The exact
/// port keeps the dense-map topology step and removes only the f64 Morton
/// sorting/truncation detail.  That is the Yap exact-object boundary applied to
/// cleanup: a vertex is removed because triangle incidence no longer names it,
/// not because a rounded coordinate bucket says it is disposable.
fn compact_unused_vertices(
    vertices: Vec<ExactPoint3>,
    triangles: Vec<Triangle>,
) -> (Vec<ExactPoint3>, Vec<Triangle>) {
    if triangles.is_empty() {
        return (Vec::new(), Vec::new());
    }
    if triangles
        .iter()
        .flat_map(|triangle| triangle.0)
        .any(|vertex| vertex >= vertices.len())
    {
        return (vertices, triangles);
    }

    let mut used = vec![false; vertices.len()];
    for triangle in &triangles {
        for vertex in triangle.0 {
            used[vertex] = true;
        }
    }

    let mut old_to_new = vec![None; vertices.len()];
    let mut compact_vertices = Vec::new();
    for (old, vertex) in vertices.into_iter().enumerate() {
        if used[old] {
            old_to_new[old] = Some(compact_vertices.len());
            compact_vertices.push(vertex);
        }
    }

    let compact_triangles = triangles
        .into_iter()
        .map(|triangle| {
            Triangle([
                old_to_new[triangle.0[0]].expect("used triangle vertex must have a compact index"),
                old_to_new[triangle.0[1]].expect("used triangle vertex must have a compact index"),
                old_to_new[triangle.0[2]].expect("used triangle vertex must have a compact index"),
            ])
        })
        .collect();

    (compact_vertices, compact_triangles)
}

/// Remove exact coincident opposite-facing triangle pairs.
///
/// Legacy boolmesh `simplify_topology` removes local zero-thickness interface
/// debris before `Manifold::new_impl`.  After exact vertex welding, that debris
/// appears as two triangles with the same three vertex ids and opposite
/// halfedge directions.  The exact port cancels only the isolated case where
/// each of the three undirected edges is used by exactly those two triangles;
/// non-isolated coincident faces are left for the later overfull-edge cleanup
/// or final validation.  This keeps the mutation inside Yap's exact-object
/// boundary: equality is exact vertex identity after `compare_reals`, and
/// topology is changed only for a replayable duplicate face pair.
fn remove_isolated_opposite_duplicate_pairs(triangles: Vec<Triangle>) -> Vec<Triangle> {
    if triangles.len() < 2 {
        return triangles;
    }

    let edge_counts = edge_use_counts(&triangles);
    let mut by_vertices = BTreeMap::<[usize; 3], Vec<usize>>::new();
    for (face, triangle) in triangles.iter().enumerate() {
        by_vertices
            .entry(sorted_triangle(triangle.0))
            .or_default()
            .push(face);
    }

    let mut removed = vec![false; triangles.len()];
    for faces in by_vertices.values() {
        if faces.len() != 2 {
            continue;
        }
        let left = triangles[faces[0]];
        let right = triangles[faces[1]];
        if !triangles_are_opposite_duplicates(left, right)
            || !duplicate_pair_edges_are_isolated(left, &edge_counts)
        {
            continue;
        }
        removed[faces[0]] = true;
        removed[faces[1]] = true;
    }

    triangles
        .into_iter()
        .enumerate()
        .filter_map(|(face, triangle)| (!removed[face]).then_some(triangle))
        .collect()
}

fn triangles_are_opposite_duplicates(left: Triangle, right: Triangle) -> bool {
    sorted_triangle(left.0) == sorted_triangle(right.0)
        && directed_edges(left.0).iter().all(|edge| {
            directed_edges(right.0)
                .iter()
                .any(|candidate| *candidate == [edge[1], edge[0]])
        })
}

fn duplicate_pair_edges_are_isolated(
    triangle: Triangle,
    edge_counts: &BTreeMap<[usize; 2], DirectedEdgeUseCount>,
) -> bool {
    directed_edges(triangle.0).iter().all(|edge| {
        edge_counts
            .get(&sorted_edge(*edge))
            .is_some_and(|count| count.forward + count.reverse == 2)
    })
}

/// Split triangle edges at existing exact vertices that lie in their interiors.
///
/// This ports the part of boolmesh `simplify_topology` that makes later
/// halfedge pairing possible after coincident slots have been welded.  A
/// positive-area coplanar overlap can leave a source vertex exactly on a
/// neighboring triangle edge; legacy boolmesh handles that before
/// `Manifold::new_impl` by refining the triangle edge.  The exact port uses
/// `hyperlimit::point_on_segment3` and exact coordinate inequality, following
/// Yap's exact-geometric-computation contract: no epsilon decides whether a
/// vertex lies on an edge, and each split preserves the original triangle
/// orientation.
fn split_edges_at_existing_vertices(points: &[Point3], triangles: Vec<Triangle>) -> Vec<Triangle> {
    let mut triangles = triangles;
    let max_splits = points
        .len()
        .saturating_mul(triangles.len())
        .saturating_mul(3);
    for _ in 0..max_splits {
        let Some(split) = find_existing_vertex_edge_split(points, &triangles) else {
            return triangles;
        };
        let triangle = triangles[split.face].0;
        let edge = triangle_edge_indices(triangle, split.edge);
        let opposite = triangle[(split.edge + 2) % 3];
        let left = [edge[0], split.vertex, opposite];
        let right = [split.vertex, edge[1], opposite];
        if !triangle_is_exactly_nondegenerate(points, left)
            || !triangle_is_exactly_nondegenerate(points, right)
        {
            return triangles;
        }
        triangles[split.face] = Triangle(left);
        triangles.push(Triangle(right));
    }
    triangles
}

fn find_existing_vertex_edge_split(points: &[Point3], triangles: &[Triangle]) -> Option<EdgeSplit> {
    for (face, triangle) in triangles.iter().enumerate() {
        for edge in 0..3 {
            let [tail, head] = triangle_edge_indices(triangle.0, edge);
            for vertex in 0..points.len() {
                if triangle.0.contains(&vertex)
                    || !point_strictly_inside_segment(points, tail, head, vertex)
                {
                    continue;
                }
                return Some(EdgeSplit { face, edge, vertex });
            }
        }
    }
    None
}

#[derive(Clone, Copy)]
struct EdgeSplit {
    face: usize,
    edge: usize,
    vertex: usize,
}

fn triangle_edge_indices(triangle: [usize; 3], edge: usize) -> [usize; 2] {
    match edge {
        0 => [triangle[0], triangle[1]],
        1 => [triangle[1], triangle[2]],
        _ => [triangle[2], triangle[0]],
    }
}

fn point_strictly_inside_segment(
    points: &[Point3],
    tail: usize,
    head: usize,
    vertex: usize,
) -> bool {
    point_on_segment3(&points[tail], &points[head], &points[vertex]).value() == Some(true)
        && !exact_points_equal(&points[tail], &points[vertex])
        && !exact_points_equal(&points[head], &points[vertex])
}

/// Fill exactly coplanar boundary cycles left by boolmesh topology cleanup.
///
/// Legacy boolmesh lets `simplify_topology` collapse and swap degenerate
/// triangles before `Manifold::new_impl`; in positive-area coplanar overlaps
/// that cleanup can leave a single oriented boundary cycle where source faces
/// have already been welded into one exact output surface.  This is the exact
/// port of that handoff, constrained by Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997): a cap is emitted only
/// when the boundary graph is a set of simple directed degree-two cycles, all
/// vertices in a cycle are certified coplanar by `hyperlimit::orient3d_report`,
/// and every fan triangle is certified nondegenerate by
/// `hyperlimit::classify_triangle3_degeneracy`.  The fan anchor is deliberately
/// not fixed: boolmesh cleanup removes folded/collinear ears before manifold
/// construction, so the exact port tries each boundary vertex and accepts only
/// an anchor whose replay contains no degenerate or duplicate triangle.
fn close_coplanar_boundary_loops(points: &[Point3], triangles: Vec<Triangle>) -> Vec<Triangle> {
    let Some(boundary_loops) = directed_boundary_loops(&triangles) else {
        return triangles;
    };
    if boundary_loops.is_empty() {
        return triangles;
    }

    let mut occupied_triangles = triangles
        .iter()
        .map(|triangle| sorted_triangle(triangle.0))
        .collect::<BTreeSet<_>>();
    let base_edge_uses = edge_use_counts(&triangles);
    let mut cap_triangles = Vec::new();
    for boundary_loop in boundary_loops {
        let Some(mut loop_caps) = triangulate_coplanar_boundary_loop(
            points,
            &boundary_loop,
            &mut occupied_triangles,
            &base_edge_uses,
        ) else {
            return triangles;
        };
        cap_triangles.append(&mut loop_caps);
    }

    if cap_triangles.is_empty() {
        return triangles;
    }

    let mut closed = triangles;
    closed.extend(cap_triangles);
    orient_triangle_components(closed)
}

/// Extract simple directed boundary cycles from an already oriented soup.
///
/// Boolmesh's final manifold constructor expects each remaining boundary edge
/// to be paired by cleanup, so this helper refuses ambiguous topology instead
/// of guessing.  The degree-one directed-edge model mirrors the halfedge
/// adjacency check used by `Manifold::new_impl`: every boundary vertex must
/// have exactly one outgoing and one incoming boundary halfedge.
fn directed_boundary_loops(triangles: &[Triangle]) -> Option<Vec<Vec<usize>>> {
    let mut edge_uses = BTreeMap::<[usize; 2], Vec<[usize; 2]>>::new();
    for triangle in triangles {
        for edge in directed_edges(triangle.0) {
            edge_uses.entry(sorted_edge(edge)).or_default().push(edge);
        }
    }

    let mut successors = BTreeMap::<usize, usize>::new();
    let mut predecessors = BTreeMap::<usize, usize>::new();
    for uses in edge_uses.values() {
        if uses.len() > 2 {
            return None;
        }
        if uses.len() != 1 {
            continue;
        }
        let [tail, head] = uses[0];
        if tail == head
            || successors.insert(tail, head).is_some()
            || predecessors.insert(head, tail).is_some()
        {
            return None;
        }
    }

    if successors.is_empty() {
        return Some(Vec::new());
    }
    if successors.len() != predecessors.len()
        || successors
            .keys()
            .any(|vertex| !predecessors.contains_key(vertex))
    {
        return None;
    }

    let mut visited = BTreeSet::new();
    let mut loops = Vec::new();
    for &start in successors.keys() {
        if visited.contains(&start) {
            continue;
        }
        let mut boundary_loop = Vec::new();
        let mut current = start;
        loop {
            if !visited.insert(current) {
                return None;
            }
            boundary_loop.push(current);
            let next = *successors.get(&current)?;
            if next == start {
                break;
            }
            if visited.contains(&next) {
                return None;
            }
            current = next;
        }
        if boundary_loop.len() < 3 {
            return None;
        }
        loops.push(boundary_loop);
    }

    Some(loops)
}

fn triangulate_coplanar_boundary_loop(
    points: &[Point3],
    boundary_loop: &[usize],
    occupied_triangles: &mut BTreeSet<[usize; 3]>,
    base_edge_uses: &BTreeMap<[usize; 2], DirectedEdgeUseCount>,
) -> Option<Vec<Triangle>> {
    let plane = coplanar_loop_plane(points, boundary_loop)?;
    if !boundary_loop
        .iter()
        .all(|&vertex| point_is_on_plane(points, plane, vertex))
    {
        return None;
    }

    for anchor_offset in 0..boundary_loop.len() {
        let mut rotated = boundary_loop[anchor_offset..].to_vec();
        rotated.extend_from_slice(&boundary_loop[..anchor_offset]);
        let mut candidate_occupied = occupied_triangles.clone();
        let mut candidate_edge_uses = base_edge_uses.clone();
        let mut caps = Vec::with_capacity(boundary_loop.len().saturating_sub(2));
        let anchor = rotated[0];
        let mut valid_anchor = true;
        for window in rotated[1..].windows(2) {
            let triangle = [anchor, window[1], window[0]];
            if !triangle_is_exactly_nondegenerate(points, triangle)
                || !candidate_occupied.insert(sorted_triangle(triangle))
                || !insert_triangle_preserving_two_manifold_edges(
                    &mut candidate_edge_uses,
                    triangle,
                )
            {
                valid_anchor = false;
                break;
            }
            caps.push(Triangle(triangle));
        }
        if valid_anchor {
            *occupied_triangles = candidate_occupied;
            return Some(caps);
        }
    }

    None
}

fn coplanar_loop_plane(points: &[Point3], boundary_loop: &[usize]) -> Option<[usize; 3]> {
    for left in 0..boundary_loop.len() {
        for middle in left + 1..boundary_loop.len() {
            for right in middle + 1..boundary_loop.len() {
                let triangle = [
                    boundary_loop[left],
                    boundary_loop[middle],
                    boundary_loop[right],
                ];
                if triangle_is_exactly_nondegenerate(points, triangle) {
                    return Some(triangle);
                }
            }
        }
    }
    None
}

fn point_is_on_plane(points: &[Point3], plane: [usize; 3], vertex: usize) -> bool {
    orient3d_report(
        &points[plane[0]],
        &points[plane[1]],
        &points[plane[2]],
        &points[vertex],
    )
    .value()
        == Some(Sign::Zero)
}

fn triangle_is_exactly_nondegenerate(points: &[Point3], triangle: [usize; 3]) -> bool {
    classify_triangle_degeneracy(
        &points[triangle[0]],
        &points[triangle[1]],
        &points[triangle[2]],
    )
    .degeneracy
        == TriangleDegeneracy::NonDegenerate
}

#[derive(Clone, Copy, Debug, Default)]
struct DirectedEdgeUseCount {
    forward: usize,
    reverse: usize,
}

fn edge_use_counts(triangles: &[Triangle]) -> BTreeMap<[usize; 2], DirectedEdgeUseCount> {
    let mut edge_uses = BTreeMap::<[usize; 2], DirectedEdgeUseCount>::new();
    for triangle in triangles {
        for edge in directed_edges(triangle.0) {
            let key = sorted_edge(edge);
            let count = edge_uses.entry(key).or_default();
            if edge == key {
                count.forward += 1;
            } else {
                count.reverse += 1;
            }
        }
    }
    edge_uses
}

fn insert_triangle_preserving_two_manifold_edges(
    edge_uses: &mut BTreeMap<[usize; 2], DirectedEdgeUseCount>,
    triangle: [usize; 3],
) -> bool {
    let mut trial = edge_uses.clone();
    for edge in directed_edges(triangle) {
        let key = sorted_edge(edge);
        let count = trial.entry(key).or_default();
        if edge == key {
            count.forward += 1;
        } else {
            count.reverse += 1;
        }
        if count.forward > 1 || count.reverse > 1 || count.forward + count.reverse > 2 {
            return false;
        }
    }
    *edge_uses = trial;
    true
}

/// Remove internal interface triangles that are provably topological debris.
///
/// Boolmesh `simplify_topology` removes folded/overlapped local faces before
/// handing the soup to `Manifold::new_impl`.  After exact on-edge refinement
/// and conservative cap insertion, the same situation appears as a triangle
/// whose three edges are all overfull: every edge has more than two incident
/// faces, so the triangle cannot be part of a two-manifold boundary.  Following
/// Yap's exact-object discipline, this pass accepts the mutation only when
/// deleting the triangle and replaying halfedge orientation strictly reduces
/// exact combinatorial edge defects; otherwise the unmodified soup is left for
/// final validation to reject.
fn remove_internal_nonmanifold_triangles(mut triangles: Vec<Triangle>) -> Vec<Triangle> {
    for _ in 0..triangles.len() {
        let baseline_defects = edge_topology_defects(&triangles);
        let Some(face) = triangles.iter().enumerate().find_map(|(face, triangle)| {
            triangle_all_edges_overfull(&triangles, *triangle).then_some(face)
        }) else {
            return triangles;
        };

        let mut candidate = triangles.clone();
        candidate.remove(face);
        candidate = orient_triangle_components(candidate);
        if edge_topology_defects(&candidate) < baseline_defects {
            triangles = candidate;
        } else {
            return triangles;
        }
    }
    triangles
}

fn triangle_all_edges_overfull(triangles: &[Triangle], triangle: Triangle) -> bool {
    let edge_uses = edge_use_counts(triangles);
    directed_edges(triangle.0).iter().all(|edge| {
        edge_uses
            .get(&sorted_edge(*edge))
            .is_some_and(|count| count.forward + count.reverse > 2)
    })
}

fn edge_topology_defects(triangles: &[Triangle]) -> usize {
    edge_use_counts(triangles)
        .values()
        .map(|count| {
            let incident = count.forward + count.reverse;
            incident.saturating_sub(2)
                + count.forward.saturating_sub(1)
                + count.reverse.saturating_sub(1)
        })
        .sum()
}

/// Orient a triangle soup by propagating halfedge direction constraints.
///
/// Each manifold edge should be used once in each direction.  Local
/// source-face orientation in `boolean45::export` can become inconsistent after
/// exact vertex welding because two pre-weld output slots may collapse to a
/// single vertex.  A breadth-first propagation over shared edges is sufficient
/// for each orientable component: if the current face uses an edge in the
/// sorted direction, its neighbor must use the reverse direction, accounting for
/// any flips already assigned.
fn orient_triangle_components(triangles: Vec<Triangle>) -> Vec<Triangle> {
    let mut edge_uses = BTreeMap::<[usize; 2], Vec<EdgeUse>>::new();
    for (face, triangle) in triangles.iter().enumerate() {
        for [tail, head] in directed_edges(triangle.0) {
            let key = sorted_edge([tail, head]);
            edge_uses.entry(key).or_default().push(EdgeUse {
                face,
                forward: [tail, head] == key,
            });
        }
    }

    let mut adjacency = vec![Vec::<NeighborConstraint>::new(); triangles.len()];
    for uses in edge_uses.values() {
        if uses.len() != 2 {
            continue;
        }
        let left = uses[0];
        let right = uses[1];
        adjacency[left.face].push(NeighborConstraint {
            face: right.face,
            current_forward: left.forward,
            neighbor_forward: right.forward,
        });
        adjacency[right.face].push(NeighborConstraint {
            face: left.face,
            current_forward: right.forward,
            neighbor_forward: left.forward,
        });
    }

    let mut flips = vec![None; triangles.len()];
    for seed in 0..triangles.len() {
        if flips[seed].is_some() {
            continue;
        }
        flips[seed] = Some(false);
        let mut queue = VecDeque::from([seed]);
        while let Some(face) = queue.pop_front() {
            let current_flip = flips[face].unwrap_or(false);
            for neighbor in &adjacency[face] {
                let current_effective = neighbor.current_forward ^ current_flip;
                let required_neighbor_flip = neighbor.neighbor_forward ^ !current_effective;
                match flips[neighbor.face] {
                    Some(existing) if existing != required_neighbor_flip => {
                        // A conflict means the component is non-orientable or
                        // nonmanifold under the current topology.  Leave the
                        // previously assigned orientation in place; final
                        // `ExactMesh` validation will reject the replay.
                    }
                    Some(_) => {}
                    None => {
                        flips[neighbor.face] = Some(required_neighbor_flip);
                        queue.push_back(neighbor.face);
                    }
                }
            }
        }
    }

    triangles
        .into_iter()
        .enumerate()
        .map(|(face, triangle)| {
            if flips[face].unwrap_or(false) {
                Triangle([triangle.0[0], triangle.0[2], triangle.0[1]])
            } else {
                triangle
            }
        })
        .collect()
}

#[derive(Clone, Copy)]
struct EdgeUse {
    face: usize,
    forward: bool,
}

#[derive(Clone, Copy)]
struct NeighborConstraint {
    face: usize,
    current_forward: bool,
    neighbor_forward: bool,
}

fn directed_edges(vertices: [usize; 3]) -> [[usize; 2]; 3] {
    [
        [vertices[0], vertices[1]],
        [vertices[1], vertices[2]],
        [vertices[2], vertices[0]],
    ]
}

fn sorted_edge(edge: [usize; 2]) -> [usize; 2] {
    if edge[0] <= edge[1] {
        edge
    } else {
        [edge[1], edge[0]]
    }
}

fn sorted_triangle(mut triangle: [usize; 3]) -> [usize; 3] {
    triangle.sort_unstable();
    triangle
}

fn exact_points_equal(left: &Point3, right: &Point3) -> bool {
    compare_reals(&left.x, &right.x).value() == Some(std::cmp::Ordering::Equal)
        && compare_reals(&left.y, &right.y).value() == Some(std::cmp::Ordering::Equal)
        && compare_reals(&left.z, &right.z).value() == Some(std::cmp::Ordering::Equal)
}

/// Exercise exact cleanup topology-cancellation branches for fuzz builds.
///
/// Normal callers reach cleanup through the staged boolmesh executor.  This
/// probe is gated behind `internal-fuzzing` so adversarial builds can keep the
/// direct `simplify_topology`-style duplicate-pair cancellation compiled
/// without exposing a partial cleanup API.
#[cfg(feature = "internal-fuzzing")]
pub(super) fn internal_fuzz_probe(selector: u8) -> bool {
    fn p(x: i64, y: i64, z: i64) -> ExactPoint3 {
        ExactPoint3::new(ExactReal::from(x), ExactReal::from(y), ExactReal::from(z))
    }

    match selector % 2 {
        0 => {
            let (vertices, triangles) = cleanup_exact_export_vertices(
                vec![p(0, 0, 0), p(2, 0, 0), p(0, 2, 0)],
                &[Triangle([0, 1, 2]), Triangle([0, 2, 1])],
            );
            vertices.is_empty() && triangles.is_empty()
        }
        _ => {
            let (vertices, triangles) = cleanup_exact_export_vertices(
                vec![p(0, 0, 0), p(2, 0, 0), p(0, 2, 0), p(0, 0, 2)],
                &[Triangle([0, 1, 2]), Triangle([0, 2, 3])],
            );
            vertices.len() == 4 && triangles.len() == 2
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::exact::scalar::ExactReal;
    use crate::exact::validation::{ValidationPolicy, validate_triangles_with_policy};

    fn p(x: i64, y: i64, z: i64) -> ExactPoint3 {
        ExactPoint3::new(ExactReal::from(x), ExactReal::from(y), ExactReal::from(z))
    }

    #[test]
    fn cleanup_welds_and_orients_skew_split_topology() {
        let raw_vertices = vec![
            p(0, 0, 1),
            p(0, 0, 1),
            p(1, 0, 0),
            p(0, 1, 0),
            p(-1, 0, 0),
            p(0, -1, 0),
        ];
        let raw_triangles = vec![
            Triangle([1, 4, 2]),
            Triangle([3, 5, 0]),
            Triangle([3, 4, 2]),
            Triangle([4, 3, 5]),
            Triangle([0, 4, 5]),
            Triangle([1, 3, 2]),
        ];

        let (vertices, triangles) = cleanup_exact_export_vertices(raw_vertices, &raw_triangles);
        let points = vertices
            .iter()
            .map(ExactPoint3::to_hyperlimit_point)
            .collect::<Vec<_>>();
        let triangle_indices = triangles
            .iter()
            .map(|triangle| triangle.0)
            .collect::<Vec<_>>();
        let report =
            validate_triangles_with_policy(&points, &triangle_indices, ValidationPolicy::CLOSED);

        assert_eq!(vertices.len(), 5);
        assert_eq!(triangles.len(), 6);
        assert!(
            report.is_valid(),
            "cleanup must remove duplicate directed edges after exact welding: {:?}",
            report.diagnostics
        );
    }

    #[test]
    fn cleanup_drops_triangles_degenerate_after_exact_weld() {
        let raw_vertices = vec![p(0, 0, 0), p(0, 0, 0), p(1, 0, 0)];
        let raw_triangles = vec![Triangle([0, 1, 2])];

        let (vertices, triangles) = cleanup_exact_export_vertices(raw_vertices, &raw_triangles);

        assert!(
            vertices.is_empty(),
            "boolmesh cleanup_unused_verts should drop vertices left unused by exact degenerate-face deletion"
        );
        assert!(triangles.is_empty());
    }

    #[test]
    fn cleanup_compacts_vertices_left_unused_after_triangle_deletion() {
        let raw_vertices = vec![p(9, 9, 9), p(0, 0, 0), p(2, 0, 0), p(0, 2, 0)];
        let raw_triangles = vec![Triangle([1, 2, 3])];

        let (vertices, triangles) = cleanup_exact_export_vertices(raw_vertices, &raw_triangles);

        assert_eq!(vertices, vec![p(0, 0, 0), p(2, 0, 0), p(0, 2, 0)]);
        assert_eq!(triangles, vec![Triangle([0, 1, 2])]);
    }

    #[test]
    fn cleanup_cancels_isolated_opposite_duplicate_triangle_pair() {
        let raw_vertices = vec![p(0, 0, 0), p(2, 0, 0), p(0, 2, 0)];
        let raw_triangles = vec![Triangle([0, 1, 2]), Triangle([0, 2, 1])];

        let (vertices, triangles) = cleanup_exact_export_vertices(raw_vertices, &raw_triangles);

        assert!(
            vertices.is_empty(),
            "cleanup_unused_verts should compact vertices left by exact duplicate-pair cancellation"
        );
        assert!(
            triangles.is_empty(),
            "opposite exact duplicate triangles are zero-thickness interface debris"
        );
    }

    #[test]
    fn cleanup_leaves_nonisolated_duplicate_pair_for_topology_validation() {
        let raw_vertices = vec![p(0, 0, 0), p(2, 0, 0), p(0, 2, 0), p(2, 2, 0)];
        let raw_triangles = vec![
            Triangle([0, 1, 2]),
            Triangle([0, 2, 1]),
            Triangle([1, 3, 2]),
        ];

        let (_vertices, triangles) = cleanup_exact_export_vertices(raw_vertices, &raw_triangles);

        assert!(
            triangles.len() >= 2,
            "non-isolated coincident faces should remain visible to later exact topology checks"
        );
    }

    #[test]
    fn cleanup_closes_exact_coplanar_boundary_cycle() {
        let raw_vertices = vec![p(0, 0, 0), p(2, 0, 0), p(2, 2, 0), p(0, 2, 0), p(1, 1, 3)];
        let raw_triangles = vec![
            Triangle([0, 1, 4]),
            Triangle([1, 2, 4]),
            Triangle([2, 3, 4]),
            Triangle([3, 0, 4]),
        ];

        let (vertices, triangles) = cleanup_exact_export_vertices(raw_vertices, &raw_triangles);
        let points = vertices
            .iter()
            .map(ExactPoint3::to_hyperlimit_point)
            .collect::<Vec<_>>();
        let triangle_indices = triangles
            .iter()
            .map(|triangle| triangle.0)
            .collect::<Vec<_>>();
        let report =
            validate_triangles_with_policy(&points, &triangle_indices, ValidationPolicy::CLOSED);

        assert_eq!(triangles.len(), 6);
        assert!(
            report.is_valid(),
            "cleanup should cap a certified coplanar boolmesh boundary cycle: {:?}",
            report.diagnostics
        );
    }

    #[test]
    fn cleanup_splits_exact_vertices_on_boundary_edges_before_capping() {
        let raw_vertices = vec![
            p(4, 0, 0),
            p(2, -1, 0),
            p(5, -1, 0),
            p(2, -1, -3),
            p(2, 0, 0),
            p(2, 2, 0),
        ];
        let raw_triangles = vec![
            Triangle([4, 2, 1]),
            Triangle([2, 4, 0]),
            Triangle([3, 1, 2]),
            Triangle([2, 5, 3]),
            Triangle([1, 3, 5]),
        ];

        let (vertices, triangles) = cleanup_exact_export_vertices(raw_vertices, &raw_triangles);
        let points = vertices
            .iter()
            .map(ExactPoint3::to_hyperlimit_point)
            .collect::<Vec<_>>();
        let triangle_indices = triangles
            .iter()
            .map(|triangle| triangle.0)
            .collect::<Vec<_>>();
        let report =
            validate_triangles_with_policy(&points, &triangle_indices, ValidationPolicy::CLOSED);

        assert!(
            triangles.len() > raw_triangles.len(),
            "edge refinement should materialize additional exact triangles"
        );
        assert!(
            report.is_valid(),
            "cleanup should split exact on-edge vertices before final capping: {:?}",
            report.diagnostics
        );
    }

    #[test]
    fn cleanup_removes_internal_coplanar_interface_after_edge_refinement() {
        let raw_vertices = vec![
            p(0, 0, 0),
            p(4, 0, 0),
            p(0, 4, 0),
            p(0, 0, 4),
            p(2, -1, 0),
            p(5, -1, 0),
            p(2, -1, -3),
            p(2, 0, 0),
            p(2, 2, 0),
        ];
        let raw_triangles = vec![
            Triangle([1, 0, 2]),
            Triangle([0, 1, 3]),
            Triangle([1, 2, 3]),
            Triangle([2, 0, 3]),
            Triangle([7, 5, 4]),
            Triangle([5, 7, 1]),
            Triangle([6, 4, 5]),
            Triangle([5, 8, 6]),
            Triangle([4, 6, 8]),
        ];

        let (vertices, triangles) = cleanup_exact_export_vertices(raw_vertices, &raw_triangles);
        let points = vertices
            .iter()
            .map(ExactPoint3::to_hyperlimit_point)
            .collect::<Vec<_>>();
        let triangle_indices = triangles
            .iter()
            .map(|triangle| triangle.0)
            .collect::<Vec<_>>();
        let report =
            validate_triangles_with_policy(&points, &triangle_indices, ValidationPolicy::CLOSED);

        assert_eq!(triangles.len(), 14);
        assert!(
            report.is_valid(),
            "cleanup should remove the overfull internal coplanar interface triangle: {:?}",
            report.diagnostics
        );
    }

    #[test]
    fn cleanup_refuses_non_coplanar_boundary_cycle() {
        let raw_vertices = vec![p(0, 0, 0), p(2, 0, 0), p(2, 2, 1), p(0, 2, 0), p(1, 1, 3)];
        let raw_triangles = vec![
            Triangle([0, 1, 4]),
            Triangle([1, 2, 4]),
            Triangle([2, 3, 4]),
            Triangle([3, 0, 4]),
        ];

        let (vertices, triangles) = cleanup_exact_export_vertices(raw_vertices, &raw_triangles);
        let points = vertices
            .iter()
            .map(ExactPoint3::to_hyperlimit_point)
            .collect::<Vec<_>>();
        let triangle_indices = triangles
            .iter()
            .map(|triangle| triangle.0)
            .collect::<Vec<_>>();
        let report =
            validate_triangles_with_policy(&points, &triangle_indices, ValidationPolicy::CLOSED);

        assert_eq!(triangles.len(), 4);
        assert!(
            !report.is_valid(),
            "cleanup must not invent a cap without exact coplanarity"
        );
    }
}
