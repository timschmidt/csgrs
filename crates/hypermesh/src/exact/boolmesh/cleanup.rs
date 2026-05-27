//! Exact final cleanup for boolmesh `boolean45` export candidates.
//!
//! Legacy boolmesh runs `simplify_topology` and `cleanup_unused_verts` between
//! `triangulate` and `Manifold::new_impl`.  This module ports the part needed at
//! the exact object boundary: coincident output slots are merged only by exact
//! coordinate equality, degenerate triangles created by that merge are dropped,
//! and the remaining triangle soup is oriented as a halfedge surface.  Yap,
//! "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
//! (1997), is the governing constraint here: cleanup may change topology only
//! when exact predicates decide the equality or incidence being used.

use std::collections::{BTreeMap, VecDeque};

use hyperlimit::{Point3, compare_reals};

use crate::exact::mesh::{ExactPoint3, Triangle};

/// Collapse exactly equal export coordinates and orient the resulting surface.
///
/// The `boolean45` stages intentionally preserve boolmesh output vertex slots:
/// multiple signed `Boolean03` rows may name the same exact point until the
/// final mesh boundary.  This function performs the boolmesh cleanup handoff
/// without introducing tolerances:
///
/// - vertices are welded only when all three coordinates compare exactly equal;
/// - triangles that become index-degenerate after welding are removed;
/// - each connected triangle component is flipped, when necessary, so every
///   two-face edge is traversed in opposite directions by its incident faces.
///
/// The orientation pass is the halfedge consistency condition used by the
/// boolmesh boolean kernel after topology simplification; see Komikado's
/// boolmesh-derived `simplify_topology`/`Manifold::new_impl` handoff in this
/// crate.  The exact port keeps that algorithmic boundary but replaces the
/// primitive-float equality shortcut with `hyperlimit::compare_reals`.
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

    (unique_vertices, orient_triangle_components(triangles))
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

fn exact_points_equal(left: &Point3, right: &Point3) -> bool {
    compare_reals(&left.x, &right.x).value() == Some(std::cmp::Ordering::Equal)
        && compare_reals(&left.y, &right.y).value() == Some(std::cmp::Ordering::Equal)
        && compare_reals(&left.z, &right.z).value() == Some(std::cmp::Ordering::Equal)
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

        assert_eq!(vertices.len(), 2);
        assert!(triangles.is_empty());
    }
}
