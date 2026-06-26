//! Exact topology validation for triangular meshes.
//!
//! The validator treats edge incidence as combinatorial data and triangle
//! degeneracy as an exact predicate question. No epsilon is used; exact
//! topology validation stays separated from approximate numeric convenience.

use std::collections::{BTreeMap, BTreeSet};

use hyperlimit::Point3;

use super::super::error::{ExactMeshBlocker, ExactMeshBlockerKind};
use super::facts::{
    EdgeFacts, FaceFacts, FacePlaneFacts, MeshFacts, MeshValidationFacts, OrientedFaceFacts,
    TriangleFacts, VertexFacts, VertexLinkKind,
};
use super::sorted_edge;
use hyperlimit::{
    TriangleDegeneracy, classify_triangle3_degeneracy as classify_triangle_degeneracy,
};
use hyperreal::Real;

/// Validation result for a triangle mesh.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ValidationReport {
    /// Exact facts collected during validation.
    pub(crate) facts: MeshValidationFacts,
    /// Blockers collected during validation.
    pub(crate) blockers: Vec<ExactMeshBlocker>,
}

/// Boundary policy for mesh validation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum BoundaryPolicy {
    /// Every undirected edge must have exactly two incident faces.
    Closed,
    /// Boundary edges are allowed, but nonmanifold edges and vertex links are
    /// still rejected.
    AllowBoundary,
}

/// Validation policy for exact triangle meshes.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct ExactMeshValidationPolicy {
    /// How boundary edges are handled.
    pub(crate) boundary: BoundaryPolicy,
}

impl ExactMeshValidationPolicy {
    /// Closed two-manifold validation.
    pub const CLOSED: Self = Self {
        boundary: BoundaryPolicy::Closed,
    };

    /// Boundary-allowed two-manifold validation.
    pub const ALLOW_BOUNDARY: Self = Self {
        boundary: BoundaryPolicy::AllowBoundary,
    };

    /// Return whether this policy is at least as strict as `requested`.
    pub(crate) const fn satisfies(self, requested: Self) -> bool {
        matches!(
            (self.boundary, requested.boundary),
            (BoundaryPolicy::Closed, BoundaryPolicy::Closed)
                | (BoundaryPolicy::Closed, BoundaryPolicy::AllowBoundary)
                | (BoundaryPolicy::AllowBoundary, BoundaryPolicy::AllowBoundary)
        )
    }
}

impl Default for ExactMeshValidationPolicy {
    fn default() -> Self {
        Self::CLOSED
    }
}

impl ValidationReport {
    /// Return whether the report contains no fatal blockers.
    pub(crate) fn is_valid(&self) -> bool {
        self.blockers.is_empty()
    }
}

pub(crate) fn validate_triangle_rows_with_policy(
    points: &[Point3],
    triangle_count: usize,
    triangles: impl IntoIterator<Item = [usize; 3]>,
    policy: ExactMeshValidationPolicy,
) -> ValidationReport {
    let mut blockers = Vec::new();
    let mut edges = BTreeMap::<[usize; 2], EdgeAccumulator>::new();
    let mut vertex_links = vec![VertexLinkAccumulator::default(); points.len()];
    let mut duplicate_triangles = BTreeSet::<[usize; 3]>::new();
    let mut seen_triangles = BTreeSet::<[usize; 3]>::new();
    let mut faces = Vec::with_capacity(triangle_count);
    let mut degenerate_triangles = 0_usize;

    for (face, tri) in triangles.into_iter().enumerate() {
        let mut has_out_of_bounds_vertex = false;
        for vertex in tri {
            if vertex >= points.len() {
                has_out_of_bounds_vertex = true;
                blockers.push(
                    ExactMeshBlocker::new(
                        ExactMeshBlockerKind::IndexOutOfBounds,
                        format!(
                            "face {face} references vertex {vertex}, but only {} vertices exist",
                            points.len()
                        ),
                    )
                    .with_face(face)
                    .with_vertex(vertex),
                );
            }
        }
        if has_out_of_bounds_vertex {
            continue;
        }

        let mut sorted_tri = tri;
        sorted_tri.sort_unstable();
        if !seen_triangles.insert(sorted_tri) && duplicate_triangles.insert(sorted_tri) {
            blockers.push(
                ExactMeshBlocker::new(
                    ExactMeshBlockerKind::DuplicateTriangle,
                    format!("face {face} duplicates triangle vertex set {sorted_tri:?}"),
                )
                .with_face(face),
            );
        }

        let directed_edges = [[tri[0], tri[1]], [tri[1], tri[2]], [tri[2], tri[0]]];
        for edge in directed_edges {
            let key = sorted_edge(edge);
            edges.entry(key).or_default().push(edge[0] == key[0]);
        }
        vertex_links[tri[0]].push_face(face, [tri[1], tri[2]]);
        vertex_links[tri[1]].push_face(face, [tri[2], tri[0]]);
        vertex_links[tri[2]].push_face(face, [tri[0], tri[1]]);

        let predicate_report =
            classify_triangle_degeneracy(&points[tri[0]], &points[tri[1]], &points[tri[2]]);
        let non_degenerate = matches!(
            predicate_report.degeneracy,
            TriangleDegeneracy::NonDegenerate
        );

        if !non_degenerate {
            degenerate_triangles += 1;
            blockers.push(
                ExactMeshBlocker::new(
                    ExactMeshBlockerKind::DegenerateTriangle,
                    format!("face {face} is not a certified non-degenerate triangle"),
                )
                .with_face(face),
            );
        }

        faces.push(FaceFacts {
            triangle: TriangleFacts {
                face,
                vertices: tri,
                non_degenerate,
                degeneracy_predicates: predicate_report.predicates,
            },
            oriented: OrientedFaceFacts { directed_edges },
            plane: face_plane_facts(points, tri),
        });
    }

    let mut edge_facts = Vec::with_capacity(edges.len());
    let mut boundary_edges = 0_usize;
    let mut non_manifold_edges = 0_usize;
    let mut duplicate_directed_edges = 0_usize;

    for (vertices, accumulator) in edges {
        let directed_uses = [accumulator.forward, accumulator.reverse];
        let incident_faces = accumulator.forward + accumulator.reverse;
        let facts = EdgeFacts {
            vertices,
            incident_faces,
            directed_uses,
        };

        if incident_faces == 1 {
            boundary_edges += 1;
            if policy.boundary == BoundaryPolicy::Closed {
                blockers.push(
                    ExactMeshBlocker::new(
                        ExactMeshBlockerKind::BoundaryEdge,
                        format!("edge {vertices:?} has only one incident face"),
                    )
                    .with_edge(vertices),
                );
            }
        } else if incident_faces > 2 {
            non_manifold_edges += 1;
            blockers.push(
                ExactMeshBlocker::new(
                    ExactMeshBlockerKind::NonManifoldEdge,
                    format!("edge {vertices:?} has {incident_faces} incident faces"),
                )
                .with_edge(vertices),
            );
        }

        if directed_uses[0] > 1 || directed_uses[1] > 1 {
            duplicate_directed_edges += 1;
            blockers.push(
                ExactMeshBlocker::new(
                    ExactMeshBlockerKind::DuplicateDirectedEdge,
                    format!("edge {vertices:?} has duplicate directed uses {directed_uses:?}"),
                )
                .with_edge(vertices),
            );
        }

        edge_facts.push(facts);
    }

    let mut vertex_incident_edge_indices = vec![Vec::<usize>::new(); points.len()];
    for (edge, facts) in edge_facts.iter().enumerate() {
        vertex_incident_edge_indices[facts.vertices[0]].push(edge);
        vertex_incident_edge_indices[facts.vertices[1]].push(edge);
    }

    let mut non_manifold_vertices = 0_usize;
    let vertices = points
        .iter()
        .enumerate()
        .map(|(index, point)| {
            let point_facts = point.structural_facts();
            let link_facts = vertex_links[index].classify();
            if link_facts.kind == VertexLinkKind::NonManifold {
                non_manifold_vertices += 1;
                blockers.push(
                    ExactMeshBlocker::new(
                        ExactMeshBlockerKind::NonManifoldVertexLink,
                        format!("vertex {index} has a nonmanifold link"),
                    )
                    .with_vertex(index),
                );
            }
            VertexFacts {
                index,
                fixed_coordinates_exact_rational: point_facts.exact.is_nonempty_exact_rational(),
                sparse_support: point_facts.has_sparse_support(),
                incident_faces: link_facts.incident_faces,
                incident_edges: link_facts.incident_edges,
                incident_face_indices: vertex_links[index].incident_face_indices.clone(),
                incident_edge_indices: vertex_incident_edge_indices[index].clone(),
                link: link_facts.kind,
            }
        })
        .collect::<Vec<_>>();

    let edge_count = edge_facts.len();
    let face_count = faces.len();
    let closed_manifold = boundary_edges == 0
        && non_manifold_edges == 0
        && non_manifold_vertices == 0
        && duplicate_directed_edges == 0
        && degenerate_triangles == 0;
    let fixed_coordinates_exact_rational = vertices
        .iter()
        .all(|facts| facts.fixed_coordinates_exact_rational);

    ValidationReport {
        facts: MeshValidationFacts {
            mesh: MeshFacts {
                vertex_count: points.len(),
                face_count,
                edge_count,
                euler_characteristic: points.len() as isize - edge_count as isize
                    + face_count as isize,
                boundary_edges,
                non_manifold_edges,
                duplicate_directed_edges,
                degenerate_triangles,
                non_manifold_vertices,
                closed_manifold,
                fixed_coordinates_exact_rational,
            },
            vertices,
            edges: edge_facts,
            faces,
        },
        blockers,
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
struct EdgeAccumulator {
    forward: usize,
    reverse: usize,
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
struct VertexLinkAccumulator {
    incident_faces: usize,
    incident_face_indices: Vec<usize>,
    neighbors: BTreeSet<usize>,
    link_edges: Vec<[usize; 2]>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct VertexLinkFacts {
    incident_faces: usize,
    incident_edges: usize,
    kind: VertexLinkKind,
}

impl VertexLinkAccumulator {
    fn push_face(&mut self, face: usize, mut opposite_edge: [usize; 2]) {
        self.incident_faces += 1;
        self.incident_face_indices.push(face);
        self.neighbors.insert(opposite_edge[0]);
        self.neighbors.insert(opposite_edge[1]);
        opposite_edge.sort_unstable();
        self.link_edges.push(opposite_edge);
    }

    fn classify(&self) -> VertexLinkFacts {
        let incident_edges = self.neighbors.len();
        if self.incident_faces == 0 {
            return VertexLinkFacts {
                incident_faces: 0,
                incident_edges,
                kind: VertexLinkKind::Isolated,
            };
        }

        let mut degree = BTreeMap::<usize, usize>::new();
        let mut adjacency = BTreeMap::<usize, BTreeSet<usize>>::new();
        for &[a, b] in &self.link_edges {
            *degree.entry(a).or_default() += 1;
            *degree.entry(b).or_default() += 1;
            adjacency.entry(a).or_default().insert(b);
            adjacency.entry(b).or_default().insert(a);
        }

        let connected = self.link_is_connected(&adjacency);
        let degree_counts = self
            .neighbors
            .iter()
            .map(|neighbor| degree.get(neighbor).copied().unwrap_or(0))
            .collect::<Vec<_>>();
        let degree_one = degree_counts.iter().filter(|&&degree| degree == 1).count();
        let all_degree_two = degree_counts.iter().all(|&degree| degree == 2);
        let all_degree_one_or_two = degree_counts
            .iter()
            .all(|&degree| degree == 1 || degree == 2);

        let kind = if connected && all_degree_two {
            VertexLinkKind::Circle
        } else if connected && degree_one == 2 && all_degree_one_or_two {
            VertexLinkKind::Disk
        } else {
            VertexLinkKind::NonManifold
        };

        VertexLinkFacts {
            incident_faces: self.incident_faces,
            incident_edges,
            kind,
        }
    }

    fn link_is_connected(&self, adjacency: &BTreeMap<usize, BTreeSet<usize>>) -> bool {
        let Some(&start) = self.neighbors.iter().next() else {
            return false;
        };
        let mut stack = vec![start];
        let mut visited = BTreeSet::new();
        while let Some(node) = stack.pop() {
            if !visited.insert(node) {
                continue;
            }
            if let Some(next) = adjacency.get(&node) {
                stack.extend(next.iter().copied());
            }
        }
        visited.len() == self.neighbors.len()
    }
}

impl EdgeAccumulator {
    fn push(&mut self, forward: bool) {
        if forward {
            self.forward += 1;
        } else {
            self.reverse += 1;
        }
    }
}

fn face_plane_facts(points: &[Point3], tri: [usize; 3]) -> FacePlaneFacts {
    let a = &points[tri[0]];
    let b = &points[tri[1]];
    let c = &points[tri[2]];
    let ux = sub(&b.x, &a.x);
    let uy = sub(&b.y, &a.y);
    let uz = sub(&b.z, &a.z);
    let vx = sub(&c.x, &a.x);
    let vy = sub(&c.y, &a.y);
    let vz = sub(&c.z, &a.z);
    let normal = [
        sub(&mul(&uy, &vz), &mul(&uz, &vy)),
        sub(&mul(&uz, &vx), &mul(&ux, &vz)),
        sub(&mul(&ux, &vy), &mul(&uy, &vx)),
    ];
    let offset = sub(
        &Real::from(0),
        &add(
            &add(&mul(&normal[0], &a.x), &mul(&normal[1], &a.y)),
            &mul(&normal[2], &a.z),
        ),
    );
    FacePlaneFacts { normal, offset }
}

fn add(left: &Real, right: &Real) -> Real {
    left.clone() + right
}

fn sub(left: &Real, right: &Real) -> Real {
    left.clone() - right
}

fn mul(left: &Real, right: &Real) -> Real {
    left.clone() * right
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(x: i64, y: i64, z: i64) -> Point3 {
        Point3::new(Real::from(x), Real::from(y), Real::from(z))
    }

    #[test]
    fn validator_returns_blocker_for_out_of_bounds_triangle_vertex() {
        let points = vec![p(0, 0, 0), p(1, 0, 0), p(0, 1, 0)];
        let report = validate_triangle_rows_with_policy(
            &points,
            1,
            [[0, 1, 3]],
            ExactMeshValidationPolicy::ALLOW_BOUNDARY,
        );

        assert!(!report.is_valid());
        assert_eq!(report.blockers.len(), 1);
        assert_eq!(
            report.blockers[0].kind(),
            ExactMeshBlockerKind::IndexOutOfBounds
        );
        assert_eq!(report.blockers[0].face(), Some(0));
        assert_eq!(report.blockers[0].vertex(), Some(3));
        assert_eq!(report.facts.faces.len(), 0);
    }
}
