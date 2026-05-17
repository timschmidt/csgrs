//! Exact topology validation for triangular meshes.
//!
//! The validator treats edge incidence as combinatorial data and triangle
//! degeneracy as an exact predicate question. No epsilon is used. This follows
//! Yap's exact-geometric-computation requirement that topology decisions be
//! separated from approximate numeric convenience.

use std::collections::{BTreeMap, BTreeSet};

use hyperlimit::Point3;

use super::error::{DiagnosticKind, MeshDiagnostic, Severity};
use super::facts::{
    EdgeFacts, FaceFacts, MeshFacts, MeshValidationFacts, OrientedFaceFacts, TriangleFacts,
    VertexFacts, VertexLinkKind,
};
use super::predicates::{TriangleDegeneracy, classify_triangle_degeneracy};

/// Validation result for a triangle mesh.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ValidationReport {
    /// Exact facts collected during validation.
    pub facts: MeshValidationFacts,
    /// Diagnostics collected during validation.
    pub diagnostics: Vec<MeshDiagnostic>,
}

/// Boundary policy for mesh validation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BoundaryPolicy {
    /// Every undirected edge must have exactly two incident faces.
    Closed,
    /// Boundary edges are allowed, but nonmanifold edges and vertex links are
    /// still rejected.
    AllowBoundary,
}

/// Validation policy for exact triangle meshes.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ValidationPolicy {
    /// How boundary edges are handled.
    pub boundary: BoundaryPolicy,
}

impl ValidationPolicy {
    /// Closed two-manifold validation.
    pub const CLOSED: Self = Self {
        boundary: BoundaryPolicy::Closed,
    };

    /// Boundary-allowed two-manifold validation.
    pub const ALLOW_BOUNDARY: Self = Self {
        boundary: BoundaryPolicy::AllowBoundary,
    };
}

impl Default for ValidationPolicy {
    fn default() -> Self {
        Self::CLOSED
    }
}

impl ValidationReport {
    /// Return whether the report contains no error diagnostics.
    pub fn is_valid(&self) -> bool {
        self.diagnostics
            .iter()
            .all(|diagnostic| diagnostic.severity != Severity::Error)
    }
}

/// Validate indexed triangles against exact points.
pub fn validate_triangles(points: &[Point3], triangles: &[[usize; 3]]) -> ValidationReport {
    validate_triangles_with_policy(points, triangles, ValidationPolicy::default())
}

/// Validate indexed triangles against exact points with an explicit policy.
///
/// Closed validation treats boundary edges as errors. Boundary-allowed
/// validation still records boundary facts but does not promote them to fatal
/// diagnostics. The policy object keeps that topological contract explicit,
/// following Yap's exact-geometric-computation principle that uncertainty and
/// approximation policies must be visible at API boundaries.
pub fn validate_triangles_with_policy(
    points: &[Point3],
    triangles: &[[usize; 3]],
    policy: ValidationPolicy,
) -> ValidationReport {
    let mut diagnostics = Vec::new();
    let mut edges = BTreeMap::<[usize; 2], EdgeAccumulator>::new();
    let mut vertex_links = vec![VertexLinkAccumulator::default(); points.len()];
    let mut duplicate_triangles = BTreeSet::<[usize; 3]>::new();
    let mut seen_triangles = BTreeSet::<[usize; 3]>::new();
    let mut faces = Vec::with_capacity(triangles.len());
    let mut degenerate_triangles = 0_usize;

    for (face, &tri) in triangles.iter().enumerate() {
        let mut sorted_tri = tri;
        sorted_tri.sort_unstable();
        if !seen_triangles.insert(sorted_tri) && duplicate_triangles.insert(sorted_tri) {
            diagnostics.push(
                MeshDiagnostic::new(
                    Severity::Error,
                    DiagnosticKind::DuplicateTriangle,
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
        vertex_links[tri[0]].push_face([tri[1], tri[2]]);
        vertex_links[tri[1]].push_face([tri[2], tri[0]]);
        vertex_links[tri[2]].push_face([tri[0], tri[1]]);

        let predicate_report =
            classify_triangle_degeneracy(&points[tri[0]], &points[tri[1]], &points[tri[2]]);
        let non_degenerate = matches!(
            predicate_report.degeneracy,
            TriangleDegeneracy::NonDegenerate
        );

        if !non_degenerate {
            degenerate_triangles += 1;
            diagnostics.push(
                MeshDiagnostic::new(
                    Severity::Error,
                    DiagnosticKind::DegenerateTriangle,
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
                diagnostics.push(
                    MeshDiagnostic::new(
                        Severity::Error,
                        DiagnosticKind::BoundaryEdge,
                        format!("edge {vertices:?} has only one incident face"),
                    )
                    .with_edge(vertices),
                );
            }
        } else if incident_faces > 2 {
            non_manifold_edges += 1;
            diagnostics.push(
                MeshDiagnostic::new(
                    Severity::Error,
                    DiagnosticKind::NonManifoldEdge,
                    format!("edge {vertices:?} has {incident_faces} incident faces"),
                )
                .with_edge(vertices),
            );
        }

        if directed_uses[0] > 1 || directed_uses[1] > 1 {
            duplicate_directed_edges += 1;
            diagnostics.push(
                MeshDiagnostic::new(
                    Severity::Error,
                    DiagnosticKind::DuplicateDirectedEdge,
                    format!("edge {vertices:?} has duplicate directed uses {directed_uses:?}"),
                )
                .with_edge(vertices),
            );
        }

        edge_facts.push(facts);
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
                diagnostics.push(
                    MeshDiagnostic::new(
                        Severity::Error,
                        DiagnosticKind::NonManifoldVertexLink,
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
                link: link_facts.kind,
            }
        })
        .collect::<Vec<_>>();

    let edge_count = edge_facts.len();
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
                face_count: triangles.len(),
                edge_count,
                euler_characteristic: points.len() as isize - edge_count as isize
                    + triangles.len() as isize,
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
        diagnostics,
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
    fn push_face(&mut self, mut opposite_edge: [usize; 2]) {
        self.incident_faces += 1;
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

fn sorted_edge(edge: [usize; 2]) -> [usize; 2] {
    if edge[0] <= edge[1] {
        edge
    } else {
        [edge[1], edge[0]]
    }
}
