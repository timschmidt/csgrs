//! Fact carriers for exact hypermesh objects.
//!
//! These are retained structural facts, not proofs by themselves. Exact
//! predicate reports are stored beside facts when a topological claim depends
//! arithmetic kernels, and certified predicate decisions.

use std::collections::{BTreeMap, BTreeSet};

use super::sorted_edge;
use super::validation::{ExactMeshValidationPolicy, validate_triangle_rows_with_policy};
use hyperlimit::Point3;
use hyperlimit::PredicateUse;
use hyperreal::Real;

/// Facts known for one mesh vertex.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct VertexFacts {
    /// Vertex index in the exact mesh.
    pub(crate) index: usize,
    /// Whether all coordinates are exact rational values in `hyperreal`.
    pub(crate) fixed_coordinates_exact_rational: bool,
    /// Whether the point has known sparse coordinate support.
    pub(crate) sparse_support: bool,
    /// Number of incident faces.
    pub(crate) incident_faces: usize,
    /// Number of incident undirected edges.
    pub(crate) incident_edges: usize,
    /// Incident face indices in retained face order.
    pub(crate) incident_face_indices: Vec<usize>,
    /// Incident edge indices in retained canonical edge-fact order.
    pub(crate) incident_edge_indices: Vec<usize>,
    /// Certified combinatorial shape of the vertex link.
    pub(crate) link: VertexLinkKind,
}

/// Combinatorial shape of one vertex link.
///
/// This is a topology fact, not a geometric predicate. CGAL-style
/// triangulation data structures validate manifoldness by the local shape of
/// stores the classification explicitly so boolean stages can reject a local
/// nonmanifold mutation before it becomes a global mesh.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum VertexLinkKind {
    /// No incident faces.
    Isolated,
    /// Link is a single cycle, as expected for a closed 2-manifold vertex.
    Circle,
    /// Link is a single path, as expected for a boundary 2-manifold vertex.
    Disk,
    /// Link has multiple components, branching, or otherwise invalid degree.
    NonManifold,
}

/// Facts known for one undirected edge.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct EdgeFacts {
    /// Canonical edge endpoints.
    pub(crate) vertices: [usize; 2],
    /// Incident face count.
    pub(crate) incident_faces: usize,
    /// Number of faces using each directed orientation.
    pub(crate) directed_uses: [usize; 2],
}

impl EdgeFacts {
    /// Return whether the edge has exactly two opposing incident faces.
    pub(crate) const fn is_closed_manifold_edge(&self) -> bool {
        self.incident_faces == 2 && self.directed_uses[0] == 1 && self.directed_uses[1] == 1
    }
}

/// Facts known for one triangle.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct TriangleFacts {
    /// Face index.
    pub(crate) face: usize,
    /// Vertex indices.
    pub(crate) vertices: [usize; 3],
    /// Whether predicate validation proved a non-degenerate triangle.
    pub(crate) non_degenerate: bool,
    /// Predicate certificates used while checking degeneracy.
    pub(crate) degeneracy_predicates: Vec<PredicateUse>,
}

/// Facts known for one oriented face.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct OrientedFaceFacts {
    /// Directed triangle edges.
    pub(crate) directed_edges: [[usize; 2]; 3],
}

/// Exact oriented plane equation retained for one face.
///
/// The coefficients satisfy `normal.x * x + normal.y * y + normal.z * z +
/// offset = 0` for every source vertex on the face. Hypermesh deliberately
/// reuse exact object facts instead of re-deriving topology from rounded
/// representatives.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct FacePlaneFacts {
    /// Oriented plane normal from the indexed triangle order.
    pub(crate) normal: [Real; 3],
    /// Exact plane offset.
    pub(crate) offset: Real,
}

/// Facts known for one face.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct FaceFacts {
    /// Triangle facts.
    pub(crate) triangle: TriangleFacts,
    /// Oriented edge facts.
    pub(crate) oriented: OrientedFaceFacts,
    /// Exact oriented plane equation.
    pub(crate) plane: FacePlaneFacts,
}

/// Topology and exactness facts for a whole mesh.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct MeshFacts {
    /// Number of vertices.
    pub(crate) vertex_count: usize,
    /// Number of triangular faces.
    pub(crate) face_count: usize,
    /// Number of undirected edges.
    pub(crate) edge_count: usize,
    /// Euler characteristic `V - E + F`.
    pub(crate) euler_characteristic: isize,
    /// Number of boundary edges.
    pub(crate) boundary_edges: usize,
    /// Number of non-manifold undirected edges.
    pub(crate) non_manifold_edges: usize,
    /// Number of duplicate directed edges.
    pub(crate) duplicate_directed_edges: usize,
    /// Number of degenerate triangles.
    pub(crate) degenerate_triangles: usize,
    /// Number of nonmanifold vertex links.
    pub(crate) non_manifold_vertices: usize,
    /// Whether all accepted triangles and edges form a closed two-manifold.
    pub(crate) closed_manifold: bool,
    /// Whether all coordinates are exact rational values in `hyperreal`.
    pub(crate) fixed_coordinates_exact_rational: bool,
}

/// Expanded validation facts for vertices, edges, and faces.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct MeshValidationFacts {
    /// Whole-mesh summary.
    pub(crate) mesh: MeshFacts,
    /// Per-vertex facts.
    pub(crate) vertices: Vec<VertexFacts>,
    /// Per-edge facts.
    pub(crate) edges: Vec<EdgeFacts>,
    /// Per-face facts.
    pub(crate) faces: Vec<FaceFacts>,
}

/// Error returned when retained mesh validation facts contradict themselves.
///
/// This audits the structural certificates that travel with an [`ExactMesh`](crate::ExactMesh).
/// It does not re-run geometric predicates; instead it checks that the retained
/// topology-affecting predicate decisions remain separately certified.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum MeshFactsValidationError {
    /// A summary count does not match the corresponding retained table length.
    SummaryLengthMismatch {
        /// Summary field name.
        field: &'static str,
        /// Count derived from the retained table.
        expected: usize,
        /// Count stored in the summary.
        actual: usize,
    },
    /// A derived summary count does not match the retained per-item facts.
    SummaryCountMismatch {
        /// Summary field name.
        field: &'static str,
        /// Count derived from retained facts.
        expected: usize,
        /// Count stored in the summary.
        actual: usize,
    },
    /// The Euler characteristic is not `V - E + F`.
    EulerCharacteristicMismatch {
        /// Value derived from retained counts.
        expected: isize,
        /// Value stored in the summary.
        actual: isize,
    },
    /// The closed-manifold summary bit disagrees with retained boundary,
    /// non-manifold, duplicate-orientation, and degeneracy facts.
    ClosedManifoldMismatch {
        /// Value derived from retained facts.
        expected: bool,
        /// Value stored in the summary.
        actual: bool,
    },
    /// The all-coordinates-exact summary bit disagrees with retained vertex
    /// exactness facts.
    FixedCoordinatesMismatch {
        /// Value derived from retained vertex facts.
        expected: bool,
        /// Value stored in the summary.
        actual: bool,
    },
    /// Recomputing facts from the supplied source vertices and triangle rows
    /// did not reproduce this retained fact object.
    SourceReplayMismatch,
    /// A vertex fact is stored at a different slot than its retained index.
    VertexIndexMismatch {
        /// Slot in the retained vertex table.
        expected: usize,
        /// Vertex index stored in the fact.
        actual: usize,
    },
    /// A vertex incident-face count disagrees with retained face rows.
    VertexIncidentFaceMismatch {
        /// Vertex index.
        vertex: usize,
        /// Count derived from retained faces.
        expected: usize,
        /// Count stored in the vertex fact.
        actual: usize,
    },
    /// A vertex incident-edge count disagrees with retained edge rows.
    VertexIncidentEdgeMismatch {
        /// Vertex index.
        vertex: usize,
        /// Count derived from retained edges.
        expected: usize,
        /// Count stored in the vertex fact.
        actual: usize,
    },
    /// A vertex incident-face list disagrees with retained face rows.
    VertexIncidentFaceListMismatch {
        /// Vertex index.
        vertex: usize,
        /// First position where retained and derived lists diverge.
        mismatch_index: usize,
        /// Incident face count derived from retained faces.
        expected_len: usize,
        /// Retained incident face list length.
        actual_len: usize,
    },
    /// A vertex incident-edge list disagrees with retained edge rows.
    VertexIncidentEdgeListMismatch {
        /// Vertex index.
        vertex: usize,
        /// First position where retained and derived lists diverge.
        mismatch_index: usize,
        /// Incident edge count derived from retained edges.
        expected_len: usize,
        /// Retained incident edge list length.
        actual_len: usize,
    },
    /// An edge fact uses an out-of-range vertex.
    EdgeVertexOutOfBounds {
        /// Edge endpoints.
        edge: [usize; 2],
        /// Retained vertex count.
        vertex_count: usize,
    },
    /// An edge fact is not in canonical sorted endpoint order.
    EdgeEndpointOrder {
        /// Edge endpoints.
        edge: [usize; 2],
    },
    /// The same undirected edge appears more than once.
    DuplicateEdgeFact {
        /// Repeated canonical edge.
        edge: [usize; 2],
    },
    /// An edge fact is retained for no derived face edge.
    UnexpectedEdgeFact {
        /// Canonical edge.
        edge: [usize; 2],
    },
    /// A face references an out-of-range vertex.
    FaceVertexOutOfBounds {
        /// Face index.
        face: usize,
        /// Referenced vertex index.
        vertex: usize,
        /// Retained vertex count.
        vertex_count: usize,
    },
    /// A face repeats a vertex.
    FaceRepeatedVertex {
        /// Face index.
        face: usize,
        /// Face vertices.
        vertices: [usize; 3],
    },
    /// A face fact is stored at a different slot than its retained face index.
    FaceIndexMismatch {
        /// Slot in the retained face table.
        expected: usize,
        /// Face index stored in the fact.
        actual: usize,
    },
    /// A face's oriented edge rows do not match its vertex order.
    FaceDirectedEdgesMismatch {
        /// Face index.
        face: usize,
        /// Directed edges derived from `triangle.vertices`.
        expected: [[usize; 2]; 3],
        /// Directed edges stored in the oriented-face facts.
        actual: [[usize; 2]; 3],
    },
    /// An edge fact disagrees with the directed uses derived from face rows.
    EdgeUseMismatch {
        /// Canonical edge.
        edge: [usize; 2],
        /// Derived directed-use counts.
        expected_directed_uses: [usize; 2],
        /// Stored directed-use counts.
        actual_directed_uses: [usize; 2],
        /// Derived incident-face count.
        expected_incident_faces: usize,
        /// Stored incident-face count.
        actual_incident_faces: usize,
    },
    /// A retained face references an edge that has no edge fact.
    MissingEdgeFact {
        /// Canonical edge.
        edge: [usize; 2],
    },
}

impl MeshValidationFacts {
    /// Validate retained topology and exactness facts against each other.
    ///
    /// The check recomputes table lengths, Euler characteristic, edge directed
    /// uses, vertex incidence, degeneracy counts, and closed-manifold summary
    /// facts from the retained rows. It is intentionally combinatorial: exact
    /// predicates remain in [`TriangleFacts::degeneracy_predicates`], while
    /// this method verifies that the structural bookkeeping has not drifted
    /// from those retained predicate outcomes.
    pub(crate) fn validate(&self) -> Result<(), MeshFactsValidationError> {
        let expect_len = |field, expected, actual| {
            if expected == actual {
                Ok(())
            } else {
                Err(MeshFactsValidationError::SummaryLengthMismatch {
                    field,
                    expected,
                    actual,
                })
            }
        };
        let expect_count = |field, expected, actual| {
            if expected == actual {
                Ok(())
            } else {
                Err(MeshFactsValidationError::SummaryCountMismatch {
                    field,
                    expected,
                    actual,
                })
            }
        };

        expect_len("vertex_count", self.vertices.len(), self.mesh.vertex_count)?;
        expect_len("edge_count", self.edges.len(), self.mesh.edge_count)?;
        expect_len("face_count", self.faces.len(), self.mesh.face_count)?;

        let expected_euler = self.mesh.vertex_count as isize - self.mesh.edge_count as isize
            + self.mesh.face_count as isize;
        if self.mesh.euler_characteristic != expected_euler {
            return Err(MeshFactsValidationError::EulerCharacteristicMismatch {
                expected: expected_euler,
                actual: self.mesh.euler_characteristic,
            });
        }

        let mut vertex_incident_face_indices = vec![Vec::<usize>::new(); self.mesh.vertex_count];
        let mut vertex_edges = vec![BTreeSet::<usize>::new(); self.mesh.vertex_count];
        let mut derived_edge_uses = BTreeMap::<[usize; 2], [usize; 2]>::new();
        let mut degenerate_triangles = 0_usize;

        for (face_index, face) in self.faces.iter().enumerate() {
            if face.triangle.face != face_index {
                return Err(MeshFactsValidationError::FaceIndexMismatch {
                    expected: face_index,
                    actual: face.triangle.face,
                });
            }

            let vertices = face.triangle.vertices;
            if vertices[0] == vertices[1]
                || vertices[1] == vertices[2]
                || vertices[2] == vertices[0]
            {
                return Err(MeshFactsValidationError::FaceRepeatedVertex {
                    face: face_index,
                    vertices,
                });
            }
            for vertex in vertices {
                if vertex >= self.mesh.vertex_count {
                    return Err(MeshFactsValidationError::FaceVertexOutOfBounds {
                        face: face_index,
                        vertex,
                        vertex_count: self.mesh.vertex_count,
                    });
                }
                vertex_incident_face_indices[vertex].push(face_index);
            }

            let expected_directed_edges = [
                [vertices[0], vertices[1]],
                [vertices[1], vertices[2]],
                [vertices[2], vertices[0]],
            ];
            if face.oriented.directed_edges != expected_directed_edges {
                return Err(MeshFactsValidationError::FaceDirectedEdgesMismatch {
                    face: face_index,
                    expected: expected_directed_edges,
                    actual: face.oriented.directed_edges,
                });
            }

            if !face.triangle.non_degenerate {
                degenerate_triangles += 1;
            }

            for directed in expected_directed_edges {
                let key = sorted_edge(directed);
                let orientation = usize::from(directed[0] != key[0]);
                derived_edge_uses.entry(key).or_default()[orientation] += 1;
                vertex_edges[directed[0]].insert(directed[1]);
                vertex_edges[directed[1]].insert(directed[0]);
            }
        }

        let mut seen_edges = BTreeSet::new();
        let mut vertex_incident_edge_indices = vec![Vec::<usize>::new(); self.mesh.vertex_count];
        let mut boundary_edges = 0_usize;
        let mut non_manifold_edges = 0_usize;
        let mut duplicate_directed_edges = 0_usize;

        for (edge_index, edge) in self.edges.iter().enumerate() {
            if edge.vertices[0] >= self.mesh.vertex_count
                || edge.vertices[1] >= self.mesh.vertex_count
            {
                return Err(MeshFactsValidationError::EdgeVertexOutOfBounds {
                    edge: edge.vertices,
                    vertex_count: self.mesh.vertex_count,
                });
            }
            if edge.vertices[0] >= edge.vertices[1] {
                return Err(MeshFactsValidationError::EdgeEndpointOrder {
                    edge: edge.vertices,
                });
            }
            if !seen_edges.insert(edge.vertices) {
                return Err(MeshFactsValidationError::DuplicateEdgeFact {
                    edge: edge.vertices,
                });
            }

            let Some(expected_uses) = derived_edge_uses.get(&edge.vertices).copied() else {
                return Err(MeshFactsValidationError::UnexpectedEdgeFact {
                    edge: edge.vertices,
                });
            };
            let expected_incident_faces = expected_uses[0] + expected_uses[1];
            if edge.directed_uses != expected_uses || edge.incident_faces != expected_incident_faces
            {
                return Err(MeshFactsValidationError::EdgeUseMismatch {
                    edge: edge.vertices,
                    expected_directed_uses: expected_uses,
                    actual_directed_uses: edge.directed_uses,
                    expected_incident_faces,
                    actual_incident_faces: edge.incident_faces,
                });
            }

            if edge.incident_faces == 1 {
                boundary_edges += 1;
            } else if edge.incident_faces > 2 {
                non_manifold_edges += 1;
            }
            if edge.directed_uses[0] > 1 || edge.directed_uses[1] > 1 {
                duplicate_directed_edges += 1;
            }
            vertex_incident_edge_indices[edge.vertices[0]].push(edge_index);
            vertex_incident_edge_indices[edge.vertices[1]].push(edge_index);
        }

        for edge in derived_edge_uses.keys().copied() {
            if !seen_edges.contains(&edge) {
                return Err(MeshFactsValidationError::MissingEdgeFact { edge });
            }
        }

        let mut non_manifold_vertices = 0_usize;
        let fixed_coordinates_exact_rational = self
            .vertices
            .iter()
            .all(|vertex| vertex.fixed_coordinates_exact_rational);
        let first_mismatch_index = |expected: &[usize], actual: &[usize]| {
            expected
                .iter()
                .zip(actual)
                .position(|(expected, actual)| expected != actual)
                .unwrap_or_else(|| expected.len().min(actual.len()))
        };
        for (index, vertex) in self.vertices.iter().enumerate() {
            if vertex.index != index {
                return Err(MeshFactsValidationError::VertexIndexMismatch {
                    expected: index,
                    actual: vertex.index,
                });
            }
            let incident_faces = vertex_incident_face_indices[index].len();
            if vertex.incident_faces != incident_faces {
                return Err(MeshFactsValidationError::VertexIncidentFaceMismatch {
                    vertex: index,
                    expected: incident_faces,
                    actual: vertex.incident_faces,
                });
            }
            if vertex.incident_face_indices != vertex_incident_face_indices[index] {
                return Err(MeshFactsValidationError::VertexIncidentFaceListMismatch {
                    vertex: index,
                    mismatch_index: first_mismatch_index(
                        &vertex_incident_face_indices[index],
                        &vertex.incident_face_indices,
                    ),
                    expected_len: vertex_incident_face_indices[index].len(),
                    actual_len: vertex.incident_face_indices.len(),
                });
            }
            let incident_edges = vertex_edges[index].len();
            if vertex.incident_edges != incident_edges {
                return Err(MeshFactsValidationError::VertexIncidentEdgeMismatch {
                    vertex: index,
                    expected: incident_edges,
                    actual: vertex.incident_edges,
                });
            }
            if vertex.incident_edge_indices != vertex_incident_edge_indices[index] {
                return Err(MeshFactsValidationError::VertexIncidentEdgeListMismatch {
                    vertex: index,
                    mismatch_index: first_mismatch_index(
                        &vertex_incident_edge_indices[index],
                        &vertex.incident_edge_indices,
                    ),
                    expected_len: vertex_incident_edge_indices[index].len(),
                    actual_len: vertex.incident_edge_indices.len(),
                });
            }
            if vertex.link == VertexLinkKind::NonManifold {
                non_manifold_vertices += 1;
            }
        }

        expect_count("boundary_edges", boundary_edges, self.mesh.boundary_edges)?;
        expect_count(
            "non_manifold_edges",
            non_manifold_edges,
            self.mesh.non_manifold_edges,
        )?;
        expect_count(
            "duplicate_directed_edges",
            duplicate_directed_edges,
            self.mesh.duplicate_directed_edges,
        )?;
        expect_count(
            "degenerate_triangles",
            degenerate_triangles,
            self.mesh.degenerate_triangles,
        )?;
        expect_count(
            "non_manifold_vertices",
            non_manifold_vertices,
            self.mesh.non_manifold_vertices,
        )?;

        let closed_manifold = boundary_edges == 0
            && non_manifold_edges == 0
            && non_manifold_vertices == 0
            && duplicate_directed_edges == 0
            && degenerate_triangles == 0;
        if self.mesh.closed_manifold != closed_manifold {
            return Err(MeshFactsValidationError::ClosedManifoldMismatch {
                expected: closed_manifold,
                actual: self.mesh.closed_manifold,
            });
        }
        if self.mesh.fixed_coordinates_exact_rational != fixed_coordinates_exact_rational {
            return Err(MeshFactsValidationError::FixedCoordinatesMismatch {
                expected: fixed_coordinates_exact_rational,
                actual: self.mesh.fixed_coordinates_exact_rational,
            });
        }

        Ok(())
    }

    pub(crate) fn validate_against_triangle_rows_with_policy(
        &self,
        points: &[Point3],
        triangle_count: usize,
        triangles: impl IntoIterator<Item = [usize; 3]>,
        policy: ExactMeshValidationPolicy,
    ) -> Result<(), MeshFactsValidationError> {
        self.validate()?;
        let replay = validate_triangle_rows_with_policy(points, triangle_count, triangles, policy);
        if self == &replay.facts {
            Ok(())
        } else {
            Err(MeshFactsValidationError::SourceReplayMismatch)
        }
    }
}
