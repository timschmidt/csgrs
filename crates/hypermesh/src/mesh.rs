//! Exact mesh construction and storage.
//!
//! `Mesh` stores coordinates as `hyperlimit::Point3` over
//! `hyperreal::Real`. Primitive-float construction is a named lossy adapter
//! and validates every coordinate before import.

use self::bounds::{BoundsValidationError, MeshBounds};
use self::error::{MeshBlocker, MeshBlockerKind, MeshError};
use self::facts::{MeshFactsValidationError, MeshValidationFacts};
use self::validation::{
    MeshValidationPolicy, ValidationReport, validate_triangle_rows_with_policy,
};
use self::view::MeshView;
use hyperlimit::{
    ConstructionProvenance, ConstructionProvenanceValidationError, Point3, PredicateUse,
    SourceProvenance, compare_reals,
};
use hyperreal::Real;
use std::{
    cmp::Ordering,
    collections::{BTreeMap, BTreeSet},
};

pub(crate) mod arrangement3d;
pub(crate) mod boolean;
pub(crate) mod bounds;
pub(crate) mod error;
pub(crate) mod facts;
pub(crate) mod graph;
pub(crate) mod prepared;
pub(crate) mod validation;
pub(crate) mod view;

/// Triangle index triplet.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct Triangle(pub [usize; 3]);

pub(crate) fn triangle_edges(triangle: [usize; 3]) -> [[usize; 2]; 3] {
    [
        [triangle[0], triangle[1]],
        [triangle[1], triangle[2]],
        [triangle[2], triangle[0]],
    ]
}

pub(crate) fn sorted_edge(edge: [usize; 2]) -> [usize; 2] {
    if edge[0] < edge[1] {
        edge
    } else {
        [edge[1], edge[0]]
    }
}

pub(crate) fn paired_triangle_orientation_flips(
    triangles: impl IntoIterator<Item = [usize; 3]>,
    triangle_count: usize,
) -> Option<Vec<bool>> {
    let mut edge_uses = BTreeMap::<[usize; 2], Vec<(usize, bool)>>::new();
    for (triangle, vertices) in triangles.into_iter().enumerate() {
        for edge in triangle_edges(vertices) {
            let key = sorted_edge(edge);
            edge_uses
                .entry(key)
                .or_default()
                .push((triangle, edge == key));
        }
    }

    let mut adjacency = vec![Vec::<(usize, bool)>::new(); triangle_count];
    for uses in edge_uses.values() {
        let [
            (left_triangle, left_forward),
            (right_triangle, right_forward),
        ] = uses.as_slice()
        else {
            continue;
        };
        let same_direction = left_forward == right_forward;
        adjacency[*left_triangle].push((*right_triangle, same_direction));
        adjacency[*right_triangle].push((*left_triangle, same_direction));
    }

    let mut flips = vec![None; triangle_count];
    for start in 0..triangle_count {
        if flips[start].is_some() {
            continue;
        }
        flips[start] = Some(false);
        let mut stack = vec![start];
        while let Some(triangle) = stack.pop() {
            let current_flip = flips[triangle]?;
            for &(neighbor, flip_relative_to_current) in &adjacency[triangle] {
                let required = current_flip ^ flip_relative_to_current;
                match flips[neighbor] {
                    Some(existing) if existing != required => return None,
                    Some(_) => {}
                    None => {
                        flips[neighbor] = Some(required);
                        stack.push(neighbor);
                    }
                }
            }
        }
    }

    flips.into_iter().collect()
}

pub(crate) fn orient_paired_triangle_edges(triangles: &mut [Triangle]) -> Option<usize> {
    let flips = paired_triangle_orientation_flips(
        triangles.iter().map(|triangle| triangle.0),
        triangles.len(),
    )?;

    let mut flipped = 0;
    for (triangle, flip) in triangles.iter_mut().zip(flips) {
        if flip {
            triangle.0.swap(1, 2);
            flipped += 1;
        }
    }
    Some(flipped)
}

pub(crate) fn remove_duplicate_triangle_vertex_sets(triangles: &mut Vec<Triangle>) -> usize {
    let before = triangles.len();
    let mut seen = BTreeSet::new();
    triangles.retain(|triangle| {
        let mut key = triangle.0;
        key.sort_unstable();
        seen.insert(key)
    });
    before - triangles.len()
}

pub(crate) fn point3_exact_equal(left: &Point3, right: &Point3) -> Option<bool> {
    Some(
        compare_reals(&left.x, &right.x).value()? == Ordering::Equal
            && compare_reals(&left.y, &right.y).value()? == Ordering::Equal
            && compare_reals(&left.z, &right.z).value()? == Ordering::Equal,
    )
}

pub(crate) fn exact_points_are_collinear(a: &Point3, b: &Point3, c: &Point3) -> Option<bool> {
    let abx = &b.x - &a.x;
    let aby = &b.y - &a.y;
    let abz = &b.z - &a.z;
    let acx = &c.x - &a.x;
    let acy = &c.y - &a.y;
    let acz = &c.z - &a.z;
    let cross_x = &(&aby * &acz) - &(&abz * &acy);
    let cross_y = &(&abz * &acx) - &(&abx * &acz);
    let cross_z = &(&abx * &acy) - &(&aby * &acx);
    Some(
        compare_reals(&cross_x, &Real::zero()).value()? == Ordering::Equal
            && compare_reals(&cross_y, &Real::zero()).value()? == Ordering::Equal
            && compare_reals(&cross_z, &Real::zero()).value()? == Ordering::Equal,
    )
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactAffineTransform3 {
    linear: [[Real; 3]; 3],
    translation: [Real; 3],
}

impl ExactAffineTransform3 {
    pub(crate) fn from_homogeneous_rows(matrix: [[Real; 4]; 4]) -> Result<Self, MeshError> {
        let [
            [m00, m01, m02, tx],
            [m10, m11, m12, ty],
            [m20, m21, m22, tz],
            affine_row,
        ] = matrix;
        let affine_row_is_exact = real_equals(&affine_row[0], &Real::zero())?
            && real_equals(&affine_row[1], &Real::zero())?
            && real_equals(&affine_row[2], &Real::zero())?
            && real_equals(&affine_row[3], &Real::one())?;
        if !affine_row_is_exact {
            return Err(MeshError::one(MeshBlocker::new(
                MeshBlockerKind::UnsupportedExactOperation,
                "homogeneous mesh transform must be affine with final row [0, 0, 0, 1]",
            )));
        }
        Ok(Self {
            linear: [[m00, m01, m02], [m10, m11, m12], [m20, m21, m22]],
            translation: [tx, ty, tz],
        })
    }

    pub(crate) fn transform_point(&self, point: &Point3) -> Point3 {
        Point3::new(
            transformed_coordinate(&self.linear[0], point, &self.translation[0]),
            transformed_coordinate(&self.linear[1], point, &self.translation[1]),
            transformed_coordinate(&self.linear[2], point, &self.translation[2]),
        )
    }

    pub(crate) fn orientation(&self) -> Result<Ordering, MeshError> {
        compare_reals(&det3_rows(&self.linear), &Real::zero())
            .value()
            .ok_or_else(|| {
                MeshError::one(MeshBlocker::new(
                    MeshBlockerKind::UndecidablePredicate,
                    "exact transform determinant sign could not be certified",
                ))
            })
    }
}

/// Exact triangular mesh with retained validation facts.
#[derive(Clone, Debug, PartialEq)]
pub struct Mesh {
    vertices: Vec<Point3>,
    triangles: Vec<Triangle>,
    bounds: MeshBounds,
    facts: MeshValidationFacts,
    validation_policy: MeshValidationPolicy,
    provenance: ConstructionProvenance,
}

fn point_from_f64_lossy(values: [f64; 3], first_coordinate: usize) -> Result<Point3, MeshError> {
    let x = import_lossy_f64_real(values[0], first_coordinate).map_err(MeshError::one)?;
    let y = import_lossy_f64_real(values[1], first_coordinate + 1).map_err(MeshError::one)?;
    let z = import_lossy_f64_real(values[2], first_coordinate + 2).map_err(MeshError::one)?;
    Ok(Point3::new(x, y, z))
}

fn import_lossy_f64_real(value: f64, coordinate_index: usize) -> Result<Real, MeshBlocker> {
    if !value.is_finite() {
        return Err(MeshBlocker::new(
            MeshBlockerKind::NonFiniteCoordinate,
            format!("coordinate {coordinate_index} is not finite"),
        )
        .with_coordinate(coordinate_index));
    }

    Real::try_from(value).map_err(|problem| {
        MeshBlocker::new(
            MeshBlockerKind::CoordinateImportFailed,
            format!("coordinate {coordinate_index} could not be imported: {problem}"),
        )
        .with_coordinate(coordinate_index)
    })
}

/// Error returned when an [`Mesh`] retained-state audit fails.
///
/// This is a whole-object consistency check over topology facts, exact bounds,
/// object facts and proof-producing predicate provenance as part of the
/// certified mesh state rather than as incidental cache entries.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum MeshValidationError {
    /// The retained vertex count disagrees with the vertex buffer length.
    VertexCountMismatch {
        /// Vertex buffer length.
        expected: usize,
        /// Retained mesh-fact count.
        actual: usize,
    },
    /// The retained face count disagrees with the triangle buffer length.
    FaceCountMismatch {
        /// Triangle buffer length.
        expected: usize,
        /// Retained mesh-fact count.
        actual: usize,
    },
    /// A retained bounds axis minimum is certified greater than its maximum.
    RetainedBoundsInvertedAxis,
    /// A retained bounds axis minimum/maximum relation could not be certified.
    RetainedBoundsUnknownAxisOrder,
    /// Retained mesh-level bounds are missing for a nonempty vertex set.
    RetainedBoundsMissingMeshBounds,
    /// Retained mesh-level bounds exist for an empty vertex set.
    RetainedBoundsUnexpectedMeshBounds,
    /// The retained face-bound vector length does not match the face count.
    RetainedBoundsFaceCountMismatch,
    /// The retained edge-bound vector length does not match the edge count.
    RetainedBoundsEdgeCountMismatch,
    /// Recomputing bounds from the source vertices and triangles did not
    /// reproduce the retained bounds object.
    RetainedBoundsSourceReplayMismatch,
    /// A retained fact summary count does not match the corresponding table length.
    RetainedFactsSummaryLengthMismatch {
        /// Summary field name.
        field: &'static str,
        /// Count derived from the retained table.
        expected: usize,
        /// Count stored in the summary.
        actual: usize,
    },
    /// A derived retained fact summary count does not match per-item facts.
    RetainedFactsSummaryCountMismatch {
        /// Summary field name.
        field: &'static str,
        /// Count derived from retained facts.
        expected: usize,
        /// Count stored in the summary.
        actual: usize,
    },
    /// The retained Euler characteristic is not `V - E + F`.
    RetainedFactsEulerCharacteristicMismatch {
        /// Value derived from retained counts.
        expected: isize,
        /// Value stored in the summary.
        actual: isize,
    },
    /// The retained closed-manifold summary bit disagrees with retained facts.
    RetainedFactsClosedManifoldMismatch {
        /// Value derived from retained facts.
        expected: bool,
        /// Value stored in the summary.
        actual: bool,
    },
    /// The retained all-coordinates-exact bit disagrees with vertex facts.
    RetainedFactsFixedCoordinatesMismatch {
        /// Value derived from retained vertex facts.
        expected: bool,
        /// Value stored in the summary.
        actual: bool,
    },
    /// Recomputing facts from the source vertices and triangle rows did not
    /// reproduce the retained facts.
    RetainedFactsSourceReplayMismatch,
    /// A retained vertex fact is stored at a different slot than its index.
    RetainedFactsVertexIndexMismatch {
        /// Slot in the retained vertex table.
        expected: usize,
        /// Vertex index stored in the fact.
        actual: usize,
    },
    /// A retained vertex incident-face count disagrees with face rows.
    RetainedFactsVertexIncidentFaceMismatch {
        /// Vertex index.
        vertex: usize,
        /// Count derived from retained faces.
        expected: usize,
        /// Count stored in the vertex fact.
        actual: usize,
    },
    /// A retained vertex incident-edge count disagrees with edge rows.
    RetainedFactsVertexIncidentEdgeMismatch {
        /// Vertex index.
        vertex: usize,
        /// Count derived from retained edges.
        expected: usize,
        /// Count stored in the vertex fact.
        actual: usize,
    },
    /// A retained vertex incident-face list disagrees with face rows.
    RetainedFactsVertexIncidentFaceListMismatch {
        /// Vertex index.
        vertex: usize,
        /// First position where retained and derived lists diverge.
        mismatch_index: usize,
        /// Incident face count derived from retained faces.
        expected_len: usize,
        /// Retained incident face list length.
        actual_len: usize,
    },
    /// A retained vertex incident-edge list disagrees with edge rows.
    RetainedFactsVertexIncidentEdgeListMismatch {
        /// Vertex index.
        vertex: usize,
        /// First position where retained and derived lists diverge.
        mismatch_index: usize,
        /// Incident edge count derived from retained edges.
        expected_len: usize,
        /// Retained incident edge list length.
        actual_len: usize,
    },
    /// A retained edge fact uses an out-of-range vertex.
    RetainedFactsEdgeVertexOutOfBounds {
        /// Edge endpoints.
        edge: [usize; 2],
        /// Retained vertex count.
        vertex_count: usize,
    },
    /// A retained edge fact is not in canonical endpoint order.
    RetainedFactsEdgeEndpointOrder {
        /// Edge endpoints.
        edge: [usize; 2],
    },
    /// The same retained undirected edge appears more than once.
    RetainedFactsDuplicateEdgeFact {
        /// Repeated canonical edge.
        edge: [usize; 2],
    },
    /// A retained edge fact is not referenced by any retained face row.
    RetainedFactsUnexpectedEdgeFact {
        /// Canonical edge.
        edge: [usize; 2],
    },
    /// A retained face references an out-of-range vertex.
    RetainedFactsFaceVertexOutOfBounds {
        /// Face index.
        face: usize,
        /// Referenced vertex index.
        vertex: usize,
        /// Retained vertex count.
        vertex_count: usize,
    },
    /// A retained face repeats a vertex.
    RetainedFactsFaceRepeatedVertex {
        /// Face index.
        face: usize,
        /// Face vertices.
        vertices: [usize; 3],
    },
    /// A retained face fact is stored at a different slot than its face index.
    RetainedFactsFaceIndexMismatch {
        /// Slot in the retained face table.
        expected: usize,
        /// Face index stored in the fact.
        actual: usize,
    },
    /// A retained face's oriented edge rows do not match its vertex order.
    RetainedFactsFaceDirectedEdgesMismatch {
        /// Face index.
        face: usize,
        /// Directed edges derived from `triangle.vertices`.
        expected: [[usize; 2]; 3],
        /// Directed edges stored in the oriented-face facts.
        actual: [[usize; 2]; 3],
    },
    /// A retained edge fact disagrees with directed uses derived from face rows.
    RetainedFactsEdgeUseMismatch {
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
    /// A retained face references an edge that has no retained edge fact.
    RetainedFactsMissingEdgeFact {
        /// Canonical edge.
        edge: [usize; 2],
    },
    /// Retained provenance failed its own validation.
    Provenance(ConstructionProvenanceValidationError),
    /// Predicate provenance no longer mirrors the retained face predicate
    /// certificates.
    PredicateRetentionMismatch,
}

impl Mesh {
    /// Construct an exact mesh from exact vertices and triangle indices.
    pub fn new(
        vertices: Vec<Point3>,
        triangles: Vec<[usize; 3]>,
        source: SourceProvenance,
    ) -> Result<Self, MeshError> {
        Self::new_with_policy_and_version(
            vertices,
            triangles.into_iter().map(Triangle).collect(),
            source,
            MeshValidationPolicy::CLOSED,
            1,
        )
    }

    pub(crate) fn new_with_policy_and_version(
        vertices: Vec<Point3>,
        triangles: Vec<Triangle>,
        source: SourceProvenance,
        policy: MeshValidationPolicy,
        construction_version: u64,
    ) -> Result<Self, MeshError> {
        let report = validate_triangle_rows_with_policy(
            &vertices,
            triangles.len(),
            triangles.iter().map(|tri| tri.0),
            policy,
        );
        if !report.blockers.is_empty() {
            return Err(MeshError::new(report.blockers));
        }
        let bounds = MeshBounds::from_triangle_rows(
            &vertices,
            triangles.len(),
            triangles.iter().map(|tri| tri.0),
        )
        .map_err(retained_bounds_validation_error)
        .map_err(|error| {
            retained_validation_mesh_error("exact mesh bounds construction failed", error)
        })?;

        let mut provenance =
            ConstructionProvenance::with_version(source, construction_version.max(1));
        retain_predicates(&mut provenance, &report);

        Ok(Self {
            vertices,
            triangles,
            bounds,
            facts: report.facts,
            validation_policy: policy,
            provenance,
        })
    }

    /// Import an exact mesh from flat primitive-float coordinates.
    ///
    /// The `f64` values are checked for finiteness and imported as exact dyadic
    /// `Real` values with lossy source provenance. They are not used later as
    /// tolerance-bearing floats.
    pub fn from_lossy_f64_triangles(pos: &[f64], idx: &[usize]) -> Result<Self, MeshError> {
        Self::from_lossy_f64_triangles_with_policy(pos, idx, MeshValidationPolicy::CLOSED)
    }

    /// Construct an exact mesh from flat hyperreal coordinates.
    pub fn from_real_triangles(pos: &[Real], idx: &[usize]) -> Result<Self, MeshError> {
        Self::from_real_triangles_with_policy(pos, idx, MeshValidationPolicy::CLOSED)
    }

    /// Construct an exact boundary-allowed surface mesh from flat hyperreal coordinates.
    ///
    /// Use this adapter for query, export, and connectivity surfaces that are
    /// not closed solids. Named Boolean entry points still require the default
    /// closed-solid constructor unless an algorithm explicitly accepts surface
    /// artifacts.
    pub fn from_real_surface_triangles(pos: &[Real], idx: &[usize]) -> Result<Self, MeshError> {
        Self::from_real_triangles_with_policy(pos, idx, MeshValidationPolicy::ALLOW_BOUNDARY)
    }

    /// Construct an exact mesh from flat hyperreal coordinates with an explicit
    /// validation policy.
    pub(crate) fn from_real_triangles_with_policy(
        pos: &[Real],
        idx: &[usize],
        policy: MeshValidationPolicy,
    ) -> Result<Self, MeshError> {
        validate_flat_mesh_buffers(pos.len(), idx.len())?;

        let vertices = pos
            .chunks_exact(3)
            .map(|coords| Point3::new(coords[0].clone(), coords[1].clone(), coords[2].clone()))
            .collect::<Vec<_>>();

        Self::new_with_policy_and_version(
            vertices,
            flat_triangles(idx),
            SourceProvenance::exact("flat hyperreal triangle mesh"),
            policy,
            1,
        )
    }

    /// Import an exact mesh from flat primitive-float coordinates with an
    /// explicit validation policy and lossy source provenance.
    pub(crate) fn from_lossy_f64_triangles_with_policy(
        pos: &[f64],
        idx: &[usize],
        policy: MeshValidationPolicy,
    ) -> Result<Self, MeshError> {
        validate_flat_mesh_buffers(pos.len(), idx.len())?;

        let mut vertices = Vec::with_capacity(pos.len() / 3);
        for (vertex, coords) in pos.chunks_exact(3).enumerate() {
            let point = point_from_f64_lossy([coords[0], coords[1], coords[2]], vertex * 3)?;
            vertices.push(point);
        }

        Self::new_with_policy_and_version(
            vertices,
            flat_triangles(idx),
            SourceProvenance::lossy_f64("flat f64 triangle mesh"),
            policy,
            1,
        )
    }

    /// Construct an exact mesh from flat integer coordinates.
    ///
    /// Integer grid input is lifted directly into `hyperreal::Real` without a
    /// primitive-float edge, keeping exact predicates and determinant schedules
    /// on structural input coordinates.
    pub fn from_i64_triangles(pos: &[i64], idx: &[usize]) -> Result<Self, MeshError> {
        Self::from_i64_triangles_with_policy(pos, idx, MeshValidationPolicy::CLOSED)
    }

    /// Construct an exact mesh from integer coordinates with an explicit
    /// validation policy.
    pub(crate) fn from_i64_triangles_with_policy(
        pos: &[i64],
        idx: &[usize],
        policy: MeshValidationPolicy,
    ) -> Result<Self, MeshError> {
        validate_flat_mesh_buffers(pos.len(), idx.len())?;

        let vertices = pos
            .chunks_exact(3)
            .map(|coords| {
                Point3::new(
                    Real::from(coords[0]),
                    Real::from(coords[1]),
                    Real::from(coords[2]),
                )
            })
            .collect::<Vec<_>>();

        Self::new_with_policy_and_version(
            vertices,
            flat_triangles(idx),
            SourceProvenance::exact("flat i64 triangle mesh"),
            policy,
            1,
        )
    }

    /// Return exact vertices for internal kernel stages.
    pub(crate) fn vertices(&self) -> &[Point3] {
        &self.vertices
    }

    /// Return retained triangle index storage.
    pub(crate) fn triangles(&self) -> &[Triangle] {
        &self.triangles
    }

    /// Return retained exact broad-phase bounds.
    ///
    /// The bounds can safely reject disjoint pairs. Non-disjoint box relations
    /// are only candidates for exact narrow-phase predicates and must not be
    /// treated as topology decisions.
    pub(crate) const fn bounds(&self) -> &MeshBounds {
        &self.bounds
    }

    /// Return retained validation facts.
    pub(crate) const fn facts(&self) -> &MeshValidationFacts {
        &self.facts
    }

    /// Return the validation policy retained at construction.
    ///
    /// The policy is part of the exact artifact boundary: an open-surface mesh
    /// constructed with [`MeshValidationPolicy::ALLOW_BOUNDARY`] must not later be
    /// mistaken for closed-solid evidence merely because its retained facts are
    /// locally coherent.
    pub(crate) const fn validation_policy(&self) -> MeshValidationPolicy {
        self.validation_policy
    }

    /// Return construction provenance.
    pub const fn provenance(&self) -> &ConstructionProvenance {
        &self.provenance
    }

    /// Borrow this exact mesh through the lightweight query view API.
    pub const fn view(&self) -> MeshView<'_> {
        MeshView { mesh: self }
    }

    /// Validate all retained state stored on this exact mesh.
    ///
    /// Mesh construction already validates inputs before returning `Ok`. This
    /// method exists for tests, fuzzing, serialization boundaries, and
    /// downstream exact algorithms that receive an `Mesh` artifact and
    /// want to audit that its retained bounds, topology facts, and provenance
    /// still agree before consuming them. The bounds and topology facts are
    /// replayed from the exact vertices and triangle rows before acceptance.
    pub(crate) fn validate_retained_state(&self) -> Result<(), MeshError> {
        self.validate_retained_state_detail().map_err(|error| {
            retained_validation_mesh_error("exact mesh retained state replay failed", error)
        })
    }

    pub(crate) fn validate_retained_state_detail(&self) -> Result<(), MeshValidationError> {
        if self.vertices.len() != self.facts.mesh.vertex_count {
            return Err(MeshValidationError::VertexCountMismatch {
                expected: self.vertices.len(),
                actual: self.facts.mesh.vertex_count,
            });
        }
        if self.triangles.len() != self.facts.mesh.face_count {
            return Err(MeshValidationError::FaceCountMismatch {
                expected: self.triangles.len(),
                actual: self.facts.mesh.face_count,
            });
        }
        self.bounds
            .validate_against_triangle_rows(
                &self.vertices,
                self.triangles.len(),
                self.triangles.iter().map(|triangle| triangle.0),
            )
            .map_err(retained_bounds_validation_error)?;
        self.facts
            .validate_against_triangle_rows_with_policy(
                &self.vertices,
                self.triangles.len(),
                self.triangles.iter().map(|triangle| triangle.0),
                self.validation_policy,
            )
            .map_err(retained_facts_validation_error)?;
        self.provenance
            .validate()
            .map_err(MeshValidationError::Provenance)?;

        let retained_predicates = self
            .facts
            .faces
            .iter()
            .flat_map(|face| face.triangle.degeneracy_predicates.iter().copied());
        if !self
            .provenance
            .predicates
            .iter()
            .copied()
            .eq(retained_predicates)
        {
            return Err(MeshValidationError::PredicateRetentionMismatch);
        }
        Ok(())
    }

    /// Validate the retained broad-phase bounds certificate without recomputing it.
    ///
    /// `Mesh` construction computes bounds from the source vertices and
    /// triangles once. Routine broad-phase consumers use this cheap certificate
    /// check to ensure the retained bounds object has the expected shape and
    /// ordered exact intervals before consuming it.
    pub(crate) fn validate_retained_bounds_certificate(&self) -> Result<(), MeshError> {
        self.validate_retained_bounds_certificate_detail()
            .map_err(|error| {
                retained_validation_mesh_error(
                    "exact mesh retained bounds certificate failed",
                    error,
                )
            })
    }

    pub(crate) fn validate_retained_bounds_certificate_detail(
        &self,
    ) -> Result<(), MeshValidationError> {
        self.bounds
            .validate(
                self.vertices.len(),
                self.facts.mesh.edge_count,
                self.triangles.len(),
            )
            .map_err(retained_bounds_validation_error)
    }

    /// Replay retained exact bounds against this mesh's source vertices and faces.
    ///
    /// This is the explicit acceleration-structure audit for tests, fuzzing,
    /// and artifact boundaries. Normal broad-phase scheduling uses
    /// [`Self::validate_retained_bounds_certificate`] so already-retained
    /// construction facts are not recomputed on every use.
    pub(crate) fn validate_retained_bounds(&self) -> Result<(), MeshError> {
        self.validate_retained_bounds_detail().map_err(|error| {
            retained_validation_mesh_error("exact mesh retained bounds replay failed", error)
        })
    }

    pub(crate) fn validate_retained_bounds_detail(&self) -> Result<(), MeshValidationError> {
        self.bounds
            .validate_against_triangle_rows(
                &self.vertices,
                self.triangles.len(),
                self.triangles.iter().map(|triangle| triangle.0),
            )
            .map_err(retained_bounds_validation_error)
    }

    /// Materialize this mesh after a row-major exact homogeneous affine transform.
    ///
    /// The matrix must have final row `[0, 0, 0, 1]`. A negative linear
    /// determinant reverses triangle winding so transformed closed shells keep
    /// their outside orientation.
    pub fn transform(&self, matrix: [[Real; 4]; 4]) -> Result<Mesh, MeshError> {
        self.view().transform(matrix)
    }

    pub(crate) fn next_construction_version(&self) -> u64 {
        self.provenance
            .construction_version
            .saturating_add(1)
            .max(1)
    }

    /// Materialize this mesh with every triangle orientation reversed.
    pub fn inverse(&self) -> Result<Mesh, MeshError> {
        self.view().inverse()
    }

    /// Materialize the exact closed union of this mesh and `right`.
    ///
    /// This is the mesh-kernel convenience entry point for named booleans. It
    /// returns only the output mesh; callers that need retained arrangement
    /// evidence should query [`MeshView::with_arrangement_view`].
    pub fn union(&self, right: &Mesh) -> Result<Mesh, MeshError> {
        self.view().union(right.view())
    }

    /// Materialize the exact closed intersection of this mesh and `right`.
    ///
    /// Lower-dimensional contact is regularized into the representable triangle
    /// mesh result for the default closed output contract.
    pub fn intersection(&self, right: &Mesh) -> Result<Mesh, MeshError> {
        self.view().intersection(right.view())
    }

    /// Materialize the exact closed difference of this mesh minus `right`.
    pub fn difference(&self, right: &Mesh) -> Result<Mesh, MeshError> {
        self.view().difference(right.view())
    }

    /// Materialize the exact closed symmetric difference of this mesh and `right`.
    ///
    /// Each side difference is materialized through the exact kernel and then
    /// unioned under the same closed output contract.
    pub fn xor(&self, right: &Mesh) -> Result<Mesh, MeshError> {
        self.view().xor(right.view())
    }
}

fn validate_flat_mesh_buffers(position_len: usize, index_len: usize) -> Result<(), MeshError> {
    if !position_len.is_multiple_of(3) {
        return Err(MeshError::one(MeshBlocker::new(
            MeshBlockerKind::VertexBufferArity,
            "position buffer length must be a multiple of 3",
        )));
    }
    if !index_len.is_multiple_of(3) {
        return Err(MeshError::one(MeshBlocker::new(
            MeshBlockerKind::IndexBufferArity,
            "index buffer length must be a multiple of 3",
        )));
    }
    Ok(())
}

fn flat_triangles(idx: &[usize]) -> Vec<Triangle> {
    idx.chunks_exact(3)
        .map(|tri| Triangle([tri[0], tri[1], tri[2]]))
        .collect()
}

fn retain_predicates(provenance: &mut ConstructionProvenance, report: &ValidationReport) {
    for face in &report.facts.faces {
        for predicate in &face.triangle.degeneracy_predicates {
            provenance.push_predicate(PredicateUse::from_certificate(predicate.certificate));
        }
    }
}

const fn retained_bounds_validation_error(error: BoundsValidationError) -> MeshValidationError {
    match error {
        BoundsValidationError::InvertedAxis => MeshValidationError::RetainedBoundsInvertedAxis,
        BoundsValidationError::UnknownAxisOrder => {
            MeshValidationError::RetainedBoundsUnknownAxisOrder
        }
        BoundsValidationError::MissingMeshBounds => {
            MeshValidationError::RetainedBoundsMissingMeshBounds
        }
        BoundsValidationError::UnexpectedMeshBounds => {
            MeshValidationError::RetainedBoundsUnexpectedMeshBounds
        }
        BoundsValidationError::FaceBoundsCountMismatch => {
            MeshValidationError::RetainedBoundsFaceCountMismatch
        }
        BoundsValidationError::EdgeBoundsCountMismatch => {
            MeshValidationError::RetainedBoundsEdgeCountMismatch
        }
        BoundsValidationError::SourceReplayMismatch => {
            MeshValidationError::RetainedBoundsSourceReplayMismatch
        }
    }
}

const fn retained_facts_validation_error(error: MeshFactsValidationError) -> MeshValidationError {
    match error {
        MeshFactsValidationError::SummaryLengthMismatch {
            field,
            expected,
            actual,
        } => MeshValidationError::RetainedFactsSummaryLengthMismatch {
            field,
            expected,
            actual,
        },
        MeshFactsValidationError::SummaryCountMismatch {
            field,
            expected,
            actual,
        } => MeshValidationError::RetainedFactsSummaryCountMismatch {
            field,
            expected,
            actual,
        },
        MeshFactsValidationError::EulerCharacteristicMismatch { expected, actual } => {
            MeshValidationError::RetainedFactsEulerCharacteristicMismatch { expected, actual }
        }
        MeshFactsValidationError::ClosedManifoldMismatch { expected, actual } => {
            MeshValidationError::RetainedFactsClosedManifoldMismatch { expected, actual }
        }
        MeshFactsValidationError::FixedCoordinatesMismatch { expected, actual } => {
            MeshValidationError::RetainedFactsFixedCoordinatesMismatch { expected, actual }
        }
        MeshFactsValidationError::SourceReplayMismatch => {
            MeshValidationError::RetainedFactsSourceReplayMismatch
        }
        MeshFactsValidationError::VertexIndexMismatch { expected, actual } => {
            MeshValidationError::RetainedFactsVertexIndexMismatch { expected, actual }
        }
        MeshFactsValidationError::VertexIncidentFaceMismatch {
            vertex,
            expected,
            actual,
        } => MeshValidationError::RetainedFactsVertexIncidentFaceMismatch {
            vertex,
            expected,
            actual,
        },
        MeshFactsValidationError::VertexIncidentEdgeMismatch {
            vertex,
            expected,
            actual,
        } => MeshValidationError::RetainedFactsVertexIncidentEdgeMismatch {
            vertex,
            expected,
            actual,
        },
        MeshFactsValidationError::VertexIncidentFaceListMismatch {
            vertex,
            mismatch_index,
            expected_len,
            actual_len,
        } => MeshValidationError::RetainedFactsVertexIncidentFaceListMismatch {
            vertex,
            mismatch_index,
            expected_len,
            actual_len,
        },
        MeshFactsValidationError::VertexIncidentEdgeListMismatch {
            vertex,
            mismatch_index,
            expected_len,
            actual_len,
        } => MeshValidationError::RetainedFactsVertexIncidentEdgeListMismatch {
            vertex,
            mismatch_index,
            expected_len,
            actual_len,
        },
        MeshFactsValidationError::EdgeVertexOutOfBounds { edge, vertex_count } => {
            MeshValidationError::RetainedFactsEdgeVertexOutOfBounds { edge, vertex_count }
        }
        MeshFactsValidationError::EdgeEndpointOrder { edge } => {
            MeshValidationError::RetainedFactsEdgeEndpointOrder { edge }
        }
        MeshFactsValidationError::DuplicateEdgeFact { edge } => {
            MeshValidationError::RetainedFactsDuplicateEdgeFact { edge }
        }
        MeshFactsValidationError::UnexpectedEdgeFact { edge } => {
            MeshValidationError::RetainedFactsUnexpectedEdgeFact { edge }
        }
        MeshFactsValidationError::FaceVertexOutOfBounds {
            face,
            vertex,
            vertex_count,
        } => MeshValidationError::RetainedFactsFaceVertexOutOfBounds {
            face,
            vertex,
            vertex_count,
        },
        MeshFactsValidationError::FaceRepeatedVertex { face, vertices } => {
            MeshValidationError::RetainedFactsFaceRepeatedVertex { face, vertices }
        }
        MeshFactsValidationError::FaceIndexMismatch { expected, actual } => {
            MeshValidationError::RetainedFactsFaceIndexMismatch { expected, actual }
        }
        MeshFactsValidationError::FaceDirectedEdgesMismatch {
            face,
            expected,
            actual,
        } => MeshValidationError::RetainedFactsFaceDirectedEdgesMismatch {
            face,
            expected,
            actual,
        },
        MeshFactsValidationError::EdgeUseMismatch {
            edge,
            expected_directed_uses,
            actual_directed_uses,
            expected_incident_faces,
            actual_incident_faces,
        } => MeshValidationError::RetainedFactsEdgeUseMismatch {
            edge,
            expected_directed_uses,
            actual_directed_uses,
            expected_incident_faces,
            actual_incident_faces,
        },
        MeshFactsValidationError::MissingEdgeFact { edge } => {
            MeshValidationError::RetainedFactsMissingEdgeFact { edge }
        }
    }
}

fn retained_validation_mesh_error(context: &'static str, error: MeshValidationError) -> MeshError {
    let kind = match error {
        MeshValidationError::VertexCountMismatch { .. }
        | MeshValidationError::FaceCountMismatch { .. }
        | MeshValidationError::RetainedBoundsSourceReplayMismatch
        | MeshValidationError::RetainedFactsSourceReplayMismatch => {
            MeshBlockerKind::StaleFactReplay
        }
        MeshValidationError::RetainedBoundsUnknownAxisOrder => {
            MeshBlockerKind::UndecidablePredicate
        }
        _ => MeshBlockerKind::ExactConstructionFailure,
    };
    MeshError::one(MeshBlocker::new(kind, format!("{context}: {error:?}")))
}

fn transformed_coordinate(row: &[Real; 3], point: &Point3, translation: &Real) -> Real {
    Real::sum_owned([
        &row[0] * &point.x,
        &row[1] * &point.y,
        &row[2] * &point.z,
        translation.clone(),
    ])
}

fn det3_rows(rows: &[[Real; 3]; 3]) -> Real {
    let x_minor = &(&rows[1][1] * &rows[2][2]) - &(&rows[1][2] * &rows[2][1]);
    let y_minor = &(&rows[1][0] * &rows[2][2]) - &(&rows[1][2] * &rows[2][0]);
    let z_minor = &(&rows[1][0] * &rows[2][1]) - &(&rows[1][1] * &rows[2][0]);
    &(&rows[0][0] * &x_minor) - &(&rows[0][1] * &y_minor) + &(&rows[0][2] * &z_minor)
}

fn real_equals(left: &Real, right: &Real) -> Result<bool, MeshError> {
    compare_reals(left, right)
        .value()
        .map(|ordering| ordering == Ordering::Equal)
        .ok_or_else(|| {
            MeshError::one(MeshBlocker::new(
                MeshBlockerKind::UndecidablePredicate,
                "exact transform coefficient comparison could not be certified",
            ))
        })
}

pub(crate) fn reverse_triangle(triangle: &Triangle) -> Triangle {
    let [a, b, c] = triangle.0;
    Triangle([a, c, b])
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::facts::EdgeFacts;

    #[test]
    fn paired_triangle_orientation_flips_follow_shared_edge_parity() {
        assert_eq!(
            paired_triangle_orientation_flips([[0, 1, 2], [0, 1, 3]], 2),
            Some(vec![false, true])
        );
        assert_eq!(
            paired_triangle_orientation_flips([[0, 1, 2], [1, 0, 3]], 2),
            Some(vec![false, false])
        );
    }

    #[test]
    fn retained_facts_reject_unexpected_zero_use_edge() {
        let mut mesh = Mesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1],
            &[0, 1, 2],
            MeshValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();

        mesh.facts.edges.push(EdgeFacts {
            vertices: [0, 3],
            incident_faces: 0,
            directed_uses: [0, 0],
        });
        mesh.facts.mesh.edge_count += 1;
        mesh.facts.mesh.euler_characteristic -= 1;

        assert_eq!(
            mesh.validate_retained_state_detail(),
            Err(MeshValidationError::RetainedFactsUnexpectedEdgeFact { edge: [0, 3] })
        );
    }

    #[test]
    fn retained_facts_reject_stale_vertex_incident_faces() {
        let mut mesh = Mesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 1, 0, 0, 0, 1, 0],
            &[0, 1, 2],
            MeshValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();

        mesh.facts.vertices[0].incident_face_indices.clear();

        assert_eq!(
            mesh.validate_retained_state_detail(),
            Err(
                MeshValidationError::RetainedFactsVertexIncidentFaceListMismatch {
                    vertex: 0,
                    mismatch_index: 0,
                    expected_len: 1,
                    actual_len: 0,
                },
            )
        );
    }

    #[test]
    fn retained_facts_reject_stale_vertex_incident_edges() {
        let mut mesh = Mesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 1, 0, 0, 0, 1, 0],
            &[0, 1, 2],
            MeshValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();

        mesh.facts.vertices[0].incident_edge_indices.clear();

        assert_eq!(
            mesh.validate_retained_state_detail(),
            Err(
                MeshValidationError::RetainedFactsVertexIncidentEdgeListMismatch {
                    vertex: 0,
                    mismatch_index: 0,
                    expected_len: 2,
                    actual_len: 0,
                },
            )
        );
    }
}
