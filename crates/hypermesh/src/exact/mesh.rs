//! Exact mesh construction and storage.
//!
//! `ExactMesh` stores coordinates as `hyperlattice::Vector3` over
//! `hyperreal::Real` and mirrors them into `hyperlimit::Point3` only when
//! predicate-facing APIs need point facts. Primitive-float construction is a
//! named lossy adapter and validates every coordinate before import.

use hyperlattice::Vector3;
use hyperlimit::Point3;

use super::bounds::MeshBounds;
use super::error::{DiagnosticKind, MeshDiagnostic, MeshError, Severity};
use super::facts::MeshValidationFacts;
use super::provenance::{ConstructionProvenance, PredicateUse, SourceProvenance};
use super::scalar::{ExactReal, LossyF64Import};
use super::validation::{ValidationPolicy, ValidationReport, validate_triangles_with_policy};

/// Exact 3D point stored in hypermesh.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactPoint3 {
    coordinates: Vector3,
}

impl ExactPoint3 {
    /// Construct a point from exact coordinates.
    pub fn new(x: ExactReal, y: ExactReal, z: ExactReal) -> Self {
        Self {
            coordinates: Vector3::new([x, y, z]),
        }
    }

    /// Import a point from a finite primitive-float triplet.
    pub fn from_f64_lossy(values: [f64; 3], first_coordinate: usize) -> Result<Self, MeshError> {
        let x = LossyF64Import::new(values[0], first_coordinate).map_err(MeshError::one)?;
        let y = LossyF64Import::new(values[1], first_coordinate + 1).map_err(MeshError::one)?;
        let z = LossyF64Import::new(values[2], first_coordinate + 2).map_err(MeshError::one)?;
        Ok(Self::new(x.value, y.value, z.value))
    }

    /// Return exact coordinates.
    pub const fn coordinates(&self) -> &Vector3 {
        &self.coordinates
    }

    /// Convert to the `hyperlimit` point carrier used by exact predicates.
    pub fn to_hyperlimit_point(&self) -> Point3 {
        Point3::new(
            self.coordinates.0[0].clone(),
            self.coordinates.0[1].clone(),
            self.coordinates.0[2].clone(),
        )
    }
}

/// Triangle index triplet.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Triangle(pub [usize; 3]);

/// Exact triangular mesh with retained validation facts.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactMesh {
    vertices: Vec<ExactPoint3>,
    triangles: Vec<Triangle>,
    bounds: MeshBounds,
    facts: MeshValidationFacts,
    provenance: ConstructionProvenance,
}

impl ExactMesh {
    /// Construct an exact mesh from exact vertices and triangle indices.
    pub fn new(
        vertices: Vec<ExactPoint3>,
        triangles: Vec<Triangle>,
        source: SourceProvenance,
    ) -> Result<Self, MeshError> {
        Self::new_with_policy(vertices, triangles, source, ValidationPolicy::CLOSED)
    }

    /// Construct an exact mesh with an explicit validation policy.
    pub fn new_with_policy(
        vertices: Vec<ExactPoint3>,
        triangles: Vec<Triangle>,
        source: SourceProvenance,
        policy: ValidationPolicy,
    ) -> Result<Self, MeshError> {
        let index_diagnostics = validate_indices(vertices.len(), &triangles);
        if !index_diagnostics.is_empty() {
            return Err(MeshError::new(index_diagnostics));
        }

        let points = vertices
            .iter()
            .map(ExactPoint3::to_hyperlimit_point)
            .collect::<Vec<_>>();
        let triangle_indices = triangles.iter().map(|tri| tri.0).collect::<Vec<_>>();
        let bounds = MeshBounds::from_triangles(&points, &triangle_indices);
        let report = validate_triangles_with_policy(&points, &triangle_indices, policy);
        if !report.is_valid() {
            return Err(MeshError::new(report.diagnostics));
        }

        let mut provenance = ConstructionProvenance::new(source);
        retain_predicates(&mut provenance, &report);

        Ok(Self {
            vertices,
            triangles,
            bounds,
            facts: report.facts,
            provenance,
        })
    }

    /// Construct an exact mesh from flat primitive-float coordinates.
    ///
    /// The `f64` values are checked for finiteness and imported as exact dyadic
    /// `Real` values. They are not used later as tolerance-bearing floats.
    pub fn from_f64_triangles(pos: &[f64], idx: &[usize]) -> Result<Self, MeshError> {
        Self::from_f64_triangles_with_policy(pos, idx, ValidationPolicy::CLOSED)
    }

    /// Construct an exact mesh from flat primitive-float coordinates with an
    /// explicit validation policy.
    pub fn from_f64_triangles_with_policy(
        pos: &[f64],
        idx: &[usize],
        policy: ValidationPolicy,
    ) -> Result<Self, MeshError> {
        if pos.len() % 3 != 0 {
            return Err(MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::VertexBufferArity,
                "position buffer length must be a multiple of 3",
            )));
        }
        if idx.len() % 3 != 0 {
            return Err(MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::IndexBufferArity,
                "index buffer length must be a multiple of 3",
            )));
        }

        let mut vertices = Vec::with_capacity(pos.len() / 3);
        for (vertex, coords) in pos.chunks_exact(3).enumerate() {
            let point = ExactPoint3::from_f64_lossy([coords[0], coords[1], coords[2]], vertex * 3)?;
            vertices.push(point);
        }

        let triangles = idx
            .chunks_exact(3)
            .map(|tri| Triangle([tri[0], tri[1], tri[2]]))
            .collect::<Vec<_>>();

        Self::new_with_policy(
            vertices,
            triangles,
            SourceProvenance::lossy_f64("flat f64 triangle mesh"),
            policy,
        )
    }

    /// Construct an exact mesh from flat integer coordinates.
    ///
    /// Integer grid input is lifted directly into `hyperreal::Real` without a
    /// primitive-float edge. Keeping grid coordinates exact and structurally
    /// visible follows Yap's recommendation to retain object-level numerical
    /// structure for downstream exact predicates and determinant schedules.
    pub fn from_i64_triangles(pos: &[i64], idx: &[usize]) -> Result<Self, MeshError> {
        Self::from_i64_triangles_with_policy(pos, idx, ValidationPolicy::CLOSED)
    }

    /// Construct an exact mesh from integer coordinates with an explicit
    /// validation policy.
    pub fn from_i64_triangles_with_policy(
        pos: &[i64],
        idx: &[usize],
        policy: ValidationPolicy,
    ) -> Result<Self, MeshError> {
        if pos.len() % 3 != 0 {
            return Err(MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::VertexBufferArity,
                "position buffer length must be a multiple of 3",
            )));
        }
        if idx.len() % 3 != 0 {
            return Err(MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::IndexBufferArity,
                "index buffer length must be a multiple of 3",
            )));
        }

        let vertices = pos
            .chunks_exact(3)
            .map(|coords| {
                ExactPoint3::new(
                    ExactReal::from(coords[0]),
                    ExactReal::from(coords[1]),
                    ExactReal::from(coords[2]),
                )
            })
            .collect::<Vec<_>>();
        let triangles = idx
            .chunks_exact(3)
            .map(|tri| Triangle([tri[0], tri[1], tri[2]]))
            .collect::<Vec<_>>();

        Self::new_with_policy(
            vertices,
            triangles,
            SourceProvenance::exact("flat i64 triangle mesh"),
            policy,
        )
    }

    /// Return exact vertices.
    pub fn vertices(&self) -> &[ExactPoint3] {
        &self.vertices
    }

    /// Return triangle indices.
    pub fn triangles(&self) -> &[Triangle] {
        &self.triangles
    }

    /// Return retained exact broad-phase bounds.
    ///
    /// The bounds can safely reject disjoint pairs. Non-disjoint box relations
    /// are only candidates for exact narrow-phase predicates and must not be
    /// treated as topology decisions.
    pub const fn bounds(&self) -> &MeshBounds {
        &self.bounds
    }

    /// Return retained validation facts.
    pub const fn facts(&self) -> &MeshValidationFacts {
        &self.facts
    }

    /// Return construction provenance.
    pub const fn provenance(&self) -> &ConstructionProvenance {
        &self.provenance
    }
}

fn validate_indices(vertex_count: usize, triangles: &[Triangle]) -> Vec<MeshDiagnostic> {
    let mut diagnostics = Vec::new();
    for (face, triangle) in triangles.iter().enumerate() {
        let [a, b, c] = triangle.0;
        for vertex in [a, b, c] {
            if vertex >= vertex_count {
                diagnostics.push(
                    MeshDiagnostic::new(
                        Severity::Error,
                        DiagnosticKind::IndexOutOfBounds,
                        format!(
                            "face {face} references vertex {vertex}, but only {vertex_count} vertices exist"
                        ),
                    )
                    .with_face(face)
                    .with_vertex(vertex),
                );
            }
        }
        if a == b || b == c || c == a {
            diagnostics.push(
                MeshDiagnostic::new(
                    Severity::Error,
                    DiagnosticKind::DegenerateTriangle,
                    format!("face {face} repeats a vertex"),
                )
                .with_face(face),
            );
        }
    }
    diagnostics
}

fn retain_predicates(provenance: &mut ConstructionProvenance, report: &ValidationReport) {
    for face in &report.facts.faces {
        for predicate in &face.triangle.degeneracy_predicates {
            provenance.push_predicate(PredicateUse::from_certificate(predicate.certificate));
        }
    }
}
