//! Report-bearing edge adapters for exact mesh construction.
//!
//! Primitive `f64` streams are an interoperability edge, not exact topology.
//! This module audits the coordinate and index stream before construction:
//! finite floats are lifted as exact dyadics, index ranges are checked against
//! the lifted vertex count, and repeated triangle indices are rejected before
//! predicate work begins. Topology is still certified later by
//! [`ExactMesh`](super::ExactMesh). Approximate input channels can propose exact
//! objects only after their approximation policy and conversion evidence are
//! explicit.

use hyperlimit::{ApproximationPolicy, MeshSource, SourceProvenance};

use super::error::{DiagnosticKind, MeshDiagnostic, Severity};
use crate::scalar::LossyF64Import;

/// Audited primitive-float mesh input stream.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct LossyF64MeshInputReport {
    /// Source provenance assigned to accepted primitive-float streams.
    pub source: SourceProvenance,
    /// Flat coordinate count supplied by the caller.
    pub coordinate_count: usize,
    /// Flat index count supplied by the caller.
    pub index_count: usize,
    /// Vertex count when coordinate arity is valid.
    pub vertex_count: Option<usize>,
    /// Face count when index arity is valid.
    pub face_count: Option<usize>,
    /// Number of coordinates proved finite and importable as exact dyadics.
    pub exact_dyadic_coordinates: usize,
    /// Number of triangle indices checked against the vertex count.
    pub checked_indices: usize,
    /// Diagnostics found at the adapter boundary.
    pub diagnostics: Vec<MeshDiagnostic>,
}

/// Audited exact-integer mesh input stream.
///
/// Integer coordinate streams are exact source data rather than lossy adapter
/// data, but their buffer shape and index rows still need an explicit report
/// before topology predicates run. Keeping this input audit separate from
/// and topology certificates should be visible artifacts, not hidden side
/// effects of a successful constructor.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ExactI64MeshInputReport {
    /// Source provenance assigned to accepted integer streams.
    pub source: SourceProvenance,
    /// Flat coordinate count supplied by the caller.
    pub coordinate_count: usize,
    /// Flat index count supplied by the caller.
    pub index_count: usize,
    /// Vertex count when coordinate arity is valid.
    pub vertex_count: Option<usize>,
    /// Face count when index arity is valid.
    pub face_count: Option<usize>,
    /// Number of coordinates accepted as exact integer scalars.
    pub exact_integer_coordinates: usize,
    /// Number of triangle indices checked against the vertex count.
    pub checked_indices: usize,
    /// Diagnostics found at the exact input boundary.
    pub diagnostics: Vec<MeshDiagnostic>,
}

/// Validation failure for a retained [`LossyF64MeshInputReport`].
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum LossyF64MeshInputReportValidationError {
    /// The source provenance is not the required lossy-float edge policy.
    SourcePolicyMismatch,
    /// Invalid coordinate arity was retained without the required diagnostic.
    MissingCoordinateArityDiagnostic,
    /// Invalid index arity was retained without the required diagnostic.
    MissingIndexArityDiagnostic,
    /// Coordinate arity and retained vertex count disagree.
    CoordinateCountMismatch,
    /// Index arity and retained face count disagree.
    IndexCountMismatch,
    /// Exact coordinate evidence and retained coordinate diagnostics disagree.
    ExactCoordinateCountMismatch,
    /// Checked-index evidence does not match the checkable input range.
    CheckedIndexCountMismatch,
    /// A coordinate diagnostic references an out-of-range coordinate.
    CoordinateDiagnosticOutOfRange {
        /// Coordinate index stored in the diagnostic.
        coordinate: usize,
        /// Number of flat coordinates in the report.
        coordinate_count: usize,
    },
    /// A face diagnostic references an out-of-range face.
    FaceDiagnosticOutOfRange {
        /// Face index stored in the diagnostic.
        face: usize,
        /// Face count retained by the report.
        face_count: Option<usize>,
    },
    /// A vertex diagnostic references an out-of-range vertex location.
    VertexDiagnosticOutOfRange {
        /// Vertex index stored in the diagnostic.
        vertex: usize,
        /// Vertex count retained by the report.
        vertex_count: Option<usize>,
    },
    /// The exact-dyadic coordinate count exceeds the supplied coordinates.
    ExactCoordinateCountOverflow,
    /// The checked-index count exceeds the supplied indices.
    CheckedIndexCountOverflow,
}

/// Validation failure for a retained [`ExactI64MeshInputReport`].
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ExactI64MeshInputReportValidationError {
    /// The source provenance is not the required exact-input policy.
    SourcePolicyMismatch,
    /// Invalid coordinate arity was retained without the required diagnostic.
    MissingCoordinateArityDiagnostic,
    /// Invalid index arity was retained without the required diagnostic.
    MissingIndexArityDiagnostic,
    /// Coordinate arity and retained vertex count disagree.
    CoordinateCountMismatch,
    /// Index arity and retained face count disagree.
    IndexCountMismatch,
    /// Exact coordinate evidence does not cover the retained coordinate stream.
    ExactCoordinateCountMismatch,
    /// Checked-index evidence does not match the checkable input range.
    CheckedIndexCountMismatch,
    /// A face diagnostic references an out-of-range face.
    FaceDiagnosticOutOfRange {
        /// Face index stored in the diagnostic.
        face: usize,
        /// Face count retained by the report.
        face_count: Option<usize>,
    },
    /// A vertex diagnostic references an out-of-range vertex location.
    VertexDiagnosticOutOfRange {
        /// Vertex index stored in the diagnostic.
        vertex: usize,
        /// Vertex count retained by the report.
        vertex_count: Option<usize>,
    },
    /// The exact-integer coordinate count exceeds the supplied coordinates.
    ExactCoordinateCountOverflow,
    /// The checked-index count exceeds the supplied indices.
    CheckedIndexCountOverflow,
}

impl LossyF64MeshInputReportValidationError {
    /// Return whether exact-coordinate replay evidence is inconsistent.
    pub fn is_exact_coordinate_count_mismatch(&self) -> bool {
        matches!(self, Self::ExactCoordinateCountMismatch)
    }

    /// Return whether checked-index replay evidence is inconsistent.
    pub fn is_checked_index_count_mismatch(&self) -> bool {
        matches!(self, Self::CheckedIndexCountMismatch)
    }

    /// Return whether invalid coordinate arity lacked its diagnostic.
    pub fn is_missing_coordinate_arity_diagnostic(&self) -> bool {
        matches!(self, Self::MissingCoordinateArityDiagnostic)
    }

    /// Return whether invalid index arity lacked its diagnostic.
    pub fn is_missing_index_arity_diagnostic(&self) -> bool {
        matches!(self, Self::MissingIndexArityDiagnostic)
    }
}

impl ExactI64MeshInputReportValidationError {
    /// Return whether exact-coordinate replay evidence is inconsistent.
    pub fn is_exact_coordinate_count_mismatch(&self) -> bool {
        matches!(self, Self::ExactCoordinateCountMismatch)
    }

    /// Return whether checked-index replay evidence is inconsistent.
    pub fn is_checked_index_count_mismatch(&self) -> bool {
        matches!(self, Self::CheckedIndexCountMismatch)
    }

    /// Return whether invalid coordinate arity lacked its diagnostic.
    pub fn is_missing_coordinate_arity_diagnostic(&self) -> bool {
        matches!(self, Self::MissingCoordinateArityDiagnostic)
    }

    /// Return whether invalid index arity lacked its diagnostic.
    pub fn is_missing_index_arity_diagnostic(&self) -> bool {
        matches!(self, Self::MissingIndexArityDiagnostic)
    }
}

/// Readiness status for a primitive-float input report.
///
/// This is a preconstruction adapter diagnostic, not a topology claim. `Ready`
/// means the flat stream can be handed to exact mesh construction; exact
/// manifoldness and predicate decisions still happen later. This is the input
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LossyF64MeshInputReadiness {
    /// The report is internally valid and has no error diagnostics.
    Ready,
    /// The report itself is internally inconsistent.
    InvalidReport,
    /// The coordinate buffer arity is invalid.
    InvalidCoordinateArity,
    /// The index buffer arity is invalid.
    InvalidIndexArity,
    /// At least one coordinate is non-finite or failed exact dyadic import.
    InvalidCoordinate,
    /// At least one index row references an unavailable vertex.
    InvalidIndex,
    /// At least one triangle repeats a vertex before predicate validation.
    RepeatedTriangleVertex,
    /// Some other fatal diagnostic is present.
    FatalDiagnostic,
}

/// Readiness status for an exact-integer input report.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactI64MeshInputReadiness {
    /// The report is internally valid and has no error diagnostics.
    Ready,
    /// The report itself is internally inconsistent.
    InvalidReport,
    /// The coordinate buffer arity is invalid.
    InvalidCoordinateArity,
    /// The index buffer arity is invalid.
    InvalidIndexArity,
    /// At least one index row references an unavailable vertex.
    InvalidIndex,
    /// At least one triangle repeats a vertex before predicate validation.
    RepeatedTriangleVertex,
    /// Some other fatal diagnostic is present.
    FatalDiagnostic,
}

impl LossyF64MeshInputReport {
    /// Audit a flat primitive-float triangle mesh input stream.
    pub fn inspect(pos: &[f64], idx: &[usize]) -> Self {
        let mut diagnostics = Vec::new();
        let vertex_count = if pos.len().is_multiple_of(3) {
            Some(pos.len() / 3)
        } else {
            diagnostics.push(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::VertexBufferArity,
                "position buffer length must be a multiple of 3",
            ));
            None
        };
        let face_count = if idx.len().is_multiple_of(3) {
            Some(idx.len() / 3)
        } else {
            diagnostics.push(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::IndexBufferArity,
                "index buffer length must be a multiple of 3",
            ));
            None
        };

        let mut exact_dyadic_coordinates = 0;
        for (coordinate, value) in pos.iter().copied().enumerate() {
            match LossyF64Import::new(value, coordinate) {
                Ok(_) => exact_dyadic_coordinates += 1,
                Err(diagnostic) => diagnostics.push(diagnostic),
            }
        }

        let mut checked_indices = 0;
        if let Some(vertex_count) = vertex_count {
            for (face, triangle) in idx.chunks_exact(3).enumerate() {
                let [a, b, c] = [triangle[0], triangle[1], triangle[2]];
                for vertex in [a, b, c] {
                    checked_indices += 1;
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
        }

        Self {
            source: SourceProvenance::lossy_f64("flat f64 triangle mesh"),
            coordinate_count: pos.len(),
            index_count: idx.len(),
            vertex_count,
            face_count,
            exact_dyadic_coordinates,
            checked_indices,
            diagnostics,
        }
    }

    /// Return whether the stream can attempt exact mesh construction.
    pub fn edge_ready(&self) -> bool {
        self.readiness() == LossyF64MeshInputReadiness::Ready
    }

    /// Classify whether this report can attempt exact construction.
    pub fn readiness(&self) -> LossyF64MeshInputReadiness {
        if self.validate().is_err() {
            return LossyF64MeshInputReadiness::InvalidReport;
        }
        for diagnostic in &self.diagnostics {
            if diagnostic.severity != Severity::Error {
                continue;
            }
            return match diagnostic.kind {
                DiagnosticKind::VertexBufferArity => {
                    LossyF64MeshInputReadiness::InvalidCoordinateArity
                }
                DiagnosticKind::IndexBufferArity => LossyF64MeshInputReadiness::InvalidIndexArity,
                DiagnosticKind::NonFiniteCoordinate | DiagnosticKind::CoordinateImportFailed => {
                    LossyF64MeshInputReadiness::InvalidCoordinate
                }
                DiagnosticKind::IndexOutOfBounds => LossyF64MeshInputReadiness::InvalidIndex,
                DiagnosticKind::DegenerateTriangle => {
                    LossyF64MeshInputReadiness::RepeatedTriangleVertex
                }
                _ => LossyF64MeshInputReadiness::FatalDiagnostic,
            };
        }
        LossyF64MeshInputReadiness::Ready
    }

    /// Validate this retained adapter report for internal consistency.
    pub fn validate(&self) -> Result<(), LossyF64MeshInputReportValidationError> {
        if self.source.source != MeshSource::LossyF64
            || self.source.approximation != ApproximationPolicy::EdgeOnly
        {
            return Err(LossyF64MeshInputReportValidationError::SourcePolicyMismatch);
        }
        if !self.coordinate_count.is_multiple_of(3)
            && !has_error_diagnostic(&self.diagnostics, DiagnosticKind::VertexBufferArity)
        {
            return Err(LossyF64MeshInputReportValidationError::MissingCoordinateArityDiagnostic);
        }
        if !self.index_count.is_multiple_of(3)
            && !has_error_diagnostic(&self.diagnostics, DiagnosticKind::IndexBufferArity)
        {
            return Err(LossyF64MeshInputReportValidationError::MissingIndexArityDiagnostic);
        }
        if self.vertex_count
            != self
                .coordinate_count
                .is_multiple_of(3)
                .then_some(self.coordinate_count / 3)
        {
            return Err(LossyF64MeshInputReportValidationError::CoordinateCountMismatch);
        }
        if self.face_count
            != self
                .index_count
                .is_multiple_of(3)
                .then_some(self.index_count / 3)
        {
            return Err(LossyF64MeshInputReportValidationError::IndexCountMismatch);
        }
        if self.exact_dyadic_coordinates > self.coordinate_count {
            return Err(LossyF64MeshInputReportValidationError::ExactCoordinateCountOverflow);
        }
        if self.checked_indices > self.index_count {
            return Err(LossyF64MeshInputReportValidationError::CheckedIndexCountOverflow);
        }
        if self
            .exact_dyadic_coordinates
            .checked_add(failed_f64_coordinate_count(&self.diagnostics))
            != Some(self.coordinate_count)
        {
            return Err(LossyF64MeshInputReportValidationError::ExactCoordinateCountMismatch);
        }
        if self.checked_indices != expected_checked_index_count(self.vertex_count, self.index_count)
        {
            return Err(LossyF64MeshInputReportValidationError::CheckedIndexCountMismatch);
        }
        for diagnostic in &self.diagnostics {
            if let Some(coordinate) = diagnostic.coordinate
                && coordinate >= self.coordinate_count
            {
                return Err(
                    LossyF64MeshInputReportValidationError::CoordinateDiagnosticOutOfRange {
                        coordinate,
                        coordinate_count: self.coordinate_count,
                    },
                );
            }
            if let Some(face) = diagnostic.face
                && self.face_count.is_none_or(|face_count| face >= face_count)
            {
                return Err(
                    LossyF64MeshInputReportValidationError::FaceDiagnosticOutOfRange {
                        face,
                        face_count: self.face_count,
                    },
                );
            }
            if let Some(vertex) = diagnostic.vertex
                && self
                    .vertex_count
                    .is_none_or(|vertex_count| vertex >= vertex_count)
                && diagnostic.kind != DiagnosticKind::IndexOutOfBounds
            {
                return Err(
                    LossyF64MeshInputReportValidationError::VertexDiagnosticOutOfRange {
                        vertex,
                        vertex_count: self.vertex_count,
                    },
                );
            }
        }
        Ok(())
    }
}

impl ExactI64MeshInputReport {
    /// Audit a flat exact-integer triangle mesh input stream.
    pub fn inspect(pos: &[i64], idx: &[usize]) -> Self {
        let mut diagnostics = Vec::new();
        let vertex_count = if pos.len().is_multiple_of(3) {
            Some(pos.len() / 3)
        } else {
            diagnostics.push(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::VertexBufferArity,
                "position buffer length must be a multiple of 3",
            ));
            None
        };
        let face_count = if idx.len().is_multiple_of(3) {
            Some(idx.len() / 3)
        } else {
            diagnostics.push(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::IndexBufferArity,
                "index buffer length must be a multiple of 3",
            ));
            None
        };

        let mut checked_indices = 0;
        if let Some(vertex_count) = vertex_count {
            for (face, triangle) in idx.chunks_exact(3).enumerate() {
                let [a, b, c] = [triangle[0], triangle[1], triangle[2]];
                for vertex in [a, b, c] {
                    checked_indices += 1;
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
        }

        Self {
            source: SourceProvenance::exact("flat i64 triangle mesh"),
            coordinate_count: pos.len(),
            index_count: idx.len(),
            vertex_count,
            face_count,
            exact_integer_coordinates: pos.len(),
            checked_indices,
            diagnostics,
        }
    }

    /// Return whether the stream can attempt exact mesh construction.
    pub fn edge_ready(&self) -> bool {
        self.readiness() == ExactI64MeshInputReadiness::Ready
    }

    /// Classify whether this report can attempt exact construction.
    pub fn readiness(&self) -> ExactI64MeshInputReadiness {
        if self.validate().is_err() {
            return ExactI64MeshInputReadiness::InvalidReport;
        }
        for diagnostic in &self.diagnostics {
            if diagnostic.severity != Severity::Error {
                continue;
            }
            return match diagnostic.kind {
                DiagnosticKind::VertexBufferArity => {
                    ExactI64MeshInputReadiness::InvalidCoordinateArity
                }
                DiagnosticKind::IndexBufferArity => ExactI64MeshInputReadiness::InvalidIndexArity,
                DiagnosticKind::IndexOutOfBounds => ExactI64MeshInputReadiness::InvalidIndex,
                DiagnosticKind::DegenerateTriangle => {
                    ExactI64MeshInputReadiness::RepeatedTriangleVertex
                }
                _ => ExactI64MeshInputReadiness::FatalDiagnostic,
            };
        }
        ExactI64MeshInputReadiness::Ready
    }

    /// Validate this retained exact-input report for internal consistency.
    pub fn validate(&self) -> Result<(), ExactI64MeshInputReportValidationError> {
        if self.source.source != MeshSource::Exact
            || self.source.approximation != ApproximationPolicy::ExactOnly
        {
            return Err(ExactI64MeshInputReportValidationError::SourcePolicyMismatch);
        }
        if !self.coordinate_count.is_multiple_of(3)
            && !has_error_diagnostic(&self.diagnostics, DiagnosticKind::VertexBufferArity)
        {
            return Err(ExactI64MeshInputReportValidationError::MissingCoordinateArityDiagnostic);
        }
        if !self.index_count.is_multiple_of(3)
            && !has_error_diagnostic(&self.diagnostics, DiagnosticKind::IndexBufferArity)
        {
            return Err(ExactI64MeshInputReportValidationError::MissingIndexArityDiagnostic);
        }
        if self.vertex_count
            != self
                .coordinate_count
                .is_multiple_of(3)
                .then_some(self.coordinate_count / 3)
        {
            return Err(ExactI64MeshInputReportValidationError::CoordinateCountMismatch);
        }
        if self.face_count
            != self
                .index_count
                .is_multiple_of(3)
                .then_some(self.index_count / 3)
        {
            return Err(ExactI64MeshInputReportValidationError::IndexCountMismatch);
        }
        if self.exact_integer_coordinates > self.coordinate_count {
            return Err(ExactI64MeshInputReportValidationError::ExactCoordinateCountOverflow);
        }
        if self.checked_indices > self.index_count {
            return Err(ExactI64MeshInputReportValidationError::CheckedIndexCountOverflow);
        }
        if self.exact_integer_coordinates != self.coordinate_count {
            return Err(ExactI64MeshInputReportValidationError::ExactCoordinateCountMismatch);
        }
        if self.checked_indices != expected_checked_index_count(self.vertex_count, self.index_count)
        {
            return Err(ExactI64MeshInputReportValidationError::CheckedIndexCountMismatch);
        }
        for diagnostic in &self.diagnostics {
            if let Some(face) = diagnostic.face
                && self.face_count.is_none_or(|face_count| face >= face_count)
            {
                return Err(
                    ExactI64MeshInputReportValidationError::FaceDiagnosticOutOfRange {
                        face,
                        face_count: self.face_count,
                    },
                );
            }
            if let Some(vertex) = diagnostic.vertex
                && self
                    .vertex_count
                    .is_none_or(|vertex_count| vertex >= vertex_count)
                && diagnostic.kind != DiagnosticKind::IndexOutOfBounds
            {
                return Err(
                    ExactI64MeshInputReportValidationError::VertexDiagnosticOutOfRange {
                        vertex,
                        vertex_count: self.vertex_count,
                    },
                );
            }
        }
        Ok(())
    }
}

/// Audit a primitive-float mesh input stream before exact construction.
pub(crate) fn inspect_f64_mesh_input(pos: &[f64], idx: &[usize]) -> LossyF64MeshInputReport {
    LossyF64MeshInputReport::inspect(pos, idx)
}

/// Audit an exact-integer mesh input stream before exact construction.
pub(crate) fn inspect_i64_mesh_input(pos: &[i64], idx: &[usize]) -> ExactI64MeshInputReport {
    ExactI64MeshInputReport::inspect(pos, idx)
}

fn has_error_diagnostic(diagnostics: &[MeshDiagnostic], kind: DiagnosticKind) -> bool {
    diagnostics
        .iter()
        .any(|diagnostic| diagnostic.severity == Severity::Error && diagnostic.kind == kind)
}

fn failed_f64_coordinate_count(diagnostics: &[MeshDiagnostic]) -> usize {
    let mut coordinates = diagnostics
        .iter()
        .filter(|diagnostic| {
            diagnostic.severity == Severity::Error
                && matches!(
                    diagnostic.kind,
                    DiagnosticKind::NonFiniteCoordinate | DiagnosticKind::CoordinateImportFailed
                )
        })
        .filter_map(|diagnostic| diagnostic.coordinate)
        .collect::<Vec<_>>();
    coordinates.sort_unstable();
    coordinates.dedup();
    coordinates.len()
}

fn expected_checked_index_count(vertex_count: Option<usize>, index_count: usize) -> usize {
    if vertex_count.is_some() {
        index_count / 3 * 3
    } else {
        0
    }
}
