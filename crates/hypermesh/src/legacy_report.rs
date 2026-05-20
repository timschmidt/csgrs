//--- Copyright (C) 2025 Saki Komikado <komietty@gmail.com>,
//--- This Source Code Form is subject to the terms of the Mozilla Public License v.2.0.

//! Auditable reports for the gated legacy boolean adapter.
//!
//! The types in this module are intentionally not exact certificates. They
//! record that the boolmesh-derived path crossed a primitive-float,
//! epsilon-bearing topology boundary and give callers a replayable envelope for
//! the input/output mesh counts and selected tolerance. Yap, "Towards Exact
//! Geometric Computation," *Computational Geometry* 7.1-2 (1997), separates
//! exact geometric decisions from approximate edge adapters; this module keeps
//! that distinction visible by requiring a report at the legacy compatibility
//! entry point.

use crate::{Manifold, OpType, Real};

/// Report for the gated legacy boolean adapter.
#[derive(Clone, Debug)]
pub struct LegacyBooleanReport {
    /// Requested legacy boolean operation.
    pub operation: OpType,
    /// Number of vertices in the left input manifold.
    pub left_vertices: usize,
    /// Number of faces in the left input manifold.
    pub left_faces: usize,
    /// Number of vertices in the right input manifold.
    pub right_vertices: usize,
    /// Number of faces in the right input manifold.
    pub right_faces: usize,
    /// Number of vertices in the output manifold.
    pub output_vertices: usize,
    /// Number of faces in the output manifold.
    pub output_faces: usize,
    /// Epsilon selected from the two input manifolds.
    pub epsilon: Real,
    /// Tolerance selected from the two input manifolds.
    pub tolerance: Real,
    /// Whether this report used the legacy primitive-float topology adapter.
    pub used_primitive_float_adapter: bool,
}

/// Result object for [`crate::compute_boolean_with_report`].
#[derive(Clone, Debug)]
pub struct LegacyBooleanResult {
    /// Mesh produced by the compatibility adapter.
    pub mesh: Manifold,
    /// Auditable adapter boundary report.
    pub report: LegacyBooleanReport,
}

/// Validation failure for a legacy adapter report.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LegacyBooleanReportError {
    /// The report did not identify itself as the primitive-float adapter.
    MissingAdapterFlag,
    /// The selected epsilon or tolerance is not finite.
    NonFiniteTolerance,
    /// The retained operation no longer matches the requested operation.
    OperationMismatch,
    /// Retained tolerance values no longer replay from the input manifolds.
    ToleranceMismatch,
    /// The report carries no input or output topology evidence.
    EmptyAdapterEvidence,
    /// Retained input counts no longer match the supplied source manifolds.
    InputCountMismatch,
    /// Retained output counts no longer match the supplied output manifold.
    OutputCountMismatch,
    /// The retained output is no longer a valid legacy manifold.
    OutputNotManifold,
}

impl LegacyBooleanReport {
    /// Validate this report against the source and output manifolds.
    ///
    /// The legacy adapter cannot provide exact predicate certificates. This
    /// method therefore validates only the adapter boundary contract: the
    /// report must be explicitly marked as approximate, its chosen tolerance
    /// values must be finite, and its retained mesh counts must match the
    /// manifolds that crossed the boundary. Keeping even this compatibility
    /// path report-bearing follows Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7.1-2 (1997): uncertified topology must remain
    /// explicit program state rather than masquerade as exact output.
    pub fn validate_against_manifolds(
        &self,
        left: &Manifold,
        right: &Manifold,
        output: &Manifold,
    ) -> Result<(), LegacyBooleanReportError> {
        if !self.used_primitive_float_adapter {
            return Err(LegacyBooleanReportError::MissingAdapterFlag);
        }
        if !self.epsilon.is_finite() || !self.tolerance.is_finite() {
            return Err(LegacyBooleanReportError::NonFiniteTolerance);
        }
        if self.epsilon != left.eps.max(right.eps) || self.tolerance != left.tol.max(right.tol) {
            return Err(LegacyBooleanReportError::ToleranceMismatch);
        }
        if self.left_vertices == 0
            || self.left_faces == 0
            || self.right_vertices == 0
            || self.right_faces == 0
            || self.output_vertices == 0
            || self.output_faces == 0
        {
            return Err(LegacyBooleanReportError::EmptyAdapterEvidence);
        }
        if self.left_vertices != left.nv
            || self.left_faces != left.nf
            || self.right_vertices != right.nv
            || self.right_faces != right.nf
        {
            return Err(LegacyBooleanReportError::InputCountMismatch);
        }
        if self.output_vertices != output.nv || self.output_faces != output.nf {
            return Err(LegacyBooleanReportError::OutputCountMismatch);
        }
        if !output.is_manifold() {
            return Err(LegacyBooleanReportError::OutputNotManifold);
        }
        Ok(())
    }

    /// Validate this report against source/output manifolds and the requested
    /// operation.
    ///
    /// Count replay alone is not enough for an approximate compatibility path:
    /// a copied report with the wrong operation would make a primitive-float
    /// topology result look like evidence for a different boolean. Keeping the
    /// requested operation in the replay contract follows Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7.1-2 (1997), by
    /// preventing adapter metadata from being treated as interchangeable exact
    /// state.
    pub fn validate_operation_against_manifolds(
        &self,
        left: &Manifold,
        right: &Manifold,
        output: &Manifold,
        operation: OpType,
    ) -> Result<(), LegacyBooleanReportError> {
        self.validate_against_manifolds(left, right, output)?;
        if self.operation != operation {
            return Err(LegacyBooleanReportError::OperationMismatch);
        }
        Ok(())
    }
}

impl LegacyBooleanResult {
    /// Validate the retained report against the original input manifolds.
    pub fn validate_against_inputs(
        &self,
        left: &Manifold,
        right: &Manifold,
    ) -> Result<(), LegacyBooleanReportError> {
        self.report
            .validate_against_manifolds(left, right, &self.mesh)
    }

    /// Validate the retained report against inputs and the requested operation.
    pub fn validate_operation_against_inputs(
        &self,
        left: &Manifold,
        right: &Manifold,
        operation: OpType,
    ) -> Result<(), LegacyBooleanReportError> {
        self.report
            .validate_operation_against_manifolds(left, right, &self.mesh, operation)
    }
}
