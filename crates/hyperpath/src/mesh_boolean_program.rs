//! Replayable exact mesh-boolean programs over retained path sources.
//!
//! A homogeneous chain is enough for repeated union or repeated cutter
//! subtraction, but CAM/EDA workflows often alternate operations: stock can be
//! trimmed by an envelope, cut by swept tool paths, and then unioned with
//! retained support/keepout solids. This module records that sequence as a
//! path-domain program. Each step lowers a retained [`PathMeshBooleanSource`]
//! to an exact `hypermesh` operand and stores the exact boolean evidence needed
//! to replay the accepted accumulator.
//!
//! The design follows Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7.1-2 (1997): the source objects and exact
//! predicates remain canonical, while cached mesh topology is accepted only
//! while it can be reproduced from those objects. The solid-operation semantics
//! are the regularized set operations described by Requicha, "Representations
//! for Rigid Solids: Theory, Methods, and Systems," *ACM Computing Surveys*
//! 12.4 (1980).

use hypermesh::exact::{
    ExactBooleanPreflight, ExactBooleanResult, ExactBoundaryBooleanPolicy, ExactMesh,
    ValidationPolicy, boolean_exact_with_boundary_policy, preflight_boolean_exact,
};

use crate::mesh_boolean::{PathMeshBooleanError, PathMeshBooleanOperation};
use crate::mesh_boolean_sources::PathMeshBooleanSource;

/// One requested operation in a retained path-source boolean program.
///
/// The left operand is always the current accumulator: the initial program
/// source for step zero, and the accepted mesh result from the previous step
/// after that. The right operand remains a path/CAM source object, so replay
/// never depends on caller-retained mesh topology.
#[derive(Clone, Debug, PartialEq)]
pub struct PathMeshBooleanProgramStep {
    /// Named regularized operation to apply at this step.
    pub operation: PathMeshBooleanOperation,
    /// Right-hand retained path source.
    pub right: PathMeshBooleanSource,
    /// Boundary-contact projection policy for this exact `hypermesh` call.
    pub boundary_policy: ExactBoundaryBooleanPolicy,
}

/// Accepted evidence for one operation in a retained boolean program.
#[derive(Clone, Debug, PartialEq)]
pub struct PathMeshBooleanProgramStepReport {
    /// Zero-based program step index.
    pub index: usize,
    /// Named regularized operation replayed for this step.
    pub operation: PathMeshBooleanOperation,
    /// Right-hand retained path source consumed by this step.
    pub right: PathMeshBooleanSource,
    /// Boundary-contact projection policy replayed for this step.
    pub boundary_policy: ExactBoundaryBooleanPolicy,
    /// Exact preflight report for this accumulator/right pair.
    pub preflight: ExactBooleanPreflight,
    /// Accepted exact boolean result for this step.
    pub result: ExactBooleanResult,
}

/// Source-bound exact boolean program with per-step operations.
///
/// Unlike [`crate::mesh_boolean_sources::PathMeshBooleanSourceChainReport`],
/// each step carries its own operation and boundary policy. The report is a
/// proof log for the exact accumulator, not a claim that operations commute or
/// reassociate.
#[derive(Clone, Debug, PartialEq)]
pub struct PathMeshBooleanProgramReport {
    /// Retained initial path/CAM source that seeds the accumulator.
    pub initial: PathMeshBooleanSource,
    /// Accepted per-step exact preflight and mesh materialization evidence.
    pub steps: Vec<PathMeshBooleanProgramStepReport>,
}

impl PathMeshBooleanProgramStep {
    /// Build a step using the default boundary policy.
    pub fn new(operation: PathMeshBooleanOperation, right: PathMeshBooleanSource) -> Self {
        Self::with_boundary_policy(operation, right, ExactBoundaryBooleanPolicy::Reject)
    }

    /// Build a step with explicit boundary-contact projection policy.
    pub const fn with_boundary_policy(
        operation: PathMeshBooleanOperation,
        right: PathMeshBooleanSource,
        boundary_policy: ExactBoundaryBooleanPolicy,
    ) -> Self {
        Self {
            operation,
            right,
            boundary_policy,
        }
    }
}

impl PathMeshBooleanProgramReport {
    /// Rebuild and validate every program step from the retained path sources.
    pub fn validate_replay(&self) -> Result<(), PathMeshBooleanError> {
        if self.steps.is_empty() {
            return Err(PathMeshBooleanError::NotEnoughSources);
        }
        let mut accumulator = self.initial.to_exact_mesh()?;
        for (expected_index, step) in self.steps.iter().enumerate() {
            if step.index != expected_index {
                return Err(PathMeshBooleanError::Replay(
                    "retained boolean-program step index no longer matches order".into(),
                ));
            }
            let operation = step.operation.to_hypermesh();
            let right_mesh = step.right.to_exact_mesh()?;
            let preflight = preflight_boolean_exact(&accumulator, &right_mesh, operation)
                .map_err(|error| PathMeshBooleanError::Preflight(format!("{error:?}")))?;
            preflight
                .validate_against_sources(&accumulator, &right_mesh)
                .map_err(|error| PathMeshBooleanError::Replay(format!("{error:?}")))?;
            if preflight != step.preflight {
                return Err(PathMeshBooleanError::Replay(
                    "retained boolean-program preflight no longer matches replay".into(),
                ));
            }
            step.result
                .validate_operation_against_sources(
                    &accumulator,
                    &right_mesh,
                    operation,
                    ValidationPolicy::CLOSED,
                    step.boundary_policy,
                )
                .map_err(|error| PathMeshBooleanError::Replay(format!("{error:?}")))?;
            accumulator = step.result.mesh.clone();
        }
        Ok(())
    }

    /// Return the final accepted exact output mesh.
    pub fn mesh(&self) -> Option<&ExactMesh> {
        self.steps.last().map(|step| &step.result.mesh)
    }
}

/// Run an exact boolean program over retained path-domain sources.
pub fn boolean_path_mesh_program(
    initial: PathMeshBooleanSource,
    steps: Vec<PathMeshBooleanProgramStep>,
) -> Result<PathMeshBooleanProgramReport, PathMeshBooleanError> {
    if steps.is_empty() {
        return Err(PathMeshBooleanError::NotEnoughSources);
    }
    let mut accumulator = initial.to_exact_mesh()?;
    let mut reports = Vec::with_capacity(steps.len());
    for (index, step) in steps.into_iter().enumerate() {
        let operation = step.operation.to_hypermesh();
        let right_mesh = step.right.to_exact_mesh()?;
        let preflight = preflight_boolean_exact(&accumulator, &right_mesh, operation)
            .map_err(|error| PathMeshBooleanError::Preflight(format!("{error:?}")))?;
        preflight
            .validate_against_sources(&accumulator, &right_mesh)
            .map_err(|error| PathMeshBooleanError::Replay(format!("{error:?}")))?;
        let result = boolean_exact_with_boundary_policy(
            &accumulator,
            &right_mesh,
            operation,
            ValidationPolicy::CLOSED,
            step.boundary_policy,
        )
        .map_err(|error| PathMeshBooleanError::Boolean(format!("{error:?}")))?;
        result
            .validate_operation_against_sources(
                &accumulator,
                &right_mesh,
                operation,
                ValidationPolicy::CLOSED,
                step.boundary_policy,
            )
            .map_err(|error| PathMeshBooleanError::Replay(format!("{error:?}")))?;
        accumulator = result.mesh.clone();
        reports.push(PathMeshBooleanProgramStepReport {
            index,
            operation: step.operation,
            right: step.right,
            boundary_policy: step.boundary_policy,
            preflight,
            result,
        });
    }

    Ok(PathMeshBooleanProgramReport {
        initial,
        steps: reports,
    })
}
