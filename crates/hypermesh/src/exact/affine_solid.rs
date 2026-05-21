//! Exact affine-frame orthogonal solid cell complexes.
//!
//! [`crate::exact::affine_box`] normalizes two parallelepiped boxes into one
//! exact `(u, v, w)` frame before reusing orthogonal box materializers. This
//! module extends that retained-object route to the bounded cell-complex case:
//! if one operand supplies an exact affine box frame and both operands replay
//! as axis-aligned orthogonal solid cell complexes in that frame, a named
//! boolean is materialized on the normalized grid and lifted back exactly.
//!
//! The key policy is Yap's model from "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7.1-2 (1997): the affine basis, normalized source
//! meshes, selected cells, and lifted output are retained computation history,
//! not an approximate fit. The normalized rectangular subdivision is the same
//! grid-arrangement idea described by de Berg, Cheong, van Kreveld, and
//! Overmars, *Computational Geometry: Algorithms and Applications*, 3rd ed.
//! (2008), Chapter 2, but every coordinate replay here is exact.

use hyperlimit::compare_reals;

use super::affine_box::{AffineBoxBasis, candidate_affine_box_bases, mesh_from_uvw, mesh_to_uvw};
use super::error::{DiagnosticKind, MeshDiagnostic, MeshError, Severity};
use super::mesh::ExactMesh;
use super::orthogonal_solid::{
    AxisAlignedOrthogonalSolidOperation, is_axis_aligned_orthogonal_solid,
    materialize_axis_aligned_orthogonal_solid_cells,
};
use super::scalar::ExactReal;
use super::validation::ValidationPolicy;
use core::cmp::Ordering;

/// Named operation retained by an affine orthogonal-solid materialization.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AffineOrthogonalSolidOperation {
    /// Regularized solid union.
    Union,
    /// Positive-volume solid intersection.
    Intersection,
    /// Regularized solid difference `left - right`.
    Difference,
}

/// Exact affine orthogonal-solid boolean output.
///
/// This artifact is deliberately narrower than a general affine volumetric
/// arrangement. It requires an exact box-derived basis, exact replay of both
/// source meshes into axis-aligned orthogonal cell complexes, and exact replay
/// of the lifted output back through the same basis.
#[derive(Clone, Debug, PartialEq)]
pub struct AffineOrthogonalSolidArrangement {
    /// Shared affine frame used to normalize source and output cell complexes.
    pub basis: AffineBoxBasis,
    /// Boolean operation that produced the retained mesh.
    pub operation: AffineOrthogonalSolidOperation,
    /// Exact lifted closed output mesh in original 3D space.
    pub mesh: ExactMesh,
}

impl AffineOrthogonalSolidArrangement {
    /// Validate local mesh state and affine normalized-cell replay.
    ///
    /// This does not inspect the original operands. It checks that the output
    /// mesh remains valid and that every lifted vertex maps back to an exact
    /// axis-aligned orthogonal solid cell complex in the retained frame. Source
    /// replay is handled by [`Self::validate_against_sources`].
    pub fn validate(&self) -> Result<(), MeshError> {
        self.mesh.validate_retained_state().map_err(|error| {
            affine_solid_error(format!(
                "affine orthogonal solid output mesh is stale: {error:?}"
            ))
        })?;
        let normalized = mesh_to_uvw(&self.mesh, &self.basis, self.mesh.validation_policy())
            .ok_or_else(|| {
                affine_solid_error("affine orthogonal solid output does not replay through basis")
            })?;
        if !is_axis_aligned_orthogonal_solid(&normalized) {
            return Err(affine_solid_error(
                "affine orthogonal solid output is not a normalized cell complex",
            ));
        }
        Ok(())
    }

    /// Validate this output by replaying the retained operation from sources.
    ///
    /// The replay recomputes basis discovery, exact normalized source meshes,
    /// normalized orthogonal cell materialization, and lifted output. That
    /// keeps the affine frame and source operands in the certificate chain,
    /// following Yap's exact-object requirement instead of trusting the output
    /// triangle soup by itself.
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), MeshError> {
        self.validate()?;
        let replay = materialize_affine_orthogonal_solids(
            left,
            right,
            self.operation,
            self.mesh.validation_policy(),
        )?
        .ok_or_else(|| {
            affine_solid_error("source replay did not reproduce affine orthogonal solid output")
        })?;
        if self == &replay {
            Ok(())
        } else {
            Err(affine_solid_error(
                "retained affine orthogonal solid output does not match source replay",
            ))
        }
    }
}

/// Certify and materialize an affine orthogonal-solid union.
pub fn materialize_affine_orthogonal_solid_union(
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ValidationPolicy,
) -> Result<Option<AffineOrthogonalSolidArrangement>, MeshError> {
    materialize_affine_orthogonal_solids(
        left,
        right,
        AffineOrthogonalSolidOperation::Union,
        validation,
    )
}

/// Certify and materialize an affine orthogonal-solid intersection.
pub fn materialize_affine_orthogonal_solid_intersection(
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ValidationPolicy,
) -> Result<Option<AffineOrthogonalSolidArrangement>, MeshError> {
    materialize_affine_orthogonal_solids(
        left,
        right,
        AffineOrthogonalSolidOperation::Intersection,
        validation,
    )
}

/// Certify and materialize an affine orthogonal-solid difference.
pub fn materialize_affine_orthogonal_solid_difference(
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ValidationPolicy,
) -> Result<Option<AffineOrthogonalSolidArrangement>, MeshError> {
    materialize_affine_orthogonal_solids(
        left,
        right,
        AffineOrthogonalSolidOperation::Difference,
        validation,
    )
}

/// Return whether an affine orthogonal-solid operation is certified.
pub(crate) fn has_affine_orthogonal_solid_cells(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: AffineOrthogonalSolidOperation,
) -> bool {
    materialize_affine_orthogonal_solids(left, right, operation, ValidationPolicy::CLOSED)
        .map(|arrangement| arrangement.is_some())
        .unwrap_or(false)
}

fn materialize_affine_orthogonal_solids(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: AffineOrthogonalSolidOperation,
    validation: ValidationPolicy,
) -> Result<Option<AffineOrthogonalSolidArrangement>, MeshError> {
    if is_axis_aligned_orthogonal_solid(left) && is_axis_aligned_orthogonal_solid(right) {
        return Ok(None);
    }
    let candidates = candidate_shared_bases(left, right);
    for basis in candidates {
        let Some(left_uvw) = mesh_to_uvw(left, &basis, ValidationPolicy::CLOSED) else {
            continue;
        };
        let Some(right_uvw) = mesh_to_uvw(right, &basis, ValidationPolicy::CLOSED) else {
            continue;
        };
        let Some(uvw_output) = materialize_axis_aligned_orthogonal_solid_cells(
            &left_uvw,
            &right_uvw,
            operation.to_axis_aligned(),
            "exact affine-normalized orthogonal solid cell boolean",
            ValidationPolicy::CLOSED,
        )?
        else {
            continue;
        };
        let mesh = mesh_from_uvw(&uvw_output, &basis, operation.output_label(), validation)?;
        let arrangement = AffineOrthogonalSolidArrangement {
            basis,
            operation,
            mesh,
        };
        arrangement.validate()?;
        return Ok(Some(arrangement));
    }
    Ok(None)
}

fn candidate_shared_bases(left: &ExactMesh, right: &ExactMesh) -> Vec<AffineBoxBasis> {
    let mut bases = candidate_affine_box_bases(left);
    bases.extend(candidate_affine_box_bases(right));
    bases
        .into_iter()
        .filter(|basis| {
            compare_reals(&basis.determinant(), &ExactReal::from(0)).value()
                != Some(Ordering::Equal)
        })
        .collect()
}

impl AffineOrthogonalSolidOperation {
    const fn to_axis_aligned(self) -> AxisAlignedOrthogonalSolidOperation {
        match self {
            Self::Union => AxisAlignedOrthogonalSolidOperation::Union,
            Self::Intersection => AxisAlignedOrthogonalSolidOperation::Intersection,
            Self::Difference => AxisAlignedOrthogonalSolidOperation::Difference,
        }
    }

    const fn output_label(self) -> &'static str {
        match self {
            Self::Union => "exact affine orthogonal solid cell union",
            Self::Intersection => "exact affine orthogonal solid cell intersection",
            Self::Difference => "exact affine orthogonal solid cell difference",
        }
    }
}

fn affine_solid_error(message: impl Into<String>) -> MeshError {
    MeshError::one(MeshDiagnostic::new(
        Severity::Error,
        DiagnosticKind::UnsupportedExactOperation,
        message,
    ))
}
