//! Exact affine-frame orthogonal solid cell complexes.
//!
//! [`crate::affine_box`] normalizes two parallelepiped boxes into one
//! exact `(u, v, w)` frame before reusing orthogonal box materializers. This
//! module extends that retained evidence route to the bounded cell-complex case:
//! if a shared frame is recovered from a retained affine box or exact
//! cell-complex edge structure, and both operands replay as axis-aligned
//! orthogonal solid cell complexes in that frame, a named boolean is
//! materialized on the normalized grid and lifted back exactly.
//! The affine basis, normalized source meshes, selected cells, and lifted
//! output are retained computation history, not an approximate fit. The
//! normalized rectangular subdivision is the same grid-arrangement idea

use hyperlimit::{Point3, compare_reals};

use super::affine_box::{AffineBoxBasis, candidate_affine_box_bases, mesh_from_uvw, mesh_to_uvw};
use super::error::{DiagnosticKind, MeshDiagnostic, MeshError, Severity};
use super::mesh::ExactMesh;
use super::orthogonal_solid::{
    AxisAlignedOrthogonalSolidOperation, OrthogonalCellPlan,
    axis_aligned_orthogonal_solid_cell_plan, has_axis_aligned_orthogonal_solid_cells,
    is_axis_aligned_orthogonal_solid, materialize_axis_aligned_orthogonal_solid_cell_plan,
};
use super::validation::ValidationPolicy;
use core::cmp::Ordering;
use hyperreal::Real;

/// Maximum origin vertices sampled when recovering an affine cell frame.
///
/// The limit keeps basis discovery bounded. Soundness does not depend on
/// exhaustiveness because every candidate is accepted only after exact replay
/// proves the complete mesh is an axis-aligned orthogonal solid.
const MAX_CELL_BASIS_ORIGINS: usize = 8;

/// Maximum incident directions sampled per origin during frame discovery.
///
/// Directions are ranked by exact mesh-edge frequency so repeated grid axes
/// enforced after sampling by [`mesh_to_uvw`] plus orthogonal-solid replay, not
/// by this heuristic ordering.
const MAX_CELL_BASIS_DIRECTIONS: usize = 8;

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
/// arrangement. It requires an exact basis derived from retained box or
/// cell-complex structure, exact replay of both source meshes into
/// axis-aligned orthogonal cell complexes, and exact replay of the lifted
/// output back through the same basis.
#[derive(Clone, Debug, PartialEq)]
pub struct AffineOrthogonalSolidArrangement {
    /// Shared affine frame used to normalize source and output cell complexes.
    pub basis: AffineBoxBasis,
    /// Boolean operation that produced the retained mesh.
    pub operation: AffineOrthogonalSolidOperation,
    /// Exact lifted closed output mesh in original 3D space.
    pub mesh: ExactMesh,
}

#[derive(Clone, Debug)]
struct AffineOrthogonalSolidInputs {
    basis: AffineBoxBasis,
    uvw_output_plan: OrthogonalCellPlan,
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
        // Empty intersections are valid regularized solids in the retained
        // decision, but the local output audit must not demand nonempty
        // topology once replay has certified an empty selected cell set.
        if self.mesh.vertices().is_empty() && self.mesh.triangles().is_empty() {
            return Ok(());
        }
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
    affine_orthogonal_solid_operation_is_supported(left, right, operation)
}

fn materialize_affine_orthogonal_solids(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: AffineOrthogonalSolidOperation,
    validation: ValidationPolicy,
) -> Result<Option<AffineOrthogonalSolidArrangement>, MeshError> {
    let Some(inputs) = certify_affine_orthogonal_solid_inputs(left, right, operation) else {
        return Ok(None);
    };
    let uvw_output = materialize_axis_aligned_orthogonal_solid_cell_plan(
        inputs.uvw_output_plan,
        "exact affine-normalized orthogonal solid cell boolean",
        ValidationPolicy::CLOSED,
    )?;
    let mesh = mesh_from_uvw(
        &uvw_output,
        &inputs.basis,
        operation.output_label(),
        validation,
    )?;
    let arrangement = AffineOrthogonalSolidArrangement {
        basis: inputs.basis,
        operation,
        mesh,
    };
    arrangement.validate()?;
    Ok(Some(arrangement))
}

fn certify_affine_orthogonal_solid_inputs(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: AffineOrthogonalSolidOperation,
) -> Option<AffineOrthogonalSolidInputs> {
    if is_axis_aligned_orthogonal_solid(left) && is_axis_aligned_orthogonal_solid(right) {
        return None;
    }
    for basis in candidate_shared_bases(left, right) {
        let Some(left_uvw) = mesh_to_uvw(left, &basis, ValidationPolicy::CLOSED) else {
            continue;
        };
        let Some(right_uvw) = mesh_to_uvw(right, &basis, ValidationPolicy::CLOSED) else {
            continue;
        };
        if let Some(uvw_output_plan) = axis_aligned_orthogonal_solid_cell_plan(
            &left_uvw,
            &right_uvw,
            operation.to_axis_aligned(),
        ) {
            return Some(AffineOrthogonalSolidInputs {
                basis,
                uvw_output_plan,
            });
        }
    }
    None
}

fn affine_orthogonal_solid_operation_is_supported(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: AffineOrthogonalSolidOperation,
) -> bool {
    if is_axis_aligned_orthogonal_solid(left) && is_axis_aligned_orthogonal_solid(right) {
        return false;
    }
    for basis in candidate_shared_bases(left, right) {
        let Some(left_uvw) = mesh_to_uvw(left, &basis, ValidationPolicy::CLOSED) else {
            continue;
        };
        let Some(right_uvw) = mesh_to_uvw(right, &basis, ValidationPolicy::CLOSED) else {
            continue;
        };
        if has_axis_aligned_orthogonal_solid_cells(
            &left_uvw,
            &right_uvw,
            operation.to_axis_aligned(),
        ) {
            return true;
        }
    }
    false
}

/// Return exact affine bases that are plausible for both source meshes.
///
/// Retained box evidence is tried first because it is already a construction
/// artifact. Cell-complex evidence is secondary and must be rediscovered from
/// exact edge structure before the pair replay below can certify it.
fn candidate_shared_bases(left: &ExactMesh, right: &ExactMesh) -> Vec<AffineBoxBasis> {
    let mut bases = candidate_affine_box_bases(left);
    bases.extend(candidate_affine_box_bases(right));
    bases.extend(candidate_affine_cell_bases(left));
    bases.extend(candidate_affine_cell_bases(right));
    let mut unique = Vec::new();
    for basis in bases {
        if compare_reals(&basis.determinant(), &Real::from(0)).value() == Some(Ordering::Equal)
            || unique.contains(&basis)
        {
            continue;
        }
        unique.push(basis);
    }
    unique
}

/// Recover exact affine bases from a single orthogonal-solid cell complex.
///
/// The search uses the mesh's retained triangle-edge graph to propose three
/// frame is not trusted as a numeric fit: it becomes evidence only if exact
/// determinant-ratio normalization and orthogonal-solid replay accept the full
/// source mesh.
fn candidate_affine_cell_bases(mesh: &ExactMesh) -> Vec<AffineBoxBasis> {
    let mut bases = Vec::new();
    if mesh.vertices().len() < 8 || mesh.triangles().len() < 12 {
        return bases;
    }
    let adjacency = vertex_adjacency(mesh);
    let direction_counts = mesh_direction_counts(mesh);
    let mut origins = (0..adjacency.len()).collect::<Vec<_>>();
    origins.sort_by_key(|&origin| adjacency[origin].len());
    for origin in origins.into_iter().take(MAX_CELL_BASIS_ORIGINS) {
        let neighbors = &adjacency[origin];
        let origin_point = mesh.vertices()[origin].clone();
        let mut directions = unique_edge_directions(mesh, origin, neighbors);
        directions.sort_by_key(|direction| {
            core::cmp::Reverse(direction_weight(direction, &direction_counts))
        });
        directions.truncate(MAX_CELL_BASIS_DIRECTIONS);
        for u in 0..directions.len() {
            for v in u + 1..directions.len() {
                for w in v + 1..directions.len() {
                    let basis = AffineBoxBasis {
                        origin: origin_point.clone(),
                        basis_u: directions[u].clone(),
                        basis_v: directions[v].clone(),
                        basis_w: directions[w].clone(),
                    };
                    if compare_reals(&basis.determinant(), &Real::from(0)).value()
                        == Some(Ordering::Equal)
                        || bases.contains(&basis)
                    {
                        continue;
                    }
                    // infer from retained combinatorial edges, then accept
                    // only through exact replay, never through angle
                    // tolerances or floating-point least-squares fitting.
                    if mesh_to_uvw(mesh, &basis, ValidationPolicy::CLOSED)
                        .as_ref()
                        .is_some_and(is_axis_aligned_orthogonal_solid)
                    {
                        bases.push(basis);
                    }
                }
            }
        }
    }
    bases
}

/// Count undirected exact triangle-edge directions in mesh space.
fn mesh_direction_counts(mesh: &ExactMesh) -> Vec<(Point3, usize)> {
    let mut counts = Vec::<(Point3, usize)>::new();
    for triangle in mesh.triangles() {
        let [a, b, c] = triangle.0;
        count_edge_direction(mesh, a, b, &mut counts);
        count_edge_direction(mesh, b, c, &mut counts);
        count_edge_direction(mesh, c, a, &mut counts);
    }
    counts
}

/// Add one exact undirected edge direction to the frequency table.
fn count_edge_direction(mesh: &ExactMesh, a: usize, b: usize, counts: &mut Vec<(Point3, usize)>) {
    let (Some(a), Some(b)) = (mesh.vertices().get(a), mesh.vertices().get(b)) else {
        return;
    };
    let direction = sub3(&b.clone(), &a.clone());
    if point_is_zero(&direction) {
        return;
    }
    if let Some((_, count)) = counts
        .iter_mut()
        .find(|(seen, _)| points_equal_or_opposite(seen, &direction))
    {
        *count += 1;
    } else {
        counts.push((direction, 1));
    }
}

/// Return the frequency assigned to an exact direction, ignoring sign.
fn direction_weight(direction: &Point3, counts: &[(Point3, usize)]) -> usize {
    counts
        .iter()
        .find(|(seen, _)| points_equal_or_opposite(seen, direction))
        .map(|(_, count)| *count)
        .unwrap_or(0)
}

/// Build a unique undirected vertex adjacency list from retained triangles.
fn vertex_adjacency(mesh: &ExactMesh) -> Vec<Vec<usize>> {
    let mut adjacency = vec![Vec::new(); mesh.vertices().len()];
    for triangle in mesh.triangles() {
        let [a, b, c] = triangle.0;
        push_edge(&mut adjacency, a, b);
        push_edge(&mut adjacency, b, c);
        push_edge(&mut adjacency, c, a);
    }
    adjacency
}

/// Insert one undirected adjacency edge if both endpoint rows exist.
fn push_edge(adjacency: &mut [Vec<usize>], a: usize, b: usize) {
    if let Some(neighbors) = adjacency.get_mut(a)
        && !neighbors.contains(&b)
    {
        neighbors.push(b);
    }
    if let Some(neighbors) = adjacency.get_mut(b)
        && !neighbors.contains(&a)
    {
        neighbors.push(a);
    }
}

/// Return unique outgoing exact edge directions at one origin vertex.
fn unique_edge_directions(mesh: &ExactMesh, origin: usize, neighbors: &[usize]) -> Vec<Point3> {
    let origin_point = mesh.vertices()[origin].clone();
    let mut directions = Vec::new();
    for &neighbor in neighbors {
        let Some(neighbor) = mesh.vertices().get(neighbor) else {
            continue;
        };
        let direction = sub3(&neighbor.clone(), &origin_point);
        if point_is_zero(&direction) || directions.iter().any(|seen| points_equal(seen, &direction))
        {
            continue;
        }
        directions.push(direction);
    }
    directions
}

/// Subtract exact 3D points componentwise.
fn sub3(left: &Point3, right: &Point3) -> Point3 {
    Point3::new(
        left.x.clone() - &right.x,
        left.y.clone() - &right.y,
        left.z.clone() - &right.z,
    )
}

/// Return whether an exact point/vector is exactly zero.
fn point_is_zero(point: &Point3) -> bool {
    compare_reals(&point.x, &Real::from(0)).value() == Some(Ordering::Equal)
        && compare_reals(&point.y, &Real::from(0)).value() == Some(Ordering::Equal)
        && compare_reals(&point.z, &Real::from(0)).value() == Some(Ordering::Equal)
}

/// Compare exact points componentwise.
fn points_equal(left: &Point3, right: &Point3) -> bool {
    compare_reals(&left.x, &right.x).value() == Some(Ordering::Equal)
        && compare_reals(&left.y, &right.y).value() == Some(Ordering::Equal)
        && compare_reals(&left.z, &right.z).value() == Some(Ordering::Equal)
}

/// Compare exact directions up to sign.
fn points_equal_or_opposite(left: &Point3, right: &Point3) -> bool {
    points_equal(left, right)
        || (compare_reals(&left.x, &(-right.x.clone())).value() == Some(Ordering::Equal)
            && compare_reals(&left.y, &(-right.y.clone())).value() == Some(Ordering::Equal)
            && compare_reals(&left.z, &(-right.z.clone())).value() == Some(Ordering::Equal))
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
