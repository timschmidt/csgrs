//! Exact affine-frame orthogonal solid cell complexes.
//!
//! This module normalizes two parallelepiped or affine orthogonal-cell meshes
//! into one exact `(u, v, w)` frame before reusing orthogonal box/cell
//! materializers. If a shared frame is recovered from exact cell-complex edge
//! structure, and both operands replay as axis-aligned orthogonal solid cell
//! complexes in that frame, a named boolean is materialized on the normalized
//! grid and lifted back exactly.
//! The affine basis, normalized source meshes, selected cells, and lifted
//! output are retained computation history, not an approximate fit. The
//! normalized rectangular subdivision is the same grid-arrangement idea

use hyperlimit::{Point3, compare_reals};

use super::super::error::{MeshBlocker, MeshBlockerKind, MeshError};
use super::super::validation::MeshValidationMode;
use super::super::{Mesh, Triangle, reverse_triangle};
use super::orthogonal_solid::{
    AxisAlignedOrthogonalSolidOperation, axis_aligned_orthogonal_solid_cell_plan,
    axis_aligned_orthogonal_solid_cell_selected_count, is_axis_aligned_orthogonal_solid,
};
use super::point3_exact_equal;
use core::cmp::Ordering;
use hyperlimit::SourceProvenance;
use hyperreal::Real;

/// Exact 3D affine frame for normalized box coordinates.
///
/// A normalized point `(u, v, w)` is interpreted as
/// `origin + u * basis_u + v * basis_v + w * basis_w`. The frame is part of
/// the certificate: every source and output vertex must replay through it
/// exactly before a copied boolean artifact is accepted.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct AffineBoxBasis {
    /// Exact 3D affine origin.
    pub(crate) origin: Point3,
    /// Exact vector for the normalized `u` axis.
    pub(crate) basis_u: Point3,
    /// Exact vector for the normalized `v` axis.
    pub(crate) basis_v: Point3,
    /// Exact vector for the normalized `w` axis.
    pub(crate) basis_w: Point3,
}

/// Named operation retained by an affine orthogonal-solid materialization.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum AffineOrthogonalSolidOperation {
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
pub(crate) struct AffineOrthogonalSolidArrangement {
    /// Shared affine frame used to normalize source and output cell complexes.
    pub basis: AffineBoxBasis,
    /// Boolean operation that produced the retained mesh.
    pub operation: AffineOrthogonalSolidOperation,
    /// Exact lifted closed output mesh in original 3D space.
    pub mesh: Mesh,
}

impl AffineOrthogonalSolidArrangement {
    /// Validate local mesh state and affine normalized-cell replay.
    ///
    /// This does not inspect the original operands. It checks that the output
    /// mesh remains valid and that every lifted vertex maps back to an exact
    /// axis-aligned orthogonal solid cell complex in the retained frame. Source
    /// replay is handled by the retained boolean result evidence.
    pub fn validate(&self) -> Result<(), MeshError> {
        self.mesh.validate_retained_state().map_err(|error| {
            MeshError::one(MeshBlocker::new(
                MeshBlockerKind::StaleFactReplay,
                format!("affine orthogonal solid output mesh is stale: {error:?}"),
            ))
        })?;
        // Empty intersections are valid regularized solids in the retained
        // decision, but the local output audit must not demand nonempty
        // topology once replay has certified an empty selected cell set.
        if self.mesh.vertices().is_empty() && self.mesh.facts().mesh.face_count == 0 {
            return Ok(());
        }
        let normalized = mesh_to_uvw(&self.mesh, &self.basis, self.mesh.validation_mode())?
            .ok_or_else(|| {
                MeshError::one(MeshBlocker::new(
                    MeshBlockerKind::UnsupportedExactOperation,
                    "affine orthogonal solid output does not replay through basis",
                ))
            })?;
        if !is_axis_aligned_orthogonal_solid(&normalized) {
            return Err(MeshError::one(MeshBlocker::new(
                MeshBlockerKind::UnsupportedExactOperation,
                "affine orthogonal solid output is not a normalized cell complex",
            )));
        }
        Ok(())
    }

    pub(crate) fn validate_against_sources(
        &self,
        left: &Mesh,
        right: &Mesh,
    ) -> Result<(), MeshError> {
        self.validate()?;
        let replay = materialize_affine_orthogonal_solid_operation(
            left,
            right,
            self.operation,
            self.mesh.validation_mode(),
        )?
        .ok_or_else(|| {
            MeshError::one(MeshBlocker::new(
                MeshBlockerKind::StaleFactReplay,
                "source replay did not reproduce affine orthogonal solid output",
            ))
        })?;
        if self == &replay {
            Ok(())
        } else {
            Err(MeshError::one(MeshBlocker::new(
                MeshBlockerKind::StaleFactReplay,
                "retained affine orthogonal solid output does not match source replay",
            )))
        }
    }
}

/// Return the exact count of selected cells for a certified affine-normalized
/// orthogonal operation.
pub(crate) fn affine_orthogonal_solid_cell_selected_count(
    left: &Mesh,
    right: &Mesh,
    operation: AffineOrthogonalSolidOperation,
) -> Option<usize> {
    if let Some((_basis, selected_count)) =
        find_affine_orthogonal_solid_basis(left, right, |left_uvw, right_uvw| {
            axis_aligned_orthogonal_solid_cell_selected_count(
                &left_uvw,
                &right_uvw,
                match operation {
                    AffineOrthogonalSolidOperation::Union => {
                        AxisAlignedOrthogonalSolidOperation::Union
                    }
                    AffineOrthogonalSolidOperation::Intersection => {
                        AxisAlignedOrthogonalSolidOperation::Intersection
                    }
                    AffineOrthogonalSolidOperation::Difference => {
                        AxisAlignedOrthogonalSolidOperation::Difference
                    }
                },
            )
        })
        .ok()
        .flatten()
    {
        Some(selected_count)
    } else {
        None
    }
}

/// Certify and materialize one affine orthogonal-solid operation.
pub(crate) fn materialize_affine_orthogonal_solid_operation(
    left: &Mesh,
    right: &Mesh,
    operation: AffineOrthogonalSolidOperation,
    validation: MeshValidationMode,
) -> Result<Option<AffineOrthogonalSolidArrangement>, MeshError> {
    let Some((basis, uvw_output_plan)) =
        find_affine_orthogonal_solid_basis(left, right, |left_uvw, right_uvw| {
            axis_aligned_orthogonal_solid_cell_plan(
                &left_uvw,
                &right_uvw,
                match operation {
                    AffineOrthogonalSolidOperation::Union => {
                        AxisAlignedOrthogonalSolidOperation::Union
                    }
                    AffineOrthogonalSolidOperation::Intersection => {
                        AxisAlignedOrthogonalSolidOperation::Intersection
                    }
                    AffineOrthogonalSolidOperation::Difference => {
                        AxisAlignedOrthogonalSolidOperation::Difference
                    }
                },
            )
        })?
    else {
        return Ok(None);
    };
    let uvw_output = uvw_output_plan.to_mesh(
        "exact affine-normalized orthogonal solid cell boolean",
        MeshValidationMode::CLOSED,
    )?;
    let vertices = uvw_output
        .vertices()
        .iter()
        .map(|point| {
            let lifted = point_from_uvw(&point.x, &point.y, &point.z, &basis);
            Point3::new(lifted.x, lifted.y, lifted.z)
        })
        .collect::<Vec<_>>();
    let triangles = if compare_reals(
        &det3(&basis.basis_u, &basis.basis_v, &basis.basis_w),
        &Real::from(0),
    )
    .value()
        == Some(Ordering::Less)
    {
        uvw_output
            .view()
            .faces()
            .map(|face| reverse_triangle(&Triangle(face.vertex_indices())))
            .collect()
    } else {
        uvw_output
            .view()
            .faces()
            .map(|face| Triangle(face.vertex_indices()))
            .collect()
    };
    let mesh = Mesh::new_with_validation_mode_and_version(
        vertices,
        triangles,
        SourceProvenance::exact(match operation {
            AffineOrthogonalSolidOperation::Union => "exact affine orthogonal solid cell union",
            AffineOrthogonalSolidOperation::Intersection => {
                "exact affine orthogonal solid cell intersection"
            }
            AffineOrthogonalSolidOperation::Difference => {
                "exact affine orthogonal solid cell difference"
            }
        }),
        validation,
        1,
    )?;
    let arrangement = AffineOrthogonalSolidArrangement {
        basis,
        operation,
        mesh,
    };
    arrangement.validate()?;
    Ok(Some(arrangement))
}

fn find_affine_orthogonal_solid_basis<T>(
    left: &Mesh,
    right: &Mesh,
    mut accept: impl FnMut(Mesh, Mesh) -> Option<T>,
) -> Result<Option<(AffineBoxBasis, T)>, MeshError> {
    if is_axis_aligned_orthogonal_solid(left) && is_axis_aligned_orthogonal_solid(right) {
        return Ok(None);
    }
    let mut seen = Vec::new();
    let blocker = std::cell::RefCell::new(None);
    let mut accept_basis = |basis: AffineBoxBasis| -> Option<(AffineBoxBasis, T)> {
        if blocker.borrow().is_some() {
            return None;
        }
        if compare_reals(
            &det3(&basis.basis_u, &basis.basis_v, &basis.basis_w),
            &Real::from(0),
        )
        .value()
            == Some(Ordering::Equal)
            || seen.contains(&basis)
        {
            return None;
        }
        seen.push(basis.clone());
        let left_uvw = match mesh_to_uvw(left, &basis, MeshValidationMode::CLOSED) {
            Ok(Some(mesh)) => mesh,
            Ok(None) => return None,
            Err(error) => {
                *blocker.borrow_mut() = Some(error);
                return None;
            }
        };
        let right_uvw = match mesh_to_uvw(right, &basis, MeshValidationMode::CLOSED) {
            Ok(Some(mesh)) => mesh,
            Ok(None) => return None,
            Err(error) => {
                *blocker.borrow_mut() = Some(error);
                return None;
            }
        };
        accept(left_uvw, right_uvw).map(|accepted| (basis, accepted))
    };

    if let Some(accepted) = find_affine_cell_basis(left, &mut accept_basis)? {
        return Ok(Some(accepted));
    }
    if let Some(error) = blocker.borrow_mut().take() {
        return Err(error);
    }
    let accepted = find_affine_cell_basis(right, &mut accept_basis)?;
    if let Some(error) = blocker.into_inner() {
        return Err(error);
    }
    Ok(accepted)
}

fn mesh_to_uvw(
    mesh: &Mesh,
    basis: &AffineBoxBasis,
    validation: MeshValidationMode,
) -> Result<Option<Mesh>, MeshError> {
    let view = mesh.view();
    let mut vertices = Vec::with_capacity(view.vertex_count());
    for point in view.vertices() {
        let Some(uvw) = point_to_uvw(point, basis) else {
            return Ok(None);
        };
        let replay = point_from_uvw(&uvw.x, &uvw.y, &uvw.z, basis);
        match point3_exact_equal(&replay, point) {
            Some(true) => {}
            Some(false) => return Ok(None),
            None => {
                return Err(MeshError::one(MeshBlocker::new(
                    MeshBlockerKind::ExactConstructionFailure,
                    "affine normalization replay equality is undecidable",
                )));
            }
        }
        vertices.push(Point3::new(uvw.x, uvw.y, uvw.z));
    }
    // A negative determinant reverses orientation under the exact affine
    // coordinate map. Reversing triangle order keeps the normalized shell
    // compatible with the orthogonal solid materializer.
    let triangles = if compare_reals(
        &det3(&basis.basis_u, &basis.basis_v, &basis.basis_w),
        &Real::from(0),
    )
    .value()
    .ok_or_else(|| {
        MeshError::one(MeshBlocker::new(
            MeshBlockerKind::ExactConstructionFailure,
            "affine normalization determinant sign is undecidable",
        ))
    })? == Ordering::Less
    {
        view.faces()
            .map(|face| reverse_triangle(&Triangle(face.vertex_indices())))
            .collect()
    } else {
        view.faces()
            .map(|face| Triangle(face.vertex_indices()))
            .collect()
    };
    Mesh::new_with_validation_mode_and_version(
        vertices,
        triangles,
        SourceProvenance::exact("exact affine-normalized box solid"),
        validation,
        1,
    )
    .map(Some)
}

fn point_to_uvw(point: &Point3, basis: &AffineBoxBasis) -> Option<Point3> {
    let delta = Point3::new(
        &point.x - &basis.origin.x,
        &point.y - &basis.origin.y,
        &point.z - &basis.origin.z,
    );
    let denominator = det3(&basis.basis_u, &basis.basis_v, &basis.basis_w);
    if compare_reals(&denominator, &Real::from(0)).value()? == Ordering::Equal {
        return None;
    }
    let u = (det3(&delta, &basis.basis_v, &basis.basis_w) / &denominator).ok()?;
    let v = (det3(&basis.basis_u, &delta, &basis.basis_w) / &denominator).ok()?;
    let w = (det3(&basis.basis_u, &basis.basis_v, &delta) / &denominator).ok()?;
    Some(Point3::new(u, v, w))
}

fn point_from_uvw(u: &Real, v: &Real, w: &Real, basis: &AffineBoxBasis) -> Point3 {
    Point3::new(
        basis.origin.x.clone()
            + &(u.clone() * &basis.basis_u.x)
            + &(v.clone() * &basis.basis_v.x)
            + &(w.clone() * &basis.basis_w.x),
        basis.origin.y.clone()
            + &(u.clone() * &basis.basis_u.y)
            + &(v.clone() * &basis.basis_v.y)
            + &(w.clone() * &basis.basis_w.y),
        basis.origin.z.clone()
            + &(u.clone() * &basis.basis_u.z)
            + &(v.clone() * &basis.basis_v.z)
            + &(w.clone() * &basis.basis_w.z),
    )
}

fn det3(a: &Point3, b: &Point3, c: &Point3) -> Real {
    let x_minor = b.y.clone() * &c.z - &(b.z.clone() * &c.y);
    let y_minor = b.x.clone() * &c.z - &(b.z.clone() * &c.x);
    let z_minor = b.x.clone() * &c.y - &(b.y.clone() * &c.x);
    (a.x.clone() * &x_minor - &(a.y.clone() * &y_minor)) + &(a.z.clone() * &z_minor)
}

/// Search exact affine bases from a single orthogonal-solid cell complex.
///
/// The search uses the complete retained triangle-edge graph to propose three
/// independent frame directions at every source vertex. A proposed frame is not
/// trusted as a numeric fit: it becomes evidence only if exact
/// determinant-ratio normalization and orthogonal-solid replay accept the full
/// source mesh. This deliberately favors completeness over heuristic sampling
/// because supportable exact cell complexes must not depend on vertex order.
fn find_affine_cell_basis<T>(
    mesh: &Mesh,
    accept_basis: &mut impl FnMut(AffineBoxBasis) -> Option<T>,
) -> Result<Option<T>, MeshError> {
    let view = mesh.view();
    if view.vertices().len() < 8 || view.faces().count() < 12 {
        return Ok(None);
    }
    let adjacency = vertex_adjacency(mesh);
    let direction_counts = mesh_direction_counts(mesh)?;
    let mut origins = (0..adjacency.len()).collect::<Vec<_>>();
    origins.sort_by_key(|&origin| adjacency[origin].len());
    for origin in origins {
        let neighbors = &adjacency[origin];
        let Ok(origin_point) = mesh
            .view()
            .vertex(origin)
            .map(|vertex| vertex.point().clone())
        else {
            continue;
        };
        let mut weighted_directions = Vec::new();
        for direction in unique_edge_directions(mesh, origin, neighbors)? {
            let mut weight = 0;
            for (seen, count) in &direction_counts {
                if points_equal_or_opposite(seen, &direction)? {
                    weight = *count;
                    break;
                }
            }
            weighted_directions.push((weight, direction));
        }
        weighted_directions.sort_by_key(|(weight, _)| core::cmp::Reverse(*weight));
        let directions = weighted_directions
            .into_iter()
            .map(|(_, direction)| direction)
            .collect::<Vec<_>>();
        for u in 0..directions.len() {
            for v in u + 1..directions.len() {
                for w in v + 1..directions.len() {
                    let basis = AffineBoxBasis {
                        origin: origin_point.clone(),
                        basis_u: directions[u].clone(),
                        basis_v: directions[v].clone(),
                        basis_w: directions[w].clone(),
                    };
                    if compare_reals(
                        &det3(&basis.basis_u, &basis.basis_v, &basis.basis_w),
                        &Real::from(0),
                    )
                    .value()
                        == Some(Ordering::Equal)
                    {
                        continue;
                    }
                    if let Some(accepted) = accept_basis(basis) {
                        return Ok(Some(accepted));
                    }
                }
            }
        }
    }
    Ok(None)
}

/// Count undirected exact triangle-edge directions in mesh space.
fn mesh_direction_counts(mesh: &Mesh) -> Result<Vec<(Point3, usize)>, MeshError> {
    let mut counts = Vec::<(Point3, usize)>::new();
    let view = mesh.view();
    for face in view.faces() {
        let [a, b, c] = face.vertex_indices();
        for [a, b] in [[a, b], [b, c], [c, a]] {
            let (a, b) = (view.vertex(a)?, view.vertex(b)?);
            let (a, b) = (a.point(), b.point());
            let direction = Point3::new(&b.x - &a.x, &b.y - &a.y, &b.z - &a.z);
            if compare_reals(&direction.x, &Real::from(0)).value() == Some(Ordering::Equal)
                && compare_reals(&direction.y, &Real::from(0)).value() == Some(Ordering::Equal)
                && compare_reals(&direction.z, &Real::from(0)).value() == Some(Ordering::Equal)
            {
                continue;
            }
            let mut existing_count = None;
            for (index, (seen, _)) in counts.iter().enumerate() {
                if points_equal_or_opposite(seen, &direction)? {
                    existing_count = Some(index);
                    break;
                }
            }
            if let Some(index) = existing_count {
                counts[index].1 += 1;
                continue;
            }
            counts.push((direction, 1));
        }
    }
    Ok(counts)
}

/// Build a unique undirected vertex adjacency list from retained triangles.
fn vertex_adjacency(mesh: &Mesh) -> Vec<Vec<usize>> {
    let mut adjacency = vec![Vec::new(); mesh.view().vertices().len()];
    for face in mesh.view().faces() {
        let [a, b, c] = face.vertex_indices();
        for [a, b] in [[a, b], [b, c], [c, a]] {
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
    }
    adjacency
}

/// Return unique outgoing exact edge directions at one origin vertex.
fn unique_edge_directions(
    mesh: &Mesh,
    origin: usize,
    neighbors: &[usize],
) -> Result<Vec<Point3>, MeshError> {
    let origin_point = mesh.view().vertex(origin)?.point();
    let mut directions = Vec::new();
    for &neighbor in neighbors {
        let neighbor = mesh.view().vertex(neighbor)?.point();
        let direction = Point3::new(
            &neighbor.x - &origin_point.x,
            &neighbor.y - &origin_point.y,
            &neighbor.z - &origin_point.z,
        );
        if compare_reals(&direction.x, &Real::from(0)).value() == Some(Ordering::Equal)
            && compare_reals(&direction.y, &Real::from(0)).value() == Some(Ordering::Equal)
            && compare_reals(&direction.z, &Real::from(0)).value() == Some(Ordering::Equal)
        {
            continue;
        }
        let mut seen_direction = false;
        for seen in &directions {
            if point3_exact_equal(seen, &direction).ok_or_else(affine_direction_equality_error)? {
                seen_direction = true;
                break;
            }
        }
        if seen_direction {
            continue;
        }
        directions.push(direction);
    }
    Ok(directions)
}

/// Compare exact directions up to sign.
fn points_equal_or_opposite(left: &Point3, right: &Point3) -> Result<bool, MeshError> {
    if point3_exact_equal(left, right).ok_or_else(affine_direction_equality_error)? {
        return Ok(true);
    }
    Ok(compare_reals(&left.x, &(-right.x.clone()))
        .value()
        .ok_or_else(affine_direction_equality_error)?
        == Ordering::Equal
        && compare_reals(&left.y, &(-right.y.clone()))
            .value()
            .ok_or_else(affine_direction_equality_error)?
            == Ordering::Equal
        && compare_reals(&left.z, &(-right.z.clone()))
            .value()
            .ok_or_else(affine_direction_equality_error)?
            == Ordering::Equal)
}

fn affine_direction_equality_error() -> MeshError {
    MeshError::one(MeshBlocker::new(
        MeshBlockerKind::UndecidablePredicate,
        "affine orthogonal solid edge direction equality is undecidable",
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn affine_direction_counts_report_stale_vertex_facts() {
        let mut mesh = Mesh::from_i64_triangles_with_validation_mode(
            &[
                0, 0, 0, //
                1, 0, 0, //
                0, 1, 0,
            ],
            &[0, 1, 2],
            MeshValidationMode::ALLOW_BOUNDARY,
        )
        .expect("test triangle should construct");
        mesh.facts.vertices.pop();

        let error = mesh_direction_counts(&mesh)
            .expect_err("stale vertex facts should propagate as a typed blocker");
        assert!(
            error.has_only_blocker_kinds(&[MeshBlockerKind::StaleFactReplay]),
            "{error:?}"
        );
        assert_eq!(error.blockers()[0].vertex(), Some(2));
    }
}
