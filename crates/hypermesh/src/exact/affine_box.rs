//! Exact affine-box solid booleans.
//!
//! Axis-aligned box shortcuts already cover a useful coplanar-volumetric cell
//! family, but they are still tied to world axes. This module retains one exact
//! affine frame for two closed parallelepiped meshes, normalizes both solids
//! into `(u, v, w)` AABB boxes, reuses the orthogonal box/cell materializers,
//! and lifts the accepted output back to 3D. The basis, normalized replay, and
//! lifted output are all exact object structure in the sense of Yap, "Towards
//! Exact Geometric Computation," *Computational Geometry* 7.1-2 (1997): no
//! primitive-float fit or tolerance decides the topology.
//!
//! The selected cells are the affine image of the rectangular box subdivision
//! described for orthogonal arrangements in de Berg, Cheong, van Kreveld, and
//! Overmars, *Computational Geometry: Algorithms and Applications*, 3rd ed.
//! (2008), Chapter 2. Affine coordinates are solved with exact Cramer-ratio
//! determinants, and every accepted source vertex must reconstruct exactly
//! from the retained frame before the orthogonal cell complex can be consumed.

use core::cmp::Ordering;

use hyperlimit::{Point3, compare_reals};

use super::box_solid::{
    cell_difference_axis_aligned_boxes, cell_union_axis_aligned_boxes,
    difference_axis_aligned_boxes, empty_difference_axis_aligned_boxes,
    intersection_axis_aligned_boxes, is_axis_aligned_box, multi_difference_axis_aligned_boxes,
    nested_difference_axis_aligned_boxes, union_axis_aligned_boxes,
};
use super::error::{DiagnosticKind, MeshDiagnostic, MeshError, Severity};
use super::mesh::{ExactMesh, ExactPoint3, Triangle};
use super::provenance::SourceProvenance;
use super::scalar::ExactReal;
use super::validation::ValidationPolicy;

/// Exact 3D affine frame for normalized box coordinates.
///
/// A normalized point `(u, v, w)` is interpreted as
/// `origin + u * basis_u + v * basis_v + w * basis_w`. The frame is part of
/// the certificate: every source and output vertex must replay through it
/// exactly before a copied boolean artifact is accepted.
#[derive(Clone, Debug, PartialEq)]
pub struct AffineBoxBasis {
    /// Exact 3D affine origin.
    pub origin: Point3,
    /// Exact vector for the normalized `u` axis.
    pub basis_u: Point3,
    /// Exact vector for the normalized `v` axis.
    pub basis_v: Point3,
    /// Exact vector for the normalized `w` axis.
    pub basis_w: Point3,
}

/// Named operation retained by an affine-box materialization.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AffineBoxOperation {
    /// Regularized solid union.
    Union,
    /// Positive-volume solid intersection.
    Intersection,
    /// Regularized solid difference `left - right`.
    Difference,
}

/// Exact affine-box boolean output.
///
/// This is a shortcut artifact, not a general convex boolean. It exists only
/// when both input meshes certify as boxes in one exact affine frame and the
/// corresponding normalized AABB operation is already supported by
/// [`crate::exact::box_solid`].
#[derive(Clone, Debug, PartialEq)]
pub struct AffineBoxArrangement {
    /// Shared affine frame used to normalize both source boxes.
    pub basis: AffineBoxBasis,
    /// Boolean operation that produced the retained mesh.
    pub operation: AffineBoxOperation,
    /// Exact lifted closed output mesh in original 3D space.
    pub mesh: ExactMesh,
}

impl AffineBoxArrangement {
    /// Validate the retained affine output mesh and basis replay.
    ///
    /// Local validation checks that the lifted mesh is a valid exact mesh and
    /// that every output vertex maps back through the retained basis to exact
    /// normalized coordinates. Source replay is handled by
    /// [`Self::validate_against_sources`], because only the original operands
    /// can prove that the retained operation was the correct normalized box
    /// materialization.
    pub fn validate(&self) -> Result<(), MeshError> {
        self.mesh.validate_retained_state().map_err(|error| {
            affine_box_error(format!("affine box output mesh is stale: {error:?}"))
        })?;
        mesh_to_uvw(&self.mesh, &self.basis, self.mesh.validation_policy())
            .ok_or_else(|| affine_box_error("affine box output does not replay through basis"))?;
        Ok(())
    }

    /// Validate this affine-box output by replaying it from source meshes.
    ///
    /// The source replay recomputes basis discovery, normalized box
    /// materialization, and 3D lifting. This follows Yap's exact-computation
    /// boundary: a closed output shell is not trusted as a standalone triangle
    /// soup when the retained source objects can still be checked.
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), MeshError> {
        self.validate()?;
        let replay = materialize_affine_boxes(
            left,
            right,
            self.operation,
            self.mesh.validation_policy(),
        )?
        .ok_or_else(|| affine_box_error("source replay did not reproduce affine box output"))?;
        if self == &replay {
            Ok(())
        } else {
            Err(affine_box_error(
                "retained affine box output does not match source replay",
            ))
        }
    }
}

/// Certify and materialize an affine-box union.
pub fn materialize_affine_box_union(
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ValidationPolicy,
) -> Result<Option<AffineBoxArrangement>, MeshError> {
    materialize_affine_boxes(left, right, AffineBoxOperation::Union, validation)
}

/// Certify and materialize an affine-box intersection.
pub fn materialize_affine_box_intersection(
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ValidationPolicy,
) -> Result<Option<AffineBoxArrangement>, MeshError> {
    materialize_affine_boxes(left, right, AffineBoxOperation::Intersection, validation)
}

/// Certify and materialize an affine-box difference.
pub fn materialize_affine_box_difference(
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ValidationPolicy,
) -> Result<Option<AffineBoxArrangement>, MeshError> {
    materialize_affine_boxes(left, right, AffineBoxOperation::Difference, validation)
}

/// Return whether an affine-box union is certified for these operands.
pub fn has_affine_box_union(left: &ExactMesh, right: &ExactMesh) -> bool {
    materialize_affine_box_union(left, right, ValidationPolicy::CLOSED)
        .map(|arrangement| arrangement.is_some())
        .unwrap_or(false)
}

/// Return whether an affine-box intersection is certified for these operands.
pub fn has_affine_box_intersection(left: &ExactMesh, right: &ExactMesh) -> bool {
    materialize_affine_box_intersection(left, right, ValidationPolicy::CLOSED)
        .map(|arrangement| arrangement.is_some())
        .unwrap_or(false)
}

/// Return whether an affine-box difference is certified for these operands.
pub fn has_affine_box_difference(left: &ExactMesh, right: &ExactMesh) -> bool {
    materialize_affine_box_difference(left, right, ValidationPolicy::CLOSED)
        .map(|arrangement| arrangement.is_some())
        .unwrap_or(false)
}

fn materialize_affine_boxes(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: AffineBoxOperation,
    validation: ValidationPolicy,
) -> Result<Option<AffineBoxArrangement>, MeshError> {
    if is_axis_aligned_box(left) && is_axis_aligned_box(right) {
        return Ok(None);
    }
    let Some(basis) = certify_affine_box_basis(left, right) else {
        return Ok(None);
    };
    let Some(left_uvw) = mesh_to_uvw(left, &basis, ValidationPolicy::CLOSED) else {
        return Ok(None);
    };
    let Some(right_uvw) = mesh_to_uvw(right, &basis, ValidationPolicy::CLOSED) else {
        return Ok(None);
    };
    if !is_axis_aligned_box(&left_uvw) || !is_axis_aligned_box(&right_uvw) {
        return Ok(None);
    }

    let uvw_output = materialize_normalized_boxes(&left_uvw, &right_uvw, operation)?;
    let Some(uvw_output) = uvw_output else {
        return Ok(None);
    };
    let mesh = mesh_from_uvw(&uvw_output, &basis, operation.output_label(), validation)?;
    let arrangement = AffineBoxArrangement {
        basis,
        operation,
        mesh,
    };
    arrangement.validate()?;
    Ok(Some(arrangement))
}

fn materialize_normalized_boxes(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: AffineBoxOperation,
) -> Result<Option<ExactMesh>, MeshError> {
    match operation {
        AffineBoxOperation::Union => {
            if let Some(mesh) = union_axis_aligned_boxes(left, right, ValidationPolicy::CLOSED)? {
                return Ok(Some(mesh));
            }
            cell_union_axis_aligned_boxes(left, right, ValidationPolicy::CLOSED)
        }
        AffineBoxOperation::Intersection => {
            intersection_axis_aligned_boxes(left, right, ValidationPolicy::CLOSED)
        }
        AffineBoxOperation::Difference => {
            if let Some(mesh) =
                difference_axis_aligned_boxes(left, right, ValidationPolicy::CLOSED)?
            {
                return Ok(Some(mesh));
            }
            if let Some(mesh) =
                multi_difference_axis_aligned_boxes(left, right, ValidationPolicy::CLOSED)?
            {
                return Ok(Some(mesh));
            }
            if let Some(mesh) =
                nested_difference_axis_aligned_boxes(left, right, ValidationPolicy::CLOSED)?
            {
                return Ok(Some(mesh));
            }
            if let Some(mesh) =
                empty_difference_axis_aligned_boxes(left, right, ValidationPolicy::CLOSED)?
            {
                return Ok(Some(mesh));
            }
            cell_difference_axis_aligned_boxes(left, right, ValidationPolicy::CLOSED)
        }
    }
}

fn certify_affine_box_basis(left: &ExactMesh, right: &ExactMesh) -> Option<AffineBoxBasis> {
    candidate_affine_box_bases(left).into_iter().find(|basis| {
        mesh_to_uvw(left, basis, ValidationPolicy::CLOSED)
            .as_ref()
            .is_some_and(is_axis_aligned_box)
            && mesh_to_uvw(right, basis, ValidationPolicy::CLOSED)
                .as_ref()
                .is_some_and(is_axis_aligned_box)
    })
}

/// Return candidate exact affine frames from a source parallelepiped mesh.
///
/// The candidates are intentionally derived only from an already retained box:
/// the eight exact corners provide the object-level frame evidence. Affine
/// cell-complex replay can then use those frames to normalize a larger
/// rectangular grid without inventing axes from approximate edge clustering.
/// This is the same object-first discipline described by Yap, "Towards Exact
/// Geometric Computation," *Computational Geometry* 7.1-2 (1997).
pub(crate) fn candidate_affine_box_bases(mesh: &ExactMesh) -> Vec<AffineBoxBasis> {
    mesh_points(mesh)
        .map(|points| candidate_bases(&points))
        .unwrap_or_default()
}

fn candidate_bases(points: &[Point3]) -> Vec<AffineBoxBasis> {
    let mut bases = Vec::new();
    for origin in 0..points.len() {
        for u in 0..points.len() {
            if u == origin {
                continue;
            }
            for v in u + 1..points.len() {
                if v == origin {
                    continue;
                }
                for w in v + 1..points.len() {
                    if w == origin {
                        continue;
                    }
                    let basis = AffineBoxBasis {
                        origin: points[origin].clone(),
                        basis_u: sub3(&points[u], &points[origin]),
                        basis_v: sub3(&points[v], &points[origin]),
                        basis_w: sub3(&points[w], &points[origin]),
                    };
                    if compare_reals(&basis.determinant(), &ExactReal::from(0)).value()
                        == Some(Ordering::Equal)
                    {
                        continue;
                    }
                    if points_match_parallelepiped_corners(points, &basis) {
                        bases.push(basis);
                    }
                }
            }
        }
    }
    bases
}

fn points_match_parallelepiped_corners(points: &[Point3], basis: &AffineBoxBasis) -> bool {
    // Yap's exact-object boundary lets us reject most candidate frames with
    // pure structural equality before using determinant ratios. For the source
    // box that supplies the basis, the eight corners must be exactly the subset
    // sums of the three basis vectors from one retained corner.
    let uv = add3(&basis.basis_u, &basis.basis_v);
    let uw = add3(&basis.basis_u, &basis.basis_w);
    let vw = add3(&basis.basis_v, &basis.basis_w);
    let uvw = add3(&uv, &basis.basis_w);
    let expected = [
        basis.origin.clone(),
        add3(&basis.origin, &basis.basis_u),
        add3(&basis.origin, &basis.basis_v),
        add3(&basis.origin, &basis.basis_w),
        add3(&basis.origin, &uv),
        add3(&basis.origin, &uw),
        add3(&basis.origin, &vw),
        add3(&basis.origin, &uvw),
    ];
    expected
        .iter()
        .all(|expected| points.iter().any(|point| points_equal(point, expected)))
}

pub(crate) fn mesh_to_uvw(
    mesh: &ExactMesh,
    basis: &AffineBoxBasis,
    validation: ValidationPolicy,
) -> Option<ExactMesh> {
    let vertices = mesh
        .vertices()
        .iter()
        .map(|point| {
            point_to_uvw_checked(&point.to_hyperlimit_point(), basis)
                .map(|uvw| ExactPoint3::new(uvw.x, uvw.y, uvw.z))
        })
        .collect::<Option<Vec<_>>>()?;
    let triangles = triangles_for_affine_orientation(mesh, basis)?;
    ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact("exact affine-normalized box solid"),
        validation,
    )
    .ok()
}

pub(crate) fn mesh_from_uvw(
    mesh: &ExactMesh,
    basis: &AffineBoxBasis,
    label: &'static str,
    validation: ValidationPolicy,
) -> Result<ExactMesh, MeshError> {
    let vertices = mesh
        .vertices()
        .iter()
        .map(|point| {
            let point = point.to_hyperlimit_point();
            let lifted = point_from_uvw(&point.x, &point.y, &point.z, basis);
            ExactPoint3::new(lifted.x, lifted.y, lifted.z)
        })
        .collect::<Vec<_>>();
    let triangles = if compare_reals(&basis.determinant(), &ExactReal::from(0)).value()
        == Some(Ordering::Less)
    {
        mesh.triangles().iter().map(reverse_triangle).collect()
    } else {
        mesh.triangles().to_vec()
    };
    ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact(label),
        validation,
    )
}

fn point_to_uvw_checked(point: &Point3, basis: &AffineBoxBasis) -> Option<Point3> {
    let uvw = point_to_uvw(point, basis)?;
    let replay = point_from_uvw(&uvw.x, &uvw.y, &uvw.z, basis);
    if points_equal(&replay, point) {
        Some(uvw)
    } else {
        None
    }
}

fn point_to_uvw(point: &Point3, basis: &AffineBoxBasis) -> Option<Point3> {
    let delta = sub3(point, &basis.origin);
    let denominator = basis.determinant();
    if compare_reals(&denominator, &ExactReal::from(0)).value()? == Ordering::Equal {
        return None;
    }
    let u = (det3(&delta, &basis.basis_v, &basis.basis_w) / &denominator).ok()?;
    let v = (det3(&basis.basis_u, &delta, &basis.basis_w) / &denominator).ok()?;
    let w = (det3(&basis.basis_u, &basis.basis_v, &delta) / &denominator).ok()?;
    Some(Point3::new(u, v, w))
}

fn triangles_for_affine_orientation(
    mesh: &ExactMesh,
    basis: &AffineBoxBasis,
) -> Option<Vec<Triangle>> {
    // A negative determinant reverses orientation under the exact affine
    // coordinate map. Reversing triangle order keeps the normalized shell
    // outward-facing while preserving Yap's exact-object/topology separation.
    if compare_reals(&basis.determinant(), &ExactReal::from(0)).value()? == Ordering::Less {
        Some(mesh.triangles().iter().map(reverse_triangle).collect())
    } else {
        Some(mesh.triangles().to_vec())
    }
}

fn reverse_triangle(triangle: &Triangle) -> Triangle {
    let [a, b, c] = triangle.0;
    Triangle([a, c, b])
}

fn point_from_uvw(u: &ExactReal, v: &ExactReal, w: &ExactReal, basis: &AffineBoxBasis) -> Point3 {
    Point3::new(
        add(
            &basis.origin.x,
            &add(
                &mul(u, &basis.basis_u.x),
                &add(&mul(v, &basis.basis_v.x), &mul(w, &basis.basis_w.x)),
            ),
        ),
        add(
            &basis.origin.y,
            &add(
                &mul(u, &basis.basis_u.y),
                &add(&mul(v, &basis.basis_v.y), &mul(w, &basis.basis_w.y)),
            ),
        ),
        add(
            &basis.origin.z,
            &add(
                &mul(u, &basis.basis_u.z),
                &add(&mul(v, &basis.basis_v.z), &mul(w, &basis.basis_w.z)),
            ),
        ),
    )
}

fn mesh_points(mesh: &ExactMesh) -> Option<Vec<Point3>> {
    if mesh.vertices().len() != 8 || mesh.triangles().len() != 12 {
        return None;
    }
    let mut points = Vec::with_capacity(8);
    for vertex in mesh.vertices() {
        let point = vertex.to_hyperlimit_point();
        if points
            .iter()
            .any(|candidate| points_equal(candidate, &point))
        {
            return None;
        }
        points.push(point);
    }
    Some(points)
}

impl AffineBoxBasis {
    pub(crate) fn determinant(&self) -> ExactReal {
        det3(&self.basis_u, &self.basis_v, &self.basis_w)
    }
}

impl AffineBoxOperation {
    const fn output_label(self) -> &'static str {
        match self {
            Self::Union => "exact affine coplanar-volumetric box union",
            Self::Intersection => "exact affine coplanar-volumetric box intersection",
            Self::Difference => "exact affine coplanar-volumetric box difference",
        }
    }
}

fn det3(a: &Point3, b: &Point3, c: &Point3) -> ExactReal {
    let x_minor = sub(&mul(&b.y, &c.z), &mul(&b.z, &c.y));
    let y_minor = sub(&mul(&b.x, &c.z), &mul(&b.z, &c.x));
    let z_minor = sub(&mul(&b.x, &c.y), &mul(&b.y, &c.x));
    add(
        &sub(&mul(&a.x, &x_minor), &mul(&a.y, &y_minor)),
        &mul(&a.z, &z_minor),
    )
}

fn sub3(left: &Point3, right: &Point3) -> Point3 {
    Point3::new(
        sub(&left.x, &right.x),
        sub(&left.y, &right.y),
        sub(&left.z, &right.z),
    )
}

fn add3(left: &Point3, right: &Point3) -> Point3 {
    Point3::new(
        add(&left.x, &right.x),
        add(&left.y, &right.y),
        add(&left.z, &right.z),
    )
}

fn points_equal(left: &Point3, right: &Point3) -> bool {
    real_eq(&left.x, &right.x) && real_eq(&left.y, &right.y) && real_eq(&left.z, &right.z)
}

fn real_eq(left: &ExactReal, right: &ExactReal) -> bool {
    compare_reals(left, right).value() == Some(Ordering::Equal)
}

fn add(left: &ExactReal, right: &ExactReal) -> ExactReal {
    left.clone() + right
}

fn sub(left: &ExactReal, right: &ExactReal) -> ExactReal {
    left.clone() - right
}

fn mul(left: &ExactReal, right: &ExactReal) -> ExactReal {
    left.clone() * right
}

fn affine_box_error(message: impl Into<String>) -> MeshError {
    MeshError::one(MeshDiagnostic::new(
        Severity::Error,
        DiagnosticKind::UnsupportedExactOperation,
        message,
    ))
}
