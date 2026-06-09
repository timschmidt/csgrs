//! Exact affine-box solid booleans.
//!
//! Axis-aligned box shortcuts already cover a useful coplanar-volumetric cell
//! family, but they are still tied to world axes. This module retains one exact
//! affine frame for two closed parallelepiped meshes, normalizes both solids
//! into `(u, v, w)` AABB boxes, reuses the orthogonal box/cell materializers,
//! and lifts the accepted output back to 3D. The basis, normalized replay, and
//! primitive-float fit or tolerance decides the topology.
//!
//! The selected cells are the affine image of the rectangular box subdivision
//! determinants, and every accepted source vertex must reconstruct exactly
//! from the retained frame before the orthogonal cell complex can be consumed.

use core::cmp::Ordering;

use hyperlimit::{Point3, compare_reals};

use super::box_solid::is_axis_aligned_box;
use super::error::MeshError;
use super::mesh::{ExactMesh, Triangle};
use super::orthogonal_solid::{
    AxisAlignedOrthogonalSolidOperation, has_axis_aligned_orthogonal_solid_cells,
};
use super::validation::ValidationPolicy;
use hyperlimit::SourceProvenance;
use hyperreal::Real;

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

/// Return whether an affine-box union is certified for these operands.
pub fn has_affine_box_union(left: &ExactMesh, right: &ExactMesh) -> bool {
    has_affine_box_operation(left, right, AffineBoxOperation::Union)
}

/// Return whether an affine-box intersection is certified for these operands.
pub fn has_affine_box_intersection(left: &ExactMesh, right: &ExactMesh) -> bool {
    has_affine_box_operation(left, right, AffineBoxOperation::Intersection)
}

/// Return whether an affine-box difference is certified for these operands.
pub fn has_affine_box_difference(left: &ExactMesh, right: &ExactMesh) -> bool {
    has_affine_box_operation(left, right, AffineBoxOperation::Difference)
}

fn has_affine_box_operation(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: AffineBoxOperation,
) -> bool {
    affine_box_operation_is_supported(left, right, operation)
}

fn has_normalized_affine_box_operation(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: AffineBoxOperation,
) -> bool {
    has_axis_aligned_orthogonal_solid_cells(left, right, operation.into())
}

fn affine_box_operation_is_supported(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: AffineBoxOperation,
) -> bool {
    if is_axis_aligned_box(left) && is_axis_aligned_box(right) {
        return false;
    }
    candidate_affine_box_bases(left).into_iter().any(|basis| {
        let Some(left_uvw) = mesh_to_uvw(left, &basis, ValidationPolicy::CLOSED) else {
            return false;
        };
        let Some(right_uvw) = mesh_to_uvw(right, &basis, ValidationPolicy::CLOSED) else {
            return false;
        };
        is_axis_aligned_box(&left_uvw)
            && is_axis_aligned_box(&right_uvw)
            && has_normalized_affine_box_operation(&left_uvw, &right_uvw, operation)
    })
}

/// Return candidate exact affine frames from a source parallelepiped mesh.
///
/// The candidates are intentionally derived only from an already retained box:
/// the eight exact corners provide the object-level frame evidence. Affine
/// cell-complex replay can then use those frames to normalize a larger
/// rectangular grid without inventing axes from approximate edge clustering.
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
                    if compare_reals(&basis.determinant(), &Real::from(0)).value()
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
            point_to_uvw_checked(&point.clone(), basis).map(|uvw| Point3::new(uvw.x, uvw.y, uvw.z))
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
            let point = point.clone();
            let lifted = point_from_uvw(&point.x, &point.y, &point.z, basis);
            Point3::new(lifted.x, lifted.y, lifted.z)
        })
        .collect::<Vec<_>>();
    let triangles =
        if compare_reals(&basis.determinant(), &Real::from(0)).value() == Some(Ordering::Less) {
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
    if compare_reals(&denominator, &Real::from(0)).value()? == Ordering::Equal {
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
    if compare_reals(&basis.determinant(), &Real::from(0)).value()? == Ordering::Less {
        Some(mesh.triangles().iter().map(reverse_triangle).collect())
    } else {
        Some(mesh.triangles().to_vec())
    }
}

fn reverse_triangle(triangle: &Triangle) -> Triangle {
    let [a, b, c] = triangle.0;
    Triangle([a, c, b])
}

fn point_from_uvw(u: &Real, v: &Real, w: &Real, basis: &AffineBoxBasis) -> Point3 {
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
        let point = vertex.clone();
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
    pub(crate) fn determinant(&self) -> Real {
        det3(&self.basis_u, &self.basis_v, &self.basis_w)
    }
}

impl From<AffineBoxOperation> for AxisAlignedOrthogonalSolidOperation {
    fn from(operation: AffineBoxOperation) -> Self {
        match operation {
            AffineBoxOperation::Union => Self::Union,
            AffineBoxOperation::Intersection => Self::Intersection,
            AffineBoxOperation::Difference => Self::Difference,
        }
    }
}

fn det3(a: &Point3, b: &Point3, c: &Point3) -> Real {
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

fn real_eq(left: &Real, right: &Real) -> bool {
    compare_reals(left, right).value() == Some(Ordering::Equal)
}

fn add(left: &Real, right: &Real) -> Real {
    left.clone() + right
}

fn sub(left: &Real, right: &Real) -> Real {
    left.clone() - right
}

fn mul(left: &Real, right: &Real) -> Real {
    left.clone() * right
}
