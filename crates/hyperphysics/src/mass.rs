//! Exact uniform-density mass-property reports.

use std::array::from_fn;

use hyperlattice::Vector3;
use hyperreal::{CertifiedRealSign, Real, RealSign};

use crate::{ClosedTriangleMesh3, ExactMaterial, PhysicsError, PhysicsResult};

/// Symmetric inertia tensor in row-major named components.
#[derive(Clone, Debug, PartialEq)]
pub struct SymmetricInertia3 {
    /// Moment around the x axis.
    pub xx: Real,
    /// Moment around the y axis.
    pub yy: Real,
    /// Moment around the z axis.
    pub zz: Real,
    /// Product component `Ixy`.
    pub xy: Real,
    /// Product component `Ixz`.
    pub xz: Real,
    /// Product component `Iyz`.
    pub yz: Real,
}

/// Certificate attached to a mass-property report.
#[derive(Clone, Debug, PartialEq)]
pub struct MassPropertyCertificate3 {
    /// Number of oriented triangles consumed.
    pub triangle_count: usize,
    /// Sign proof for the accumulated signed volume.
    pub signed_volume_sign: CertifiedRealSign,
    /// Whether the final physical integrals were flipped from inward winding.
    pub orientation_was_negative: bool,
}

/// Uniform-density mass properties for a closed oriented triangle mesh.
#[derive(Clone, Debug, PartialEq)]
pub struct MassPropertyReport3 {
    /// Certified positive density used by the report.
    pub density: Real,
    /// Non-negative enclosed volume.
    pub volume: Real,
    /// Oriented volume before absolute-value normalization.
    pub signed_volume: Real,
    /// Total mass.
    pub mass: Real,
    /// Exact center of mass.
    pub center_of_mass: Vector3,
    /// Inertia tensor about the coordinate origin.
    pub inertia_about_origin: SymmetricInertia3,
    /// Inertia tensor about the center of mass.
    pub inertia_about_center_of_mass: SymmetricInertia3,
    /// Audit certificate for the report.
    pub certificate: MassPropertyCertificate3,
}

impl SymmetricInertia3 {
    /// Returns an all-zero tensor.
    pub fn zero() -> Self {
        Self {
            xx: Real::zero(),
            yy: Real::zero(),
            zz: Real::zero(),
            xy: Real::zero(),
            xz: Real::zero(),
            yz: Real::zero(),
        }
    }

    fn scale(self, factor: &Real) -> Self {
        Self {
            xx: &self.xx * factor,
            yy: &self.yy * factor,
            zz: &self.zz * factor,
            xy: &self.xy * factor,
            xz: &self.xz * factor,
            yz: &self.yz * factor,
        }
    }

    fn sub_parallel_axis(self, mass: &Real, center: &Vector3) -> Self {
        let cx2 = &center[0] * &center[0];
        let cy2 = &center[1] * &center[1];
        let cz2 = &center[2] * &center[2];
        let cxy = &center[0] * &center[1];
        let cxz = &center[0] * &center[2];
        let cyz = &center[1] * &center[2];
        Self {
            xx: self.xx - (mass * &(cy2.clone() + cz2.clone())),
            yy: self.yy - (mass * &(cx2.clone() + cz2)),
            zz: self.zz - (mass * &(cx2 + cy2)),
            xy: self.xy + (mass * &cxy),
            xz: self.xz + (mass * &cxz),
            yz: self.yz + (mass * &cyz),
        }
    }
}

impl ClosedTriangleMesh3 {
    /// Computes exact uniform-density volume, center of mass, and inertia.
    ///
    /// The mesh is decomposed into oriented tetrahedra with one vertex at the
    /// origin. Volume, first moments, and second moments are accumulated over
    /// those tetrahedra using exact [`Real`] arithmetic. This is the same
    /// divergence/integral strategy used by Mirtich, "Fast and Accurate
    /// Computation of Polyhedral Mass Properties," *Journal of Graphics Tools*
    /// 1(2), 1996 (<https://doi.org/10.1080/10867651.1996.10487458>), but kept
    /// at the exact object layer in Yap's EGC sense instead of evaluating the
    /// integrals through primitive floating point.
    pub fn uniform_density_mass_properties(
        &self,
        density: Real,
    ) -> PhysicsResult<MassPropertyReport3> {
        require_positive_density(&density)?;

        let mut signed_volume = Real::zero();
        let mut first_moment = Vector3::zero();
        let mut second_moments = from_fn(|_| from_fn(|_| Real::zero()));

        for triangle in self.triangles() {
            let [a, b, c] = triangle.vertices();
            let det = determinant3(a, b, c);
            signed_volume = signed_volume + div_exact(&det, 6)?;

            let vertex_sum = a + b + c;
            let tetra_first = vertex_sum * div_exact(&det, 24)?;
            first_moment = first_moment + tetra_first;

            for (row_index, row) in second_moments.iter_mut().enumerate() {
                for (col_index, value) in row.iter_mut().enumerate() {
                    *value = value.clone() + tetra_second_moment(a, b, c, row_index, col_index)?;
                }
            }
        }

        let sign_certificate = signed_volume.certified_sign_until(-64);
        let sign = sign_certificate.sign();
        let orientation_was_negative = match sign {
            Some(RealSign::Positive) => false,
            Some(RealSign::Negative) => true,
            Some(RealSign::Zero) => return Err(PhysicsError::ZeroVolume),
            None => return Err(PhysicsError::UnknownSignedVolume),
        };

        let volume = if orientation_was_negative {
            -signed_volume.clone()
        } else {
            signed_volume.clone()
        };
        let center_of_mass = div_vector_by_real(first_moment, &signed_volume)?;
        if orientation_was_negative {
            for row in &mut second_moments {
                for value in row {
                    *value = -value.clone();
                }
            }
        }

        let mass = &density * &volume;
        let inertia_about_origin = inertia_from_second_moments(&second_moments).scale(&density);
        let inertia_about_center_of_mass = inertia_about_origin
            .clone()
            .sub_parallel_axis(&mass, &center_of_mass);

        Ok(MassPropertyReport3 {
            density,
            volume,
            signed_volume,
            mass,
            center_of_mass,
            inertia_about_origin,
            inertia_about_center_of_mass,
            certificate: MassPropertyCertificate3 {
                triangle_count: self.triangle_count(),
                signed_volume_sign: sign_certificate,
                orientation_was_negative,
            },
        })
    }

    /// Computes mass properties using a material's certified exact density.
    pub fn material_mass_properties(
        &self,
        material: &ExactMaterial,
    ) -> PhysicsResult<MassPropertyReport3> {
        self.uniform_density_mass_properties(material.density().clone())
    }
}

fn require_positive_density(density: &Real) -> PhysicsResult<()> {
    match density.refine_sign_until(-64) {
        Some(RealSign::Positive) => Ok(()),
        Some(RealSign::Negative | RealSign::Zero) | None => Err(PhysicsError::NonPositiveDensity),
    }
}

fn determinant3(a: &Vector3, b: &Vector3, c: &Vector3) -> Real {
    let bxcy = &b[1] * &c[2];
    let bzcy = &b[2] * &c[1];
    let bzcx = &b[2] * &c[0];
    let bxcz = &b[0] * &c[2];
    let bxcy_z = &b[0] * &c[1];
    let bycx = &b[1] * &c[0];
    &a[0] * &(bxcy - bzcy) + (&a[1] * &(bzcx - bxcz)) + (&a[2] * &(bxcy_z - bycx))
}

fn tetra_second_moment(
    a: &Vector3,
    b: &Vector3,
    c: &Vector3,
    row: usize,
    col: usize,
) -> PhysicsResult<Real> {
    let columns = [a, b, c];
    let det = determinant3(a, b, c);
    let mut numerator = Real::zero();
    for p in 0..3 {
        for q in 0..3 {
            let weight = if p == q { Real::from(2) } else { Real::one() };
            numerator = numerator + (&columns[p][row] * &columns[q][col]) * weight;
        }
    }
    div_exact(&(det * numerator), 120)
}

fn inertia_from_second_moments(second: &[[Real; 3]; 3]) -> SymmetricInertia3 {
    SymmetricInertia3 {
        xx: second[1][1].clone() + second[2][2].clone(),
        yy: second[0][0].clone() + second[2][2].clone(),
        zz: second[0][0].clone() + second[1][1].clone(),
        xy: -second[0][1].clone(),
        xz: -second[0][2].clone(),
        yz: -second[1][2].clone(),
    }
}

fn div_vector_by_real(vector: Vector3, rhs: &Real) -> PhysicsResult<Vector3> {
    Ok(Vector3::new([
        div_real(&vector[0], rhs)?,
        div_real(&vector[1], rhs)?,
        div_real(&vector[2], rhs)?,
    ]))
}

fn div_exact(value: &Real, denominator: i32) -> PhysicsResult<Real> {
    div_real(value, &Real::from(denominator))
}

fn div_real(lhs: &Real, rhs: &Real) -> PhysicsResult<Real> {
    (lhs / rhs).map_err(|_| PhysicsError::ZeroVolume)
}
