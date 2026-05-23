use hyperlattice::Vector2;
pub(crate) use hyperlattice::{Aabb, Matrix4, Point3, Real, Vector3};
use hyperreal::{Problem, RealSign};
use std::cmp::Ordering;

pub(crate) fn pi() -> Real {
    Real::pi()
}

pub(crate) fn tau() -> Real {
    Real::tau()
}

pub(crate) fn frac_pi_2() -> Real {
    (Real::pi() / Real::from(2_u8)).expect("nonzero exact denominator")
}

#[cfg(test)]
pub(crate) fn tolerance() -> Real {
    hreal_from_f64(1.0e-9).expect("finite test tolerance")
}

pub(crate) trait IntoReal {
    fn into_real(self) -> Result<Real, Problem>;
}

impl IntoReal for Real {
    fn into_real(self) -> Result<Real, Problem> {
        Ok(self)
    }
}

impl IntoReal for &Real {
    fn into_real(self) -> Result<Real, Problem> {
        Ok(self.clone())
    }
}

impl IntoReal for f64 {
    fn into_real(self) -> Result<Real, Problem> {
        Real::try_from(self)
    }
}

impl IntoReal for &f64 {
    fn into_real(self) -> Result<Real, Problem> {
        Real::try_from(*self)
    }
}

impl IntoReal for f32 {
    fn into_real(self) -> Result<Real, Problem> {
        Real::try_from(self)
    }
}

impl IntoReal for &f32 {
    fn into_real(self) -> Result<Real, Problem> {
        Real::try_from(*self)
    }
}

impl IntoReal for usize {
    fn into_real(self) -> Result<Real, Problem> {
        Ok(Real::from(self as u64))
    }
}

impl IntoReal for i64 {
    fn into_real(self) -> Result<Real, Problem> {
        Ok(Real::from(self))
    }
}

impl IntoReal for u64 {
    fn into_real(self) -> Result<Real, Problem> {
        Ok(Real::from(self))
    }
}

impl IntoReal for u32 {
    fn into_real(self) -> Result<Real, Problem> {
        Ok(Real::from(self))
    }
}

impl IntoReal for u16 {
    fn into_real(self) -> Result<Real, Problem> {
        Ok(Real::from(self))
    }
}

impl IntoReal for u8 {
    fn into_real(self) -> Result<Real, Problem> {
        Ok(Real::from(self))
    }
}

impl IntoReal for i16 {
    fn into_real(self) -> Result<Real, Problem> {
        Ok(Real::from(self))
    }
}

impl IntoReal for i32 {
    fn into_real(self) -> Result<Real, Problem> {
        Ok(Real::from(self))
    }
}

impl IntoReal for i8 {
    fn into_real(self) -> Result<Real, Problem> {
        Ok(Real::from(self))
    }
}

pub(crate) fn hreal_from_f64<T: IntoReal>(value: T) -> Result<Real, Problem> {
    value.into_real()
}

#[allow(dead_code)]
pub(crate) fn hreal_from_f32(value: f32) -> Result<Real, Problem> {
    Real::try_from(value)
}

#[allow(dead_code)]
pub(crate) fn hreal_to_f64(value: &Real) -> Option<f64> {
    value.to_f64_lossy().filter(|value| value.is_finite())
}

pub(crate) fn hreal_sign(value: &Real) -> Option<RealSign> {
    value.refine_sign_until(128)
}

pub(crate) fn hreal_cmp_f64<L: IntoReal, R: IntoReal>(lhs: L, rhs: R) -> Ordering {
    let Ok(lhs) = lhs.into_real() else {
        return Ordering::Equal;
    };
    let Ok(rhs) = rhs.into_real() else {
        return Ordering::Equal;
    };
    hyperlimit::compare_reals(&lhs, &rhs)
        .value()
        .unwrap_or_else(|| match (lhs - rhs).refine_sign_until(128) {
            Some(RealSign::Positive) => Ordering::Greater,
            Some(RealSign::Negative) => Ordering::Less,
            Some(RealSign::Zero) | None => Ordering::Equal,
        })
}

pub(crate) fn hreal_gt_f64<L: IntoReal, R: IntoReal>(lhs: L, rhs: R) -> bool {
    matches!(hreal_cmp_f64(lhs, rhs), Ordering::Greater)
}

pub(crate) fn hreal_lt_f64<L: IntoReal, R: IntoReal>(lhs: L, rhs: R) -> bool {
    matches!(hreal_cmp_f64(lhs, rhs), Ordering::Less)
}

#[allow(dead_code)]
pub(crate) fn hreal_f64_gt<L: IntoReal, R: IntoReal>(lhs: L, rhs: R) -> bool {
    hreal_gt_f64(lhs, rhs)
}

pub(crate) fn hreal_f64s_exactly_equal<L: IntoReal, R: IntoReal>(lhs: L, rhs: R) -> bool {
    let Ok(lhs) = lhs.into_real() else {
        return false;
    };
    let Ok(rhs) = rhs.into_real() else {
        return false;
    };
    matches!((lhs - rhs).refine_sign_until(128), Some(RealSign::Zero))
}

pub(crate) fn hreal_mul<L: IntoReal, R: IntoReal>(lhs: L, rhs: R) -> Option<Real> {
    Some(lhs.into_real().ok()? * rhs.into_real().ok()?)
}

pub(crate) fn hreal_sub<L: IntoReal, R: IntoReal>(lhs: L, rhs: R) -> Option<Real> {
    Some(lhs.into_real().ok()? - rhs.into_real().ok()?)
}

pub(crate) fn hreal_div<L: IntoReal, R: IntoReal>(lhs: L, rhs: R) -> Option<Real> {
    (lhs.into_real().ok()? / rhs.into_real().ok()?).ok()
}

pub(crate) fn hreal_sum(values: &[Real]) -> Option<Real> {
    Some(
        values
            .iter()
            .cloned()
            .fold(Real::zero(), |sum, value| sum + value),
    )
}

pub(crate) fn hreal_abs<T: IntoReal>(value: T) -> Option<Real> {
    Some(value.into_real().ok()?.abs())
}

#[allow(dead_code)]
pub(crate) fn hreal_min(values: &[Real]) -> Option<Real> {
    values.iter().cloned().reduce(|best, value| {
        if hreal_cmp_f64(&value, &best) == Ordering::Less {
            value
        } else {
            best
        }
    })
}

#[allow(dead_code)]
pub(crate) fn hreal_max(values: &[Real]) -> Option<Real> {
    values.iter().cloned().reduce(|best, value| {
        if hreal_cmp_f64(&value, &best) == Ordering::Greater {
            value
        } else {
            best
        }
    })
}

pub(crate) fn hreal_clamp_f64(value: Real, min: f64, max: f64) -> Option<Real> {
    let min = hreal_from_f64(min).ok()?;
    let max = hreal_from_f64(max).ok()?;
    hyperlimit::real_clamp(value, &min, &max).value()
}

pub(crate) fn hreal_sqrt<T: IntoReal>(value: T) -> Option<Real> {
    value.into_real().ok()?.sqrt().ok()
}

pub(crate) fn hreal_atan<T: IntoReal>(value: T) -> Option<Real> {
    value.into_real().ok()?.atan().ok()
}

pub(crate) fn hreal_tan<T: IntoReal>(value: T) -> Option<Real> {
    value.into_real().ok()?.tan().ok()
}

pub(crate) fn hreal_pow<B: IntoReal, E: IntoReal>(base: B, exponent: E) -> Option<Real> {
    base.into_real().ok()?.pow(exponent.into_real().ok()?).ok()
}

pub(crate) fn hreal_affine<O: IntoReal, T: IntoReal, D: IntoReal>(
    origin: O,
    t: T,
    delta: D,
) -> Option<Real> {
    Some(origin.into_real().ok()? + t.into_real().ok()? * delta.into_real().ok()?)
}

pub(crate) fn hdegrees_to_radians<T: IntoReal>(degrees: T) -> Option<Real> {
    (degrees.into_real().ok()? * Real::pi() / Real::from(180_u8)).ok()
}

pub(crate) fn hangle_sin_cos<T: IntoReal>(angle: T) -> Option<(Real, Real)> {
    let angle = angle.into_real().ok()?;
    Some((angle.clone().sin(), angle.cos()))
}

pub(crate) fn hvector3_from_point3(point: &Point3) -> Option<Vector3> {
    Some(point.to_vector())
}

pub(crate) fn hvector3_from_vector3(vector: &Vector3) -> Option<Vector3> {
    Some(vector.clone())
}

pub(crate) fn hunit_vector3(vector: &Vector3) -> Option<Vector3> {
    vector.normalize_checked().ok()
}

pub(crate) fn hunit_cross_vector3(lhs: &Vector3, rhs: &Vector3) -> Option<Vector3> {
    lhs.unit_cross_checked(rhs).ok()
}

pub(crate) fn hrotation_between_vectors(from: &Vector3, to: &Vector3) -> Option<Matrix4> {
    Matrix4::rotation_between_vectors(from, to).ok()
}

pub(crate) fn htranslation_matrix(vector: &Vector3) -> Option<Matrix4> {
    Some(Matrix4::affine_translation([
        vector.0[0].clone(),
        vector.0[1].clone(),
        vector.0[2].clone(),
    ]))
}

#[allow(dead_code)]
pub(crate) fn hpoint_lerp(from: &Point3, to: &Point3, t: Real) -> Option<Point3> {
    Some(from.lerp(to, &t))
}

pub(crate) fn hxy_lerp(from: (Real, Real), to: (Real, Real), t: Real) -> Option<(Real, Real)> {
    let from = hvector2_from_xy(from);
    let to = hvector2_from_xy(to);
    let result = from.lerp(&to, &t);
    Some((result.0[0].clone(), result.0[1].clone()))
}

#[allow(dead_code)]
pub(crate) fn htriangle_area2_is_nonzero(a: &Point3, b: &Point3, c: &Point3) -> bool {
    let ab = b - a;
    let ac = c - a;
    let area2 = ab.cross(&ac);
    !matches!(area2.dot(&area2).refine_sign_until(128), Some(RealSign::Zero))
}

pub(crate) fn hvector2_from_xy(point: (Real, Real)) -> Vector2 {
    Vector2::new([point.0, point.1])
}
