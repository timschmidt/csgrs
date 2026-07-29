use hyperlattice::{Point3, Real, Vector3};
use hyperreal::{Problem, RealSign};
use std::cmp::Ordering;

pub(crate) fn pi() -> Real {
    Real::pi()
}

pub(crate) fn tau() -> Real {
    Real::tau()
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

pub(crate) fn hreal_from_f32(value: f32) -> Result<Real, Problem> {
    Real::try_from(value)
}

pub(crate) fn hreal_to_f64(value: &Real) -> Option<f64> {
    value.to_f64_lossy().filter(|value| value.is_finite())
}

pub(crate) fn hreal_sign(value: &Real) -> Option<RealSign> {
    hyperlimit::classify_real_sign(value)
        .value()
        .map(|sign| match sign {
            hyperlimit::Sign::Negative => RealSign::Negative,
            hyperlimit::Sign::Zero => RealSign::Zero,
            hyperlimit::Sign::Positive => RealSign::Positive,
        })
}

pub(crate) fn hreal_try_cmp<L: IntoReal, R: IntoReal>(lhs: L, rhs: R) -> Option<Ordering> {
    let lhs = lhs.into_real().ok()?;
    let rhs = rhs.into_real().ok()?;
    hyperlimit::compare_reals(&lhs, &rhs).value()
}

pub(crate) fn hreal_cmp_f64<L: IntoReal, R: IntoReal>(lhs: L, rhs: R) -> Ordering {
    hreal_try_cmp(lhs, rhs).unwrap_or(Ordering::Equal)
}

pub(crate) fn hreal_gt_f64<L: IntoReal, R: IntoReal>(lhs: L, rhs: R) -> bool {
    matches!(hreal_cmp_f64(lhs, rhs), Ordering::Greater)
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

pub(crate) fn hreal_min(values: &[Real]) -> Option<Real> {
    values.iter().cloned().reduce(|best, value| {
        if hreal_cmp_f64(&value, &best) == Ordering::Less {
            value
        } else {
            best
        }
    })
}

pub(crate) fn hreal_max(values: &[Real]) -> Option<Real> {
    values.iter().cloned().reduce(|best, value| {
        if hreal_cmp_f64(&value, &best) == Ordering::Greater {
            value
        } else {
            best
        }
    })
}

pub(crate) fn hvector3_from_point3(point: &Point3) -> Option<Vector3> {
    Some(point.to_vector())
}

pub(crate) fn hvector3_from_vector3(vector: &Vector3) -> Option<Vector3> {
    Some(vector.clone())
}

pub(crate) fn hpoint_lerp(from: &Point3, to: &Point3, t: Real) -> Option<Point3> {
    Some(from.lerp(to, &t))
}

pub(crate) fn htriangle_area2_is_nonzero(a: &Point3, b: &Point3, c: &Point3) -> bool {
    let ab = b - a;
    let ac = c - a;
    let area2 = ab.cross(&ac);
    !matches!(hreal_sign(&area2.dot(&area2)), Some(RealSign::Zero))
}
