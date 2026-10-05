//! Shared exact comparisons for assembly frames and drawings.
//!
//! Ordering goes through `hyperlimit` with the crate predicate policy. A
//! comparison that the policy cannot decide stays `None` so callers can record
//! an unknown drawing or reject a frame, instead of substituting a float test.

use std::cmp::Ordering;

use hyperlattice::{Matrix4, Point3, Real, Vector3};

pub(super) fn vector_is_zero(vector: &Vector3) -> Option<bool> {
    let length_squared = vector.dot(vector);
    hyperlimit::classify_real_sign(&length_squared, crate::PREDICATE_POLICY)
        .value()
        .map(|sign| sign == hyperlimit::Sign::Zero)
}

pub(super) fn real_cmp(lhs: &Real, rhs: &Real) -> Option<Ordering> {
    hyperlimit::compare_reals(lhs, rhs, crate::PREDICATE_POLICY).value()
}

pub(super) fn real_eq(lhs: &Real, rhs: &Real) -> Option<bool> {
    real_cmp(lhs, rhs).map(|ordering| ordering == Ordering::Equal)
}

pub(super) fn real_lt(lhs: &Real, rhs: &Real) -> Option<bool> {
    real_cmp(lhs, rhs).map(|ordering| ordering == Ordering::Less)
}

pub(super) fn real_le(lhs: &Real, rhs: &Real) -> Option<bool> {
    real_cmp(lhs, rhs).map(|ordering| ordering != Ordering::Greater)
}

pub(super) fn real_gt(lhs: &Real, rhs: &Real) -> Option<bool> {
    real_cmp(lhs, rhs).map(|ordering| ordering == Ordering::Greater)
}

pub(super) fn real_ge(lhs: &Real, rhs: &Real) -> Option<bool> {
    real_cmp(lhs, rhs).map(|ordering| ordering != Ordering::Less)
}

pub(super) fn translation(offset: &Vector3) -> Matrix4 {
    Matrix4::affine_translation([
        offset.0[0].clone(),
        offset.0[1].clone(),
        offset.0[2].clone(),
    ])
}

pub(super) fn lerp_point(start: &Point3, end: &Point3, parameter: &Real) -> Point3 {
    Point3::new(
        &start.x + (&end.x - &start.x) * parameter,
        &start.y + (&end.y - &start.y) * parameter,
        &start.z + (&end.z - &start.z) * parameter,
    )
}
