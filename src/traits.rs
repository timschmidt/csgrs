use crate::float_types::Real;
use nalgebra::{Matrix4, Vector3};

/// Boolean operations in *native* space.
pub trait BooleanOps<Other = Self> {
    type Output;

    fn union(&self, other: &Other)        -> Self::Output;
    fn difference(&self, other: &Other)   -> Self::Output;
    fn intersection(&self, other: &Other) -> Self::Output;
}

/// Rigid + affine transformations in *native* space.
pub trait TransformOps {
	fn new() -> Self;
    fn transform(&self, m: &Matrix4<Real>) -> Self;
    fn translate(&self, v: Vector3<Real>) -> Self where Self: Sized {
        self.transform(&nalgebra::Translation3::from(v).to_homogeneous())
    }
    // rotate / scale convenience helpers...
}

/// Lossy or exact conversion between kernels.
pub trait Convert<Target> {
    fn to(&self)              -> Target;
    fn from(source: &Target)  -> Self;
}

