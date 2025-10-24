use crate::math_ndsp::{Point3};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Aabb {
    pub mins: Point3<f64>,
    pub maxs: Point3<f64>,
}

impl Aabb {
    #[inline]
    pub const fn new(mins: Point3<f64>, maxs: Point3<f64>) -> Self {
        Self { mins, maxs }
    }
}
