use crate::math_ndsp::{Point3, Scalar};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Aabb<T: Scalar> {
    pub mins: Point3<T>,
    pub maxs: Point3<T>,
}

impl<T> Aabb<T: Scalar> {
    #[inline]
    pub const fn new(mins: Point3<T>, maxs: Point3<T>) -> Self {
        Self { mins, maxs }
    }
    
    pub fn intersects(&self, other: &Aabb) -> bool {
        self.max.x >= other.min.x && self.min.x <= other.max.x &&
        self.max.y >= other.min.y && self.min.y <= other.max.y &&
        self.max.z >= other.min.z && self.min.z <= other.max.z
    }
}
