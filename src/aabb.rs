use crate::math_ndsp::{Point3, Scalar};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Aabb<T: Scalar> {
    pub mins: Point3<T>,
    pub maxs: Point3<T>,
}

impl<T: Scalar + PartialOrd> Aabb<T> {
    #[inline]
    pub const fn new(mins: Point3<T>, maxs: Point3<T>) -> Self {
        Self { mins, maxs }
    }
    
    #[inline]
    pub fn intersects(&self, other: &Self) -> bool {
        self.maxs.x >= other.mins.x
            && self.mins.x <= other.maxs.x
            && self.maxs.y >= other.mins.y
            && self.mins.y <= other.maxs.y
            && self.maxs.z >= other.mins.z
            && self.mins.z <= other.maxs.z
    }

    #[inline]
    pub fn center(&self) -> Point3<T> {
        Point3::new(
            (self.mins.x + self.maxs.x) / T::mixed_from(2.0),
            (self.mins.y + self.maxs.y) / T::mixed_from(2.0),
            (self.mins.z + self.maxs.z) / T::mixed_from(2.0),
        )
    }
}
