pub mod bounds;
pub mod collider;

use nalgebra::{RowVector3, Vector3};
use bounds::BoundingBox;
use crate::collider::Collider;
use crate::{Half, Hmesh};

#[derive(Clone)]
pub struct Halfedge {
    pub v0: i32,
    pub v1: i32,
    pub pair: i32,
}

impl Halfedge {
    pub fn default() -> Self {
        Self { v0: -1, v1: -1, pair: -1 }
    }

    pub fn is_forward(&self) -> bool { self.v0 < self.v1 }
    // need partial eq
}

pub struct MfdBuff {
    pub verts: Vec<Vector3<f64>>,
    pub fnmls: Vec<Vector3<f64>>,
    pub faces: Vec<Vector3<i32>>,
    pub halfs: Vec<Halfedge>,
}

pub struct Manifold {
    pub hmesh: Hmesh,
    pub bbox: BoundingBox,
    pub halfs: Vec<Halfedge>,
    pub f_normal: Vec<RowVector3<f64>>,
    pub collider: dyn Collider,
}

impl Manifold {

}