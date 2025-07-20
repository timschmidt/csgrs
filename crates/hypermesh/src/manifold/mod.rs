pub mod bounds;
pub mod collider;

use nalgebra::{RowVector3, Vector3};
use bounds::BoundingBox;
use crate::collider::Collider;
use crate::{Half, Hmesh};

#[derive(Clone)]
pub struct Halfedge {
    pub tail: i32,
    pub head: i32,
    pub pair: i32,
}

impl Halfedge {
    pub fn default() -> Self { Self { tail: -1, head: -1, pair: -1 } }
    pub fn is_forward(&self) -> bool { self.tail < self.head }
    // need partial eq
}

pub struct MfdBuffer {
    pub pos: Vec<RowVector3<f64>>,
    pub idx: Vec<RowVector3<i32>>,
    pub fnmls: Vec<RowVector3<f64>>,
    pub halfs: Vec<Halfedge>,
}

pub struct Manifold {
    pub hmesh: Hmesh,
    pub bbox: BoundingBox,
    pub collider: dyn Collider,
}