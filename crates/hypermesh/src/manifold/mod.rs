pub mod bounds;
pub mod collider;

use nalgebra::{RowVector3, Vector3};
use bounds::BoundingBox;
use crate::collider::Collider;
use crate::{Half, Hmesh};

pub struct Manifold {
    pub hmesh: Hmesh,
    pub bbox: BoundingBox,
    pub n_halfs: usize,
    pub f_normal: Vec<RowVector3<f64>>,
    pub collider: dyn Collider,
}

impl Manifold {

}