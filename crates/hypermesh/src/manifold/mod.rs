pub mod bounds;
mod collider;

use crate::Hmesh;
use bounds::BoundingBox;

pub struct Manifold {
    pub hmesh: Hmesh,
    pub bbox: BoundingBox,
}
