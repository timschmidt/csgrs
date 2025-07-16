pub mod bounds;
pub mod collider;

use bounds::BoundingBox;
use crate::collider::Collider;
use crate::Hmesh;

pub struct Manifold {
    pub hmesh: Hmesh,
    pub bbox: BoundingBox,
    pub collider: dyn Collider
}

