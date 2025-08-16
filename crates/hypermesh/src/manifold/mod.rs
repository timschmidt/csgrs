pub mod bounds;
pub mod collider;

use nalgebra::{RowVector3, Vector3};
use bounds::BoundingBox;
use crate::collider::{BvhCollider, Collider};
use crate::{Half, Hmesh};

#[derive(Clone, Debug)]
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

pub struct Manifold {
    pub hmesh: Hmesh,
    pub bbox: BoundingBox,
    pub collider: BvhCollider,
}

impl Manifold {
    // todo: better accepting move. find out a way to do it
    pub fn new(hmesh: &Hmesh) -> Self {
        let leaf = hmesh.faces.iter().map(|f| {
            BoundingBox::new(
                f.id,
                &vec![
                    f.half().tail().pos(),
                    f.half().next().tail().pos(),
                    f.half().prev().tail().pos()
                ]
            )
        }).collect::<Vec<BoundingBox>>();

        Manifold{
            hmesh: hmesh.clone(),
            bbox: BoundingBox::new_from_matrix(usize::MAX, &hmesh.pos),
            collider: BvhCollider::new(&leaf)
        }
    }
}
