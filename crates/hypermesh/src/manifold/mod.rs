pub mod bounds;
pub mod collider;

use nalgebra::{RowVector3, Vector3};
use bounds::BoundingBox;
use crate::collider::{morton_code, Collider, MortonCollider};
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

fn get_face_morton(hmesh: &Hmesh, bbox: &BoundingBox) -> (Vec<BoundingBox>, Vec<u32>) {
    assert!(hmesh.halfs.iter().all(|h| h.twin().id < usize::MAX)); // maybe not necessary
    let n = hmesh.faces.len();
    let mut fbs = vec![BoundingBox::default(); n];
    let mut fms = vec![0; n];
    for f in hmesh.faces.iter() {
        let p0 = f.half().tail().pos();
        let p1 = f.half().next().tail().pos();
        let p2 = f.half().prev().tail().pos();
        fbs[f.id].union(&p0);
        fbs[f.id].union(&p1);
        fbs[f.id].union(&p2);
        fms[f.id] = morton_code(&((p0 + p1 + p2) / 3.), bbox);
    }
    (fbs, fms)
}



pub struct Manifold {
    pub hmesh: Hmesh,
    pub bbox: BoundingBox,
    pub collider: MortonCollider,
}

impl Manifold {
    // todo: better accepting move hmesh. find out a way to do it
    pub fn new(hmesh: &Hmesh) -> Self {
        let bbox = BoundingBox::new_from_matrix(usize::MAX, &hmesh.pos);
        let (f_bboxes, f_morton) = get_face_morton(&hmesh, &bbox);
        Manifold {
            hmesh: hmesh.clone(),
            bbox,
            collider: MortonCollider::new(&f_bboxes, &f_morton)
        }
    }

    //fn sort_faces(
    //    &mut self,
    //    face_bboxes: &mut Vec<BoundingBox>,
    //    face_morton: &mut Vec<u32>
    //) {
    //    panic!("not implemented. not sure if this is necessary");
    //}
}
