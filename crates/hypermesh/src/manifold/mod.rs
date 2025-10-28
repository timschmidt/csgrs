pub mod bounds;
pub mod collider;
pub mod hmesh;
mod coplanar;

use std::sync::Arc;
use nalgebra::{DMatrix};
use bounds::BoundingBox;
use crate::collider::{morton_code, MortonCollider};
use crate::hmesh::Hmesh;
use crate::common::K_PRECISION;
use crate::manifold::coplanar::compute_coplanar_idx;

#[derive(Clone, Debug)]
pub struct Halfedge {
    pub tail: usize,
    pub head: usize,
    pub pair: usize,
}

impl Default for Halfedge {
    fn default() -> Self {
        Self {
            tail: usize::MAX,
            head: usize::MAX,
            pair: usize::MAX
        }
    }
}

impl Halfedge {
    pub fn new(tail: usize, head: usize, pair: usize) -> Self { Self { tail, head, pair } }
    pub fn is_forward(&self) -> bool { self.tail < self.head }
    pub fn has_tail(&self) -> bool { self.tail != usize::MAX }
    pub fn has_head(&self) -> bool { self.head != usize::MAX }
    pub fn has_pair(&self) -> bool { self.pair != usize::MAX }
    pub fn no_tail(&self) -> bool { self.tail == usize::MAX }
    pub fn no_head(&self) -> bool { self.head == usize::MAX }
    pub fn no_pair(&self) -> bool { self.pair == usize::MAX }

    pub fn tail(&self) -> Option<usize> { if self.tail == usize::MAX { None } else { Some(self.tail) } }
    pub fn head(&self) -> Option<usize> { if self.head == usize::MAX { None } else { Some(self.head) } }
    pub fn pair(&self) -> Option<usize> { if self.pair == usize::MAX { None } else { Some(self.pair) } }

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

fn sort_faces(
    hmesh: &Hmesh,
    face_bboxes: &mut Vec<BoundingBox>,
    face_morton: &mut Vec<u32>
) -> Arc<Hmesh> {
    let mut table = (0..face_morton.len()).collect::<Vec<_>>();
    table.sort_by_key(|&i| face_morton[i]);
    *face_bboxes = table.iter().map(|&i| face_bboxes[i].clone()).collect::<Vec<_>>();
    *face_morton = table.iter().map(|&i| face_morton[i]).collect::<Vec<_>>();
    // sort faces...
    let mut idx = DMatrix::<usize>::zeros(table.len(), 3);
    for i in 0..table.len() {
        idx.set_row(i, &hmesh.idx.fixed_view::<1, 3>(table[i], 0));
    }
    Hmesh::new(hmesh.pos.clone(), idx)
}

pub struct Manifold {
    pub hmesh: Arc<Hmesh>,
    pub bbox: BoundingBox,
    pub collider: MortonCollider,
    pub coplanar: Vec<i32>,
    pub epsilon: f64,
    pub tolerance: f64,
}

impl Manifold {
    //
    // todo: better accepting move hmesh. find out a way to do it
    //
    pub fn new(hmesh: &Hmesh) -> Self {
        let bbox = BoundingBox::new_from_matrix(usize::MAX, &hmesh.pos);
        let (mut f_bboxes, mut f_morton) = get_face_morton(&hmesh, &bbox);

        let hmesh_sorted = sort_faces(&hmesh, &mut f_bboxes, &mut f_morton);

        let coplanar = compute_coplanar_idx(
            &hmesh_sorted.verts.iter().map(|v| v.pos()).collect::<Vec<_>>(),
            &hmesh_sorted.faces.iter().map(|f| f.normal()).collect::<Vec<_>>(),
            &hmesh_sorted.halfs.iter().map(|h| Halfedge{ tail: h.tail().id, head: h.head().id, pair: h.twin().id }).collect::<Vec<_>>(),
            1e-6 // todo: temporary!
        );

        let mut mfd = Manifold {
            hmesh: hmesh_sorted,
            bbox,
            collider: MortonCollider::new(&f_bboxes, &f_morton),
            coplanar,
            epsilon: -1.,
            tolerance: -1.,
        };

        mfd.set_epsilon(K_PRECISION * mfd.bbox.scale(), false);
        mfd
    }

    pub fn set_epsilon(&mut self, min_epsilon: f64, use_single: bool) {
        let s = self.bbox.scale();
        let mut e = min_epsilon.max(K_PRECISION * s);
        e = if e.is_finite() { e } else { -1. };
        let t = if use_single { e.max(f64::EPSILON * s) } else { e };
        self.epsilon = e;
        self.tolerance = self.tolerance.max(t);
    }
}
