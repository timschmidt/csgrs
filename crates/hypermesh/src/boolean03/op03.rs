use super::kernel02::Kernel02;
use crate::bounds::{BoundingBox, Query};
use crate::collider::Recorder;
use crate::Manifold;

struct SimpleRecorder<F> where F: FnMut(usize, usize) {
    callback: F,
}

impl<F> SimpleRecorder<F> where F: FnMut(usize, usize) {
    fn new(callback: F) -> Self {
        Self { callback }
    }
}

impl<'a, F> Recorder for SimpleRecorder<F> where F: FnMut(usize, usize) {
    fn record(&mut self, query_idx: usize, leaf_idx: usize) {
        (self.callback)(query_idx, leaf_idx);
    }
}

pub fn winding03(
    p: &Manifold,
    q: &Manifold,
    expand: f64,
    forward: bool
) -> Vec<i32> {
    let a = if forward {p} else {q};
    let b = if forward {q} else {p};

    let mut w03 = vec![0; a.nv()];
    let k02 = Kernel02 {
        vpos_p: &a.pos,
        vpos_q: &b.pos,
        half_q: &b.hs,
        normal: &p.vert_normals,
        expand,
        forward,
    };

    let bbs = a.pos.iter()
        .enumerate()
        .map(|(i, p)|
            Query::Pt(BoundingBox::new(i, &vec![*p, *p]))
        ).collect::<Vec<_>>();
    let mut rec = SimpleRecorder::new(
        |a, b| {
            let (s02, z02) = k02.op(a, b);
            if z02.is_some() { w03[a] += s02 * if forward {1} else {-1}; }
        }
    );
    b.collider.collision(&bbs, &mut rec);
    w03
}
