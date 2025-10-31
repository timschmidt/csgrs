use super::kernel02::Kernel02;
use crate::bounds::{BPos, Query};
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
    mp: &Manifold,
    mq: &Manifold,
    expand: f64,
    fwd: bool
) -> Vec<i32> {
    let a = if fwd { mp } else { mq };
    let b = if fwd { mq } else { mp };

    let mut w03 = vec![0; a.nv];
    let k02 = Kernel02 {
        ps_p: &a.ps,
        ps_q: &b.ps,
        hs_q: &b.hs,
        ns: &mp.vns,
        expand,
        fwd,
    };

    let bbs = a.ps.iter()
        .enumerate()
        .map(|(i, p)| Query::Pt(BPos{id: Some(i), pos: *p}))
        .collect::<Vec<_>>();
    let mut rec = SimpleRecorder::new(
        |a, b| {
            let (s02, z02) = k02.op(a, b);
            if z02.is_some() { w03[a] += s02 * if fwd {1} else {-1}; }
        }
    );
    b.collider.collision(&bbs, &mut rec);
    w03
}
