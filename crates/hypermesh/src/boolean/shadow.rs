use nalgebra::{Vector2, Vector3};
use crate::boolean::intersect::interpolate;
use crate::{Half, Vert};

pub fn shadows(p: f64, q: f64, dir: f64) -> bool { if p == q { dir < 0. } else { p < q } }

pub fn shadows01(
    p0: usize,
    q1: usize,
    vp: &[Vert],
    vq: &[Vert],
    hq: &[Half],
    expand_p: f64,
    reverse: bool
) -> (i64, Vector2<f64>) {
    let q1s = hq[q1].tail().id;
    let q1e = hq[q1].head().id;
    let p0x  = vp[p0].pos().x;
    let q1sx = vp[q1s].pos().x;
    let q1ex = vp[q1e].pos().x;

    let mut s01 = if reverse {
        let sa = if shadows(q1sx, p0x, expand_p * vp[q1s].normal().x) {1} else {0};
        let sb = if shadows(q1ex, p0x, expand_p * vp[q1e].normal().x) {1} else {0};
        sa - sb
    } else {
        let sa = if shadows(p0x, q1ex, expand_p * vp[p0].normal().x) {1} else {0};
        let sb = if shadows(p0x, q1sx, expand_p * vp[p0].normal().x) {1} else {0};
        sa - sb
    };

    let mut yz01 = Vector2::new(0., 0.);
    if s01 != 0 {
        yz01 = interpolate(vq[q1s].pos().transpose(), vq[q1e].pos().transpose(), vp[p0].pos().x);
        if reverse {
            let diff = vq[q1s].pos() - vp[p0].pos();
            let sta2 = diff.clone().dot(&diff);
            let diff = vq[q1e].pos() - vp[p0].pos();
            let end2 = diff.clone().dot(&diff);
            let dir = if sta2 < end2 { vp[q1s].normal().y } else { vp[q1e].normal().y };
            if !shadows(yz01[0], vp[p0].pos().y, expand_p * dir) { s01 = 0; }
        } else {
            if!shadows(vp[p0].pos().y, yz01[0], expand_p * vp[p0].normal().y) { s01 = 0; }
        }
    }
    (s01, yz01)
}
