use nalgebra::{RowVector2, Vector2, Vector3};
use crate::boolean::intersect::interpolate;
use crate::{test_data, Half, Vert};
use crate::boolean::kernel02::Kernel02;

pub fn shadows(p: f64, q: f64, dir: f64) -> bool { if p == q { dir < 0. } else { p < q } }


// This is equivalent to Kernel01 or X01 in the thesis
// given vert from mfd_p and half from mfd_q, find out whether
pub fn shadows01(
    p0: usize,
    q1: usize,
    vp: &[Vert],
    vq: &[Vert],
    hq: &[Half],
    expand: f64,  // sign of normal
    reverse: bool //
) -> Option<(i32, RowVector2<f64>)> {
    let q1s = hq[q1].tail().id;
    let q1e = hq[q1].head().id;
    let p0x  = vp[p0].pos().x;
    let q1sx = vq[q1s].pos().x;
    let q1ex = vq[q1e].pos().x;

    // check weather the vert is in between the half from the x-axis point of view
    let mut s01 = if reverse {
        let a = if shadows(q1sx, p0x, expand * vq[q1s].normal().x) {1} else {0};
        let b = if shadows(q1ex, p0x, expand * vq[q1e].normal().x) {1} else {0};
        a - b
    } else {
        let a = if shadows(p0x, q1ex, expand * vp[p0].normal().x) {1} else {0};
        let b = if shadows(p0x, q1sx, expand * vp[p0].normal().x) {1} else {0};

        //println!("p0x: {}, q1ex: {}, a: {}, b: {}", p0x, q1ex, a, b);
        a - b
    };

    // if in between...
    if s01 != 0 {
        let yz01 = interpolate(vq[q1s].pos(), vq[q1e].pos(), vp[p0].pos().x);
        if reverse {
            let diff = vq[q1s].pos() - vp[p0].pos();
            let sta2 = diff.clone().dot(&diff);
            let diff = vq[q1e].pos() - vp[p0].pos();
            let end2 = diff.clone().dot(&diff);
            let dir = if sta2 < end2 { vq[q1s].normal().y } else { vq[q1e].normal().y };
            if !shadows(yz01[0], vp[p0].pos().y, expand * dir) { s01 = 0; }
        } else {
            // return sign as 0 if vert from mfd_p is above
            if !shadows(vp[p0].pos().y, yz01[0], expand * vp[p0].normal().y) { s01 = 0; }
        }
        return Some((s01, yz01));
    }
    None
}


#[test]
fn shadow01_test() {
    let mfd_p = test_data::gen_tri_c();
    let mfd_q = test_data::gen_tri_a();

    for ip0 in 0..3 {
        for iq1 in 0..3 {
            println!("ip0: {}, iq1: {}", ip0, iq1);
            let res = shadows01(ip0, iq1, &mfd_p.verts, &mfd_q.verts, &mfd_q.halfs, 1., false);
            if res.is_some() {
                println!("s: {}, pos: {:#}", res.unwrap().0, res.unwrap().1);
            }
        }
    }
}
