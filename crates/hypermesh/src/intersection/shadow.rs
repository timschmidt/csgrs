use nalgebra::{RowVector2, RowVector3};
use crate::hmesh::Half;
use super::intersect::interpolate;
type Row2f = RowVector2<f64>;
type Row3f = RowVector3<f64>;

pub fn shadows(p: f64, q: f64, dir: f64) -> bool {
    if p == q { dir < 0. } else { p < q }
}

// This is equivalent to Kernel01 or X01 in the thesis
// given vert from mfd_p and half from mfd_q, find out whether
pub fn shadows01(
    p0: usize,
    q1: usize,
    vpos_p: &[Row3f],
    vpos_q: &[Row3f],
    hq: &[Half],
    normal: &[Row3f],
    expand: f64,  // sign of normal
    reverse: bool //
) -> Option<(i32, Row2f)> {
    let q1s = hq[q1].tail().id;
    let q1e = hq[q1].head().id;
    let p0x  = vpos_p[p0].x;
    let q1sx = vpos_q[q1s].x;
    let q1ex = vpos_q[q1e].x;

    // check weather the vert is in between the half from the x-axis point of view
    let mut s01 = if reverse {
        let a = if shadows(q1sx, p0x, expand * normal[q1s].x) {1} else {0};
        let b = if shadows(q1ex, p0x, expand * normal[q1e].x) {1} else {0};
        a - b
    } else {
        let a = if shadows(p0x, q1ex, expand * normal[p0].x) {1} else {0};
        let b = if shadows(p0x, q1sx, expand * normal[p0].x) {1} else {0};
        a - b
    };

    // if in between...
    if s01 != 0 {
        let yz01 = interpolate(
            vpos_q[q1s],
            vpos_q[q1e],
            vpos_p[p0].x
        );
        if reverse {
            let diff = vpos_q[q1s] - vpos_p[p0];
            let sta2 = diff.clone().dot(&diff);
            let diff = vpos_q[q1e] - vpos_p[p0];
            let end2 = diff.clone().dot(&diff);
            let dir = if sta2 < end2 { normal[q1s].y } else { normal[q1e].y };
            if !shadows(yz01[0], vpos_p[p0].y, expand * dir) { s01 = 0; }
        } else {
            // return sign as 0 if vert from mfd_p is above
            if !shadows(vpos_p[p0].y, yz01[0], expand * normal[p0].y) { s01 = 0; }
        }
        return Some((s01, yz01));
    }
    None
}
