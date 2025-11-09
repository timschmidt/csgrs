use crate::common::{Half, Row2f, Row3f, Row4f};

// These two functions (Interpolate and Intersect) are the only places where
// floating-point operations take place in the whole Boolean function. These
// are carefully designed to minimize rounding error and to remove it at edge
// cases to ensure consistency.

pub fn interpolate(pl: Row3f, pr: Row3f, x: f64) -> Row2f {
    let dx_l = x - pl.x;
    let dx_r = x - pr.x;
    let diff = pr - pl;
    let use_l = dx_l.abs() < dx_r.abs();
    let lambda = if use_l { dx_l } else { dx_r } / diff.x ;

    if lambda.is_infinite() || lambda.is_nan() ||
       diff.y.is_infinite() || diff.y.is_nan() ||
       diff.z.is_infinite() || diff.z.is_nan() { return Row2f::new(pl.y, pl.z); }

    Row2f::new(
        lambda * diff.y + if use_l { pl.y } else { pr.y },
        lambda * diff.z + if use_l { pl.z } else { pr.z }
    )
}

pub fn intersect(
    pl: Row3f,
    pr: Row3f,
    ql: Row3f,
    qr: Row3f,
) -> Row4f {
    let dy_l = ql.y - pl.y;
    let dy_r = qr.y - pr.y;
    assert!(dy_l * dy_r <= 0., "Boolean manifold error: no intersection");
    let use_l = dy_l.abs() < dy_r.abs();
    let dx = pr.x - pl.x;
    let mut lambda = if use_l {dy_l} else {dy_r} / (dy_l - dy_r);
    if lambda.is_infinite() || lambda.is_nan() { lambda = 0.; }
    let mut xyzz = Row4f::default();
    xyzz.x = lambda * dx + if use_l {pl.x} else {pr.x};
    let p_dy = pr.y - pl.y;
    let q_dy = qr.y - ql.y;
    let use_p = p_dy.abs() < q_dy.abs();
    xyzz.y = lambda * if use_p {p_dy} else{q_dy} + (if use_l {if use_p {pl.y} else {ql.y}} else {if use_p {pr.y} else {qr.y}});
    xyzz.z = lambda * (pr.z - pl.z) + if use_l {pl.z} else {pr.z};
    xyzz.w = lambda * (qr.z - ql.z) + if use_l {ql.z} else {qr.z};
    xyzz
}

pub fn shadows(p: f64, q: f64, dir: f64) -> bool {
    if p == q { dir < 0. } else { p < q }
}

// This is equivalent to Kernel01 or X01 in the thesis.
// Expand represents the sign of the normal.
pub fn shadows01(
    p0: usize,
    q1: usize,
    ps_p: &[Row3f],
    ps_q: &[Row3f],
    hs_q: &[Half],
    ns: &[Row3f],
    expand: f64,
    reverse: bool
) -> Option<(i32, Row2f)> {
    let q1s = hs_q[q1].tail;
    let q1e = hs_q[q1].head;
    let p0x  = ps_p[p0].x;
    let q1sx = ps_q[q1s].x;
    let q1ex = ps_q[q1e].x;

    // check weather the vert is in between the half from the x-axis point of view
    let mut s01 = if reverse {
        let a = if shadows(q1sx, p0x, expand * ns[q1s].x) {1} else {0};
        let b = if shadows(q1ex, p0x, expand * ns[q1e].x) {1} else {0};
        a - b
    } else {
        let a = if shadows(p0x, q1ex, expand * ns[p0].x) {1} else {0};
        let b = if shadows(p0x, q1sx, expand * ns[p0].x) {1} else {0};
        a - b
    };

    // if in between...
    if s01 != 0 {
        let yz01 = interpolate(
            ps_q[q1s],
            ps_q[q1e],
            ps_p[p0].x
        );
        if reverse {
            let d1 = ps_q[q1s] - ps_p[p0];
            let d2 = ps_q[q1e] - ps_p[p0];
            let sta2 = d1.norm_squared();
            let end2 = d2.norm_squared();
            let dir = if sta2 < end2 { ns[q1s].y } else { ns[q1e].y };
            if !shadows(yz01[0], ps_p[p0].y, expand * dir) { s01 = 0; }
        } else {
            // return sign as 0 if vert from mfd_p is above
            if !shadows(ps_p[p0].y, yz01[0], expand * ns[p0].y) { s01 = 0; }
        }
        return Some((s01, yz01));
    }
    None
}

