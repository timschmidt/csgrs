use nalgebra::{RowVector2, RowVector3, RowVector4};
type Row2f = RowVector2<f64>;
type Row3f = RowVector3<f64>;
type Row4f = RowVector4<f64>;

/**
 * These two functions (Interpolate and Intersect) are the only places where
 * floating-point operations take place in the whole Boolean function. These
 * are carefully designed to minimize rounding error and to remove it at edge
 * cases to ensure consistency.
 */

pub fn interpolate(pl: Row3f, pr: Row3f, x: f64) -> Row2f {
    let dx_l = x - pl.x;
    let dx_r = x - pr.x;
    let use_l = dx_l.abs() < dx_r.abs();
    let diff = pr - pl;
    let lambda = if use_l { dx_l / diff.x } else { dx_r / diff.x };

    if lambda.is_infinite() || // todo: need to consider how to handle 0 divide case
       diff.y.is_infinite() ||
       diff.z.is_infinite() ||
       lambda.is_nan() ||
       diff.y.is_nan() ||
       diff.z.is_nan() {
        return Row2f::new(pl.y, pl.z);
    }

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

