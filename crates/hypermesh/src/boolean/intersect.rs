use nalgebra::{Vector2, Vector3, Vector4};

/**
 * These two functions (Interpolate and Intersect) are the only places where
 * floating-point operations take place in the whole Boolean function. These
 * are carefully designed to minimize rounding error and to remove it at edge
 * cases to ensure consistency.
 */

// not checked yet
pub fn interpolate(
    pl: Vector3<f64>,
    pr: Vector3<f64>,
    x: f64
) -> Vector2<f64> {
    let dx_l = x - pl.x;
    let dx_r = x - pr.x;
    let use_l = dx_l.abs() < dx_r.abs();
    let diff = pr - pl;
    let lambda = if use_l { dx_l / diff.x } else { dx_r / diff.x };

    if lambda.is_infinite() || diff.y.is_infinite() || diff.z.is_infinite() {
        return Vector2::new(pl.y, pl.z);
    }

    Vector2::new(
        lambda * diff.y + if use_l { pl.y } else { pr.y },
        lambda * diff.z + if use_l { pl.z } else { pr.z }
    )
}

// not checked yet
pub fn intersect(
    pl: Vector3<f64>,
    pr: Vector3<f64>,
    ql: Vector3<f64>,
    qr: Vector3<f64>,
) -> Vector4<f64> {
    let dy_l = ql.y - pl.y;
    let dy_r = qr.y - pr.y;
    assert!(dy_l * dy_r <= 0., "Boolean manifold error: no intersection");
    let use_l = dy_l.abs() < dy_r.abs();
    let dx = pr.x - pl.x;
    let mut lambda = if use_l {dy_l} else {dy_r} / (dy_l - dy_r);
    if lambda.is_infinite() { lambda = 0.; }
    let mut xyzz: Vector4<f64> = Default::default();
    xyzz.x = lambda * dx + if use_l {pl.x} else {pr.x};
    let p_dy = pr.y - pl.y;
    let q_dy = qr.y - ql.y;
    let use_p = p_dy.abs() < q_dy.abs();
    xyzz.y = lambda * if use_p {p_dy} else{q_dy} +
        (if use_l {if use_p {pl.y} else {ql.y}} else {if use_p {pr.y} else {qr.y}});
    xyzz.z = lambda * (pr.z - pl.z) + if use_l {pl.z} else {pr.z};
    xyzz.w = lambda * (qr.z - ql.z) + if use_l {ql.z} else {qr.z};
    xyzz
}

