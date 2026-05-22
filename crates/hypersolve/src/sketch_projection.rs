//! Exact symbolic workplane projection polynomials for sketch constraints.
//!
//! This module keeps SolveSpace-style projected-distance algebra separate from
//! the retained sketch handle layer. The caller resolves workplane/point
//! handles, then this module builds the exact proof expressions: a unit
//! quaternion guard and a squared `U/V` projected distance. That separation is
//! the point of Yap's exact geometric computation paradigm: retain geometric
//! objects, lower only to explicit proof obligations, and leave lossy
//! projection coordinates to proposal adapters. See C. K. Yap, "Towards Exact
//! Geometric Computation" (1997). The `U/V/N` axes use the standard unit
//! quaternion rotation matrix from K. Shoemake, "Animating Rotation with
//! Quaternion Curves" (1985).

use crate::symbolic::Expr;

/// Build the exact unit-quaternion guard `w^2+x^2+y^2+z^2-1`.
pub(crate) fn unit_quaternion_residual(quaternion: &[Expr; 4]) -> Expr {
    squared_norm4(quaternion) - Expr::int(1)
}

/// Build the exact squared distance after projecting `delta` onto a workplane.
///
/// The returned expression is `(delta . U)^2 + (delta . V)^2`, where `U` and
/// `V` are the polynomial unit-quaternion frame axes. It is intentionally not
/// divided or square-rooted; callers compare it to a squared distance carrier
/// and keep true-distance residuals as proposal-only forms.
pub(crate) fn projected_distance_squared(delta: &[Expr; 3], quaternion: &[Expr; 4]) -> Expr {
    let (u_axis, v_axis, _) = quaternion_frame_axes_expr(quaternion);
    let u = dot3(delta, &u_axis);
    let v = dot3(delta, &v_axis);
    u.clone() * u + v.clone() * v
}

fn squared_norm4(direction: &[Expr; 4]) -> Expr {
    direction[0].clone() * direction[0].clone()
        + direction[1].clone() * direction[1].clone()
        + direction[2].clone() * direction[2].clone()
        + direction[3].clone() * direction[3].clone()
}

fn dot3(a: &[Expr; 3], b: &[Expr; 3]) -> Expr {
    a[0].clone() * b[0].clone() + a[1].clone() * b[1].clone() + a[2].clone() * b[2].clone()
}

pub(crate) fn quaternion_frame_axes_expr(
    quaternion: &[Expr; 4],
) -> ([Expr; 3], [Expr; 3], [Expr; 3]) {
    let w = quaternion[0].clone();
    let x = quaternion[1].clone();
    let y = quaternion[2].clone();
    let z = quaternion[3].clone();
    let two = Expr::int(2);

    let ww = w.clone() * w.clone();
    let xx = x.clone() * x.clone();
    let yy = y.clone() * y.clone();
    let zz = z.clone() * z.clone();
    let wx = w.clone() * x.clone();
    let wy = w.clone() * y.clone();
    let wz = w.clone() * z.clone();
    let xy = x.clone() * y.clone();
    let xz = x.clone() * z.clone();
    let yz = y.clone() * z.clone();

    let u_axis = [
        ww.clone() + xx.clone() - yy.clone() - zz.clone(),
        two.clone() * (xy.clone() + wz.clone()),
        two.clone() * (xz.clone() - wy.clone()),
    ];
    let v_axis = [
        two.clone() * (xy - wz),
        ww.clone() - xx.clone() + yy.clone() - zz.clone(),
        two.clone() * (yz.clone() + wx.clone()),
    ];
    let n_axis = [two.clone() * (xz + wy), two * (yz - wx), ww - xx - yy + zz];

    (u_axis, v_axis, n_axis)
}
