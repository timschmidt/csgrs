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

/// Project a retained 3D direction into exact workplane `U/V` coordinates.
///
/// The two returned expressions are `(direction . U, direction . V)` using the
/// same polynomial unit-quaternion frame as projected-distance rows. This
/// helper intentionally does not normalize the direction; zero projected
/// length remains a report-bearing domain obligation for callers. The
/// construction follows Yap's exact replay boundary and Shoemake's
/// unit-quaternion frame formula cited in the module docs.
pub(crate) fn projected_direction2(direction: &[Expr; 3], quaternion: &[Expr; 4]) -> [Expr; 2] {
    let (u_axis, v_axis, _) = quaternion_frame_axes_expr(quaternion);
    [dot3(direction, &u_axis), dot3(direction, &v_axis)]
}

/// Build the exact squared length of a retained 3D direction after workplane projection.
///
/// The expression is `(direction . U)^2 + (direction . V)^2`, using the same
/// retained unit-quaternion frame as projected distance rows.  This is the
/// exact proof form for SolveSpace-style projected line-length equality: the
/// true projected length and its square root stay out of topology/proof
/// decisions, following Yap, "Towards Exact Geometric Computation" (1997),
/// while the axes follow Shoemake's unit-quaternion frame formula.
pub(crate) fn projected_direction_squared_length(
    direction: &[Expr; 3],
    quaternion: &[Expr; 4],
) -> Expr {
    let [u, v] = projected_direction2(direction, quaternion);
    u.clone() * u + v.clone() * v
}

/// Build exact projected point-line distance parts in a workplane frame.
///
/// The returned pair is `(cross_uv, |dir_uv|^2)`, where `cross_uv` is the
/// 2D cross product after projecting both vectors to the workplane `U/V`
/// basis. Callers can compare `cross_uv^2` with `distance^2 * |dir_uv|^2`
/// without normalizing the line direction. This is the same denominator-
/// clearing proof style advocated by Yap, "Towards Exact Geometric
/// Computation" (1997), with the retained unit-quaternion frame from
/// Shoemake, "Animating Rotation with Quaternion Curves" (1985).
pub(crate) fn projected_point_line_distance_squared_parts(
    point_delta: &[Expr; 3],
    line_direction: &[Expr; 3],
    quaternion: &[Expr; 4],
) -> (Expr, Expr) {
    let (u_axis, v_axis, _) = quaternion_frame_axes_expr(quaternion);
    let point_u = dot3(point_delta, &u_axis);
    let point_v = dot3(point_delta, &v_axis);
    let line_u = dot3(line_direction, &u_axis);
    let line_v = dot3(line_direction, &v_axis);
    let cross = point_u * line_v.clone() - point_v * line_u.clone();
    let direction_squared = line_u.clone() * line_u + line_v.clone() * line_v;
    (cross, direction_squared)
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
