//! Exact workplane-symmetry polynomial packages for retained sketch constraints.
//!
//! This module owns the algebra for reflecting two 3D points across a retained
//! workplane. It consumes already-resolved symbolic coordinates and a retained
//! quaternion-like workplane normal, then emits explicit proof rows: a
//! unit-quaternion guard, a midpoint-on-plane equation, and normal-offset
//! cross-product equations. The split follows Yap, "Towards Exact Geometric
//! Computation," *Computational Geometry* 7.1-2 (1997): the workplane remains a
//! source object, while candidate acceptance is decided by exact polynomial
//! replay. The normal axis is derived from the standard unit-quaternion frame
//! matrix described by Shoemake, "Animating Rotation with Quaternion Curves,"
//! *SIGGRAPH Computer Graphics* 19.3 (1985).

use crate::sketch_projection::{quaternion_frame_axes_expr, unit_quaternion_residual};
use crate::symbolic::Expr;

/// Exact residual expressions for a retained 3D workplane reflection.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct WorkplaneSymmetryExprs {
    /// Unit-quaternion guard for the retained workplane normal.
    pub(crate) unit_guard: Expr,
    /// Equation certifying that the midpoint lies on the workplane.
    pub(crate) midpoint_plane: Expr,
    /// Cross-product rows certifying that the point offset is parallel to the
    /// workplane normal axis.
    pub(crate) normal_offset_cross: [Expr; 3],
}

/// Build exact proof expressions for two points mirrored across a workplane.
///
/// For points `a` and `b`, workplane origin `o`, and unit normal `n`, symmetry
/// is certified by `(a + b - 2o) . n == 0` and `(a - b) x n == 0`. The first
/// row puts the midpoint on the plane without constructing a rounded midpoint;
/// the second package proves the connecting segment is normal to the plane
/// without dividing by `|a-b|` or normalizing the quaternion with floats.
pub(crate) fn workplane_point_symmetry_exprs(
    origin: &[Expr; 3],
    quaternion: &[Expr; 4],
    a: &[Expr; 3],
    b: &[Expr; 3],
) -> WorkplaneSymmetryExprs {
    let (_, _, normal_axis) = quaternion_frame_axes_expr(quaternion);
    let doubled_midpoint_delta = [
        a[0].clone() + b[0].clone() - Expr::int(2) * origin[0].clone(),
        a[1].clone() + b[1].clone() - Expr::int(2) * origin[1].clone(),
        a[2].clone() + b[2].clone() - Expr::int(2) * origin[2].clone(),
    ];
    let point_offset = [
        a[0].clone() - b[0].clone(),
        a[1].clone() - b[1].clone(),
        a[2].clone() - b[2].clone(),
    ];

    WorkplaneSymmetryExprs {
        unit_guard: unit_quaternion_residual(quaternion),
        midpoint_plane: dot3(&doubled_midpoint_delta, &normal_axis),
        normal_offset_cross: cross3(&point_offset, &normal_axis),
    }
}

fn dot3(a: &[Expr; 3], b: &[Expr; 3]) -> Expr {
    a[0].clone() * b[0].clone() + a[1].clone() * b[1].clone() + a[2].clone() * b[2].clone()
}

fn cross3(a: &[Expr; 3], b: &[Expr; 3]) -> [Expr; 3] {
    [
        a[1].clone() * b[2].clone() - a[2].clone() * b[1].clone(),
        a[2].clone() * b[0].clone() - a[0].clone() * b[2].clone(),
        a[0].clone() * b[1].clone() - a[1].clone() * b[0].clone(),
    ]
}
