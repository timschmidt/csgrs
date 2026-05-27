//! Exact port of boolmesh `boolean03::kernel01` and `kernel11` shadows.
//!
//! Legacy boolmesh discovers edge/edge boundary events through a two-axis
//! shadow test: `kernel01::shadows01` first determines whether one endpoint
//! crosses the other edge in projected `x`, interpolates the opposite edge at
//! that `x`, then applies a `y` shadow test.  `kernel11::op` accumulates the
//! two endpoint directions from both source edges and intersects the two
//! retained shadow witnesses when the signed sum is non-zero.  This module
//! ports that algorithm with exact [`hyperreal::Real`] arithmetic; the only
//! structural change is that the former `f64` NaN/infinity fallbacks become
//! explicit zero-denominator branches over exact objects.
//!
//! The tie rule in `shadows` is the boolmesh simulation-of-simplicity hook:
//! equal coordinates compare by the signed expansion direction.  Keeping that
//! symbolic tie separate from arithmetic equality follows Yap, "Towards Exact
//! Geometric Computation," *Computational Geometry* 7.1-2 (1997): exact
//! predicates and combinatorial topology are represented as replayable facts
//! instead of being recovered from rounded coordinates.

#![allow(dead_code)]

use std::cmp::Ordering;

use hyperlimit::{Point2, Point3, compare_reals};

use super::ExactReal;

/// Exact directed halfedge handle used by the `Kernel11` port.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) struct ExactKernel11Halfedge {
    /// Tail vertex in the corresponding point array.
    pub tail: usize,
    /// Head vertex in the corresponding point array.
    pub head: usize,
}

/// Exact four-coordinate edge/edge witness emitted by `Kernel11`.
///
/// This mirrors legacy boolmesh's `Vec4`: `x` and `y` are the projected
/// intersection coordinates, while `p_z` and `q_z` retain the two source-edge
/// heights used by the final shadow test.
#[derive(Clone, Debug, PartialEq)]
pub(super) struct ExactKernel11Intersection {
    /// Projected `x` coordinate.
    pub x: ExactReal,
    /// Projected `y` coordinate.
    pub y: ExactReal,
    /// Interpolated `z` on the first edge.
    pub p_z: ExactReal,
    /// Interpolated `z` on the second edge.
    pub q_z: ExactReal,
}

/// Signed exact `Kernel11` edge/edge shadow contribution.
#[derive(Clone, Debug, PartialEq)]
pub(super) struct ExactKernel11Hit {
    /// Signed contribution accumulated exactly like legacy `s11`.
    pub sign: i32,
    /// Exact `Vec4`-equivalent intersection witness.
    pub point: ExactKernel11Intersection,
}

/// Input package for the exact `Kernel11::op` port.
pub(super) struct ExactKernel11Input<'a> {
    /// First operand points in boolmesh's projected working coordinates.
    pub ps_p: &'a [Point3],
    /// Second operand points in boolmesh's projected working coordinates.
    pub ps_q: &'a [Point3],
    /// First operand halfedges.
    pub hs_p: &'a [ExactKernel11Halfedge],
    /// Second operand halfedges.
    pub hs_q: &'a [ExactKernel11Halfedge],
    /// Exact expansion directions for first operand vertices.
    pub ns_p: &'a [Point3],
    /// Exact expansion directions for second operand vertices.
    pub ns_q: &'a [Point3],
    /// Signed expansion scale.  Only the sign participates in tie decisions.
    pub expand: &'a ExactReal,
}

/// Port of legacy boolmesh `kernel11::Kernel11::op`.
///
/// The implementation intentionally follows the legacy loop structure: first
/// test endpoints of `p` against edge `q`, then endpoints of `q` against edge
/// `p` in reverse mode, accumulate `s11`, and finally run the z-shadow guard
/// on the constructed `Vec4` witness.  Floating-point "nearer endpoint" choices
/// are replaced by exact squared-distance comparisons.
pub(super) fn kernel11_op(
    input: &ExactKernel11Input<'_>,
    p1: usize,
    q1: usize,
) -> Option<ExactKernel11Hit> {
    let p_half = *input.hs_p.get(p1)?;
    let q_half = *input.hs_q.get(q1)?;
    let p0 = [p_half.tail, p_half.head];
    let q0 = [q_half.tail, q_half.head];

    let mut k = 0usize;
    let mut p_rl: [Option<Point3>; 2] = [None, None];
    let mut q_rl: [Option<Point3>; 2] = [None, None];
    let mut shadow = false;
    let mut s11 = 0i32;

    for (index, &vertex) in p0.iter().enumerate() {
        if let Some((s, yz)) = shadows01(
            vertex,
            q1,
            input.ps_p,
            input.ps_q,
            input.hs_q,
            input.ns_p,
            input.ns_q,
            input.expand,
            false,
        ) {
            s11 += s * if index == 0 { -1 } else { 1 };
            if k < 2 && (k == 0 || (s != 0) != shadow) {
                shadow = s != 0;
                let point = input.ps_p.get(vertex)?.clone();
                p_rl[k] = Some(point.clone());
                q_rl[k] = Some(Point3::new(point.x.clone(), yz.x, yz.y));
                k += 1;
            }
        }
    }

    for (index, &vertex) in q0.iter().enumerate() {
        if let Some((s, yz)) = shadows01(
            vertex,
            p1,
            input.ps_q,
            input.ps_p,
            input.hs_p,
            input.ns_q,
            input.ns_p,
            input.expand,
            true,
        ) {
            s11 += s * if index == 0 { -1 } else { 1 };
            if k < 2 && (k == 0 || (s != 0) != shadow) {
                shadow = s != 0;
                let point = input.ps_q.get(vertex)?.clone();
                q_rl[k] = Some(point.clone());
                p_rl[k] = Some(Point3::new(point.x.clone(), yz.x, yz.y));
                k += 1;
            }
        }
    }

    if s11 == 0 {
        return None;
    }

    let point = intersect(
        &p_rl[0].clone()?,
        &p_rl[1].clone()?,
        &q_rl[0].clone()?,
        &q_rl[1].clone()?,
    )?;
    let p_tail = input.ps_p.get(p_half.tail)?;
    let p_head = input.ps_p.get(p_half.head)?;
    let intersection_on_p = Point3::new(point.x.clone(), point.y.clone(), point.p_z.clone());
    let tail_distance = distance_squared(p_tail, &intersection_on_p);
    let head_distance = distance_squared(p_head, &intersection_on_p);
    let direction = if abs_less(&tail_distance, &head_distance)? {
        &input.ns_p.get(p_half.tail)?.z
    } else {
        &input.ns_p.get(p_half.head)?.z
    };
    if !shadows(&point.p_z, &point.q_z, &mul(input.expand, direction))? {
        s11 = 0;
    }
    Some(ExactKernel11Hit { sign: s11, point })
}

/// Port of legacy boolmesh `kernel01::shadows`.
///
/// A strict coordinate order shadows directly.  Equal coordinates use the
/// signed expansion direction (`dir < 0`) exactly like the legacy float code;
/// this is the symbolic tie rule that lets `Kernel11` choose a consistent side
/// for endpoint-on-edge contacts.
pub(super) fn shadows(p: &ExactReal, q: &ExactReal, direction: &ExactReal) -> Option<bool> {
    match compare_reals(p, q).value()? {
        Ordering::Less => Some(true),
        Ordering::Greater => Some(false),
        Ordering::Equal => {
            Some(compare_reals(direction, &ExactReal::from(0)).value()? == Ordering::Less)
        }
    }
}

/// Port of legacy boolmesh `kernel01::shadows01`.
///
/// The returned point stores the interpolated `y,z` pair from the opposite
/// edge at the source endpoint's `x` coordinate.  The signed integer is the
/// same `s01` value consumed by boolmesh `Kernel11::op`.
pub(super) fn shadows01(
    p0: usize,
    q1: usize,
    ps_p: &[Point3],
    ps_q: &[Point3],
    hs_q: &[ExactKernel11Halfedge],
    ns_p: &[Point3],
    ns_q: &[Point3],
    expand: &ExactReal,
    reverse: bool,
) -> Option<(i32, Point2)> {
    let q_half = *hs_q.get(q1)?;
    let q1s = q_half.tail;
    let q1e = q_half.head;
    let p = ps_p.get(p0)?;
    let q_start = ps_q.get(q1s)?;
    let q_end = ps_q.get(q1e)?;

    let mut s01 = if reverse {
        let a = i32::from(shadows(&q_start.x, &p.x, &mul(expand, &ns_q.get(q1s)?.x))?);
        let b = i32::from(shadows(&q_end.x, &p.x, &mul(expand, &ns_q.get(q1e)?.x))?);
        a - b
    } else {
        let direction = mul(expand, &ns_p.get(p0)?.x);
        let a = i32::from(shadows(&p.x, &q_end.x, &direction)?);
        let b = i32::from(shadows(&p.x, &q_start.x, &direction)?);
        a - b
    };

    if s01 == 0 {
        return None;
    }

    let yz01 = interpolate(q_start, q_end, &p.x)?;
    if reverse {
        let start_distance = distance_squared(q_start, p);
        let end_distance = distance_squared(q_end, p);
        let direction = if abs_less(&start_distance, &end_distance)? {
            &ns_q.get(q1s)?.y
        } else {
            &ns_q.get(q1e)?.y
        };
        if !shadows(&yz01.x, &p.y, &mul(expand, direction))? {
            s01 = 0;
        }
    } else if !shadows(&p.y, &yz01.x, &mul(expand, &ns_p.get(p0)?.y))? {
        s01 = 0;
    }

    Some((s01, yz01))
}

/// Exact port of legacy boolmesh `kernel01::interpolate`.
///
/// When the projected edge has zero `x` extent, the legacy float code falls
/// back to the left endpoint after producing an invalid quotient.  The exact
/// port makes that branch explicit: interpolation is underdetermined along
/// this axis, so the retained endpoint witness is the stable boolmesh choice.
pub(super) fn interpolate(left: &Point3, right: &Point3, x: &ExactReal) -> Option<Point2> {
    let dx = sub(&right.x, &left.x);
    if real_is_zero(&dx)? {
        return Some(Point2::new(left.y.clone(), left.z.clone()));
    }
    let lambda = (sub(x, &left.x) / &dx).ok()?;
    Some(Point2::new(
        add(&left.y, &mul(&lambda, &sub(&right.y, &left.y))),
        add(&left.z, &mul(&lambda, &sub(&right.z, &left.z))),
    ))
}

/// Exact port of legacy boolmesh `kernel01::intersect`.
pub(super) fn intersect(
    p_left: &Point3,
    p_right: &Point3,
    q_left: &Point3,
    q_right: &Point3,
) -> Option<ExactKernel11Intersection> {
    let dy_left = sub(&q_left.y, &p_left.y);
    let dy_right = sub(&q_right.y, &p_right.y);
    if compare_reals(&mul(&dy_left, &dy_right), &ExactReal::from(0)).value()? == Ordering::Greater {
        return None;
    }
    let use_left = abs_less(&dy_left, &dy_right)?;
    let denominator = sub(&dy_left, &dy_right);
    let lambda = if real_is_zero(&denominator)? {
        ExactReal::from(0)
    } else if use_left {
        (dy_left.clone() / &denominator).ok()?
    } else {
        (dy_right.clone() / &denominator).ok()?
    };
    let p_dy = sub(&p_right.y, &p_left.y);
    let q_dy = sub(&q_right.y, &q_left.y);
    let use_p = abs_less(&p_dy, &q_dy)?;
    let y_delta = if use_p { p_dy } else { q_dy };
    let y_base = if use_left {
        if use_p { &p_left.y } else { &q_left.y }
    } else if use_p {
        &p_right.y
    } else {
        &q_right.y
    };
    let x_base = if use_left { &p_left.x } else { &p_right.x };
    let p_z_base = if use_left { &p_left.z } else { &p_right.z };
    let q_z_base = if use_left { &q_left.z } else { &q_right.z };
    Some(ExactKernel11Intersection {
        x: add(x_base, &mul(&lambda, &sub(&p_right.x, &p_left.x))),
        y: add(y_base, &mul(&lambda, &y_delta)),
        p_z: add(p_z_base, &mul(&lambda, &sub(&p_right.z, &p_left.z))),
        q_z: add(q_z_base, &mul(&lambda, &sub(&q_right.z, &q_left.z))),
    })
}

#[cfg(feature = "internal-fuzzing")]
pub(super) fn internal_fuzz_probe(selector: u8) -> bool {
    let y_high = ExactReal::from(3 + i64::from(selector % 2));
    let ps_p = vec![
        Point3::new(ExactReal::from(2), ExactReal::from(1), ExactReal::from(0)),
        Point3::new(ExactReal::from(2), y_high, ExactReal::from(0)),
    ];
    let ps_q = vec![
        Point3::new(ExactReal::from(0), ExactReal::from(0), ExactReal::from(4)),
        Point3::new(ExactReal::from(4), ExactReal::from(4), ExactReal::from(4)),
    ];
    let hs_p = vec![ExactKernel11Halfedge { tail: 0, head: 1 }];
    let hs_q = vec![ExactKernel11Halfedge { tail: 0, head: 1 }];
    let ns_p = vec![
        Point3::new(ExactReal::from(1), ExactReal::from(1), ExactReal::from(1)),
        Point3::new(ExactReal::from(1), ExactReal::from(1), ExactReal::from(1)),
    ];
    let ns_q = ns_p.clone();
    let expand = ExactReal::from(1);
    let input = ExactKernel11Input {
        ps_p: &ps_p,
        ps_q: &ps_q,
        hs_p: &hs_p,
        hs_q: &hs_q,
        ns_p: &ns_p,
        ns_q: &ns_q,
        expand: &expand,
    };
    kernel11_op(&input, 0, 0)
        .is_some_and(|hit| hit.sign <= 0 && real_order(&hit.point.x, &ExactReal::from(2)).is_some())
}

fn abs_less(left: &ExactReal, right: &ExactReal) -> Option<bool> {
    Some(compare_reals(&abs(left)?, &abs(right)?).value()? == Ordering::Less)
}

fn abs(value: &ExactReal) -> Option<ExactReal> {
    match compare_reals(value, &ExactReal::from(0)).value()? {
        Ordering::Less => Some(sub(&ExactReal::from(0), value)),
        Ordering::Equal | Ordering::Greater => Some(value.clone()),
    }
}

fn distance_squared(left: &Point3, right: &Point3) -> ExactReal {
    let dx = sub(&left.x, &right.x);
    let dy = sub(&left.y, &right.y);
    let dz = sub(&left.z, &right.z);
    add(&add(&mul(&dx, &dx), &mul(&dy, &dy)), &mul(&dz, &dz))
}

fn real_is_zero(value: &ExactReal) -> Option<bool> {
    Some(compare_reals(value, &ExactReal::from(0)).value()? == Ordering::Equal)
}

fn real_order(left: &ExactReal, right: &ExactReal) -> Option<Ordering> {
    compare_reals(left, right).value()
}

fn add(left: &ExactReal, right: &ExactReal) -> ExactReal {
    left.clone() + right
}

fn sub(left: &ExactReal, right: &ExactReal) -> ExactReal {
    left.clone() - right
}

fn mul(left: &ExactReal, right: &ExactReal) -> ExactReal {
    left.clone() * right
}

#[cfg(test)]
mod tests {
    use super::*;

    fn point(x: i64, y: i64, z: i64) -> Point3 {
        Point3::new(ExactReal::from(x), ExactReal::from(y), ExactReal::from(z))
    }

    fn assert_real_eq(left: &ExactReal, right: i64) {
        assert_eq!(
            compare_reals(left, &ExactReal::from(right)).value(),
            Some(Ordering::Equal)
        );
    }

    #[test]
    fn shadows_uses_exact_order_and_tie_direction() {
        assert!(
            shadows(
                &ExactReal::from(1),
                &ExactReal::from(2),
                &ExactReal::from(0)
            )
            .unwrap()
        );
        assert!(
            !shadows(
                &ExactReal::from(3),
                &ExactReal::from(2),
                &ExactReal::from(0)
            )
            .unwrap()
        );
        assert!(
            shadows(
                &ExactReal::from(2),
                &ExactReal::from(2),
                &ExactReal::from(-1)
            )
            .unwrap()
        );
        assert!(
            !shadows(
                &ExactReal::from(2),
                &ExactReal::from(2),
                &ExactReal::from(0)
            )
            .unwrap()
        );
        assert!(
            !shadows(
                &ExactReal::from(2),
                &ExactReal::from(2),
                &ExactReal::from(1)
            )
            .unwrap()
        );
    }

    #[test]
    fn interpolate_replays_exact_yz_and_zero_x_extent_fallback() {
        let yz = interpolate(&point(0, 0, 10), &point(4, 4, 18), &ExactReal::from(2)).unwrap();
        assert_real_eq(&yz.x, 2);
        assert_real_eq(&yz.y, 14);

        let fallback =
            interpolate(&point(2, 7, 11), &point(2, 9, 13), &ExactReal::from(2)).unwrap();
        assert_real_eq(&fallback.x, 7);
        assert_real_eq(&fallback.y, 11);
    }

    #[test]
    fn shadows01_ports_nonreverse_y_shadow_filter() {
        let ps_p = vec![point(2, 1, 0), point(2, 3, 0)];
        let ps_q = vec![point(0, 0, 0), point(4, 4, 0)];
        let hs_q = vec![ExactKernel11Halfedge { tail: 0, head: 1 }];
        let ns_p = vec![point(1, 1, 1), point(1, 1, 1)];
        let ns_q = ns_p.clone();
        let expand = ExactReal::from(1);

        let (inside, yz) =
            shadows01(0, 0, &ps_p, &ps_q, &hs_q, &ns_p, &ns_q, &expand, false).unwrap();
        assert_eq!(inside, 1);
        assert_real_eq(&yz.x, 2);
        assert_real_eq(&yz.y, 0);

        let (filtered, _) =
            shadows01(1, 0, &ps_p, &ps_q, &hs_q, &ns_p, &ns_q, &expand, false).unwrap();
        assert_eq!(filtered, 0);
    }

    #[test]
    fn intersect_ports_legacy_vec4_witness() {
        let hit = intersect(
            &point(2, 1, 0),
            &point(2, 3, 0),
            &point(2, 2, 4),
            &point(2, 2, 4),
        )
        .unwrap();
        assert_real_eq(&hit.x, 2);
        assert_real_eq(&hit.y, 2);
        assert_real_eq(&hit.p_z, 0);
        assert_real_eq(&hit.q_z, 4);
    }

    #[test]
    fn kernel11_op_accumulates_endpoint_shadows() {
        let ps_p = vec![point(2, 1, 0), point(2, 3, 0)];
        let ps_q = vec![point(0, 0, 4), point(4, 4, 4)];
        let hs_p = vec![ExactKernel11Halfedge { tail: 0, head: 1 }];
        let hs_q = vec![ExactKernel11Halfedge { tail: 0, head: 1 }];
        let ns_p = vec![point(1, 1, 1), point(1, 1, 1)];
        let ns_q = ns_p.clone();
        let expand = ExactReal::from(1);
        let input = ExactKernel11Input {
            ps_p: &ps_p,
            ps_q: &ps_q,
            hs_p: &hs_p,
            hs_q: &hs_q,
            ns_p: &ns_p,
            ns_q: &ns_q,
            expand: &expand,
        };

        let hit = kernel11_op(&input, 0, 0).unwrap();
        assert_eq!(hit.sign, -1);
        assert_real_eq(&hit.point.x, 2);
        assert_real_eq(&hit.point.y, 2);
        assert_real_eq(&hit.point.p_z, 0);
        assert_real_eq(&hit.point.q_z, 4);
    }
}
