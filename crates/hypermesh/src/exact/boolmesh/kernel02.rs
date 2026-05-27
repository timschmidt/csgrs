//! Exact port of boolmesh `boolean03::kernel02`.
//!
//! Legacy boolmesh uses `Kernel02` as the vertex/face shadow accumulator that
//! feeds both `kernel03` winding and `kernel12` edge/face intersection signs.
//! The algorithm reduces a point/triangle relation to three directed
//! `kernel01::shadows01` edge shadows in projected `x/y`, then runs a final
//! projected `z` shadow guard.  This module ports that control flow directly
//! over exact [`hyperreal::Real`] objects and leaves broad-phase collision
//! enumeration to later workspace wiring.
//!
//! The exact split follows Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7.1-2 (1997): shadow predicates, interpolation
//! witnesses, and signed topology counters are retained as exact replayable
//! decisions.  The loop and sign rules intentionally mirror
//! `boolean03::kernel02` from the boolmesh kernel instead of replacing it with
//! a newly invented point-in-triangle classifier.

#![allow(dead_code)]

use std::cmp::Ordering;

use hyperlimit::{Point3, compare_reals};

use super::ExactReal;
use super::kernel11::{ExactKernel11Halfedge, interpolate, shadows, shadows01};

/// Exact halfedge record needed by the `Kernel02` port.
///
/// Boolmesh `Half::is_forward` is `tail < head`, and the three halfedges for a
/// face can include a backward edge whose forward partner is reached through
/// `pair`.  Keeping `pair` here lets `Kernel02` reproduce the legacy
/// `q1_f = if half.is_forward() { q1 } else { half.pair }` branch exactly.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) struct ExactKernel02Halfedge {
    /// Tail vertex in the point/normal arrays.
    pub tail: usize,
    /// Head vertex in the point/normal arrays.
    pub head: usize,
    /// Opposite directed halfedge index.
    pub pair: usize,
}

impl ExactKernel02Halfedge {
    fn is_forward(self) -> bool {
        self.tail < self.head
    }

    fn kernel11(self) -> ExactKernel11Halfedge {
        ExactKernel11Halfedge {
            tail: self.tail,
            head: self.head,
        }
    }
}

/// Input package for the exact `Kernel02::op` port.
pub(super) struct ExactKernel02Input<'a> {
    /// Source vertex points in boolmesh projected working coordinates.
    pub ps_p: &'a [Point3],
    /// Opposite face points in boolmesh projected working coordinates.
    pub ps_q: &'a [Point3],
    /// Opposite mesh halfedges, grouped three per face like legacy boolmesh.
    pub hs_q: &'a [ExactKernel02Halfedge],
    /// Exact expansion directions for source vertices.
    pub ns_p: &'a [Point3],
    /// Exact expansion directions for opposite-face vertices.
    pub ns_q: &'a [Point3],
    /// Signed expansion scale.  Only the sign is used by equal-coordinate ties.
    pub expand: &'a ExactReal,
    /// Legacy direction flag: `true` for `p` against `q`, `false` for reverse.
    pub fwd: bool,
}

/// Signed exact vertex/face shadow contribution.
#[derive(Clone, Debug, PartialEq)]
pub(super) struct ExactKernel02Hit {
    /// Signed contribution accumulated exactly like legacy `s02`.
    pub sign: i32,
    /// Final interpolated face height used by `Kernel12` witness construction.
    pub z: ExactReal,
}

/// Port of legacy boolmesh `kernel02::Kernel02::op`.
///
/// The method visits the three halfedges of face `q2`, lowers backward face
/// halfedges to their forward partner for `shadows01`, accumulates the legacy
/// signed crossing count, and then applies the `z` shadow guard.  Exact
/// arithmetic removes the old NaN/infinity checks; the only underdetermined
/// interpolation case is handled in `kernel01::interpolate` exactly as the
/// boolmesh fallback endpoint choice.
pub(super) fn kernel02_op(
    input: &ExactKernel02Input<'_>,
    p0: usize,
    q2: usize,
) -> Option<ExactKernel02Hit> {
    let mut s02 = 0i32;
    let mut k = 0usize;
    let mut yzz_rl: [Option<Point3>; 2] = [None, None];
    let mut shadows_state = false;
    let mut closest_vid = None::<usize>;
    let mut min_metric = None::<ExactReal>;

    let pos_p = input.ps_p.get(p0)?;
    let shadow_halfedges = input
        .hs_q
        .iter()
        .copied()
        .map(ExactKernel02Halfedge::kernel11)
        .collect::<Vec<_>>();

    for i in 0..3 {
        let q1 = 3 * q2 + i;
        let half = *input.hs_q.get(q1)?;
        let q1_f = if half.is_forward() { q1 } else { half.pair };
        let forward_half = *input.hs_q.get(q1_f)?;

        if !input.fwd {
            let q_vert = forward_half.tail;
            let metric = distance_squared(pos_p, input.ps_q.get(q_vert)?);
            if min_metric
                .as_ref()
                .and_then(|current| real_less(&metric, current))
                .unwrap_or(true)
            {
                min_metric = Some(metric);
                closest_vid = Some(q_vert);
            }
        }

        let Some((s01, yz01)) = shadows01(
            p0,
            q1_f,
            input.ps_p,
            input.ps_q,
            &shadow_halfedges,
            input.ns_p,
            input.ns_q,
            input.expand,
            !input.fwd,
        ) else {
            continue;
        };

        s02 += s01
            * if input.fwd == half.is_forward() {
                -1
            } else {
                1
            };
        if k < 2 && (k == 0 || (s01 != 0) != shadows_state) {
            shadows_state = s01 != 0;
            yzz_rl[k] = Some(Point3::new(yz01.x, yz01.y.clone(), yz01.y));
            k += 1;
        }
    }

    if s02 == 0 {
        return None;
    }

    let left = yzz_rl[0].clone()?;
    let right = yzz_rl[1].clone()?;
    let z02 = interpolate(&left, &right, &pos_p.y)?.y;
    if input.fwd {
        if !shadows(&pos_p.z, &z02, &mul(input.expand, &input.ns_p.get(p0)?.z))? {
            s02 = 0;
        }
    } else {
        let closest_vid = closest_vid?;
        if !shadows(
            &z02,
            &pos_p.z,
            &mul(input.expand, &input.ns_q.get(closest_vid)?.z),
        )? {
            s02 = 0;
        }
    }

    Some(ExactKernel02Hit { sign: s02, z: z02 })
}

#[cfg(feature = "internal-fuzzing")]
pub(super) fn internal_fuzz_probe(selector: u8) -> bool {
    let lift = ExactReal::from(4 + i64::from(selector % 2));
    let ps_p = vec![point(1, 1, 0), point(3, 3, 0)];
    let ps_q = vec![
        Point3::new(ExactReal::from(0), ExactReal::from(0), lift.clone()),
        Point3::new(ExactReal::from(4), ExactReal::from(0), lift.clone()),
        Point3::new(ExactReal::from(0), ExactReal::from(4), lift),
    ];
    let hs_q = triangle_halfedges();
    let ns_p = vec![point(1, 1, 1), point(1, 1, 1)];
    let ns_q = vec![point(1, 1, 1), point(1, 1, 1), point(1, 1, 1)];
    let expand = ExactReal::from(1);
    let input = ExactKernel02Input {
        ps_p: &ps_p,
        ps_q: &ps_q,
        hs_q: &hs_q,
        ns_p: &ns_p,
        ns_q: &ns_q,
        expand: &expand,
        fwd: true,
    };

    let inside = kernel02_op(&input, 0, 0)
        .is_some_and(|hit| hit.sign == 1 && real_order(&hit.z, &ExactReal::from(4)).is_some());
    let outside = kernel02_op(&input, 1, 0).is_none();
    inside && outside
}

fn triangle_halfedges() -> Vec<ExactKernel02Halfedge> {
    vec![
        ExactKernel02Halfedge {
            tail: 0,
            head: 1,
            pair: 5,
        },
        ExactKernel02Halfedge {
            tail: 1,
            head: 2,
            pair: 4,
        },
        ExactKernel02Halfedge {
            tail: 2,
            head: 0,
            pair: 3,
        },
        ExactKernel02Halfedge {
            tail: 0,
            head: 2,
            pair: 2,
        },
        ExactKernel02Halfedge {
            tail: 2,
            head: 1,
            pair: 1,
        },
        ExactKernel02Halfedge {
            tail: 1,
            head: 0,
            pair: 0,
        },
    ]
}

fn point(x: i64, y: i64, z: i64) -> Point3 {
    Point3::new(ExactReal::from(x), ExactReal::from(y), ExactReal::from(z))
}

fn distance_squared(left: &Point3, right: &Point3) -> ExactReal {
    let dx = sub(&left.x, &right.x);
    let dy = sub(&left.y, &right.y);
    let dz = sub(&left.z, &right.z);
    add(&add(&mul(&dx, &dx), &mul(&dy, &dy)), &mul(&dz, &dz))
}

fn real_less(left: &ExactReal, right: &ExactReal) -> Option<bool> {
    Some(compare_reals(left, right).value()? == Ordering::Less)
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

    fn assert_real_eq(left: &ExactReal, right: i64) {
        assert_eq!(
            compare_reals(left, &ExactReal::from(right)).value(),
            Some(Ordering::Equal)
        );
    }

    #[test]
    fn kernel02_ports_forward_vertex_face_shadow() {
        let ps_p = vec![point(1, 1, 0), point(3, 3, 0)];
        let ps_q = vec![point(0, 0, 4), point(4, 0, 4), point(0, 4, 4)];
        let hs_q = triangle_halfedges();
        let ns_p = vec![point(1, 1, 1), point(1, 1, 1)];
        let ns_q = vec![point(1, 1, 1), point(1, 1, 1), point(1, 1, 1)];
        let expand = ExactReal::from(1);
        let input = ExactKernel02Input {
            ps_p: &ps_p,
            ps_q: &ps_q,
            hs_q: &hs_q,
            ns_p: &ns_p,
            ns_q: &ns_q,
            expand: &expand,
            fwd: true,
        };

        let inside = kernel02_op(&input, 0, 0).unwrap();
        assert_eq!(inside.sign, 1);
        assert_real_eq(&inside.z, 4);
        assert!(kernel02_op(&input, 1, 0).is_none());
    }

    #[test]
    fn kernel02_ports_reverse_vertex_face_shadow() {
        let ps_p = vec![point(1, 1, 5), point(1, 1, 3)];
        let ps_q = vec![point(0, 0, 4), point(4, 0, 4), point(0, 4, 4)];
        let hs_q = triangle_halfedges();
        let ns_p = vec![point(1, 1, 1), point(1, 1, 1)];
        let ns_q = vec![point(1, 1, 1), point(1, 1, 1), point(1, 1, 1)];
        let expand = ExactReal::from(1);
        let input = ExactKernel02Input {
            ps_p: &ps_p,
            ps_q: &ps_q,
            hs_q: &hs_q,
            ns_p: &ns_p,
            ns_q: &ns_q,
            expand: &expand,
            fwd: false,
        };

        let above = kernel02_op(&input, 0, 0).unwrap();
        assert_eq!(above.sign, 1);
        assert_real_eq(&above.z, 4);
        let below = kernel02_op(&input, 1, 0).unwrap();
        assert_eq!(below.sign, 0);
        assert_real_eq(&below.z, 4);
    }

    #[test]
    fn kernel02_preserves_forward_pair_for_backward_face_halfedge() {
        let ps_p = vec![point(1, 1, 0)];
        let ps_q = vec![point(0, 0, 4), point(4, 0, 4), point(0, 4, 4)];
        let mut hs_q = triangle_halfedges();
        hs_q[2].pair = usize::MAX;
        let ns_p = vec![point(1, 1, 1)];
        let ns_q = vec![point(1, 1, 1), point(1, 1, 1), point(1, 1, 1)];
        let expand = ExactReal::from(1);
        let input = ExactKernel02Input {
            ps_p: &ps_p,
            ps_q: &ps_q,
            hs_q: &hs_q,
            ns_p: &ns_p,
            ns_q: &ns_q,
            expand: &expand,
            fwd: true,
        };

        assert!(
            kernel02_op(&input, 0, 0).is_none(),
            "a malformed backward halfedge pair must not invent a shadow"
        );
    }
}
