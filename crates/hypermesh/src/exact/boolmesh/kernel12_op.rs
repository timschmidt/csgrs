//! Exact port of boolmesh `boolean03::kernel12::Kernel12::op`.
//!
//! Legacy boolmesh computes one edge/face row by combining two endpoint
//! vertex/face shadows from `Kernel02` with three opposite-face edge/edge
//! shadows from `Kernel11`.  The signed accumulator becomes `x12`/`x21`; the
//! two retained shadow witnesses are intersected to construct `v12`/`v21`.
//! This module ports that join point directly over exact
//! [`hyperreal::Real`] objects while leaving broad-phase face-pair enumeration
//! and workspace table insertion to the surrounding exact boolmesh stages.
//!
//! The retained witness construction follows Yap, "Towards Exact Geometric
//! Computation," *Computational Geometry* 7.1-2 (1997): exact predicates and
//! constructed coordinates remain replayable artifacts, and no primitive-float
//! tolerance is used to recover topology.  The control flow and sign rules
//! intentionally mirror boolmesh `boolean03::kernel12`, which is the published
//! kernel path this port is converging toward.

#![allow(dead_code)]

use std::cmp::Ordering;

use hyperlimit::{Point3, compare_reals};

use super::ExactReal;
use super::kernel02::{ExactKernel02Halfedge, ExactKernel02Input, kernel02_op};
use super::kernel11::{ExactKernel11Halfedge, ExactKernel11Input, intersect, kernel11_op};

/// Input package for exact `Kernel12::op`.
///
/// `ps_p`/`hs_p` and `ps_q`/`hs_q` are the canonical boolmesh operand order
/// used by legacy `intersect12`: forward calls use `p` as the source edge and
/// `q` as the opposite face; reverse calls swap the source/opposite view but
/// still call `Kernel11` with canonical operand indices.
pub(super) struct ExactKernel12Input<'a> {
    /// Canonical first operand points in boolmesh projected working coordinates.
    pub ps_p: &'a [Point3],
    /// Canonical second operand points in boolmesh projected working coordinates.
    pub ps_q: &'a [Point3],
    /// Canonical first operand halfedges.
    pub hs_p: &'a [ExactKernel02Halfedge],
    /// Canonical second operand halfedges.
    pub hs_q: &'a [ExactKernel02Halfedge],
    /// Exact expansion directions for canonical first operand vertices.
    pub ns_p: &'a [Point3],
    /// Exact expansion directions for canonical second operand vertices.
    pub ns_q: &'a [Point3],
    /// Signed expansion scale used by equal-coordinate shadow ties.
    pub expand: &'a ExactReal,
    /// Legacy direction flag: `true` emits `p1q2`, `false` emits `p2q1`.
    pub fwd: bool,
}

/// Signed exact `Kernel12` edge/face contribution.
#[derive(Clone, Debug, PartialEq)]
pub(super) struct ExactKernel12Hit {
    /// Signed event multiplicity accumulated exactly like legacy `x12`.
    pub sign: i32,
    /// Exact intersection witness emitted to `v12`/`v21`.
    pub point: Point3,
}

/// Port of legacy boolmesh `Kernel12::op`.
///
/// The two endpoint `Kernel02` contributions are accumulated first, then the
/// three opposite-face halfedges contribute `Kernel11` shadows.  The retained
/// witness arrays use the legacy `xzy` coordinate trick: source point `y/z`
/// are swapped so the final `kernel01::intersect` call can reuse the same
/// two-polyline intersection primitive that `Kernel11` uses.
pub(super) fn kernel12_op(
    input: &ExactKernel12Input<'_>,
    p1: usize,
    q2: usize,
) -> Option<ExactKernel12Hit> {
    let source_ps = if input.fwd { input.ps_p } else { input.ps_q };
    let opposite_ps = if input.fwd { input.ps_q } else { input.ps_p };
    let source_hs = if input.fwd { input.hs_p } else { input.hs_q };
    let opposite_hs = if input.fwd { input.hs_q } else { input.hs_p };
    let source_ns = if input.fwd { input.ns_p } else { input.ns_q };
    let opposite_ns = if input.fwd { input.ns_q } else { input.ns_p };

    let source_half = *source_hs.get(p1)?;
    let mut x12 = 0i32;
    let mut xzy_lr0: [Option<Point3>; 2] = [None, None];
    let mut xzy_lr1: [Option<Point3>; 2] = [None, None];
    let mut shadow = false;
    let mut k = 0usize;

    let k02 = ExactKernel02Input {
        ps_p: source_ps,
        ps_q: opposite_ps,
        hs_q: opposite_hs,
        ns_p: source_ns,
        ns_q: opposite_ns,
        expand: input.expand,
        fwd: input.fwd,
    };

    for vertex in [source_half.tail, source_half.head] {
        if let Some(hit) = kernel02_op(&k02, vertex, q2) {
            let forward_endpoint = (vertex == source_half.tail) == input.fwd;
            x12 += hit.sign * if forward_endpoint { 1 } else { -1 };
            if k < 2 && (k == 0 || (hit.sign != 0) != shadow) {
                shadow = hit.sign != 0;
                let source = source_ps.get(vertex)?;
                let xzy = Point3::new(source.x.clone(), source.z.clone(), source.y.clone());
                xzy_lr0[k] = Some(xzy.clone());
                xzy_lr1[k] = Some(Point3::new(xzy.x, hit.z, xzy.z));
                k += 1;
            }
        }
    }

    let hs_p11 = input
        .hs_p
        .iter()
        .copied()
        .map(kernel11_halfedge)
        .collect::<Vec<_>>();
    let hs_q11 = input
        .hs_q
        .iter()
        .copied()
        .map(kernel11_halfedge)
        .collect::<Vec<_>>();
    let k11 = ExactKernel11Input {
        ps_p: input.ps_p,
        ps_q: input.ps_q,
        hs_p: &hs_p11,
        hs_q: &hs_q11,
        ns_p: input.ns_p,
        ns_q: input.ns_q,
        expand: input.expand,
    };

    for i in 0..3 {
        let q1 = 3 * q2 + i;
        let half = *opposite_hs.get(q1)?;
        let q1f = if is_forward(half) { q1 } else { half.pair };
        opposite_hs.get(q1f)?;
        let op = if input.fwd {
            kernel11_op(&k11, p1, q1f)
        } else {
            kernel11_op(&k11, q1f, p1)
        };
        if let Some(hit) = op {
            x12 -= hit.sign * if is_forward(half) { 1 } else { -1 };
            if k < 2 && (k == 0 || (hit.sign != 0) != shadow) {
                shadow = hit.sign != 0;
                let mut first = Point3::new(
                    hit.point.x.clone(),
                    hit.point.p_z.clone(),
                    hit.point.y.clone(),
                );
                let mut second = Point3::new(hit.point.x, hit.point.q_z, hit.point.y);
                if !input.fwd {
                    std::mem::swap(&mut first.y, &mut second.y);
                }
                xzy_lr0[k] = Some(first);
                xzy_lr1[k] = Some(second);
                k += 1;
            }
        }
    }

    if x12 == 0 {
        return None;
    }

    let xzyy = intersect(
        &xzy_lr0[0].clone()?,
        &xzy_lr0[1].clone()?,
        &xzy_lr1[0].clone()?,
        &xzy_lr1[1].clone()?,
    )?;
    Some(ExactKernel12Hit {
        sign: x12,
        point: Point3::new(xzyy.x, xzyy.p_z, xzyy.y),
    })
}

#[cfg(feature = "internal-fuzzing")]
pub(super) fn internal_fuzz_probe(selector: u8) -> bool {
    let top = ExactReal::from(5 + i64::from(selector % 2));
    let ps_p = vec![
        point(1, 1, 0),
        Point3::new(ExactReal::from(1), ExactReal::from(1), top),
    ];
    let ps_q = vec![point(0, 0, 4), point(4, 0, 4), point(0, 4, 4)];
    let hs_p = vec![ExactKernel02Halfedge {
        tail: 0,
        head: 1,
        pair: usize::MAX,
    }];
    let hs_q = triangle_halfedges();
    let ns_p = vec![point(1, 1, 1), point(1, 1, 1)];
    let ns_q = vec![point(1, 1, 1), point(1, 1, 1), point(1, 1, 1)];
    let expand = ExactReal::from(1);
    let input = ExactKernel12Input {
        ps_p: &ps_p,
        ps_q: &ps_q,
        hs_p: &hs_p,
        hs_q: &hs_q,
        ns_p: &ns_p,
        ns_q: &ns_q,
        expand: &expand,
        fwd: true,
    };

    kernel12_op(&input, 0, 0).is_some_and(|hit| {
        hit.sign == 1
            && real_eq(&hit.point.x, 1)
            && real_eq(&hit.point.y, 1)
            && real_eq(&hit.point.z, 4)
    })
}

fn kernel11_halfedge(halfedge: ExactKernel02Halfedge) -> ExactKernel11Halfedge {
    ExactKernel11Halfedge {
        tail: halfedge.tail,
        head: halfedge.head,
    }
}

fn is_forward(halfedge: ExactKernel02Halfedge) -> bool {
    halfedge.tail < halfedge.head
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

fn real_eq(value: &ExactReal, expected: i64) -> bool {
    compare_reals(value, &ExactReal::from(expected)).value() == Some(Ordering::Equal)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_real_eq(left: &ExactReal, right: i64) {
        assert!(real_eq(left, right), "expected exact real to equal {right}");
    }

    #[test]
    fn kernel12_op_accumulates_kernel02_endpoint_witnesses() {
        let ps_p = vec![point(1, 1, 0), point(1, 1, 5)];
        let ps_q = vec![point(0, 0, 4), point(4, 0, 4), point(0, 4, 4)];
        let hs_p = vec![ExactKernel02Halfedge {
            tail: 0,
            head: 1,
            pair: usize::MAX,
        }];
        let hs_q = triangle_halfedges();
        let ns_p = vec![point(1, 1, 1), point(1, 1, 1)];
        let ns_q = vec![point(1, 1, 1), point(1, 1, 1), point(1, 1, 1)];
        let expand = ExactReal::from(1);
        let input = ExactKernel12Input {
            ps_p: &ps_p,
            ps_q: &ps_q,
            hs_p: &hs_p,
            hs_q: &hs_q,
            ns_p: &ns_p,
            ns_q: &ns_q,
            expand: &expand,
            fwd: true,
        };

        let hit = kernel12_op(&input, 0, 0).unwrap();
        assert_eq!(hit.sign, 1);
        assert_real_eq(&hit.point.x, 1);
        assert_real_eq(&hit.point.y, 1);
        assert_real_eq(&hit.point.z, 4);
    }

    #[test]
    fn kernel12_op_drops_zero_sum_endpoint_witnesses() {
        let ps_p = vec![point(1, 1, 0), point(1, 1, 0)];
        let ps_q = vec![point(0, 0, 4), point(4, 0, 4), point(0, 4, 4)];
        let hs_p = vec![ExactKernel02Halfedge {
            tail: 0,
            head: 1,
            pair: usize::MAX,
        }];
        let hs_q = triangle_halfedges();
        let ns_p = vec![point(1, 1, 1), point(1, 1, 1)];
        let ns_q = vec![point(1, 1, 1), point(1, 1, 1), point(1, 1, 1)];
        let expand = ExactReal::from(1);
        let input = ExactKernel12Input {
            ps_p: &ps_p,
            ps_q: &ps_q,
            hs_p: &hs_p,
            hs_q: &hs_q,
            ns_p: &ns_p,
            ns_q: &ns_q,
            expand: &expand,
            fwd: true,
        };

        assert!(kernel12_op(&input, 0, 0).is_none());
    }
}
