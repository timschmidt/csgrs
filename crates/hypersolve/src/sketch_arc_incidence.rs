//! Exact retained circular-arc point-incidence packages for sketch constraints.
//!
//! A point lying on the parent circle is not enough to prove point-on-arc
//! incidence: the retained sweep branch also has to be replayed. Following
//! Yap, "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
//! (1997), this module keeps those branch predicates as explicit proof rows
//! instead of recovering arc membership from sampled angles or primitive
//! tolerances. The split between retained construction objects and exact
//! replay is also the modeling boundary used by Bouma, Fudos, Hoffmann, Cai,
//! and Paige, "A Geometric Constraint Solver," *Computer-Aided Design* 27.6
//! (1995).

use crate::sketch::SketchArcPointSweep;
use crate::symbolic::Expr;

/// Exact residual expressions for a retained point-on-circular-arc relation.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ArcPointIncidenceExprs {
    /// Equations certifying that the arc endpoints lie on the retained radius.
    pub(crate) endpoint_radius: [Expr; 2],
    /// Equation certifying that the candidate point lies on the parent circle.
    pub(crate) point_radius: Expr,
    /// Signed orientation predicate certifying the retained arc sweep branch.
    /// Callers lower this as `>= 0`.
    pub(crate) sweep_branch: Expr,
    /// Signed orientation predicates certifying the point's location on the
    /// selected sweep branch. Callers lower each as `>= 0`.
    pub(crate) point_branch: Vec<Expr>,
}

/// Build exact rows for a retained point lying on a selected circular-arc
/// sweep branch.
///
/// Let `s = start - center`, `e = end - center`, and `p = point - center`.
/// The package contains:
///
/// - `|s|^2 - radius^2 == 0`;
/// - `|e|^2 - radius^2 == 0`;
/// - `|p|^2 - radius^2 == 0`;
/// - a signed `s x e` predicate proving the retained sweep direction/size;
/// - one or two signed point-sector predicates.
///
/// Minor sweeps are a conjunction of two half-planes. Major sweeps are the
/// union of two half-plane regions, so [`SketchArcPointSweep`] deliberately
/// carries an additional half-branch (`AfterStart` or `BeforeEnd`) that makes
/// replay conjunctive and reportable. This is the exact-geometric-computation
/// policy from Yap (1997): branch choices are retained evidence, not solver
/// side effects.
pub(crate) fn arc_point_incidence_exprs(
    center: &[Expr; 2],
    start: &[Expr; 2],
    end: &[Expr; 2],
    point: &[Expr; 2],
    radius: Expr,
    sweep: SketchArcPointSweep,
) -> ArcPointIncidenceExprs {
    let start_radius = [
        start[0].clone() - center[0].clone(),
        start[1].clone() - center[1].clone(),
    ];
    let end_radius = [
        end[0].clone() - center[0].clone(),
        end[1].clone() - center[1].clone(),
    ];
    let point_radius = [
        point[0].clone() - center[0].clone(),
        point[1].clone() - center[1].clone(),
    ];
    let start_cross_end = cross2(&start_radius, &end_radius);
    let start_cross_point = cross2(&start_radius, &point_radius);
    let point_cross_end = cross2(&point_radius, &end_radius);

    ArcPointIncidenceExprs {
        endpoint_radius: [
            squared_norm2(&start_radius) - radius.clone() * radius.clone(),
            squared_norm2(&end_radius) - radius.clone() * radius.clone(),
        ],
        point_radius: squared_norm2(&point_radius) - radius.clone() * radius,
        sweep_branch: signed_for_nonnegative(start_cross_end, sweep.arc_cross_sign()),
        point_branch: sweep
            .point_cross_signs()
            .into_iter()
            .map(|(kind, sign)| {
                let predicate = match kind {
                    ArcPointBranchPredicate::StartToPoint => start_cross_point.clone(),
                    ArcPointBranchPredicate::PointToEnd => point_cross_end.clone(),
                };
                signed_for_nonnegative(predicate, sign)
            })
            .collect(),
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ArcPointBranchPredicate {
    StartToPoint,
    PointToEnd,
}

fn signed_for_nonnegative(expr: Expr, sign: i8) -> Expr {
    if sign >= 0 { expr } else { Expr::zero() - expr }
}

fn cross2(a: &[Expr; 2], b: &[Expr; 2]) -> Expr {
    a[0].clone() * b[1].clone() - a[1].clone() * b[0].clone()
}

fn dot2(a: &[Expr; 2], b: &[Expr; 2]) -> Expr {
    a[0].clone() * b[0].clone() + a[1].clone() * b[1].clone()
}

fn squared_norm2(v: &[Expr; 2]) -> Expr {
    dot2(v, v)
}
