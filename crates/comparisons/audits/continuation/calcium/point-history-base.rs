use hyperlimit::PredicatePolicy;
use hyperreal::{Rational, Real};
use hypersolve::{
    AlgebraicRootArithmeticOp as Op, AlgebraicRootBinaryTransformReport as Report,
    AlgebraicRootRepresentation as Root, AlgebraicRootValidationReport,
    AlgebraicRootValidationStatus, IsolatedRootInterval, SymbolId,
    transform_algebraic_roots_binary,
};
use serde_json::{Value, json};
use std::hint::black_box;

fn fraction(n: i64, d: u64) -> Real {
    Real::from(Rational::fraction(n, d).unwrap())
}
fn terminal_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    &sine * &sine + &cosine * &cosine - Real::one()
}
fn obscured_positive() -> Real {
    let r = Real::from(2).sqrt().unwrap();
    let r_over_pi = (r.clone() / Real::pi()).unwrap();
    let shift = r.clone() * Real::from(3) + fraction(1, 2);
    let contact = (((r.clone() * Real::from(4) - shift.clone()) * Real::pi()) * r_over_pi.clone()
        / Real::from(4))
    .unwrap();
    let domain = (((r * Real::from(2) - shift) * Real::pi()) * r_over_pi / Real::from(4)).unwrap()
        + Real::one();
    contact - domain + Real::from(2).powi_i64(-3000).unwrap()
}
fn represented(poly: &[i64], lower: Real, upper: Real, exact: Option<Real>) -> Root {
    Root {
        constraint_index: 7,
        symbol: SymbolId(11),
        interval_index: 3,
        polynomial_coefficients: poly.iter().copied().map(Real::from).collect(),
        interval: IsolatedRootInterval {
            lower,
            upper,
            exact_root: exact,
            distinct_root_count: 1,
        },
        validation: AlgebraicRootValidationReport {
            status: AlgebraicRootValidationStatus::Valid,
            message: None,
        },
    }
}
fn point(poly: &[i64], value: Real) -> Root {
    represented(poly, value.clone(), value.clone(), Some(value))
}
fn case(which: usize) -> (Root, Root, Op) {
    let r = Real::from(2).sqrt().unwrap();
    let p2 = || point(&[-2, 0, 1], r.clone());
    let p0 = || point(&[0, 1], Real::zero());
    let p1 = || point(&[-1, 1], Real::one());
    let wide = || represented(&[-2, 0, 1], Real::one(), Real::from(2), None);
    let (a, b) = match which / 4 {
        0 => (p2(), p2()),
        1 => (p2(), point(&[-3, 0, 1], Real::from(3).sqrt().unwrap())),
        2 => (p2(), point(&[-3, 2], fraction(3, 2))),
        3 => (
            represented(&[-1, 1], &r * fraction(1, 2), r.clone(), None),
            p2(),
        ),
        4 => {
            let a = represented(&[-2, 0, 1], &r - fraction(1, 8), &r + fraction(1, 8), None);
            (a.clone(), a)
        }
        5 => (
            represented(&[-2, 0, 1], Real::one(), r.clone(), Some(r.clone())),
            point(&[-3, 2], fraction(3, 2)),
        ),
        6 => {
            let a = represented(&[0, 1], -r.clone(), r.clone(), None);
            (a.clone(), a)
        }
        7 => (p0(), wide()),
        8 => {
            let z = terminal_zero();
            (
                represented(&[-1, 1], &z + fraction(3, 4), &z + fraction(5, 4), None),
                p1(),
            )
        }
        9 => {
            let z = terminal_zero();
            let e = Real::from(2).powi_i64(-3000).unwrap();
            (represented(&[0, 1], &z - &e, &z + &e, None), p0())
        }
        10 => {
            let radius = obscured_positive();
            (represented(&[0, 1], -radius.clone(), radius, None), p0())
        }
        11 => {
            let radius = terminal_zero() + Real::from(2).powi_i64(-3000).unwrap();
            (
                represented(&[-1, 1], Real::one() - &radius, Real::one() + radius, None),
                p1(),
            )
        }
        _ => unreachable!(),
    };
    (
        a,
        b,
        [Op::Add, Op::Subtract, Op::Multiply, Op::Divide][which % 4],
    )
}
fn history(root: &mut Root, mode: usize) {
    let visit = |r: &mut Real| match mode {
        0 => {}
        1 | 2 => {
            black_box(
                r.certified_dyadic_interval(if mode == 1 { -64 } else { -4096 })
                    .unwrap(),
            );
        }
        3 => {
            *r = Real::from_json(&r.to_json()).unwrap();
        }
        _ => unreachable!(),
    };
    visit(&mut root.interval.lower);
    visit(&mut root.interval.upper);
    if let Some(r) = &mut root.interval.exact_root {
        visit(r);
    }
    for r in &mut root.polynomial_coefficients {
        visit(r);
    }
}
fn wire_root(r: &Root) -> Value {
    json!({"constraint":r.constraint_index,"symbol":r.symbol.0,"intervalIndex":r.interval_index,
        "polynomial":r.polynomial_coefficients,"lower":r.interval.lower,"upper":r.interval.upper,
        "exact":r.interval.exact_root,"count":r.interval.distinct_root_count,
        "validation":format!("{:?}",r.validation.status),"validationMessage":r.validation.message})
}
fn wire_report(r: &Report) -> Value {
    json!({"operation":format!("{:?}",r.operation),"status":format!("{:?}",r.status),
        "message":r.message,"root":r.representation.as_ref().map(wire_root)})
}
fn query(a: &Root, b: &Root, op: Op, policy: usize) -> Report {
    transform_algebraic_roots_binary(
        black_box(a),
        black_box(b),
        op,
        [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512][policy],
    )
}
