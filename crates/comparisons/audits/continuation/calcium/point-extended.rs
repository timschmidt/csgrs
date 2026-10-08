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
use std::time::Instant;

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
    let mut visit = |r: &mut Real| match mode {
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
fn run(snapshot: Option<fn(bool) -> [usize; 4]>) {
    let args = std::env::args().collect::<Vec<_>>();
    let mode = args.get(1).map(String::as_str).unwrap_or("check");
    if mode == "check" {
        let mut rows = 0;
        for which in 0..48 {
            for policy in 0..2 {
                for state in 0..4 {
                    let (mut a, mut b, op) = case(which);
                    history(&mut a, state);
                    history(&mut b, state);
                    let before_a = wire_root(&a);
                    let before_b = wire_root(&b);
                    let report = wire_report(&query(&a, &b, op, policy));
                    assert_eq!(before_a, wire_root(&a));
                    assert_eq!(before_b, wire_root(&b));
                    println!(
                        "{}",
                        json!({"type":"query","case":which,"policy":policy,"history":state,
                        "left":before_a,"right":before_b,"report":report})
                    );
                    rows += 1;
                }
            }
        }
        println!(
            "{}",
            json!({"type":"terminal","rows":rows,"cases":48,"policies":2,"histories":4})
        );
        return;
    }
    assert!(mode == "cpu" || mode == "allocation");
    assert_eq!(mode == "allocation", snapshot.is_some());
    let which = args[2].parse::<usize>().unwrap();
    let policy = args[3].parse::<usize>().unwrap();
    let state = args[4].parse::<usize>().unwrap();
    let lifecycle = &args[5];
    let iterations = args[6].parse::<usize>().unwrap();
    assert!(which < 48 && policy < 2 && state < 4 && iterations > 0);
    assert!(lifecycle == "retained" || lifecycle == "fresh");
    let (mut a, mut b, op) = case(which);
    history(&mut a, state);
    history(&mut b, state);
    let expected = wire_report(&query(&a, &b, op, policy));
    for _ in 0..8 {
        black_box(query(&a, &b, op, policy));
    }
    let before = snapshot.map_or([0; 4], |s| s(true));
    let start = Instant::now();
    let mut checksum = 0usize;
    for _ in 0..iterations {
        let report = if lifecycle == "fresh" {
            let (mut a, mut b, op) = case(which);
            history(&mut a, state);
            history(&mut b, state);
            query(&a, &b, op, policy)
        } else {
            query(&a, &b, op, policy)
        };
        checksum = checksum.wrapping_add(
            report
                .representation
                .as_ref()
                .map_or(1, |r| r.polynomial_coefficients.len()),
        );
        black_box(report);
    }
    let elapsed = start.elapsed().as_nanos();
    let after = snapshot.map_or([0; 4], |s| s(false));
    println!(
        "{}",
        json!({"mode":mode,"case":which,"policy":policy,"history":state,"lifecycle":lifecycle,"iterations":iterations,
        "elapsed_ns":elapsed,"checksum":checksum,"expected":expected,
        "requests":after[0]-before[0],"requested_bytes":after[1]-before[1],
        "live_delta":after[2] as i128-before[2] as i128,"peak_delta":after[3]-before[2]})
    );
}
