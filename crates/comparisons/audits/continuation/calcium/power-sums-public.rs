use hyperlimit::PredicatePolicy;
use hyperreal::{Rational, Real};
use hypersolve::{
    AlgebraicRootArithmeticOp as Op, AlgebraicRootBinaryTransformReport,
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
fn coefficients(i: usize) -> Vec<i64> {
    match i {
        0 => vec![-1, 1],
        1 => vec![1, 2],
        2 => vec![0, 1],
        3 => vec![-2, 0, 1],
        4 => vec![-3, 0, 1],
        5 => vec![1, -2, 1],
        6 => vec![0, -1, 1],
        7 => vec![-2, 1, 1],
        8 => vec![-2, 0, 0, 1],
        9 => vec![-3, 0, 0, 2],
        10 => vec![-1, 3, -3, 1],
        11 => vec![0, -2, 0, 1],
        12 => vec![2, -3, 0, 1],
        13 => vec![1, 0, -10, 0, 1],
        14 => vec![4, 0, -4, 0, 1],
        15 => vec![0, 0, 1],
        16 => vec![-2, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        17 => vec![-7, 0, 0, 0, 0, 0, 0, 1],
        18 => vec![6, -5, 1],
        19 => vec![0, 1, 0, 1],
        _ => unreachable!(),
    }
}
fn root(i: usize, scale: i64, denominator: u64) -> Root {
    let (lo, hi, exact) = match i {
        0 => (fraction(1, 1), fraction(1, 1), Some(fraction(1, 1))),
        1 => (fraction(-1, 2), fraction(-1, 2), Some(fraction(-1, 2))),
        2 | 15 | 19 => (Real::zero(), Real::zero(), Some(Real::zero())),
        3 | 11 | 14 => (fraction(7, 5), fraction(3, 2), None),
        4 => (fraction(5, 3), fraction(7, 4), None),
        5 | 6 | 7 | 10 | 12 => (fraction(3, 4), fraction(5, 4), None),
        8 => (fraction(5, 4), fraction(4, 3), None),
        9 => (fraction(11, 10), fraction(6, 5), None),
        13 => (fraction(31, 10), fraction(16, 5), None),
        16 => (fraction(1, 1), fraction(11, 10), None),
        17 => (fraction(13, 10), fraction(7, 5), None),
        18 => (fraction(19, 10), fraction(21, 10), None),
        _ => unreachable!(),
    };
    Root {
        constraint_index: i,
        symbol: SymbolId(i as u32),
        interval_index: 0,
        polynomial_coefficients: coefficients(i)
            .into_iter()
            .map(|c| fraction(c * scale, denominator))
            .collect(),
        interval: IsolatedRootInterval {
            lower: lo,
            upper: hi,
            exact_root: exact,
            distinct_root_count: 1,
        },
        validation: AlgebraicRootValidationReport {
            status: AlgebraicRootValidationStatus::Valid,
            message: None,
        },
    }
}
fn operation(i: usize) -> Op {
    [Op::Add, Op::Subtract, Op::Multiply, Op::Divide][i]
}
fn wire_real(r: &Real) -> String {
    let q = r
        .exact_rational_ref()
        .expect("rational coefficient or endpoint");
    format!(
        "{}{}/{}",
        if q.is_negative() { "-" } else { "" },
        q.numerator(),
        q.denominator()
    )
}
fn wire_root(r: &Root) -> Value {
    json!({"constraint":r.constraint_index,"symbol":r.symbol.0,"intervalIndex":r.interval_index,
    "polynomial":r.polynomial_coefficients.iter().map(wire_real).collect::<Vec<_>>(),
    "lower":wire_real(&r.interval.lower),"upper":wire_real(&r.interval.upper),
    "exact":r.interval.exact_root.as_ref().map(wire_real),"count":r.interval.distinct_root_count,
    "validation":format!("{:?}",r.validation.status),"validationMessage":r.validation.message})
}
fn wire_report(r: &AlgebraicRootBinaryTransformReport) -> Value {
    json!({"status":format!("{:?}",r.status),
    "operation":format!("{:?}",r.operation),"message":r.message,"root":r.representation.as_ref().map(wire_root)})
}
fn query(a: &Root, b: &Root, op: Op) -> AlgebraicRootBinaryTransformReport {
    transform_algebraic_roots_binary(black_box(a), black_box(b), op, PredicatePolicy::STRICT)
}
fn case(which: usize) -> (Root, Root, Op) {
    // Includes cheap rational, quadratic, cubic, nonmonic, repeated/square-free,
    // unused-zero divisor, degree-nine, unsupported-degree and zero guards.
    let pairs = [
        (0, 1),
        (3, 4),
        (8, 9),
        (13, 3),
        (14, 14),
        (3, 6),
        (16, 0),
        (17, 3),
        (3, 2),
        (11, 12),
    ];
    let (i, j) = pairs[which / 4];
    (root(i, -3, 7), root(j, 5, 11), operation(which % 4))
}
fn run(snapshot: Option<fn(bool) -> [usize; 4]>) {
    let args = std::env::args().collect::<Vec<_>>();
    let mode = args.get(1).map(String::as_str).unwrap_or("check");
    if mode == "check" {
        let mut rows = 0;
        for i in 0..20 {
            for j in 0..20 {
                for op in 0..4 {
                    for (scale, (ls, rs)) in
                        [(1, 1), (-1, 1), (1, -1), (-1, -1)].into_iter().enumerate()
                    {
                        let a = root(i, ls, 3);
                        let b = root(j, rs, 5);
                        let before_a = wire_root(&a);
                        let before_b = wire_root(&b);
                        let report = query(&a, &b, operation(op));
                        assert_eq!(wire_root(&a), before_a);
                        assert_eq!(wire_root(&b), before_b);
                        println!(
                            "{}",
                            json!({"type":"public","i":i,"j":j,"op":op,"scale":scale,"left":before_a,"right":before_b,"report":wire_report(&report)})
                        );
                        rows += 1;
                    }
                }
            }
        }
        for which in 0..40 {
            let (a, b, op) = case(which);
            let report = query(&a, &b, op);
            println!(
                "{}",
                json!({"type":"cost-case","case":which,"left":wire_root(&a),"right":wire_root(&b),"report":wire_report(&report)})
            );
        }
        println!(
            "{}",
            json!({"type":"terminal","publicRows":rows,"costCases":40})
        );
        return;
    }
    assert!(mode == "cpu" || mode == "allocation");
    assert_eq!(mode == "allocation", snapshot.is_some());
    let which = args[2].parse::<usize>().unwrap();
    let lifecycle = &args[3];
    let iterations = args[4].parse::<usize>().unwrap();
    assert!(which < 40 && iterations > 0);
    assert!(lifecycle == "retained" || lifecycle == "fresh");
    let (a, b, op) = case(which);
    let expected = wire_report(&query(&a, &b, op));
    for _ in 0..8 {
        black_box(query(&a, &b, op));
    }
    let before = snapshot.map_or([0; 4], |s| s(true));
    let start = Instant::now();
    let mut checksum = 0usize;
    for _ in 0..iterations {
        let report = if lifecycle == "fresh" {
            let (a, b, op) = case(which);
            query(&a, &b, op)
        } else {
            query(&a, &b, op)
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
        json!({"mode":mode,"case":which,"lifecycle":lifecycle,"iterations":iterations,
        "elapsed_ns":elapsed,"checksum":checksum,"expected":expected,
        "requests":after[0]-before[0],"requested_bytes":after[1]-before[1],
        "live_delta":after[2] as i128-before[2] as i128,"peak_delta":after[3]-before[2]})
    );
}
