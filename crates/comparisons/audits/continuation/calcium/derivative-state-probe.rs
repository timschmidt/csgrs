use hypercurve::{CurveContext, Point2, RationalBezier2};
use hyperreal::{Rational, Real};
use num::{BigInt, BigRational, One};
use std::sync::{
    Arc, Barrier,
    atomic::{AtomicBool, Ordering},
};

fn real(q: &BigRational) -> Real {
    Real::from(
        Rational::from_bigint_fraction(q.numer().clone(), q.denom().to_biguint().unwrap()).unwrap(),
    )
}
fn curve(scale: &Real) -> RationalBezier2 {
    let half = (scale / Real::from(2)).unwrap();
    RationalBezier2::try_new(
        vec![
            Point2::new(Real::zero(), scale.clone()),
            Point2::new(half.clone(), half),
        ],
        vec![Real::one(), Real::from(2)],
    )
    .unwrap()
}
fn check(curve: &RationalBezier2, scale: &Real, t: &BigRational, max: usize) -> usize {
    let got = curve
        .derivatives_at(&real(t), max, &CurveContext::STRICT)
        .unwrap();
    assert_eq!(got.len(), max);
    let mut factorial = BigInt::one();
    let mut flattened = Vec::new();
    for (i, d) in got.iter().enumerate() {
        let k = i + 1;
        factorial *= k;
        let y = BigRational::from_integer(&factorial * if k % 2 == 0 { 1 } else { -1 })
            / (BigRational::one() + t).pow((k + 1) as i32);
        assert_eq!(d.dx(), &(real(&(-&y)) * scale));
        assert_eq!(d.dy(), &(real(&y) * scale));
        flattened.push(d.dx().clone());
        flattened.push(d.dy().clone());
    }
    let encoded = serde_json::to_string(&flattened).unwrap();
    let decoded: Vec<Real> = serde_json::from_str(&encoded).unwrap();
    assert_eq!(decoded, flattened);
    assert_eq!(serde_json::to_string(&decoded).unwrap(), encoded);
    flattened.len()
}
fn emit(kind: usize, parameter: usize, order: usize, state: &str, coordinates: usize) {
    println!(
        "{}",
        serde_json::json!({"kind":kind,"parameter":parameter,"order":order,"state":state,"coordinates":coordinates,"result":"Equal"})
    );
}
fn main() {
    let mut queries = 0;
    let mut unchanged = 0;
    for kind in 0..4 {
        let scale = match kind {
            0 => Real::from(7),
            1 => Real::pi(),
            2 => Real::from(2).sqrt().unwrap(),
            3 => Real::from(2).ln().unwrap(),
            _ => unreachable!(),
        };
        for (parameter, t) in [
            BigRational::from_integer(0.into()),
            BigRational::one(),
            BigRational::new(1.into(), 3.into()),
        ]
        .into_iter()
        .enumerate()
        {
            for order in [0, 1, 3, 24, 80, 128] {
                let original = serde_json::to_string(&scale).unwrap();
                let c = curve(&scale);
                emit(
                    kind,
                    parameter,
                    order,
                    "fresh",
                    check(&c, &scale, &t, order),
                );
                queries += 1;
                let decoded: Real = serde_json::from_str(&original).unwrap();
                emit(
                    kind,
                    parameter,
                    order,
                    "input-roundtrip",
                    check(&curve(&decoded), &scale, &t, order),
                );
                queries += 1;
                for floor in [-32, -128, -512] {
                    let _ = scale.certified_dyadic_interval(floor).unwrap();
                    emit(
                        kind,
                        parameter,
                        order,
                        &format!("warm{floor}"),
                        check(&c.clone(), &scale, &t, order),
                    );
                    queries += 1;
                }
                let signal = Arc::new(AtomicBool::new(true));
                let mut cancelled = scale.clone();
                cancelled.abort(signal.clone());
                // Cancellation permits invalid numeric observations: discard the
                // cancelled observation and check only after clearing the signal.
                let _ = cancelled.certified_dyadic_interval(-1024);
                signal.store(false, Ordering::Relaxed);
                emit(
                    kind,
                    parameter,
                    order,
                    "after-abort",
                    check(&curve(&cancelled), &scale, &t, order),
                );
                queries += 1;
                let shared = curve(&scale);
                let barrier = Barrier::new(4);
                let results = std::thread::scope(|scope| {
                    let handles: Vec<_> = (0..4)
                        .map(|worker| {
                            let (shared, scale, t, barrier, original) =
                                (&shared, &scale, &t, &barrier, &original);
                            scope.spawn(move || {
                                barrier.wait();
                                (0..4)
                                    .map(|iteration| {
                                        let coordinates = if iteration % 2 == 0 {
                                            check(&shared.clone(), scale, t, order)
                                        } else {
                                            let decoded: Real =
                                                serde_json::from_str(original).unwrap();
                                            check(&curve(&decoded), scale, t, order)
                                        };
                                        (format!("worker{worker}-{iteration}"), coordinates)
                                    })
                                    .collect::<Vec<_>>()
                            })
                        })
                        .collect();
                    handles
                        .into_iter()
                        .flat_map(|h| h.join().unwrap())
                        .collect::<Vec<_>>()
                });
                for (state, coordinates) in results {
                    emit(kind, parameter, order, &state, coordinates);
                    queries += 1;
                }
                assert_eq!(serde_json::to_string(&scale).unwrap(), original);
                unchanged += 1;
            }
        }
    }
    let c = curve(&Real::one());
    let mut domain = 0;
    for t in [Real::from(-1), Real::from(2)] {
        for order in [0, 1, 128] {
            assert!(c.derivatives_at(&t, order, &CurveContext::STRICT).is_err());
            domain += 1;
        }
    }
    assert!(
        c.derivatives_at(&Real::one(), usize::MAX, &CurveContext::STRICT)
            .is_err()
    );
    domain += 1;
    let pole = RationalBezier2::try_new(
        vec![
            Point2::new(Real::zero(), Real::one()),
            Point2::new(Real::one(), Real::zero()),
        ],
        vec![Real::one(), Real::from(-1)],
    )
    .unwrap();
    for order in [0, 1, 128] {
        assert!(
            pole.derivatives_at(
                &real(&BigRational::new(1.into(), 2.into())),
                order,
                &CurveContext::STRICT
            )
            .is_err()
        );
        domain += 1;
    }
    println!(
        "{}",
        serde_json::json!({"suite":"derivative-state","queries":queries,"unchanged_inputs":unchanged,"domain_rejections":domain,"workers_per_case":4})
    );
}
