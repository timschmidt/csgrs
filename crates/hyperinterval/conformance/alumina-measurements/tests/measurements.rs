use alumina_interface_core::{BoundaryError, BoundedMeasurement, Millimetres, Seconds};
use hyperinterval::{IntervalError, RationalInterval};
use hyperlimit::{PredicatePolicy, compare_reals};
use hyperreal::{Rational, Real};
use interval_measurement_conformance::sample;

#[test]
fn independent_fraction_grid_preserves_units_bounds_and_replay() {
    for nominal in -16_i64..=16 {
        for uncertainty in -4_i64..=8 {
            for denominator in [1, 2, 3, 7, 31] {
                let n = Rational::fraction(nominal, denominator).unwrap();
                let u = Rational::fraction(uncertainty, denominator).unwrap();
                let actual = BoundedMeasurement::<Millimetres>::from_nominal_uncertainty(
                    n.clone(),
                    u.clone(),
                );
                let shared = RationalInterval::from_nominal_uncertainty(&n, &u);
                if uncertainty < 0 {
                    assert_eq!(actual, Err(BoundaryError::NegativeUncertainty));
                    assert_eq!(shared, Err(IntervalError::NegativeRadius));
                    continue;
                }
                let actual = actual.unwrap();
                let shared = shared.unwrap();
                // Independent common-denominator expectations: no call to the
                // endpoint implementation or arithmetic in RationalInterval.
                let lo = Rational::fraction(nominal - uncertainty, denominator).unwrap();
                let hi = Rational::fraction(nominal + uncertainty, denominator).unwrap();
                assert_eq!((actual.lower(), actual.upper()), (&lo, &hi));
                assert_eq!(shared.bounds(), Some((&lo, &hi)));
                let encoded = serde_json::to_string(&shared).unwrap();
                let replay: RationalInterval = serde_json::from_str(&encoded).unwrap();
                assert_eq!(shared, replay);
                assert_eq!(serde_json::to_string(&replay).unwrap(), encoded);
                let reused = replay.add(&RationalInterval::point(1.into()));
                assert_eq!(
                    reused.bounds(),
                    Some((&(&lo + Rational::from(1)), &(&hi + Rational::from(1))))
                );
                assert_eq!(BoundedMeasurement::<Millimetres>::unit_symbol(), "mm");
                let seconds =
                    BoundedMeasurement::<Seconds>::from_nominal_uncertainty(n, u).unwrap();
                assert_eq!((seconds.lower(), seconds.upper()), (&lo, &hi));
                assert_eq!(BoundedMeasurement::<Seconds>::unit_symbol(), "s");
            }
        }
    }
}

#[test]
fn large_representations_and_policy_consumption_match() {
    for index in 0..8 {
        let (n, u) = sample(index);
        let actual =
            BoundedMeasurement::<Millimetres>::from_nominal_uncertainty(n.clone(), u.clone());
        if index == 7 {
            assert_eq!(actual, Err(BoundaryError::NegativeUncertainty));
            continue;
        }
        let actual = actual.unwrap();
        let exact = RationalInterval::from_nominal_uncertainty(&n, &u).unwrap();
        assert_eq!(exact.bounds(), Some((actual.lower(), actual.upper())));
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let point = Real::from(n.clone());
            let low = compare_reals(&Real::from(actual.lower().clone()), &point, policy);
            let high = compare_reals(&point, &Real::from(actual.upper().clone()), policy);
            assert_eq!(
                low,
                compare_reals(&Real::from(exact.lower().unwrap().clone()), &point, policy)
            );
            assert_eq!(
                high,
                compare_reals(&point, &Real::from(exact.upper().unwrap().clone()), policy)
            );
            for outcome in [low, high] {
                assert!(matches!(
                    outcome,
                    hyperlimit::PredicateOutcome::Decided {
                        value: std::cmp::Ordering::Less | std::cmp::Ordering::Equal,
                        certainty: hyperlimit::Certainty::Exact,
                        ..
                    }
                ));
            }
        }
    }
}

#[test]
fn domain_error_and_kernel_source_remain_distinct() {
    use std::error::Error;
    assert_eq!(
        BoundaryError::NegativeUncertainty.to_string(),
        "measurement uncertainty is negative"
    );
    assert!(BoundaryError::NegativeUncertainty.source().is_none());
    let kernel = IntervalError::Scalar(hyperreal::Problem::Exhausted);
    let error = BoundaryError::Interval(kernel);
    assert_eq!(error.to_string(), "measurement interval failed");
    let source = error.source().unwrap();
    assert_eq!(source.downcast_ref::<IntervalError>(), Some(&kernel));
    assert_eq!(
        source
            .source()
            .unwrap()
            .downcast_ref::<hyperreal::Problem>(),
        Some(&hyperreal::Problem::Exhausted)
    );
}

#[cfg(feature = "trace")]
#[test]
fn construction_dispatch_counts_match_the_original_formula() {
    use hyperreal::dispatch_trace::{self, with_recording};
    const SIDE: &str = "MEASUREMENT_TRACE_SIDE";
    const INDEX: &str = "MEASUREMENT_TRACE_INDEX";
    const WARM: &str = "MEASUREMENT_TRACE_WARM";
    if let Ok(side) = std::env::var(SIDE) {
        let index = std::env::var(INDEX).unwrap().parse::<usize>().unwrap();
        let (n, u) = sample(index);
        let run = |n: Rational, u: Rational| {
            if side == "original" {
                if u.is_negative() {
                    return Err(BoundaryError::NegativeUncertainty);
                }
                Ok((n.clone() - u.clone(), n + u))
            } else {
                assert_eq!(side, "public");
                BoundedMeasurement::<Millimetres>::from_nominal_uncertainty(n, u)
                    .map(|x| (x.lower().clone(), x.upper().clone()))
            }
        };
        if std::env::var(WARM).unwrap() == "true" {
            let _ = std::hint::black_box(run(n.clone(), u.clone()));
        }
        dispatch_trace::reset();
        let result = with_recording(|| run(n, u));
        let trace = dispatch_trace::take_trace();
        println!("TRACE={trace:?}");
        match result {
            Ok(bounds) => {
                println!("RESULT={}", serde_json::to_string(&bounds).unwrap());
                for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
                    let low = Real::from(bounds.0.clone());
                    let high = Real::from(bounds.1.clone());
                    dispatch_trace::reset();
                    let receipt = with_recording(|| compare_reals(&low, &high, policy));
                    let trace = dispatch_trace::take_trace();
                    assert_eq!(
                        trace.path_count("hyperlimit", "compare_reals", "exact-rational"),
                        1
                    );
                    assert!(matches!(
                        receipt,
                        hyperlimit::PredicateOutcome::Decided {
                            value: std::cmp::Ordering::Less | std::cmp::Ordering::Equal,
                            certainty: hyperlimit::Certainty::Exact,
                            ..
                        }
                    ));
                    println!("POLICY={policy:?} {receipt:?} {trace:?}");
                }
            }
            Err(error) => println!("RESULT={error:?}"),
        }
        return;
    }
    // Small rationals may be interned. Each side and cache phase needs a fresh
    // process: rebuilding equal numbers in one process does not reset facts.
    for index in 0..8 {
        for warm in [false, true] {
            let mut evidence = Vec::new();
            for side in ["original", "public"] {
                let child = std::process::Command::new(std::env::current_exe().unwrap())
                    .args([
                        "--exact",
                        "construction_dispatch_counts_match_the_original_formula",
                        "--nocapture",
                    ])
                    .env(SIDE, side)
                    .env(INDEX, index.to_string())
                    .env(WARM, warm.to_string())
                    .output()
                    .unwrap();
                assert!(
                    child.status.success(),
                    "{}",
                    String::from_utf8_lossy(&child.stderr)
                );
                let output = String::from_utf8(child.stdout).unwrap();
                let lines: Vec<_> = output
                    .lines()
                    .filter(|line| {
                        line.starts_with("TRACE=")
                            || line.starts_with("RESULT=")
                            || line.starts_with("POLICY=")
                    })
                    .map(str::to_owned)
                    .collect();
                assert_eq!(lines.len(), if index == 7 { 2 } else { 4 }, "{output}");
                evidence.push(lines);
            }
            assert_eq!(evidence[0], evidence[1], "fixture {index}, warm={warm}");
        }
    }
}
