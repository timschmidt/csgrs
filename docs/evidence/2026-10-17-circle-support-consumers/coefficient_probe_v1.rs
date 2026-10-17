//! Identical private-kernel diagnostic for shared large exact coefficients.
use super::*;
use hyperlimit::PredicatePolicy;
use num::BigInt;
use std::time::Instant;

fn concentric(center: &Point2, radius: i32) -> Curve2 {
    let start = Point2::new(center.x() + Real::from(radius), center.y().clone());
    CircularArc2::try_from_center(start.clone(), start, center.clone(), false)
        .unwrap()
        .into()
}

#[test]
#[ignore = "explicit coefficient reuse diagnostic; excluded from semantic corpus totals"]
fn shared_coefficient_lookup_cost() {
    let bits: usize = std::env::var("CIRCLE_SHARED_BITS").unwrap().parse().unwrap();
    let repeats: usize = std::env::var("CIRCLE_SHARED_REPEATS").unwrap().parse().unwrap();
    assert!([64, 65_536, 1_048_576].contains(&bits));
    assert_eq!(repeats, 20_000);
    let x = Real::from(hyperreal::Rational::from_bigint(
        (BigInt::from(1_u8) << bits) + BigInt::from(3_u8),
    ));
    assert_eq!(x.exact_rational_ref().unwrap().numerator().bits(), bits as u64 + 1);
    let center = Point2::new(x, Real::zero());
    let first = concentric(&center, 5);
    let second = concentric(&center, 7);
    // Independent exact construction oracle: identical centers, squared radii
    // 25 and 49. No point can satisfy both circle equations.
    for (curve, radius_squared) in [(&first, 25), (&second, 49)] {
        let support = retained_curve_circular_support(curve).unwrap();
        assert_eq!(support.center, &center);
        assert_eq!(support.radius_squared, &Real::from(radius_squared));
    }
    let receipt = crate::evaluate_under(PredicatePolicy::STRICT, || {
        let mut cache = CurveIntersectionBatchCache::default();
        for (phase, count) in [("cold", 1), ("warm", repeats)] {
            let started = Instant::now();
            let mut rows = Vec::<u8>::with_capacity(count);
            for _ in 0..count {
                let relation = cache
                    .circular_support_relation(
                        std::hint::black_box(&first),
                        std::hint::black_box(&second),
                        &CurveContext::STRICT,
                    )
                    .unwrap()
                    .unwrap();
                assert!(matches!(relation, CircleCircleRelation::Disjoint));
                rows.push(0);
            }
            let seconds = started.elapsed().as_secs_f64();
            assert_eq!(cache.circular_support_relations.len(), 1);
            println!(
                "CIRCLE_SHARED_SAMPLE {{\"coefficient_shift_bits\":{bits},\"numerator_bits\":{},\"queries\":{count},\"phase\":\"{phase}\",\"seconds\":{seconds},\"entries\":1}}",
                bits + 1
            );
            println!("CIRCLE_SHARED_ROWS {rows:?}");
        }
    });
    assert!(receipt.certified().is_some());
    println!("CIRCLE_SHARED_CERTAINTY Certified");
}
