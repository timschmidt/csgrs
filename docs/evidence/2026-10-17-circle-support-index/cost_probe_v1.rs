//! Diagnostic only: identical source is injected into both isolated test crates.
use super::*;
use std::time::Instant;

fn circle(x: i64) -> Curve2 {
    let start = Point2::from_values(x + 5, 0);
    CircularArc2::try_from_center(start.clone(), start, Point2::from_values(x, 0), false)
        .unwrap().into()
}

#[test]
#[ignore = "explicit cost diagnostic; never part of semantic test counts"]
fn support_lookup_cost() {
    let count: usize = std::env::var("CIRCLE_CACHE_COST_N").unwrap().parse().unwrap();
    assert!((1..=8192).contains(&count));
    let input = (0..count).map(|n| (circle(n as i64 * 32), circle(n as i64 * 32 + 6))).collect::<Vec<_>>();
    let mut cache = CurveIntersectionBatchCache::default();
    for phase in ["cold", "warm"] {
        let started = Instant::now();
        let mut rows = Vec::with_capacity(count);
        for (n, (first, second)) in input.iter().enumerate() {
            let relation = cache.circular_support_relation(first, second, &CurveContext::STRICT).unwrap().unwrap();
            let CircleCircleRelation::Secant { first_point, second_point } = relation else {
                panic!("expected secants, got {relation:?}");
            };
            let x = Real::from(n as i64 * 32 + 3);
            for p in [&first_point, &second_point] {
                assert_eq!(hyperlimit::compare_reals(p.x(), &x, hyperlimit::PredicatePolicy::STRICT).value(), Some(std::cmp::Ordering::Equal));
                assert_eq!(hyperlimit::compare_reals(&(p.y() * p.y()), &Real::from(16), hyperlimit::PredicatePolicy::STRICT).value(), Some(std::cmp::Ordering::Equal));
            }
            assert_eq!(hyperlimit::compare_reals(&(first_point.y() + second_point.y()), &Real::zero(), hyperlimit::PredicatePolicy::STRICT).value(), Some(std::cmp::Ordering::Equal));
            rows.push([first_point.x(), first_point.y(), second_point.x(), second_point.y()].map(|v| v.exact_rational_ref().unwrap().to_string()));
        }
        let seconds = started.elapsed().as_secs_f64();
        println!("CIRCLE_CACHE_SAMPLE {{\"count\":{count},\"phase\":\"{phase}\",\"seconds\":{seconds},\"entries\":{}}}", cache.circular_support_relations.len());
        println!("CIRCLE_CACHE_ROWS {rows:?}");
        assert_eq!(cache.circular_support_relations.len(), count);
    }
}
