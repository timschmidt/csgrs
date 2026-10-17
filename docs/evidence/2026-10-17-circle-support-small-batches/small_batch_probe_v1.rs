//! Repeated complete small batches, identical in both diagnostic variants.
use super::*;
use hyperlimit::PredicatePolicy;
use std::hint::black_box;
use std::time::Instant;

type Row = [String; 4];

fn circle(x: i64) -> Curve2 {
    let start = Point2::new(Real::from(x + 5), Real::zero());
    CircularArc2::try_from_center(
        start.clone(),
        start,
        Point2::new(Real::from(x), Real::zero()),
        false,
    )
    .unwrap()
    .into()
}

fn same(actual: &Real, expected: &Real) {
    assert_eq!(
        hyperlimit::compare_reals(actual, expected, PredicatePolicy::STRICT).value(),
        Some(std::cmp::Ordering::Equal)
    );
}

fn query_rows(
    cache: &mut CurveIntersectionBatchCache,
    input: &[(Curve2, Curve2)],
) -> Vec<Row> {
    let mut rows = Vec::with_capacity(input.len());
    for (n, (first, second)) in input.iter().enumerate() {
        let relation = cache
            .circular_support_relation(first, second, &CurveContext::STRICT)
            .unwrap()
            .unwrap();
        let CircleCircleRelation::Secant {
            first_point,
            second_point,
        } = relation
        else {
            panic!("independent 3-4-5 geometry requires secants: {relation:?}");
        };
        let x = Real::from(n as i64 * 32 + 3);
        for point in [&first_point, &second_point] {
            same(point.x(), &x);
            same(&(point.y() * point.y()), &Real::from(16));
            for center in [n as i64 * 32, n as i64 * 32 + 6] {
                let dx = point.x() - Real::from(center);
                same(&(&dx * &dx + point.y() * point.y()), &Real::from(25));
            }
        }
        same(&(first_point.y() + second_point.y()), &Real::zero());
        assert_ne!(first_point.y(), second_point.y());
        rows.push([
            first_point.x().exact_rational_ref().unwrap().to_string(),
            first_point.y().exact_rational_ref().unwrap().to_string(),
            second_point.x().exact_rational_ref().unwrap().to_string(),
            second_point.y().exact_rational_ref().unwrap().to_string(),
        ]);
    }
    assert_eq!(cache.circular_support_relations.len(), input.len());
    rows
}

#[test]
#[ignore = "explicit amortized cost diagnostic; excluded from semantic corpus totals"]
fn small_batch_complete_cost() {
    let count: usize = std::env::var("CIRCLE_SMALL_BATCH_N")
        .unwrap()
        .parse()
        .unwrap();
    assert!([1, 4, 8, 9, 16, 128].contains(&count));
    let batches = 65_536_usize.div_ceil(count);
    let input = (0..count)
        .map(|n| (circle(n as i64 * 32), circle(n as i64 * 32 + 6)))
        .collect::<Vec<_>>();
    let receipt = crate::evaluate_under(PredicatePolicy::STRICT, || {
        let mut reference_cache = CurveIntersectionBatchCache::default();
        let reference = query_rows(&mut reference_cache, &input);
        drop(reference_cache);
        for phase in ["cold", "warm"] {
            let mut prepared = CurveIntersectionBatchCache::default();
            if phase == "warm" {
                assert_eq!(query_rows(&mut prepared, &input), reference);
            }
            let started = Instant::now();
            for _ in 0..batches {
                let rows = if phase == "cold" {
                    let mut cache = CurveIntersectionBatchCache::default();
                    query_rows(&mut cache, black_box(&input))
                } else {
                    query_rows(&mut prepared, black_box(&input))
                };
                assert_eq!(black_box(&rows), &reference);
                drop(rows);
            }
            let seconds = started.elapsed().as_secs_f64();
            println!(
                "CIRCLE_SMALL_SAMPLE {{\"count\":{count},\"batches\":{batches},\"queries\":{},\"phase\":\"{phase}\",\"seconds\":{seconds}}}",
                batches * count
            );
            println!("CIRCLE_SMALL_ROWS {reference:?}");
        }
    });
    assert!(receipt.certified().is_some());
    println!("CIRCLE_SMALL_CERTAINTY Certified");
}
