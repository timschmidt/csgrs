//! Private-kernel diagnostic, identical in the baseline and candidate builds.
use super::*;
use hyperlimit::PredicatePolicy;
use std::time::Instant;

fn circle(x: Real, radius: &Real) -> Curve2 {
    let start = Point2::new(&x + radius, Real::zero());
    CircularArc2::try_from_center(start.clone(), start, Point2::new(x, Real::zero()), false)
        .unwrap()
        .into()
}

fn same(a: &Real, b: &Real) {
    assert_eq!(
        hyperlimit::compare_reals(a, b, PredicatePolicy::STRICT).value(),
        Some(std::cmp::Ordering::Equal)
    );
}

#[test]
#[ignore = "explicit cost diagnostic; excluded from semantic corpus totals"]
fn support_lookup_cost() {
    let count: usize = std::env::var("CIRCLE_CACHE_COST_N").unwrap().parse().unwrap();
    let mode = std::env::var("CIRCLE_CACHE_COST_MODE").unwrap();
    assert!((1..=8192).contains(&count));
    assert!(["rational", "symbolic", "mixed", "superrange", "subrange"].contains(&mode.as_str()));
    let scale: Real = if mode == "subrange" { "1e-400" } else { "1" }.parse().unwrap();
    let offset: Real = if mode == "superrange" { "1e400" } else { "0" }.parse().unwrap();
    let radius = Real::from(5) * &scale;
    let input = (0..count)
        .map(|n| {
            let symbolic = mode == "symbolic" || (mode == "mixed" && n % 2 == 0);
            let base = if symbolic { Real::pi() } else { Real::zero() };
            let residual = &offset + Real::from(n as i64 * 32) * &scale;
            let x = &base + &residual;
            let first = circle(x.clone(), &radius);
            let second = circle(&x + Real::from(6) * &scale, &radius);
            (first, second, symbolic, x, residual)
        })
        .collect::<Vec<_>>();
    let max_rational_component_bits = input.iter().flat_map(|(a, b, _, _, _)| [a, b])
        .flat_map(|curve| {
            let support = retained_curve_circular_support(curve).unwrap();
            [support.center.x(), support.center.y(), support.radius_squared]
        })
        .filter_map(Real::exact_rational_ref)
        .map(|v| v.numerator().bits().max(v.denominator().bits()))
        .max().unwrap();
    let receipt = crate::evaluate_under(PredicatePolicy::STRICT, || {
        let mut cache = CurveIntersectionBatchCache::default();
        for phase in ["cold", "warm"] {
            let started = Instant::now();
            let mut rows = Vec::with_capacity(count);
            for (first, second, symbolic, x, residual) in &input {
                let relation = cache.circular_support_relation(first, second, &CurveContext::STRICT).unwrap().unwrap();
                let CircleCircleRelation::Secant { first_point, second_point } = relation else {
                    panic!("expected secants, got {relation:?}");
                };
                let expected_residual = residual + Real::from(3) * &scale;
                let expected_x = x + Real::from(3) * &scale;
                let expected_y = Real::from(4) * &scale;
                for point in [&first_point, &second_point] {
                    same(point.x(), &expected_x);
                    same(&(point.y() * point.y()), &(&expected_y * &expected_y));
                    for center_x in [x.clone(), x + Real::from(6) * &scale] {
                        let dx = point.x() - &center_x;
                        same(&(&dx * &dx + point.y() * point.y()), &(&radius * &radius));
                    }
                }
                same(&(first_point.y() + second_point.y()), &Real::zero());
                assert_ne!(first_point.y(), second_point.y());
                // The row represents x = (0 or pi) + the exact rational residual.
                // The strict comparisons above prove that encoding for each point.
                // Subtracting pi need not expose an exact_rational_ref. The
                // preceding strict coordinate equalities prove this exact encoding.
                // Preserve y witness order using a strict sign proof, then
                // prove the chosen rational encoding equals the actual value.
                let encode_y = |actual: &Real| {
                    let exact = match hyperlimit::compare_reals(actual, &Real::zero(), PredicatePolicy::STRICT).value() {
                        Some(std::cmp::Ordering::Greater) => expected_y.clone(),
                        Some(std::cmp::Ordering::Less) => -&expected_y,
                        other => panic!("nonzero secant y needs a strict sign proof: {other:?}"),
                    };
                    same(actual, &exact);
                    exact.exact_rational_ref().unwrap().to_string()
                };
                let x1 = &expected_residual;
                let x2 = &expected_residual;
                rows.push([
                    if *symbolic { "pi" } else { "0" }.to_owned(),
                    x1.exact_rational_ref().unwrap().to_string(),
                    encode_y(first_point.y()),
                    x2.exact_rational_ref().unwrap().to_string(),
                    encode_y(second_point.y()),
                ]);
            }
            let seconds = started.elapsed().as_secs_f64();
            println!("CIRCLE_CACHE_SAMPLE {{\"mode\":\"{mode}\",\"count\":{count},\"phase\":\"{phase}\",\"seconds\":{seconds},\"entries\":{},\"max_rational_component_bits\":{max_rational_component_bits}}}", cache.circular_support_relations.len());
            println!("CIRCLE_CACHE_ROWS {rows:?}");
            assert_eq!(cache.circular_support_relations.len(), count);
        }
    });
    assert!(receipt.certified().is_some());
    println!("CIRCLE_CACHE_CERTAINTY Certified");
}
