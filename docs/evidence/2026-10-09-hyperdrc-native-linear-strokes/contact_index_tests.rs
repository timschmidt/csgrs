use super::{ContactPointIndex, exact_contact_point_box};
use crate::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, Curve2, CurveGeometry2, CurveParameterRange2, CurvePoint2,
    Point2, PredicatePolicy, RationalBezier2, Real, evaluate_under,
};

fn endpoints() -> Vec<f64> {
    let mut values = vec![-f64::MAX, -1.0, -0.0, 0.0, 1.0, f64::MAX];
    for exponent in [-1022, -1000, -600, -54, -53, 0, 53, 600, 1000, 1023] {
        let power = f64::from_bits(((1023 + exponent) as u64) << 52);
        for value in [power.next_down(), power, power.next_up()] {
            values.extend([value, -value]);
        }
    }
    for bits in [1, 2, 3, (1_u64 << 52) - 1] {
        let value = f64::from_bits(bits);
        values.extend([value, -value]);
    }
    values.sort_by(f64::total_cmp);
    values.dedup_by(|a, b| a.to_bits() == b.to_bits());
    values
}

#[test]
fn contact_index_keys_follow_mathematical_endpoint_order() {
    let mut values = endpoints();
    values.extend([f64::NEG_INFINITY, f64::INFINITY]);
    for &a in &values {
        for &b in &values {
            assert_eq!(
                ContactPointIndex::key(a).cmp(&ContactPointIndex::key(b)),
                a.partial_cmp(&b).unwrap(),
                "{a:?}, {b:?}",
            );
        }
    }
}

#[test]
fn contact_index_matches_independent_box_overlap_at_float_extremes() {
    let values = endpoints();
    let queries: Vec<_> = values
        .iter()
        .map(|&x| CurvePoint2::from(Point2::new(Real::try_from(x).unwrap(), Real::zero())))
        .collect();
    let mut shared = ContactPointIndex::default();
    let mut boxes = Vec::new();
    for &low in &values {
        for &high in &values {
            if low > high {
                continue;
            }
            let bounds = [low, high, 0.0, 0.0];
            // A contact may inherit any conservative box from its vertex.
            // Supply that box directly so the oracle also covers wide and
            // overflowing intervals independently of a particular producer.
            let mut isolated = ContactPointIndex::default();
            isolated.ensure_vertex(0);
            isolated.vertex_boxes[0] = Some(bounds);
            isolated.push(0, None, 0);
            isolated.push(1, None, 1);
            for (&x, query) in values.iter().zip(&queries) {
                let expected = if low <= x && x <= high {
                    vec![0, 1]
                } else {
                    vec![1]
                };
                assert_eq!(isolated.candidates(Some(query)), Some(expected));
            }
            let index = boxes.len();
            shared.ensure_vertex(index);
            shared.vertex_boxes[index] = Some(bounds);
            shared.push(index, None, index);
            boxes.push(bounds);
        }
    }
    assert!(shared.max_width.is_infinite());
    let unboxed = boxes.len();
    shared.push(unboxed, None, unboxed);
    shared.merge_vertex(unboxed, 0);
    assert_eq!(shared.incidences(0), &[0, unboxed]);
    assert!(shared.candidates(None).is_none());
    for (&x, query) in values.iter().zip(&queries) {
        let mut expected: Vec<_> = boxes
            .iter()
            .enumerate()
            .filter_map(|(index, bounds)| (bounds[0] <= x && x <= bounds[1]).then_some(index))
            .collect();
        expected.push(unboxed);
        assert_eq!(shared.candidates(Some(query)), Some(expected));
    }
    let above = CurvePoint2::from(Point2::from_values(0, 1));
    assert_eq!(shared.candidates(Some(&above)), Some(vec![unboxed]));
    assert_eq!(values.len(), 70);
    assert_eq!(boxes.len(), 2486);
}

fn selected_point(
    lower: Real,
    polynomial: &BezierParameterPolynomial,
    scale: &Real,
) -> CurvePoint2 {
    let interval = BezierParameterInterval::try_new(lower, Real::one()).unwrap();
    let parameter =
        BezierAlgebraicParameter2::try_isolate(polynomial.clone(), interval).unwrap();
    let line = RationalBezier2::try_new(
        vec![Point2::from_values(0, 0), Point2::new(scale.clone(), Real::zero())],
        vec![Real::one(); 2],
    )
    .unwrap();
    let range = CurveParameterRange2::try_new(
        BezierParameter2::Algebraic(parameter).into(),
        Real::from(2).into(),
    )
    .unwrap();
    Curve2::try_from_bezier_range(CurveGeometry2::RationalBezier(line), range)
        .unwrap()
        .start()
}

#[test]
fn contact_index_retains_identical_selected_points_with_different_isolators() {
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        evaluate_under(policy, || {
            let ulp = (Real::one() / Real::from(1_u64 << 52)).unwrap();
            let delta = (&ulp / Real::from(4)).unwrap();
            // p'(t) = 5t^4+1 > 0; p(1-2^-53) < 0 < p(1).
            // Both brackets therefore select the same unique real root.
            let polynomial = BezierParameterPolynomial::try_new_power_basis(vec![
                delta - Real::from(2),
                Real::one(),
                Real::zero(),
                Real::zero(),
                Real::zero(),
                Real::one(),
            ])
            .unwrap();
            for exponent in [-1000, 0, 1000] {
                let power = f64::from_bits(((1023 + exponent) as u64) << 52);
                for scale in [power, -power] {
                    let scale = Real::try_from(scale).unwrap();
                    for warm in [false, true] {
                        let wide = selected_point(-Real::one() - &ulp, &polynomial, &scale);
                        let narrow = selected_point(
                            Real::one() - (&ulp / Real::from(2)).unwrap(),
                            &polynomial,
                            &scale,
                        );
                        assert!(wide.coordinates().is_none());
                        assert!(narrow.coordinates().is_none());
                        assert!(wide.coincides_with(&narrow).unwrap());
                        if warm {
                            for point in [&wide, &narrow] {
                                assert!(exact_contact_point_box(point).is_some());
                                point.bounds().unwrap();
                            }
                        }
                        for (stored, query) in [(&wide, &narrow), (&narrow, &wide)] {
                            let mut index = ContactPointIndex::default();
                            index.push(0, Some(stored), 0);
                            index.push(1, None, 0);
                            index.push(2, None, 1);
                            index.merge_vertex(1, 0);
                            assert_eq!(index.incidences(0), &[0, 1, 2]);
                            for point in [query, &query.clone(), query] {
                                assert_eq!(index.candidates(Some(point)), Some(vec![0, 1, 2]));
                            }
                        }
                    }
                }
            }
        })
        .certified()
        .expect("all selected-point decisions must be certified");
    }
}
