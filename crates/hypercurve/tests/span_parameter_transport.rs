use hypercurve::{Curve2, CurveParameter2, Point2};
use hyperreal::Real;

#[test]
fn overlap_parameters_use_the_original_nonunit_nurbs_chart() {
    let curve = Curve2::try_nurbs(
        1,
        [0, 1, 2]
            .map(|x| Point2::new(Real::from(x), Real::zero()))
            .to_vec(),
        vec![Real::one(); 3],
        [2, 2, 3, 5, 5].map(Real::from).to_vec(),
    )
    .unwrap();
    for curve in [curve.clone(), curve.reversed().unwrap()] {
        let intersection = curve.intersect_curve(&curve).unwrap();
        assert!(intersection.is_complete());
        assert_eq!(intersection.overlaps().len(), 2);
        for overlap in intersection.overlaps() {
            let index = overlap.first_span_index();
            let range = overlap.first_range();
            for parameter in [range.start(), range.end()] {
                let authored = curve.parameter_from_span(index, parameter).unwrap();
                let point = curve.point_at(&authored).unwrap();
                let span = &curve.native_bezier_fragments().unwrap()[index];
                let span_curve = Curve2::from(span.curve());
                let expected = span_curve.point_at(parameter).unwrap();
                assert!(point.coincides_with(&expected).unwrap());
            }
        }
        assert!(
            curve
                .parameter_from_span(usize::MAX, &CurveParameter2::from(Real::zero()))
                .is_err()
        );
    }
}

#[test]
fn selected_span_parameters_keep_root_evidence_in_a_nonunit_chart() {
    let curve = Curve2::try_nurbs(
        3,
        [-1, -1, -1, 1]
            .into_iter()
            .zip(0..4)
            .map(|(x, y)| Point2::from_values(x, y))
            .collect(),
        vec![Real::one(); 4],
        [2, 2, 2, 2, 5, 5, 5, 5].map(Real::from).to_vec(),
    )
    .unwrap();
    let line = Curve2::from(
        hypercurve::LineSeg2::try_new(Point2::from_values(0, -1), Point2::from_values(0, 4))
            .unwrap(),
    );
    let intersection = curve.intersect_curve(&line).unwrap();
    assert!(intersection.is_complete());
    assert_eq!(intersection.contacts().len(), 1);
    let contact = &intersection.contacts()[0];
    let local = contact.first().local_parameter();
    assert!(local.scalar().is_none());
    let authored = curve
        .parameter_from_span(contact.first().span_index(), local)
        .unwrap();
    assert!(authored.scalar().is_none());
    assert_eq!(
        authored
            .compare(&contact.first().parameter().unwrap())
            .unwrap(),
        std::cmp::Ordering::Equal
    );
    assert_eq!(
        authored
            .compare(&(Real::from(17) / Real::from(4)).unwrap().into())
            .unwrap(),
        std::cmp::Ordering::Greater
    );
    assert_eq!(
        authored
            .compare(&(Real::from(22) / Real::from(5)).unwrap().into())
            .unwrap(),
        std::cmp::Ordering::Less
    );
    assert!(
        curve
            .point_at(&authored)
            .unwrap()
            .coincides_with(contact.point())
            .unwrap()
    );
}

#[test]
fn retained_restrictions_preserve_the_source_chart_and_traversal() {
    use hypercurve::{BezierParameterRange2, CurveParameterRange2, QuadraticBezier2};
    let q = |n, d| (Real::from(n) / Real::from(d)).unwrap();
    let curve = Curve2::try_analytic_parallel(
        QuadraticBezier2::new(
            Point2::from_values(0, 0),
            Point2::from_values(2, 2),
            Point2::from_values(4, 0),
        )
        .parallel_left(Real::one())
        .unwrap(),
        BezierParameterRange2::try_new(
            hypercurve::BezierParameter2::Exact(Real::from(-1)),
            hypercurve::BezierParameter2::Exact(Real::from(2)),
        )
        .unwrap(),
    )
    .unwrap();
    for curve in [curve.clone(), curve.reversed().unwrap()] {
        let restricted = curve
            .restrict_source(CurveParameterRange2::try_new(q(1, 4).into(), q(3, 4).into()).unwrap())
            .unwrap();
        assert_eq!(
            restricted
                .parameter_domain()
                .start()
                .compare(&q(1, 4).into())
                .unwrap(),
            std::cmp::Ordering::Equal
        );
        assert_eq!(
            restricted
                .parameter_domain()
                .end()
                .compare(&q(3, 4).into())
                .unwrap(),
            std::cmp::Ordering::Equal
        );
        for parameter in [q(1, 4), q(1, 2), q(3, 4)] {
            assert!(
                restricted
                    .point_at(&parameter.clone().into())
                    .unwrap()
                    .coincides_with(&curve.point_at(&parameter.into()).unwrap())
                    .unwrap()
            );
        }
        assert_eq!(
            restricted
                .parameter_endpoints()
                .0
                .compare(restricted.parameter_endpoints().1)
                .unwrap(),
            curve
                .parameter_endpoints()
                .0
                .compare(curve.parameter_endpoints().1)
                .unwrap()
        );
        restricted.bounds().unwrap();
    }
}
