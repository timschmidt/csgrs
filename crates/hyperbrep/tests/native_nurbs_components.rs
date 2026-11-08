use hyperbrep::{
    ArchiveLimits, Curve3, CurveParameterLocation, GeometricPoint3, GeometryError, Matrix4,
    ModelBuilder, OperationContext, Parameter, Point3, RawModel, Real, WorkBudget,
};
use hypercurve::{CurveLocationComponent2, CurveParameterRange2, CurveParameterSet2};
use std::cmp::Ordering;
fn q(n: i64, d: i64) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
fn point(x: Real) -> Point3 {
    Point3::new(x, 0.into(), 0.into())
}
fn source(offset: i64) -> Curve3 {
    Curve3::nurbs(
        1,
        [offset, offset, offset + 1, offset]
            .map(|x| point(x.into()))
            .to_vec(),
        vec![Real::one(); 4],
        [2, 2, 5, 8, 11, 11].map(Real::from).to_vec(),
    )
    .unwrap()
}
fn selected_zero() -> GeometricPoint3 {
    let selector = Curve3::rational_bezier(
        [-5, -7, -9, -11, -13, 145].map(|n| point(q(n, 5))).to_vec(),
        vec![Real::one(); 6],
    )
    .unwrap();
    let CurveParameterLocation::Parameters(roots) =
        selector.parameters_of(Point3::origin()).unwrap()
    else {
        panic!("selected zero")
    };
    let query = selector.point_at(&roots[0]).unwrap();
    assert!(query.try_coordinates().is_none());
    query
}
fn assert_locus(curve: &Curve3, query: &GeometricPoint3) {
    let locus = curve.parameters_of(query).unwrap();
    let CurveParameterLocation::Components(components) = &locus else {
        panic!("complete mixed locus")
    };
    assert_eq!(
        components
            .iter()
            .map(|c| c.span_index())
            .collect::<Vec<_>>(),
        [0, 1, 2]
    );
    let CurveParameterSet2::Range(r) = components[0].parameters().unwrap() else {
        panic!("interval")
    };
    assert_eq!(
        r.start().compare(&Real::from(2).into()).unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        r.end().compare(&Real::from(5).into()).unwrap(),
        Ordering::Equal
    );
    for (c, t) in components[1..].iter().zip([5, 11]) {
        let CurveParameterSet2::Single(p) = c.parameters().unwrap() else {
            panic!("source-germ endpoint")
        };
        assert_eq!(p.compare(&Real::from(t).into()).unwrap(), Ordering::Equal);
        assert!(
            curve
                .point_at_component(c, &Real::from(t).into())
                .unwrap()
                .coincides_with(query)
                .unwrap()
        );
    }
    for (t, expected) in [
        (q(3, 2), false),
        (2.into(), true),
        (q(7, 2), true),
        (5.into(), true),
        (q(13, 2), false),
        (8.into(), false),
        (q(19, 2), false),
        (11.into(), true),
        (12.into(), false),
    ] {
        assert_eq!(locus.contains(&t.into(), curve.domain()).unwrap(), expected);
    }
    let selected: hypercurve::CurveParameter2 =
        hypercurve::BezierParameterPolynomial::try_new_power_basis(
            [-1, 1, 0, 0, 0, 1].map(Real::from).to_vec(),
        )
        .unwrap()
        .isolate_unit_interval_roots()
        .unwrap()[0]
            .clone()
            .into();
    let selected: Parameter = selected.affine_image(&3.into(), &2.into()).unwrap().into();
    assert!(selected.try_real().is_none());
    assert!(locus.contains(&selected, curve.domain()).unwrap());
    let result = curve.point_at_component(&components[0], &selected).unwrap();
    assert!(result.coincides_with(query).unwrap());
    assert!(
        curve
            .parameters_of(result)
            .unwrap()
            .contains(&selected, curve.domain())
            .unwrap()
    );
}
#[test]
fn native_mixed_inverse_keeps_all_spans_and_selected_parameters() {
    let curve = source(0);
    let query = selected_zero();
    // Independently derived x(t)=0, (t-5)/3, (11-t)/3 on the three knot spans.
    for (t, x) in [
        (2.into(), 0.into()),
        (q(7, 2), 0.into()),
        (5.into(), 0.into()),
        (q(13, 2), q(1, 2)),
        (8.into(), 1.into()),
        (q(19, 2), q(1, 2)),
        (11.into(), 0.into()),
    ] {
        assert!(
            curve
                .point_at(&t.into())
                .unwrap()
                .coincides_with(&point(x).into())
                .unwrap()
        );
    }
    assert_locus(&curve, &query);
    assert_locus(&curve, &Point3::origin().into());
    let CurveParameterLocation::Parameters(finite) = curve.parameters_of(point(q(1, 2))).unwrap()
    else {
        panic!("original finite path")
    };
    assert_eq!(finite.len(), 2);
    for (p, t) in finite.iter().zip([q(13, 2), q(19, 2)]) {
        assert_eq!(p.compare(&t.into()).unwrap(), Ordering::Equal);
    }
    assert!(matches!(
        curve
            .parameters_of(Point3::new(0.into(), 1.into(), 0.into()))
            .unwrap(),
        CurveParameterLocation::None
    ));
}
#[test]
fn native_components_validate_charts_and_recompute_changed_geometry() {
    let curve = source(0);
    let query = selected_zero();
    let CurveParameterLocation::Components(c) = curve.parameters_of(&query).unwrap() else {
        panic!("components")
    };
    let changed = source(5);
    let actual = changed
        .point_at_component(&c[0], &Real::from(3).into())
        .unwrap();
    assert!(actual.coincides_with(&point(5.into()).into()).unwrap());
    assert!(!actual.coincides_with(&query).unwrap());
    for (index, a, b) in [(usize::MAX, 2, 5), (0, 2, 6)] {
        let invalid = CurveLocationComponent2::from_span_parameters(
            index,
            a.into(),
            b.into(),
            CurveParameterSet2::Range(CurveParameterRange2::unit()),
            [true, true],
        )
        .unwrap();
        assert_eq!(
            curve
                .point_at_component(&invalid, &Real::from(3).into())
                .unwrap_err(),
            GeometryError::ParameterOutsideDomain
        );
    }
    assert_eq!(
        curve
            .point_at_component(&c[0], &Real::from(6).into())
            .unwrap_err(),
        GeometryError::ParameterOutsideDomain
    );
    let invalid = Curve3::nurbs(
        1,
        [0, 0, 1, 0].map(|x| point(x.into())).to_vec(),
        vec![Real::one(); 4],
        [0, 0, 1, 1, 2, 2].map(Real::from).to_vec(),
    );
    assert_eq!(invalid.unwrap_err(), GeometryError::InvalidKnotMultiplicity);
}
#[test]
fn native_components_feed_reversal_restriction_affine_and_cold_replay() {
    let curve = source(0);
    let query = selected_zero();
    let reversed = curve.reversed().unwrap();
    let locus = reversed.parameters_of(&query).unwrap();
    for (t, want) in [(2, true), (5, false), (8, true), (9, true), (11, true)] {
        assert_eq!(
            locus
                .contains(&Real::from(t).into(), reversed.domain())
                .unwrap(),
            want
        );
    }
    let trim = curve.subcurve(&3.into(), &10.into()).unwrap();
    let locus = trim.parameters_of(&query).unwrap();
    // Native NURBS restriction retains its authored knot chart.
    for (t, want) in [(3, true), (4, true), (5, true), (6, false), (10, false)] {
        assert_eq!(
            locus
                .contains(&Real::from(t).into(), trim.domain())
                .unwrap(),
            want
        );
    }
    let transform =
        Matrix4::from_row_major([1, 2, 0, 3, 0, -1, 0, 4, 0, 0, 2, 5, 0, 0, 0, 1].map(Real::from));
    let moved = Curve3::affine_image(curve.clone(), transform.clone()).unwrap();
    assert_locus(&moved, &query.transformed(&transform).unwrap());
    let mut builder = ModelBuilder::new();
    builder.curve(curve).unwrap();
    builder.vertex(query).unwrap();
    let model = builder.finish().unwrap();
    let text = model.to_json().unwrap();
    let replay = RawModel::from_json(&text, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(text, replay.to_json().unwrap());
    assert_locus(
        replay.curves().next().unwrap().geometry(),
        replay.vertices().next().unwrap().point(),
    );
}
#[test]
fn native_component_queries_share_actual_solver_budget_and_retry() {
    let curve = source(0);
    let query = selected_zero();
    let low = OperationContext::strict(WorkBudget::ZERO).evaluate(|| curve.parameters_of(&query));
    // The fixture's endpoint/constant proofs may require no solver work. A zero
    // allowance alone does not justify reporting exhaustion.
    if let Err(ref error) = low.result {
        assert_eq!(error.limit, 0);
        assert_eq!(error.used, 0);
        assert!(error.requested > 0);
    }
    if let Ok(ref result) = low.result {
        assert!(matches!(
            result.as_ref().unwrap(),
            CurveParameterLocation::Components(_)
        ));
    }
    assert_locus(&curve, &query);
    let enough = OperationContext::strict(WorkBudget {
        refinements: 100_000,
        promotions: 10_000,
    });
    enough
        .evaluate(|| assert_locus(&curve, &query))
        .result
        .unwrap();
    assert!(!hypersolve::work_budget::is_active());
}

#[test]
fn rational_quadratic_constant_span_retains_its_endpoint_visits() {
    // Local numerator on the second span is 10s(1-s); all weights
    // are positive, so only its two endpoints are zeros.
    let curve = Curve3::nurbs(
        2,
        [0, 0, 0, 1, 0].map(|x| point(x.into())).to_vec(),
        [1, 2, 3, 5, 7].map(Real::from).to_vec(),
        [2, 2, 2, 5, 5, 8, 8, 8].map(Real::from).to_vec(),
    )
    .unwrap();
    let query = selected_zero();
    let locus = curve.parameters_of(&query).unwrap();
    let CurveParameterLocation::Components(c) = &locus else {
        panic!("complete quadratic locus")
    };
    assert_eq!(
        c.iter().map(|c| c.span_index()).collect::<Vec<_>>(),
        [0, 1, 1]
    );
    for (t, want) in [
        (2, true),
        (3, true),
        (5, true),
        (6, false),
        (7, false),
        (8, true),
    ] {
        assert_eq!(
            locus
                .contains(&Real::from(t).into(), curve.domain())
                .unwrap(),
            want
        );
    }
    let midpoint = curve.point_at(&q(13, 2).into()).unwrap();
    assert!(midpoint.coincides_with(&point(q(1, 2)).into()).unwrap());
    assert!(!midpoint.coincides_with(&query).unwrap());
    for (component, t) in c.iter().zip([3, 5, 8]) {
        assert!(
            curve
                .point_at_component(component, &Real::from(t).into())
                .unwrap()
                .coincides_with(&query)
                .unwrap()
        );
    }
}

#[test]
fn mixed_native_inverse_retains_an_isolated_quintic_parameter() {
    // f(s)=s^5+s-1 has exactly one unit root alpha, since f'>0.
    let selector = Curve3::rational_bezier(
        [-5, -4, -3, -2, -1, 5].map(|n| point(q(n, 5))).to_vec(),
        vec![Real::one(); 6],
    )
    .unwrap();
    let CurveParameterLocation::Parameters(roots) =
        selector.parameters_of(Point3::origin()).unwrap()
    else {
        panic!("alpha")
    };
    assert_eq!(roots.len(), 1);
    let alpha = &roots[0];
    assert!(alpha.try_real().is_none());
    let query = selector.point_at(alpha).unwrap();
    assert!(query.try_coordinates().is_none());
    // q(s)=s*f(s)=s^6+s^2-s; exact Bernstein controls of degree6.
    let controls = [
        Real::zero(),
        q(-1, 6),
        q(-4, 15),
        q(-3, 10),
        q(-4, 15),
        q(-1, 6),
        Real::one(),
    ]
    .map(point)
    .to_vec();
    let span = Curve3::rational_bezier(controls.clone(), vec![Real::one(); 7]).unwrap();
    let CurveParameterLocation::Parameters(visits) = span.parameters_of(&query).unwrap() else {
        panic!("existing finite baseline")
    };
    assert_eq!(visits.len(), 2);
    assert_eq!(
        visits[0].compare(&Real::zero().into()).unwrap(),
        std::cmp::Ordering::Equal
    );
    assert_eq!(visits[1].compare(alpha).unwrap(), std::cmp::Ordering::Equal);
    let mut net = vec![Point3::origin(); 7];
    net.extend(controls.into_iter().skip(1));
    let knots = std::iter::repeat_n(Real::from(2), 7)
        .chain(std::iter::repeat_n(Real::from(5), 6))
        .chain(std::iter::repeat_n(Real::from(8), 7))
        .collect();
    let curve = Curve3::nurbs(6, net, vec![Real::one(); 13], knots).unwrap();
    let locus = curve
        .parameters_of(&query)
        .expect("complete [2,5] union {5+3alpha}");
    let CurveParameterLocation::Components(c) = &locus else {
        panic!("mixed interval/selected finite locus")
    };
    assert_eq!(
        c.iter().map(|c| c.span_index()).collect::<Vec<_>>(),
        [0, 1, 1]
    );
    let expected = alpha.affine_image(&3.into(), &5.into()).unwrap();
    assert!(expected.try_real().is_none());
    assert!(
        c[2].contains_parameter(&expected.curve_parameter())
            .unwrap()
    );
    assert!(
        curve
            .point_at_component(&c[2], &expected)
            .unwrap()
            .coincides_with(&query)
            .unwrap()
    );
    assert!(locus.contains(&expected, curve.domain()).unwrap());
    for (t, want) in [
        (2, true),
        (3, true),
        (5, true),
        (6, false),
        (7, false),
        (8, false),
    ] {
        assert_eq!(
            locus
                .contains(&Real::from(t).into(), curve.domain())
                .unwrap(),
            want
        );
    }
}

#[test]
fn component_point_retains_original_native_source_and_global_parameter_after_replay() {
    let curve = source(0);
    let query = selected_zero();
    let parameter = query
        .curve_source()
        .unwrap()
        .1
        .affine_image(&3.into(), &2.into())
        .unwrap();
    assert!(parameter.try_real().is_none());
    let CurveParameterLocation::Components(c) = curve.parameters_of(&query).unwrap() else {
        panic!("components")
    };
    let point = curve.point_at_component(&c[0], &parameter).unwrap();
    assert!(point.coincides_with(&query).unwrap());
    let (original, retained) = point
        .curve_source()
        .expect("original selected native image");
    assert_eq!(original.kind(), hyperbrep::Curve3Kind::Nurbs);
    assert_eq!(retained.compare(&parameter).unwrap(), Ordering::Equal);
    assert_eq!(
        original.nurbs_geometry().unwrap().knots,
        curve.nurbs_geometry().unwrap().knots
    );
    let mut builder = ModelBuilder::new();
    builder.curve(curve).unwrap();
    builder.vertex(point).unwrap();
    let text = builder.finish().unwrap().to_json().unwrap();
    let replay = RawModel::from_json(&text, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(text, replay.to_json().unwrap());
    let replayed = replay.vertices().next().unwrap();
    let point = replayed.point();
    let (original, retained) = point
        .curve_source()
        .expect("replayed original native image");
    assert_eq!(original.kind(), hyperbrep::Curve3Kind::Nurbs);
    assert_eq!(retained.compare(&parameter).unwrap(), Ordering::Equal);
    assert!(retained.try_real().is_none());
    assert_locus(original, point);
}
