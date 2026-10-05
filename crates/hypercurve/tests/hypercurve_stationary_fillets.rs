//! Stationary contacts retain the tangent of the side that survives a fillet.
//! Circles and comparison regions below are independently constructed.

mod support;
mod contacts {
    use hypercurve::{
        Classification, CubicBezier2, Curve2, CurveCertainty, CurveContext, CurveCornerMode2,
        CurveFillet2, CurveFilletContact2, CurvePath2, ExactCurveError, LineSeg2, Point2,
        QuadraticBezier2, RationalBezier2, Real,
    };
    fn q(n: i64, d: i64) -> Real {
        (Real::from(n) / Real::from(d)).unwrap()
    }
    fn point(x: Real, y: Real) -> Point2 {
        Point2::new(x, y)
    }
    fn source(kind: u8) -> Curve2 {
        match kind {
            0 => QuadraticBezier2::new(
                Point2::from_values(0, 0),
                point(q(1, 2), Real::zero()),
                Point2::from_values(1, 1),
            )
            .into(),
            1 => RationalBezier2::try_new(
                vec![
                    Point2::from_values(0, 0),
                    Point2::from_values(0, 0),
                    point(q(1, 6), Real::zero()),
                    point(q(1, 2), Real::zero()),
                    Point2::from_values(1, 1),
                ],
                vec![Real::one(); 5],
            )
            .unwrap()
            .into(),
            2 => CubicBezier2::new(
                Point2::from_values(0, 0),
                Point2::from_values(0, 0),
                point(q(1, 3), Real::zero()),
                Point2::from_values(1, 1),
            )
            .into(),
            _ => unreachable!(),
        }
    }
    fn check_mode(
        kind: u8,
        policy: CurveContext,
        reversed: bool,
        mode: CurveCornerMode2,
        quadratic_line: bool,
    ) {
        // Kind 0 is (t,t^2), kind 1 is the same image (t^2,t^4), and
        // kind 2 is the one-sided cusp (t^2,t^3). Both stationary charts have
        // the exact right-hand tangent +x at their authored start.
        let (radius, cx, cy, x, y) = if kind == 2 {
            (q(5, 8), q(5, 8), q(-3, 8), q(1, 4), q(1, 8))
        } else {
            (q(15, 16), q(15, 16), q(-39, 64), q(3, 8), q(9, 64))
        };
        let contact0 = point(Real::zero(), cy.clone());
        let contact1 = point(x, y);
        // Independently fixed rational circle: horizontal radial at the line
        // contact, and a (3,4,5) normal at the curved contact.
        let mut request = CurveFillet2::new(radius);
        request.center = Some(point(cx, cy).into());
        let line: Curve2 = if quadratic_line {
            QuadraticBezier2::new(
                Point2::from_values(0, -2),
                Point2::from_values(0, -1),
                Point2::from_values(0, 0),
            )
            .into()
        } else {
            LineSeg2::try_new(Point2::from_values(0, -2), Point2::from_values(0, 0))
                .unwrap()
                .into()
        };
        let path = CurvePath2::try_new(vec![line, source(kind)]).unwrap();
        let path = if reversed {
            path.reversed(&policy).unwrap().into_value()
        } else {
            path
        };
        let (contact0, contact1) = if reversed {
            (contact1, contact0)
        } else {
            (contact0, contact1)
        };
        let outcome = match path.fillet_vertex(1, &request, mode, &policy) {
            Ok(outcome) => outcome,
            Err(ExactCurveError::Blocked(blocker)) => {
                eprintln!("blocked {:?}", blocker.reason());
                panic!("a rational tangent circle is independently known")
            }
            Err(_) => panic!("unexpected corner-input error"),
        };
        assert!(outcome.certainty == CurveCertainty::Certified);
        let solutions = outcome.into_value();
        eprintln!(
            "candidate_count={} no_solution={:?}",
            solutions.candidate_count(),
            solutions.no_solution_reason()
        );
        assert_eq!(solutions.candidate_count(), 1);
        let solution = solutions.into_solutions().pop().unwrap();
        assert_eq!(solution.curves().len(), 3);
        let arc = &solution.curves()[1];
        let first = arc.start().coincides_with(&contact0.into(), &policy);
        let second = arc.end().coincides_with(&contact1.into(), &policy);
        assert!(
            first.certainty == CurveCertainty::Certified
                && first.value == Classification::Decided(true)
        );
        assert!(
            second.certainty == CurveCertainty::Certified
                && second.value == Classification::Decided(true)
        );
    }
    fn check(kind: u8, policy: CurveContext) {
        check_reversed(kind, policy, false)
    }
    #[test]
    fn stationary_reparameterization_reversed_strict() {
        check_reversed(1, CurveContext::STRICT, true)
    }
    #[test]
    fn stationary_reparameterization_reversed_approximate() {
        check_reversed(1, CurveContext::APPROXIMATE_512, true)
    }
    #[test]
    fn one_sided_cusp_reversed_strict() {
        check_reversed(2, CurveContext::STRICT, true)
    }
    #[test]
    fn one_sided_cusp_reversed_approximate() {
        check_reversed(2, CurveContext::APPROXIMATE_512, true)
    }
    #[test]
    fn regular_parabola_strict() {
        check(0, CurveContext::STRICT)
    }
    #[test]
    fn stationary_reparameterization_strict() {
        check(1, CurveContext::STRICT)
    }
    #[test]
    fn one_sided_cusp_strict() {
        check(2, CurveContext::STRICT)
    }
    #[test]
    fn regular_parabola_approximate() {
        check(0, CurveContext::APPROXIMATE_512)
    }
    #[test]
    fn stationary_reparameterization_approximate() {
        check(1, CurveContext::APPROXIMATE_512)
    }
    #[test]
    fn one_sided_cusp_approximate() {
        check(2, CurveContext::APPROXIMATE_512)
    }

    #[test]
    fn quadratic_line_preserves_stationary_contacts_strict() {
        for kind in [1, 2] {
            for reversed in [false, true] {
                for mode in [CurveCornerMode2::TrimOnly, CurveCornerMode2::TrimOrExtend] {
                    check_mode(kind, CurveContext::STRICT, reversed, mode, true);
                }
            }
        }
    }

    #[test]
    fn quadratic_line_preserves_stationary_contacts_approximate() {
        for kind in [1, 2] {
            for reversed in [false, true] {
                for mode in [CurveCornerMode2::TrimOnly, CurveCornerMode2::TrimOrExtend] {
                    check_mode(kind, CurveContext::APPROXIMATE_512, reversed, mode, true);
                }
            }
        }
    }

    fn interior_stationary_contact_mode(
        policy: CurveContext,
        reversed: bool,
        mode: CurveCornerMode2,
        quadratic_line: bool,
        constraint: u8,
    ) {
        // C(t)=(t^2,t^3), -1<=t<=1, with public t=2*u-1. The
        // previous retained branch has tangent -x at u=1/2, although C'=0.
        let source = CubicBezier2::new(
            Point2::from_values(1, -1),
            point(q(-1, 3), Real::one()),
            point(q(-1, 3), -Real::one()),
            Point2::from_values(1, 1),
        );
        let next: Curve2 = if quadratic_line {
            QuadraticBezier2::new(
                Point2::from_values(1, 1),
                point(-Real::one(), q(-1, 2)),
                Point2::from_values(-3, -2),
            )
            .into()
        } else {
            LineSeg2::try_new(Point2::from_values(1, 1), Point2::from_values(-3, -2))
                .unwrap()
                .into()
        };
        let path = CurvePath2::try_new(vec![source.into(), next]).unwrap();
        let mut request = CurveFillet2::new(Real::one());
        if constraint < 3 {
            request.center = Some(Point2::from_values(0, -1).into());
        }
        request.contacts[usize::from(reversed)] = match constraint {
            0 | 3 => Some(CurveFilletContact2::Parameter(q(1, 2).into())),
            1 => Some(CurveFilletContact2::Point(Point2::from_values(0, 0).into())),
            2 => None,
            _ => unreachable!(),
        };
        let path = if reversed {
            path.reversed(&policy).unwrap().into_value()
        } else {
            path
        };
        let outcome = match path.fillet_vertex(1, &request, mode, &policy) {
            Ok(outcome) => outcome,
            Err(ExactCurveError::Blocked(blocker)) => {
                eprintln!("blocked {:?}", blocker.reason());
                panic!("one-sided contact has a rational normal and tangent circle")
            }
            Err(_) => panic!("unexpected interior-contact input error"),
        };
        assert!(outcome.certainty == CurveCertainty::Certified);
        let solutions = outcome.into_value();
        eprintln!(
            "candidate_count={} no_solution={:?}",
            solutions.candidate_count(),
            solutions.no_solution_reason()
        );
        assert_eq!(solutions.candidate_count(), 1);
        let solution = solutions.into_solutions().pop().unwrap();
        assert_eq!(solution.curves().len(), 3);
        let arc = &solution.curves()[1];
        let contacts = [Point2::from_values(0, 0), point(q(-3, 5), q(-1, 5))];
        let first = arc
            .start()
            .coincides_with(&contacts[usize::from(reversed)].clone().into(), &policy);
        let second = arc
            .end()
            .coincides_with(&contacts[usize::from(!reversed)].clone().into(), &policy);
        assert!(
            first.certainty == CurveCertainty::Certified
                && first.value == Classification::Decided(true)
        );
        assert!(
            second.certainty == CurveCertainty::Certified
                && second.value == Classification::Decided(true)
        );
    }
    fn interior_stationary_contact(policy: CurveContext) {
        interior_stationary_contact_reversed(policy, false)
    }
    #[test]
    fn interior_stationary_contact_reversed_strict() {
        interior_stationary_contact_reversed(CurveContext::STRICT, true)
    }
    #[test]
    fn interior_stationary_contact_reversed_approximate() {
        interior_stationary_contact_reversed(CurveContext::APPROXIMATE_512, true)
    }
    #[test]
    fn interior_stationary_contact_strict() {
        interior_stationary_contact(CurveContext::STRICT)
    }
    #[test]
    fn interior_stationary_contact_approximate() {
        interior_stationary_contact(CurveContext::APPROXIMATE_512)
    }

    fn opposite_stationary_sheet_mode(
        policy: CurveContext,
        reversed: bool,
        mode: CurveCornerMode2,
        quadratic_line: bool,
    ) {
        // C(t)=(t²,t³), -1<=t<=2. At u=1/3 the retained previous
        // branch approaches the origin in direction -x. The circle centered at
        // (0,1) has the opposite source normal there. Its other contact is the
        // strictly interior point (-4/5,8/5) on the line, so endpoint rejection
        // cannot substitute for the missing normal-sheet decision.
        let curve = CubicBezier2::new(
            Point2::from_values(1, -1),
            Point2::from_values(-1, 2),
            Point2::from_values(0, -4),
            Point2::from_values(4, 8),
        );
        let line: Curve2 = if quadratic_line {
            QuadraticBezier2::new(
                Point2::from_values(4, 8),
                Point2::from_values(1, 4),
                Point2::from_values(-2, 0),
            )
            .into()
        } else {
            LineSeg2::try_new(Point2::from_values(4, 8), Point2::from_values(-2, 0))
                .unwrap()
                .into()
        };
        let path = CurvePath2::try_new(vec![curve.into(), line]).unwrap();
        let path = if reversed {
            path.reversed(&policy).unwrap().into_value()
        } else {
            path
        };
        let mut request = CurveFillet2::new(Real::one());
        request.center = Some(Point2::from_values(0, 1).into());
        request.contacts[usize::from(reversed)] = Some(CurveFilletContact2::Parameter(
            if reversed { q(2, 3) } else { q(1, 3) }.into(),
        ));
        let outcome = path
            .fillet_vertex(1, &request, mode, &policy)
            .unwrap_or_else(|_| panic!("opposite one-sided normal must be decidable"));
        assert!(outcome.certainty == CurveCertainty::Certified);
        assert_eq!(outcome.into_value().candidate_count(), 0);
    }
    #[test]
    fn opposite_stationary_sheet_strict() {
        opposite_stationary_sheet(CurveContext::STRICT, false)
    }
    #[test]
    fn opposite_stationary_sheet_approximate() {
        opposite_stationary_sheet(CurveContext::APPROXIMATE_512, false)
    }
    #[test]
    fn opposite_stationary_sheet_reversed_strict() {
        opposite_stationary_sheet(CurveContext::STRICT, true)
    }
    #[test]
    fn opposite_stationary_sheet_reversed_approximate() {
        opposite_stationary_sheet(CurveContext::APPROXIMATE_512, true)
    }

    fn check_reversed(kind: u8, policy: CurveContext, reversed: bool) {
        for mode in [CurveCornerMode2::TrimOnly, CurveCornerMode2::TrimOrExtend] {
            check_mode(kind, policy, reversed, mode, false)
        }
    }
    fn quadratic_line_interior_contacts(policy: CurveContext) {
        for reversed in [false, true] {
            for mode in [CurveCornerMode2::TrimOnly, CurveCornerMode2::TrimOrExtend] {
                for constraint in 0..4 {
                    interior_stationary_contact_mode(policy, reversed, mode, true, constraint);
                }
            }
        }
    }
    fn quadratic_line_opposite_sheet(policy: CurveContext) {
        for reversed in [false, true] {
            for mode in [CurveCornerMode2::TrimOnly, CurveCornerMode2::TrimOrExtend] {
                opposite_stationary_sheet_mode(policy, reversed, mode, true);
            }
        }
    }
    #[test]
    fn quadratic_line_interior_contacts_strict() {
        quadratic_line_interior_contacts(CurveContext::STRICT);
    }
    #[test]
    fn quadratic_line_interior_contacts_approximate() {
        quadratic_line_interior_contacts(CurveContext::APPROXIMATE_512);
    }
    #[test]
    fn quadratic_line_opposite_sheet_strict() {
        quadratic_line_opposite_sheet(CurveContext::STRICT);
    }
    #[test]
    fn quadratic_line_opposite_sheet_approximate() {
        quadratic_line_opposite_sheet(CurveContext::APPROXIMATE_512);
    }

    fn interior_stationary_contact_reversed(policy: CurveContext, reversed: bool) {
        for mode in [CurveCornerMode2::TrimOnly, CurveCornerMode2::TrimOrExtend] {
            interior_stationary_contact_mode(policy, reversed, mode, false, 0)
        }
    }
    fn opposite_stationary_sheet(policy: CurveContext, reversed: bool) {
        for mode in [CurveCornerMode2::TrimOnly, CurveCornerMode2::TrimOrExtend] {
            opposite_stationary_sheet_mode(policy, reversed, mode, false)
        }
    }
}
mod composition {
    use hypercurve::{
        BooleanOp, CircularArc2, CubicBezier2, Curve2, CurveCertainty, CurveContext,
        CurveCornerMode2, CurveFillet2, CurveFilletContact2, CurvePath2, CurveRegion2,
        CurveRegionLoopRole, FillRule, LineSeg2, OffsetCornerStyle2, Point2, Real,
    };
    fn q(n: i64, d: i64) -> Real {
        (Real::from(n) / Real::from(d)).unwrap()
    }
    fn p(x: i64, y: i64) -> Point2 {
        Point2::from_values(x, y)
    }
    fn line(a: Point2, b: Point2) -> Curve2 {
        LineSeg2::try_new(a, b).unwrap().into()
    }
    fn region(path: CurvePath2, policy: &CurveContext) -> CurveRegion2 {
        let result = crate::support::under(policy, || {
            CurveRegion2::try_from_boundary_paths_with_loop_semantics(
                &[path],
                &[CurveRegionLoopRole::Material],
                &[FillRule::NonZero],
            )
        })
        .unwrap_or_else(|_| panic!("exact fillet path must enter region topology"));
        assert!(result.certainty == CurveCertainty::Certified);
        result.into_value()
    }
    fn regions(policy: CurveContext) -> (CurveRegion2, CurveRegion2) {
        let source = CubicBezier2::new(
            p(1, -1),
            Point2::new(q(-1, 3), Real::one()),
            Point2::new(q(-1, 3), -Real::one()),
            p(1, 1),
        );
        let path = CurvePath2::try_new(vec![source.into(), line(p(1, 1), p(-3, -2))]).unwrap();
        let mut request = CurveFillet2::new(Real::one());
        request.center = Some(p(0, -1).into());
        request.contacts[0] = Some(CurveFilletContact2::Parameter(q(1, 2).into()));
        let outcome = path
            .fillet_vertex(1, &request, CurveCornerMode2::TrimOnly, &policy)
            .unwrap_or_else(|_| panic!("stationary fillet must close"));
        assert!(outcome.certainty == CurveCertainty::Certified);
        let solution = outcome.into_value().into_solutions().pop().unwrap();
        let mut curves = solution.curves().to_vec();
        curves.push(line(p(-3, -2), p(1, -1)));
        let actual = region(CurvePath2::try_new(curves).unwrap(), &policy);
        let contact = Point2::new(q(-3, 5), q(-1, 5));
        let expected = region(
            CurvePath2::try_new(vec![
                CubicBezier2::new(
                    p(1, -1),
                    Point2::new(q(1, 3), Real::zero()),
                    p(0, 0),
                    p(0, 0),
                )
                .into(),
                CircularArc2::try_from_center(p(0, 0), contact.clone(), p(0, -1), false)
                    .unwrap()
                    .into(),
                line(contact, p(-3, -2)),
                line(p(-3, -2), p(1, -1)),
            ])
            .unwrap(),
            &policy,
        );
        (actual, expected)
    }
    fn compare(actual: &CurveRegion2, expected: &CurveRegion2, policy: &CurveContext) {
        let xor = crate::support::under(policy, || actual.boolean_region(expected, BooleanOp::Xor))
            .unwrap_or_else(|_| panic!("independent exact regions must compare"));
        assert!(xor.certainty == CurveCertainty::Certified);
        assert!(xor.value.is_empty());
    }
    fn topology(policy: CurveContext) {
        let (actual, expected) = regions(policy);
        compare(&actual, &expected, &policy);
        let paths = crate::support::under(&policy, || actual.boundary_paths()).unwrap();
        assert!(paths.certainty == CurveCertainty::Certified);
        let paths = paths.value;
        assert_eq!(paths.len(), 1);
        compare(&region(paths[0].clone(), &policy), &expected, &policy);
    }
    fn offset(policy: CurveContext) {
        let (actual, expected) = regions(policy);
        let offset = |region: &CurveRegion2| {
            let outcome = crate::support::under(&policy, || {
                region.offset(q(1, 20), &OffsetCornerStyle2::Round)
            })
            .unwrap_or_else(|_| panic!("offset of stationary fillet must close"));
            assert!(outcome.certainty == CurveCertainty::Certified);
            outcome.into_value()
        };
        compare(&offset(&actual), &offset(&expected), &policy);
    }
    #[test]
    fn stationary_fillet_region_strict() {
        topology(CurveContext::STRICT)
    }
    #[test]
    fn stationary_fillet_region_approximate() {
        topology(CurveContext::APPROXIMATE_512)
    }
    #[test]
    fn stationary_fillet_offset_strict() {
        offset(CurveContext::STRICT)
    }
    #[test]
    fn stationary_fillet_offset_approximate() {
        offset(CurveContext::APPROXIMATE_512)
    }
}
mod exact_scalars {
    use hypercurve::{
        Classification, CubicBezier2, CurveCertainty, CurveContext, CurveCornerMode2, CurveFillet2,
        CurveFilletContact2, CurvePath2, LineSeg2, Point2, Real,
    };
    fn q(n: i64, d: i64) -> Real {
        (Real::from(n) / Real::from(d)).unwrap()
    }
    fn check(policy: CurveContext) {
        let scale = Real::from(2).sqrt().unwrap();
        let tx = Real::from(3).sqrt().unwrap();
        let ty = Real::from(5).sqrt().unwrap();
        let a = &scale * q(3, 5);
        let b = &scale * q(4, 5);
        let point = |x: Real, y: Real| Point2::new(&tx + &a * &x - &b * &y, &ty + &b * x + &a * y);
        for reversed in [false, true] {
            for mode in [CurveCornerMode2::TrimOnly, CurveCornerMode2::TrimOrExtend] {
                let source = CubicBezier2::new(
                    point(Real::one(), -Real::one()),
                    point(q(-1, 3), Real::one()),
                    point(q(-1, 3), -Real::one()),
                    point(Real::one(), Real::one()),
                );
                let line = LineSeg2::try_new(
                    point(Real::one(), Real::one()),
                    point(Real::from(-3), Real::from(-2)),
                )
                .unwrap();
                let path = CurvePath2::try_new(vec![source.into(), line.into()]).unwrap();
                let path = if reversed {
                    path.reversed(&policy).unwrap().into_value()
                } else {
                    path
                };
                let mut request = CurveFillet2::new(scale.clone());
                request.center = Some(point(Real::zero(), -Real::one()).into());
                request.contacts[usize::from(reversed)] =
                    Some(CurveFilletContact2::Parameter(q(1, 2).into()));
                let outcome = path
                    .fillet_vertex(1, &request, mode, &policy)
                    .unwrap_or_else(|_| {
                        panic!("one-sided exact frames must not require rational coefficients")
                    });
                assert!(outcome.certainty == CurveCertainty::Certified);
                let solutions = outcome.into_value();
                assert_eq!(solutions.candidate_count(), 1);
                let solution = solutions.into_solutions().pop().unwrap();
                assert_eq!(solution.curves().len(), 3);
                let arc = &solution.curves()[1];
                let contacts = [point(Real::zero(), Real::zero()), point(q(-3, 5), q(-1, 5))];
                for (actual, expected) in [
                    (arc.start(), &contacts[usize::from(reversed)]),
                    (arc.end(), &contacts[usize::from(!reversed)]),
                ] {
                    let result = actual.coincides_with(&expected.clone().into(), &policy);
                    assert!(
                        result.certainty == CurveCertainty::Certified
                            && result.value == Classification::Decided(true)
                    );
                }
            }
        }
    }
    #[test]
    fn stationary_contact_exact_scalars_strict() {
        check(CurveContext::STRICT)
    }
    #[test]
    fn stationary_contact_exact_scalars_approximate() {
        check(CurveContext::APPROXIMATE_512)
    }
}

mod retained_domains {
    use hypercurve::{
        BezierParameter2, BezierParameterRange2, Classification, CubicBezier2, Curve2,
        CurveCertainty, CurveContext, CurveCornerMode2, CurveFillet2, CurvePath2, LineSeg2, Point2,
        Real,
    };
    fn q(n: i64, d: i64) -> Real {
        (Real::from(n) / Real::from(d)).unwrap()
    }
    fn decided<T>(value: Classification<T>) -> T {
        match value {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => panic!("fixture classification {reason:?}"),
        }
    }
    fn check_case(policy: CurveContext, nonzero: bool, reversed: bool, mode: CurveCornerMode2) {
        let source = CubicBezier2::new(
            Point2::from_values(0, 0),
            Point2::from_values(0, 0),
            Point2::new(q(1, 3), Real::zero()),
            Point2::from_values(1, 1),
        );
        let distance = if nonzero { q(1, 8) } else { Real::zero() };
        let parallel = source.parallel_left(distance.clone()).unwrap();
        let start = decided(parallel.point_at(&q(1, 4), &policy).unwrap());
        // At t=1/2 the primitive tangent is (4/5,3/5). The clockwise
        // fillet center must be start.x+r horizontally from its vertical line,
        // and C(1/2)+(distance-r)*(-3/5,4/5) on the source normal.
        let radius = ((q(1, 4) - q(3, 5) * &distance - start.x()) / q(2, 5)).unwrap();
        let center = Point2::new(
            start.x() + &radius,
            q(1, 8) + q(4, 5) * (&distance - &radius),
        );
        let contact = Point2::new(
            q(1, 4) - q(3, 5) * distance.clone(),
            q(1, 8) + q(4, 5) * distance,
        );
        let range = decided(
            BezierParameterRange2::try_new(
                BezierParameter2::Exact(q(1, 4)),
                BezierParameter2::Exact(Real::one()),
                &policy,
            )
            .unwrap(),
        );
        let fragment = decided(Curve2::try_analytic_parallel(parallel, range, &policy).unwrap());
        let line =
            LineSeg2::try_new(Point2::new(start.x().clone(), Real::from(-2)), start).unwrap();
        let path = CurvePath2::try_new(vec![line.into(), fragment]).unwrap();
        let path = if reversed {
            path.reversed(&policy).unwrap().into_value()
        } else {
            path
        };
        let mut request = CurveFillet2::new(radius);
        request.center = Some(center.into());
        let result = path
            .fillet_vertex(1, &request, mode, &policy)
            .unwrap_or_else(|_| {
                panic!(
                    "stationarity outside the retained source range cannot reject its exact fillet"
                )
            });
        assert!(result.certainty == CurveCertainty::Certified);
        let solutions = result.into_value();
        assert_eq!(solutions.candidate_count(), 1);
        let solution = solutions.into_solutions().pop().unwrap();
        let arc = &solution.curves()[1];
        let endpoint = if reversed { arc.start() } else { arc.end() };
        let result = endpoint.coincides_with(&contact.into(), &policy);
        assert!(
            result.certainty == CurveCertainty::Certified
                && result.value == Classification::Decided(true)
        );
    }
    #[test]
    fn retained_regular_source_range_strict() {
        check(CurveContext::STRICT, false)
    }
    #[test]
    fn retained_regular_source_range_approximate() {
        check(CurveContext::APPROXIMATE_512, false)
    }
    #[test]
    fn retained_nonzero_parallel_range_strict() {
        check(CurveContext::STRICT, true)
    }
    #[test]
    fn retained_nonzero_parallel_range_approximate() {
        check(CurveContext::APPROXIMATE_512, true)
    }

    fn check(policy: CurveContext, nonzero: bool) {
        for reversed in [false, true] {
            for mode in [CurveCornerMode2::TrimOnly, CurveCornerMode2::TrimOrExtend] {
                check_case(policy, nonzero, reversed, mode)
            }
        }
    }
}
