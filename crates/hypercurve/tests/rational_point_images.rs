use hypercurve::*;
use hypersolve::BivariatePolynomial;
use std::cmp::Ordering;

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn polynomial(rows: &[&[i32]]) -> BivariatePolynomial {
    BivariatePolynomial::new(
        rows.iter()
            .map(|row| row.iter().copied().map(Real::from).collect())
            .collect(),
    )
}

fn source_and_independent_circle_point() -> (CurvePoint2, CurvePoint2) {
    let parameter = BezierParameter2::Algebraic(
        BezierAlgebraicParameter2::try_isolate(
            BezierParameterPolynomial::try_new_power_basis(vec![
                (-1).into(),
                0.into(),
                0.into(),
                0.into(),
                0.into(),
                0.into(),
                0.into(),
                2.into(),
            ])
            .unwrap(),
            BezierParameterInterval::try_new(q(3, 4), Real::one()).unwrap(),
        )
        .unwrap(),
    );
    let parabola = Curve2::from(
        RationalBezier2::try_new(
            vec![
                Point2::from_values(0, 0),
                Point2::new(q(1, 2), 0.into()),
                Point2::from_values(1, 1),
            ],
            vec![1.into(); 3],
        )
        .unwrap(),
    );
    // Independent Bernstein data for (2t/(1+t²), (1-t²)/(1+t²)).
    let circle = Curve2::from(
        RationalBezier2::try_new(
            vec![
                Point2::from_values(0, 1),
                Point2::from_values(1, 1),
                Point2::from_values(1, 0),
            ],
            vec![1.into(), 1.into(), 2.into()],
        )
        .unwrap(),
    );
    (
        parabola.point_at(&parameter.clone().into()).unwrap(),
        circle.point_at(&parameter.into()).unwrap(),
    )
}

fn circle_map() -> RationalMap2 {
    RationalMap2::new(
        [polynomial(&[&[0], &[2]]), polynomial(&[&[1, -1]])],
        polynomial(&[&[1, 1]]),
    )
}

#[test]
fn native_scalar_image_rational_query_control() {
    let (source, expected) = source_and_independent_circle_point();
    assert!(source.coordinates().is_none());
    for (numerator, coordinate) in [
        (polynomial(&[&[0], &[2]]), polynomial(&[&[0], &[1]])),
        (polynomial(&[&[1, -1]]), polynomial(&[&[0, 1]])),
    ] {
        let actual = source
            .rational_form(numerator, polynomial(&[&[1, 1]]))
            .unwrap();
        let independent = expected
            .rational_form(coordinate, polynomial(&[&[1]]))
            .unwrap();
        assert_eq!(actual.compare(&independent).unwrap(), Ordering::Equal);
    }
}

#[test]
fn native_scalar_point_image_replays_and_reenters_another_rational_map() {
    let (source, expected) = source_and_independent_circle_point();
    let mapped = source.rational_image(&circle_map()).unwrap();
    assert!(mapped.coincides_with(&expected).unwrap());
    let unit = mapped
        .rational_form(polynomial(&[&[-1, 0, 1], &[], &[1]]), polynomial(&[&[1]]))
        .unwrap();
    assert_eq!(
        unit.compare_to_real(&Real::zero()).unwrap(),
        Ordering::Equal
    );
    let replay = mapped.definition().unwrap().into_point().unwrap();
    assert!(replay.coincides_with(&expected).unwrap());
    // Recover (t,t²), using t = X/(1+Y) and preserving its original guard.
    let inverse = RationalMap2::new(
        [polynomial(&[&[0], &[1, 1]]), polynomial(&[&[], &[], &[1]])],
        polynomial(&[&[1, 2, 1]]),
    );
    assert!(
        replay
            .rational_image(&inverse)
            .unwrap()
            .coincides_with(&source)
            .unwrap()
    );
}

#[test]
fn native_scalar_point_map_rejects_the_original_pole() {
    let (source, _) = source_and_independent_circle_point();
    let pole = polynomial(&[&[-1], &[], &[], &[], &[], &[], &[], &[2]]);
    let invalid = RationalMap2::new([pole.clone(), pole.clone()], pole);
    assert!(matches!(
        source.rational_image(&invalid),
        Err(ExactCurveError::Invalid {
            cause: CurveError::ZeroRationalPointDenominator,
            ..
        })
    ));
}

fn shifted(point: CurvePoint2, dx: Real, dy: Real) -> CurvePoint2 {
    CurvePointDefinition2::Similarity {
        source: point,
        transform: Similarity2::try_from_real_affine(
            1.into(),
            0.into(),
            0.into(),
            1.into(),
            dx,
            dy,
        )
        .unwrap(),
    }
    .into_point()
    .unwrap()
}

#[test]
fn native_scalar_point_images_preserve_negative_denominators_and_order() {
    let (source, expected) = source_and_independent_circle_point();
    let negative = RationalMap2::new(
        [polynomial(&[&[0], &[-2]]), polynomial(&[&[-1, 1]])],
        polynomial(&[&[-1, -1]]),
    );
    let mapped = source.rational_image(&negative).unwrap();
    assert!(mapped.coincides_with(&expected).unwrap());
    let near = shifted(expected.clone(), q(1, 1_000_000), 0.into());
    assert_eq!(
        mapped.compare_coordinate(&near, Axis2::X).unwrap(),
        Ordering::Less
    );
    assert_eq!(
        near.compare_coordinate(&mapped, Axis2::X).unwrap(),
        Ordering::Greater
    );
    assert_eq!(
        mapped.compare_coordinate(&near, Axis2::Y).unwrap(),
        Ordering::Equal
    );
    assert!(!mapped.coincides_with(&near).unwrap());
    for image in [
        mapped.clone(),
        shifted(mapped.clone(), 1.into(), (-1).into()),
    ] {
        let replay = image.definition().unwrap().into_point().unwrap();
        assert!(image.coincides_with(&replay).unwrap());
        let bounds = image.bounds().unwrap();
        for axis in [Axis2::X, Axis2::Y] {
            assert!(matches!(
                image
                    .compare_coordinate(&bounds.min().clone().into(), axis)
                    .unwrap(),
                Ordering::Equal | Ordering::Greater
            ));
            assert!(matches!(
                image
                    .compare_coordinate(&bounds.max().clone().into(), axis)
                    .unwrap(),
                Ordering::Equal | Ordering::Less
            ));
        }
    }
    // A downstream 0/0 must remain invalid even when numerator and denominator cancel.
    let unit = polynomial(&[&[-1, 0, 1], &[], &[1]]);
    let bad = CurvePointDefinition2::RationalImage {
        source: mapped.clone(),
        map: RationalMap2::new([unit.clone(), unit.clone()], unit),
    };
    assert!(matches!(
        bad.into_point(),
        Err(ExactCurveError::Invalid {
            cause: CurveError::ZeroRationalPointDenominator,
            ..
        })
    ));
    assert!(mapped.coincides_with(&expected).unwrap());
}

#[test]
#[cfg(feature = "svg")]
fn native_scalar_point_images_replay_in_a_fresh_process_before_exact_split() {
    const INPUT: &str = "HYPERCURVE_RATIONAL_POINT_IMAGE_REPLAY_INPUT";
    let document = |path| {
        export_svg_document(&SvgGeometry2::new(
            CurveRegion2::empty(),
            vec![],
            vec![path],
        ))
        .unwrap()
    };
    let verify = |text: &str| {
        let replay = import_svg_document(text).unwrap();
        let (_, expected) = source_and_independent_circle_point();
        let line = &replay.paths()[0].curves()[0];
        assert!(line.start().coincides_with(&expected).unwrap());
        assert!(
            line.end()
                .coincides_with(&shifted(expected.clone(), 1.into(), 0.into()))
                .unwrap()
        );
        let midpoint = shifted(expected, q(1, 2), 0.into());
        let CurvePointLocations2::Locations(locations) = line.point_locations(&midpoint).unwrap()
        else {
            panic!("exact midpoint location")
        };
        assert_eq!(locations.len(), 1);
        let (first, second) = line.split_at(locations[0].parameter().unwrap()).unwrap();
        assert!(first.end().coincides_with(&midpoint).unwrap());
        assert!(second.start().coincides_with(&midpoint).unwrap());
        let next = document(CurvePath2::try_new(vec![first, second]).unwrap());
        assert_eq!(
            export_svg_document(&import_svg_document(&next).unwrap()).unwrap(),
            next
        );
        assert_eq!(export_svg_document(&replay).unwrap(), text);
        // Continue with a rational map after native reconstruction.
        let identity = RationalMap2::new(
            [polynomial(&[&[0], &[1]]), polynomial(&[&[0, 1]])],
            polynomial(&[&[1]]),
        );
        assert!(
            line.start()
                .rational_image(&identity)
                .unwrap()
                .coincides_with(&line.start())
                .unwrap()
        );
    };
    if let Some(file) = std::env::var_os(INPUT) {
        verify(&std::fs::read_to_string(file).unwrap());
        return;
    }
    let (source, _) = source_and_independent_circle_point();
    let mapped = source.rational_image(&circle_map()).unwrap();
    let line = Curve2::try_line(mapped.clone(), shifted(mapped, 1.into(), 0.into())).unwrap();
    let text = document(CurvePath2::try_new(vec![line]).unwrap());
    verify(&text);
    let file = std::env::temp_dir().join(format!(
        "hypercurve-rational-point-image-{}.svg",
        std::process::id()
    ));
    std::fs::write(&file, &text).unwrap();
    let output = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "native_scalar_point_images_replay_in_a_fresh_process_before_exact_split",
            "--nocapture",
        ])
        .env(INPUT, &file)
        .output()
        .unwrap();
    std::fs::remove_file(file).unwrap();
    assert!(
        output.status.success(),
        "{}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
}

#[test]
fn native_scalar_point_map_uses_no_independent_coordinate_promotion() {
    use hypersolve::work_budget::{WorkBudget, with_budget};
    let (source, expected) = source_and_independent_circle_point();
    let attempt = with_budget(
        WorkBudget {
            refinements: 4096,
            promotions: 0,
        },
        || {
            let image = source.rational_image(&circle_map())?;
            image.coincides_with(&expected)
        },
    );
    assert!(attempt.result.unwrap().unwrap());
    assert_eq!(attempt.usage.promotions, 0);
}

#[test]
fn native_scalar_point_image_does_not_publish_after_reached_exhaustion() {
    use hypersolve::work_budget::{WorkBudget, with_budget};
    let (source, _) = source_and_independent_circle_point();
    let attempt = with_budget(WorkBudget::ZERO, || {
        let result = source.rational_image(&circle_map());
        assert!(hypersolve::work_budget::exhaustion().is_some());
        assert!(
            matches!(result,Err(ExactCurveError::Blocked(blocker)) if blocker.reason()==UncertaintyReason::BudgetExhausted)
        );
    });
    let reached = attempt.result.unwrap_err();
    assert_eq!((reached.limit, reached.used), (0, 0));
    assert!(reached.requested > 0);
}

#[test]
fn native_scalar_point_images_retry_after_actual_exhaustion() {
    use hypersolve::work_budget::{WorkBudget, with_budget};
    for _ in 0..2 {
        let (source, expected) = source_and_independent_circle_point();
        let denied = with_budget(WorkBudget::ZERO, || source.rational_image(&circle_map()));
        let reached = denied.result.unwrap_err();
        assert_eq!((reached.limit, reached.used), (0, 0));
        assert!(reached.requested > 0);
        let retry = with_budget(
            WorkBudget {
                refinements: 4096,
                promotions: 0,
            },
            || {
                let image = source.rational_image(&circle_map())?;
                image.coincides_with(&expected)
            },
        );
        assert!(retry.result.unwrap().unwrap());
        assert_eq!(retry.usage.promotions, 0);
        for policy in [
            PredicatePolicy::STRICT,
            PredicatePolicy::APPROXIMATE_512,
            PredicatePolicy::STRICT,
        ] {
            let request = evaluate_under(policy, || source.rational_image(&circle_map()).unwrap());
            assert_eq!(request.certainty(), CurveCertainty::Certified);
            assert!(
                request
                    .certified()
                    .unwrap()
                    .coincides_with(&expected)
                    .unwrap()
            );
        }
    }
}
