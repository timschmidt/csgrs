//! Generated normalized points preserve selected fields, distance and traversal.
use hypercurve::{
    Curve2, CurvePoint2, CurvePointDefinition2, CurvePointRationalForm2, ExactCurveError, Real,
    UncertaintyReason,
};
use hypersolve::{BivariatePolynomial, IsolatedRootInterval};
use std::cmp::Ordering;

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
fn p(rows: &[&[i32]]) -> BivariatePolynomial {
    BivariatePolynomial::new(
        rows.iter()
            .map(|row| row.iter().copied().map(Real::from).collect())
            .collect(),
    )
}
fn form(point: &CurvePoint2, numerator: BivariatePolynomial) -> CurvePointRationalForm2 {
    point.rational_form(numerator, p(&[&[1]])).unwrap()
}
fn source() -> Curve2 {
    let a = CurvePointDefinition2::PolynomialFiber {
        equation: p(&[&[-2, 0, 0, 1]]),
        parameter: Real::zero().into(),
        root: IsolatedRootInterval {
            lower: 1.into(),
            upper: 2.into(),
            exact_root: None,
            distinct_root_count: 1,
        },
    }
    .into_point()
    .unwrap();
    let b = CurvePointDefinition2::RationalQueryPair {
        sources: [a.clone(), a.clone()],
        numerators: [p(&[&[1]]), p(&[&[0, 0, 1]])],
        denominators: [p(&[&[1]]), p(&[&[1]])],
    }
    .into_point()
    .unwrap();
    // A=(0,alpha), B=(1,alpha^2), alpha^3=2. Its squared speed
    // alpha^2+2alpha-3 is nonrational; the positive sheet is necessary.
    Curve2::try_line(a, b).unwrap()
}
fn displaced(source: &Curve2, normal: bool, end: bool, distance: Real) -> CurvePoint2 {
    CurvePointDefinition2::ChordDisplacement {
        source: source.clone(),
        origin: None,
        at_end: end,
        distance,
        translation: [2.into(), (-3).into()],
        left_normal: normal,
    }
    .into_point()
    .unwrap()
}
fn verify(source: &Curve2, point: &CurvePoint2, normal: bool, end: bool, distance: &Real) {
    let a = source.start();
    let b = source.end();
    let origin = if end { &b } else { &a };
    let x = p(&[&[], &[1]]);
    let y = p(&[&[0, 1]]);
    let dx = form(&b, x.clone()).subtract(&form(&a, x.clone())).unwrap();
    let dy = form(&b, y.clone()).subtract(&form(&a, y.clone())).unwrap();
    let vx = form(point, x)
        .subtract(&form(origin, p(&[&[2], &[1]])))
        .unwrap();
    let vy = form(point, y)
        .subtract(&form(origin, p(&[&[-3, 1]])))
        .unwrap();
    let cross = dx
        .multiply(&vy)
        .unwrap()
        .subtract(&dy.multiply(&vx).unwrap())
        .unwrap();
    let dot = dx
        .multiply(&vx)
        .unwrap()
        .add(&dy.multiply(&vy).unwrap())
        .unwrap();
    assert_eq!(
        vx.multiply(&vx)
            .unwrap()
            .add(&vy.multiply(&vy).unwrap())
            .unwrap()
            .compare_to_real(&(distance * distance))
            .unwrap(),
        Ordering::Equal
    );
    let expected = if distance.definitely_zero() {
        Ordering::Equal
    } else if distance.exact_rational_ref().unwrap().is_negative() {
        Ordering::Less
    } else {
        Ordering::Greater
    };
    if normal {
        assert_eq!(dot.compare_to_real(&Real::zero()).unwrap(), Ordering::Equal);
        assert_eq!(cross.compare_to_real(&Real::zero()).unwrap(), expected);
    } else {
        assert_eq!(
            cross.compare_to_real(&Real::zero()).unwrap(),
            Ordering::Equal
        );
        assert_eq!(dot.compare_to_real(&Real::zero()).unwrap(), expected);
    }
}

#[test]
fn selected_chord_displacements_keep_distance_side_and_reversal() {
    let line = source();
    for reversed in [false, true] {
        let line = if reversed {
            line.reversed().unwrap()
        } else {
            line.clone()
        };
        for normal in [false, true] {
            for end in [false, true] {
                for distance in [q(-1, 8), Real::zero(), q(1, 8)] {
                    let point = displaced(&line, normal, end, distance.clone());
                    verify(&line, &point, normal, end, &distance);
                    let replay = point.definition().unwrap().into_point().unwrap();
                    verify(&line, &replay, normal, end, &distance);
                }
            }
        }
    }
}

#[test]
fn selected_chord_displacement_budget_refusal_keeps_retry_available() {
    use hypersolve::work_budget::{WorkBudget, with_budget};
    let line = source();
    let point = displaced(&line, false, false, q(1, 8));
    let refused = with_budget(WorkBudget::ZERO, || {
        assert!(matches!(point.rational_form(p(&[&[], &[1]]),p(&[&[1]])),
            Err(ExactCurveError::Blocked(blocker)) if blocker.reason()==UncertaintyReason::BudgetExhausted));
    });
    assert!(refused.result.is_err());
    let retry = with_budget(
        WorkBudget {
            refinements: 10000,
            promotions: 10000,
        },
        || {
            verify(&line, &point, false, false, &q(1, 8));
        },
    );
    assert!(retry.result.is_ok());
}

#[cfg(feature = "svg")]
#[test]
fn selected_chord_displacements_replay_in_a_fresh_process() {
    use hypercurve::{
        CurvePath2, CurveRegion2, SvgGeometry2, export_svg_document, import_svg_document,
    };
    const INPUT: &str = "HYPERCURVE_SELECTED_CHORD_DISPLACEMENT_REPLAY";
    fn replay(text: &str) {
        let geometry = import_svg_document(text).unwrap();
        let source = &geometry.paths()[0].curves()[0];
        let line = &geometry.paths()[1].curves()[0];
        verify(source, &line.start(), true, false, &q(1, 8));
        verify(source, &line.end(), true, true, &q(1, 8));
        let next = displaced(source, false, false, q(-1, 8));
        verify(source, &next, false, false, &q(-1, 8));
    }
    if let Some(path) = std::env::var_os(INPUT) {
        replay(&std::fs::read_to_string(path).unwrap());
        return;
    }
    let line = source();
    let shifted = Curve2::try_line(
        displaced(&line, true, false, q(1, 8)),
        displaced(&line, true, true, q(1, 8)),
    )
    .unwrap();
    let text = export_svg_document(&SvgGeometry2::new(
        CurveRegion2::empty(),
        vec![],
        vec![
            CurvePath2::try_new(vec![line]).unwrap(),
            CurvePath2::try_new(vec![shifted]).unwrap(),
        ],
    ))
    .unwrap();
    replay(&text);
    let path = std::env::temp_dir().join(format!(
        "hypercurve-selected-chord-displacement-{}.svg",
        std::process::id()
    ));
    std::fs::write(&path, text).unwrap();
    let child = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "selected_chord_displacements_replay_in_a_fresh_process",
            "--nocapture",
        ])
        .env(INPUT, &path)
        .output();
    std::fs::remove_file(path).unwrap();
    let child = child.unwrap();
    assert!(
        child.status.success(),
        "{}{}",
        String::from_utf8_lossy(&child.stdout),
        String::from_utf8_lossy(&child.stderr)
    );
}

#[test]
fn selected_chord_displacement_keeps_explicit_origin_and_pole_guards() {
    use hypercurve::{CurveError, Point2};
    let line = source();
    for distance in [Real::zero(), q(1, 8)] {
        let point = CurvePointDefinition2::ChordDisplacement {
            source: line.clone(),
            origin: Some(Point2::from_values(3, -1).into()),
            at_end: true,
            distance: distance.clone(),
            translation: [2.into(), (-3).into()],
            left_normal: false,
        }
        .into_point()
        .unwrap();
        let vx = form(&point, p(&[&[-5], &[1]]));
        let vy = form(&point, p(&[&[4, 1]]));
        let dy = form(&line.end(), p(&[&[0, 1]]))
            .subtract(&form(&line.start(), p(&[&[0, 1]])))
            .unwrap();
        assert_eq!(
            vx.multiply(&vx)
                .unwrap()
                .add(&vy.multiply(&vy).unwrap())
                .unwrap()
                .compare_to_real(&(&distance * &distance))
                .unwrap(),
            Ordering::Equal
        );
        assert_eq!(
            vy.subtract(&dy.multiply(&vx).unwrap())
                .unwrap()
                .compare_to_real(&Real::zero())
                .unwrap(),
            Ordering::Equal
        );
        assert_eq!(
            vx.compare_to_real(&Real::zero()).unwrap(),
            if distance.definitely_zero() {
                Ordering::Equal
            } else {
                Ordering::Greater
            }
        );
        assert!(matches!(
            point.rational_form(p(&[&[0]]), p(&[&[0]])),
            Err(ExactCurveError::Invalid {
                cause: CurveError::ZeroRationalPointDenominator,
                ..
            })
        ));
    }
}
