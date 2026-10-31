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

/// `|v|^2` and the signed cross and dot of `v` against `source`'s direction,
/// where `v` is `point - origin - translation`.
fn relation(
    source: &Curve2,
    point: &CurvePoint2,
    origin: &CurvePoint2,
    translation: [i32; 2],
) -> (Ordering, Ordering, CurvePointRationalForm2) {
    let x = p(&[&[], &[1]]);
    let y = p(&[&[0, 1]]);
    let (a, b) = (source.start(), source.end());
    let dx = form(&b, x.clone()).subtract(&form(&a, x.clone())).unwrap();
    let dy = form(&b, y.clone()).subtract(&form(&a, y.clone())).unwrap();
    let vx = form(point, x)
        .subtract(&form(origin, p(&[&[translation[0]], &[1]])))
        .unwrap();
    let vy = form(point, y)
        .subtract(&form(origin, p(&[&[translation[1], 1]])))
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
    let squared = vx
        .multiply(&vx)
        .unwrap()
        .add(&vy.multiply(&vy).unwrap())
        .unwrap();
    (
        cross.compare_to_real(&Real::zero()).unwrap(),
        dot.compare_to_real(&Real::zero()).unwrap(),
        squared,
    )
}

fn distance_squared(first: &CurvePoint2, second: &CurvePoint2) -> CurvePointRationalForm2 {
    let x = p(&[&[], &[1]]);
    let y = p(&[&[0, 1]]);
    let dx = form(first, x.clone()).subtract(&form(second, x)).unwrap();
    let dy = form(first, y.clone()).subtract(&form(second, y)).unwrap();
    dx.multiply(&dx)
        .unwrap()
        .add(&dy.multiply(&dy).unwrap())
        .unwrap()
}

/// The reflection `(x, y) -> (1 - 2x, 2y)`: scale 2 with reversed orientation.
fn reflection() -> hypercurve::Similarity2 {
    hypercurve::Similarity2::try_from_real_affine(
        Real::from(-2),
        Real::zero(),
        Real::zero(),
        Real::from(2),
        Real::one(),
        Real::zero(),
    )
    .unwrap()
}

/// The image of the left-normal displacement by 1/8, translated by (2, -3),
/// is the displacement of the image line by 1/4 on its right, translated by
/// the reflected (-4, -6). A chamfer of the image with setback 1/16 is the
/// image of the original chamfer with setback 1/32.
fn check_reflected_displacement(line: &Curve2, shifted: &Curve2) {
    for (point, origin) in [(shifted.start(), line.start()), (shifted.end(), line.end())] {
        let (cross, dot, squared) = relation(line, &point, &origin, [-4, -6]);
        assert_eq!(cross, Ordering::Less, "the reflected normal turns right");
        assert_eq!(dot, Ordering::Equal);
        assert_eq!(squared.compare_to_real(&q(1, 16)).unwrap(), Ordering::Equal);
    }
}

fn chamfer(path: &hypercurve::CurvePath2, setback: Real) -> hypercurve::CurvePath2 {
    match path
        .chamfer_vertex_by_setbacks(
            1,
            setback.clone(),
            setback,
            hypercurve::CurveCornerMode2::TrimOnly,
        )
        .unwrap()
    {
        hypercurve::CurveCornerSolutions2::Unique(path) => path,
        other => panic!("one chamfer: {} solutions", other.solutions().len()),
    }
}

#[cfg(feature = "svg")]
#[test]
fn reflected_translated_displacement_replays_then_chamfers_with_scaled_setbacks() {
    use hypercurve::{
        CurvePath2, CurveRegion2, Point2, SvgGeometry2, export_svg_document, import_svg_document,
    };
    const INPUT: &str = "HYPERCURVE_REFLECTED_DISPLACEMENT_REPLAY";
    // The far corner, in the image and in the original: (1 - 2(-2), 2 * 5/2).
    let far = Point2::from_values(5, 5);
    let far_source = Point2::new(Real::from(-2), q(5, 2));
    fn chamfer_and_check(image_line: &Curve2, image_shifted: &Curve2, far: &Point2) -> CurvePath2 {
        check_reflected_displacement(image_line, image_shifted);
        let corner = image_shifted.end();
        let path = CurvePath2::try_new(vec![
            image_shifted.clone(),
            Curve2::try_line(corner.clone(), far.clone().into()).unwrap(),
        ])
        .unwrap();
        let cut = chamfer(&path, q(1, 16));
        let chord = &cut.curves()[1];
        for end in [chord.start(), chord.end()] {
            assert_eq!(
                distance_squared(&end, &corner)
                    .compare_to_real(&q(1, 256))
                    .unwrap(),
                Ordering::Equal
            );
        }
        cut
    }
    if let Some(path) = std::env::var_os(INPUT) {
        let geometry = import_svg_document(&std::fs::read_to_string(path).unwrap()).unwrap();
        chamfer_and_check(
            &geometry.paths()[0].curves()[0],
            &geometry.paths()[1].curves()[0],
            &far,
        );
        return;
    }
    let line = source();
    let shifted = Curve2::try_line(
        displaced(&line, true, false, q(1, 8)),
        displaced(&line, true, true, q(1, 8)),
    )
    .unwrap();
    let image_line = line.transform_similarity(&reflection()).unwrap();
    let image_shifted = shifted.transform_similarity(&reflection()).unwrap();
    let cut = chamfer_and_check(&image_line, &image_shifted, &far);
    // Setback scaling: the image of the original chamfer with setback 1/32.
    let original = CurvePath2::try_new(vec![
        shifted.clone(),
        Curve2::try_line(shifted.end(), far_source.into()).unwrap(),
    ])
    .unwrap();
    let mapped = chamfer(&original, q(1, 32))
        .transform_similarity(&reflection())
        .unwrap();
    for (first, second) in cut.curves().iter().zip(mapped.curves()) {
        assert!(first.start().coincides_with(&second.start()).unwrap());
        assert!(first.end().coincides_with(&second.end()).unwrap());
    }
    let text = export_svg_document(&SvgGeometry2::new(
        CurveRegion2::empty(),
        vec![],
        vec![
            CurvePath2::try_new(vec![image_line]).unwrap(),
            CurvePath2::try_new(vec![image_shifted]).unwrap(),
        ],
    ))
    .unwrap();
    let file = std::env::temp_dir().join(format!(
        "hypercurve-reflected-displacement-{}.svg",
        std::process::id()
    ));
    std::fs::write(&file, text).unwrap();
    let child = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "reflected_translated_displacement_replays_then_chamfers_with_scaled_setbacks",
            "--nocapture",
        ])
        .env(INPUT, &file)
        .output();
    std::fs::remove_file(file).unwrap();
    let child = child.unwrap();
    assert!(
        child.status.success(),
        "{}{}",
        String::from_utf8_lossy(&child.stdout),
        String::from_utf8_lossy(&child.stderr)
    );
}
