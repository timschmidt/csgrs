//! Zero displacement retains the origin without requiring a normal enclosure.
use hypercurve::{Curve2, CurvePoint2, CurvePointDefinition2, Point2, Real, Similarity2};
use hypersolve::{BivariatePolynomial, IsolatedRootInterval};

fn tiny_source(selected: bool) -> Curve2 {
    let a = if selected {
        CurvePointDefinition2::PolynomialFiber {
            equation: BivariatePolynomial::new(vec![vec![
                (-2).into(),
                0.into(),
                0.into(),
                1.into(),
            ]]),
            parameter: Real::zero().into(),
            root: IsolatedRootInterval {
                lower: 1.into(),
                upper: 2.into(),
                exact_root: None,
                distinct_root_count: 1,
            },
        }
        .into_point()
        .unwrap()
    } else {
        Point2::from_values(0, 1).into()
    };
    let factor = (Real::one() / Real::from(2_u128.pow(100))).unwrap();
    let epsilon = (0..11).fold(Real::one(), |value, _| value * &factor);
    // This direction is nonzero but its length is below the dyadic precision
    // normally needed to project its zero-distance origin.
    let b = CurvePointDefinition2::Similarity {
        source: a.clone(),
        transform: Similarity2::try_from_real_affine(
            1.into(),
            0.into(),
            0.into(),
            1.into(),
            epsilon.clone(),
            epsilon,
        )
        .unwrap(),
    }
    .into_point()
    .unwrap();
    Curve2::try_line(a, b).unwrap()
}

fn verify_bounds(point: &CurvePoint2, origin: &CurvePoint2, translation: &[Real; 2]) {
    let expected = origin.bounds().unwrap();
    let actual = point.bounds().unwrap();
    assert_eq!(actual.min_x(), &(expected.min_x() + &translation[0]));
    assert_eq!(actual.max_x(), &(expected.max_x() + &translation[0]));
    assert_eq!(actual.min_y(), &(expected.min_y() + &translation[1]));
    assert_eq!(actual.max_y(), &(expected.max_y() + &translation[1]));
}

#[test]
fn zero_displacement_bounds_preserve_origin_translation_and_reversal() {
    for selected in [false, true] {
        let source = tiny_source(selected);
        for reverse in [false, true] {
            let source = if reverse {
                source.reversed().unwrap()
            } else {
                source.clone()
            };
            for normal in [false, true] {
                for at_end in [false, true] {
                    for explicit in [false, true] {
                        let origin = explicit.then(|| Point2::from_values(3, -1).into());
                        let expected = origin
                            .clone()
                            .unwrap_or_else(|| if at_end { source.end() } else { source.start() });
                        let translation = [2.into(), (-3).into()];
                        let point = CurvePointDefinition2::ChordDisplacement {
                            source: source.clone(),
                            origin,
                            at_end,
                            distance: Real::zero(),
                            translation: translation.clone(),
                            left_normal: normal,
                        }
                        .into_point()
                        .unwrap();
                        verify_bounds(&point, &expected, &translation);
                        let replay = point.definition().unwrap().into_point().unwrap();
                        verify_bounds(&replay, &expected, &translation);
                    }
                }
            }
        }
    }
}

#[cfg(feature = "svg")]
#[test]
fn zero_displacement_of_a_tiny_selected_chord_replays_in_a_fresh_process() {
    use hypercurve::{
        CurvePath2, CurveRegion2, SvgGeometry2, export_svg_document, import_svg_document,
    };
    const INPUT: &str = "HYPERCURVE_ZERO_CHORD_DISPLACEMENT_REPLAY";
    let source = tiny_source(true);
    let geometry = |line| {
        SvgGeometry2::new(
            CurveRegion2::empty(),
            vec![],
            vec![CurvePath2::try_new(vec![line]).unwrap()],
        )
    };
    fn replay(text: &str, expected: &Curve2) {
        let geometry = import_svg_document(text).unwrap();
        let line = &geometry.paths()[0].curves()[0];
        let point = line.start();
        verify_bounds(&point, &expected.start(), &[0.into(), 0.into()]);
        let CurvePointDefinition2::ChordDisplacement {
            source,
            origin,
            at_end,
            distance,
            translation,
            ..
        } = point.definition().unwrap()
        else {
            panic!("native replay must retain the exact displacement definition");
        };
        assert!(origin.is_none() && !at_end && distance.definitely_zero());
        assert!(translation.iter().all(Real::definitely_zero));
        assert!(source.start().coincides_with(&expected.start()).unwrap());
        assert!(source.end().coincides_with(&expected.end()).unwrap());
        assert!(line.end().coincides_with(&expected.end()).unwrap());
        export_svg_document(&geometry).unwrap();
    }
    if let Some(path) = std::env::var_os(INPUT) {
        replay(&std::fs::read_to_string(path).unwrap(), &source);
        return;
    }
    export_svg_document(&geometry(source.clone())).unwrap();
    for normal in [false, true] {
        let point = CurvePointDefinition2::ChordDisplacement {
            source: source.clone(),
            origin: None,
            at_end: false,
            distance: Real::zero(),
            translation: [0.into(), 0.into()],
            left_normal: normal,
        }
        .into_point()
        .unwrap();
        let line = Curve2::try_line(point, source.end()).unwrap();
        let text = export_svg_document(&geometry(line)).unwrap();
        replay(&text, &source);
        let path = std::env::temp_dir().join(format!(
            "hypercurve-zero-chord-displacement-{}.svg",
            std::process::id()
        ));
        std::fs::write(&path, text).unwrap();
        let child = std::process::Command::new(std::env::current_exe().unwrap())
            .args([
                "--exact",
                "zero_displacement_of_a_tiny_selected_chord_replays_in_a_fresh_process",
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
}
