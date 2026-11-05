mod support;
use hypercurve::*;

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn mixed() -> (CurvePath2, CertifiedCurvePathParallel2) {
    let source = CurvePath2::try_new(vec![
        LineSeg2::try_new(Point2::from_values(-1, 0), Point2::from_values(0, 0))
            .unwrap()
            .into(),
        QuadraticBezier2::new(
            Point2::from_values(0, 0),
            Point2::from_values(1, 0),
            Point2::from_values(1, 1),
        )
        .into(),
    ])
    .unwrap();
    let options = BezierParallelVerificationOptions::try_new(q(1, 20), 16).unwrap();
    let output = source
        .approximate_parallel_blend2d_certified(q(1, 10), &options)
        .unwrap();
    (source, output)
}

fn check_mapping(curve: &Curve2, expected: Real) {
    let receipt = curve.parallel_fidelity().unwrap();
    let mapped = receipt
        .source_parameter_at(&CurveParameter2::from(q(1, 4)))
        .unwrap()
        .unwrap();
    assert_eq!(
        mapped.compare(&CurveParameter2::from(expected)).unwrap(),
        std::cmp::Ordering::Equal
    );
}

#[test]
fn restriction_reversal_and_scale_preserve_the_pointwise_source_map() {
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        support::under_value(&policy, || -> ExactCurveResult<()> {
            let (_, output) = mixed();
            let curve = output
                .path()
                .curves()
                .iter()
                .find(|c| {
                    c.parallel_fidelity().unwrap().metric()
                        == CurveParallelFidelityMetric2::SameParameterEuclidean
                })
                .unwrap();
            let receipt = curve.parallel_fidelity().unwrap();
            assert_eq!(receipt.construction_certainty(), CurveCertainty::Certified);
            assert_eq!(receipt.error_bound(), q(1, 20));
            assert!(receipt.verification_leaf_count() > 0);
            let start = receipt.construction_source_range().start().clone();
            let length = receipt.construction_source_range().end() - &start;
            let (left, _) = curve.split_at(CurveParameter2::from(q(1, 2)))?;
            check_mapping(&left, &start + &length * q(1, 8));
            let reversed = left.reversed()?;
            check_mapping(&reversed, &start + &length * q(3, 8));
            let map = Similarity2::try_from_real_affine(
                (-3).into(),
                0.into(),
                0.into(),
                3.into(),
                7.into(),
                2.into(),
            )
            .unwrap();
            let image = reversed.transform_similarity(&map)?;
            check_mapping(&image, &start + &length * q(3, 8));
            assert_eq!(image.parallel_fidelity().unwrap().error_bound(), q(3, 20));
            assert_eq!(
                image.parallel_fidelity().unwrap().source_distance(),
                &q(1, 10)
            );
            assert_eq!(
                image.parallel_fidelity().unwrap().image_transforms().len(),
                1
            );
            assert!(
                image
                    .parallel_fidelity()
                    .unwrap()
                    .source_parameter_at(&CurveParameter2::from(q(-1, 2)))
                    .is_err()
            );
            let native = image.native_bezier_fragments()?[0].clone().into_curve();
            assert_eq!(native.parallel_fidelity().unwrap().error_bound(), q(3, 20));
            check_mapping(&native, &start + &length * q(3, 8));
            let unverified = Curve2::new(native.geometry().unwrap().clone());
            assert!(unverified.parallel_fidelity().is_none());
            Ok(())
        })
        .unwrap();
    }
}

#[test]
fn selected_restrictions_keep_the_selected_parameter_without_projection() {
    support::under_value(&PredicatePolicy::STRICT, || -> ExactCurveResult<()> {
        let (_, output) = mixed();
        let curve = output
            .path()
            .curves()
            .iter()
            .find(|c| {
                c.parallel_fidelity().unwrap().metric()
                    == CurveParallelFidelityMetric2::SameParameterEuclidean
            })
            .unwrap();
        let selected = CurveParameter2::from(BezierParameter2::Algebraic(
            BezierAlgebraicParameter2::try_isolate(
                BezierParameterPolynomial::try_new_power_basis(vec![
                    (-1).into(),
                    0.into(),
                    2.into(),
                ])?,
                BezierParameterInterval::try_new(q(1, 2), q(3, 4))?,
            )?,
        ));
        let (left, right) = curve.split_at(selected.clone())?;
        let first = left
            .parallel_fidelity()
            .unwrap()
            .source_parameter_at(&selected)?
            .unwrap();
        let second = right
            .parallel_fidelity()
            .unwrap()
            .source_parameter_at(&selected)?
            .unwrap();
        assert_eq!(first.compare(&second)?, std::cmp::Ordering::Equal);
        assert!(
            first.scalar().is_none(),
            "a selected endpoint must not be rounded into an authored scalar"
        );
        assert!(left.end().coincides_with(&right.start())?);
        let reversed = left.reversed()?;
        assert_eq!(
            reversed
                .parallel_fidelity()
                .unwrap()
                .source_parameter_at(&selected)?
                .unwrap()
                .compare(&first)?,
            std::cmp::Ordering::Equal
        );
        #[cfg(feature = "svg")]
        replay(&[reversed, right]);
        Ok(())
    })
    .unwrap();
}

#[test]
fn zero_distance_keeps_inherited_fidelity_and_unverified_geometry_stays_unverified() {
    support::under_value(&PredicatePolicy::STRICT, || -> ExactCurveResult<()> {
        let (_, output) = mixed();
        let path = output.into_path();
        let options = BezierParallelVerificationOptions::try_new(q(1, 20), 16)?;
        let unchanged = path.approximate_parallel_blend2d_certified(Real::zero(), &options)?;
        assert_eq!(unchanged.exact_source_curve_count(), path.curves().len());
        for (a, b) in path.curves().iter().zip(unchanged.path().curves()) {
            assert_eq!(
                a.parallel_fidelity().unwrap().metric(),
                b.parallel_fidelity().unwrap().metric()
            );
            assert_eq!(
                a.parallel_fidelity().unwrap().error_bound(),
                b.parallel_fidelity().unwrap().error_bound()
            );
        }
        let next = path.approximate_parallel_blend2d_certified(q(1, 20), &options)?;
        for curve in next.path().curves() {
            let receipt = curve.parallel_fidelity().unwrap();
            assert_eq!(receipt.source_distance(), &q(1, 20));
            let inherited = receipt.source_curve().parallel_fidelity().unwrap();
            assert_eq!(inherited.source_distance(), &q(1, 10));
            if receipt.metric() == CurveParallelFidelityMetric2::SameParameterEuclidean {
                // This is an immediate-source bound, not the sum of errors
                // from successive constructions. The earlier receipt remains.
                assert_eq!(receipt.error_bound(), q(1, 20));
                assert_eq!(inherited.error_bound(), q(1, 20));
            }
        }
        #[cfg(feature = "svg")]
        replay(next.path().curves());
        // Equal images do not prove a same-parameter error bound: t^3 and
        // its chord differ by 3/8 at t=1/2.
        let source = CubicBezier2::new(
            Point2::from_values(0, 0),
            Point2::from_values(0, 0),
            Point2::from_values(0, 0),
            Point2::from_values(1, 0),
        );
        let chord =
            LineSeg2::try_new(Point2::from_values(0, 0), Point2::from_values(1, 0)).unwrap();
        assert_eq!(
            chord.point_at(q(1, 2)).x() - source.point_at(q(1, 2)).x(),
            q(3, 8)
        );
        assert!(Curve2::from(chord).parallel_fidelity().is_none());
        assert!(Curve2::from(source).parallel_fidelity().is_none());
        Ok(())
    })
    .unwrap();
}

#[cfg(feature = "svg")]
fn replay(curves: &[Curve2]) -> Vec<Curve2> {
    // Disjoint edited pieces are separate paths; preserve each original chart.
    let geometry = SvgGeometry2::new(
        CurveRegion2::empty(),
        vec![],
        curves
            .iter()
            .map(|c| CurvePath2::try_new(vec![c.clone()]).unwrap())
            .collect(),
    );
    let document = export_svg_document(&geometry).unwrap();
    let restored = import_svg_document(&document).unwrap();
    assert_eq!(restored.paths().len(), curves.len());
    let result = restored
        .paths()
        .iter()
        .map(|p| p.curves()[0].clone())
        .collect::<Vec<_>>();
    for (a, b) in curves.iter().zip(&result) {
        assert!(a.start().coincides_with(&b.start()).unwrap());
        assert!(a.end().coincides_with(&b.end()).unwrap());
        assert_eq!(
            a.parallel_fidelity().unwrap().metric(),
            b.parallel_fidelity().unwrap().metric()
        );
        assert_eq!(
            a.parallel_fidelity().unwrap().error_bound(),
            b.parallel_fidelity().unwrap().error_bound()
        );
    }
    assert_eq!(export_svg_document(&restored).unwrap(), document);
    result
}

#[cfg(feature = "svg")]
#[test]
fn shear_transports_the_reference_image_and_uses_a_proved_operator_bound() {
    support::under_value(&PredicatePolicy::STRICT, || -> ExactCurveResult<()> {
        let (_, output) = mixed();
        let path = output.into_path();
        let geometry = SvgGeometry2::new(CurveRegion2::empty(), vec![], vec![path.clone()]);
        let document = export_svg_document(&geometry).unwrap().replacen(
            "<path ",
            "<path transform=\"matrix(2 0 1 1 3 -2)\" ",
            1,
        );
        let image = import_svg_document(&document).unwrap();
        for (source, current) in path.curves().iter().zip(image.paths()[0].curves()) {
            let receipt = current.parallel_fidelity().unwrap();
            assert_eq!(
                receipt.error_bound(),
                source.parallel_fidelity().unwrap().error_bound() * Real::from(4)
            );
            assert!(matches!(
                receipt.image_transforms().as_slice(),
                [CurveParallelFidelityTransform2::Affine(_)]
            ));
            let p = source.point_at(&CurveParameter2::from(q(1, 2)))?;
            let p = p.coordinates().expect("native polynomial point");
            let expected = Point2::new(
                Real::from(2) * p.x() + p.y() + Real::from(3),
                p.y() - Real::from(2),
            );
            assert!(
                current
                    .point_at(&CurveParameter2::from(q(1, 2)))?
                    .coincides_with(&expected.into())?
            );
        }
        let derived = image.paths()[0]
            .curves()
            .iter()
            .map(|c| {
                c.subcurve(
                    CurveParameter2::from(q(1, 4)),
                    CurveParameter2::from(q(3, 4)),
                )
                .unwrap()
                .reversed()
                .unwrap()
            })
            .collect::<Vec<_>>();
        replay(&derived);
        Ok(())
    })
    .unwrap();
}

#[cfg(feature = "svg")]
#[test]
fn exact_arc_edits_replay_their_actual_charts_and_native_spans() {
    support::under_value(&PredicatePolicy::STRICT, || -> ExactCurveResult<()> {
        let arc = CircularArc2::from_bulge(
            Point2::from_values(-1, 0),
            Point2::from_values(1, 0),
            Real::one(),
        )
        .unwrap();
        let source = CurvePath2::try_new(vec![arc.into()])?;
        let options = BezierParallelVerificationOptions::try_new(q(1, 20), 16)?;
        let output = source.approximate_parallel_blend2d_certified(q(1, 4), &options)?;
        let arc = output.path().curves()[0]
            .subcurve(q(1, 5).into(), q(4, 5).into())?
            .subcurve(q(1, 4).into(), q(3, 4).into())?
            .reversed()?;
        assert_eq!(
            arc.parallel_fidelity().unwrap().metric(),
            CurveParallelFidelityMetric2::ExactImage
        );
        assert_eq!(
            arc.parallel_fidelity()
                .unwrap()
                .source_parameter_at(&q(1, 2).into())?,
            None
        );
        let mut curves = vec![arc.clone()];
        curves.extend(
            arc.native_bezier_fragments()?
                .iter()
                .cloned()
                .map(NativeBezierFragment2::into_curve),
        );
        for curve in &curves {
            if curve.geometry().is_some() {
                let definition = curve.to_native_parallel_fidelity_definition().unwrap();
                let restored = Curve2::from_native_parallel_fidelity_definition(
                    &definition,
                    SvgOptions::default(),
                )
                .unwrap();
                assert_eq!(
                    restored.parallel_fidelity().unwrap().metric(),
                    CurveParallelFidelityMetric2::ExactImage
                );
                assert!(curve.start().coincides_with(&restored.start())?);
                assert!(curve.end().coincides_with(&restored.end())?);
            }
        }
        replay(&curves);
        Ok(())
    })
    .unwrap();
}

#[test]
fn extracted_parallel_curves_keep_their_individual_source_and_metric() {
    support::under_value(&PredicatePolicy::STRICT, || -> ExactCurveResult<()> {
        let source = CurvePath2::try_new(vec![
            LineSeg2::try_new(Point2::from_values(-1, 0), Point2::from_values(0, 0))
                .unwrap()
                .into(),
            QuadraticBezier2::new(
                Point2::from_values(0, 0),
                Point2::from_values(1, 0),
                Point2::from_values(1, 1),
            )
            .into(),
        ])?;
        let options = BezierParallelVerificationOptions::try_new(q(1, 20), 16)?;
        let output = source.approximate_parallel_blend2d_certified(q(1, 10), &options)?;
        assert_eq!(output.exact_source_curve_count(), 1);
        assert_eq!(output.approximated_source_curve_count(), 1);
        let path = output.into_path();
        for curve in path.curves() {
            let receipt = curve
                .parallel_fidelity()
                .expect("per-output fidelity survives path extraction");
            assert!(receipt.source_curve_index() < 2);
            assert_eq!(
                receipt.metric(),
                if receipt.source_curve_index() == 0 {
                    CurveParallelFidelityMetric2::ExactImage
                } else {
                    CurveParallelFidelityMetric2::SameParameterEuclidean
                }
            );
            assert_eq!(
                receipt.source_curve(),
                &source.curves()[receipt.source_curve_index()]
            );
        }
        #[cfg(feature = "svg")]
        {
            let geometry = SvgGeometry2::new(CurveRegion2::empty(), vec![], vec![path.clone()]);
            let document = export_svg_document(&geometry).unwrap();
            let restored = import_svg_document(&document).unwrap();
            assert!(
                restored.paths()[0]
                    .curves()
                    .iter()
                    .all(|curve| curve.parallel_fidelity().is_some()),
                "native replay must preserve and revalidate each fidelity relation"
            );
        }
        Ok(())
    })
    .unwrap();
}

#[cfg(feature = "svg")]
#[test]
fn native_fidelity_admission_counts_syntax_without_certifying_scalars() {
    let (_, output) = mixed();
    let curve = output
        .path()
        .curves()
        .iter()
        .find(|c| {
            c.parallel_fidelity().unwrap().metric()
                == CurveParallelFidelityMetric2::SameParameterEuclidean
        })
        .unwrap();
    let definition = curve.to_native_parallel_fidelity_definition().unwrap();
    assert!(matches!(
        Curve2::inspect_native_parallel_fidelity_definition(&definition, 1),
        Err(SvgError::SizeOverflow { .. })
    ));
    let shape =
        Curve2::inspect_native_parallel_fidelity_definition(&definition, definition.len()).unwrap();
    assert_eq!(shape.records, 10);
    assert_eq!(shape.degree, 2);
    assert_eq!(shape.geometry_elements, 6);
    assert_eq!(shape.dependency_depth, 2);
    let restored =
        Curve2::from_native_parallel_fidelity_definition(&definition, SvgOptions::default())
            .unwrap();
    assert_eq!(
        restored.parallel_fidelity().unwrap().error_bound(),
        q(1, 20)
    );
    let mut bad = definition.clone();
    bad.replace_range(30..32, "ff");
    assert_eq!(
        Curve2::inspect_native_parallel_fidelity_definition(&bad, bad.len())
            .unwrap()
            .geometry_elements,
        6
    );
    assert!(Curve2::from_native_parallel_fidelity_definition(&bad, SvgOptions::default()).is_err());
    let mut bad_reference = definition;
    let length = bad_reference.len();
    bad_reference.replace_range(length - 8..length, "ffffffff");
    assert!(
        Curve2::inspect_native_parallel_fidelity_definition(&bad_reference, bad_reference.len())
            .is_err()
    );
    // PH cubic C=(t-t³/3,t²), C'=(1-t²,2t), speed 1+t².
    // Its offset has a degree-five rational numerator over 1+t².
    let points = [
        Point2::from_values(0, 0),
        Point2::new(q(1, 3), 0.into()),
        Point2::new(q(2, 3), q(1, 3)),
        Point2::new(q(2, 3), 1.into()),
    ];
    let cubic = CubicBezier2::new(
        points[0].clone(),
        points[1].clone(),
        points[2].clone(),
        points[3].clone(),
    );
    let rational = RationalBezier2::try_new(points.to_vec(), vec![Real::one(); 4]).unwrap();
    for source in [Curve2::from(cubic), Curve2::from(rational)] {
        let path = CurvePath2::try_new(vec![source]).unwrap();
        let output = path
            .approximate_parallel_blend2d_certified(
                q(1, 10),
                &BezierParallelVerificationOptions::try_new(q(1, 20), 16).unwrap(),
            )
            .unwrap();
        assert_eq!(output.exact_source_curve_count(), 1);
        let definition = output.path().curves()[0]
            .to_native_parallel_fidelity_definition()
            .unwrap();
        let shape =
            Curve2::inspect_native_parallel_fidelity_definition(&definition, definition.len())
                .unwrap();
        assert_eq!(shape.degree, 5);
        assert_eq!(shape.geometry_elements, 10);
        let curve =
            Curve2::from_native_parallel_fidelity_definition(&definition, SvgOptions::default())
                .unwrap();
        assert_eq!(
            curve.parallel_fidelity().unwrap().metric(),
            CurveParallelFidelityMetric2::ExactImage
        );
        assert_eq!(
            curve.parallel_fidelity().unwrap().error_bound(),
            Real::zero()
        );
    }
    let x = Real::from(2).sqrt().unwrap();
    let y = Real::from(3).sqrt().unwrap();
    let radius = Real::from(5).sqrt().unwrap();
    let line = LineSeg2::try_new(
        Point2::new(x.clone(), y.clone()),
        Point2::new(&x + Real::from(2), &y + Real::from(3)),
    )
    .unwrap();
    let arc = CircularArc2::try_from_center(
        Point2::new(&x + &radius, y.clone()),
        Point2::new(x.clone(), &y + &radius),
        Point2::new(x, y),
        false,
    )
    .unwrap();
    for source in [Curve2::from(line), Curve2::from(arc)] {
        let path = CurvePath2::try_new(vec![source]).unwrap();
        let output = path
            .approximate_parallel_blend2d_certified(
                q(1, 10),
                &BezierParallelVerificationOptions::try_new(q(1, 20), 16).unwrap(),
            )
            .unwrap();
        assert_eq!(output.exact_source_curve_count(), 1);
        let original = &output.path().curves()[0];
        let definition = original.to_native_parallel_fidelity_definition().unwrap();
        let restored =
            Curve2::from_native_parallel_fidelity_definition(&definition, SvgOptions::default())
                .unwrap();
        assert_eq!(
            restored.parallel_fidelity().unwrap().metric(),
            CurveParallelFidelityMetric2::ExactImage
        );
        assert!(original.start().coincides_with(&restored.start()).unwrap());
        assert!(original.end().coincides_with(&restored.end()).unwrap());
    }
}

#[test]
fn repeated_exact_edits_keep_a_compact_replay_and_the_pointwise_bound() {
    support::under_value(&PredicatePolicy::STRICT, || -> ExactCurveResult<()> {
        let (_, output) = mixed();
        let mut curve = output
            .path()
            .curves()
            .iter()
            .find(|c| {
                c.parallel_fidelity().unwrap().metric()
                    == CurveParallelFidelityMetric2::SameParameterEuclidean
            })
            .unwrap()
            .clone();
        let original = curve.clone();
        let forward = Similarity2::try_from_real_affine(
            2.into(),
            0.into(),
            0.into(),
            2.into(),
            3.into(),
            4.into(),
        )
        .unwrap();
        let backward = Similarity2::try_from_real_affine(
            q(1, 2),
            0.into(),
            0.into(),
            q(1, 2),
            q(-3, 2),
            (-2).into(),
        )
        .unwrap();
        for _ in 0..128 {
            curve = curve
                .reversed()?
                .reversed()?
                .transform_similarity(&forward)?
                .transform_similarity(&backward)?;
            curve = curve.native_bezier_fragments()?[0].clone().into_curve();
        }
        assert!(
            curve
                .parallel_fidelity()
                .unwrap()
                .image_transforms()
                .is_empty()
        );
        assert_eq!(
            curve.parallel_fidelity().unwrap().error_bound(),
            original.parallel_fidelity().unwrap().error_bound()
        );
        // Independent source formula: Q(t)=(2t-t²,t²), left unit normal
        // (-t,1-t)/sqrt(t²+(1-t)²). These point checks supplement the
        // continuous verifier, rather than replacing its interval proof.
        for local in [q(0, 1), q(1, 4), q(1, 2), q(3, 4), q(1, 1)] {
            let receipt = curve.parallel_fidelity().unwrap();
            let t = receipt
                .source_parameter_at(&local.clone().into())?
                .unwrap()
                .scalar()
                .unwrap()
                .clone();
            let one_minus = Real::one() - &t;
            let length = (&t * &t + &one_minus * &one_minus).sqrt().unwrap();
            let expected = Point2::new(
                Real::from(2) * &t - &t * &t - (q(1, 10) * &t / &length).unwrap(),
                &t * &t + (q(1, 10) * one_minus / length).unwrap(),
            );
            let point = curve.point_at(&local.into())?;
            let p = point.coordinates().unwrap();
            let dx = p.x() - expected.x();
            let dy = p.y() - expected.y();
            let slack = receipt.error_bound() * receipt.error_bound() - &dx * &dx - &dy * &dy;
            assert!(matches!(
                slack.certified_sign_until(-2048).sign(),
                Some(hyperreal::RealSign::Positive | hyperreal::RealSign::Zero)
            ));
        }
        #[cfg(feature = "svg")]
        {
            let restored = replay(&[curve]);
            let geometry = |c: Curve2| {
                SvgGeometry2::new(
                    CurveRegion2::empty(),
                    vec![],
                    vec![CurvePath2::try_new(vec![c]).unwrap()],
                )
            };
            assert_eq!(
                export_svg_document(&geometry(original)).unwrap(),
                export_svg_document(&geometry(restored[0].clone())).unwrap()
            );
        }
        Ok(())
    })
    .unwrap();
}

#[cfg(feature = "svg")]
#[test]
fn cold_process_rechecks_fidelity_before_further_editing() {
    const KEY: &str = "CSGRS_R24_COLD_DOCUMENT";
    if let Some(path) = std::env::var_os(KEY) {
        let geometry = import_svg_document(&std::fs::read_to_string(path).unwrap()).unwrap();
        let curves = geometry.paths()[0].curves();
        assert!(
            curves
                .iter()
                .any(|c| c.parallel_fidelity().unwrap().metric()
                    == CurveParallelFidelityMetric2::ExactImage)
        );
        assert!(
            curves
                .iter()
                .any(|c| c.parallel_fidelity().unwrap().metric()
                    == CurveParallelFidelityMetric2::SameParameterEuclidean)
        );
        let edited = curves
            .iter()
            .map(|c| {
                c.subcurve(q(1, 4).into(), q(3, 4).into())
                    .unwrap()
                    .reversed()
                    .unwrap()
            })
            .collect::<Vec<_>>();
        replay(&edited);
        return;
    }
    let (_, output) = mixed();
    let geometry = SvgGeometry2::new(CurveRegion2::empty(), vec![], vec![output.into_path()]);
    let document = export_svg_document(&geometry).unwrap();
    let path = std::env::temp_dir().join(format!("csgrs-r24-cold-{}.svg", std::process::id()));
    std::fs::write(&path, document).unwrap();
    let result = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "cold_process_rechecks_fidelity_before_further_editing",
            "--nocapture",
        ])
        .env(KEY, &path)
        .output();
    std::fs::remove_file(path).unwrap();
    let result = result.unwrap();
    assert!(
        result.status.success(),
        "{}\n{}",
        String::from_utf8_lossy(&result.stdout),
        String::from_utf8_lossy(&result.stderr)
    );
}
