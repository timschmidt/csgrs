use super::*;
use crate::bezier_offset::{
    BezierAlgebraicCuspSemicircle2, BezierAlgebraicCuspSemicircleFragment2,
};
use crate::test_support::decided_ok;
use crate::{CurvePoint2, Point2, QuadraticBezier2};

fn graph_bytes(writer: GraphWriter) -> Vec<u8> {
    let mut bytes = writer.count.to_be_bytes().to_vec();
    bytes.extend_from_slice(&writer.writer.bytes);
    bytes
}

fn decoded_circle(bytes: &[u8], depth: usize) -> SvgResult<BezierAlgebraicCuspSemicircle2> {
    let mut reader = ExactReader::new(bytes, 1_048_576);
    let definitions = read_definitions(&mut reader, depth, 64)?;
    assert!(reader.is_empty());
    match definitions.last() {
        Some(Definition::Circle(circle)) => Ok(circle.clone()),
        _ => panic!("the test record ends with a circle definition"),
    }
}

#[test]
fn replay_parallel_contact_checks_untrusted_graph_records_before_publishing() {
    let record = |bad_reference: Option<(usize, u8)>, radius: Real, location: u8, value: Real| {
        let policy = CurveContext::STRICT;
        let source = QuadraticBezier2::new(pt!(-1, 0), pt!(0, 0), pt!(1, 0));
        let circle = decided_ok(
            BezierAlgebraicCuspSemicircle2::from_selected_parallel_normal(
                source.parallel_left(r!(0)).unwrap(),
                r!(1 / 2).into(),
                r!(1),
                false,
                &policy,
            ),
        )
        .unwrap();
        let other = QuadraticBezier2::new(pt!(-1, -1), pt!(-1, 0), pt!(-1, 1))
            .parallel_left(r!(0))
            .unwrap();
        let mut writer = GraphWriter::new(1_048_576, 256);
        let support = writer.circle(&circle).unwrap();
        let parallel = writer.parallel(&other).unwrap();
        let parameter = writer.scalar_parameter(&value.into()).unwrap();
        let mut references = [support, parallel, parameter];
        let index = writer.begin(circles::PARALLEL_NORMAL_CONTACT).unwrap();
        if let Some((slot, kind)) = bad_reference {
            references[slot] = match kind {
                0 => u32::MAX,
                1 => index,
                2 => references[(slot + 1) % references.len()],
                _ => unreachable!(),
            };
        }
        for reference in references {
            writer.writer.write_u32(reference).unwrap();
        }
        writer.writer.write_real(&radius).unwrap();
        writer.writer.write_u8(location).unwrap();
        graph_bytes(writer)
    };
    let decode = |bytes: &[u8], depth| {
        let mut reader = ExactReader::new(bytes, 1_048_576);
        read_definitions(&mut reader, depth, 64)
    };
    for slot in 0..3 {
        for kind in 0..3 {
            assert!(matches!(
                decode(&record(Some((slot, kind)), r!(1), 1, r!(1 / 2)), 256),
                Err(SvgError::MalformedInput(_))
            ));
        }
    }
    for location in [3, u8::MAX] {
        assert!(matches!(
            decode(&record(None, r!(1), location, r!(1 / 2)), 256),
            Err(SvgError::MalformedInput(_))
        ));
    }
    for (radius, location, value) in [
        (r!(1), 0, r!(1 / 2)),
        (r!(1), 2, r!(1 / 2)),
        (r!(0), 1, r!(1 / 2)),
        (r!(-1), 1, r!(1 / 2)),
        (r!(2), 1, r!(1 / 2)),
        (r!(1), 1, r!(1 / 4)),
    ] {
        assert!(matches!(
            decode(&record(None, radius, location, value), 256),
            Err(SvgError::Exact(ExactCurveError::Invalid {
                cause: crate::CurveError::InvalidCurveParameter,
                ..
            }))
        ));
    }
    let valid = record(None, r!(1), 1, r!(1 / 2));
    assert!(matches!(
        decode(&valid, 0),
        Err(SvgError::SizeOverflow { .. })
    ));
    for removed in [1, 2] {
        assert!(decode(&valid[..valid.len() - removed], 256).is_err());
    }
    let definitions = decode(&valid, 1).unwrap();
    let Some(Definition::CircleParameter { parameter, .. }) = definitions.last() else {
        panic!("a checked angular contact is published only after complete replay");
    };
    assert_eq!(
        decided_ok(parameter.order_to_real(&r!(1 / 2), &CurveContext::STRICT)),
        std::cmp::Ordering::Equal
    );
}

#[test]
fn replay_parallel_normal_frame_preserves_selected_center_and_oriented_image() {
    let parameter = BezierParameter2::Algebraic(
        BezierAlgebraicParameter2::try_isolate(
            BezierParameterPolynomial::try_new_power_basis(vec![r!(-1), r!(0), r!(2)]).unwrap(),
            BezierParameterInterval::try_new(r!(1 / 2), r!(1)).unwrap(),
        )
        .unwrap(),
    );
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for radius in [r!(2), r!(-2)] {
            for clockwise in [false, true] {
                let source = QuadraticBezier2::new(pt!(0, 0), pt!(1, 0), pt!(2, 0));
                let support = source.parallel_left(r!(3)).unwrap();
                let circle = decided_ok(
                    BezierAlgebraicCuspSemicircle2::from_selected_parallel_normal(
                        support,
                        parameter.clone().into(),
                        radius.clone(),
                        clockwise,
                        &policy,
                    ),
                )
                .unwrap();
                let mut writer = GraphWriter::new(1_048_576, 256);
                writer.circle(&circle).unwrap();
                let bytes = graph_bytes(writer);
                let restored = decoded_circle(&bytes, 256).unwrap();
                assert!(restored.uses_selected_parallel_normal_frame());
                let definition = restored.native_parallel_normal_definition().unwrap();
                assert_eq!(
                    definition
                        .center_parameter
                        .same_value(&parameter.clone().into(), &policy)
                        .unwrap(),
                    Classification::Decided(true)
                );
                // The source has unit tangent (1,0). Its selected center is
                // (sqrt(2),3), independent of the replay implementation.
                let x = r!(2).sqrt().unwrap();
                let expected: [CurvePoint2; 3] = [
                    Point2::new(x.clone(), r!(3) + &radius).into(),
                    Point2::new(
                        &x + if clockwise {
                            radius.clone()
                        } else {
                            -radius.clone()
                        },
                        r!(3),
                    )
                    .into(),
                    Point2::new(x, r!(3) - &radius).into(),
                ];
                for (value, point) in [r!(0), r!(1 / 2), r!(1)].into_iter().zip(&expected) {
                    let actual = decided_ok(restored.point_evidence_at(&value, &policy));
                    assert_eq!(
                        actual.same_point(point, &policy),
                        Classification::Decided(true)
                    );
                }
                let mut writer = GraphWriter::new(1_048_576, 256);
                writer.circle(&restored).unwrap();
                assert_eq!(graph_bytes(writer), bytes);

                for reversed in [false, true] {
                    let curve = Curve2::from(BezierAlgebraicCuspSemicircleFragment2::full(
                        restored.clone(),
                        &policy,
                    ));
                    let curve = if reversed {
                        curve.reversed().unwrap()
                    } else {
                        curve
                    };
                    let mut writer = GraphWriter::new(1_048_576, 256);
                    let reference = writer.curve(&curve).unwrap();
                    let bytes = graph_bytes(writer);
                    let mut reader = ExactReader::new(&bytes, 1_048_576);
                    let definitions = read_definitions(&mut reader, 256, 64).unwrap();
                    let replayed = curve_reference(&definitions, reference as usize).unwrap();
                    let Some(crate::BezierSplitFragment2::AlgebraicCuspSemicircle(fragment)) =
                        replayed.retained_fragment()
                    else {
                        panic!("the source-normal circle remains a selected circle");
                    };
                    assert_eq!(fragment.is_reversed(), reversed);
                    assert_eq!(
                        replayed.reversal_parameter_affine_map().unwrap(),
                        (r!(1), r!(0))
                    );
                    assert_eq!(
                        replayed
                            .start()
                            .same_point(&expected[if reversed { 2 } else { 0 }], &policy),
                        Classification::Decided(true)
                    );
                    assert_eq!(
                        replayed
                            .end()
                            .same_point(&expected[if reversed { 0 } else { 2 }], &policy),
                        Classification::Decided(true)
                    );
                    for (index, value) in [r!(0), r!(1 / 2), r!(1)].into_iter().enumerate() {
                        let actual = replayed.point_at(&value.into()).unwrap();
                        assert_eq!(
                            actual.same_point(&expected[index], &policy),
                            Classification::Decided(true)
                        );
                    }
                    let mut writer = GraphWriter::new(1_048_576, 256);
                    writer.curve(replayed).unwrap();
                    assert_eq!(graph_bytes(writer), bytes);
                }
            }
        }
    }
}

#[test]
fn replay_parallel_normal_frame_rechecks_radius_stationary_source_and_references() {
    let source = QuadraticBezier2::new(pt!(0, 0), pt!(0, 0), pt!(1, 0));
    let support = source.parallel_left(r!(3)).unwrap();
    for (parameter, radius, zero_radius) in [(r!(1 / 2), r!(0), true), (r!(0), r!(2), false)] {
        // Author deliberately untrusted records without using the constructor.
        let mut writer = GraphWriter::new(1_048_576, 256);
        let support = writer.parallel(&support).unwrap();
        let parameter = writer.scalar_parameter(&parameter.into()).unwrap();
        writer.begin(circles::PARALLEL_NORMAL_CIRCLE).unwrap();
        writer.writer.write_u32(support).unwrap();
        writer.writer.write_u32(parameter).unwrap();
        writer.writer.write_real(&radius).unwrap();
        writer.writer.write_bool(false).unwrap();
        let Err(SvgError::Exact(error)) = decoded_circle(&graph_bytes(writer), 256) else {
            panic!("the untrusted frame must retain its typed exact failure");
        };
        if zero_radius {
            assert!(matches!(
                error,
                ExactCurveError::Invalid {
                    cause: crate::CurveError::InvalidCurveParameter,
                    ..
                }
            ));
        } else {
            let ExactCurveError::Blocked(blocker) = error else {
                panic!("an undefined normal retains the source owner's blocker");
            };
            assert_eq!(blocker.reason(), crate::UncertaintyReason::Boundary);
        }
    }
    let pole = crate::RationalBezier2::from_homogeneous_controls(vec![
        crate::HomogeneousControl2::new(r!(0), r!(0), r!(1)),
        crate::HomogeneousControl2::new(r!(-1), r!(0), r!(-1)),
    ])
    .unwrap()
    .parallel_left(r!(3))
    .unwrap();
    let mut writer = GraphWriter::new(1_048_576, 256);
    let pole = writer.parallel(&pole).unwrap();
    let parameter = writer.scalar_parameter(&r!(1 / 2).into()).unwrap();
    writer.begin(circles::PARALLEL_NORMAL_CIRCLE).unwrap();
    writer.writer.write_u32(pole).unwrap();
    writer.writer.write_u32(parameter).unwrap();
    writer.writer.write_real(&r!(2)).unwrap();
    writer.writer.write_bool(false).unwrap();
    let Err(SvgError::Exact(ExactCurveError::Blocked(blocker))) =
        decoded_circle(&graph_bytes(writer), 256)
    else {
        panic!("a pole must retain the source owner's blocker");
    };
    assert_eq!(blocker.reason(), crate::UncertaintyReason::Boundary);
    for references in [[0, 2], [3, 2], [99, 2], [1, 1], [1, 3], [1, 99]] {
        let mut writer = GraphWriter::new(1_048_576, 256);
        assert_eq!(writer.parallel(&support).unwrap(), 1);
        assert_eq!(
            writer
                .parameter(&BezierParameter2::Exact(r!(1 / 2)))
                .unwrap(),
            2
        );
        assert_eq!(writer.begin(circles::PARALLEL_NORMAL_CIRCLE).unwrap(), 3);
        for reference in references {
            writer.writer.write_u32(reference).unwrap();
        }
        writer.writer.write_real(&r!(2)).unwrap();
        writer.writer.write_bool(false).unwrap();
        assert!(matches!(
            decoded_circle(&graph_bytes(writer), 256),
            Err(SvgError::MalformedInput(_))
        ));
    }
}

#[test]
fn parallel_normal_frame_depth_failure_does_not_poison_retry_or_policy() {
    let build = |policy: &CurveContext| {
        let source = QuadraticBezier2::new(pt!(0, 0), pt!(1, 0), pt!(2, 0));
        decided_ok(
            BezierAlgebraicCuspSemicircle2::from_selected_parallel_normal(
                source.parallel_left(r!(3)).unwrap(),
                r!(1 / 2).into(),
                r!(2),
                false,
                policy,
            ),
        )
        .unwrap()
    };
    let circle = build(&CurveContext::STRICT);
    let mut writer = GraphWriter::new(1_048_576, 256);
    writer.circle(&circle).unwrap();
    let bytes = graph_bytes(writer);
    assert!(matches!(
        decoded_circle(&bytes, 0),
        Err(SvgError::SizeOverflow { .. })
    ));
    assert!(decoded_circle(&bytes, 1).is_ok());
    let mut writer = GraphWriter::new(1_048_576, 0);
    assert!(matches!(
        writer.circle(&circle),
        Err(SvgError::SizeOverflow { .. })
    ));
    let mut writer = GraphWriter::new(1_048_576, 1);
    writer.circle(&circle).unwrap();
    assert_eq!(graph_bytes(writer), bytes);
    let mut writer = GraphWriter::new(16, 256);
    assert!(matches!(
        writer.circle(&circle),
        Err(SvgError::SizeOverflow { .. })
    ));
    let mut writer = GraphWriter::new(1_048_576, 256);
    writer.circle(&circle).unwrap();
    assert_eq!(graph_bytes(writer), bytes);
    let outcome =
        crate::policy::resolve_certified_operation(&CurveContext::APPROXIMATE_512, |policy| {
            policy.observe_approximate_512();
            Ok::<_, crate::CurveError>(build(policy))
        })
        .unwrap();
    assert_eq!(
        outcome.certainty,
        crate::CurveCertainty::Approximate512Consumed
    );
    let qualified = outcome.into_value();
    assert!(qualified.native_parallel_normal_definition().is_none());
    let mut writer = GraphWriter::new(1_048_576, 256);
    assert!(matches!(
        writer.circle(&qualified),
        Err(SvgError::Unsupported(_))
    ));
}

#[test]
fn parallel_normal_frame_replay_accepts_a_regular_source_at_a_center_locus_cusp() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        // C(t)=(t,t^2), k(0)=2: its distance-1/2 parallel has zero
        // derivative at t=0, while the original source normal stays regular.
        let source = QuadraticBezier2::new(pt!(0, 0), Point2::new(r!(1 / 2), r!(0)), pt!(1, 1));
        let support = source.parallel_left(r!(1 / 2)).unwrap();
        assert_eq!(
            decided_ok(support.parallel_derivative_scale_sign(&r!(0).into(), &policy)),
            hyperreal::RealSign::Zero
        );
        let circle = decided_ok(
            BezierAlgebraicCuspSemicircle2::from_selected_parallel_normal(
                support,
                r!(0).into(),
                r!(2),
                false,
                &policy,
            ),
        )
        .unwrap();
        let mut writer = GraphWriter::new(1_048_576, 256);
        writer.circle(&circle).unwrap();
        let bytes = graph_bytes(writer);
        let restored = decoded_circle(&bytes, 256).unwrap();
        assert_eq!(
            decided_ok(restored.center_point_evidence(&policy))
                .same_point(&Point2::new(r!(0), r!(1 / 2)).into(), &policy),
            Classification::Decided(true)
        );
        for (parameter, x, y) in [
            (r!(0), r!(0), r!(5 / 2)),
            (r!(1 / 2), r!(-2), r!(1 / 2)),
            (r!(1), r!(0), r!(-3 / 2)),
        ] {
            assert_eq!(
                decided_ok(restored.point_evidence_at(&parameter, &policy))
                    .same_point(&Point2::new(x, y).into(), &policy),
                Classification::Decided(true)
            );
        }
        let mut writer = GraphWriter::new(1_048_576, 256);
        writer.circle(&restored).unwrap();
        assert_eq!(graph_bytes(writer), bytes);
    }
}
