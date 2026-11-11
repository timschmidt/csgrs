use super::*;
use crate::{
    BezierParameterInterval, BezierParameterPolynomial, LineSeg2, Point2, RationalQuadraticBezier2,
};

fn point(x: i32, y: i32) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn ratio(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

/// The quarter of the unit circle from (1, 0) to (0, 1).
fn quarter() -> RationalQuadraticBezier2 {
    RationalQuadraticBezier2::try_new(
        point(1, 0),
        point(1, 1),
        point(0, 1),
        Real::one(),
        ratio(1, 2).sqrt().unwrap(),
        Real::one(),
    )
    .unwrap()
}

/// The root of `2t^2 - 1` in `(1/2, 1)`, an irrational split parameter.
fn root() -> BezierParameter2 {
    BezierParameter2::Algebraic(
        BezierAlgebraicParameter2::try_isolate(
            BezierParameterPolynomial::try_new_power_basis(vec![
                Real::from(-1),
                Real::zero(),
                Real::from(2),
            ])
            .unwrap(),
            BezierParameterInterval::try_new(ratio(1, 2), Real::one()).unwrap(),
        )
        .unwrap(),
    )
}

/// The quarter circle restricted to `[0, 1/sqrt 2]` in its own chart.
fn retained_piece() -> Curve2 {
    let range = BezierParameterRange2::new_validated(BezierParameter2::Exact(Real::zero()), root());
    let Classification::Decided(split) = crate::BezierSubcurve2::RationalQuadratic(quarter())
        .split_at_parameters_refined(&range, &[], &CurveContext::STRICT)
        .unwrap()
    else {
        panic!("the split is decided");
    };
    let [fragment] = split.fragments() else {
        panic!("one fragment");
    };
    assert!(matches!(
        fragment,
        crate::BezierSplitFragment2::RetainedBezier { .. }
    ));
    Curve2::from_retained_fragment(fragment.clone())
}

fn decode(writer: GraphWriter, reference: u32) -> SvgResult<Curve2> {
    let mut input = writer.count.to_be_bytes().to_vec();
    input.extend_from_slice(&writer.writer.bytes);
    let mut reader = ExactReader::new(&input, 1_048_576);
    let definitions = read_definitions(&mut reader, 256, 64)?;
    assert!(reader.is_empty());
    curve_reference(&definitions, reference as usize).cloned()
}

fn replay(curve: &Curve2) -> SvgResult<Curve2> {
    let mut writer = GraphWriter::new(1_048_576, 256);
    let reference = writer.curve(curve)?;
    decode(writer, reference)
}

fn coincide(first: &crate::CurvePoint2, second: &crate::CurvePoint2) -> bool {
    first.same_point(second, &CurveContext::STRICT) == Classification::Decided(true)
}

#[test]
fn replay_rebuilds_retained_bezier_ranges_in_their_source_chart() {
    for reversed in [false, true] {
        let piece = if reversed {
            retained_piece().reversed().unwrap()
        } else {
            retained_piece()
        };
        let restored = replay(&piece).unwrap();
        let Some(crate::BezierSplitFragment2::RetainedBezier {
            reversed: restored_reversed,
            start,
            end,
            ..
        }) = restored.retained_fragment()
        else {
            panic!("the piece keeps its source chart");
        };
        assert_eq!(*restored_reversed, reversed);
        assert_eq!(
            start.cmp_by_refinement_with_policy(
                &BezierParameter2::Exact(Real::zero()),
                &CurveContext::STRICT
            ),
            Ok(Classification::Decided(std::cmp::Ordering::Equal))
        );
        assert_eq!(
            end.cmp_by_refinement_with_policy(&root(), &CurveContext::STRICT),
            Ok(Classification::Decided(std::cmp::Ordering::Equal))
        );
        assert!(coincide(&restored.start(), &piece.start()));
        assert!(coincide(&restored.end(), &piece.end()));
        let path = CurvePath2::try_new(vec![piece]).unwrap();
        let encoded = super::super::encode_path(&path, 1_048_576, 64).unwrap();
        let restored = super::super::decode_path(&encoded, 1_048_576, 64, 256).unwrap();
        assert!(coincide(&restored.start(), &path.start()));
        assert!(coincide(&restored.end(), &path.end()));
        assert_eq!(
            super::super::encode_path(&restored, 1_048_576, 64).unwrap(),
            encoded
        );
        let geometry =
            crate::SvgGeometry2::new(crate::CurveRegion2::empty(), Vec::new(), vec![path.clone()]);
        let document = crate::export_svg_document(&geometry).unwrap();
        let imported = crate::import_svg_document(&document).unwrap();
        let [restored] = imported.paths() else {
            panic!("one standalone retained Bezier path");
        };
        assert!(coincide(&restored.start(), &path.start()));
        assert!(coincide(&restored.end(), &path.end()));
    }
}

#[test]
fn replay_rejects_invalid_retained_bezier_ranges() {
    let record = |source: Curve2, start: Real, end: Real| {
        let mut writer = GraphWriter::new(1_048_576, 256);
        let source = writer.native_curve(&source).unwrap();
        let start = writer.parameter(&BezierParameter2::Exact(start)).unwrap();
        let end = writer.parameter(&BezierParameter2::Exact(end)).unwrap();
        let index = writer.begin(RETAINED_BEZIER).unwrap();
        for reference in [source, start, end] {
            writer.writer.write_u32(reference).unwrap();
        }
        writer.writer.write_bool(false).unwrap();
        decode(writer, index)
    };
    let quarter = Curve2::from(quarter());
    // Descending, empty and out-of-chart ranges.
    for (start, end) in [
        (ratio(3, 4), ratio(1, 4)),
        (ratio(1, 2), ratio(1, 2)),
        (ratio(-1, 2), ratio(1, 2)),
        (ratio(1, 2), ratio(3, 2)),
    ] {
        assert!(matches!(
            record(quarter.clone(), start, end),
            Err(SvgError::Exact(_))
        ));
    }
    let line = Curve2::from(LineSeg2::try_new(point(0, 0), point(1, 0)).unwrap());
    assert!(matches!(
        record(line, ratio(1, 4), ratio(3, 4)),
        Err(SvgError::MalformedInput(_))
    ));
    // A valid record over the same source decodes.
    assert!(record(quarter, ratio(1, 4), ratio(3, 4)).is_ok());
}

#[test]
fn replay_preserves_selected_rational_endpoint_authority() {
    use crate::bezier_split::{BezierSelectedFiberFragment2, BezierSelectedFiberSource2};
    use crate::{CurvePointDefinition2, RationalBezier2};

    let strict = &CurveContext::STRICT;
    let source = RationalBezier2::try_new(
        vec![point(0, 0), point(1, 2), point(3, 1)],
        vec![Real::one(), ratio(2, 3), Real::one()],
    )
    .unwrap();
    let end = root().into();
    let contact = source
        .parallel_left(Real::zero())
        .unwrap()
        .checked_native_point(&end, strict)
        .unwrap();
    let piece: Curve2 = BezierSelectedFiberFragment2::new(
        BezierSelectedFiberSource2::Rational(source),
        CurveParameterRange2::new_validated(Real::zero().into(), end),
        point(0, 0).into(),
        contact.clone(),
    )
    .into();
    for piece in [piece.clone(), piece.reversed().unwrap()] {
        let restored = replay(&piece).unwrap();
        let Some(crate::BezierSplitFragment2::SelectedFiber(fragment)) =
            restored.retained_fragment()
        else {
            panic!("the replayed piece retains its selected rational carrier");
        };
        assert!(fragment.rational_curve().is_some());
        assert_eq!(fragment.is_reversed(), piece.source_traversal_is_reversed());
        assert!(coincide(&restored.start(), &piece.start()));
        assert!(coincide(&restored.end(), &piece.end()));
        let endpoint = if fragment.is_reversed() {
            restored.start()
        } else {
            restored.end()
        };
        let CurvePointDefinition2::Parallel { source, parameter } = endpoint.definition().unwrap()
        else {
            panic!("the selected contact retains its original point definition");
        };
        assert!(source.distance().definitely_zero());
        assert_eq!(
            parameter.compare(&root().into()).unwrap(),
            std::cmp::Ordering::Equal
        );
        let path = CurvePath2::try_new(vec![piece]).unwrap();
        assert!(matches!(
            super::super::encode_path(&path, 1, 64),
            Err(SvgError::SizeOverflow { .. })
        ));
        assert!(matches!(
            super::super::encode_path(&path, 1_048_576, 0),
            Err(SvgError::SizeOverflow { .. })
        ));
        let encoded = super::super::encode_path(&path, 1_048_576, 64).unwrap();
        assert!(matches!(
            super::super::decode_path(&encoded, 1_048_576, 0, 256),
            Err(SvgError::SizeOverflow { .. })
        ));
        let restored = super::super::decode_path(&encoded, 1_048_576, 64, 256).unwrap();
        assert!(coincide(&restored.start(), &path.start()));
        assert!(coincide(&restored.end(), &path.end()));
        assert_eq!(
            super::super::encode_path(&restored, 1_048_576, 64).unwrap(),
            encoded
        );
    }
}

#[test]
fn replay_rejects_forged_selected_rational_ranges() {
    use crate::{HomogeneousControl2, RationalBezier2};

    let record = |source: Curve2, start: i32, end: i32, upper: Point2, wrong_reference: bool| {
        let mut writer = GraphWriter::new(1_048_576, 256);
        let source = writer.native_curve(&source).unwrap();
        let start = writer
            .parameter(&BezierParameter2::Exact(Real::from(start)))
            .unwrap();
        let end = writer
            .parameter(&BezierParameter2::Exact(Real::from(end)))
            .unwrap();
        let lower = writer.point(&point(0, 0).into()).unwrap();
        let upper = writer.point(&upper.into()).unwrap();
        let index = writer.begin(selected_rational::RANGE).unwrap();
        for reference in [
            source,
            start,
            end,
            lower,
            if wrong_reference { index } else { upper },
        ] {
            writer.writer.write_u32(reference).unwrap();
        }
        writer.writer.write_bool(false).unwrap();
        decode(writer, index)
    };
    let rational = Curve2::from(
        RationalBezier2::try_new(vec![point(0, 0), point(1, 0)], vec![Real::one(); 2]).unwrap(),
    );
    let pole = Curve2::from(
        RationalBezier2::from_homogeneous_controls(vec![
            HomogeneousControl2::new(Real::zero(), Real::zero(), Real::one()),
            HomogeneousControl2::new(Real::from(-1), Real::zero(), Real::from(-1)),
        ])
        .unwrap(),
    );
    assert!(record(rational.clone(), 0, 1, point(1, 0), false).is_ok());
    for (result, reason) in [
        (
            record(rational.clone(), 0, 1, point(2, 0), false),
            "disagrees",
        ),
        (
            record(rational.clone(), 0, 1, point(1, 0), true),
            "point reference",
        ),
        (record(rational, 1, 0, point(0, 0), false), "ascending"),
        (record(pole, 0, 1, point(1, 0), false), "source pole"),
    ] {
        assert!(matches!(result, Err(SvgError::MalformedInput(detail)) if detail.contains(reason)));
    }
}
