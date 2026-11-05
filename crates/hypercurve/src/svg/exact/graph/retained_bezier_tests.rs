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
