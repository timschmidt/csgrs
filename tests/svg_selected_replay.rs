#![cfg(feature = "svg-io")]

use csgrs::errors::{ValidationError, ValidationErrorKind};
use csgrs::io::{
    IoError,
    svg::{export_svg, import_svg},
};
use hypercurve::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, Curve2, CurveParameterRange2, CurvePath2, CurveRegion2,
    ExactCurveError, Point2, QuadraticBezier2, Real,
};
use std::fmt::Write;

#[test]
fn svg_adapter_replays_selected_source_before_further_restriction() {
    let point = |x: i32, y: i32| Point2::new(x.into(), y.into());
    let source: Curve2 = QuadraticBezier2::new(point(0, 0), point(1, 2), point(2, 0)).into();
    let beta = BezierParameter2::Algebraic(
        BezierAlgebraicParameter2::try_isolate(
            BezierParameterPolynomial::try_new_power_basis(vec![
                (-1).into(),
                0.into(),
                2.into(),
            ])
            .unwrap(),
            BezierParameterInterval::try_new(
                (Real::one() / Real::from(2)).unwrap(),
                (Real::from(3) / Real::from(4)).unwrap(),
            )
            .unwrap(),
        )
        .unwrap(),
    );
    let curve = source
        .restrict_source(
            CurveParameterRange2::try_new(Real::zero().into(), beta.clone().into()).unwrap(),
        )
        .unwrap();
    let path = CurvePath2::try_new(vec![curve]).unwrap();
    let document = export_svg(&CurveRegion2::empty(), &[], &[path]).unwrap();
    let (region, wires, paths) = import_svg(&document).unwrap();
    assert!(region.is_empty() && wires.is_empty());
    let imported = &paths[0].curves()[0];
    assert_eq!(
        imported.source_restriction().unwrap().0.geometry(),
        source.geometry()
    );
    let smaller = imported
        .restrict_source(
            CurveParameterRange2::try_new(
                (Real::one() / Real::from(2)).unwrap().into(),
                beta.into(),
            )
            .unwrap(),
        )
        .unwrap();
    assert!(smaller.start().coincides_with(&point(1, 1).into()).unwrap());
}

#[test]
fn invalid_native_selector_keeps_its_typed_cause_at_the_svg_adapter() {
    // HCP4 with one selected-root record for t² + 1 on (0, 1).
    // The invalid definition must be rejected even though the final path is empty.
    let mut bytes = b"HCP4\x00\x00\x00\x01\x02\x00\x00\x00\x03".to_vec();
    for value in [1, 0, 1, 0, 1] {
        let encoded = Real::from(value).to_bytes();
        bytes.extend_from_slice(&u32::try_from(encoded.len()).unwrap().to_be_bytes());
        bytes.extend(encoded);
    }
    bytes.extend_from_slice(&0_u32.to_be_bytes());
    let mut extension = String::from("4:");
    for byte in bytes {
        write!(&mut extension, "{byte:02x}").unwrap();
    }
    let document = format!(
        r#"<svg xmlns="http://www.w3.org/2000/svg"><path fill="none" stroke="black" data-hypercurve-path="{extension}" d="M0 0 L1 1"/></svg>"#
    );
    let error = import_svg(&document).unwrap_err();
    let IoError::Construction {
        format: "SVG",
        source,
    } = error
    else {
        panic!("typed SVG replay failure required: {error:?}")
    };
    assert_eq!(source.kind(), ValidationErrorKind::InvalidInput);
    assert!(matches!(
        source,
        ValidationError::ExactCurve(ExactCurveError::Invalid { .. })
    ));
}
