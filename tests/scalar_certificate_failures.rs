use csgrs::errors::{ValidationError, ValidationErrorKind};
use hyperreal::Problem;

#[test]
fn scalar_uncertainty_remains_undecided_through_modeling_errors() {
    for problem in [Problem::UnknownCertificate, Problem::UnknownZero] {
        let direct = ValidationError::from(problem);
        assert_eq!(direct.kind(), ValidationErrorKind::Undecided);
        assert_eq!(direct.kind().as_str(), "undecided");
        assert!(matches!(direct, ValidationError::Scalar(source) if source == problem));

        let contour = ValidationError::from(hypercurve::MarchingSquaresError::Scalar(problem));
        assert_eq!(contour.kind(), ValidationErrorKind::Undecided);
        assert!(contour.to_string().contains(&problem.to_string()));
    }
    assert_eq!(
        ValidationError::from(Problem::Exhausted).kind(),
        ValidationErrorKind::ResourceExhausted,
    );
}

#[test]
#[cfg(any(feature = "sdf", feature = "metaballs"))]
fn field_and_sample_errors_preserve_the_undecided_scalar_cause() {
    let source = hypersdf::SdfEvaluationError::Scalar {
        operation: "import exact coordinate",
        source: Problem::UnknownCertificate,
    };
    for error in [
        ValidationError::from(source.clone()),
        ValidationError::SdfSample {
            sample: [1, 2, 3],
            source,
        },
    ] {
        assert_eq!(error.kind(), ValidationErrorKind::Undecided);
        let field = std::error::Error::source(&error).unwrap();
        let scalar = field.source().unwrap();
        assert_eq!(
            scalar.downcast_ref::<Problem>(),
            Some(&Problem::UnknownCertificate),
        );
    }
}

#[test]
#[cfg(feature = "svg-io")]
fn an_undecided_import_cannot_publish_a_model() {
    let wire = r#"{"rational":{"sign":1,"numerator":[1],"denominator":[1]},"class":"Pi","computable":{"internal":{"Constant":"E"}}}"#;
    let mut constructed = false;
    let result = csgrs::Real::from_json(wire)
        .map_err(ValidationError::from)
        .and_then(|side| {
            constructed = true;
            csgrs::solid::cube(side)
        });
    assert!(!constructed);
    assert_eq!(result.unwrap_err().kind(), ValidationErrorKind::Undecided);
}

#[test]
#[cfg(feature = "svg-io")]
fn svg_import_retains_the_typed_scalar_failure() {
    use csgrs::io::{IoError, svg::import_svg};
    use std::fmt::Write;

    // CBOR Real: rational=1, Class Pi, expression Constant E. The native
    // importer leaves this relationship unproved; the modeling adapter must
    // retain that outcome instead of converting it to invalid input or text.
    let unproved = b"\xa3\x68rational\xa3\x64sign\x01\x69numerator\x81\x01\
        \x6bdenominator\x81\x01\x65class\x62Pi\x6acomputable\xa1\
        \x68internal\xa1\x68Constant\x61E";
    let zero = csgrs::Real::zero().to_bytes();
    let one = csgrs::Real::one().to_bytes();
    let mut bytes = b"HCP3\x00\x00\x00\x01\x00".to_vec();
    for coordinate in [unproved.as_slice(), &zero, &one, &zero] {
        bytes.extend_from_slice(&u32::try_from(coordinate.len()).unwrap().to_be_bytes());
        bytes.extend_from_slice(coordinate);
    }
    let mut extension = String::from("3:");
    for byte in bytes {
        write!(&mut extension, "{byte:02x}").unwrap();
    }
    let document = format!(
        r#"<svg xmlns="http://www.w3.org/2000/svg"><path fill="none" stroke="black" data-hypercurve-path="{extension}" d="M0 0 L1 0"/></svg>"#,
    );
    let error = import_svg(&document).unwrap_err();
    assert!(matches!(
        error,
        IoError::Construction {
            format: "SVG",
            source: ValidationError::Scalar(Problem::UnknownCertificate),
        },
    ));
    let source = std::error::Error::source(&error).unwrap();
    let source = source.downcast_ref::<ValidationError>().unwrap();
    assert_eq!(source.kind(), ValidationErrorKind::Undecided);
}
