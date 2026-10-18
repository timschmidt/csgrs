#![cfg(feature = "svg")]

use hypercurve::{Real, SvgError, import_svg_document};
use std::fmt::Write;

fn document_with_first_coordinate(invalid: &[u8]) -> String {
    let zero = Real::zero().to_bytes();
    let one = Real::one().to_bytes();
    // HCP3, one line, then its two points: (invalid, 0) to (1, 0).
    let mut bytes = b"HCP3\x00\x00\x00\x01\x00".to_vec();
    for coordinate in [invalid, &zero, &one, &zero] {
        bytes.extend_from_slice(&u32::try_from(coordinate.len()).unwrap().to_be_bytes());
        bytes.extend_from_slice(coordinate);
    }
    let mut extension = String::from("3:");
    for byte in bytes {
        write!(&mut extension, "{byte:02x}").unwrap();
    }
    format!(
        r#"<svg xmlns="http://www.w3.org/2000/svg"><path fill="none" stroke="black" data-hypercurve-path="{extension}" d="M0 0 L1 0"/></svg>"#
    )
}

#[test]
fn exact_svg_rejects_a_coordinate_without_its_required_scalar_payload() {
    // CBOR for rational=1, class=Irrational, computable=null. The map is
    // syntactically valid but cannot describe an opaque real value.
    let invalid = b"\xa3\x68rational\xa3\x64sign\x01\x69numerator\x81\x01\
        \x6bdenominator\x81\x01\x65class\x6aIrrational\x6acomputable\xf6";
    let document = document_with_first_coordinate(invalid);
    let error = import_svg_document(&document).unwrap_err();
    assert!(
        matches!(error, SvgError::MalformedInput(ref detail)
            if detail.contains("invalid exact coordinate") && detail.contains("ParseError")),
        "{error:?}",
    );
}

#[test]
fn exact_svg_rejects_undefined_scalar_graphs_before_publishing_a_path() {
    // CBOR Real records with rational=1 and class=Irrational. The respective
    // graphs are Sqrt(Int(-1)) and Inverse(Int(0)); neither denotes a real.
    for invalid in [
        b"\xa3\x68rational\xa3\x64sign\x01\x69numerator\x81\x01\
            \x6bdenominator\x81\x01\x65class\x6aIrrational\x6acomputable\
            \xa1\x68internal\xa1\x64Sqrt\xa1\x68internal\xa1\x63Int\x82\x20\x81\x01"
            .as_slice(),
        b"\xa3\x68rational\xa3\x64sign\x01\x69numerator\x81\x01\
            \x6bdenominator\x81\x01\x65class\x6aIrrational\x6acomputable\
            \xa1\x68internal\xa1\x67Inverse\xa1\x68internal\xa1\x63Int\x82\x00\x80"
            .as_slice(),
    ] {
        let document = document_with_first_coordinate(invalid);
        let error = import_svg_document(&document).unwrap_err();
        assert!(
            matches!(error, SvgError::MalformedInput(ref detail)
                if detail.contains("invalid exact coordinate") && detail.contains("ParseError")),
            "{error:?}",
        );
    }
}
