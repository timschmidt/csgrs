#![cfg(feature = "svg")]

mod support;

use hypercurve::{
    CurveCertainty, PredicatePolicy, SvgError, SvgOptions, export_svg_document,
    import_svg_document, import_svg_document_with_options,
};

#[test]
fn hidden_path_checks_authored_grammar_before_returning_empty() {
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        for fields in [
            "",
            r#"d="M 0 0 L 1""#,
            r#"d="M 0 0 L NaN 1""#,
            r#"d="M 0 0 X 1 1""#,
            r#"d="M 0 0 A 2 1 0 2 1 4 0""#,
            r#"d="M 0 0 A 2 1 0 0 1 4 0 Z L""#,
        ] {
            for style in [
                r#"display="none""#,
                r#"visibility="hidden""#,
                r#"opacity="0""#,
            ] {
                let document = format!("<svg><g {style}><path {fields}/></g></svg>");
                assert!(
                    matches!(support::under(&policy, || import_svg_document(&document)),
                        Err(SvgError::MalformedInput(detail)) if !detail.is_empty()),
                    "{document}"
                );
            }
        }
    }
}

#[test]
fn hidden_path_keeps_valid_unselected_commands_empty() {
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        let outcome = support::under(&policy, || {
            for data in ["", "M0 0", "M0 0 H1 V1 H0 Z", "M0 0 A2 1 0 014 0 Z"] {
                let document = format!(r#"<svg><path opacity="0" d="{data}"/></svg>"#);
                assert!(import_svg_document(&document)?.is_empty(), "{document}");
            }
            assert!(matches!(
                import_svg_document(r#"<svg><path d="M0 0 A2 1 0 014 0 Z"/></svg>"#),
                Err(SvgError::Unsupported(_))
            ));
            Ok::<_, SvgError>(())
        })
        .unwrap();
        assert_eq!(outcome.certainty, CurveCertainty::Certified);
    }
}

#[test]
fn hidden_path_validates_native_metadata_and_uses_its_authority() {
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        let outcome = support::under(&policy, || {
            let square = import_svg_document(r#"<svg><path d="M0 0 H1 V1 H0 Z"/></svg>"#)?;
            let native = export_svg_document(&square)?;
            let marker = " data-hypercurve-path=\"";
            let extension = native
                .split_once(marker)
                .unwrap()
                .1
                .split('"')
                .next()
                .unwrap();
            let hidden = |extension| {
                format!(r#"<svg><path visibility="hidden"{marker}{extension}" d="X"/></svg>"#)
            };
            let valid = hidden(extension);
            assert!(import_svg_document(&valid)?.is_empty());
            assert!(matches!(
                import_svg_document_with_options(
                    &valid,
                    SvgOptions {
                        max_extension_bytes: 64,
                        ..SvgOptions::default()
                    }
                ),
                Err(SvgError::SizeOverflow { .. })
            ));
            assert!(matches!(
                import_svg_document(&hidden("3:0")),
                Err(SvgError::MalformedInput(detail)) if detail.contains("odd hexadecimal length")
            ));
            assert!(matches!(
                import_svg_document(&hidden("999:00")),
                Err(SvgError::Unsupported(detail)) if detail.contains("extension version")
            ));
            Ok::<_, SvgError>(())
        })
        .unwrap();
        assert_eq!(outcome.certainty, CurveCertainty::Certified);
    }
}
