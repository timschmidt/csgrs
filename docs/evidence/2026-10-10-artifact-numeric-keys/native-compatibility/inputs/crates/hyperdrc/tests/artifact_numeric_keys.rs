use hyperdrc::checks::{
    TextArtifact, TextArtifactParserKind, production_artifact_readiness,
    text_artifact_parser_diagnostics,
};

fn artifact(path: &str, text: String) -> TextArtifact {
    TextArtifact {
        path: path.to_owned(),
        text,
    }
}

fn centroid(rows: &str) -> TextArtifact {
    artifact(
        "centroid.csv",
        format!(
            "# units mm; origin aux origin; rotation convention clockwise degrees\n\
             Ref,X,Y,Rotation,Side,Value,Footprint\n{rows}"
        ),
    )
}

fn messages(bom: &[TextArtifact], centroid: &[TextArtifact]) -> Vec<String> {
    production_artifact_readiness(bom, centroid, &[], &[], &[], &[], &[])
        .into_iter()
        .filter_map(|v| v.message)
        .collect()
}

#[test]
fn placement_identity_preserves_small_differences_and_numeric_aliases() {
    for (left, right, equal) in [
        ("0.000111", "0.000114", false),
        ("1e20", "100000000000000000001", false),
        ("1e-400", "2e-400", false),
        ("-0.00001", "0.00001", false),
        ("0.1", "1/10", true),
        ("+0.0", "-0", true),
        ("1e-9", "0.000000001", true),
        ("1/3", "2/6", true),
        ("0.5mm", "1/2", true),
    ] {
        for axis in [0, 1] {
            for side in ["F.Cu", "Bottom"] {
                let mut a = ["2", "2"];
                let mut b = a;
                a[axis] = left;
                b[axis] = right;
                let rows = format!(
                    "R1,{},{},0,Top,10k,0603\nR2,{},{},0,{side},10k,0603\n",
                    a[0], a[1], b[0], b[1]
                );
                let messages = messages(&[], &[centroid(&rows)]);
                assert_eq!(
                    messages
                        .iter()
                        .any(|m| m.contains("places multiple references")),
                    equal && side == "F.Cu",
                    "axis={axis}, side={side}, left={left}, right={right}: {messages:?}"
                );
            }
        }
    }
}

#[test]
fn rotation_parity_uses_exact_euclidean_remainders() {
    for (left, right, equal) in [
        ("10.00011", "10.00014", false),
        ("0.0001", "360.0002", false),
        ("1e-400", "2e-400", false),
        ("-1e-400", "0", false),
        ("-180", "180", true),
        ("-720", "0", true),
        ("360", "-0", true),
        ("10.00011", "370.00011", true),
        ("1/3", "1081/3", true),
        ("90deg", "450", true),
    ] {
        let bom = artifact(
            "bom.csv",
            format!("Ref,Qty,MPN,Rotation\nR1,1,RC0603,{left}\n"),
        );
        let messages = messages(
            &[bom],
            &[centroid(&format!("R1,0,0,{right},Top,10k,0603\n"))],
        );
        assert_eq!(
            messages.iter().any(|m| m.contains("BOM rotation for R1")),
            !equal,
            "left={left}, right={right}: {messages:?}"
        );
    }
}

#[test]
fn distinct_close_rotations_reach_duplicate_and_polarized_checks() {
    let messages = messages(
        &[],
        &[centroid(
            "R1,0,0,10.00011,Top,10k,0603\nR1,1,0,10.00014,Top,10k,0603\n",
        )],
    );
    assert!(
        messages
            .iter()
            .any(|m| m.contains("R1 appears with multiple rotations"))
    );
    let bom = artifact(
        "bom.csv",
        "Ref,Qty,MPN,Value,Footprint,Polarity\n\
         D1,1,LED0603,LED,0603 LED,Cathode mark\n\
         D2,1,LED0603,LED,0603 LED,Cathode mark\n"
            .to_owned(),
    );
    let messages = self::messages(
        &[bom],
        &[centroid(
            "D1,0,0,10.00011,Top,LED,0603 LED\nD2,1,0,10.00014,Top,LED,0603 LED\n",
        )],
    );
    assert!(
        messages
            .iter()
            .any(|m| m.contains("at multiple centroid rotations"))
    );
}

#[test]
fn exhausted_numbers_are_visible_and_cannot_form_false_matches() {
    for token in ["1e1000000000", "1e-1000000000"] {
        let bom = artifact(
            "bom.csv",
            format!("Ref,Qty,MPN,Rotation\nR1,1,RC0603,{token}\n"),
        );
        let placements = centroid(&format!(
            "R1,{token},0,{token},Top,10k,0603\nR2,{token},0,{token},Top,10k,0603\n"
        ));
        let diagnostics =
            text_artifact_parser_diagnostics(&placements, TextArtifactParserKind::Centroid);
        assert_eq!(diagnostics.len(), 4, "{diagnostics:?}");
        assert!(
            diagnostics.iter().all(
                |d| d.code.ends_with("::number-unavailable") && d.message.contains("Exhausted")
            )
        );
        let messages = messages(&[bom], &[placements]);
        assert_eq!(
            messages
                .iter()
                .filter(|m| m.contains("unavailable exact"))
                .count(),
            5,
            "{messages:?}"
        );
        assert!(!messages.iter().any(|m| m.contains("places multiple references") || m.contains("BOM rotation for")));
    }
}

#[test]
fn parser_and_readiness_share_numeric_syntax_and_failure_classes() {
    let placements = centroid("R1,0.5mm,1/2,90deg,Top,10k,0603\nR2,NaN,1/0,bad,Top,10k,0603\n");
    let diagnostics =
        text_artifact_parser_diagnostics(&placements, TextArtifactParserKind::Centroid);
    assert_eq!(diagnostics.len(), 3, "{diagnostics:?}");
    assert!(
        diagnostics
            .iter()
            .all(|d| d.code.ends_with("::invalid-number"))
    );
    let messages = messages(&[], &[placements]);
    assert_eq!(
        messages
            .iter()
            .filter(|m| m.contains("has invalid"))
            .count(),
        3,
        "{messages:?}"
    );
}
