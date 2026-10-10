use hyperdrc::Scalar;
use hyperdrc::config::{EffectiveRules, RuleConfig, RuleOverrides, effective_rules};
use hyperdrc::kicad_dru::{
    KicadDruError, ThresholdFailure, merge_rules_into_kicad_dru, rules_to_kicad_dru,
};
use std::sync::{Arc, atomic::AtomicBool};

fn rules_with_width(width: Scalar) -> EffectiveRules {
    effective_rules(
        &RuleConfig::default(),
        RuleOverrides {
            min_width: Some(width),
            ..RuleOverrides::default()
        },
    )
}

fn assert_width(width: Scalar, expected: &str) {
    for prepared in [false, true] {
        let rules = rules_with_width(width.clone());
        if prepared {
            let _ = rules.min_width.certified_rational_interval(-256);
        }
        let deck = rules_to_kicad_dru(&rules).unwrap();
        assert!(
            deck.contains(&format!("(constraint track_width (min {expected}mm))")),
            "unexpected deck for {expected}: {deck}"
        );
    }
}

#[test]
fn exact_thresholds_round_on_the_nanometer_grid() {
    for (source, expected) in [
        ("0.15", "0.15"),
        ("1", "1"),
        ("10", "10"),
        ("0.0000005", "0.000001"),
        ("0.00000149999999999999999999999999", "0.000001"),
        ("0.0000015", "0.000002"),
        ("0.0000025", "0.000003"),
        ("0.12345649999999999999999999999999", "0.123456"),
        ("0.1234565", "0.123457"),
        ("0.1234567", "0.123457"),
        ("0.9999995", "1"),
        ("2147.483647", "2147.483647"),
    ] {
        assert_width(source.parse().unwrap(), expected);
    }
}

#[test]
fn computable_thresholds_require_matching_exact_enclosure_rounding() {
    assert_width(Scalar::pi(), "3.141593");
    assert_width(Scalar::from(2).sqrt().unwrap(), "1.414214");
    assert_width(unresolved_zero() + "0.2".parse::<Scalar>().unwrap(), "0.2");
}

#[test]
fn tiny_or_large_thresholds_cannot_become_a_different_rule() {
    for (tokens, reason) in [
        (
            ["1e-7", "1e-40", "1e-400", "0.0000004999999999999999999999"],
            ThresholdFailure::BelowResolution,
        ),
        (
            ["1e20", "1e400", "2147.483648", "2147.48364700000000000001"],
            ThresholdFailure::OutOfRange,
        ),
    ] {
        for token in tokens {
            let rules = rules_with_width(token.parse().unwrap());
            assert_eq!(
                rules_to_kicad_dru(&rules),
                Err(KicadDruError::Threshold {
                    rule: "hyperdrc-track-width",
                    reason,
                }),
                "{token}"
            );
        }
    }
}

fn unresolved_zero() -> Scalar {
    let sine = Scalar::e().sin();
    let cosine = Scalar::e().cos();
    &sine * &sine + &cosine * &cosine - Scalar::one()
}

#[test]
fn unresolved_sign_and_rounding_are_errors_with_cold_or_warm_caches() {
    for width in [
        unresolved_zero(),
        unresolved_zero() + "0.1234565".parse::<Scalar>().unwrap(),
    ] {
        for prepared in [false, true] {
            let rules = rules_with_width(width.clone());
            if prepared {
                let _ = rules.min_width.certified_rational_interval(-512);
            }
            assert_eq!(
                rules_to_kicad_dru(&rules),
                Err(KicadDruError::Threshold {
                    rule: "hyperdrc-track-width",
                    reason: ThresholdFailure::Uncertified,
                })
            );
        }
    }
}

#[test]
fn final_value_abort_cannot_export_a_zero_minimum() {
    for prepared in [false, true] {
        let mut width = Scalar::pi();
        if prepared {
            let _ = width.certified_rational_interval(-256);
        }
        width.abort(Arc::new(AtomicBool::new(true)));
        assert_eq!(
            rules_to_kicad_dru(&rules_with_width(width)),
            Err(KicadDruError::Threshold {
                rule: "hyperdrc-track-width",
                reason: ThresholdFailure::Aborted,
            })
        );
    }
}

#[test]
fn every_exported_field_is_checked_and_nonpositive_values_are_omitted() {
    for field in 0..5 {
        for token in ["0", "-0.1", "1e40"] {
            let mut rules = rules_with_width("0.15".parse().unwrap());
            let (target, name) = match field {
                0 => (&mut rules.net_clearance, "hyperdrc-copper-clearance"),
                1 => (&mut rules.min_width, "hyperdrc-track-width"),
                2 => (&mut rules.annular_ring, "hyperdrc-annular-ring"),
                3 => (&mut rules.clearance, "hyperdrc-silk-clearance"),
                _ => (&mut rules.drill_clearance, "hyperdrc-hole-clearance"),
            };
            *target = token.parse().unwrap();
            let result = rules_to_kicad_dru(&rules);
            if token == "1e40" {
                assert_eq!(
                    result,
                    Err(KicadDruError::Threshold {
                        rule: name,
                        reason: ThresholdFailure::OutOfRange,
                    })
                );
            } else {
                let deck = result.unwrap();
                assert!(!deck.contains(name));
                if field == 3 {
                    assert!(!deck.contains("hyperdrc-board-edge-clearance"));
                }
            }
        }
    }
}

#[test]
fn exhausted_shared_budget_prevents_publication() {
    use hypersolve::work_budget::{WorkBudget, WorkKind, consume, with_budget};
    let outcome = with_budget(WorkBudget::ZERO, || {
        let denied = consume(WorkKind::AlgebraicPromotion, 1).unwrap_err();
        assert_eq!(
            rules_to_kicad_dru(&rules_with_width(Scalar::one())),
            Err(KicadDruError::BudgetExceeded(denied))
        );
    });
    assert!(outcome.result.is_err());
}

#[test]
fn generated_comments_use_kicad_syntax() {
    let deck = rules_to_kicad_dru(&rules_with_width(Scalar::one())).unwrap();
    assert!(!deck.lines().any(|line| line.trim_start().starts_with(';')));
    assert!(deck.contains("# Generated by HyperDRC"));
    assert!(deck.contains("# Minimum copper width"));
}

#[test]
fn merge_preserves_same_line_rules_and_ignores_comment_parentheses() {
    let rules = rules_with_width(Scalar::one());
    let custom = " (rule \"keep-λ\" (condition \"A.NetName == 'paren()'\") (constraint clearance (min 0.2mm))) # keep ()\r\n";
    let owned = "(rule \"hyperdrc-track-width\"\n # unmatched comment (\n (constraint track_width (min 0.01mm)))";
    let existing = format!("(version 1)\r\n{owned}{custom}");
    let merged = merge_rules_into_kicad_dru(&existing, &rules).unwrap();
    assert!(merged.starts_with(&format!("(version 1)\r\n{custom}")));
    assert!(!merged.contains("0.01mm"));
    assert_eq!(merged.matches("(rule \"hyperdrc-track-width\"").count(), 1);
    assert_eq!(merge_rules_into_kicad_dru(&merged, &rules).unwrap(), merged);
}

#[test]
fn merge_keeps_unreserved_names_and_quoted_marker_text() {
    let custom = r##"(version 1)
# an unrelated comment containing ( ) and "
(rule 'hyperdrc-custom' (condition "A.NetName == 'quote\"()'"))
(rule "# BEGIN HYPERDRC GENERATED RULES" (condition "A.NetName == 'a#b'"))
"##;
    let rules = rules_with_width(Scalar::one());
    let merged = merge_rules_into_kicad_dru(custom, &rules).unwrap();
    assert!(merged.starts_with(custom));
    assert_eq!(merge_rules_into_kicad_dru(&merged, &rules).unwrap(), merged);
}

#[test]
fn merge_adds_a_header_and_is_idempotent_even_without_a_final_newline() {
    let rules = rules_with_width(Scalar::one());
    for input in ["", "# just a comment", "(rule \"custom\")", " \n\t"] {
        let merged = merge_rules_into_kicad_dru(input, &rules).unwrap();
        assert!(merged.starts_with("(version 1)\n"));
        assert_eq!(merged.matches("(version 1)").count(), 1);
        assert!(merged.contains(input));
        assert_eq!(merge_rules_into_kicad_dru(&merged, &rules).unwrap(), merged);
    }
}

#[test]
fn merge_migrates_only_known_legacy_comments_and_owned_blocks() {
    let rules = rules_with_width(Scalar::one());
    let custom = "(rule \"custom\" (constraint clearance (min 0.25mm)))\n";
    let old = format!(
        "(version 1)\n; Generated by HyperDRC. Review before merging with project-specific KiCad rules.\n; Units are millimeters.\n; BEGIN HYPERDRC GENERATED RULES\n; owned legacy comment\n(rule \"old-generated\")\n; END HYPERDRC GENERATED RULES\n{custom}"
    );
    let merged = merge_rules_into_kicad_dru(&old, &rules).unwrap();
    assert!(merged.contains(custom));
    assert!(!merged.contains("old-generated"));
    assert!(
        !merged
            .lines()
            .any(|line| line.trim_start().starts_with(';'))
    );
}

#[test]
fn malformed_decks_return_errors_instead_of_discarding_custom_rules() {
    let rules = rules_with_width(Scalar::one());
    for input in [
        "(version 2)",
        "(version 1 extra)",
        "(version (1))",
        "(version 1)(version 1)",
        "(rule \"custom\")(version 1)",
        "(version 1))",
        "(rule \"custom\"",
        "(rule \"unterminated)",
        "(rule \"new\nline\")",
        "(rule \"escape\\\nline\")",
        "(version 1)\n; unknown semicolon comment\n",
        "# BEGIN HYPERDRC GENERATED RULES\n(rule \"custom\")",
        "; END HYPERDRC GENERATED RULES\n(rule \"custom\")",
        "# BEGIN HYPERDRC GENERATED RULES\n# BEGIN HYPERDRC GENERATED RULES\n# END HYPERDRC GENERATED RULES\n",
        "# BEGIN HYPERDRC GENERATED RULES\n(version 1)\n# END HYPERDRC GENERATED RULES\n",
        "(rule \"custom\"\n# BEGIN HYPERDRC GENERATED RULES\n)\n# END HYPERDRC GENERATED RULES\n",
        "(version 1) # BEGIN HYPERDRC GENERATED RULES\n# END HYPERDRC GENERATED RULES\n",
    ] {
        assert!(
            matches!(
                merge_rules_into_kicad_dru(input, &rules),
                Err(KicadDruError::InvalidDeck { .. })
            ),
            "accepted malformed deck: {input}"
        );
    }
}

#[test]
fn cli_conversion_failures_preserve_existing_destination_bytes() {
    use clap::Parser;
    use std::sync::atomic::{AtomicU64, Ordering};
    static NEXT_ID: AtomicU64 = AtomicU64::new(0);
    struct Directory(std::path::PathBuf);
    impl Drop for Directory {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }
    let directory = Directory(std::env::temp_dir().join(format!(
        "hyperdrc-rule-export-{}-{}",
        std::process::id(),
        NEXT_ID.fetch_add(1, Ordering::Relaxed)
    )));
    std::fs::create_dir(&directory.0).unwrap();
    let board = directory.0.join("board.kicad_pcb");
    let destination = directory.0.join("rules.kicad_dru");
    std::fs::write(&board, include_str!("fixtures/rule-export.kicad_pcb")).unwrap();
    for merge in [false, true] {
        for width in ["1e-40", "1e20"] {
            let original = b"(version 1)\n(rule \"keep-destination\")\n";
            std::fs::write(&destination, original).unwrap();
            let mut arguments = vec![
                "hyperdrc",
                "--kicad-pcb",
                board.to_str().unwrap(),
                "--check",
                "drill-spacing",
                "--min-width",
                width,
            ];
            if merge {
                arguments.extend([
                    "--kicad-dru-merge-input",
                    destination.to_str().unwrap(),
                    "--kicad-dru-merge-output",
                    destination.to_str().unwrap(),
                ]);
            } else {
                arguments.extend(["--kicad-dru-output", destination.to_str().unwrap()]);
            }
            let cli = hyperdrc::Cli::try_parse_from(arguments).unwrap();
            let error = hyperdrc::run(cli).unwrap_err();
            assert!(error.downcast_ref::<KicadDruError>().is_some(), "{error:#}");
            assert_eq!(std::fs::read(&destination).unwrap(), original);
        }
    }
    let original = b"(version 1)\n# BEGIN HYPERDRC GENERATED RULES\n(rule \"keep-destination\")\n";
    std::fs::write(&destination, original).unwrap();
    let cli = hyperdrc::Cli::try_parse_from([
        "hyperdrc",
        "--kicad-pcb",
        board.to_str().unwrap(),
        "--check",
        "drill-spacing",
        "--kicad-dru-merge-input",
        destination.to_str().unwrap(),
        "--kicad-dru-merge-output",
        destination.to_str().unwrap(),
    ])
    .unwrap();
    let error = hyperdrc::run(cli).unwrap_err();
    assert!(
        matches!(
            error.downcast_ref::<KicadDruError>(),
            Some(KicadDruError::InvalidDeck { .. })
        ),
        "{error:#}"
    );
    assert_eq!(std::fs::read(&destination).unwrap(), original);
}
