//! Public archive syntax admission, measured against an independent JSON tree.

use hyperbrep::{
    ArchiveLimitExceeded, ArchiveLimits, ArchiveResource, PersistenceError, Point3, RawModel, Real,
    builder,
};
use serde_json::{Value, json};

fn source() -> String {
    builder::cuboid(
        Point3::origin(),
        Point3::new(Real::from(2), Real::from(3), Real::from(4)),
    )
    .unwrap()
    .model()
    .to_json()
    .unwrap()
}

// This oracle operates on a parsed tree, independently of the production byte
// scanner. Keys and values are distinct tokens, including ignored input fields.
fn tree_usage(value: &Value) -> (usize, usize) {
    let children: Vec<_> = match value {
        Value::Array(values) => values.iter().map(tree_usage).collect(),
        Value::Object(values) => values.values().map(tree_usage).collect(),
        _ => return (0, 1),
    };
    let keys = value.as_object().map_or(0, |object| object.len());
    (
        1 + children.iter().map(|c| c.0).max().unwrap_or(0),
        1 + keys + children.iter().map(|c| c.1).sum::<usize>(),
    )
}

fn exact_limits(json: &str) -> ArchiveLimits {
    let tree: Value = serde_json::from_str(json).unwrap();
    let (nesting_depth, tokens) = tree_usage(&tree);
    ArchiveLimits {
        input_bytes: json.len(),
        nesting_depth,
        tokens,
        ..ArchiveLimits::default()
    }
}

fn exhausted(json: &str, limits: ArchiveLimits, expected: ArchiveLimitExceeded) {
    let error = RawModel::from_json(json, &limits).unwrap_err();
    assert_eq!(error, PersistenceError::ResourceLimit(expected));
    assert!(std::error::Error::source(&error).is_some());
    assert!(error.to_string().contains("limit exceeded"));
}

#[test]
fn each_input_limit_accepts_its_boundary_and_allows_an_exact_retry() {
    let json = source();
    let limits = exact_limits(&json);
    for resource in [
        ArchiveResource::InputBytes,
        ArchiveResource::NestingDepth,
        ArchiveResource::Tokens,
    ] {
        let mut reduced = limits;
        let allowance = match resource {
            ArchiveResource::InputBytes => &mut reduced.input_bytes,
            ArchiveResource::NestingDepth => &mut reduced.nesting_depth,
            ArchiveResource::Tokens => &mut reduced.tokens,
            _ => unreachable!("this test enumerates syntax limits"),
        };
        *allowance -= 1;
        let expected = ArchiveLimitExceeded {
            resource,
            limit: *allowance,
            observed: *allowance + 1,
        };
        exhausted(&json, reduced, expected);
        let loaded = RawModel::from_json(&json, &limits)
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(loaded.to_json().unwrap(), json);
        assert_eq!(
            loaded.solids().next().unwrap().volume().unwrap(),
            Real::from(24)
        );
    }
}

#[test]
fn whitespace_and_multibyte_strings_consume_the_byte_allowance() {
    let mut tree: Value = serde_json::from_str(&source()).unwrap();
    tree["ignored"] = json!("π 😀");
    let json = format!(" \t\r\n{tree}\n ");
    let limits = exact_limits(&json);
    let reduced = ArchiveLimits {
        input_bytes: json.chars().count(),
        ..limits
    };
    exhausted(
        &json,
        reduced,
        ArchiveLimitExceeded {
            resource: ArchiveResource::InputBytes,
            limit: reduced.input_bytes,
            observed: json.len(),
        },
    );
    RawModel::from_json(&json, &limits)
        .unwrap()
        .validate()
        .unwrap();
}

#[test]
fn quoted_delimiters_and_escapes_do_not_inflate_structure_counts() {
    let mut tree: Value = serde_json::from_str(&source()).unwrap();
    tree["quote\"\\[]{}"] = json!([
        "{{[[]]}}",
        "\" \\ \\\" \\u005b",
        "π\n\t",
        true,
        null,
        -1.25e30
    ]);
    let json = tree.to_string();
    let limits = exact_limits(&json);
    let parsed = RawModel::from_json(&json, &limits).unwrap();
    assert_eq!(parsed.validate().unwrap().to_json().unwrap(), source());
    exhausted(
        &json,
        ArchiveLimits {
            tokens: limits.tokens - 1,
            ..limits
        },
        ArchiveLimitExceeded {
            resource: ArchiveResource::Tokens,
            limit: limits.tokens - 1,
            observed: limits.tokens,
        },
    );
}

#[test]
fn unknown_fields_consume_limits_before_version_or_record_decoding() {
    // The version is intentionally old: admission must precede the header pass.
    // Avoid a recursive value tree so the hostile fixture itself is stack-safe.
    let deep = format!(
        "{{\"version\":8,\"ignored\":{}0{}}}",
        "[".repeat(50_000),
        "]".repeat(50_000)
    );
    let limits = ArchiveLimits {
        nesting_depth: 16,
        ..ArchiveLimits::default()
    };
    exhausted(
        &deep,
        limits,
        ArchiveLimitExceeded {
            resource: ArchiveResource::NestingDepth,
            limit: 16,
            observed: 17,
        },
    );
    let wide = format!("{{\"version\":8,\"ignored\":[{}0]}}", "0,".repeat(20_000));
    let limits = ArchiveLimits {
        tokens: 32,
        ..ArchiveLimits::default()
    };
    exhausted(
        &wide,
        limits,
        ArchiveLimitExceeded {
            resource: ArchiveResource::Tokens,
            limit: 32,
            observed: 33,
        },
    );
    exhausted(
        "not JSON",
        ArchiveLimits {
            input_bytes: 0,
            ..ArchiveLimits::default()
        },
        ArchiveLimitExceeded {
            resource: ArchiveResource::InputBytes,
            limit: 0,
            observed: 8,
        },
    );
}

#[test]
fn admission_does_not_replace_json_or_version_validation() {
    let limits = ArchiveLimits::default();
    for invalid in [
        "{".to_owned(),
        r#"{"version":11,"bad":"\q"}"#.to_owned(),
        r#"{"version":11,"bad":[true false]}"#.to_owned(),
        source() + " false",
        r#"{"version":11}"#.to_owned(),
    ] {
        assert!(matches!(
            RawModel::from_json(&invalid, &limits),
            Err(PersistenceError::Json(_))
        ));
    }
    assert!(matches!(
        RawModel::from_json(r#"{"version":8}"#, &limits),
        Err(PersistenceError::UnsupportedVersion(8))
    ));
    assert!(matches!(
        RawModel::from_json(r#"{"version":9}"#, &limits),
        Err(PersistenceError::UnsupportedVersion(9))
    ));
}
