//! README quickstart byte synchronization and selected release/layout assertions.
//! These tests inspect embedded text; they do not execute the modeling example
//! or validate every documentation claim, dependency or referenced asset.

/// README snapshot embedded when this test binary is compiled.
const README: &str = include_str!("../readme.md");
/// Entire runnable quickstart source, including Rustdoc and ordinary comments.
const QUICKSTART: &str = include_str!("../examples/basic.rs");

#[test]
/// Requires the first marked Rust fence to equal the example after outer whitespace trimming.
/// Internal whitespace/comments remain significant; marker absence or drift fails.
fn readme_quickstart_matches_the_runnable_example() {
    let start = "<!-- quickstart:start -->\n```rust\n";
    let end = "\n```\n<!-- quickstart:end -->";
    let body = README
        .split_once(start)
        .expect("README quick-start start marker")
        .1
        .split_once(end)
        .expect("README quick-start end marker")
        .0;
    assert_eq!(body.trim(), QUICKSTART.trim());
}

#[test]
/// Checks the manifest version token, required headings and current adapter naming.
/// Rejects selected obsolete headings/names without validating section contents.
fn readme_release_metadata_matches_the_manifest() {
    let version = env!("CARGO_PKG_VERSION");
    assert!(README.contains(&format!("csgrs {version}")));
    for heading in [
        "## Primary types",
        "## Quick start",
        "## Modeling API",
        "## File formats",
        "## Boundary adapters",
        "## Features",
        "## Correctness boundaries",
        "## Performance and hard tests",
        "## References",
        "## Acknowledgements",
        "## Contributing and license",
    ] {
        assert!(README.contains(heading), "missing {heading}");
    }
    assert!(!README.contains("\n## Roadmap"));
    assert!(!README.contains("\n## Todo"));
    assert!(README.contains("csgrs::adapter"));
    assert!(!README.contains(concat!("csgrs", "-adapter")));
    assert!(!README.contains("[adapters]"));
}
