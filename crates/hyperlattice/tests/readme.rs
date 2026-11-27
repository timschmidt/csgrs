//! Source-text synchronization checks for the README quick start and selected metadata.
//!
//! These tests inspect compile-time included text; they do not execute the example
//! or validate its mathematics, every README claim, or a published package.

/// Current crate README embedded at test compile time.
const README: &str = include_str!("../README.md");
/// Entire runnable example source, including docs, embedded at test compile time.
const QUICKSTART: &str = include_str!("../examples/readme_quickstart.rs");

/// Require the first exact marker-delimited Rust block to equal the example after outer trimming.
///
/// Missing markers panic; internal whitespace and documentation remain significant.
/// Does not parse Markdown/Rust, check marker uniqueness, or execute the block.
#[test]
fn readme_quickstart_matches_the_runnable_example() {
    let marker = "<!-- quickstart:start -->\n```rust\n";
    let start = README
        .find(marker)
        .expect("README quick-start opening marker")
        + marker.len();
    let end_marker = "\n```\n<!-- quickstart:end -->";
    let end = README[start..]
        .find(end_marker)
        .map(|offset| start + offset)
        .expect("README quick-start closing marker");

    assert_eq!(README[start..end].trim(), QUICKSTART.trim());
}

/// Require the package-version substring and eight literal section-heading substrings.
///
/// Substring presence does not validate heading structure/order, dependency versions,
/// publication status, or the contents of those sections.
#[test]
fn readme_release_metadata_matches_the_manifest() {
    assert!(
        README.contains(env!("CARGO_PKG_VERSION")),
        "README must show the current package version"
    );
    for heading in [
        "## Primary types",
        "## Quick start",
        "## API guide",
        "## Features",
        "## Guarantees and boundaries",
        "## References",
        "## Acknowledgements",
        "## License and contributing",
    ] {
        assert!(
            README.contains(heading),
            "missing README section: {heading}"
        );
    }
}
