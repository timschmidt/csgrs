//! Textual consistency checks for the package README and runnable quickstart.
//!
//! These checks compare embedded source and required substrings. They do not
//! execute the example or validate every README claim and link.

/// Package README embedded at compile time for source and metadata checks.
const README: &str = include_str!("../README.md");
/// Runnable example source embedded for comparison with the marked README block.
const QUICKSTART: &str = include_str!("../examples/readme_quickstart.rs");

/// Compare the first marked Rust quickstart block with the example source.
///
/// Only leading/trailing whitespace is ignored; interior text must match exactly.
/// Missing opening or closing markers fail the test.
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

/// Require the manifest version string and eight section-heading substrings.
///
/// This does not parse heading structure or constrain where the version appears.
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
