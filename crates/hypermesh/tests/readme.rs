const README: &str = include_str!("../README.md");
const QUICKSTART: &str = include_str!("../examples/basic.rs");

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
        "## Optional benchmark fixture",
        "## References",
        "## Acknowledgements",
        "## License and contributing",
    ] {
        assert!(
            README.contains(heading),
            "missing README section: {heading}"
        );
    }
    assert!(README.contains("YEAHRIGHT_BENCH=1"));
}
