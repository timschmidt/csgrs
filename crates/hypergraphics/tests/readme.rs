const README: &str = include_str!("../README.md");
const QUICKSTART: &str = include_str!("../examples/readme_quickstart.rs");
const LICENSE_MIT: &str = include_str!("../LICENSE-MIT");
const LICENSE_APACHE: &str = include_str!("../LICENSE-APACHE");

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
        README.contains(&format!("version = \"{}\"", env!("CARGO_PKG_VERSION"))),
        "README must show the current package version"
    );
    assert!(
        README.contains(&format!("Rust {}", env!("CARGO_PKG_RUST_VERSION"))),
        "README must show the declared MSRV"
    );
    assert!(LICENSE_MIT.starts_with("MIT License\n"));
    assert!(LICENSE_APACHE.starts_with("Apache License\n"));
    for heading in [
        "## Primary types",
        "## Quick start",
        "## API guide",
        "## Features",
        "## Errors and guarantees",
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
